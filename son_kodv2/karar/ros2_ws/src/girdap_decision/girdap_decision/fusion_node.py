"""
Girdap İDA — iSAM2 sensör füzyonu node'u (Layer 2).

Pixhawk → mavros → bu node → diğer karar modülleri.

Subscribed topics:
    /mavros/imu/data                 sensor_msgs/Imu
        ~50-100 Hz raw IMU. Yaw rate (gyro z) → BetweenFactor adımı.
    /mavros/global_position/global   sensor_msgs/NavSatFix
        ~1 Hz RTK GPS. İlk fix origin → ENU projeksiyon, prior factor.
    /mavros/local_position/velocity_body  geometry_msgs/TwistStamped
        Body-frame hız. Pre-integration için vx·dt, vy·dt kullanılır.

Published topics:
    /girdap/fusion/odom              nav_msgs/Odometry
        Smooth ENU pose + velocity (planlama node'unun girdisi).
    /girdap/fusion/pose              geometry_msgs/PoseStamped
        Salt poz; RViz/telemetri için ek kanal.

Çift mod (config algorithm.use_isam2):
    use_isam2=true  (yarışma): iSAM2/GTSAM smoother — IMU + GPS + velocity_body.
    use_isam2=false (video):   MAVROS EKF pass-through — /mavros/local_position/
                               pose doğrudan iletilir; GTSAM hiç yüklenmez
                               (FusionPipeline lazy import edilir).
    Her iki modda velocity_body dinlenir ve odom.twist'e yazılır (F8.1 —
    planning_node MPPI durumuna u,v,r'yi twist'ten okur; boş bırakılamaz).

Notlar:
    - Fusion mantığı prototype.fusion.pipeline.FusionPipeline (iSAM2) veya
      prototype.fusion.bypass.PosePassthrough (video); ikisi de current_pose()
      sözleşmesini paylaşır → yayım kodu moddan bağımsız. Birim testler aynı
      çekirdekleri kullanır (rclpy'siz .venv).
    - Frame politikası: ENU. Pixhawk dahili NED → mavros çevirir.
"""

from __future__ import annotations

import math

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped, TwistStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, NavSatFix

from girdap_decision.qos_profiles import sensor_data_qos


def _stamp_to_seconds(stamp) -> float:
    return stamp.sec + stamp.nanosec * 1e-9


class FusionNode(Node):
    """Poz kaynağı: iSAM2 smoother (yarışma) veya MAVROS EKF geçişi (video)."""

    def __init__(self, **node_kwargs) -> None:
        # node_kwargs → parameter_overrides passthrough (test enjeksiyonu).
        super().__init__("fusion_node", **node_kwargs)

        # --- Parametreler (config/params.yaml ile override edilebilir) ---
        self.declare_parameter("use_isam2", True)       # false → video bypass
        # F-P.13 (robustness taraması, 2026-07-15): params.yaml bunu 10.0'a
        # düşürüyordu ("odom yayını = smoother flush hızı") — kod varsayılanı
        # 50.0 idi, params.yaml uygulanmadan standalone koşulursa sessizce
        # yanlış (5x) hıza düşerdi. Config-drift önlemek için hizalandı.
        self.declare_parameter("publish_rate_hz", 10.0)
        # F8.2: son sensör girdisinden bu kadar süre geçtiyse odom yayını
        # KESİLİR (bayat pozla 50 Hz yayın, downstream'i donmuş pozla plan
        # yapmaya iter). 0 → devre dışı.
        self.declare_parameter("pose_timeout_s", 1.0)
        # F-P.7 (robustness taraması, 2026-07-15): pose_timeout_s YALNIZ
        # poz kaynağını (IMU/EKF) kapsar — velocity_body AYRI bir topic,
        # kendi kaynağı tek başına donarsa (IMU/GPS akışı sürerken) bu
        # bekçi hiç tetiklenmez, od.twist SONSUZA DEK donuk son hızı
        # yayınlamaya devam eder (planning_node MPPI'ye yanlış u,v,r besler).
        self.declare_parameter("vel_timeout_s", 1.0)
        self.declare_parameter("odom_period_s", 0.1)
        self.declare_parameter("gps_sigma_xy", 0.30)
        self.declare_parameter("imu_sigma_xy", 0.05)
        self.declare_parameter("imu_sigma_psi", 0.01)

        self._use_isam2 = bool(self.get_parameter("use_isam2").value)
        sensor_qos = sensor_data_qos()

        # Diagnostic sayaçları
        self._n_imu = 0
        self._n_gps = 0
        self._n_vel = 0
        self._n_pose = 0

        # F8.1: son body-frame hız (velocity_body) — odom.twist'e yazılır.
        # planning_node MPPI durum vektörüne (u, v, r) buradan okur; boş
        # bırakılırsa MPPI her adımda araç duruyormuş sanır (istemsiz hareket).
        self._last_vx = 0.0
        self._last_vy = 0.0
        self._last_wz = 0.0

        # F8.2: poz kaynağının son güncelleme zamanı (bayatlık bekçisi)
        self._pose_timeout_s = float(self.get_parameter("pose_timeout_s").value)
        self._last_input_t: float | None = None
        # F-P.7: velocity_body'nin KENDİ bayatlık bekçisi (pose_timeout_s'ten
        # bağımsız — bkz. declare_parameter yorumu).
        self._vel_timeout_s = float(self.get_parameter("vel_timeout_s").value)
        self._last_vel_t: float | None = None
        self._vel_stale_warned = False
        self._stale_warned = False

        if self._use_isam2:
            self._setup_isam2(sensor_qos)              # GTSAM sadece burada
        else:
            self._setup_bypass(sensor_qos)             # GTSAM yüklenmez

        # --- Publishers ---
        self._pub_odom = self.create_publisher(
            Odometry, "/girdap/fusion/odom", 10
        )
        self._pub_pose = self.create_publisher(
            PoseStamped, "/girdap/fusion/pose", 10
        )

        # --- Periyodik yayım ---
        rate = float(self.get_parameter("publish_rate_hz").value)
        self._timer = self.create_timer(1.0 / rate, self._on_publish_timer)
        self._diag_timer = self.create_timer(5.0, self._log_diag)

        mode = "iSAM2" if self._use_isam2 else "MAVROS EKF geçişi (video)"
        self.get_logger().info(
            f"fusion_node aktif [{mode}]: publish={rate} Hz"
        )

    # ----- mod kurulumları -----

    def _setup_isam2(self, sensor_qos) -> None:
        """Yarışma modu — GTSAM lazy import + IMU/GPS/velocity abonelikleri."""
        from prototype.fusion.pipeline import (        # lazy: GTSAM burada yüklenir
            FusionPipeline,
            FusionPipelineConfig,
        )
        cfg = FusionPipelineConfig(
            odom_period_s=float(self.get_parameter("odom_period_s").value),
            gps_sigma_xy=float(self.get_parameter("gps_sigma_xy").value),
            odom_sigma_xy=float(self.get_parameter("imu_sigma_xy").value),
            odom_sigma_psi=float(self.get_parameter("imu_sigma_psi").value),
        )
        self._source = FusionPipeline(cfg)
        self._sub_imu = self.create_subscription(
            Imu, "/mavros/imu/data", self._on_imu, sensor_qos
        )
        self._sub_gps = self.create_subscription(
            NavSatFix, "/mavros/global_position/global",
            self._on_gps, sensor_qos,
        )
        self._sub_vel_body = self.create_subscription(
            TwistStamped, "/mavros/local_position/velocity_body",
            self._on_vel_body, sensor_qos,
        )

    def _setup_bypass(self, sensor_qos) -> None:
        """Video modu — MAVROS EKF pozunu (/mavros/local_position/pose) ilet."""
        from prototype.fusion.bypass import PosePassthrough, quat_to_yaw
        self._quat_to_yaw = quat_to_yaw
        self._source = PosePassthrough()
        self._sub_pose = self.create_subscription(
            PoseStamped, "/mavros/local_position/pose",
            self._on_ekf_pose, sensor_qos,
        )
        # F8.1: bypass'ta da hız aboneliği — odom.twist boş kalmasın.
        self._sub_vel_body = self.create_subscription(
            TwistStamped, "/mavros/local_position/velocity_body",
            self._on_vel_body, sensor_qos,
        )

    # ----- iSAM2 callback'leri -----

    def _mark_input(self) -> None:
        """F8.2: poz kaynağını süren bir girdi geldi — bayatlık saatini sıfırla."""
        self._last_input_t = self.get_clock().now().nanoseconds * 1e-9
        if self._stale_warned:
            self._stale_warned = False
            self.get_logger().info("poz kaynağı geri geldi — odom yayını sürüyor")

    def _on_imu(self, msg: Imu) -> None:
        """IMU yaw rate → boru hattına ilet, periyot dolduğunda smoother step."""
        t = _stamp_to_seconds(msg.header.stamp)
        self._source.on_imu(t, msg.angular_velocity.z)
        self._n_imu += 1
        self._mark_input()                       # iSAM2 pozunu IMU sürer

    def _on_vel_body(self, msg: TwistStamped) -> None:
        """Body-frame hız: twist cache (her mod) + smoother beslemesi (iSAM2)."""
        self._last_vx = msg.twist.linear.x
        self._last_vy = msg.twist.linear.y
        self._last_wz = msg.twist.angular.z
        self._last_vel_t = self.get_clock().now().nanoseconds * 1e-9  # F-P.7
        if self._use_isam2:
            self._source.on_velocity(msg.twist.linear.x, msg.twist.linear.y)
        self._n_vel += 1

    def _vel_stale(self) -> bool:
        """F-P.7: son velocity_body `vel_timeout_s`'ten eski mi? Hiç
        gelmediyse False (boot gürültüsü, F-P.1/F8.2 ile aynı ilke)."""
        if self._vel_timeout_s <= 0.0 or self._last_vel_t is None:
            return False
        now = self.get_clock().now().nanoseconds * 1e-9
        return (now - self._last_vel_t) > self._vel_timeout_s

    def _on_gps(self, msg: NavSatFix) -> None:
        """RTK GPS fix → ENU projeksiyon → smoother prior."""
        if msg.status.status < 0:
            self.get_logger().warn("GPS fix yok (status<0), atlanıyor")
            return
        self._source.on_gps(msg.latitude, msg.longitude)
        self._n_gps += 1

    # ----- bypass callback'i -----

    def _on_ekf_pose(self, msg: PoseStamped) -> None:
        """MAVROS EKF pozu → doğrudan poz kaynağına ilet (video bypass)."""
        p = msg.pose.position
        q = msg.pose.orientation
        psi = self._quat_to_yaw(q.x, q.y, q.z, q.w)
        self._source.update(p.x, p.y, psi)
        self._n_pose += 1
        self._mark_input()                       # bypass pozunu EKF sürer

    # ----- yayım -----

    def _on_publish_timer(self) -> None:
        try:
            x, y, psi = self._source.current_pose()
        except RuntimeError:
            return

        # F8.2: girdi akışı kesildiyse DONMUŞ pozu yayınlamaya devam etme.
        if self._pose_timeout_s > 0.0 and self._last_input_t is not None:
            age = self.get_clock().now().nanoseconds * 1e-9 - self._last_input_t
            if age > self._pose_timeout_s:
                if not self._stale_warned:
                    self._stale_warned = True
                    self.get_logger().warn(
                        f"poz kaynağı {age:.1f}s'dir sessiz — odom yayını "
                        "KESİLDİ (bayat pozla plan yapılmasın)"
                    )
                return

        now = self.get_clock().now().to_msg()
        qz = math.sin(psi / 2.0)
        qw = math.cos(psi / 2.0)

        ps = PoseStamped()
        ps.header.stamp = now
        ps.header.frame_id = "map"
        ps.pose.position.x = x
        ps.pose.position.y = y
        ps.pose.orientation.z = qz
        ps.pose.orientation.w = qw
        self._pub_pose.publish(ps)

        od = Odometry()
        od.header.stamp = now
        od.header.frame_id = "map"
        od.child_frame_id = "base_link"
        od.pose.pose = ps.pose
        # F8.1: twist = son velocity_body (body-frame → child_frame_id
        # semantiğiyle doğru). planning_node MPPI durumuna (u, v, r) okur;
        # telemetry_node hız yedeği de buradan beslenir.
        # F-P.7: velocity_body TEK BAŞINA bayatlaşırsa (pose_timeout_s'i
        # tetiklemeden, çünkü IMU/EKF akışı sürüyor olabilir) donuk hızı
        # yayınlama — sıfırla + uyar.
        if self._vel_stale():
            if not self._vel_stale_warned:
                self._vel_stale_warned = True
                age = (
                    self.get_clock().now().nanoseconds * 1e-9
                    - (self._last_vel_t or 0.0)
                )
                self.get_logger().warn(
                    f"velocity_body {age:.1f}s'dir sessiz — odom twist'i "
                    "sıfırlandı (F-P.7: bayat hızla MPPI beslenmesin)"
                )
            od.twist.twist.linear.x = 0.0
            od.twist.twist.linear.y = 0.0
            od.twist.twist.angular.z = 0.0
        else:
            if self._vel_stale_warned:
                self._vel_stale_warned = False
                self.get_logger().info("velocity_body geri geldi")
            od.twist.twist.linear.x = self._last_vx
            od.twist.twist.linear.y = self._last_vy
            od.twist.twist.angular.z = self._last_wz
        self._pub_odom.publish(od)

    def _log_diag(self) -> None:
        """5 sn'lik akış sayımları — düşük rate'leri erken yakala."""
        try:
            x, y, psi = self._source.current_pose()
            pose_str = f"x={x:.2f} y={y:.2f} ψ={math.degrees(psi):.1f}°"
        except RuntimeError:
            pose_str = "henüz tahmin yok"
        if self._use_isam2:
            counts = (
                f"imu={self._n_imu} gps={self._n_gps} vel={self._n_vel}"
            )
        else:
            counts = f"ekf_pose={self._n_pose}"
        self.get_logger().info(f"[diag 5s] {counts} | {pose_str}")
        self._n_imu = self._n_gps = self._n_vel = self._n_pose = 0


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = FusionNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
