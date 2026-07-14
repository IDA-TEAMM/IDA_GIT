"""
Girdap İDA — Telemetri logger node'u (Layer 2).

Şartname 4.2 — Dosya 2:
    Telemetri CSV, ≥1 Hz, header satırlı, alanlar:
        zaman, lat, lon, hiz, roll, pitch, heading,
        hiz_setpoint, yon_setpoint, mission_state
    Görev bitiminden 20 dk içinde teslim. Geç teslim = 5 ceza puanı.

Subscribed topics:
    /mavros/global_position/global   sensor_msgs/NavSatFix       (lat/lon)
    /mavros/imu/data                 sensor_msgs/Imu             (roll/pitch/heading)
    /mavros/local_position/velocity_body  geometry_msgs/TwistStamped (hiz)
    /girdap/mission/current_target   geometry_msgs/PoseStamped
                                     (yon_setpoint = atan2(y, x), ENU AÇI)
    /girdap/fusion/odom              nav_msgs/Odometry           (yedek poz/hız)
    /girdap/mission/state            std_msgs/String             (durum etiketi)

    setpoint_source == "girdap" (GUIDED/MPPI — yarışma varsayılanı):
    /mavros/setpoint_velocity/cmd_vel_unstamped  geometry_msgs/Twist
                                                 (hiz_setpoint)
    /girdap/control/thrust           std_msgs/Float32MultiArray  [T_sol, T_sag] N

    setpoint_source == "fc" (AUTO — Otonomi videosu kararı 2026-07-13):
    /mavros/rc/out                   mavros_msgs/RCOut
        Thrust FC'nin GERÇEK PWM çıkışından (% normalize). AUTO'da aracı FC
        sürer; MPPI thrust'ı/cmd_vel aracı SÜRMEZ → o kaynaklar Ekran-2'de
        sahte/senkronsuz veri olur (md 3.3.1.1) ve YOK SAYILIR. hiz_setpoint
        görev aktifken fc_cruise_setpoint_mps sabiti (QGC WP_SPEED değeri).

Publishes:
    Yok — diske CSV yazar:
    - Dosya-2 (Şartname 4.2): telemetri_<UTC>.csv, 2 Hz, CSV_HEADER sabit.
    - Grafik CSV (T0-g, md 3.3.1.1 Ekran-2): grafik_<UTC>.csv, 10 Hz —
      hız+setpoint, heading+setpoint, thrust isteği. Video montajında
      Ekran-2 senkron grafikleri buradan çizilir; Dosya-2'ye thrust GİRMEZ.

Notlar:
    - CSV formatı + fsync mantığı prototype.telemetry.csv_logger'da; bu node
      yalnızca ROS 2 mesaj alanlarını cache'ler ve her yazma tick'inde
      TelemetrySample üretir. Test (test_telemetry_logger.py) aynı çekirdeği
      kullanır.
    - roll/pitch/heading /mavros/imu/data quaternion'undan (ENU yaw = heading).
      IMU henüz gelmediyse heading fusion odom'dan yedeklenir.
    - Yazma 2 Hz (≥1 Hz şartname garantisi). Her satırda os.fsync — güç
      kesintisinde en fazla son yarım saniye kaybı.
"""

from __future__ import annotations

import math
from pathlib import Path
from typing import Optional

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped, Twist, TwistStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, NavSatFix
from std_msgs.msg import Float32MultiArray, String

from girdap_decision.qos_profiles import sensor_data_qos

# Setpoint'lerin anlamlı olduğu FSM durumları (F-V.2): görev aktif değilken
# (BOOT/BEKLEMEDE/TAMAMLANDI/KILL) cache'teki son istek CSV'ye YAZILMAZ —
# donuk setpoint çizgisi md 3.3.1.1 senkron grafiklerini yanıltır; boş hücre
# ekran2'de NaN boşluğu olur (sahte çizgi yok).
_SETPOINT_ACTIVE_STATES = ("PARKUR1", "PARKUR2", "PARKUR3")
from prototype.telemetry.csv_logger import (
    GRAPH_CSV_HEADER,
    GraphSample,
    TelemetryCsvLogger,
    TelemetrySample,
    pwm_to_thrust_pct,
    quat_to_rpy,
    utc_isoformat,
    utc_timestamp,
)

# fc kaynağında hiz_setpoint yalnız görev aktifken yazılır (mission_manager
# _ACTIVE_STATES ile aynı küme) — görev dışında sahte setpoint basılmaz.
_ACTIVE_STATES = ("PARKUR1", "PARKUR2", "PARKUR3")


class TelemetryNode(Node):
    """Şartname 4.2 Dosya 2 üreticisi."""

    def __init__(self, **node_kwargs) -> None:
        # node_kwargs → parameter_overrides passthrough (test enjeksiyonu).
        super().__init__("telemetry_node", **node_kwargs)

        # --- Parametreler ---
        # Boş → ~/girdap_logs/telemetry (MUTLAK yol). Göreli yol KULLANMA:
        # csv_logger expanduser göreli yolu çözmez → systemd'de cwd=/ →
        # mkdir(/data/...) PermissionError → Dosya-2 üretilmez → 5 ceza puanı.
        # local_map_node ile aynı desen (Şartname 4.2 Dosya-3).
        self.declare_parameter("csv_output_dir", "")
        self.declare_parameter("log_rate_hz", 2.0)          # ≥1 Hz garanti
        # Grafik CSV (Ekran-2): MPPI control_rate ile aynı 10 Hz — 2 Hz
        # örnekleme thrust isteğini alias'lar, grafik yanıltıcı olur.
        self.declare_parameter("graph_output_dir", "")
        self.declare_parameter("graph_rate_hz", 10.0)
        # --- Setpoint/thrust kaynağı (AUTO video kararı 2026-07-13) ---
        # "girdap": GUIDED/MPPI — mevcut davranış birebir (yarışma varsayılanı).
        # "fc":     AUTO — thrust /mavros/rc/out'tan (% normalize), hiz_setpoint
        #           görev aktifken fc_cruise_setpoint_mps; cmd_vel ve
        #           /girdap/control/thrust'a hiç abone olunmaz (sahte veri yok).
        self.declare_parameter("setpoint_source", "girdap")
        self.declare_parameter("fc_cruise_setpoint_mps", 0.0)  # 0 = boş bırak
        self.declare_parameter("fc_thrust_left_ch", 1)    # SERVOn, 1 tabanlı
        self.declare_parameter("fc_thrust_right_ch", 3)   # (73/74 çıkışları)
        self.declare_parameter("fc_pwm_neutral", 1500)    # µs, sıfır itki
        self.declare_parameter("fc_pwm_range", 500)       # µs, ±tam itki

        out_dir = str(self.get_parameter("csv_output_dir").value)
        if not out_dir:
            out_dir = str(Path.home() / "girdap_logs" / "telemetry")
        # DİKKAT: rclpy Node dahili logger'ını `self._logger`'da tutar; bu isimle
        # CSV logger'ı atamak get_logger()'ı ezer → `self._csv` kullanılır.
        self._csv = TelemetryCsvLogger(out_dir)

        # Grafik CSV ayrı dizinde — Dosya-2 teslim klasörüne karışmasın
        # (USB'ye yalnız telemetry/ kopyalanır, grafik yarışma çıktısı değil).
        graph_dir = str(self.get_parameter("graph_output_dir").value)
        if not graph_dir:
            graph_dir = str(Path.home() / "girdap_logs" / "grafik")
        self._graph_csv = TelemetryCsvLogger(
            graph_dir,
            filename=f"grafik_{utc_timestamp()}.csv",
            header=GRAPH_CSV_HEADER,
        )

        # --- Cache (en son alınan değer her tick'te yazılır) ---
        self._lat: Optional[float] = None
        self._lon: Optional[float] = None
        self._speed: Optional[float] = None
        self._speed_from_body: bool = False       # F15.4: odom yedeği bayrağı
        self._roll: Optional[float] = None
        self._pitch: Optional[float] = None
        self._heading: Optional[float] = None
        self._heading_from_imu: bool = False
        self._speed_sp: Optional[float] = None
        self._yaw_sp: Optional[float] = None
        self._mission_state: str = ""
        self._thrust_sol: Optional[float] = None  # Ekran-2c: [T_sol, T_sag] N
        self._thrust_sag: Optional[float] = None

        # --- Subscribers ---
        # mavros sensör topic'leri BEST_EFFORT (sensor_data) yayınlar; RELIABLE
        # abone olursak mesaj gelmez (QoS uyumsuz). Setpoint/odom/state internal
        # ve RELIABLE olduğundan varsayılan depth=10 yeterli.
        self._sub_gps = self.create_subscription(
            NavSatFix, "/mavros/global_position/global",
            self._on_gps, sensor_data_qos(),
        )
        self._sub_imu = self.create_subscription(
            Imu, "/mavros/imu/data", self._on_imu, sensor_data_qos()
        )
        self._sub_vel = self.create_subscription(
            TwistStamped, "/mavros/local_position/velocity_body",
            self._on_vel_body, sensor_data_qos(),
        )
        # F-V.1 (md 3.3.1.1 Ekran-2b): yon_setpoint bir AÇI olmalı; cmd_vel
        # angular.z yaw HIZIdır (rad/s) — o yüzden setpoint açısı hedeften
        # türetilir: current_target araç-göreli ENU ofset → atan2(y, x).
        self._sub_target = self.create_subscription(
            PoseStamped, "/girdap/mission/current_target", self._on_target, 10
        )
        self._sub_odom = self.create_subscription(
            Odometry, "/girdap/fusion/odom", self._on_odom, 10
        )
        self._sub_state = self.create_subscription(
            String, "/girdap/mission/state", self._on_mission_state, 10
        )

        # Kaynak seçimi: fc modunda MPPI kanallarına HİÇ abone olunmaz —
        # AUTO'da planning thrust'ı aracı sürmez, Ekran-2'ye sahte/senkronsuz
        # veri yazardı (md 3.3.1.1 "istemsiz/senkronsuz = BAŞARISIZ").
        self._source_fc = (
            str(self.get_parameter("setpoint_source").value) == "fc"
        )
        self._fc_cruise = float(
            self.get_parameter("fc_cruise_setpoint_mps").value
        )
        if self._source_fc:
            # mavros_msgs yalnız fc modunda gerekir (lazy import) — girdap
            # modunda node mavros_msgs'siz ortamda da (CI) import edilebilir.
            from mavros_msgs.msg import RCOut

            self._fc_left = int(self.get_parameter("fc_thrust_left_ch").value)
            self._fc_right = int(
                self.get_parameter("fc_thrust_right_ch").value
            )
            self._fc_neutral = int(self.get_parameter("fc_pwm_neutral").value)
            self._fc_range = int(self.get_parameter("fc_pwm_range").value)
            # mavros rc/out sensör kategorisi → BEST_EFFORT QoS.
            self._sub_rcout = self.create_subscription(
                RCOut, "/mavros/rc/out", self._on_rc_out, sensor_data_qos()
            )
        else:
            self._sub_setpoint = self.create_subscription(
                Twist, "/mavros/setpoint_velocity/cmd_vel_unstamped",
                self._on_setpoint, 10,
            )
            self._sub_thrust = self.create_subscription(
                Float32MultiArray, "/girdap/control/thrust",
                self._on_thrust, 10,
            )

        # --- Yazma timer'ları ---
        rate = float(self.get_parameter("log_rate_hz").value)
        self._timer = self.create_timer(1.0 / rate, self._on_write)
        graph_rate = float(self.get_parameter("graph_rate_hz").value)
        self._graph_timer = self.create_timer(
            1.0 / graph_rate, self._on_graph_write
        )

        source = "fc (AUTO — rc/out)" if self._source_fc else "girdap (MPPI)"
        self.get_logger().info(
            f"telemetry_node aktif → {self._csv.path} (rate={rate} Hz), "
            f"grafik → {self._graph_csv.path} (rate={graph_rate} Hz), "
            f"setpoint kaynağı={source}"
        )

    # ----- subscriber callback'leri -----

    def _on_gps(self, msg: NavSatFix) -> None:
        if msg.status.status >= 0:               # fix yoksa (-1) yazma
            self._lat = msg.latitude
            self._lon = msg.longitude

    def _on_imu(self, msg: Imu) -> None:
        q = msg.orientation
        roll, pitch, yaw = quat_to_rpy(q.x, q.y, q.z, q.w)
        self._roll = roll
        self._pitch = pitch
        # mavros IMU yaw'ı ENU heading olarak verir → doğrudan kullan
        self._heading = yaw
        self._heading_from_imu = True

    def _on_vel_body(self, msg: TwistStamped) -> None:
        v = msg.twist.linear
        self._speed = math.sqrt(v.x * v.x + v.y * v.y)
        self._speed_from_body = True

    def _on_setpoint(self, msg: Twist) -> None:
        # angular.z yaw HIZI — yon_setpoint'e YAZILMAZ (F-V.1).
        self._speed_sp = msg.linear.x

    def _on_target(self, msg: PoseStamped) -> None:
        # Araç-göreli ENU ofsetten istenen rota açısı (heading ile aynı
        # konvansiyon). IDLE/COMPLETE'te yayın durur → son açı cache kalır.
        p = msg.pose.position
        self._yaw_sp = math.atan2(p.y, p.x)

    def _on_odom(self, msg: Odometry) -> None:
        # IMU henüz gelmediyse heading'i fusion odom'dan yedekle
        if not self._heading_from_imu:
            q = msg.pose.pose.orientation
            self._heading = 2.0 * math.atan2(q.z, q.w)
        # F15.4: velocity_body gelmiyorsa hızı odom twist'ten yedekle —
        # yoksa Ekran-2 hız grafiği boş kalır (Odometry twist child frame'de).
        if not self._speed_from_body:
            v = msg.twist.twist.linear
            self._speed = math.sqrt(v.x * v.x + v.y * v.y)

    def _on_mission_state(self, msg: String) -> None:
        self._mission_state = msg.data
        # fc modu: hiz_setpoint = operatörün FC'ye girdiği seyir hızı, yalnız
        # görev aktifken (AUTO'da cmd_vel yok — FC kendi sürer). Görev
        # dışında None → CSV boş (sahte setpoint basılmaz).
        if self._source_fc and self._fc_cruise > 0.0:
            self._speed_sp = (
                self._fc_cruise if msg.data in _ACTIVE_STATES else None
            )

    def _on_thrust(self, msg: Float32MultiArray) -> None:
        if len(msg.data) >= 2:
            self._thrust_sol = float(msg.data[0])
            self._thrust_sag = float(msg.data[1])

    def _on_rc_out(self, msg) -> None:                    # mavros_msgs/RCOut
        """fc modu (AUTO): Ekran-2c thrust'ı FC'nin GERÇEK PWM çıkışından.

        Kanal numaraları 1 tabanlı SERVO çıkışıdır (fc_thrust_left/right_ch);
        kanal yayında yoksa ya da PWM<=0 (çıkış kapalı) → None (CSV boş).
        """
        ch = msg.channels
        left = int(ch[self._fc_left - 1]) if len(ch) >= self._fc_left else 0
        right = int(ch[self._fc_right - 1]) if len(ch) >= self._fc_right else 0
        self._thrust_sol = pwm_to_thrust_pct(
            left, self._fc_neutral, self._fc_range
        )
        self._thrust_sag = pwm_to_thrust_pct(
            right, self._fc_neutral, self._fc_range
        )

    # ----- yazma -----

    def _mission_active(self) -> bool:
        """F-V.2: setpoint sütunları yalnız görev aktifken yazılır."""
        return self._mission_state in _SETPOINT_ACTIVE_STATES

    def _on_write(self) -> None:
        active = self._mission_active()
        sample = TelemetrySample(
            lat=self._lat,
            lon=self._lon,
            hiz=self._speed,
            roll=self._roll,
            pitch=self._pitch,
            heading=self._heading,
            hiz_setpoint=self._speed_sp if active else None,
            yon_setpoint=self._yaw_sp if active else None,
            mission_state=self._mission_state,
        )
        self._csv.write_sample(utc_isoformat(), sample)

    def _on_graph_write(self) -> None:
        active = self._mission_active()
        sample = GraphSample(
            hiz=self._speed,
            hiz_setpoint=self._speed_sp if active else None,
            heading=self._heading,
            yon_setpoint=self._yaw_sp if active else None,
            thrust_sol=self._thrust_sol,
            thrust_sag=self._thrust_sag,
        )
        self._graph_csv.write_sample(utc_isoformat(), sample)

    # ----- yaşam döngüsü -----

    def destroy_node(self) -> bool:
        for logger in (self._csv, self._graph_csv):
            try:
                logger.close()
            except Exception as e:               # yıkım sırasında loglama
                self.get_logger().error(f"CSV kapatma hatası: {e}")
        return super().destroy_node()


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = TelemetryNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
