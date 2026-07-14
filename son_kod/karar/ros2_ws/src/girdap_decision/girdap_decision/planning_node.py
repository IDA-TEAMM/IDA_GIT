"""
Girdap İDA — Planlama node'u (Layer 2): RRT* global + MPPI lokal.

Akış:
    fusion_node (smooth pose) ─┐
    perception (engel haritası) ┼─→ planning_node ─→ thrust komutu
    mission waypoints ─────────┤                  └─→ /mavros/setpoint_velocity
    fsm (durum) ───────────────┘

Subscribed topics:
    /girdap/fusion/odom              nav_msgs/Odometry      (smooth pose+vel)
    /girdap/mission/state            std_msgs/String        (FSM durumu)
    /perception/obstacle_map         geometry_msgs/PoseArray
        Engel merkezleri; her poz position.{x,y} = merkez, orientation.z =
        yarıçap (PLACEHOLDER şema — perception ekibi topic'i teslim edince
        güncellenecek; OccupancyGrid gelirse costmap→circle çıkarımı eklenir).
    /girdap/mission/waypoints        nav_msgs/Path          (görev hedefleri)

Published topics:
    /mavros/setpoint_velocity/cmd_vel_unstamped  geometry_msgs/Twist
        Cascade PID dış döngü çıktısı (CLAUDE.md MAVROS bölümü).
    /girdap/planning/global_path     nav_msgs/Path
        RRT* çıkışı; RViz görselleştirmesi ve replan izleme için.
    /girdap/control/thrust           std_msgs/Float32MultiArray
        Diferansiyel thruster komutu [T_left, T_right] (N) — Layer 1 ESC kanalı.

Notlar:
    - Tüm planlama mantığı prototype.planning.pipeline.PlanningPipeline'da;
      bu node yalnızca ROS 2 mesaj alanlarını okuyup boru hattına yönlendirir.
      Uçtan uca test (test_planning_pipeline.py) aynı sınıfı kullanır.
    - Kontrol döngüsü 20 Hz (MPPI dt=0.05 ile hizalı). CLAUDE.md 50 Hz hedefi
      Jetson CUDA sürümüne aittir; CPU Layer 2'de 20 Hz doğrulama içindir.
    - Parkur bazlı davranış PlanningPipeline'da: FSM durumu değişince MPPI
      ağırlık profili (w_track/w_obstacle/kamikaze) otomatik değişir.
    - FSM durumu PARKUR1/2/3 değilse thrust 0.0 yayınlanır (FSM otoritesi).
"""

from __future__ import annotations

import math
from typing import Optional

import numpy as np
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseArray, PoseStamped, Twist
from mavros_msgs.msg import State as MavState
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from std_msgs.msg import Float32MultiArray, String

from girdap_decision.qos_profiles import sensor_data_qos
from prototype.control.mavros_bridge import MavrosBridge, MavrosBridgeConfig
from prototype.planning.pipeline import PlanningPipeline, PlanningPipelineConfig
from prototype.planning.rrt_star import Bounds, CircleObstacle


class PlanningNode(Node):
    """RRT* global + MPPI lokal planlayıcı sarmalayıcısı."""

    def __init__(self, **node_kwargs) -> None:
        # node_kwargs → parameter_overrides passthrough (test enjeksiyonu;
        # diğer node'larla aynı desen).
        super().__init__("planning_node", **node_kwargs)

        # --- Parametreler ---
        self.declare_parameter("control_rate_hz", 20.0)
        self.declare_parameter("bounds_x", [0.0, 200.0])
        self.declare_parameter("bounds_y", [0.0, 200.0])
        self.declare_parameter("replan_proximity", 2.0)     # m
        self.declare_parameter("mppi_K", 1000)
        self.declare_parameter("mppi_T", 50)
        self.declare_parameter("heartbeat_timeout_s", 5.0)  # MAVROS geçidi
        self.declare_parameter("mode_name", "GUIDED")        # otonomi modu
        self.declare_parameter("map_rate_hz", 10.0)          # Dosya-3 yayım hızı
        # F-P.1: fusion odom'u kesse bile (F8.2) planning SON pozla MPPI
        # koşmaya devam ediyordu → GPS/EKF kesintisinde KÖR sürüş. Odom bu
        # süreden bayatsa thrust sıfırlanır. 0 = kapalı (test/sim).
        self.declare_parameter("odom_timeout_s", 1.0)
        self.declare_parameter("use_rrt", True)              # false → video bypass

        bx = self.get_parameter("bounds_x").value
        by = self.get_parameter("bounds_y").value
        bounds = Bounds(bx[0], bx[1], by[0], by[1])

        cfg = PlanningPipelineConfig(
            replan_proximity=float(self.get_parameter("replan_proximity").value),
            mppi_K=int(self.get_parameter("mppi_K").value),
            mppi_T=int(self.get_parameter("mppi_T").value),
        )
        self._pipe = PlanningPipeline(bounds, cfg)

        # Video bypass: use_rrt=false → global plan atlanır, current_target
        # doğrudan MPPI referansı. Son poz absolute hedef hesabı için tutulur.
        self._use_rrt = bool(self.get_parameter("use_rrt").value)
        self._last_xy: Optional[tuple] = None
        # F-P.1: son odom zamanı (node saati) — bayatlık bekçisi.
        self._odom_timeout_s = float(
            self.get_parameter("odom_timeout_s").value
        )
        self._last_odom_t: Optional[float] = None

        # MAVROS mod/arm geçidi — mavros_bridge ile aynı karar çekirdeği (DRY).
        # Hedef mod (mode_name) değilse cmd_vel yayınlanmaz; armed değilse thrust
        # sıfırlanır. mode_name mavros_bridge ile AYNI olmalı (tek kaynak: köke
        # bakan hardware.yaml → hardware.launch her iki node'a aktarır).
        self._bridge = MavrosBridge(
            MavrosBridgeConfig(
                heartbeat_timeout_s=float(
                    self.get_parameter("heartbeat_timeout_s").value
                ),
                target_mode=str(self.get_parameter("mode_name").value),
            )
        )

        # --- Subscribers ---
        self._sub_odom = self.create_subscription(
            Odometry, "/girdap/fusion/odom", self._on_odom, 10
        )
        self._sub_state = self.create_subscription(
            String, "/girdap/mission/state", self._on_mission_state, 10
        )
        self._sub_mav_state = self.create_subscription(
            MavState, "/mavros/state", self._on_mav_state, 10
        )
        self._sub_obs = self.create_subscription(
            PoseArray, "/perception/obstacle_map", self._on_obstacles, 10
        )
        self._sub_wp = self.create_subscription(
            Path, "/girdap/mission/waypoints", self._on_waypoints, 10
        )
        # Video bypass (use_rrt=false): mission_manager'dan doğrudan hedef.
        self._sub_target = self.create_subscription(
            PoseStamped, "/girdap/mission/current_target", self._on_target, 10
        )

        # --- Publishers ---
        self._pub_cmd_vel = self.create_publisher(
            Twist, "/mavros/setpoint_velocity/cmd_vel_unstamped", 10
        )
        self._pub_path = self.create_publisher(
            Path, "/girdap/planning/global_path", 10
        )
        self._pub_thrust = self.create_publisher(
            Float32MultiArray, "/girdap/control/thrust", 10
        )
        # Dosya-3: yerel maliyet haritası (RViz + local_map_node PNG dumper).
        self._pub_map = self.create_publisher(
            OccupancyGrid, "/girdap/map/local", sensor_data_qos()
        )

        # --- Kontrol döngüsü ---
        rate = float(self.get_parameter("control_rate_hz").value)
        self._timer = self.create_timer(1.0 / rate, self._on_control_step)

        # --- Yerel harita yayım döngüsü (Dosya-3, ~10 Hz) ---
        map_rate = float(self.get_parameter("map_rate_hz").value)
        self._map_timer = self.create_timer(1.0 / map_rate, self._publish_local_map)

        planner = "RRT*+MPPI" if self._use_rrt else "düz hedef+MPPI (video)"
        self.get_logger().info(
            f"planning_node aktif [{planner}] (MPPI K={cfg.mppi_K}, "
            f"T={cfg.mppi_T}, control={rate} Hz, map={map_rate} Hz)"
        )

    # ----- subscriber callback'leri -----

    def _on_odom(self, msg: Odometry) -> None:
        """ENU pose + velocity → durum vektörü [x, y, ψ, u, v, r]."""
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        psi = 2.0 * math.atan2(q.z, q.w)             # z-eksen quaternion → yaw
        v = msg.twist.twist.linear
        w = msg.twist.twist.angular
        self._pipe.set_state(np.array([p.x, p.y, psi, v.x, v.y, w.z]))
        self._last_xy = (p.x, p.y)               # bypass absolute hedef için
        self._last_odom_t = self._now()          # F-P.1: tazelik damgası

    def _on_obstacles(self, msg: PoseArray) -> None:
        """PLACEHOLDER şema: position.{x,y} merkez, orientation.z yarıçap."""
        obstacles = [
            CircleObstacle(pp.position.x, pp.position.y, abs(pp.orientation.z))
            for pp in msg.poses
        ]
        self._pipe.set_obstacles(obstacles)

    def _on_waypoints(self, msg: Path) -> None:
        if not self._use_rrt:                    # video bypass → RRT* girişi yok
            return
        waypoints = [
            (ps.pose.position.x, ps.pose.position.y) for ps in msg.poses
        ]
        if waypoints:
            self._pipe.set_waypoints(waypoints)
            path = self._pipe.global_path
            if path is not None:
                self._publish_path(path)

    def _on_target(self, msg: PoseStamped) -> None:
        """Video bypass: mission_manager hedefi → düz çizgi MPPI referansı.

        current_target base_link'te araç-göreli ENU ofsetidir; absolute hedef
        için son odom pozuna eklenir (RRT* atlanır).
        """
        if self._use_rrt or self._last_xy is None:
            return
        tx = self._last_xy[0] + msg.pose.position.x
        ty = self._last_xy[1] + msg.pose.position.y
        self._pipe.set_reference_direct(tx, ty)
        path = self._pipe.global_path
        if path is not None:
            self._publish_path(path)

    def _on_mission_state(self, msg: String) -> None:
        self._pipe.set_mission_state(msg.data)

    def _on_mav_state(self, msg: MavState) -> None:
        """MAVROS mod/arm geçidi için FCU durumunu güncelle."""
        self._bridge.update_state(
            self._now(), msg.connected, msg.armed, msg.guided, msg.mode
        )

    # ----- kontrol döngüsü -----

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _odom_stale(self) -> bool:
        """F-P.1: odom hiç gelmediyse ya da timeout'tan eskiyse True.

        Hiç odom yokken pipeline zaten hedefsiz/durağandır ama BAYAT poz
        MPPI'yi son bilinen konumla salınıma sokar — ikisi de sıfır thrust'a
        kapılanır. odom_timeout_s <= 0 → bekçi kapalı (test/sim).
        """
        if self._odom_timeout_s <= 0.0:
            return False
        if self._last_odom_t is None:
            return True
        return (self._now() - self._last_odom_t) > self._odom_timeout_s

    def _on_control_step(self) -> None:
        """20 Hz'te MPPI step → thrust komut + Twist setpoint (MAVROS geçitli).

        Geçit kuralları (prototype.control.mavros_bridge):
            - armed=False / heartbeat kaybı → thrust sıfır
            - mode != GUIDED → cmd_vel yayınlanmaz (mavros zaten yok sayar)
        """
        gate = self._bridge.control_gate(self._now())

        u = self._pipe.compute_control()
        if u is None:                                # FSM parkur dışı → motor stop
            u = np.zeros(2)
        if gate.zero_thrust:                         # disarm / KILL → motor stop
            u = np.zeros(2)
        if self._odom_stale():                       # F-P.1: bayat pozla KÖR sürüş yok
            u = np.zeros(2)
            self.get_logger().warn(
                "odom bayat (F-P.1) — thrust sıfırlandı; GPS/EKF akışını kontrol et",
                throttle_duration_sec=5.0,
            )

        self._publish_thrust(u)
        if gate.allow_cmd_vel:                       # yalnız GUIDED + armed
            self._publish_cmd_vel(u)

    # ----- yayım yardımcıları -----

    def _publish_path(self, path) -> None:
        msg = Path()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        for x, y in path:
            ps = PoseStamped()
            ps.header = msg.header
            ps.pose.position.x = x
            ps.pose.position.y = y
            ps.pose.orientation.w = 1.0
            msg.poses.append(ps)
        self._pub_path.publish(msg)

    def _publish_thrust(self, u: np.ndarray) -> None:
        msg = Float32MultiArray()
        msg.data = [float(u[0]), float(u[1])]
        self._pub_thrust.publish(msg)

    def _publish_cmd_vel(self, u: np.ndarray) -> None:
        # Diferansiyel thruster → ileri sürat + yaw rate (kaba yaklaşım;
        # gerçek dönüşüm Cascade PID iç döngüsünde yapılır).
        p = self._pipe._dyn.p
        twist = Twist()
        twist.linear.x = float((u[0] + u[1]) / max(1.0, 2.0 * p.mass))
        twist.angular.z = float((u[1] - u[0]) / max(1e-6, p.inertia_z))
        self._pub_cmd_vel.publish(twist)

    def _publish_local_map(self) -> None:
        """Dosya-3: araç merkezli yerel maliyet haritası (OccupancyGrid).

        Frame base_link, origin (-w·res/2, -h·res/2) → araç pencere merkezinde,
        kuzey yukarı. Veri MPPI engel maliyetinden 0-100 normalize; arena dışı -1.
        """
        cg = self._pipe.local_cost_grid()
        msg = OccupancyGrid()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"
        msg.info.resolution = float(cg.resolution)
        msg.info.width = int(cg.width)
        msg.info.height = int(cg.height)
        msg.info.origin.position.x = -cg.width * cg.resolution / 2.0
        msg.info.origin.position.y = -cg.height * cg.resolution / 2.0
        msg.info.origin.orientation.w = 1.0
        msg.data = cg.data.tolist()
        self._pub_map.publish(msg)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = PlanningNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
