"""
Girdap İDA — Yarışma Simülasyon Veri Üretici (Layer 2, donanım-yok testi).

Donanım ekibi (Livox/OAK-D/Pixhawk) hazır olmadığında `son_kodv2`'yi TAM
yarışma modunda (use_rrt=true, use_isam2=true, Parkur-1→2→3) uçtan uca test
etmek için `mock_sensors.py`'nin (GPS/IMU/state) YANINDA çalışır; onun
kapsamadığı algı/darbe sinyallerini sentetik üretir:

    /perception/obstacle_map   geometry_msgs/PoseArray   (1 Hz)
        Sahte LiDAR engeli — RRT*/MPPI kaçınma yolunun gerçekten tetiklendiğini
        doğrulamak için sürekli yayınlanır. CLAUDE.md sözleşmesi:
        position.xy = merkez, orientation.z = çevrel yarıçap (quaternion DEĞİL).
    /perception/gate_passed    std_msgs/Bool             (tek atış)
        PARKUR2→PARKUR3 tetiği — /girdap/mission/current_parkur 2'ye
        geçtiğinde elle gönderilir (gerçek duba-ikilisi geçişi yerine).
    /mavros/imu/data           sensor_msgs/Imu           (tek atış)
        PARKUR3→TAMAMLANDI tetiği — mock_sensors'ın sürekli düşük-genlikli
        akışının YANINA, current_parkur 3'e geçtiğinde tek bir yüksek-genlik
        darbe (shock_threshold_g üstü) eklenir.

mock_sensors.py'nin kapsamadığı gerçek node'ları (perception_lidar_node,
perception_camera_node, perception_fusion_node) BAYPAS EDER — bu script
onların ÇIKTI sözleşmesini taklit eder, kendilerini çalıştırmaz (donanım
girdisi olmadan zaten anlamlı üretemezler).

Çalıştır (mock_sensors + tam yarışma modu karar yığınıyla BİRLİKTE):
    ros2 launch girdap_decision hardware.launch.py with_mavros:=false \\
        fsm.start_on_mode:=GUIDED fsm.start_on_arm_in_mode:=true \\
        use_isam2:=true use_rrt:=true \\
        mission_source:=file mission_file:=masa_test_mission.yaml
    ros2 run girdap_decision yarisma_simulasyonu
"""

from __future__ import annotations

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Pose, PoseArray
from sensor_msgs.msg import Imu
from std_msgs.msg import Bool, Int32

from girdap_decision.qos_profiles import sensor_data_qos


class YarismaSimulasyonuNode(Node):
    """Donanım-yok yarışma simülasyonu — algı/darbe sinyali üretici."""

    def __init__(self) -> None:
        super().__init__("yarisma_simulasyonu")

        self.declare_parameter("obstacle_x", 5.0)   # base_link'e göre ENU (m)
        self.declare_parameter("obstacle_y", 1.0)
        self.declare_parameter("obstacle_radius_m", 1.2)
        self.declare_parameter("shock_accel_mss", 60.0)  # ~6.1g (>5.0g varsayılan eşik)

        self._pub_obs = self.create_publisher(
            PoseArray, "/perception/obstacle_map", 10
        )
        self._pub_gate = self.create_publisher(
            Bool, "/perception/gate_passed", 10
        )
        # fsm_node /mavros/imu/data'yı sensor_data_qos (BEST_EFFORT) ile
        # dinliyor — mock_sensors'la aynı profil, yoksa mesaj sessizce düşer.
        self._pub_imu = self.create_publisher(
            Imu, "/mavros/imu/data", sensor_data_qos()
        )
        self._sub_parkur = self.create_subscription(
            Int32, "/girdap/mission/current_parkur", self._on_parkur, 10
        )

        self._current_parkur = 1
        self._gate_sent = False
        self._impact_sent = False
        self._timer = self.create_timer(1.0, self._on_tick)

        self.get_logger().info(
            "yarisma_simulasyonu aktif — sahte engel (1 Hz) + "
            "gate_passed/IMU-darbe (parkur geçişini izleyerek tek atış)"
        )

    def _on_parkur(self, msg: Int32) -> None:
        self._current_parkur = int(msg.data)

    def _on_tick(self) -> None:
        obs = PoseArray()
        obs.header.frame_id = "base_link"
        obs.header.stamp = self.get_clock().now().to_msg()
        p = Pose()
        p.position.x = float(self.get_parameter("obstacle_x").value)
        p.position.y = float(self.get_parameter("obstacle_y").value)
        p.orientation.z = float(self.get_parameter("obstacle_radius_m").value)
        p.orientation.w = 1.0
        obs.poses.append(p)
        self._pub_obs.publish(obs)

        if self._current_parkur == 2 and not self._gate_sent:
            self._gate_sent = True
            self.get_logger().info(
                "PARKUR2 görüldü → sahte gate_passed (PARKUR2→PARKUR3)"
            )
            self._pub_gate.publish(Bool(data=True))

        if self._current_parkur == 3 and not self._impact_sent:
            self._impact_sent = True
            self.get_logger().info(
                "PARKUR3 görüldü → sahte IMU çarpma darbesi (PARKUR3→TAMAMLANDI)"
            )
            shock = Imu()
            shock.header.stamp = self.get_clock().now().to_msg()
            shock.header.frame_id = "base_link"
            shock.linear_acceleration.x = float(
                self.get_parameter("shock_accel_mss").value
            )
            self._pub_imu.publish(shock)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = YarismaSimulasyonuNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
