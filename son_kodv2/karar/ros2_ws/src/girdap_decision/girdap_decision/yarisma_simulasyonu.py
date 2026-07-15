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
        `use_real_course_obstacles:=true` ile tek sahte blob yerine GERÇEK
        parkur dosyasından (p_GUNCEL_20260625.world, Parkur-2 "engel1-9" sarı
        şamandıraları) alınan 9 sabit dünya-konumu, aracın CANLI GPS/IMU'suna
        göre her tick'te base_link'e dönüştürülüp menzil-filtrelenerek (gerçek
        perception_lidar_node'un max_range davranışını taklit eder) yayınlanır.
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

Gerçek parkur testi (use_real_course_obstacles:=true, course_home_lat/lon
görev dosyasının home_ref'iyle AYNI olmalı — aksi halde engeller yanlış yerde
görünür):
    ros2 run girdap_decision yarisma_simulasyonu --ros-args \\
        -p use_real_course_obstacles:=true \\
        -p course_home_lat:=40.7160 -p course_home_lon:=31.5249
"""

from __future__ import annotations

import math

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Pose, PoseArray
from sensor_msgs.msg import Imu, NavSatFix
from std_msgs.msg import Bool, Int32

from girdap_decision.qos_profiles import sensor_data_qos
from prototype.mission.mission_manager import latlon_to_enu
from prototype.telemetry.csv_logger import quat_to_rpy

# p_GUNCEL_20260625.world — Parkur-2 "PARKUR 2" bölümündeki 9 sarı engel
# şamandırası (sari_buoy, "engel1".."engel9"), dünya-çerçevesi ENU (east,
# north) metre — course_home_lat/lon parametresine göre sabit, taşınmaz.
_REAL_ENGEL_BUOYS_ENU = [
    (43.0, 3.5), (47.0, -3.5), (51.0, 3.5), (55.0, -3.5), (59.0, 3.5),
    (63.0, -3.5), (67.0, 3.5), (71.0, -3.5), (75.0, 3.5),
]
_REAL_BUOY_RADIUS_M = 0.3   # sari_buoy fiziksel yarıçapı (gövde+güvenlik payı)


class YarismaSimulasyonuNode(Node):
    """Donanım-yok yarışma simülasyonu — algı/darbe sinyali üretici."""

    def __init__(self) -> None:
        super().__init__("yarisma_simulasyonu")

        self.declare_parameter("obstacle_x", 5.0)   # base_link'e göre ENU (m)
        self.declare_parameter("obstacle_y", 1.0)
        self.declare_parameter("obstacle_radius_m", 1.2)
        self.declare_parameter("shock_accel_mss", 60.0)  # ~6.1g (>5.0g varsayılan eşik)
        # Gerçek parkur modu (p_GUNCEL_20260625.world Parkur-2 engelleri).
        self.declare_parameter("use_real_course_obstacles", False)
        self.declare_parameter("course_home_lat", 0.0)
        self.declare_parameter("course_home_lon", 0.0)
        self.declare_parameter("obstacle_range_m", 25.0)  # perception_lidar max_range ile aynı

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
        # Gerçek-parkur modu: aracın canlı GPS/heading'i — dünya-konumundaki
        # sabit engelleri her tick base_link'e dönüştürmek için gerekli.
        self._veh_lat: float | None = None
        self._veh_lon: float | None = None
        self._veh_yaw: float = 0.0
        self._sub_gps = self.create_subscription(
            NavSatFix, "/mavros/global_position/global",
            self._on_gps, sensor_data_qos(),
        )
        self._sub_imu_pose = self.create_subscription(
            Imu, "/mavros/imu/data", self._on_imu_pose, sensor_data_qos()
        )

        self._current_parkur = 1
        self._gate_sent = False
        self._impact_sent = False
        self._timer = self.create_timer(1.0, self._on_tick)

        mode = (
            "GERÇEK PARKUR (p_GUNCEL_20260625.world, 9 engel şamandırası)"
            if bool(self.get_parameter("use_real_course_obstacles").value)
            else "basit tek sahte engel"
        )
        self.get_logger().info(
            f"yarisma_simulasyonu aktif — engel modu: {mode} (1 Hz) + "
            "gate_passed/IMU-darbe (parkur geçişini izleyerek tek atış)"
        )

    def _on_parkur(self, msg: Int32) -> None:
        self._current_parkur = int(msg.data)

    def _on_gps(self, msg: NavSatFix) -> None:
        if msg.status.status >= 0:
            self._veh_lat = msg.latitude
            self._veh_lon = msg.longitude

    def _on_imu_pose(self, msg: Imu) -> None:
        q = msg.orientation
        _, _, yaw = quat_to_rpy(q.x, q.y, q.z, q.w)
        self._veh_yaw = yaw

    def _real_course_obstacle_poses(self) -> list[Pose]:
        """Dünya-sabit engelleri aracın canlı GPS/heading'ine göre base_link'e
        çevirir; yalnız `obstacle_range_m` içindekiler döner (gerçek LiDAR
        menzil filtresini taklit eder — perception_lidar_node max_range)."""
        if self._veh_lat is None or self._veh_lon is None:
            return []
        home_lat = float(self.get_parameter("course_home_lat").value)
        home_lon = float(self.get_parameter("course_home_lon").value)
        veh_east, veh_north = latlon_to_enu(
            home_lat, home_lon, self._veh_lat, self._veh_lon
        )
        rng = float(self.get_parameter("obstacle_range_m").value)
        cos_yaw, sin_yaw = math.cos(self._veh_yaw), math.sin(self._veh_yaw)
        poses = []
        for east, north in _REAL_ENGEL_BUOYS_ENU:
            dx, dy = east - veh_east, north - veh_north
            if math.hypot(dx, dy) > rng:
                continue
            # Dünya-ENU → base_link (araç heading'ine göre ters döndür).
            bx = dx * cos_yaw + dy * sin_yaw
            by = -dx * sin_yaw + dy * cos_yaw
            p = Pose()
            p.position.x = bx
            p.position.y = by
            p.orientation.z = _REAL_BUOY_RADIUS_M
            p.orientation.w = 1.0
            poses.append(p)
        return poses

    def _on_tick(self) -> None:
        obs = PoseArray()
        obs.header.frame_id = "base_link"
        obs.header.stamp = self.get_clock().now().to_msg()
        if bool(self.get_parameter("use_real_course_obstacles").value):
            obs.poses = self._real_course_obstacle_poses()
        else:
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
