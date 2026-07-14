"""
Girdap İDA — Mock Sensör Yayıncısı (Layer 2 entegrasyon testi için).

ROS 2 ortamında fusion_node'u standalone test etmek üzere mavros
topic'lerini sentetik veriyle taklit eder.

Yörünge senaryosu (test_fusion_pipeline.py ile aynı):
    0-10 s   : düz, +x boyunca, u=1 m/s
    10-15 s  : 90° sol dönüş (ω = π/10 rad/s)
    15-30 s  : düz, +y boyunca

Yayınlanan topic'ler:
    /mavros/imu/data                 sensor_msgs/Imu       (50 Hz)
    /mavros/local_position/velocity_body  TwistStamped     (50 Hz)
    /mavros/global_position/global   sensor_msgs/NavSatFix (1 Hz)
    /girdap/groundtruth/pose         PoseStamped           (50 Hz, debug)

Tasarım notu:
    Hız ve yaw rate gürültülü; GPS de gürültülü. Origin sabit
    (Marmaris yarışma alanı temsili). Senaryo bittiğinde aracı
    durdurur ve sürekli son pozda yayın yapar.

Çalıştır:
    ros2 run girdap_decision mock_sensors
veya launch ile fusion_node'la birlikte:
    ros2 launch girdap_decision fusion_test.launch.py
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped, TwistStamped
from mavros_msgs.msg import State
from sensor_msgs.msg import Imu, NavSatFix

from girdap_decision.qos_profiles import sensor_data_qos


# Test'le aynı origin koordinatları (Marmaris ofset)
_LAT0 = 36.85
_LON0 = 28.27
_EARTH_R = 6378137.0


@dataclass
class _Phase:
    """Tek bir hareket fazı."""
    duration: float
    forward_speed: float    # m/s
    yaw_rate: float         # rad/s


_SCENARIO = [
    _Phase(duration=10.0, forward_speed=1.0, yaw_rate=0.0),
    _Phase(duration=5.0,  forward_speed=1.0, yaw_rate=math.pi / 10.0),
    _Phase(duration=15.0, forward_speed=1.0, yaw_rate=0.0),
]


def _enu_to_latlon(x: float, y: float) -> tuple[float, float]:
    """Origin (LAT0, LON0) etrafında eşit-dikdörtgensel ters projeksiyon."""
    cos_lat0 = math.cos(math.radians(_LAT0))
    lat = _LAT0 + math.degrees(y / _EARTH_R)
    lon = _LON0 + math.degrees(x / (_EARTH_R * cos_lat0))
    return lat, lon


class MockSensorsNode(Node):
    """Sentetik IMU + velocity_body + GPS yayını."""

    def __init__(self) -> None:
        super().__init__("mock_sensors")

        # --- Parametreler ---
        self.declare_parameter("imu_rate_hz", 50.0)
        self.declare_parameter("gps_rate_hz", 1.0)
        self.declare_parameter("vel_sigma", 0.05)        # m/s
        self.declare_parameter("omega_sigma", 0.005)     # rad/s
        self.declare_parameter("gps_sigma_xy", 0.30)     # m
        self.declare_parameter("seed", 0)

        self._vel_sigma = float(self.get_parameter("vel_sigma").value)
        self._omega_sigma = float(self.get_parameter("omega_sigma").value)
        self._gps_sigma = float(self.get_parameter("gps_sigma_xy").value)
        self._rng = np.random.default_rng(int(self.get_parameter("seed").value))

        # --- Yörünge durumu ---
        self._t = 0.0
        self._x = 0.0
        self._y = 0.0
        self._psi = 0.0

        # --- Publisher'lar: gerçek mavros gibi BEST_EFFORT (ortak profil) ---
        sensor_qos = sensor_data_qos()
        self._pub_imu = self.create_publisher(
            Imu, "/mavros/imu/data", sensor_qos
        )
        self._pub_vel = self.create_publisher(
            TwistStamped,
            "/mavros/local_position/velocity_body",
            sensor_qos,
        )
        self._pub_gps = self.create_publisher(
            NavSatFix, "/mavros/global_position/global", sensor_qos
        )
        self._pub_truth = self.create_publisher(
            PoseStamped, "/girdap/groundtruth/pose", 10
        )
        # /mavros/state RELIABLE yayınlanır (mavros konvansiyonu); fsm_node
        # armed durumunu buradan okur → ARM→BEKLEMEDE geçişi mock'ta mümkün olur.
        self._pub_state = self.create_publisher(State, "/mavros/state", 10)

        # --- Timer'lar ---
        imu_rate = float(self.get_parameter("imu_rate_hz").value)
        gps_rate = float(self.get_parameter("gps_rate_hz").value)
        self._dt_imu = 1.0 / imu_rate
        self._timer_imu = self.create_timer(self._dt_imu, self._on_imu_tick)
        self._timer_gps = self.create_timer(1.0 / gps_rate, self._on_gps_tick)
        self._timer_state = self.create_timer(1.0, self._on_state_tick)

        self.get_logger().info(
            f"mock_sensors aktif: IMU={imu_rate} Hz, GPS={gps_rate} Hz, "
            f"σ_vel={self._vel_sigma}, σ_ω={self._omega_sigma}, "
            f"σ_gps={self._gps_sigma} m"
        )

    # ----- yörünge yürütücüsü -----

    def _current_phase(self) -> _Phase | None:
        elapsed = 0.0
        for ph in _SCENARIO:
            if self._t < elapsed + ph.duration:
                return ph
            elapsed += ph.duration
        return None  # senaryo bitti — durağan kal

    # ----- IMU + velocity tick (50 Hz) -----

    def _on_imu_tick(self) -> None:
        ph = self._current_phase()
        if ph is None:
            u, w = 0.0, 0.0
        else:
            u, w = ph.forward_speed, ph.yaw_rate

        # Ground truth integrasyonu (Euler — IMU adımı küçük)
        self._x += u * math.cos(self._psi) * self._dt_imu
        self._y += u * math.sin(self._psi) * self._dt_imu
        self._psi += w * self._dt_imu
        self._t += self._dt_imu

        now = self.get_clock().now().to_msg()

        # IMU mesajı: yaw rate (gyro z) ana sinyal; orientation quaternion
        # da yaz (saha node'u ihtiyaç duyabilir).
        imu = Imu()
        imu.header.stamp = now
        imu.header.frame_id = "base_link"
        imu.angular_velocity.z = float(
            w + self._rng.normal(0.0, self._omega_sigma)
        )
        imu.orientation.z = math.sin(self._psi / 2.0)
        imu.orientation.w = math.cos(self._psi / 2.0)
        # Doğrusal ivme stub (fusion_node accel kullanmıyor)
        self._pub_imu.publish(imu)

        # Body-frame hız (gürültülü)
        vel = TwistStamped()
        vel.header.stamp = now
        vel.header.frame_id = "base_link"
        vel.twist.linear.x = float(
            u + self._rng.normal(0.0, self._vel_sigma)
        )
        vel.twist.linear.y = float(self._rng.normal(0.0, self._vel_sigma))
        vel.twist.angular.z = float(w)
        self._pub_vel.publish(vel)

        # Ground truth poz (test/RViz overlay için)
        gt = PoseStamped()
        gt.header.stamp = now
        gt.header.frame_id = "map"
        gt.pose.position.x = self._x
        gt.pose.position.y = self._y
        gt.pose.orientation.z = math.sin(self._psi / 2.0)
        gt.pose.orientation.w = math.cos(self._psi / 2.0)
        self._pub_truth.publish(gt)

    # ----- GPS tick (1 Hz) -----

    def _on_gps_tick(self) -> None:
        # Gürültülü ENU → lat/lon
        x_noisy = self._x + self._rng.normal(0.0, self._gps_sigma)
        y_noisy = self._y + self._rng.normal(0.0, self._gps_sigma)
        lat, lon = _enu_to_latlon(x_noisy, y_noisy)

        msg = NavSatFix()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "gps"
        msg.status.status = 2          # GBAS_FIX (RTK temsili)
        msg.status.service = 1         # SERVICE_GPS
        msg.latitude = lat
        msg.longitude = lon
        msg.altitude = 0.0
        self._pub_gps.publish(msg)

    def _on_state_tick(self) -> None:
        # Sahada Pixhawk arm edilmiş + GUIDED modda kabul edilir; mock bunu
        # taklit eder ki FSM ARM→BEKLEMEDE geçişi otomatik tamamlansın.
        msg = State()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.connected = True
        msg.armed = True
        msg.guided = True
        msg.mode = "GUIDED"
        self._pub_state.publish(msg)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = MockSensorsNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
