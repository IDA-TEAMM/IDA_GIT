#!/usr/bin/env python3
"""
IDA/Girdap USV - GPS + IMU Driver Node
========================================
Holybro H-RTK F9P Rover (IST8310 Compass) → MAVROS köprüsü

NOT (2026-07-13): Bu modül tek bir birleşik konnektörle Pixhawk'a bağlanıyor
(GPS UART + kompas I2C aynı kabloda), bağımsız bir ikinci UART çıkışı yok.
Ayrıca Pixhawk'a bağlandığında ArduPilot modülü kendi NMEA parser'ının
beklediği düz NMEA yerine UBX ikili protokolüne göre yapılandırıyor - yani
ham seri port okuma (eski yaklaşım) bu donanımla çalışmıyor. Bunun yerine
MAVROS'un zaten Pixhawk'tan aldığı GPS/IMU verisini /gps/fix ve /imu/data'ya
yeniden yayınlıyoruz; sensor_node.py ve sonrası hiç değişmeden çalışmaya
devam ediyor (driver katmanı "gerçek donanım → standart topic" görevini
MAVROS üzerinden yerine getiriyor).

Subscribe (MAVROS, BEST_EFFORT QoS gerekli):
  /mavros/global_position/global → sensor_msgs/NavSatFix
  /mavros/imu/data                → sensor_msgs/Imu

Publish:
  /gps/fix    → sensor_msgs/NavSatFix
  /imu/data   → sensor_msgs/Imu

Yazar: IDA/Girdap Takım 989124 - Alt Alan B
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import NavSatFix, Imu


class GpsImuDriverNode(Node):
    def __init__(self):
        super().__init__('gps_imu_driver_node')

        mavros_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10)

        # ── Publishers ────────────────────────────────────────────────────────
        self.gps_pub = self.create_publisher(NavSatFix, '/gps/fix', 10)
        self.imu_pub = self.create_publisher(Imu, '/imu/data', 10)

        # ── MAVROS subscriber'lar ────────────────────────────────────────────
        self.create_subscription(
            NavSatFix, '/mavros/global_position/global',
            self._gps_callback, mavros_qos)
        self.create_subscription(
            Imu, '/mavros/imu/data',
            self._imu_callback, mavros_qos)

        self.get_logger().info(
            'GPS/IMU Driver başlatıldı (MAVROS köprüsü: '
            '/mavros/global_position/global, /mavros/imu/data)')

    def _gps_callback(self, msg: NavSatFix):
        msg.header.frame_id = 'gps'
        self.gps_pub.publish(msg)
        self.get_logger().info(
            f'GPS: lat={msg.latitude:.7f} lon={msg.longitude:.7f} '
            f'alt={msg.altitude:.1f}m',
            throttle_duration_sec=2.0)

    def _imu_callback(self, msg: Imu):
        self.imu_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = GpsImuDriverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass  # launch/systemd SIGINT'i normal kapanıştır (traceback basma)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
