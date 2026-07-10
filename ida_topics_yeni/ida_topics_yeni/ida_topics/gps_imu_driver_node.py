#!/usr/bin/env python3
"""
IDA/Girdap USV - GPS + IMU Driver Node
========================================
SparkFun RTK F9P (ZED-F9P) UART/USB driver

NMEA protokolü üzerinden:
  - $GNGGA → GPS konum (lat, lon, alt)
  - $GNRMC → Hız, heading
  - $PSSN,IMU → IMU verisi (bazı modüllerde)

Publish:
  /gps/fix    → sensor_msgs/NavSatFix
  /imu/data   → sensor_msgs/Imu (varsa)

Bağlantı: USB/UART (/dev/ttyUSB0 veya /dev/ttyACM0)
Baud: 115200 (varsayılan F9P)

Yazar: IDA/Girdap Takım 989124 - Alt Alan B
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, NavSatStatus, Imu
from std_msgs.msg import Header

import serial
import serial.tools.list_ports
import threading
import math


class GpsImuDriverNode(Node):
    def __init__(self):
        super().__init__('gps_imu_driver_node')

        # ── Parametreler ──────────────────────────────────────────────────────
        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baud', 115200)
        self.declare_parameter('timeout', 1.0)

        self.port    = self.get_parameter('port').value
        self.baud    = self.get_parameter('baud').value
        self.timeout = self.get_parameter('timeout').value

        # ── Publishers ────────────────────────────────────────────────────────
        self.gps_pub = self.create_publisher(NavSatFix, '/gps/fix', 10)
        self.imu_pub = self.create_publisher(Imu, '/imu/data', 10)

        # ── Serial bağlantı ───────────────────────────────────────────────────
        self.serial = None
        self.running = True
        self._connect()

        # ── Okuma thread'i ────────────────────────────────────────────────────
        self.thread = threading.Thread(target=self._read_loop, daemon=True)
        self.thread.start()

        self.get_logger().info(
            f'GPS/IMU Driver başlatıldı: {self.port} @ {self.baud} baud')

    def _connect(self):
        """Serial porta bağlan."""
        try:
            self.serial = serial.Serial(
                port=self.port,
                baudrate=self.baud,
                timeout=self.timeout)
            self.get_logger().info(f'Serial bağlantı kuruldu: {self.port}')
        except serial.SerialException as e:
            self.get_logger().error(f'Serial bağlantı hatası: {e}')
            self.get_logger().error(
                'Mevcut portlar: ' +
                str([p.device for p in serial.tools.list_ports.comports()]))
            self.serial = None

    def _read_loop(self):
        """Sürekli NMEA satırları oku ve işle."""
        while self.running:
            if self.serial is None or not self.serial.is_open:
                self.get_logger().warn('Serial port kapalı, yeniden bağlanıyor...',
                                       throttle_duration_sec=5.0)
                self._connect()
                continue

            try:
                line = self.serial.readline().decode('ascii', errors='ignore').strip()
                if line:
                    self._parse_nmea(line)
            except serial.SerialException as e:
                self.get_logger().error(f'Okuma hatası: {e}')
                self.serial = None

    def _parse_nmea(self, line: str):
        """NMEA cümlesini ayrıştır ve publish et."""
        try:
            if line.startswith('$GNGGA') or line.startswith('$GPGGA'):
                self._parse_gga(line)
            elif line.startswith('$GNRMC') or line.startswith('$GPRMC'):
                self._parse_rmc(line)
        except Exception as e:
            self.get_logger().debug(f'NMEA parse hatası: {e} | Satır: {line}')

    def _parse_gga(self, line: str):
        """$GNGGA: GPS fix verisi."""
        parts = line.split(',')
        if len(parts) < 10:
            return

        lat_str  = parts[2]
        lat_dir  = parts[3]
        lon_str  = parts[4]
        lon_dir  = parts[5]
        fix_qual = parts[6]
        alt_str  = parts[9]

        if not lat_str or not lon_str:
            return

        # DDMM.MMMMM → ondalık derece
        lat = self._nmea_to_decimal(lat_str, lat_dir)
        lon = self._nmea_to_decimal(lon_str, lon_dir)
        alt = float(alt_str) if alt_str else 0.0

        # Fix kalitesi
        fix = int(fix_qual) if fix_qual else 0

        msg = NavSatFix()
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = 'gps'
        msg.latitude        = lat
        msg.longitude       = lon
        msg.altitude        = alt

        if fix == 0:
            msg.status.status = NavSatStatus.STATUS_NO_FIX
        elif fix in [1, 2]:
            msg.status.status = NavSatStatus.STATUS_FIX
        elif fix in [4, 5]:
            msg.status.status = NavSatStatus.STATUS_GBAS_FIX  # RTK

        msg.status.service = NavSatStatus.SERVICE_GPS

        # Konum kovaryansı (RTK ise daha küçük)
        if fix >= 4:  # RTK fix
            cov = 0.01
        elif fix >= 1:
            cov = 1.0
        else:
            cov = 9999.0

        msg.position_covariance = [
            cov, 0.0, 0.0,
            0.0, cov, 0.0,
            0.0, 0.0, cov * 4
        ]
        msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN

        self.gps_pub.publish(msg)
        self.get_logger().info(
            f'GPS: lat={lat:.7f} lon={lon:.7f} alt={alt:.1f}m fix={fix}',
            throttle_duration_sec=2.0)

    def _parse_rmc(self, line: str):
        """$GNRMC: Hız ve heading (şimdilik sadece log)."""
        parts = line.split(',')
        if len(parts) < 9:
            return
        status = parts[2]  # A=active, V=void
        if status != 'A':
            return
        speed_knots = parts[7]
        heading     = parts[8]
        if speed_knots:
            speed_ms = float(speed_knots) * 0.514444
            self.get_logger().debug(
                f'RMC: hız={speed_ms:.2f}m/s hdg={heading}°',
                throttle_duration_sec=2.0)

    @staticmethod
    def _nmea_to_decimal(value: str, direction: str) -> float:
        """DDMM.MMMMM formatını ondalık dereceye çevir."""
        if not value:
            return 0.0
        dot_pos = value.index('.')
        degrees = float(value[:dot_pos - 2])
        minutes = float(value[dot_pos - 2:])
        decimal = degrees + minutes / 60.0
        if direction in ['S', 'W']:
            decimal = -decimal
        return decimal

    def destroy_node(self):
        self.running = False
        if self.serial and self.serial.is_open:
            self.serial.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = GpsImuDriverNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
