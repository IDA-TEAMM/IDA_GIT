#!/usr/bin/env python3
"""
IDA/Girdap USV - Telemetri Node
================================
Şartname zorunlu CSV kaydı:
  lat, lon, hız, roll, pitch, heading, hız/yön setpoint

Kaynak topic'ler:
  /mavros/global_position/global  → lat, lon
  /mavros/imu/data                → roll, pitch
  /mavros/vfr_hud                 → hız, heading
  /cmd_vel                        → hız/yön setpoint

Çıktı: ~/girdap_logs/telemetri/telemetri_YYYYMMDD_HHMMSS.csv (1Hz)

Yazar: IDA/Girdap Takım 989124 - Alt Alan B
"""

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from sensor_msgs.msg import NavSatFix, Imu
from geometry_msgs.msg import Twist
from mavros_msgs.msg import VfrHud

import csv
import math
import os
from datetime import datetime


class TelemetriNode(Node):
    def __init__(self):
        super().__init__('telemetri_node')
        self.cb_group = ReentrantCallbackGroup()
        self.qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10)

        # ── Veri alanları ─────────────────────────────────────────────────────
        self.lat        = 0.0
        self.lon        = 0.0
        self.hiz        = 0.0   # m/s
        self.heading    = 0.0   # derece (0-360)
        self.roll       = 0.0   # derece
        self.pitch      = 0.0   # derece
        self.setpoint_hiz  = 0.0  # cmd_vel linear
        self.setpoint_yon  = 0.0  # cmd_vel angular

        # ── CSV dosyası ───────────────────────────────────────────────────────
        # Sartname 4.2 Dosya-2 teslim dosyasi - /tmp KULLANMA: tmpfs'te
        # reboot/guc kesintisinde kaybolur (dosya basi 5 ceza puani).
        self.declare_parameter('output_dir', '')
        self.output_dir = self.get_parameter('output_dir').value or \
            os.path.expanduser('~/girdap_logs/telemetri')
        os.makedirs(self.output_dir, exist_ok=True)
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        self.csv_path = os.path.join(
            self.output_dir, f'telemetri_{timestamp}.csv')

        self.csv_file = open(self.csv_path, 'w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow([
            'lat', 'lon', 'hiz', 'roll', 'pitch',
            'heading', 'hiz_setpoint', 'yon_setpoint', 'zaman'
        ])
        self.get_logger().info(f'CSV kaydı başlatıldı: {self.csv_path}')

        # ── Subscribers ───────────────────────────────────────────────────────
        self.create_subscription(
            NavSatFix, '/mavros/global_position/global',
            self._gps_cb, self.qos, callback_group=self.cb_group)

        self.create_subscription(
            Imu, '/mavros/imu/data',
            self._imu_cb, self.qos, callback_group=self.cb_group)

        self.create_subscription(
            VfrHud, '/mavros/vfr_hud',
            self._vfr_cb, self.qos, callback_group=self.cb_group)

        self.create_subscription(
            Twist, '/cmd_vel',
            self._cmd_vel_cb, 10, callback_group=self.cb_group)

        # ── 1Hz kayıt timer'ı ─────────────────────────────────────────────────
        self.create_timer(1.0, self._kaydet)

        self.get_logger().info('Telemetri Node başlatıldı (1Hz CSV kaydı).')

    # ── Callback'ler ──────────────────────────────────────────────────────────

    def _gps_cb(self, msg: NavSatFix):
        self.lat = msg.latitude
        self.lon = msg.longitude

    def _imu_cb(self, msg: Imu):
        """Quaternion'dan roll/pitch hesapla."""
        q = msg.orientation
        # Roll (x ekseni etrafında dönme)
        sinr = 2.0 * (q.w * q.x + q.y * q.z)
        cosr = 1.0 - 2.0 * (q.x * q.x + q.y * q.y)
        self.roll = math.degrees(math.atan2(sinr, cosr))

        # Pitch (y ekseni etrafında dönme)
        sinp = 2.0 * (q.w * q.y - q.z * q.x)
        sinp = max(-1.0, min(1.0, sinp))
        self.pitch = math.degrees(math.asin(sinp))

    def _vfr_cb(self, msg: VfrHud):
        self.hiz     = msg.groundspeed   # m/s
        self.heading = float(msg.heading) # derece (0-360)

    def _cmd_vel_cb(self, msg: Twist):
        self.setpoint_hiz = msg.linear.x if msg.linear.x != 0.0 else msg.linear.y
        self.setpoint_yon = msg.angular.z

    # ── CSV kayıt ─────────────────────────────────────────────────────────────

    def _kaydet(self):
        """1Hz'de CSV'ye yaz."""
        zaman = datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]
        try:
            self.csv_writer.writerow([
                f'{self.lat:.8f}',
                f'{self.lon:.8f}',
                f'{self.hiz:.3f}',
                f'{self.roll:.2f}',
                f'{self.pitch:.2f}',
                f'{self.heading:.2f}',
                f'{self.setpoint_hiz:.3f}',
                f'{self.setpoint_yon:.3f}',
                zaman
            ])
            self.csv_file.flush()
            os.fsync(self.csv_file.fileno())
        except OSError as e:
            # Disk dolu / yazma hatasi: bu ornegi atla, node'u OLDURME - Dosya-2
            # zorunlu teslim dosyasi, tek satir kaybi tum kaydin durmasindan iyi.
            self.get_logger().error(
                f'CSV yazma hatasi (disk dolu olabilir): {e}',
                throttle_duration_sec=5.0)
            return

        self.get_logger().info(
            f'[TEL] lat={self.lat:.6f} lon={self.lon:.6f} '
            f'hız={self.hiz:.2f}m/s hdg={self.heading:.1f}° '
            f'roll={self.roll:.1f}° pitch={self.pitch:.1f}°',
            throttle_duration_sec=5.0)

    def destroy_node(self):
        """Node kapatılırken CSV dosyasını kapat."""
        self.csv_file.close()
        self.get_logger().info(f'CSV kaydı tamamlandı: {self.csv_path}')
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = TelemetriNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass  # launch/systemd SIGINT'i normal kapanistir (traceback basma)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
