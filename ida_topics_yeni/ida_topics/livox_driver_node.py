#!/usr/bin/env python3
"""
IDA/Girdap USV - Livox Mid-360 LiDAR Driver Node
==================================================
Livox Mid-360 UDP protokolü üzerinden nokta bulutu okuma

Livox Mid-360 özellikleri:
  - 360° FOV (yatay), -7° ~ +52° (dikey)
  - 200m menzil
  - 200,000 nokta/saniye
  - UDP port: 56100 (data), 56200 (command)
  - IP: 192.168.1.1xx (varsayılan)

Publish:
  /lidar/points  → sensor_msgs/PointCloud2
  /lidar/scan    → sensor_msgs/LaserScan (2D dilim)

Gereksinim: pip3 install livoxsdk2 (veya resmi Livox ROS2 driver)

NOT: Resmi Livox ROS2 driver tercih edilmeli:
  https://github.com/Livox-SDK/livox_ros_driver2
  Bu node, driver kurulu değilse fallback olarak çalışır.

Yazar: IDA/Girdap Takım 989124 - Alt Alan B
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import PointCloud2, PointField, LaserScan
from std_msgs.msg import Header

import socket
import struct
import threading
import numpy as np
import math


# Livox Mid-360 UDP protokol sabitleri
LIVOX_DATA_PORT    = 56100
LIVOX_CMD_PORT     = 56200
LIVOX_DEVICE_IP    = '192.168.1.100'  # Varsayılan Mid-360 IP
POINT_STRUCT_SIZE  = 14  # Her nokta: x(4) + y(4) + z(4) + intensity(1) + tag(1)


class LivoxDriverNode(Node):
    def __init__(self):
        super().__init__('livox_driver_node')

        # ── Parametreler ──────────────────────────────────────────────────────
        self.declare_parameter('device_ip', LIVOX_DEVICE_IP)
        self.declare_parameter('data_port', LIVOX_DATA_PORT)
        self.declare_parameter('host_ip', '192.168.1.5')
        self.declare_parameter('scan_height', 0.0)   # 2D dilim yüksekliği (m)
        self.declare_parameter('scan_thickness', 0.3) # Dilim kalınlığı (m)

        self.device_ip      = self.get_parameter('device_ip').value
        self.data_port      = self.get_parameter('data_port').value
        self.host_ip        = self.get_parameter('host_ip').value
        self.scan_height    = self.get_parameter('scan_height').value
        self.scan_thickness = self.get_parameter('scan_thickness').value

        # ── Publishers ────────────────────────────────────────────────────────
        self.pc_pub   = self.create_publisher(
            PointCloud2, '/lidar/points', 10)
        self.scan_pub = self.create_publisher(
            LaserScan, '/lidar/scan', 10)

        # ── UDP socket ────────────────────────────────────────────────────────
        self.sock    = None
        self.running = True
        self._init_socket()

        # ── Nokta tamponu ─────────────────────────────────────────────────────
        self.point_buffer = []
        self.buffer_lock  = threading.Lock()

        # ── Thread'ler ────────────────────────────────────────────────────────
        self.recv_thread = threading.Thread(
            target=self._recv_loop, daemon=True)
        self.recv_thread.start()

        # 10Hz'de publish
        self.create_timer(0.1, self._publish)

        self.get_logger().info(
            f'Livox Mid-360 Driver başlatıldı | '
            f'Device: {self.device_ip}:{self.data_port}')

    def _init_socket(self):
        """UDP socket'i başlat."""
        try:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self.sock.bind((self.host_ip, self.data_port))
            self.sock.settimeout(1.0)
            self.get_logger().info(
                f'UDP socket açıldı: {self.host_ip}:{self.data_port}')
        except Exception as e:
            self.get_logger().error(f'UDP socket hatası: {e}')
            self.sock = None

    def _recv_loop(self):
        """UDP paketlerini sürekli al."""
        while self.running:
            if self.sock is None:
                import time
                time.sleep(1.0)
                self._init_socket()
                continue

            try:
                data, addr = self.sock.recvfrom(65535)
                points = self._parse_packet(data)
                if points:
                    with self.buffer_lock:
                        self.point_buffer.extend(points)
                        # Buffer'ı sınırla (max 100k nokta)
                        if len(self.point_buffer) > 100000:
                            self.point_buffer = self.point_buffer[-50000:]

            except socket.timeout:
                self.get_logger().debug(
                    'LiDAR veri bekliyor...',
                    throttle_duration_sec=5.0)
            except Exception as e:
                self.get_logger().error(
                    f'Alım hatası: {e}',
                    throttle_duration_sec=5.0)

    def _parse_packet(self, data: bytes):
        """Livox UDP paketini ayrıştır."""
        points = []
        try:
            # Livox paket başlığını atla (ilk 28 byte)
            offset = 28
            while offset + POINT_STRUCT_SIZE <= len(data):
                x, y, z = struct.unpack_from('<fff', data, offset)
                intensity = data[offset + 12]
                offset += POINT_STRUCT_SIZE
                # Geçerli nokta kontrolü
                if not (math.isnan(x) or math.isnan(y) or math.isnan(z)):
                    if x != 0.0 or y != 0.0 or z != 0.0:
                        points.append((x/1000.0, y/1000.0, z/1000.0, intensity))
        except Exception:
            pass
        return points

    def _publish(self):
        """Biriken noktaları PointCloud2 ve LaserScan olarak yayınla."""
        with self.buffer_lock:
            if not self.point_buffer:
                return
            points = list(self.point_buffer)
            self.point_buffer.clear()

        now = self.get_clock().now().to_msg()

        # ── PointCloud2 ───────────────────────────────────────────────────────
        arr = np.array(points, dtype=np.float32)

        pc_msg = PointCloud2()
        pc_msg.header.stamp    = now
        pc_msg.header.frame_id = 'livox_frame'
        pc_msg.height = 1
        pc_msg.width  = len(points)
        pc_msg.fields = [
            PointField(name='x', offset=0,  datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4,  datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8,  datatype=PointField.FLOAT32, count=1),
            PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
        ]
        pc_msg.is_bigendian = False
        pc_msg.point_step   = 16
        pc_msg.row_step     = 16 * len(points)
        pc_msg.data         = arr.tobytes()
        pc_msg.is_dense     = True
        self.pc_pub.publish(pc_msg)

        # ── LaserScan (2D dilim) ──────────────────────────────────────────────
        self._publish_scan(points, now)

        self.get_logger().info(
            f'LiDAR: {len(points)} nokta yayınlandı',
            throttle_duration_sec=2.0)

    def _publish_scan(self, points, stamp):
        """3D nokta bulutundan 2D LaserScan oluştur."""
        # Yalnızca belirli yükseklikteki noktaları al
        z_min = self.scan_height - self.scan_thickness / 2
        z_max = self.scan_height + self.scan_thickness / 2

        scan_points = [(x, y, z) for x, y, z, _ in points
                       if z_min <= z <= z_max]

        if not scan_points:
            return

        scan_msg = LaserScan()
        scan_msg.header.stamp    = stamp
        scan_msg.header.frame_id = 'livox_frame'
        scan_msg.angle_min       = -math.pi
        scan_msg.angle_max       =  math.pi
        scan_msg.angle_increment = math.pi / 180.0  # 1 derece
        scan_msg.range_min       = 0.1
        scan_msg.range_max       = 200.0

        num_bins = int(2 * math.pi / scan_msg.angle_increment)
        ranges   = [float('inf')] * num_bins

        for x, y, z in scan_points:
            angle = math.atan2(y, x)
            dist  = math.sqrt(x*x + y*y)
            idx   = int((angle + math.pi) / scan_msg.angle_increment)
            if 0 <= idx < num_bins:
                if dist < ranges[idx]:
                    ranges[idx] = dist

        scan_msg.ranges = ranges
        self.scan_pub.publish(scan_msg)

    def destroy_node(self):
        self.running = False
        if self.sock:
            self.sock.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = LivoxDriverNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
