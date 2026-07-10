#!/usr/bin/env python3
"""
IDA/Girdap USV - Local Map Node
=================================
Şartname zorunlu local map/costmap kaydı (≥1Hz)

LiDAR scan verisinden basit 2D OccupancyGrid oluşturur.
TF gerektirmez, doğrudan /lidar/scan okur.

Yayınlar:
  /local_map  → nav_msgs/OccupancyGrid (1Hz)

Kaydeder:
  /tmp/local_map/map_YYYYMMDD_HHMMSS.pgm (PGM görüntü formatı)
  /tmp/local_map/map_YYYYMMDD_HHMMSS.yaml (metadata)

Yazar: IDA/Girdap Takım 989124 - Alt Alan B
"""

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Header

import numpy as np
import math
import os
from datetime import datetime


class LocalMapNode(Node):
    def __init__(self):
        super().__init__('local_map_node')
        self.cb_group = ReentrantCallbackGroup()

        # ── Harita parametreleri ──────────────────────────────────────────────
        self.resolution  = 0.1   # metre/hücre
        self.width_m     = 20.0  # harita genişliği (metre)
        self.height_m    = 20.0  # harita yüksekliği (metre)
        self.width       = int(self.width_m / self.resolution)   # hücre sayısı
        self.height      = int(self.height_m / self.resolution)  # hücre sayısı
        self.origin_x    = -self.width_m / 2.0
        self.origin_y    = -self.height_m / 2.0

        # ── Durum ─────────────────────────────────────────────────────────────
        self.latest_scan = None
        self.map_count   = 0

        # ── Çıktı klasörü ─────────────────────────────────────────────────────
        os.makedirs('/tmp/local_map', exist_ok=True)
        self.timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')

        # ── Publisher ─────────────────────────────────────────────────────────
        self.map_pub = self.create_publisher(
            OccupancyGrid, '/local_map', 10)

        # ── Subscriber ────────────────────────────────────────────────────────
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10)

        self.create_subscription(
            LaserScan, '/lidar/scan',
            self._scan_cb, qos,
            callback_group=self.cb_group)

        # ── 1Hz timer ─────────────────────────────────────────────────────────
        self.create_timer(1.0, self._yayinla,
                          callback_group=self.cb_group)

        self.get_logger().info(
            f'Local Map Node başlatıldı '
            f'({self.width}x{self.height} hücre, '
            f'{self.resolution}m/hücre, '
            f'{self.width_m}x{self.height_m}m alan)')

    # ── Callback ──────────────────────────────────────────────────────────────

    def _scan_cb(self, msg: LaserScan):
        self.latest_scan = msg

    # ── Harita oluştur ve yayınla ─────────────────────────────────────────────

    def _yayinla(self):
        """1Hz'de OccupancyGrid oluşturup yayınla ve kaydet."""
        # Boş harita (bilinmiyor = -1)
        grid = np.full((self.height, self.width), -1, dtype=np.int8)

        # Merkezi serbest alan olarak işaretle
        cx = self.width // 2
        cy = self.height // 2
        r  = int(1.0 / self.resolution)  # 1 metre yarıçap
        for dy in range(-r, r+1):
            for dx in range(-r, r+1):
                if dx*dx + dy*dy <= r*r:
                    nx, ny = cx+dx, cy+dy
                    if 0 <= nx < self.width and 0 <= ny < self.height:
                        grid[ny][nx] = 0  # serbest

        # LiDAR verisini haritaya işle
        if self.latest_scan is not None:
            scan = self.latest_scan
            angle = scan.angle_min
            for r_val in scan.ranges:
                if scan.range_min < r_val < scan.range_max:
                    # Polar → Kartezyen
                    x = r_val * math.cos(angle)
                    y = r_val * math.sin(angle)

                    # Harita hücresi
                    gx = int((x - self.origin_x) / self.resolution)
                    gy = int((y - self.origin_y) / self.resolution)

                    if 0 <= gx < self.width and 0 <= gy < self.height:
                        grid[gy][gx] = 100  # engel

                    # Ray trace — engele kadar serbest
                    steps = int(r_val / self.resolution)
                    for s in range(1, steps):
                        fx = (s * self.resolution) * math.cos(angle)
                        fy = (s * self.resolution) * math.sin(angle)
                        fgx = int((fx - self.origin_x) / self.resolution)
                        fgy = int((fy - self.origin_y) / self.resolution)
                        if 0 <= fgx < self.width and 0 <= fgy < self.height:
                            if grid[fgy][fgx] != 100:
                                grid[fgy][fgx] = 0

                angle += scan.angle_increment

        # ── OccupancyGrid mesajı ──────────────────────────────────────────────
        msg = OccupancyGrid()
        msg.header = Header()
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'

        msg.info.resolution = self.resolution
        msg.info.width      = self.width
        msg.info.height     = self.height
        msg.info.origin.position.x = self.origin_x
        msg.info.origin.position.y = self.origin_y
        msg.info.origin.orientation.w = 1.0

        msg.data = grid.flatten().tolist()
        self.map_pub.publish(msg)

        # ── PGM kaydet (her 10 saniyede bir) ─────────────────────────────────
        self.map_count += 1
        if self.map_count % 10 == 0:
            self._pgm_kaydet(grid)

        engel_sayisi = int(np.sum(grid == 100))
        self.get_logger().info(
            f'[MAP] {self.width}x{self.height} harita yayınlandı, '
            f'engel={engel_sayisi} hücre',
            throttle_duration_sec=5.0)

    def _pgm_kaydet(self, grid: np.ndarray):
        """Haritayı PGM formatında kaydet."""
        zaman = datetime.now().strftime('%Y%m%d_%H%M%S')
        pgm_path  = f'/tmp/local_map/map_{zaman}.pgm'
        yaml_path = f'/tmp/local_map/map_{zaman}.yaml'

        # OccupancyGrid → görüntü (0=beyaz/serbest, 100=siyah/engel, -1=gri/bilinmiyor)
        img = np.zeros((self.height, self.width), dtype=np.uint8)
        img[grid == 0]   = 255  # serbest → beyaz
        img[grid == 100] = 0    # engel   → siyah
        img[grid == -1]  = 128  # bilinmiyor → gri

        # PGM yaz
        with open(pgm_path, 'wb') as f:
            f.write(f'P5\n{self.width} {self.height}\n255\n'.encode())
            f.write(img.tobytes())

        # YAML metadata
        with open(yaml_path, 'w') as f:
            f.write(f"""image: {pgm_path}
resolution: {self.resolution}
origin: [{self.origin_x}, {self.origin_y}, 0.0]
negate: 0
occupied_thresh: 0.65
free_thresh: 0.196
""")
        self.get_logger().info(f'Harita kaydedildi: {pgm_path}')


def main(args=None):
    rclpy.init(args=args)
    node = LocalMapNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
