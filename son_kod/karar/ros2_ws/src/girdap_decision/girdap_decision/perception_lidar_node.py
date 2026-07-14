"""
Girdap İDA — LiDAR engel tespiti node'u (Layer 2, Sprint 1).

/livox/lidar point cloud'unu ROS-bağımsız çekirdekten (prototype.perception.
lidar_obstacles) geçirip planning'in beklediği /perception/obstacle_map
sözleşmesine yayınlar. Kaynak-bağımsız (replaceable design): topic adı sabit,
arkasındaki üretici (gerçek Livox sürücüsü / sentetik / Gazebo) değişebilir.

⚠ PLACEHOLDER mesaj şeması (planning_node._on_obstacles ile birebir):
    PoseArray içindeki her Pose:
        position.x / position.y = cluster centroid (engel merkezi, base_link)
        orientation.z           = çevrel yarıçap (m) — quaternion DEĞİL, hack
        orientation.w           = 1.0
    Gerçek quaternion semantiği yok; girdap_msgs custom mesajı gelene kadar
    bu şema korunur (downstream: planning_node, Sprint 3 fusion).

Subscribed:
    /livox/lidar               sensor_msgs/PointCloud2   (SensorDataQoS)
Published:
    /perception/obstacle_map   geometry_msgs/PoseArray   (default RELIABLE —
                               planning depth-10 default QoS ile tüketiyor)
"""

from __future__ import annotations

from typing import Optional

import numpy as np
from numpy.lib.recfunctions import structured_to_unstructured
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Pose, PoseArray
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2

from girdap_decision.qos_profiles import sensor_data_qos
from prototype.perception.lidar_obstacles import (
    LidarObstacleConfig,
    detect_obstacles,
)


class PerceptionLidarNode(Node):
    """PointCloud2 → filter/cluster → PoseArray daire engel listesi."""

    def __init__(self) -> None:
        super().__init__("perception_lidar_node")

        # --- Parametreler (config/hardware.yaml perception.lidar bloğu) ---
        self.declare_parameter("z_min", 0.1)
        self.declare_parameter("z_max", 3.0)
        self.declare_parameter("cluster_tolerance", 0.5)
        self.declare_parameter("min_cluster_size", 5)
        self.declare_parameter("max_cluster_size", 500)
        self.declare_parameter("split_cell_m", 1.0)  # F5.4: büyük küme bölme
        self.declare_parameter("max_range", 25.0)
        self.declare_parameter("voxel_size", 0.1)   # F5.3; 0 = kapalı
        self.declare_parameter("log_period_s", 5.0)

        p = self.get_parameter
        self._cfg = LidarObstacleConfig(
            z_min=float(p("z_min").value),
            z_max=float(p("z_max").value),
            cluster_tolerance=float(p("cluster_tolerance").value),
            min_cluster_size=int(p("min_cluster_size").value),
            max_cluster_size=int(p("max_cluster_size").value),
            split_cell_m=float(p("split_cell_m").value),
            max_range=float(p("max_range").value),
            voxel_size=float(p("voxel_size").value),
        )
        self._log_period_s = float(p("log_period_s").value)
        self._last_log_t: Optional[float] = None

        # --- I/O ---
        self._pub = self.create_publisher(
            PoseArray, "/perception/obstacle_map", 10
        )
        # F7.3: depth=1 — kümeleme (F5.3) 10 Hz'e yetişemezse kuyrukta bayat
        # taramalar birikmesin; her callback ELDEKİ EN YENİ taramayı işlesin.
        # depth=10 ile ~1 s gecikmiş bulutla plan yapılıyordu.
        self._sub = self.create_subscription(
            PointCloud2, "/livox/lidar", self._on_cloud, sensor_data_qos(depth=1)
        )

        self.get_logger().info(
            "perception_lidar_node aktif: /livox/lidar → "
            "/perception/obstacle_map "
            f"(z=[{self._cfg.z_min},{self._cfg.z_max}] m, "
            f"tol={self._cfg.cluster_tolerance} m, "
            f"size=[{self._cfg.min_cluster_size},{self._cfg.max_cluster_size}], "
            f"menzil={self._cfg.max_range} m)"
        )

    # ------------------------------------------------------------- callback

    def _on_cloud(self, msg: PointCloud2) -> None:
        # F-L.1: read_points_numpy KULLANMA — gerçek Livox bulutu karışık
        # dtype'lı (x/y/z/intensity float32 + tag/line uint8 + timestamp
        # float64) ve read_points_numpy, field_names'ten BAĞIMSIZ olarak TÜM
        # alanların aynı tipte olmasını assert eder → ilk gerçek mesajda
        # AssertionError. Yapılandırılmış okuma + seçili alanları düz diziye
        # çevirme aynı işi güvenle yapar.
        structured = point_cloud2.read_points(
            msg, field_names=("x", "y", "z"), skip_nans=True
        )
        points = structured_to_unstructured(structured).reshape(-1, 3)
        obstacles = detect_obstacles(
            np.asarray(points, dtype=np.float64), self._cfg
        )
        self._pub.publish(self._to_pose_array(obstacles, msg))

        self.get_logger().debug(
            f"{len(points)} nokta → {len(obstacles)} engel"
        )
        self._periodic_info(len(obstacles))

    def _to_pose_array(self, obstacles: list, msg: PointCloud2) -> PoseArray:
        """CircleObstacle listesi → placeholder PoseArray (docstring'e bak)."""
        out = PoseArray()
        out.header.stamp = msg.header.stamp          # kaynak damgasını koru
        out.header.frame_id = "base_link"            # LiDAR → base_link static TF
        for obs in obstacles:
            pose = Pose()
            pose.position.x = obs.center_x
            pose.position.y = obs.center_y
            pose.position.z = 0.0
            pose.orientation.z = obs.radius          # ⚠ yarıçap hack'i
            pose.orientation.w = 1.0
            out.poses.append(pose)
        return out

    def _periodic_info(self, n_obstacles: int) -> None:
        """log_period_s'de bir INFO — her callback'te log seli olmasın."""
        now = self.get_clock().now().nanoseconds * 1e-9
        if self._last_log_t is None or now - self._last_log_t >= self._log_period_s:
            self._last_log_t = now
            self.get_logger().info(f"tespit: {n_obstacles} engel")


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node = PerceptionLidarNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
