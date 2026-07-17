"""
Girdap İDA — perception_lidar_node entegrasyon testleri (Sprint 1).

rclpy gerektirir → sistem python3.10 + ROS Humble ile koşar:
    source /opt/ros/humble/setup.bash
    source ros2_ws/install/setup.bash
    python3 -m pytest prototype/tests/test_perception_lidar_node.py -v

.venv'de (rclpy yok) otomatik SKIP edilir — Layer-0 test koşusunu bozmaz.
"""

from __future__ import annotations

import numpy as np
import pytest

rclpy = pytest.importorskip("rclpy", reason="rclpy yok (.venv) — ROS ortamında koş")
# F16.2b: scipy×numpy ABI kırığı ValueError fırlatır; importorskip yalnız
# ImportError yakalar → çekirdek modülü elle kapıla (dürüst skip, hata değil).
try:
    import scipy.spatial  # noqa: F401
except Exception as exc:  # ImportError VEYA ABI ValueError
    pytest.skip(f"scipy kullanılamıyor: {exc}", allow_module_level=True)

from rclpy.node import Node                              # noqa: E402
from geometry_msgs.msg import PoseArray                  # noqa: E402
from sensor_msgs.msg import PointCloud2, PointField      # noqa: E402
from sensor_msgs_py import point_cloud2                  # noqa: E402
from std_msgs.msg import Header                          # noqa: E402

girdap = pytest.importorskip(
    "girdap_decision.perception_lidar_node",
    reason="girdap_decision.perception_lidar_node import edilemedi "
    "(ros2_ws source'lanmamış ya da bağımlılık eksik)",
)

from prototype.perception.synthetic_lidar import scene_minimum  # noqa: E402


@pytest.fixture(scope="module")
def ros_context():                                       # noqa: ANN201
    rclpy.init()
    yield
    rclpy.shutdown()


@pytest.fixture
def node(ros_context) -> "girdap.PerceptionLidarNode":   # noqa: ANN001
    n = girdap.PerceptionLidarNode()
    yield n
    n.destroy_node()


def _make_cloud(points: np.ndarray, stamp_sec: int = 0) -> PointCloud2:
    header = Header()
    header.frame_id = "livox_frame"
    header.stamp.sec = stamp_sec
    header.stamp.nanosec = 123
    return point_cloud2.create_cloud_xyz32(
        header, points.astype(np.float32).tolist()
    )


def _exchange(node: Node, cloud: PointCloud2, timeout_s: float = 5.0) -> PoseArray:
    """Cloud'u /livox/lidar'a bas, /perception/obstacle_map cevabını bekle."""
    helper = rclpy.create_node("test_helper")
    received: list[PoseArray] = []
    helper.create_subscription(
        PoseArray, "/perception/obstacle_map", received.append, 10
    )
    pub = helper.create_publisher(PointCloud2, "/livox/lidar", 10)
    try:
        deadline = helper.get_clock().now().nanoseconds * 1e-9 + timeout_s
        while not received:
            pub.publish(cloud)
            rclpy.spin_once(node, timeout_sec=0.1)
            rclpy.spin_once(helper, timeout_sec=0.1)
            assert (
                helper.get_clock().now().nanoseconds * 1e-9 < deadline
            ), "PoseArray zamanında gelmedi"
        return received[0]
    finally:
        helper.destroy_node()


def _make_livox_cloud(points: np.ndarray, stamp_sec: int = 0) -> PointCloud2:
    """Gerçek Livox sürücüsünün (livox_ros_driver2, xfer_format=0) alan düzeni.

    KARIŞIK dtype: x/y/z/intensity float32 + tag/line uint8 + timestamp
    float64, point_step=26 — canlı Mid-360'tan okundu (2026-07-12).
    create_cloud_xyz32 bunu TEMSİL ETMEZ (tüm alanlar float32) → F-L.1
    maskeleme deseni; bu üreteç gerçek sürücü şemasını birebir taklit eder.
    """
    header = Header()
    header.frame_id = "livox_frame"
    header.stamp.sec = stamp_sec
    header.stamp.nanosec = 123
    fields = [
        PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        PointField(name="intensity", offset=12,
                   datatype=PointField.FLOAT32, count=1),
        PointField(name="tag", offset=16, datatype=PointField.UINT8, count=1),
        PointField(name="line", offset=17, datatype=PointField.UINT8, count=1),
        PointField(name="timestamp", offset=18,
                   datatype=PointField.FLOAT64, count=1),
    ]
    data = [
        (float(x), float(y), float(z), 100.0, 16, 1, 0.0)
        for x, y, z in points
    ]
    return point_cloud2.create_cloud(header, fields, data)


# ---------------------------------------------------------------- testler

def test_node_subscribes_livox_lidar_topic(node) -> None:   # noqa: ANN001
    assert node._sub.topic_name == "/livox/lidar"


def test_node_publishes_obstacle_map_pose_array(node) -> None:  # noqa: ANN001
    assert node._pub.topic_name == "/perception/obstacle_map"
    assert node._pub.msg_type is PoseArray


def test_synthetic_pointcloud2_produces_expected_poses(node) -> None:  # noqa: ANN001
    rng = np.random.default_rng(42)
    cloud = _make_cloud(scene_minimum(rng))
    result = _exchange(node, cloud)
    assert len(result.poses) == 3                 # scene_minimum → 3 duba
    # orientation.z = yarıçap placeholder'ı duba ölçeğinde olmalı
    for pose in result.poses:
        assert 0.0 < pose.orientation.z < 0.5
        assert pose.orientation.w == pytest.approx(1.0)


def test_real_livox_mixed_dtype_cloud_is_processed(node) -> None:  # noqa: ANN001
    """F-L.1: gerçek Livox bulutu karışık dtype'lı — read_points_numpy'nin
    'tüm alanlar aynı tipte' assert'i node'u İLK gerçek mesajda öldürüyordu
    (canlı Mid-360 ile 2026-07-12'de yakalandı; sentetikler hep xyz32'ydi)."""
    rng = np.random.default_rng(42)
    cloud = _make_livox_cloud(scene_minimum(rng))
    result = _exchange(node, cloud)
    assert len(result.poses) == 3                 # scene_minimum → 3 duba


def test_pose_array_frame_id_is_base_link(node) -> None:   # noqa: ANN001
    rng = np.random.default_rng(42)
    result = _exchange(node, _make_cloud(scene_minimum(rng)))
    assert result.header.frame_id == "base_link"


def test_pose_array_stamp_matches_source(node) -> None:    # noqa: ANN001
    rng = np.random.default_rng(42)
    cloud = _make_cloud(scene_minimum(rng), stamp_sec=1234)
    result = _exchange(node, cloud)
    assert result.header.stamp.sec == 1234        # kaynak damgası korunur
    assert result.header.stamp.nanosec == 123
