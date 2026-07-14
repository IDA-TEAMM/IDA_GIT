"""
Girdap İDA — perception_fusion_node entegrasyon testleri (Sprint 3).

rclpy + message_filters gerektirir → sistem python3.10 + ROS Humble ile koşar:
    source /opt/ros/humble/setup.bash
    source ros2_ws/install/setup.bash
    python3 -m pytest prototype/tests/test_perception_fusion_node.py -v

.venv'de (rclpy yok) otomatik SKIP edilir — Layer-0 test koşusunu bozmaz.
"""

from __future__ import annotations

import time

import numpy as np
import pytest

rclpy = pytest.importorskip("rclpy", reason="rclpy yok (.venv) — ROS ortamında koş")
# F16.2a: rclpy var ama vision_msgs kurulmamış olabilir (apt ayrı paket) —
# çıplak import toplama HATASI verir, importorskip dürüst skip'e çevirir.
pytest.importorskip("vision_msgs", reason="vision_msgs yok — ros-humble-vision-msgs kur")

from rclpy.node import Node                              # noqa: E402
from geometry_msgs.msg import Pose, PoseArray             # noqa: E402
from vision_msgs.msg import (                              # noqa: E402
    Detection2D,
    Detection2DArray,
    Detection3DArray,
    ObjectHypothesisWithPose,
)

girdap = pytest.importorskip(
    "girdap_decision.perception_fusion_node",
    reason="girdap_decision source'lanmamış (ros2_ws/install/setup.bash)",
)

from prototype.perception.fusion import CameraDetection, LidarDetection  # noqa: E402
from prototype.perception.synthetic_fusion import scene_fusion_matched  # noqa: E402


@pytest.fixture(scope="module")
def ros_context():                                       # noqa: ANN201
    rclpy.init()
    yield
    rclpy.shutdown()


@pytest.fixture
def node(ros_context) -> "girdap.PerceptionFusionNode":  # noqa: ANN001
    n = girdap.PerceptionFusionNode()
    yield n
    n.destroy_node()


def _pose_array(dets: list[LidarDetection], stamp_sec: int = 0) -> PoseArray:
    msg = PoseArray()
    msg.header.frame_id = "base_link"
    msg.header.stamp.sec = stamp_sec
    msg.header.stamp.nanosec = 123
    for d in dets:
        pose = Pose()
        pose.position.x = d.x
        pose.position.y = d.y
        pose.orientation.z = d.radius          # placeholder hack (Sprint 1)
        pose.orientation.w = 1.0
        msg.poses.append(pose)
    return msg


def _detection2d_array(
    dets: list[CameraDetection],
    image_w: int = 640,
    image_h: int = 480,
    stamp_sec: int = 0,
) -> Detection2DArray:
    """Normalize [0,1] CameraDetection → piksel-uzayı Detection2DArray.

    Üretim node'unun (perception_camera_node) yayınladığı piksel bbox'ı
    taklit eder; fusion node bunu kendi image_w/h parametresiyle normalize
    edip geri CameraDetection'a çevirir (round-trip).
    """
    msg = Detection2DArray()
    msg.header.frame_id = "oak_frame"
    msg.header.stamp.sec = stamp_sec
    msg.header.stamp.nanosec = 123
    for d in dets:
        det = Detection2D()
        det.header = msg.header
        det.bbox.center.position.x = d.bbox_cx * image_w
        det.bbox.center.position.y = d.bbox_cy * image_h
        det.bbox.size_x = 40.0
        det.bbox.size_y = 40.0
        hyp = ObjectHypothesisWithPose()
        hyp.hypothesis.class_id = str(d.class_id)
        hyp.hypothesis.score = d.score
        det.results.append(hyp)
        msg.detections.append(det)
    return msg


def _exchange(
    node: Node,
    pose_msg: PoseArray,
    det_msg: Detection2DArray,
    timeout_s: float = 5.0,
) -> Detection3DArray:
    """İki mesajı (aynı stamp) bas, /perception/classified_obstacles bekle.

    ⚠ Test-only pub/sub AYNI `node` nesnesi üzerinde açılır (ikinci bir
    rclpy.create_node() ile ayrı bir "helper" node KULLANILMAZ). Sebep:
    message_filters.ApproximateTimeSynchronizer + iki ayrı Node nesnesinin
    AYNI Python process'inde bulunması, elle doğrulanmış bir rclpy/DDS
    etkileşiminde giden mesajın karşı node'a hiç ulaşmamasına yol açıyor
    (get_subscription_count() eşleşmeyi doğrulasa bile). Gerçek dağıtımda
    (ayrı `ros2 run` process'leri) bu sorun YOKTUR — yalnız test harness'inin
    "iki node tek process" kısayoluna özgü bir DDS tuhaflığı. Tek node'a
    self-loopback (kendi yayınına kendi abone olması) bu tuzağı by-pass eder.
    """
    received: list[Detection3DArray] = []
    node.create_subscription(
        Detection3DArray, "/perception/classified_obstacles", received.append, 10
    )
    pose_pub = node.create_publisher(PoseArray, "/perception/obstacle_map", 10)
    det_pub = node.create_publisher(Detection2DArray, "/perception/buoys", 10)

    deadline = time.monotonic() + timeout_s
    while (
        pose_pub.get_subscription_count() < 1
        or det_pub.get_subscription_count() < 1
        or node._pub.get_subscription_count() < 1
    ):
        rclpy.spin_once(node, timeout_sec=0.1)
        assert time.monotonic() < deadline, "DDS discovery zamanında tamamlanmadı"

    while not received:
        pose_pub.publish(pose_msg)
        det_pub.publish(det_msg)
        rclpy.spin_once(node, timeout_sec=0.1)
        assert time.monotonic() < deadline, "Detection3DArray zamanında gelmedi"
    return received[0]


# ---------------------------------------------------------------- testler

def test_node_subscribes_both_perception_topics(node) -> None:  # noqa: ANN001
    assert node._lidar_sub.topic == "/perception/obstacle_map"
    assert node._camera_sub.topic == "/perception/buoys"


def test_node_publishes_classified_obstacles(node) -> None:  # noqa: ANN001
    assert node._pub.topic_name == "/perception/classified_obstacles"
    assert node._pub.msg_type is Detection3DArray


def test_synced_messages_produce_expected_fusion(node) -> None:  # noqa: ANN001
    rng = np.random.default_rng(42)
    lidar, camera = scene_fusion_matched(rng)
    result = _exchange(node, _pose_array(lidar), _detection2d_array(camera))
    assert len(result.detections) == 3           # 3 LiDAR tespiti (A,B eşleşti, C unknown)
    class_ids = [d.results[0].hypothesis.class_id for d in result.detections]
    assert class_ids == ["0", "1", "99"]          # A=turuncu, B=sarı, C=unknown
    unknown = result.detections[2]
    assert unknown.results[0].hypothesis.score == pytest.approx(0.0)
    assert unknown.bbox.center.position.x == pytest.approx(lidar[2].x)


def test_classified_obstacles_frame_id_is_base_link(node) -> None:  # noqa: ANN001
    rng = np.random.default_rng(42)
    lidar, camera = scene_fusion_matched(rng)
    result = _exchange(node, _pose_array(lidar), _detection2d_array(camera))
    assert result.header.frame_id == "base_link"


def test_classified_obstacles_stamp_matches_lidar_source(node) -> None:  # noqa: ANN001
    rng = np.random.default_rng(42)
    lidar, camera = scene_fusion_matched(rng)
    result = _exchange(
        node,
        _pose_array(lidar, stamp_sec=555),
        _detection2d_array(camera, stamp_sec=555),
    )
    assert result.header.stamp.sec == 555         # LiDAR referans stamp'i korunur
