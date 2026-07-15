"""
Girdap İDA — perception_camera_node entegrasyon testleri (Sprint 2).

rclpy + cv_bridge gerektirir → sistem python3.10 + ROS Humble ile koşar:
    source /opt/ros/humble/setup.bash
    source ros2_ws/install/setup.bash
    python3 -m pytest prototype/tests/test_perception_camera_node.py -v

.venv'de (rclpy yok) otomatik SKIP edilir — Layer-0 test koşusunu bozmaz.
"""

from __future__ import annotations

import numpy as np
import pytest

rclpy = pytest.importorskip("rclpy", reason="rclpy yok (.venv) — ROS ortamında koş")
# F16.2a: rclpy var ama vision_msgs kurulmamış olabilir (apt ayrı paket) —
# çıplak import toplama HATASI verir, importorskip dürüst skip'e çevirir.
pytest.importorskip("vision_msgs", reason="vision_msgs yok — ros-humble-vision-msgs kur")

from rclpy.node import Node                              # noqa: E402
from sensor_msgs.msg import Image                        # noqa: E402
from vision_msgs.msg import Detection2DArray             # noqa: E402

girdap = pytest.importorskip(
    "girdap_decision.perception_camera_node",
    reason="girdap_decision source'lanmamış (ros2_ws/install/setup.bash)",
)

from girdap_decision.image_codec import bgr_to_imgmsg, imgmsg_to_bgr  # noqa: E402
from prototype.perception.synthetic_camera import scene_camera_minimum  # noqa: E402


@pytest.fixture(scope="module")
def ros_context():                                       # noqa: ANN201
    rclpy.init()
    yield
    rclpy.shutdown()


@pytest.fixture
def node(ros_context) -> "girdap.PerceptionCameraNode":  # noqa: ANN001
    n = girdap.PerceptionCameraNode()
    yield n
    n.destroy_node()


def _make_image(frame: np.ndarray, stamp_sec: int = 0) -> Image:
    msg = bgr_to_imgmsg(frame)                    # cv_bridge'siz (image_codec)
    msg.header.frame_id = "oak_frame"
    msg.header.stamp.sec = stamp_sec
    msg.header.stamp.nanosec = 456
    return msg


def _exchange(
    node: Node, image: Image, timeout_s: float = 5.0
) -> Detection2DArray:
    """Frame'i /oak/rgb/image_raw'a bas, /perception/buoys cevabını bekle."""
    helper = rclpy.create_node("test_camera_helper")
    received: list[Detection2DArray] = []
    helper.create_subscription(
        Detection2DArray, "/perception/buoys", received.append, 10
    )
    pub = helper.create_publisher(Image, "/oak/rgb/image_raw", 10)
    try:
        deadline = helper.get_clock().now().nanoseconds * 1e-9 + timeout_s
        while not received:
            pub.publish(image)
            rclpy.spin_once(node, timeout_sec=0.1)
            rclpy.spin_once(helper, timeout_sec=0.1)
            assert (
                helper.get_clock().now().nanoseconds * 1e-9 < deadline
            ), "Detection2DArray zamanında gelmedi"
        return received[0]
    finally:
        helper.destroy_node()


# ---------------------------------------------------------------- testler

def test_image_codec_roundtrip() -> None:
    """bgr_to_imgmsg → imgmsg_to_bgr birebir geri dönmeli (cv_bridge yerine)."""
    rng = np.random.default_rng(42)
    frame = rng.integers(0, 256, (48, 64, 3), dtype=np.uint8)
    np.testing.assert_array_equal(imgmsg_to_bgr(bgr_to_imgmsg(frame)), frame)


def test_node_subscribes_oak_rgb_topic(node) -> None:    # noqa: ANN001
    assert node._sub.topic_name == "/oak/rgb/image_raw"


def test_node_publishes_buoys_detection_array(node) -> None:  # noqa: ANN001
    assert node._pub.topic_name == "/perception/buoys"
    assert node._pub.msg_type is Detection2DArray


def test_synthetic_frame_produces_expected_detections(node) -> None:  # noqa: ANN001
    rng = np.random.default_rng(42)
    result = _exchange(node, _make_image(scene_camera_minimum(rng)))
    assert len(result.detections) == 3            # 2 turuncu + 1 sarı
    class_ids = sorted(
        d.results[0].hypothesis.class_id for d in result.detections
    )
    assert class_ids == ["0", "0", "1"]
    for det in result.detections:
        assert det.bbox.size_x > 0.0
        assert 0.0 < det.results[0].hypothesis.score <= 1.0


def test_detection_frame_id_preserved(node) -> None:     # noqa: ANN001
    rng = np.random.default_rng(42)
    result = _exchange(node, _make_image(scene_camera_minimum(rng)))
    assert result.header.frame_id == "oak_frame"  # kaynak frame korunur


def test_detection_stamp_matches_source(node) -> None:   # noqa: ANN001
    rng = np.random.default_rng(42)
    image = _make_image(scene_camera_minimum(rng), stamp_sec=987)
    result = _exchange(node, image)
    assert result.header.stamp.sec == 987         # kaynak damgası korunur
    assert result.header.stamp.nanosec == 456


def test_unsupported_encoding_does_not_kill_node(node) -> None:  # noqa: ANN001
    """F-P.6 (robustness taraması, 2026-07-15): imgmsg_to_bgr desteklenmeyen
    bir encoding'de (bgr8/rgb8 dışı) ValueError fırlatır; _on_image'da
    try/except yoktu → tek bozuk kare (format değişimi, sürücü hatası) node'u
    KALICI öldürürdü, buoy/gate tespiti görevin geri kalanı için sessizce
    sıfır kalırdı. Düzeltme: bozuk kare atlanır, node bir SONRAKİ geçerli
    kareye doğru yanıt vermeye devam etmelidir."""
    rng = np.random.default_rng(42)
    bad = _make_image(scene_camera_minimum(rng))
    bad.encoding = "mono8"                        # imgmsg_to_bgr desteklemiyor

    helper = rclpy.create_node("test_camera_malformed_helper")
    received: list[Detection2DArray] = []
    helper.create_subscription(
        Detection2DArray, "/perception/buoys", received.append, 10
    )
    pub = helper.create_publisher(Image, "/oak/rgb/image_raw", 10)
    try:
        for _ in range(5):
            pub.publish(bad)
            rclpy.spin_once(node, timeout_sec=0.1)
            rclpy.spin_once(helper, timeout_sec=0.1)

        good = _make_image(scene_camera_minimum(rng))
        deadline = helper.get_clock().now().nanoseconds * 1e-9 + 5.0
        while not received:
            pub.publish(good)
            rclpy.spin_once(node, timeout_sec=0.1)
            rclpy.spin_once(helper, timeout_sec=0.1)
            assert helper.get_clock().now().nanoseconds * 1e-9 < deadline, (
                "bozuk kareden sonra node geçerli kareye yanıt vermiyor "
                "— muhtemelen çökmüş (F-P.6)"
            )
        assert len(received[0].detections) == 3
    finally:
        helper.destroy_node()
