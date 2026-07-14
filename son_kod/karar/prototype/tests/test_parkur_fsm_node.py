"""
Girdap İDA — fsm_node parkur katmanı entegrasyon testleri (Sprint 4).

rclpy gerektirir → sistem python3.10 + ROS Humble ile koşar:
    source /opt/ros/humble/setup.bash
    source ros2_ws/install/setup.bash
    python3 -m pytest prototype/tests/test_parkur_fsm_node.py -v

.venv'de (rclpy yok) otomatik SKIP edilir.

Doğrulanan: waypoint_reached sinyali → parkur geçişi, /girdap/parkur/impact →
COMPLETED, /girdap/parkur/state topic yayını. mission_file competition YAML'ı
parameter_overrides ile enjekte edilir (parkur etiketleri [1,1,2,2,3]).
"""

from __future__ import annotations

import textwrap
import time

import pytest

rclpy = pytest.importorskip("rclpy", reason="rclpy yok (.venv) — ROS ortamında koş")

from rclpy.node import Node                              # noqa: E402
from rclpy.parameter import Parameter                    # noqa: E402
from std_msgs.msg import Bool, Int32, String             # noqa: E402

girdap = pytest.importorskip(
    "girdap_decision.fsm_node",
    # F16.2c: en sık gerçek neden mavros_msgs eksikliği (fsm_node:66 import
    # eder) — eski "source'lanmamış" metni operatörü yanlış yöne itiyordu.
    reason="girdap_decision.fsm_node import edilemedi (ros2_ws source'lanmamış "
    "YA DA mavros_msgs kurulu değil: sudo apt install ros-humble-mavros-msgs)",
)

from prototype.mission.parkur_fsm import ParkurState     # noqa: E402


@pytest.fixture(scope="module")
def ros_context():                                       # noqa: ANN201
    rclpy.init()
    yield
    rclpy.shutdown()


@pytest.fixture
def comp_mission(tmp_path_factory) -> str:               # noqa: ANN001
    """Geçici yarışma görev dosyası: parkur etiketleri [1,1,2,2,3]."""
    path = tmp_path_factory.mktemp("mission") / "competition.yaml"
    path.write_text(
        textwrap.dedent(
            """
            waypoints:
              - {lat: 0.0, lon: 0.0, parkur: 1}
              - {lat: 0.0, lon: 0.0, parkur: 1}
              - {lat: 0.0, lon: 0.0, parkur: 2}
              - {lat: 0.0, lon: 0.0, parkur: 2}
              - {lat: 0.0, lon: 0.0, parkur: 3}
            """
        ),
        encoding="utf-8",
    )
    return str(path)


@pytest.fixture
def node(ros_context, comp_mission):                     # noqa: ANN001
    n = girdap.FSMNode(
        parameter_overrides=[
            Parameter("mission_file", Parameter.Type.STRING, comp_mission),
        ]
    )
    yield n
    n.destroy_node()


def _feed_reached(node: Node, index: int, spins: int = 5) -> None:
    """waypoint_reached(index) yayınla, node callback'i işlesin."""
    helper = rclpy.create_node("test_wp_pub")
    pub = helper.create_publisher(Int32, "/girdap/mission/waypoint_reached", 10)
    try:
        deadline = time.monotonic() + 3.0
        while pub.get_subscription_count() < 1:
            rclpy.spin_once(node, timeout_sec=0.05)
            assert time.monotonic() < deadline, "waypoint_reached aboneliği kurulmadı"
        for _ in range(spins):
            pub.publish(Int32(data=index))
            rclpy.spin_once(node, timeout_sec=0.05)
    finally:
        helper.destroy_node()


def _feed_impact(node: Node, spins: int = 5) -> None:
    helper = rclpy.create_node("test_impact_pub")
    pub = helper.create_publisher(Bool, "/girdap/parkur/impact", 10)
    try:
        deadline = time.monotonic() + 3.0
        while pub.get_subscription_count() < 1:
            rclpy.spin_once(node, timeout_sec=0.05)
            assert time.monotonic() < deadline, "impact aboneliği kurulmadı"
        for _ in range(spins):
            pub.publish(Bool(data=True))
            rclpy.spin_once(node, timeout_sec=0.05)
    finally:
        helper.destroy_node()


# ---------------------------------------------------------------- testler

def test_node_wires_parkur_topics(node) -> None:         # noqa: ANN001
    assert node._pub_parkur.topic_name == "/girdap/parkur/state"
    assert node._sub_wp_reached.topic_name == "/girdap/mission/waypoint_reached"
    assert node._sub_impact.topic_name == "/girdap/parkur/impact"


def test_parkur_logic_built_from_mission_file(node) -> None:  # noqa: ANN001
    # competition YAML [1,1,2,2,3] → parkur son index'leri {1:1, 2:3, 3:4}
    assert node._parkur.last_index_of_parkur == {1: 1, 2: 3, 3: 4}


def test_waypoint_reached_drives_transitions(node) -> None:  # noqa: ANN001
    assert node._parkur.state is ParkurState.PARKUR_1
    _feed_reached(node, 1)                        # parkur-1 son wp → PARKUR_2
    assert node._parkur.state is ParkurState.PARKUR_2
    _feed_reached(node, 3)                        # parkur-2 son wp → PARKUR_3
    assert node._parkur.state is ParkurState.PARKUR_3


def test_impact_completes_parkur3(node) -> None:         # noqa: ANN001
    _feed_reached(node, 1)
    _feed_reached(node, 3)
    _feed_impact(node)
    assert node._parkur.state is ParkurState.COMPLETED


def test_parkur_state_published(node) -> None:           # noqa: ANN001
    """/girdap/parkur/state topic'i güncel parkur durumunu yayınlamalı."""
    helper = rclpy.create_node("test_parkur_sub")
    received: list[str] = []
    helper.create_subscription(
        String, "/girdap/parkur/state", lambda m: received.append(m.data), 10
    )
    pub = helper.create_publisher(Int32, "/girdap/mission/waypoint_reached", 10)
    try:
        deadline = time.monotonic() + 5.0
        # Önce PARKUR_1 yayınını gör (timer 10 Hz).
        while not received:
            rclpy.spin_once(node, timeout_sec=0.05)
            rclpy.spin_once(helper, timeout_sec=0.05)
            assert time.monotonic() < deadline, "parkur state yayını gelmedi"
        assert received[-1] == "PARKUR_1"

        # parkur-1 son wp → PARKUR_2 yayınına dönmeli
        while pub.get_subscription_count() < 1:
            rclpy.spin_once(node, timeout_sec=0.05)
            assert time.monotonic() < deadline
        received.clear()
        while "PARKUR_2" not in received:
            pub.publish(Int32(data=1))
            rclpy.spin_once(node, timeout_sec=0.05)
            rclpy.spin_once(helper, timeout_sec=0.05)
            assert time.monotonic() < deadline, "PARKUR_2 yayını gelmedi"
    finally:
        helper.destroy_node()


def test_gercek_imu_darbesi_parkur3u_tamamlar(node) -> None:  # noqa: ANN001
    """F-S.8: gerçek IMU şok yolu parkur katmanını da TAMAMLANDI'ya taşımalı.

    `/girdap/parkur/impact` Sprint-5 placeholder'ı ve onu publish eden HİÇBİR
    node yok — `_on_imu` yalnız MissionFSM gözlemini (`shock_detected_p3`)
    besliyordu, `ParkurTransitionLogic.confirm_impact()` çağrılmıyordu →
    yarışmada /girdap/parkur/state sonsuza dek PARKUR_3'te kalırdı.
    """
    from sensor_msgs.msg import Imu

    _feed_reached(node, 1)
    _feed_reached(node, 3)
    assert node._parkur.state is ParkurState.PARKUR_3

    msg = Imu()
    msg.linear_acceleration.x = 80.0            # ~8.2 g > shock_threshold_g=5
    node._on_imu(msg)
    assert node._parkur.state is ParkurState.COMPLETED, (
        "IMU darbesi parkur katmanına iletilmedi (F-S.8)"
    )


def test_erken_darbe_parkur_katmanini_kirletmez(node) -> None:  # noqa: ANN001
    """F-S.8 bekçisi: PARKUR_1'deki dalga çarpması impact latch'i kurmamalı."""
    from sensor_msgs.msg import Imu

    msg = Imu()
    msg.linear_acceleration.x = 80.0
    node._on_imu(msg)
    assert node._parkur.state is ParkurState.PARKUR_1
    assert node._parkur.impact_confirmed is False
