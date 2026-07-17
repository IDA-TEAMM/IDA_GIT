"""
Girdap İDA — fusion_node bypass modu testi (F8.1).

Doğrular (video zinciri, use_isam2=false → gtsam GEREKMEZ):
    - /mavros/local_position/pose → /girdap/fusion/odom pose geçişi
    - F8.1: /mavros/local_position/velocity_body → odom.twist doldurulur
      (planning_node MPPI durum vektörüne u,v,r'yi buradan okur)

rclpy gerektirir → sistem python3.10 + ROS Humble; .venv'de SKIP.
"""

from __future__ import annotations

import time

import pytest

rclpy = pytest.importorskip("rclpy", reason="rclpy yok (.venv) — ROS ortamında koş")

from rclpy.parameter import Parameter                    # noqa: E402
from geometry_msgs.msg import PoseStamped, TwistStamped  # noqa: E402
from nav_msgs.msg import Odometry                        # noqa: E402

girdap = pytest.importorskip(
    "girdap_decision.fusion_node",
    reason="girdap_decision source'lanmamış (ros2_ws/install/setup.bash)",
)


@pytest.fixture(scope="module")
def ros_context():                                       # noqa: ANN201
    rclpy.init()
    yield
    rclpy.shutdown()


def test_bypass_odom_carries_pose_and_twist(ros_context) -> None:
    """EKF poz + body hız yayınla → odom'da pose VE twist dolu olmalı (F8.1)."""
    node = girdap.FusionNode(
        parameter_overrides=[
            Parameter("use_isam2", Parameter.Type.BOOL, False),
        ]
    )
    helper = rclpy.create_node("test_fusion_helper")
    pose_pub = helper.create_publisher(
        PoseStamped, "/mavros/local_position/pose", 10
    )
    vel_pub = helper.create_publisher(
        TwistStamped, "/mavros/local_position/velocity_body", 10
    )
    odoms: list[Odometry] = []
    helper.create_subscription(
        Odometry, "/girdap/fusion/odom", odoms.append, 10
    )
    try:
        pose = PoseStamped()
        pose.pose.position.x = 3.0
        pose.pose.position.y = -1.5
        pose.pose.orientation.w = 1.0
        vel = TwistStamped()
        vel.twist.linear.x = 1.2                 # ileri sürat (body u)
        vel.twist.linear.y = -0.1                # yanal (body v)
        vel.twist.angular.z = 0.25               # yaw rate (r)

        deadline = time.monotonic() + 5.0
        good: Odometry | None = None
        while time.monotonic() < deadline and good is None:
            pose_pub.publish(pose)
            vel_pub.publish(vel)
            rclpy.spin_once(helper, timeout_sec=0.01)
            # 50 Hz timer spin_once'ı doyurabilir — birkaç kez spin et
            for _ in range(6):
                rclpy.spin_once(node, timeout_sec=0.01)
            good = next(
                (o for o in odoms if o.twist.twist.linear.x != 0.0), None
            )
        assert good is not None, "twist'i dolu odom mesajı gelmedi (F8.1)"
        assert good.pose.pose.position.x == pytest.approx(3.0)
        assert good.pose.pose.position.y == pytest.approx(-1.5)
        assert good.twist.twist.linear.x == pytest.approx(1.2)
        assert good.twist.twist.linear.y == pytest.approx(-0.1)
        assert good.twist.twist.angular.z == pytest.approx(0.25)
        assert good.child_frame_id == "base_link"  # twist body-frame sözleşmesi
    finally:
        helper.destroy_node()
        node.destroy_node()


def test_bypass_stale_pose_stops_publishing(ros_context) -> None:
    """F8.2: EKF poz akışı kesilince fusion odom yayını DURMALI (bayat pozla
    50 Hz yayına devam etmek downstream'i donmuş pozla plan yapmaya iter).

    Ölçüm node'un GERÇEK ``publish`` çağrılarını sayar; ikinci bir abonenin
    aldığı mesaj sayısını DEĞİL. Sebep: RELIABLE DDS teslimi eşikten önce
    yayınlanmış son mesajları herhangi bir sabit/dingin pencereyi aşacak kadar
    geciktirebilir → alıcı-sayımı doğası gereği yarışlı. Yayın kararı ise timer
    callback'iyle EŞZAMANLI; publish sayacı gecikmesiz, deterministik.
    """
    node = girdap.FusionNode(
        parameter_overrides=[
            Parameter("use_isam2", Parameter.Type.BOOL, False),
            Parameter("pose_timeout_s", Parameter.Type.DOUBLE, 0.3),
        ]
    )
    helper = rclpy.create_node("test_fusion_stale_helper")
    pose_pub = helper.create_publisher(
        PoseStamped, "/mavros/local_position/pose", 10
    )

    # Node'un odom yayınlarını kaynağında say (DDS teslim gecikmesi devre dışı).
    n_pub = 0
    _orig_publish = node._pub_odom.publish

    def _counting_publish(msg: Odometry) -> None:
        nonlocal n_pub
        n_pub += 1
        _orig_publish(msg)

    node._pub_odom.publish = _counting_publish  # type: ignore[method-assign]

    try:
        pose = PoseStamped()
        pose.pose.orientation.w = 1.0
        # Canlı akışta node odom yayınlamalı
        deadline = time.monotonic() + 5.0
        while time.monotonic() < deadline and n_pub == 0:
            pose_pub.publish(pose)
            rclpy.spin_once(helper, timeout_sec=0.01)
            for _ in range(6):
                rclpy.spin_once(node, timeout_sec=0.01)
        assert n_pub > 0, "canlı akışta node odom yayınlamadı"

        # Akışı KES; bayatlık eşiğini (0.3 s) rahat marjla aşacak kadar spin et.
        t_stop = time.monotonic()
        while time.monotonic() - t_stop < 0.6:
            rclpy.spin_once(node, timeout_sec=0.02)
        assert node._stale_warned, "bayatlık guard'ı tetiklenmedi (F8.2)"
        n_after_stale = n_pub

        # Bayat durumda EK spin'de node ARTIK yayınlamamalı. (Node hatalı olsa
        # 50 Hz'de sayaç burada büyürdü — deterministik regresyon kapanı.)
        end = time.monotonic() + 0.6
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.02)
        assert n_pub == n_after_stale, "bayat pozla odom yayını sürüyor (F8.2)"
    finally:
        node._pub_odom.publish = _orig_publish  # type: ignore[method-assign]
        helper.destroy_node()
        node.destroy_node()
