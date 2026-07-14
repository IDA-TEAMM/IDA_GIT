"""
Girdap İDA — planning_node güvenlik testleri.

F-P.1 (2026-07-14 kod denetimi): fusion_node'un F8.2 bekçisi poz kaynağı
susunca `/girdap/fusion/odom` yayınını KESER ("bayat pozla plan yapılmasın").
Ama planning_node odom'un YAŞINA BAKMIYORDU: `_on_odom` son durumu saklıyor,
`_on_control_step` 10 Hz'te o durumla MPPI koşmaya devam ediyordu → GPS/EKF
kesilse bile araç KÖR sürer (yarışmada çarpma; md 3.3.1.1 istemsiz hareket).
AUTO videosunda MPPI zaten cmd_vel basmaz (mod geçidi) → orada etkisiz;
YARIŞMA (GUIDED+MPPI) için gerçek güvenlik açığı.

rclpy gerektirir → .venv'de SKIP.
"""

from __future__ import annotations

import pytest

rclpy = pytest.importorskip("rclpy", reason="rclpy yok (.venv) — ROS ortamında koş")

from geometry_msgs.msg import PoseStamped               # noqa: E402
from nav_msgs.msg import Odometry, Path                 # noqa: E402
from rclpy.parameter import Parameter                   # noqa: E402

pn = pytest.importorskip(
    "girdap_decision.planning_node",
    reason="girdap_decision source'lanmamış (ros2_ws/install/setup.bash)",
)


@pytest.fixture(scope="module")
def ros_context():                                      # noqa: ANN201
    rclpy.init()
    yield
    rclpy.shutdown()


def _odom(x: float = 5.0) -> Odometry:
    msg = Odometry()
    msg.pose.pose.position.x = x
    msg.pose.pose.orientation.w = 1.0
    return msg


def test_fp1_bayat_odom_bayati_isaretlenir(ros_context) -> None:  # noqa: ANN001
    """odom_timeout_s'i aşan pozla MPPI koşulmamalı (thrust sıfırlanır)."""
    node = pn.PlanningNode(
        parameter_overrides=[
            Parameter("odom_timeout_s", Parameter.Type.DOUBLE, 1.0)
        ]
    )
    try:
        t = [100.0]
        node._now = lambda: t[0]                     # sahte saat
        node._on_odom(_odom())
        assert node._odom_stale() is False            # taze poz

        t[0] = 100.5
        assert node._odom_stale() is False            # eşik içinde

        t[0] = 101.5                                  # 1.5 s sessizlik
        assert node._odom_stale() is True, (
            "bayat pozla MPPI koşmaya devam ediyor (F-P.1)"
        )
    finally:
        node.destroy_node()


def test_fp1_odom_hic_gelmediyse_bayat_degil(ros_context) -> None:  # noqa: ANN001
    """Görev öncesi odom hiç gelmediyse 'bayat' alarmı basılmaz (boot gürültüsü).

    Durum yok → MPPI zaten kontrol üretmez (compute_control None döner);
    burada bayat işaretlemek yanlış alarmdır.
    """
    node = pn.PlanningNode()
    try:
        assert node._odom_stale() is False
    finally:
        node.destroy_node()


def test_fp1_kapatilabilir(ros_context) -> None:  # noqa: ANN001
    """odom_timeout_s=0 → bekçi devre dışı (mock/offline koşular)."""
    node = pn.PlanningNode(
        parameter_overrides=[
            Parameter("odom_timeout_s", Parameter.Type.DOUBLE, 0.0)
        ]
    )
    try:
        t = [100.0]
        node._now = lambda: t[0]
        node._on_odom(_odom())
        t[0] = 999.0
        assert node._odom_stale() is False
    finally:
        node.destroy_node()


# --------------------------------------------------------------------------- #
# F-S.6: /girdap/mission/waypoints hiç publish edilmiyordu — RRT* modu
# (use_rrt=true) global plan hiç oluşturamıyordu, thrust sıfırda kalıyordu.
# mission_manager_node artık current_target'la AYNI referansta (base_link
# göreli ENU) tüm waypoint listesini yayınlıyor; burada son bilinen odom
# xy'sine eklenerek mutlak "map" konumuna çevrilir (_on_target ile aynı desen).
# --------------------------------------------------------------------------- #


def _wp_path(offsets):  # noqa: ANN001, ANN201
    msg = Path()
    msg.header.frame_id = "base_link"
    for east, north in offsets:
        ps = PoseStamped()
        ps.pose.position.x = east
        ps.pose.position.y = north
        ps.pose.orientation.w = 1.0
        msg.poses.append(ps)
    return msg


def test_fs6_on_waypoints_son_xyye_ekler(ros_context) -> None:  # noqa: ANN001
    node = pn.PlanningNode(
        parameter_overrides=[Parameter("use_rrt", Parameter.Type.BOOL, True)]
    )
    try:
        node._on_odom(_odom(x=10.0))              # _last_xy = (10.0, 0.0)
        node._on_waypoints(_wp_path([(5.0, 3.0), (8.0, -2.0)]))
        assert node._pipe._waypoints == [(15.0, 3.0), (18.0, -2.0)], (
            "waypoints son bilinen xy'ye eklenmedi (F-S.6)"
        )
    finally:
        node.destroy_node()


def test_fs6_odom_yoksa_waypoints_yok_sayilir(ros_context) -> None:  # noqa: ANN001
    """Henüz odom gelmediyse (_last_xy None) waypoints işlenmez — crash yok."""
    node = pn.PlanningNode(
        parameter_overrides=[Parameter("use_rrt", Parameter.Type.BOOL, True)]
    )
    try:
        node._on_waypoints(_wp_path([(5.0, 3.0)]))
        assert node._pipe._waypoints == []
    finally:
        node.destroy_node()


def test_fs6_video_bypass_modda_yok_sayilir(ros_context) -> None:  # noqa: ANN001
    """use_rrt=false (video bypass) — waypoints RRT*'a hiç girmez."""
    node = pn.PlanningNode(
        parameter_overrides=[Parameter("use_rrt", Parameter.Type.BOOL, False)]
    )
    try:
        node._on_odom(_odom(x=10.0))
        node._on_waypoints(_wp_path([(5.0, 3.0)]))
        assert node._pipe._waypoints == []
    finally:
        node.destroy_node()
