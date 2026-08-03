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
# F-P.2 (robustness taraması, 2026-07-15): /perception/obstacle_map için
# HİÇ tazelik bekçisi yoktu (F-P.1 yalnız odom'u kapsıyordu). perception_
# lidar_node kaynağı (Livox sürücüsü/USB) donarsa PlanningPipeline son
# bilinen engel listesini SONSUZA DEK kullanmaya devam eder — MPPI artık
# var olmayan bir engelden kaçınmaya çalışabilir ya da (daha kötü) gerçek
# bir engelin oradan gittiğini sanıp üstüne sürebilir. perception_lidar_node
# her LiDAR taramasında (engel olsun olmasın) publish ettiği için topic'in
# kendisi zaten bir heartbeat — tazelik kontrolü güvenle yapılabilir.
# --------------------------------------------------------------------------- #


def _obstacles_msg():                                    # noqa: ANN201
    from geometry_msgs.msg import Pose, PoseArray
    msg = PoseArray()
    p = Pose()
    p.position.x, p.position.y = 3.0, 0.0
    p.orientation.z, p.orientation.w = 1.0, 1.0
    msg.poses.append(p)
    return msg


def test_fp2_bayat_engel_haritasi_isaretlenir(ros_context) -> None:  # noqa: ANN001
    """obstacle_timeout_s'i aşan engel verisiyle MPPI koşulmamalı (thrust sıfır)."""
    node = pn.PlanningNode(
        parameter_overrides=[
            Parameter("obstacle_timeout_s", Parameter.Type.DOUBLE, 1.0)
        ]
    )
    try:
        t = [100.0]
        node._now = lambda: t[0]
        node._on_obstacles(_obstacles_msg())
        assert node._obstacles_stale() is False           # taze

        t[0] = 100.5
        assert node._obstacles_stale() is False            # eşik içinde

        t[0] = 101.5                                       # 1.5 s sessizlik
        assert node._obstacles_stale() is True, (
            "bayat engel haritasıyla MPPI koşmaya devam ediyor (F-P.2)"
        )
    finally:
        node.destroy_node()


def test_fp2_engel_hic_gelmediyse_bayat_degil(ros_context) -> None:  # noqa: ANN001
    """Perception henüz hiç veri göndermediyse 'bayat' alarmı basılmaz
    (boot gürültüsü — F-P.1'deki aynı prensip)."""
    node = pn.PlanningNode()
    try:
        assert node._obstacles_stale() is False
    finally:
        node.destroy_node()


def test_fp2_kapatilabilir(ros_context) -> None:  # noqa: ANN001
    """obstacle_timeout_s=0 → bekçi devre dışı (mock/offline koşular)."""
    node = pn.PlanningNode(
        parameter_overrides=[
            Parameter("obstacle_timeout_s", Parameter.Type.DOUBLE, 0.0)
        ]
    )
    try:
        t = [100.0]
        node._now = lambda: t[0]
        node._on_obstacles(_obstacles_msg())
        t[0] = 999.0
        assert node._obstacles_stale() is False
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


# --------------------------------------------------------------------------- #
# MPPI saha tuning parametreleri (2026-08-02) — yaml/CLI → MPPIConfig yolu.
# Drift kapıları ROS'suz test_planning_config_drift.py'de; bunlar node'un
# parametreyi GERÇEKTEN okuyup boru hattına geçirdiğini doğrular.
# --------------------------------------------------------------------------- #


def test_mppi_tuning_parametreleri_pipeline_e_gecer(ros_context) -> None:  # noqa: ANN001
    """Verilen mppi_* parametreleri MPPIConfig'e ulaşmalı."""
    node = pn.PlanningNode(
        parameter_overrides=[
            Parameter("mppi_lambda", Parameter.Type.DOUBLE, 42.0),
            Parameter("mppi_sigma_u", Parameter.Type.DOUBLE, 8.5),
            Parameter("mppi_obstacle_margin", Parameter.Type.DOUBLE, 1.4),
            Parameter("mppi_terminal_mode", Parameter.Type.STRING, "global"),
            Parameter("mppi_terminal_lookahead_m", Parameter.Type.DOUBLE, 22.0),
            Parameter("mppi_ref_window_size", Parameter.Type.INTEGER, 64),
            Parameter("mppi_ref_window_enabled", Parameter.Type.BOOL, False),
        ]
    )
    try:
        base = node._pipe._base_mppi_cfg
        assert base.lambda_ == 42.0
        assert base.sigma_u == 8.5
        assert base.obstacle_margin == 1.4
        assert base.terminal_mode == "global"
        assert base.terminal_lookahead_m == 22.0
        assert base.ref_window_size == 64
        assert base.ref_window_enabled is False
        # λ override'ı PARKUR PROFİLİNİ de ezmeli
        node._pipe.set_waypoints([(5.0, 5.0), (20.0, 20.0)])
        node._pipe.set_mission_state("PARKUR3")
        assert node._pipe._active_mppi_cfg().lambda_ == 42.0
    finally:
        node.destroy_node()


def test_mppi_lambda_nobetcisi_profili_birakir(ros_context) -> None:  # noqa: ANN001
    """mppi_lambda=0 (varsayılan nöbetçi) → parkur profili kazanır."""
    from prototype.planning.pipeline import _PARKUR_PROFILES

    node = pn.PlanningNode()
    try:
        assert node._pipe.cfg.mppi_lambda is None
        node._pipe.set_waypoints([(5.0, 5.0), (20.0, 20.0)])
        for parkur in ("PARKUR1", "PARKUR2", "PARKUR3"):
            node._pipe.set_mission_state(parkur)
            assert (
                node._pipe._active_mppi_cfg().lambda_
                == _PARKUR_PROFILES[parkur].lambda_
            )
    finally:
        node.destroy_node()


def test_mppi_tuning_varsayilanlari_kod_ile_ayni(ros_context) -> None:  # noqa: ANN001
    """Parametre verilmezse davranış MPPIConfig varsayılanıyla BİREBİR
    (node kendi kopya varsayılanını dayatmasın — config-drift kapısı)."""
    from prototype.planning.mppi import MPPIConfig

    kod = MPPIConfig()
    node = pn.PlanningNode()
    try:
        base = node._pipe._base_mppi_cfg
        for alan in ("sigma_u", "obstacle_margin", "terminal_mode",
                     "terminal_lookahead_m", "ref_window_size",
                     "ref_window_enabled", "lambda_"):
            assert getattr(base, alan) == getattr(kod, alan), alan
    finally:
        node.destroy_node()


def test_mppi_terminal_mode_gecersizse_varsayilana_duser(ros_context) -> None:  # noqa: ANN001
    """Yazım hatası node'u ÖLDÜRMEMELİ (F10.1) — WARN + varsayılana düşüş."""
    from prototype.planning.mppi import MPPIConfig

    node = pn.PlanningNode(
        parameter_overrides=[
            Parameter("mppi_terminal_mode", Parameter.Type.STRING, "lookahed")
        ]
    )
    try:
        assert node._pipe._base_mppi_cfg.terminal_mode == MPPIConfig().terminal_mode
    finally:
        node.destroy_node()
