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

from prototype.mission.gate_follower import ONAY_TICK       # noqa: E402


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


# --------------------------------------------------------------------------- #
# F-P.26 (2026-07-27 yarışma-simülasyonu denetimi): planning_node'un 7 callback'i
# + 2 timer'ı try/except'SİZ'di — perception node'larına uygulanan F-P.3 çökme-
# güvenliği en KRİTİK node'da (thrust hesaplayan) eksikti. Tek bozuk mesaj ya da
# MPPI sayısal çökmesi node'u öldürüp tekneyi SON cmd_vel'le komutsuz
# bırakabilirdi (hiçbir restart supervisor'ı yok). Girdi callback'lerine _guard
# decorator, kontrol timer'ına fail-safe (_safe_stop → sıfır thrust) eklendi.
# --------------------------------------------------------------------------- #


def test_fp26_bozuk_callback_node_oldurmez(ros_context) -> None:  # noqa: ANN001
    """Bir callback'in içi beklenmedik hata fırlatırsa _guard yakalar; exception
    SIZMAZ (spin ölmez, node yaşamaya devam eder)."""
    node = pn.PlanningNode()
    try:
        def _patlat(_state):  # noqa: ANN001, ANN202
            raise ValueError("sahte pipe hatası")
        node._pipe.set_state = _patlat            # callback içi hata simülasyonu
        node._on_odom(_odom())                    # _guard yoksa burada patlardı
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


# --------------------------------------------------------------------------- #
# Kapı takibi entegrasyonu (2026-08-03) + gövde→dünya frame düzeltmesi
# --------------------------------------------------------------------------- #

import math                                             # noqa: E402

from geometry_msgs.msg import PoseArray, Pose           # noqa: E402
from vision_msgs.msg import (                           # noqa: E402
    Detection3D,
    Detection3DArray,
    ObjectHypothesisWithPose,
)


def _odom_poz(x: float, y: float, psi: float) -> Odometry:
    """Verilen ψ ile odom mesajı (z-eksen quaternion; node 2·atan2(z,w) okur)."""
    msg = Odometry()
    msg.pose.pose.position.x = x
    msg.pose.pose.position.y = y
    msg.pose.pose.orientation.z = math.sin(psi / 2.0)
    msg.pose.pose.orientation.w = math.cos(psi / 2.0)
    return msg


def _classified(items) -> Detection3DArray:
    """items: [(x, y, yaricap, class_id)] — GÖVDE çerçevesinde."""
    msg = Detection3DArray()
    msg.header.frame_id = "base_link"
    for x, y, r, cls in items:
        d = Detection3D()
        d.bbox.center.position.x = float(x)
        d.bbox.center.position.y = float(y)
        d.bbox.size.x = float(r) * 2.0
        hyp = ObjectHypothesisWithPose()
        hyp.hypothesis.class_id = str(cls)
        d.results.append(hyp)
        msg.detections.append(d)
    return msg


def test_govde_dunya_donusumu_psi_ile_dondurur(ros_context) -> None:  # noqa: ANN001
    """🔴 2026-08-03 bulgusu: obstacle_map base_link'te (x=ileri) yayınlanıyor
    ama planlama DÜNYA çerçevesinde çalışıyor. Dönüşüm eksikti."""
    node = pn.PlanningNode()
    try:
        # Araç (10, 20)'de, burnu KUZEYE (ψ=90°). Gövdede 5 m "ileri" olan
        # nokta dünyada (10, 25) olmalı — eski kod (10+5, 20+0)=(15,20) derdi.
        node._on_odom(_odom_poz(10.0, 20.0, math.pi / 2.0))
        wx, wy = node._body_to_world(5.0, 0.0)
        assert wx == pytest.approx(10.0, abs=1e-6)
        assert wy == pytest.approx(25.0, abs=1e-6)
    finally:
        node.destroy_node()


def test_obstacle_map_dunya_cercevesine_cevrilir(ros_context) -> None:  # noqa: ANN001
    """`_on_obstacles` artık gövde koordinatını olduğu gibi geçmiyor."""
    node = pn.PlanningNode()
    try:
        node._on_odom(_odom_poz(0.0, 0.0, math.pi / 2.0))   # burun kuzeye
        msg = PoseArray()
        p = Pose()
        p.position.x = 4.0          # gövdede 4 m İLERİ
        p.position.y = 0.0
        p.orientation.z = 0.5        # yarıçap (placeholder şema)
        msg.poses.append(p)
        node._on_obstacles(msg)
        obs = node._pipe._obstacles
        assert len(obs) == 1
        assert obs[0].cx == pytest.approx(0.0, abs=1e-6)
        assert obs[0].cy == pytest.approx(4.0, abs=1e-6)   # kuzeye 4 m
    finally:
        node.destroy_node()


def test_turuncu_kenar_dubasi_engel_torbasindan_cikarilir(ros_context) -> None:  # noqa: ANN001
    """Kapı dubası ENGEL DEĞİLDİR — engel kalırsa MPPI kapıya girmeyi pahalı
    bulur (CLAUDE.md 'Emniyet Payları': margin 1.5 m'de geçitten HİÇ geçmiyor)."""
    node = pn.PlanningNode()
    try:
        node._on_odom(_odom_poz(0.0, 0.0, 0.0))
        node._on_classified(_classified([
            (10.0, +2.0, 0.15, 0),      # turuncu kenar (kapı sol)
            (10.0, -2.0, 0.15, 0),      # turuncu kenar (kapı sağ)
            (12.0, +1.0, 0.20, 1),      # sarı ENGEL
            (14.0, -1.0, 0.20, 99),     # eşleşmeyen (CLASS_UNKNOWN) → engel KALIR
        ]))
        # Yalnız sarı + bilinmeyen engel olmalı; iki turuncu kapıya gitmeli.
        assert len(node._pipe._obstacles) == 2
        assert len(node._edge_buoys) == 2
    finally:
        node.destroy_node()


def test_kapi_ortasi_ham_gorev_noktasini_ezer(ros_context) -> None:  # noqa: ANN001
    """md 5.5.2.2: hakemin noktası kapı ortasında OLMAYABİLİR → araç kapı
    orta noktasına yönelmeli, ham GN'ye değil."""
    node = pn.PlanningNode(
        parameter_overrides=[
            Parameter("use_rrt", Parameter.Type.BOOL, False)   # bypass yolu
        ]
    )
    try:
        node._on_odom(_odom_poz(0.0, 0.0, 0.0))
        # Kapı x=10'da, ortası y=0. Ham GN ise y=+3'te (kapı ortasında DEĞİL).
        dubalar = [(10.0, +2.0, 0.15, 0), (10.0, -2.0, 0.15, 0)]
        target = PoseStamped()
        target.pose.position.x = 20.0
        target.pose.position.y = 3.0
        # B5: kapı, `ONAY_TICK` AYRI ALGI KARESİNDE görülmeden kilitlenmez.
        # Onay boyunca referans ham GN'de kalır (kapısız davranışla birebir).
        for _ in range(ONAY_TICK - 1):
            node._on_classified(_classified(dubalar))    # yeni algı karesi
            node._on_target(target)
            assert node._pipe._ref_path[-1][1] == pytest.approx(3.0, abs=1e-6)
        node._on_classified(_classified(dubalar))
        node._on_target(target)
        # Referansın son noktası kapı ortası (10, 0) olmalı — ham GN (20, 3) değil.
        ref = node._pipe._ref_path
        assert ref is not None
        assert ref[-1][0] == pytest.approx(10.0, abs=1e-6)
        assert ref[-1][1] == pytest.approx(0.0, abs=1e-6)
    finally:
        node.destroy_node()


def test_B5_ayni_algi_karesinde_tekrar_hedef_ONAYI_ILERLETMEZ(ros_context) -> None:  # noqa: ANN001
    """🔑 Kontrol tick'i ≠ algı karesi — B5'in gerçekten çalıştığı yer burası.

    `current_target` 5 Hz akar; algı ise kapalı alanda ~1 Hz'e kadar
    düşebiliyor (§11.3: kümeleme 1-3,3 s/kare ölçüldü). Onay ÇAĞRI başına
    ilerleseydi aynı algı karesi defalarca sayılır ve B5 tam da algının
    zorlandığı — yani yanlış tespitin en olası olduğu — durumda susardı.
    Bu yüzden sayaç `gozlem_no` değişmeden ilerlemez.
    """
    node = pn.PlanningNode(
        parameter_overrides=[Parameter("use_rrt", Parameter.Type.BOOL, False)]
    )
    try:
        node._on_odom(_odom_poz(0.0, 0.0, 0.0))
        node._on_classified(_classified([        # TEK algı karesi
            (10.0, +2.0, 0.15, 0),
            (10.0, -2.0, 0.15, 0),
        ]))
        target = PoseStamped()
        target.pose.position.x = 20.0
        target.pose.position.y = 3.0
        for _ in range(5 * ONAY_TICK):           # ama çok sayıda hedef tick'i
            node._on_target(target)
        assert node._gate.committed_gate is None            # kilitlenmedi
        assert node._pipe._ref_path[-1][1] == pytest.approx(3.0, abs=1e-6)
    finally:
        node.destroy_node()


def test_kapi_yokken_ham_gorev_noktasina_dusulur(ros_context) -> None:  # noqa: ANN001
    """Geriye uyumluluk: kapı görünmüyorsa davranış DEĞİŞMEZ (fallback)."""
    node = pn.PlanningNode(
        parameter_overrides=[
            Parameter("use_rrt", Parameter.Type.BOOL, False)
        ]
    )
    try:
        node._on_odom(_odom_poz(0.0, 0.0, 0.0))
        target = PoseStamped()
        target.pose.position.x = 20.0
        target.pose.position.y = 3.0
        node._on_target(target)                       # hiç kenar dubası yok
        ref = node._pipe._ref_path
        assert ref is not None
        assert ref[-1][0] == pytest.approx(20.0, abs=1e-6)
        assert ref[-1][1] == pytest.approx(3.0, abs=1e-6)
    finally:
        node.destroy_node()


def test_kapi_takibi_kapatilabilir(ros_context) -> None:  # noqa: ANN001
    """gate_following_enabled=false → turuncu duba yine ENGEL, hedef ham GN."""
    node = pn.PlanningNode(
        parameter_overrides=[
            Parameter("use_rrt", Parameter.Type.BOOL, False),
            Parameter("gate_following_enabled", Parameter.Type.BOOL, False),
        ]
    )
    try:
        node._on_odom(_odom_poz(0.0, 0.0, 0.0))
        node._on_classified(_classified([
            (10.0, +2.0, 0.15, 0),
            (10.0, -2.0, 0.15, 0),
        ]))
        assert len(node._pipe._obstacles) == 2        # turuncu ENGEL kaldı
        assert node._edge_buoys == []
        target = PoseStamped()
        target.pose.position.x = 20.0
        target.pose.position.y = 3.0
        node._on_target(target)
        assert node._pipe._ref_path[-1][1] == pytest.approx(3.0, abs=1e-6)
    finally:
        node.destroy_node()


def test_parkur_degisince_kilitli_kapi_birakilir(ros_context) -> None:  # noqa: ANN001
    """Parkur-1'in son kapısına kilitliyken Parkur-2'ye geçilirse eski kapı
    hedefi taşınmamalı (gate_follower.reset sözleşmesi)."""
    from std_msgs.msg import String

    node = pn.PlanningNode()
    try:
        node._on_odom(_odom_poz(0.0, 0.0, 0.0))
        dubalar = [(10.0, +2.0, 0.15, 0), (10.0, -2.0, 0.15, 0)]
        for _ in range(ONAY_TICK):            # B5 onay penceresi (ayrı kareler)
            node._on_classified(_classified(dubalar))
            node._refine_target((20.0, 3.0))
        assert node._gate.committed_gate is not None

        msg = String()
        msg.data = "PARKUR2"
        node._on_mission_state(msg)
        assert node._gate.committed_gate is None
    finally:
        node.destroy_node()


def test_classified_aktiginda_obstacle_map_susar(ros_context) -> None:  # noqa: ANN001
    """İki kaynak aynı engelleri verir; sınıflı olan kazanır, yoksa çift sayım
    (ve kapı dubalarının sınıfsız yoldan engel olarak geri sızması) olurdu."""
    node = pn.PlanningNode()
    try:
        node._on_odom(_odom_poz(0.0, 0.0, 0.0))
        node._on_classified(_classified([(12.0, 1.0, 0.2, 1)]))
        assert len(node._pipe._obstacles) == 1

        msg = PoseArray()                              # sınıfsız yol 3 engel verse de
        for i in range(3):
            p = Pose()
            p.position.x = float(20 + i)
            p.orientation.z = 0.3
            msg.poses.append(p)
        node._on_obstacles(msg)
        assert len(node._pipe._obstacles) == 1         # yok sayıldı
    finally:
        node.destroy_node()


def test_fp26_kontrol_adimi_hatasi_motorlari_durdurur(ros_context) -> None:  # noqa: ANN001
    """_on_control_step içi hata fırlatırsa: exception sızmaz VE motorlar aktif
    DURDURULUR (_safe_stop çağrılır) — son komut kalıcı olmaz."""
    node = pn.PlanningNode()
    try:
        def _patlat():  # noqa: ANN202
            raise RuntimeError("sahte MPPI sayısal çökme")
        node._pipe.compute_control = _patlat
        durduruldu = [False]
        orijinal = node._safe_stop
        def _spy():  # noqa: ANN202
            durduruldu[0] = True
            orijinal()
        node._safe_stop = _spy
        node._on_control_step()                   # exception sızmamalı
        assert durduruldu[0] is True, (
            "kontrol adımı hatasında motorlar durdurulmadı (F-P.26 fail-safe)"
        )
    finally:
        node.destroy_node()
