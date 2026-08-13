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

import numpy as np
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


def test_KAR03_odom_HIC_gelmediyse_BAYAT_sayilir(ros_context) -> None:  # noqa: ANN001
    """🔴 BEKLENTİ 12.08'de TERSİNE ÇEVRİLDİ (KAR-03) — bilerek.

    Bu test eskiden `is False` bekliyordu; gerekçesi *"durum yok → MPPI zaten
    kontrol üretmez"* idi. Gerekçe YANLIŞTI: `PlanningPipeline.__init__`
    durumu `np.zeros(6)` ile kurar, yani "durum yok" diye bir hal yoktur —
    poz hiç gelmemişken bile tam geçerli görünen bir (0,0,0) pozu vardır.
    FSM aktif duruma geçtiği an MPPI o uydurma orijinden GERÇEK thrust üretir
    ve tam o anda bekçi susardı.

    Kaptanın bag analizinde bu yol kuramsal değil: KAR-01'de `ARM ↔ PARKUR2`
    salınımı odometri (0,0,0) iken yaşandı.

    Yanlış alarm endişesi geçerliydi ama çözümü bekçiyi kapatmak DEĞİL,
    uyarıyı ayırmaktı (`_warn_stale_odom` "hiç gelmedi" kolunu 5 s'de bir ve
    farklı metinle basar).
    """
    node = pn.PlanningNode()
    try:
        assert node._odom_stale() is True, (
            "poz hic gelmemisken bekci susarsa MPPI uydurma orijinden surer"
        )
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


def test_KAR03_engel_HIC_gelmediyse_BAYAT_sayilir(ros_context) -> None:  # noqa: ANN001
    """🔴 BEKLENTİ 12.08'de TERSİNE ÇEVRİLDİ (KAR-03) — odomdakiyle aynı gerekçe.

    Burada sonuç daha ağır: engel haritası hiç gelmediyse MPPI'nin engel
    torbası BOŞTUR, yani maliyet fonksiyonu için "önüm tamamen açık" demektir.
    Algı çöktükten SONRAKİ 2 saniye korunuyordu ama algının HİÇ açılmamış
    olması korunmuyordu — yani en tehlikeli hâl bekçinin dışındaydı.
    """
    node = pn.PlanningNode()
    try:
        assert node._obstacles_stale() is True, (
            "engel haritasi hic gelmemisken 'onum acik' varsayilamaz"
        )
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


def test_turuncu_kenar_dubasi_HEM_kapiya_HEM_huniyle_torbaya_gider(ros_context) -> None:  # noqa: ANN001
    """Kapı direği kenar OLARAK KALIR **ve** engel torbasına HUNİ PAYIYLA girer.

    ⚠️ **2026-08-10: bu testin eski hâli bayattı ve CI'yı kırmızı tutuyordu.**
    Adı `..._engel_torbasindan_cikarilir` idi ve `len(_obstacles) == 2` bekliyordu
    — yani direklerin torbadan TAMAMEN çıkarıldığı davranışı donduruyordu. O
    davranış **B2 HUNİ ile (§0.18d, 09.08) bilerek değiştirildi**: direkleri
    torbadan çıkarmanın bedeli ölçülmüştü — dubalardan iten hiçbir kuvvet
    kalmıyor, gövde payı **−0,23 m** (temas) çıkıyordu. Şimdi direkler torbada
    kalıyor ama payları küresel `obstacle_margin` değil, ölçülen açıklıktan
    türeyen `_huni_payi`. Kod doğruydu, testi güncellenmemişti.

    Korunan asıl sözleşme değişmedi ve burada da doğrulanıyor: kapı direği
    `_edge_buoys`'a gider (kapı takibi onu görür) ve payı küresel değerden
    KÜÇÜKTÜR (yoksa 1,0 m'lik halka dar geçidin içini kaplar).
    """
    node = pn.PlanningNode()
    try:
        node._on_odom(_odom_poz(0.0, 0.0, 0.0))
        # 🔴 F-A.1 (13.08.2026): tek kare turuncu ONAY DEĞİL — kenar dubası
        # olmak için aynı konum ≥2 karede turuncu görülmeli (ölçüm: kameranın
        # yanlış pozitifleri tek kare parlamasıydı, 0 tekrar). İkinci kare
        # eklendi; testin korduğu güvence aynen duruyor.
        for _ in range(2):
            node._on_classified(_classified([
                (10.0, +2.0, 0.15, 0),      # turuncu kenar (kapı sol)
                (10.0, -2.0, 0.15, 0),      # turuncu kenar (kapı sağ)
                (12.0, +1.0, 0.20, 1),      # sarı ENGEL
                (14.0, -1.0, 0.20, 99),     # eşleşmeyen (CLASS_UNKNOWN) → engel KALIR
            ]))
        # İki turuncu kapı takibine gider…
        assert len(node._edge_buoys) == 2
        # …ve torbada 2 normal engel + 2 huni paylı direk bulunur.
        normal = [o for o in node._pipe._obstacles if o.margin is None]
        direkler = [o for o in node._pipe._obstacles if o.margin is not None]
        assert len(normal) == 2, "sarı + UNKNOWN engel kalmalı"
        assert len(direkler) == 2, "kapı direkleri huni payıyla torbada olmalı"
        # Huni payı ölçülen açıklıktan türer: 4 m kapıda tavana dayanır.
        assert all(0.0 < o.margin <= node._gate_post_margin for o in direkler)
    finally:
        node.destroy_node()


def test_hatirlanan_cisim_MENZIL_DISINDA_engel_torbasina_KONMAZ(ros_context) -> None:  # noqa: ANN001
    """🔴 YAYIM MENZİLİ nöbetçisi (2026-08-10, §0.26b-c).

    Kalıcı harita hiçbir kaydı silmiyor (kaptan kararı 09.08). Ölçüldü ki konum
    sıçraması çakışma bandının üçte birini geçince aynı duba için ikinci kayıt
    açılıyor ve torba sınırsız büyüyor; bedeli `_huni_payi`'nin O(n²) taraması
    ile MPPI'nin (K,T+1,N) engel tensöründe ödeniyor. Çözüm silme değil, engel
    torbasına yalnız yerel harita penceresi içindekileri koymak.

    Bu test iki yönü birden donduruyor: uzaktaki kayıt torbaya GİRMEZ, araç
    yaklaşınca GERİ GELİR (yani unutulmamıştır).
    """
    node = pn.PlanningNode()
    try:
        yaricap = node._harita_yaricapi
        assert yaricap > 0.0
        uzak = yaricap + 20.0                  # 45 m
        # ⚠ `_classified` GÖVDE çerçevesinde verir; node `_body_to_world` ile
        # çevirir. Aşağıdaki ofsetler bu yüzden araca GÖRELİ.
        # 1) Araç (uzak, 0)'da; 5 m ilerideki cismi GÖR → dünya (uzak+5, 0).
        node._on_odom(_odom_poz(uzak, 0.0, 0.0))
        node._on_classified(_classified([(5.0, 0.0, 0.15, 1)]))
        assert node._edge_memory.boyut == 1
        # 2) Başlangıca dön, BAŞKA bir cisim gör → eski kayıt menzil DIŞI.
        node._on_odom(_odom_poz(0.0, 0.0, 0.0))
        node._on_classified(_classified([(5.0, 0.0, 0.20, 1)]))
        assert node._edge_memory.boyut == 2, "kayıt SİLİNMEMELİ"
        assert node._edge_memory.son_menzil_disi == 1
        assert all(abs(o.cx) < yaricap for o in node._pipe._obstacles), \
            "menzil dışı kayıt engel torbasına konmamalı"
        # 3) Geri yaklaş → hatırlanan cisim torbaya GERİ GELİR (unutulmamış).
        node._on_odom(_odom_poz(uzak, 0.0, 0.0))
        node._on_classified(_classified([(30.0, 0.0, 0.20, 1)]))
        assert node._edge_memory.son_menzil_disi == 1, "başlangıçtaki cisim geride kaldı"
        assert any(abs(o.cx - (uzak + 5.0)) < 0.5 for o in node._pipe._obstacles), \
            "araç yaklaşınca hatırlanan cisim geri gelmeli"
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
        # F-A.1: ilk turuncu kare KENAR ONAYI için harcanır (tek kare
        # kenar dubası yapmaz) → kapı onay penceresi bir kare kayar.
        node._on_classified(_classified(dubalar))
        # B5: kapı, `ONAY_TICK` AYRI ALGI KARESİNDE görülmeden kilitlenmez.
        # Onay boyunca referans ham GN'de kalır (kapısız davranışla birebir).
        for _ in range(ONAY_TICK - 1):
            node._on_classified(_classified(dubalar))    # yeni algı karesi
            node._on_target(target)
            assert node._pipe._ref_path[-1][1] == pytest.approx(3.0, abs=1e-6)
        node._on_classified(_classified(dubalar))
        node._on_target(target)
        # Referansın son noktası kapının ÖTESİ olmalı — ham GN (20, 3) değil.
        # 🔴 F-K.1 (13.08.2026) — SÖZLEŞME İNCELDİ: eskiden kapı ORTASI (10, 0)
        # bekleniyordu; artık orta + gövde boyu (10 + 1,04 = 11,04). Sebep
        # sanal gölde kapalı döngüde ÖLÇÜLDÜ: nişan kapı düzleminin ÜSTÜNDE
        # bırakılınca MPPI referansı orada bitiyor, `_terminal_goal` referans
        # sonuna kırpıyor ve araç TAM KAPI ORTASINDA duruyor (ölçüm: konum
        # (0.02, 24.95), thrust 0,13 N). Düzlem geçilmediği için kilit de
        # çözülmüyor → görev bir daha ilerlemiyor. Kapı bir VARIŞ değil,
        # GEÇİLECEK EŞİKTİR. Uzatma = ölçülmüş gövde boyu; gerekçe yarışma
        # tanımı: geçiş *pruva* girince başlar, ***kıç* çıkınca* biter.
        # Korunan asıl güvence aynen duruyor: kapı ham GN'yi EZİYOR.
        # F-K.1: hedef kapı ortası DEĞİL, ötesi (10 + gövde boyu 1,04).
        # (F-K.2'nin hizalanma fazı ölçümle elendi — sürüş yolunda değil.)
        ref = node._pipe._ref_path
        assert ref is not None
        assert ref[-1][0] == pytest.approx(10.0 + 1.04, abs=1e-6)
        assert ref[-1][1] == pytest.approx(0.0, abs=1e-6)
        assert ref[-1][0] > 0.0, "hedef aracin gerisinde"
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
        # F-A.1: kenar onayı için 2 kare (B5'in ölçtüğü şey KAPI onayı,
        # kenar onayı değil — kapı hâlâ kilitlenmemeli).
        for _ in range(2):
            node._on_classified(_classified([
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
        # F-A.1: ilk turuncu kare KENAR ONAYI için harcanır (tek kare
        # kenar dubası yapmaz) → kapı onay penceresi bir kare kayar.
        node._on_classified(_classified(dubalar))
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


# --------------------------------------------------------------------------- #
# cmd_vel ÖLÇEĞİ (2026-08-06, GIRDAP_DURUM §0.7d) — kapalı döngüde ÖLÇÜLDÜ.
# Eski formül (u₀+u₁)/(2·m) kuvvet/kütle = İVME veriyordu ve teknenin fiilen
# yaptığı hızın 0,10×'ini komut ediyordu. Doğrusu denge hızı (u₀+u₁)/|Xu|.
# Aşağıdaki iki test o düzeltmeyi DONDURUR — eski formüle dönülürse kırmızı.
# --------------------------------------------------------------------------- #


def test_cmd_vel_tam_itkide_modelin_DENGE_hizini_komut_eder(ros_context) -> None:  # noqa: ANN001
    """Tam itkide linear.x = 2·max_thrust/|Xu| (modelin terminal hızı)."""
    import numpy as np
    from geometry_msgs.msg import Twist

    node = pn.PlanningNode()
    try:
        p = node._pipe._dyn.p
        yakalanan: list[Twist] = []
        node._pub_cmd_vel.publish = yakalanan.append   # type: ignore[assignment]

        node._publish_cmd_vel(np.array([p.max_thrust, p.max_thrust]))

        beklenen = 2.0 * p.max_thrust / abs(p.Xu)
        assert yakalanan[-1].linear.x == pytest.approx(beklenen, rel=1e-6)
        # Eski formül (2·m paydası) burada 0,12 m/s verirdi — 10× düşük.
        assert yakalanan[-1].linear.x > 4.0 * (
            2.0 * p.max_thrust / (2.0 * p.mass)
        )
    finally:
        node.destroy_node()


def test_cmd_vel_tavani_OLCULEN_tam_gaz_hiziyla_uyumlu(ros_context) -> None:  # noqa: ANN001
    """Komut tavanı, log 58'de ölçülen 1,26 m/s ile aynı büyüklük sırasında.

    Bu bir tuning eşiği değil TUTARLILIK kapısı: cmd_vel tavanı ile teknenin
    fiziksel tavanı arasında 2×'ten fazla açıklık varsa ya dinamik modeli ya
    formülü bozmuşuzdur (ikisinden biri kesinlikle yanlıştır).
    """
    import numpy as np
    from geometry_msgs.msg import Twist

    OLCULEN_TAM_GAZ = 1.26        # m/s — log 58 (GIRDAP_DURUM §7)

    node = pn.PlanningNode()
    try:
        p = node._pipe._dyn.p
        yakalanan: list[Twist] = []
        node._pub_cmd_vel.publish = yakalanan.append   # type: ignore[assignment]
        node._publish_cmd_vel(np.array([p.max_thrust, p.max_thrust]))
        tavan = yakalanan[-1].linear.x
        assert 0.5 * OLCULEN_TAM_GAZ <= tavan <= 2.0 * OLCULEN_TAM_GAZ, (
            f"cmd_vel tavanı {tavan:.3f} m/s, ölçülen tam gaz "
            f"{OLCULEN_TAM_GAZ} m/s ile uyumsuz"
        )
    finally:
        node.destroy_node()


def test_cmd_vel_angular_z_DENGE_yaw_hizini_komut_eder(ros_context) -> None:  # noqa: ANN001
    """angular.z = (u₁−u₀)·(B/2)/|Nr| — linear.x'teki denge mantığının ikizi.

    🔴 2026-08-06 (§0.9e): eski formül `(u₁−u₀)/inertia_z` iki yerden yanlıştı:
    tork/atalet = açısal İVME (boyut) ve moment kolu B/2 EKSİK. Kapalı döngüde
    ölçüldü: eski formül fiili yaw hızının **2,01×**'ini komut ediyordu; denge
    formülü 1,00×. Bu test o düzeltmeyi dondurur.
    """
    import numpy as np
    from geometry_msgs.msg import Twist

    node = pn.PlanningNode()
    try:
        p = node._pipe._dyn.p
        yakalanan: list[Twist] = []
        node._pub_cmd_vel.publish = yakalanan.append   # type: ignore[assignment]

        # Saf dönüş: sol geri, sağ ileri (net ileri itki sıfır).
        node._publish_cmd_vel(np.array([-p.max_thrust, p.max_thrust]))

        du = 2.0 * p.max_thrust
        beklenen = du * (p.thruster_spacing / 2.0) / abs(p.Nr)
        assert yakalanan[-1].angular.z == pytest.approx(beklenen, rel=1e-6)
        # Saf dönüşte ileri hız komutu sıfır olmalı (itkiler birbirini götürür).
        assert yakalanan[-1].linear.x == pytest.approx(0.0, abs=1e-9)
        # Eski formül (du/inertia_z) burada 2× büyük çıkardı — geri dönülürse kırmızı.
        eski = du / p.inertia_z
        assert abs(eski - beklenen) > 0.3 * beklenen, (
            "test kurulumu bozuk: eski ve yeni formül ayırt edilemiyor"
        )
        assert yakalanan[-1].angular.z < 0.75 * eski
    finally:
        node.destroy_node()


def test_cmd_vel_iki_ekseni_de_AYNI_denge_mantigini_kullanir(ros_context) -> None:  # noqa: ANN001
    """Tasarım kuralı: her iki eksen de "itki = sönümleme" dengesinden türer.

    Sayıları değil İLİŞKİYİ dondurur — dinamik yeniden tanılanırsa (Xu, Nr)
    komutlar onunla birlikte taşınmak zorunda; biri elle sabitlenirse kırmızı.
    """
    import numpy as np
    from geometry_msgs.msg import Twist

    node = pn.PlanningNode()
    try:
        p = node._pipe._dyn.p
        yakalanan: list[Twist] = []
        node._pub_cmd_vel.publish = yakalanan.append   # type: ignore[assignment]

        node._publish_cmd_vel(np.array([0.3 * p.max_thrust, 0.9 * p.max_thrust]))
        tw = yakalanan[-1]
        toplam = 1.2 * p.max_thrust
        fark = 0.6 * p.max_thrust
        assert tw.linear.x == pytest.approx(toplam / abs(p.Xu), rel=1e-6)
        assert tw.angular.z == pytest.approx(
            fark * (p.thruster_spacing / 2.0) / abs(p.Nr), rel=1e-6
        )
    finally:
        node.destroy_node()


# --------------------------------------------------------------------------- #
# B3 — GEÇİŞ SAYACI teşhis kanalı (2026-08-06). Çekirdek sayıyordu ama sayı
# hiçbir yere çıkmıyordu: ne operatör görüyordu ne de md 5.5.2.4'ün "en az iki
# duba ikilisinden geçiş" şartı için kanıt üretiliyordu.
# 🔴 KASITLI OLARAK FSM'E BAĞLI DEĞİL — aşağıdaki son test onu donduruyor.
# --------------------------------------------------------------------------- #


def _kapidan_gecir(node, kapi_x: float = 10.0) -> None:  # noqa: ANN001
    """Aracı kapıya kilitleyip düzlemini geçir (ONAY_TICK + geçiş)."""
    dubalar = [(kapi_x, +2.0, 0.15, 0), (kapi_x, -2.0, 0.15, 0)]
    target = PoseStamped()
    target.pose.position.x = kapi_x + 10.0
    target.pose.position.y = 0.0
    # F-A.1: ilk kare kenar onayına gider → pencere bir kare uzun.
    for _ in range(ONAY_TICK + 1):
        node._on_odom(_odom_poz(0.0, 0.0, 0.0))
        node._on_classified(_classified(dubalar))
        node._on_target(target)
    # Kapının ötesine geç: kilitli kapı bırakılır ve GEÇİŞ sayılır.
    node._on_odom(_odom_poz(kapi_x + 1.0, 0.0, 0.0))
    node._on_classified(_classified([]))          # kapı artık görünmüyor
    node._on_target(target)


def test_B3_gecis_sayaci_teshis_kanalina_yayinlaniyor(ros_context) -> None:  # noqa: ANN001
    """Kapı geçilince /girdap/planning/gate_count artmalı (md 5.5.2.4 kanıtı)."""
    from std_msgs.msg import Int32

    node = pn.PlanningNode(
        parameter_overrides=[Parameter("use_rrt", Parameter.Type.BOOL, False)]
    )
    try:
        yayin: list = []
        node._pub_gate_count.publish = yayin.append   # type: ignore[assignment]
        assert node._gate.passed_gate_count == 0

        _kapidan_gecir(node, kapi_x=10.0)

        assert node._gate.passed_gate_count == 1
        assert [m.data for m in yayin if isinstance(m, Int32)][-1] == 1
    finally:
        node.destroy_node()


def test_B3_sayac_yalniz_DEGISINCE_yayinlanir(ros_context) -> None:  # noqa: ANN001
    """20 Hz'te sabit sayıyı tekrar tekrar basmak telemetriyi kirletir."""
    node = pn.PlanningNode(
        parameter_overrides=[Parameter("use_rrt", Parameter.Type.BOOL, False)]
    )
    try:
        yayin: list = []
        node._pub_gate_count.publish = yayin.append   # type: ignore[assignment]
        _kapidan_gecir(node, kapi_x=10.0)
        onceki = len(yayin)

        target = PoseStamped()
        target.pose.position.x = 30.0
        for _ in range(20):                # kapı yok, sayı değişmiyor
            node._on_target(target)
        assert len(yayin) == onceki, "sayı değişmediği hâlde tekrar yayınlandı"
    finally:
        node.destroy_node()


def test_B3_sayac_FSMe_BAGLANMADI_kasitli(ros_context) -> None:  # noqa: ANN001
    """🔴 Bu kanal parkur geçişini SÜRMEZ — bağlanırsa Parkur-2 kırılır.

    `fsm_node._on_gate_passed` gelen HERHANGİ bir True'yu PARKUR3'e atlama
    tetiği sayıyor. Sayaç oraya bağlanırsa araç İLK kapıdan geçtiğinde
    Parkur-2 yarıda kesilir → md 5.5.2.4'ün "en az 2 duba ikilisi" şartı
    sağlanmaz → md 657 gereği P3'ün 145 puanı hiç açılmaz.
    Çözüm ayrı bir karar (A: tetiğe sayaç≥2 şartı · B: geçişi waypoint
    ilerlemesinden sür) — GIRDAP_DURUM §0.6d. O karar verilene kadar bu
    test, kanalın kontrol yoluna sızmasını engeller.
    """
    import inspect

    fsm = pytest.importorskip("girdap_decision.fsm_node")
    kaynak = inspect.getsource(fsm)
    assert "/girdap/planning/gate_count" not in kaynak, (
        "fsm_node geçiş sayacına abone olmuş — Parkur-2 ilk kapıda kesilir "
        "(önce §0.6d'deki A/B kararı verilmeli)"
    )


# --------------------------------------------------------------------------- #
# KAR-03 (2026-08-12) — "sahte yeşil": sistem BOOT'ta kilitliyken 25 dakika
# boyunca 10 Hz yayın yaptı, operatör `ros2 topic hz` ile sağlıklı gördü.
# Aşağıdaki test, tehlikeli hâli DAVRANIŞ seviyesinde dondurur: görev aktif
# olsa bile poz gelmemişken thrust sıfır kalmalı.
# --------------------------------------------------------------------------- #


def test_KAR03_gorev_AKTIFken_poz_yoksa_thrust_SIFIR(ros_context) -> None:  # noqa: ANN001
    """🔴 En kritik KAR-03 testi — bekçilerin birim testi değil, SONUCU.

    Senaryo: FSM aktif parkura geçti (KAR-01'de bu, odometri (0,0,0) iken
    saniyede 10 kez oldu) ama `/girdap/fusion/odom` hiç akmadı. Boru hattının
    iç durumu `np.zeros(6)` olduğu için MPPI'nin *kontrol üretmemesi* için
    hiçbir sebep yok — üretirse tekne, nerede olduğunu bilmediği hâlde
    "orijindeyim" varsayımıyla waypoint'e doğru gaz verir.

    Bu test bekçilere DEĞİL, yayınlanan thrust'a bakar: iç mantık nasıl
    yeniden yazılırsa yazılsın, sonuç sıfır olmak zorunda.
    """
    node = pn.PlanningNode()
    try:
        node._pipe.set_mission_state("PARKUR1")       # görev aktif
        assert node._last_odom_t is None               # ama poz HİÇ gelmedi

        yayinlanan = []
        node._pub_thrust.publish = lambda m: yayinlanan.append(list(m.data))

        node._on_control_step()

        assert yayinlanan, "thrust hic yayinlanmadi — test kurulumu bozuk"
        assert yayinlanan[-1] == [0.0, 0.0], (
            f"poz yokken thrust {yayinlanan[-1]} — uydurma orijinden sürüş "
            "(KAR-03). Bekçi ya kapalı ya da 'hic gelmedi' halini atliyor."
        )
    finally:
        node.destroy_node()


# --------------------------------------------------------------------------- #
# KAR-04 / KAR-10 (2026-08-12) — "komut sıfır" ≠ "komut yok".
# --------------------------------------------------------------------------- #


def test_KAR04_kilit_sebebi_yayinlaniyor(ros_context) -> None:  # noqa: ANN001
    """🔴 Kaptanın 30.874 mesajlık analizinde her oturumda BAŞKA bir kilit
    devredeydi (BOOT / KILL / BEKLEMEDE) ama bag'e bakan kişi bunu ancak dört
    ayrı topic'i çapraz okuyarak çıkarabildi. Sebep komutun yanında olmalı.
    """
    node = pn.PlanningNode()
    try:
        sebep = []
        node._pub_inhibit.publish = lambda m: sebep.append(m.data)
        node._on_control_step()
        assert sebep, "inhibit_reason hic yayinlanmadi"
        # FSM parkur dışı + poz yok + engel yok → hepsi görünmeli
        assert "FSM-DISI" in sebep[-1]
        assert "POZ-YOK" in sebep[-1]
    finally:
        node.destroy_node()


def test_KAR04_birden_fazla_kilit_HEPSI_yaziliyor(ros_context) -> None:  # noqa: ANN001
    """Yalnız ilk sebebi yazmak, operatör birini düzeltince 'hâlâ sıfır'
    sürprizi üretirdi — bir komutu birden fazla kilit sıfırlayabilir."""
    node = pn.PlanningNode()
    try:
        sebep = []
        node._pub_inhibit.publish = lambda m: sebep.append(m.data)
        node._pipe.set_mission_state("PARKUR1")      # FSM kilidini KALDIR
        node._on_control_step()
        assert sebep
        assert "FSM-DISI" not in sebep[-1]
        # Kontrolcü henüz kurulu değil (referans yok) — bu AYRI bir sebep:
        # "görev başladı ama araç kıpırdamıyor" hâli, "görev başlamadı"dan
        # tamamen farklı bir teşhis.
        assert "KONTROLCU-HAZIR-DEGIL" in sebep[-1]
        assert "POZ-YOK" in sebep[-1] and "ENGEL-YOK" in sebep[-1], (
            f"kalan kilitler eksik: {sebep[-1]!r}"
        )
    finally:
        node.destroy_node()


def test_KAR04_kilit_YOKKEN_acikca_YOK_yaziliyor(ros_context) -> None:  # noqa: ANN001
    """🔑 Asıl ayrım: `[0,0]` bir kilidin sonucu da olabilir, MPPI'nin
    gerçekten sıfır istemesi de. "YOK" bu ikincisini adlandırır."""
    node = pn.PlanningNode()
    try:
        sebep = []
        node._pub_inhibit.publish = lambda m: sebep.append(m.data)
        node._pipe.set_mission_state("PARKUR1")
        node._last_odom_t = node._now()              # poz taze
        node._last_obstacle_t = node._now()          # engel taze
        node._bridge.update_state(node._now(), True, True, True, "GUIDED")
        node._pipe.compute_control = lambda: np.zeros(2)
        node._on_control_step()
        assert sebep and sebep[-1].startswith("YOK"), (
            f"kilit yokken sebep {sebep[-1]!r}"
        )
    finally:
        node.destroy_node()


def test_KAR04_sebep_DEGISMEDIKCE_tekrar_yayinlanmaz(ros_context) -> None:  # noqa: ANN001
    """20 Hz'te sabit metin basmak bag'i şişirir ve asıl geçiş anını
    gürültüye gömer — tam da teşhisi zorlaştıran şey."""
    node = pn.PlanningNode()
    try:
        sebep = []
        node._pub_inhibit.publish = lambda m: sebep.append(m.data)
        for _ in range(5):
            node._on_control_step()
        assert len(sebep) == 1, f"{len(sebep)} kez yayinlandi (degisim yok)"
    finally:
        node.destroy_node()


def test_KAR10_setpoint_boslugu_yakalaniyor(ros_context) -> None:  # noqa: ANN001
    """🔴 ArduPilot GUIDED'da setpoint kesilirse FAILSAFE. Kaptanın bag'inde
    en büyük sessizlik 30 DAKİKAYDI ve bunu ancak sonradan bag analizinde
    gördük — bekçi olayı ANINDA log'a düşürüyor."""
    node = pn.PlanningNode()
    try:
        hatalar = []
        node.get_logger().error = lambda m, **kw: hatalar.append(m)  # type: ignore[method-assign]
        node._setpoint_bosluk_s = 0.05
        node._setpoint_akisini_denetle()             # ilk yayin — olcum yok
        assert not hatalar
        import time
        time.sleep(0.1)
        node._setpoint_akisini_denetle()
        assert hatalar and "BOSLUK" in hatalar[-1], f"bosluk yakalanmadi: {hatalar}"
    finally:
        node.destroy_node()


def test_KAR10_gecit_KAPALIYKEN_yanlis_alarm_yok(ros_context) -> None:  # noqa: ANN001
    """🔑 Geçit kapalıyken yayın yapmamak DOĞRU davranıştır (disarm / GUIDED
    değil). Kapalıdan açığa geçişteki "boşluk" gerçek kesinti değil, kasıtlı
    sessizliktir. Ayırmazsak her arm'da yanlış alarm basar ve bekçi
    güvenilirliğini kaybeder — operatör kısa sürede onu yok saymayı öğrenir.
    """
    node = pn.PlanningNode()
    try:
        hatalar = []
        node.get_logger().error = lambda m, **kw: hatalar.append(m)  # type: ignore[method-assign]
        node._setpoint_bosluk_s = 0.05
        node._setpoint_akisini_denetle()
        import time
        time.sleep(0.1)
        node._son_setpoint_t = None                  # geçit kapandı (kontrol adımı)
        node._setpoint_akisini_denetle()             # yeniden açıldı
        assert not hatalar, f"kasitli sessizlik kesinti sayildi: {hatalar}"
    finally:
        node.destroy_node()


def test_KAR04_sebep_etiketleri_boru_hattiyla_AYNI_kumede(ros_context) -> None:  # noqa: ANN001
    """Drift kapısı: `_AKTIF_DURUMLAR` boru hattının kendi kümesiyle aynı olmalı.

    Ayrışırsa sebep etiketi sessizce YALAN söyler — örneğin boru hattı
    PARKUR4 diye bir durumu aktif sayarsa, node hâlâ "FSM-DISI" yazar ve
    operatörü olmayan bir arızaya yönlendirir.
    """
    from prototype.planning.pipeline import _ACTIVE_STATES
    assert pn._AKTIF_DURUMLAR == _ACTIVE_STATES, (
        f"sebep etiketi kumesi ayristi: {pn._AKTIF_DURUMLAR} != {_ACTIVE_STATES}"
    )


# --------------------------------------------------------------------------- #
# ARIZA KODLARI → YER KONTROL İSTASYONU (kaptan isteği, 2026-08-13)
#
# Ölçülen boşluk: 13.08 02:09'da LiDAR ağı düşmüştü, planning_node saniyede
# bir "engel haritası HİÇ gelmedi → MPPI DURDURULDU" basıyordu ama
# `/mavros/statustext/send` hattında 12 saniyede TEK mesaj yoktu — kıyıdaki
# operatör Mission Planner'da hiçbir şey göremiyordu.
# --------------------------------------------------------------------------- #


def _statustext_casusu():                               # noqa: ANN202
    """`/mavros/statustext/send`'i dinleyen ayrı düğüm + gelen mesaj listesi."""
    from mavros_msgs.msg import StatusText
    gelenler = []
    casus = rclpy.create_node("ariza_statustext_casusu")
    casus.create_subscription(
        StatusText, "/mavros/statustext/send", gelenler.append, 10
    )
    return casus, gelenler


def _abone_bekle_ve_don(node, casus, tur: int = 200):    # noqa: ANN001, ANN202
    """İki düğümü döndür; abonelik eşleşene kadar bekle (eşleşmezse False)."""
    from rclpy.executors import SingleThreadedExecutor
    yurutucu = SingleThreadedExecutor()
    yurutucu.add_node(node)
    yurutucu.add_node(casus)
    for _ in range(tur):
        yurutucu.spin_once(timeout_sec=0.01)
        if node._pub_statustext.get_subscription_count() > 0:
            return yurutucu
    return None


def test_ariza_kodu_yer_istasyonuna_GIDER(ros_context) -> None:  # noqa: ANN001
    """Engel haritası hiç gelmemişken YKİ'ye `ENGEL-YOK` kodu düşmeli.

    Bu testin yakaladığı gerçek arıza: kod ROS günlüğüne yazıyordu ama
    telsize HİÇBİR ŞEY çıkmıyordu.
    """
    from mavros_msgs.msg import StatusText
    node = pn.PlanningNode()
    casus, gelenler = _statustext_casusu()
    try:
        yurutucu = _abone_bekle_ve_don(node, casus)
        assert yurutucu is not None, "STATUSTEXT abonesi eşleşmedi (test altyapısı)"

        node._on_odom(_odom())          # poz VAR → tek eksik engel haritası
        node._on_control_step()         # sebep listesi burada üretilir
        node._ariza_gonder()
        for _ in range(50):
            yurutucu.spin_once(timeout_sec=0.01)
            if gelenler:
                break

        assert gelenler, (
            "arıza sürerken yer kontrol istasyonuna HİÇ mesaj gitmedi — "
            "operatör teknenin neden durduğunu göremez"
        )
        metin = gelenler[0].text
        assert "ENGEL-YOK" in metin, f"beklenen kod yok: {metin!r}"
        assert metin.startswith("GIRDAP "), f"önek yok: {metin!r}"
        assert len(metin) <= 50, f"MAVLink sınırı aşıldı ({len(metin)}): {metin!r}"
        assert gelenler[0].severity == StatusText.ERROR
    finally:
        casus.destroy_node()
        node.destroy_node()


def test_abone_yokken_gonderilmis_SAYILMAZ(ros_context) -> None:  # noqa: ANN001
    """MAVROS henüz hazır değilken yollanan mesaj kaybolur → tekrar denenmeli.

    `fsm_node`'un aynı dersi: abonesiz yayın sessizce çöpe gider. Arıza
    "gönderildi" diye işaretlenirse operatör o kodu BİR DAHA hiç görmez.
    """
    node = pn.PlanningNode()
    try:
        assert node._pub_statustext.get_subscription_count() == 0
        node._on_odom(_odom())
        node._on_control_step()
        node._ariza_gonder()                     # abone yok → sessiz dönmeli
        # Bildirici hiç "gönderildi" işaretlememeli:
        assert node._ariza._son_metin is None, (
            "abone yokken gönderilmiş sayıldı — abone belirince arıza bir daha "
            "hiç bildirilmez"
        )
    finally:
        node.destroy_node()


def test_ariza_gecince_TEMIZ_bildirilir(ros_context) -> None:  # noqa: ANN001
    """Operatör arızanın düzeldiğini de görmeli (yoksa ekranda asılı kalır)."""
    from geometry_msgs.msg import PoseArray
    node = pn.PlanningNode()
    casus, gelenler = _statustext_casusu()
    try:
        yurutucu = _abone_bekle_ve_don(node, casus)
        assert yurutucu is not None

        node._on_odom(_odom())
        node._on_control_step()
        node._ariza_gonder()                     # ENGEL-YOK gitti
        for _ in range(50):
            yurutucu.spin_once(timeout_sec=0.01)
            if gelenler:
                break
        assert gelenler and "ENGEL-YOK" in gelenler[0].text
        gelenler.clear()

        node._on_obstacles(PoseArray())          # engel haritası akmaya başladı
        node._on_control_step()
        node._ariza_gonder()
        for _ in range(50):
            yurutucu.spin_once(timeout_sec=0.01)
            if gelenler:
                break
        assert gelenler, "arıza düştü ama YKİ'ye haber verilmedi"
        assert "ariza yok" in gelenler[0].text, f"beklenmedik metin: {gelenler[0].text!r}"
    finally:
        casus.destroy_node()
        node.destroy_node()


# --------------------------------------------------------------------------- #
# 🔴 ARIZA KODU MANDAL KUSURU (13.08.2026 düzeltmesi)
#
# İlk sürümde altı kod yalnız `bildir()` ediliyor, hiçbir yerde `temizle()`
# edilmiyordu. `KAPI-YOK` parkurun OLAĞAN bir anıdır (iki turuncu duba görünüp
# çift kurulamadığı her an) ⇒ ilk kapı yaklaşmasında kesin ateşliyor ⇒ o andan
# sonra "ariza yok" bir daha HİÇ basılamıyor ve gerçek arıza düzeldiğinde ekran
# temiz görünmek yerine dakikalar önce olmuş bir olaya düşüyordu.
#
# Kural: **arıza kodu DURUMDUR, olay değil.** Tek istisna `GPU-YOK`.
# --------------------------------------------------------------------------- #


def test_KAPI_YOK_kapi_kilitlenince_DUSER(ros_context) -> None:  # noqa: ANN001
    """Mandal kusurunun ta kendisi: kapı kilitlenince kod düşmeli.

    Bu test mandal geri gelirse KIRMIZI olur — düşmeyen `KAPI-YOK` görev
    boyunca "ariza yok" satırını da öldürür.
    """
    from prototype.telemetry.ariza_bildirici import KAPI_YOK
    node = pn.PlanningNode()
    try:
        # Kapı kilitli DEĞİL + iki turuncu duba görünüyor → arıza.
        node._last_gate_used_fallback = True
        node._gate.last_diagnostics.n_edge_buoys = 2
        node._ariza_durumlardan_guncelle()
        assert KAPI_YOK.kod in node._ariza.aktif_kodlar

        # Kapı KİLİTLENDİ → arıza DÜŞMELİ.
        node._last_gate_used_fallback = False
        node._ariza_durumlardan_guncelle()
        assert KAPI_YOK.kod not in node._ariza.aktif_kodlar, (
            "kapı kilitlendiği hâlde KAPI-YOK aktif kaldı — mandal kusuru geri geldi"
        )
    finally:
        node.destroy_node()


def test_KAPI_YOK_tek_duba_varken_ARIZA_DEGIL(ros_context) -> None:  # noqa: ANN001
    """İki dubadan azken kapı beklemek anlamsız — yanlış alarm basılmamalı."""
    from prototype.telemetry.ariza_bildirici import KAPI_YOK
    node = pn.PlanningNode()
    try:
        node._last_gate_used_fallback = True
        node._gate.last_diagnostics.n_edge_buoys = 1
        node._ariza_durumlardan_guncelle()
        assert KAPI_YOK.kod not in node._ariza.aktif_kodlar
    finally:
        node.destroy_node()


def test_SINIF_YOK_akis_donunce_DUSER(ros_context) -> None:  # noqa: ANN001
    """Sınıflı algı geri gelince kod düşmeli (kurtarma dalı zaten vardı)."""
    from prototype.telemetry.ariza_bildirici import SINIF_YOK
    node = pn.PlanningNode()
    try:
        # Akış bir kez görüldü, sonra bayatladı.
        node._classified_seen = True
        node._last_classified_t = node._now() - (node._classified_timeout + 5.0)
        node._ariza_durumlardan_guncelle()
        assert SINIF_YOK.kod in node._ariza.aktif_kodlar

        # Akış DÖNDÜ.
        node._last_classified_t = node._now()
        node._ariza_durumlardan_guncelle()
        assert SINIF_YOK.kod not in node._ariza.aktif_kodlar, (
            "sınıflı akış döndüğü hâlde SINIF-YOK aktif kaldı"
        )
    finally:
        node.destroy_node()


def test_ENGEL_BOS_dolu_kare_gelince_DUSER(ros_context) -> None:  # noqa: ANN001
    """Algı yeniden cisim görmeye başlayınca kod düşmeli."""
    from prototype.telemetry.ariza_bildirici import ENGEL_BOS
    node = pn.PlanningNode()
    try:
        node._son_dolu_akis_t = node._now() - (node._bos_akis_uyari_s + 5.0)
        node._ariza_durumlardan_guncelle()
        assert ENGEL_BOS.kod in node._ariza.aktif_kodlar

        node._son_dolu_akis_t = node._now()          # dolu kare geldi
        node._ariza_durumlardan_guncelle()
        assert ENGEL_BOS.kod not in node._ariza.aktif_kodlar, (
            "algı yeniden cisim gördüğü hâlde ENGEL-BOS aktif kaldı"
        )
    finally:
        node.destroy_node()


def test_olay_arizalari_tutma_suresi_sonunda_DUSER(ros_context) -> None:  # noqa: ANN001
    """`SETPOINT`/`CMDVEL` olaydır: tutma süresi dolunca kendiliğinden düşer.

    Bir olay hiçbir zaman "düzelmez", bu yüzden durum yüklemi yazılamaz —
    ama sonsuza kadar da asılı kalmamalı.
    """
    from prototype.telemetry.ariza_bildirici import CMDVEL_KESIK, SETPOINT_BOSLUK
    node = pn.PlanningNode()
    try:
        simdi = node._now()
        node._son_setpoint_bosluk_t = simdi
        node._son_cmdvel_bosluk_t = simdi
        node._ariza_durumlardan_guncelle()
        assert SETPOINT_BOSLUK.kod in node._ariza.aktif_kodlar
        assert CMDVEL_KESIK.kod in node._ariza.aktif_kodlar

        # Tutma süresi geçmiş gibi davran (saati ileri almak yerine olayı geri al).
        gecmis = simdi - (node._ariza_olay_tutma_s + 1.0)
        node._son_setpoint_bosluk_t = gecmis
        node._son_cmdvel_bosluk_t = gecmis
        node._ariza_durumlardan_guncelle()
        assert SETPOINT_BOSLUK.kod not in node._ariza.aktif_kodlar, (
            "tutma süresi dolduğu hâlde SETPOINT aktif kaldı"
        )
        assert CMDVEL_KESIK.kod not in node._ariza.aktif_kodlar
    finally:
        node.destroy_node()


def test_GPU_YOK_bilerek_mandalli_kalir(ros_context) -> None:  # noqa: ANN001
    """`GPU-YOK` istisnadır: hesap yolu bir kez seçilir, koşu ortasında değişmez.

    Bu testin işi mandalı KORUMAK — biri "tutarlılık olsun" diye onu da durum
    yüklemine çevirirse burada durur.
    """
    from prototype.telemetry.ariza_bildirici import GPU_YOK
    node = pn.PlanningNode()
    try:
        node._ariza.bildir(GPU_YOK)
        for _ in range(5):
            node._ariza_durumlardan_guncelle()
        assert GPU_YOK.kod in node._ariza.aktif_kodlar, (
            "GPU-YOK düşürüldü — tekne koşu ortasında GPU'ya kavuşmaz, "
            "bu kodun mandallı kalması DOĞRU davranıştır"
        )
    finally:
        node.destroy_node()


def test_hicbir_ariza_SONSUZA_KADAR_asili_kalmaz(ros_context) -> None:  # noqa: ANN001
    """🔑 YAPISAL NÖBETÇİ — mandal kusurunun SINIFINI kapatır.

    Tek tek testler yeni bir kod eklendiğinde sessiz kalır. Bu test kaynağı
    okuyup şunu dayatır: `_ariza.bildir(X)` ile basılan her kodun ya açık bir
    `temizle(X)` karşılığı olmalı, ya da bilerek mandallı olduğu burada
    yazılmalı. Yeni bir arıza kodu temizleme yolu olmadan eklenirse KIRMIZI.
    """
    import inspect
    import re

    # Bilerek mandallı kalanlar — her birinin gerekçesi koddaki yorumda.
    MANDALLI_KALMASI_DOGRU = {
        "GPU_YOK",        # hesap yolu açılışta bir kez seçilir (bkz. test yukarıda)
    }

    kaynak = inspect.getsource(pn)
    basilan = set(re.findall(r"_ariza\.bildir\(\s*([A-Z_]+)\s*\)", kaynak))
    temizlenen = set(re.findall(r"_ariza\.temizle\(\s*([A-Z_]+)\s*\)", kaynak))

    asili = basilan - temizlenen - MANDALLI_KALMASI_DOGRU
    assert not asili, (
        f"şu arıza kodları basılıyor ama HİÇ temizlenmiyor: {sorted(asili)}. "
        "Mandallı bir kod görev boyunca 'ariza yok' satırını da öldürür ve "
        "operatör kodun ŞİMDİ mi GEÇMİŞTE mi olduğunu ayırt edemez. Kodu "
        "`_ariza_durumlardan_guncelle`'de durum yüklemiyle kurun ya da "
        "bilerek mandallıysa MANDALLI_KALMASI_DOGRU'ya gerekçesiyle ekleyin."
    )


def test_RRT_RED_abonesiz_donemde_BAYAT_alarm_uretmez(ros_context) -> None:  # noqa: ANN001
    """Abone yokken sayaç tabanı donarsa, abone gelince sahte alarm çakar.

    MAVROS abone olduğu an operatörün ekranı açılır; ilk gördüğü şey dakikalar
    önce olmuş bir düşüş OLMAMALI.
    """
    from prototype.telemetry.ariza_bildirici import RRT_RED
    node = pn.PlanningNode()
    try:
        # Abone YOK. Bu dönemde RRT birkaç kez düz çizgiye düştü.
        # (`duz_cizgiye_dusuldu` salt-okunur property → arkasındaki alan.)
        node._pipe._duz_cizgiye_dusuldu = 3
        node._ariza_gonder()                 # abonesiz tur — taban güncellenmeli
        assert node._son_duz_cizgi_sayaci == 3, (
            "abonesiz turda RRT sayaç tabanı donmuş — abone gelince bayat "
            "RRT-RED çakar"
        )
        # Abone belirdi, yeni düşüş YOK.
        node._ariza_gonder()
        assert RRT_RED.kod not in node._ariza.aktif_kodlar
    finally:
        node.destroy_node()
