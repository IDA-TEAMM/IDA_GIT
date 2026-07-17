"""
Girdap İDA — fsm_node entegrasyon testleri (F12.1 / F16.5).

F12.1: `last_waypoint_xy` parametresini HİÇBİR ŞEY yazmıyordu ([0,0] varsayılan)
→ görev başlar başlamaz odom origin'e 0 m = SAHTE P1→P2 geçişi. Düzeltme:
(a) [0,0] varsayılanı "ayarlanmamış" sayılır (mesafe hesaplanmaz),
(b) gerçek tetik: /girdap/mission/waypoint_reached index'i parkur-1'in SON
    index'ine ulaşınca dist_to_last_wp_p1=0 beslenir (waypoint-index tabanlı,
    CLAUDE.md FSM ilkesi).

Callback'ler DOĞRUDAN çağrılır (DDS keşif kırılganlığı yok); mesajlar gerçek
tiplerdir. mavros_msgs gerektirir → yoksa dürüst SKIP.

Çalıştır: pytest prototype/tests/test_fsm_node.py -v
"""

from __future__ import annotations

import textwrap

import pytest

rclpy = pytest.importorskip("rclpy", reason="rclpy yok (.venv) — ROS ortamında koş")

from rclpy.parameter import Parameter                    # noqa: E402
from std_msgs.msg import Bool, Int32                     # noqa: E402
from nav_msgs.msg import Odometry                        # noqa: E402

girdap = pytest.importorskip(
    "girdap_decision.fsm_node",
    reason="girdap_decision.fsm_node import edilemedi (ros2_ws source'lanmamış "
    "YA DA mavros_msgs kurulu değil: bkz. ~/girdap_deps_ws)",
)
from mavros_msgs.msg import State as MavState            # noqa: E402
from std_srvs.srv import Trigger                         # noqa: E402

from prototype.fsm.mission_fsm import MissionState      # noqa: E402


@pytest.fixture(scope="module")
def ros_context():                                       # noqa: ANN201
    rclpy.init()
    yield
    rclpy.shutdown()


def _make_node(ros_context, tmp_path, labels=None, last_wp=None, mission_source=None):  # noqa: ANN001
    """Parametre enjeksiyonlu FSMNode kur (timer'a spin edilmez)."""
    overrides = []
    if labels is not None:
        mission = tmp_path / "mission.yaml"
        wps = "\n".join(
            f"  - {{lat: 0.0, lon: 0.0, parkur: {p}}}" for p in labels
        )
        mission.write_text(f"waypoints:\n{wps}\n", encoding="utf-8")
        overrides.append(Parameter("mission_file", value=str(mission)))
    if last_wp is not None:
        overrides.append(Parameter("last_waypoint_xy", value=last_wp))
    if mission_source is not None:
        overrides.append(Parameter("mission_source", value=mission_source))
    return girdap.FSMNode(parameter_overrides=overrides)


def _drive_to_parkur1(node) -> None:                     # noqa: ANN001
    """BOOT→ARM→BEKLEMEDE→PARKUR1 (gerçek callback + tick zinciriyle)."""
    mav = MavState()
    mav.connected = True
    mav.armed = True
    node._on_mav_state(mav)
    node._on_tick()                                      # BOOT→ARM
    node._on_tick()                                      # ARM→BEKLEMEDE
    node._on_start_srv(Trigger.Request(), Trigger.Response())
    node._on_tick()                                      # BEKLEMEDE→PARKUR1
    assert node._fsm.state is MissionState.PARKUR1


def _odom_at(x: float, y: float) -> Odometry:
    od = Odometry()
    od.pose.pose.position.x = x
    od.pose.pose.position.y = y
    return od


# ---------------------------------------------------------------- F12.1

def test_default_param_no_spurious_p1_to_p2(ros_context, tmp_path) -> None:  # noqa: ANN001
    """F12.1 repro: param [0,0] (ayarlanmamış) + araç odom origin'de →
    P1→P2 geçişi OLMAMALI (eski kod: dist=0 → anında sahte PARKUR2)."""
    node = _make_node(ros_context, tmp_path, labels=[1, 1, 2])
    try:
        _drive_to_parkur1(node)
        node._on_odom(_odom_at(0.0, 0.0))                # boot konumu = origin
        node._on_tick()
        node._on_tick()
        assert node._fsm.state is MissionState.PARKUR1, (
            "ayarlanmamış last_waypoint_xy [0,0] sahte P1→P2 tetikledi (F12.1)"
        )
    finally:
        node.destroy_node()


# ------------------------------------------------------ F-P.23: armed watchdog


def test_fp23_armed_bekleme_uzarsa_uyari_basar(ros_context, tmp_path) -> None:  # noqa: ANN001
    """F-P.23 (2026-07-17): 2026-07-16 gerçek donanım testinde start_on_mode
    ("AUTO") ile aracın GERÇEK modu (GUIDED) uyuşmadığı için FSM hiç
    BEKLEMEDE'den çıkmadı, hiçbir hata/uyarı basılmadan sessizce kaldı.
    Artık armed+BEKLEMEDE eşiği aşınca GÜRÜLTÜLÜ uyarı basılmalı."""
    node = girdap.FSMNode(
        parameter_overrides=[
            Parameter("start_on_mode", value="GUIDED"),
            Parameter("armed_bekleme_watchdog_s", value=0.05),
        ]
    )
    try:
        errors: list[str] = []
        node.get_logger().error = lambda msg, **kw: errors.append(msg)  # type: ignore[method-assign]

        mav = MavState()
        mav.connected = True
        mav.armed = True
        mav.mode = "AUTO"                 # start_on_mode (GUIDED) ile UYUŞMUYOR
        node._on_mav_state(mav)
        node._on_tick()                   # BOOT→ARM
        node._on_tick()                   # ARM→BEKLEMEDE (mod uyuşmuyor, hiç başlamaz)

        import time
        time.sleep(0.1)                   # eşiği (0.05s) rahatça geç

        node._on_tick()
        assert node._fsm.state is MissionState.BEKLEMEDE, (
            "mod uyuşmuyorken görev başlamamalıydı"
        )
        assert len(errors) == 1
        assert "ARMED" in errors[0] and "BEKLEMEDE" in errors[0]
        assert "GUIDED" in errors[0] and "AUTO" in errors[0]
    finally:
        node.destroy_node()


def test_fp23_mod_uyusursa_uyari_hic_basmaz(ros_context, tmp_path) -> None:  # noqa: ANN001
    """Mod start_on_mode ile eşleşiyorsa görev normal başlar, watchdog hiç
    tetiklenmemeli (yanlış alarm yok).

    NOT: request_start() bir bayrak set eder, gerçek BEKLEMEDE→PARKUR1
    geçişi BİR SONRAKİ tick'te olur (normal FSM davranışı) — bu yüzden
    eşik, birkaç tick'in normal yürütme süresinden (milisaniyeler) belirgin
    büyük tutulmalı (0.01s gibi aşırı küçük bir eşik bunu bile "sorun"
    sayıp yanlış alarm verirdi — canlı olarak bulundu, testte de düzeltildi)."""
    node = girdap.FSMNode(
        parameter_overrides=[
            Parameter("start_on_mode", value="GUIDED"),
            Parameter("start_on_arm_in_mode", value=True),
            Parameter("armed_bekleme_watchdog_s", value=5.0),
        ]
    )
    try:
        errors: list[str] = []
        node.get_logger().error = lambda msg, **kw: errors.append(msg)  # type: ignore[method-assign]

        mav = MavState()
        mav.connected = True
        mav.armed = True
        mav.mode = "GUIDED"                # start_on_mode ile UYUŞUYOR
        node._on_mav_state(mav)
        node._on_tick()                    # BOOT→ARM
        node._on_tick()                    # ARM→BEKLEMEDE (+ F-V.6 request_start bayrağı)
        node._on_tick()                    # BEKLEMEDE→PARKUR1 (bayrak bir tick sonra işlenir)

        assert node._fsm.state is MissionState.PARKUR1
        assert errors == []
    finally:
        node.destroy_node()


def test_waypoint_index_triggers_p1_to_p2(ros_context, tmp_path) -> None:  # noqa: ANN001
    """Gerçek tetik: parkur-1'in SON waypoint'ine (index 1) varış → PARKUR2.
    Ara waypoint (index 0) geçiş tetiklememeli."""
    node = _make_node(ros_context, tmp_path, labels=[1, 1, 2])
    try:
        _drive_to_parkur1(node)
        node._on_waypoint_reached(Int32(data=0))         # ara wp
        node._on_tick()
        assert node._fsm.state is MissionState.PARKUR1
        node._on_waypoint_reached(Int32(data=1))         # parkur-1 SON wp
        node._on_tick()
        assert node._fsm.state is MissionState.PARKUR2
    finally:
        node.destroy_node()


def test_waypoint_reached_no_spurious_p1_to_p2_when_no_parkur2(ros_context, tmp_path) -> None:  # noqa: ANN001
    """BULGU 1 repro (Yahya, son_kod video koşul matrisi 2026-07-14): tek
    parkurlu görevde (parkur-2 YOK) son waypoint'e varış hâlâ koşulsuz
    dist_to_last_wp_p1=0 besliyordu → PARKUR1→PARKUR2 sahte geçişi;
    mission_complete (dwell_time_s kadar gecikmeli) gelene dek birkaç saniye
    yanlış PARKUR2 gösteriyordu (Dosya-2'de yanıltıcı satır). Düzeltme:
    yalnız gerçekten bir parkur-2 varsa beslenir (ParkurTransitionLogic'in
    kendi _has_parkur guard'ıyla tutarlı)."""
    node = _make_node(ros_context, tmp_path, labels=[1, 1])   # parkur-2 YOK
    try:
        _drive_to_parkur1(node)
        node._on_waypoint_reached(Int32(data=1))              # tek parkurun SON wp'si
        node._on_tick()
        assert node._fsm.state is MissionState.PARKUR1, (
            "parkur-2 hiç yokken son waypoint'e varış sahte PARKUR2 tetikledi (BULGU 1)"
        )
        node._on_mission_complete(Bool(data=True))
        node._on_tick()
        assert node._fsm.state is MissionState.TAMAMLANDI
    finally:
        node.destroy_node()


def test_fp9_bozuk_parkur_dosyasi_node_coker_mi(ros_context, tmp_path) -> None:  # noqa: ANN001
    """F-P.9: contiguous-olmayan parkur etiketleri (veri girişi hatası)
    ParkurTransitionLogic'te ValueError fırlatır — fsm_node bunu yakalayıp
    tek parkur GÜVENLİ moduna düşmeli, ÇÖKMEMELİ."""
    node = _make_node(ros_context, tmp_path, labels=[1, 1, 2, 1, 3])
    try:
        assert node._parkur.last_index_of_parkur == {}   # güvenli tek-parkur
    finally:
        node.destroy_node()


def test_fp8_fc_coklu_parkur_uyarisi_coker_mi(ros_context, tmp_path) -> None:  # noqa: ANN001
    """F-P.8 (robustness taraması, 2026-07-15): mission_source=fc + çoklu
    parkur içeren mission_file kombinasyonu KRİTİK bir senkron riski (FC
    waypoint'leri her zaman parkur=1 sayılır — bkz. kod yorumu). Bu test tam
    düzeltmeyi (otomatik senkron, kod düzeyinde mümkün değil) DEĞİL, en
    azından node'un çökmediğini ve parkur logic'inin normal kurulduğunu
    doğrular (uyarı metni ROS logger'a gider, pytest'te doğrudan yakalanmaz)."""
    node = _make_node(
        ros_context, tmp_path, labels=[1, 1, 2, 2, 3], mission_source="fc",
    )
    try:
        assert node._parkur.last_index_of_parkur == {1: 1, 2: 3, 3: 4}
    finally:
        node.destroy_node()


def test_fp8_file_kaynagi_uyari_uretmez(ros_context, tmp_path) -> None:  # noqa: ANN001
    """mission_source=file (varsayılan) çoklu parkurla tamamen NORMAL —
    F-P.8 uyarısı yalnız fc modunda anlamlı, burada tetiklenmemeli."""
    node = _make_node(
        ros_context, tmp_path, labels=[1, 1, 2, 2, 3], mission_source="file",
    )
    try:
        assert node._parkur.last_index_of_parkur == {1: 1, 2: 3, 3: 4}
    finally:
        node.destroy_node()


def test_explicit_param_distance_path_still_works(ros_context, tmp_path) -> None:  # noqa: ANN001
    """Operatör gerçek koordinat verirse odom-mesafe yolu çalışmaya devam
    etmeli (guard yalnız [0,0] varsayılanını devre dışı bırakır)."""
    node = _make_node(ros_context, tmp_path, labels=[1, 1, 2], last_wp=[30.0, 10.0])
    try:
        _drive_to_parkur1(node)
        node._on_odom(_odom_at(0.0, 0.0))                # 31.6 m uzak
        node._on_tick()
        assert node._fsm.state is MissionState.PARKUR1
        node._on_odom(_odom_at(30.5, 10.0))              # 0.5 m < 1.5 m eşik
        node._on_tick()
        assert node._fsm.state is MissionState.PARKUR2
    finally:
        node.destroy_node()


def test_video_mission_completes_from_parkur1(ros_context, tmp_path) -> None:  # noqa: ANN001
    """Video regresyonu (F12.2 ile etkileşim): etiketsiz tek parkur + origin
    odom → PARKUR1'de kalır; mission_complete → TAMAMLANDI (temiz duruş)."""
    node = _make_node(ros_context, tmp_path)             # mission_file yok
    try:
        _drive_to_parkur1(node)
        node._on_odom(_odom_at(0.0, 0.0))
        node._on_tick()
        assert node._fsm.state is MissionState.PARKUR1   # sahte geçiş yok
        node._on_mission_complete(Bool(data=True))
        node._on_tick()
        assert node._fsm.state is MissionState.TAMAMLANDI
    finally:
        node.destroy_node()


# ------------------------------------------------- md 3.3.1(3) GUIDED tetiği

def _mav(mode: str, armed: bool = True, connected: bool = True) -> MavState:
    m = MavState()
    m.connected = connected
    m.armed = armed
    m.mode = mode
    return m


def test_guided_edge_starts_mission_in_beklemede(ros_context, tmp_path) -> None:  # noqa: ANN001
    """md 3.3.1(3): BEKLEMEDE'de operatör modu GUIDED'a ÇEVİRİNCE (QGC →
    RFD868 → FCU → /mavros/state) görev başlar — YKİ'den tek komut."""
    node = _make_node(ros_context, tmp_path)
    try:
        node._on_mav_state(_mav("MANUAL"))
        node._on_tick()                                  # BOOT→ARM
        node._on_tick()                                  # ARM→BEKLEMEDE
        assert node._fsm.state is MissionState.BEKLEMEDE
        node._on_mav_state(_mav("GUIDED"))               # operatör komutu
        node._on_tick()                                  # BEKLEMEDE→PARKUR1
        assert node._fsm.state is MissionState.PARKUR1
    finally:
        node.destroy_node()


def test_boot_already_guided_no_edge_no_start(ros_context, tmp_path) -> None:  # noqa: ANN001
    """İlk görülen mod zaten GUIDED ise kenar yok → arm etmek başlatmak
    DEĞİLDİR; araç BEKLEMEDE'de kalır (güvenlik: iki ayrı operatör komutu)."""
    node = _make_node(ros_context, tmp_path)
    try:
        node._on_mav_state(_mav("GUIDED"))
        node._on_tick()                                  # BOOT→ARM
        node._on_tick()                                  # ARM→BEKLEMEDE
        node._on_mav_state(_mav("GUIDED"))               # aynı mod, kenar yok
        node._on_tick()
        assert node._fsm.state is MissionState.BEKLEMEDE
    finally:
        node.destroy_node()


def test_guided_edge_outside_beklemede_ignored(ros_context, tmp_path) -> None:  # noqa: ANN001
    """BEKLEMEDE dışında (disarmed, ARM'da) mod geçişi görevi başlatmaz."""
    node = _make_node(ros_context, tmp_path)
    try:
        node._on_mav_state(_mav("MANUAL", armed=False))
        node._on_tick()                                  # BOOT→ARM
        assert node._fsm.state is MissionState.ARM
        node._on_mav_state(_mav("GUIDED", armed=False))
        node._on_tick()
        assert node._fsm.state is MissionState.ARM
    finally:
        node.destroy_node()


def test_start_on_mode_empty_disables_trigger(ros_context, tmp_path) -> None:  # noqa: ANN001
    """start_on_mode="" → tetik kapalı; başlatma yalnız servisle."""
    node = girdap.FSMNode(
        parameter_overrides=[Parameter("start_on_mode", value="")]
    )
    try:
        node._on_mav_state(_mav("MANUAL"))
        node._on_tick()
        node._on_tick()
        assert node._fsm.state is MissionState.BEKLEMEDE
        node._on_mav_state(_mav("GUIDED"))
        node._on_tick()
        assert node._fsm.state is MissionState.BEKLEMEDE
    finally:
        node.destroy_node()


# ----- F-V.6: AUTO videosunda "önce mod, sonra ARM" sırası -----
#
# B1 ile görevi FC AUTO'da uçuruyor. Operatör QGC'de modu AUTO yapıp SONRA arm
# ederse (QGC "Start Mission" akışı; ArduRover AUTO'da arm olunca görevi
# başlatır) mod KENARI hiç görülmez: FSM BEKLEMEDE'ye armed olarak girer ve
# mod ZATEN AUTO'dur. Eski kenar-şartı bu durumda görevi BAŞLATMIYORDU →
# mission_state="BEKLEMEDE" → telemetry F-V.2 gereği setpoint sütunlarını BOŞ
# bırakır → Ekran-2'nin ZORUNLU hız/yön setpoint eğrileri boş çıkar
# (md 3.3.1.1) — üstelik video tek çekim, çekerken fark edilmez.


def test_fv6_mod_once_sonra_arm_gorevi_baslatir(ros_context, tmp_path) -> None:  # noqa: ANN001
    """AUTO'dayken ARM edilirse (mod kenarı YOK) görev yine başlamalı."""
    node = girdap.FSMNode(
        parameter_overrides=[
            Parameter("start_on_mode", value="AUTO"),
            Parameter("start_on_arm_in_mode", value=True),   # video config
        ]
    )
    try:
        # Operatör önce modu AUTO yaptı — henüz DISARM (FSM BOOT/ARM'da).
        node._on_mav_state(_mav("AUTO", armed=False))
        node._on_tick()
        assert node._fsm.state is MissionState.ARM

        # Sonra ARM etti: mod DEĞİŞMEDİ (kenar yok), FSM BEKLEMEDE'ye girer.
        node._on_mav_state(_mav("AUTO", armed=True))
        node._on_tick()          # ARM → BEKLEMEDE
        node._on_tick()          # BEKLEMEDE + mod zaten AUTO → başlamalı
        assert node._fsm.state is MissionState.PARKUR1, (
            "FC görevi koşuyor ama FSM BEKLEMEDE'de kaldı → Ekran-2 setpoint "
            "sütunları boş kalır (md 3.3.1.1)"
        )
    finally:
        node.destroy_node()


def test_fv6_varsayilan_kapali_yarisma_guvenligi_korunur(ros_context, tmp_path) -> None:  # noqa: ANN001
    """VARSAYILAN (start_on_arm_in_mode=false): kenarsız başlatma YOK.

    Yarışma (GUIDED+MPPI): FC zaten GUIDED'dayken arm edilirse görev
    KENDİLİĞİNDEN başlamamalı — MPPI motorları sürerdi. Kasıtlı mod komutu şart.
    """
    node = girdap.FSMNode(
        parameter_overrides=[Parameter("start_on_mode", value="GUIDED")]
    )
    try:
        node._on_mav_state(_mav("GUIDED", armed=True))
        node._on_tick()
        node._on_tick()
        assert node._fsm.state is MissionState.BEKLEMEDE   # başlamadı ✓
    finally:
        node.destroy_node()


def test_fv6_kenar_tetigi_hala_calisiyor(ros_context, tmp_path) -> None:  # noqa: ANN001
    """Regresyon: klasik yol (ARM → sonra mod komutu) bozulmamalı."""
    node = girdap.FSMNode(
        parameter_overrides=[
            Parameter("start_on_mode", value="AUTO"),
            Parameter("start_on_arm_in_mode", value=True),
        ]
    )
    try:
        node._on_mav_state(_mav("HOLD", armed=True))
        node._on_tick()
        node._on_tick()
        assert node._fsm.state is MissionState.BEKLEMEDE
        node._on_mav_state(_mav("AUTO", armed=True))       # kenar
        node._on_tick()
        assert node._fsm.state is MissionState.PARKUR1
    finally:
        node.destroy_node()
