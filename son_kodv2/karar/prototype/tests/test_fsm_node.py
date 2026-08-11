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
from pathlib import Path

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


def _make_node(ros_context, tmp_path, labels=None, last_wp=None, mission_source=None, extra_params=None):  # noqa: ANN001
    """Parametre enjeksiyonlu FSMNode kur (timer'a spin edilmez)."""
    overrides = list(extra_params) if extra_params else []
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


# ------------------------------------------------- F-A.4 STATUSTEXT (md 4.2)
# Şartname md 4.2: "Aracın anlık durum ve mod bilgileri İDA YKİ ekranında
# görülecektir." MOD MAVLink'ten zaten geliyordu; DURUM (görev/parkur) hiçbir
# yerde görünmüyordu. fsm_node artık /mavros/statustext/send'e yayın yapıyor.


def _statustext_spy(node, abone_sayisi: int = 1):         # noqa: ANN001, ANN202
    """Gerçek publisher'ı casusla değiştir (DDS'e bağımlı olmadan doğrula).

    `abone_sayisi` ŞART: fsm_node abonesi olmayan topic'e yayın yapmıyor
    (abonesiz yayın sessizce çöpe gider — 11.08 bulgusu). Varsayılan 1 =
    "MAVROS hazır". 0 vererek açılış yarışı taklit edilir.
    """
    sent = []
    class _Spy:
        def publish(self, msg):                          # noqa: ANN001, ANN202
            sent.append(msg)
        def get_subscription_count(self) -> int:
            return abone_sayisi
    node._pub_statustext = _Spy()
    return sent


def test_fa4_durum_degisiminde_statustext_gonderilir(ros_context, tmp_path) -> None:  # noqa: ANN001
    """Görev durumu değişince YKİ'ye tek satır gider (md 4.2)."""
    node = _make_node(ros_context, tmp_path, labels=[1, 1, 2])
    try:
        sent = _statustext_spy(node)
        _drive_to_parkur1(node)
        assert sent, "durum değişti ama STATUSTEXT gönderilmedi"
        metinler = [m.text for m in sent]
        assert any("GIRDAP" in t for t in metinler), metinler
        assert any("PARKUR1" in t for t in metinler), metinler
    finally:
        node.destroy_node()


def test_fa4_ayni_durumda_tekrar_gondermez(ros_context, tmp_path) -> None:  # noqa: ANN001
    """10 Hz tick MAVLink hattını doldurmamalı — periyot içinde tek yayın."""
    node = _make_node(ros_context, tmp_path, labels=[1, 1, 2])
    try:
        sent = _statustext_spy(node)                     # casus ÖNCE takılır
        _drive_to_parkur1(node)
        sent.clear()                                     # geçiş mesajlarını at
        for _ in range(20):                              # 2 saniyelik tick
            node._on_tick()
        # 20 tick gerçek zamanda mikrosaniyeler sürer; tazeleme periyodu
        # (10 s) dolmadığı için hiçbir şey gitmemeli.
        assert sent == [], f"durum sabitken {len(sent)} gereksiz mesaj gitti"
    finally:
        node.destroy_node()


def test_fa4_periyot_dolunca_TAZELENIR(ros_context, tmp_path) -> None:  # noqa: ANN001
    """🔴 11.08 SAHA BULGUSU: durum değişmese de periyodik tazeleme ŞART.

    Yalnız-değişimde yayın, FSM açılışta oturduktan sonra hattı sonsuza kadar
    sessiz bırakıyordu; canlı ölçümde 20 saniyede SIFIR mesaj vardı ve MP'yi
    sonradan açan operatör hiçbir şey görmüyordu. 868 MHz'de kopma+yeniden
    bağlanma normal olduğu için bu yarışmada da tekrarlanır.
    """
    node = _make_node(ros_context, tmp_path, labels=[1, 1, 2])
    try:
        sent = _statustext_spy(node)
        _drive_to_parkur1(node)
        sent.clear()
        # Son gönderimi periyottan daha geriye al = süre geçmiş gibi yap
        node._statustext_son_gonderim -= node._statustext_periyot_s + 1.0
        node._on_tick()
        assert sent, "periyot doldu ama durum TAZELENMEDİ — YKİ ekranı boş kalır"
        assert "PARKUR1" in sent[0].text, sent[0].text
    finally:
        node.destroy_node()


def test_fa4_periyot_sifir_ise_yalniz_degisimde(ros_context, tmp_path) -> None:  # noqa: ANN001
    """`statustext_periyot_s=0` → eski yalnız-değişimde davranışı BİREBİR."""
    node = _make_node(
        ros_context, tmp_path, labels=[1, 1, 2],
        extra_params=[Parameter("statustext_periyot_s", value=0.0)],
    )
    try:
        sent = _statustext_spy(node)
        _drive_to_parkur1(node)
        sent.clear()
        node._statustext_son_gonderim -= 3600.0          # 1 saat geçmiş olsun
        node._on_tick()
        assert sent == [], "periyot 0 iken tazeleme YAPILMAMALI"
    finally:
        node.destroy_node()


def test_fa4_abonesiz_yayin_GONDERILMIS_SAYILMAZ(ros_context, tmp_path) -> None:  # noqa: ANN001
    """🔴 AÇILIŞ YARIŞI: MAVROS abone olmadan yollanan mesaj çöpe gider.

    `girdap-karar` fsm_node ile MAVROS'u birlikte başlatıyor; MAVROS'un `sys`
    eklentisi abone olana kadarki pencerede açılış geçişleri (BOOT→ARM→
    BEKLEMEDE) kayboluyordu. Doğru davranış: gönderilmiş SAYMA, abone
    belirince aynı metni tekrar dene.
    """
    node = _make_node(ros_context, tmp_path, labels=[1, 1, 2])
    try:
        yok = _statustext_spy(node, abone_sayisi=0)      # MAVROS hazır DEĞİL
        _drive_to_parkur1(node)
        assert yok == [], "abonesiz yayın yapılmamalı"
        assert node._last_statustext == "", (
            "abonesiz denemede 'gönderildi' diye kaydedilmiş — abone "
            "belirdiğinde durum bir daha ASLA yollanmaz"
        )
        # Abone belirdi → bir sonraki tick mevcut durumu yollamalı
        var = _statustext_spy(node, abone_sayisi=1)
        node._on_tick()
        assert var, "abone belirdi ama durum yollanmadı"
        assert "PARKUR1" in var[0].text, var[0].text
    finally:
        node.destroy_node()


def test_fa4_metin_mavlink_sinirini_asmaz(ros_context, tmp_path) -> None:  # noqa: ANN001
    """MAVLink STATUSTEXT metni 50 karakterle sınırlı."""
    node = _make_node(ros_context, tmp_path, labels=[1, 1, 2])
    try:
        sent = _statustext_spy(node)
        _drive_to_parkur1(node)
        for m in sent:
            assert len(m.text) <= 50, f"{len(m.text)} karakter: {m.text!r}"
    finally:
        node.destroy_node()


def test_fa4_kill_kritik_seviyede_gider(ros_context, tmp_path) -> None:  # noqa: ANN001
    """KILL operatörün ANINDA görmesi gereken tek durum → CRITICAL."""
    from mavros_msgs.msg import StatusText

    node = _make_node(ros_context, tmp_path, labels=[1, 1, 2])
    try:
        _drive_to_parkur1(node)
        sent = _statustext_spy(node)
        node._on_kill_srv(Trigger.Request(), Trigger.Response())
        node._on_tick()
        kill_msgs = [m for m in sent if "KILL" in m.text]
        assert kill_msgs, f"KILL bildirilmedi: {[m.text for m in sent]}"
        assert kill_msgs[0].severity == StatusText.CRITICAL
    finally:
        node.destroy_node()


def test_fa4_kapatilabilir(ros_context, tmp_path) -> None:  # noqa: ANN001
    """statustext_enabled=false → publisher hiç kurulmaz (mavros'suz test)."""
    node = girdap.FSMNode(
        parameter_overrides=[Parameter("statustext_enabled", value=False)]
    )
    try:
        assert node._pub_statustext is None
        node._on_tick()                                  # çökmemeli
    finally:
        node.destroy_node()


# --------------------------------------------------------------------------- #
# ŞOK EŞİĞİ — 2026-08-06, log 58'in GERÇEK IMU'sundan atandı (§0.8l).
# Eski 5.0 hiçbir ölçüme dayanmıyordu.
# --------------------------------------------------------------------------- #

#: Log 58 (50 Hz, 270 s) ölçülen |a|/g — yerçekimi DAHİL.
_LOG58_OTONOM_MAKS = 1.067      # AUTO görevi boyunca en yüksek
_LOG58_TUM_MAKS = 1.474         # suya indirme + elle taşıma + manevra dahil


def test_sok_esigi_gercek_isletme_gurultusunun_USTUNDE() -> None:
    """Sahte tetik = koşu ölür → eşik ölçülen maksimumun en az 2 katı olmalı.

    Asimetri: sahte tetik P3'ü hedefe varmadan "tamamlandı" yapar, motorlar
    durur ve 145 puan gider. Kaçırılan darbe ise görevi öldürmez — tüm
    waypoint'ler bitince MissionFSM zaten TAMAMLANDI'ya geçiyor. Bu yüzden
    eşik CÖMERT tarafta olmalı.
    """
    import yaml

    yol = (
        Path(__file__).resolve().parents[2]
        / "ros2_ws" / "src" / "girdap_decision" / "config" / "params.yaml"
    )
    with open(yol, "r", encoding="utf-8") as f:
        cfg = yaml.safe_load(f)
    esik = float(cfg["fsm_node"]["ros__parameters"]["shock_threshold_g"])
    assert esik >= 2.0 * _LOG58_TUM_MAKS - 0.05, (
        f"şok eşiği {esik} g — log 58'de ölçülen maksimum {_LOG58_TUM_MAKS} g'nin "
        "2 katının altında; manevra/dalga SAHTE TETİK verebilir"
    )


def test_sok_esigi_SERT_carpismanin_erisebilecegi_yerde() -> None:
    """Eşik yalnız yüksek olmasın: gerçek bir sert çarpışma da geçebilmeli.

    1 m/s'lik temas 50 ms'de dururken ~2 g üretir; 100 ms'de ~1 g. Eşik
    4 g'nin üstüne çıkarsa hiçbir gerçekçi temas onu tetikleyemez ve kanal
    tamamen ölü olur (eski 5.0'ın durumu).
    ⚠ Yüzen dubaya çarpmanın kendisi zaten bu eşiğe ulaşmayabilir — duba
    yana savrulur, Δv küçük kalır. Kanal "sert çarpışma dedektörü"dür,
    P3 tamamlanma mekanizması DEĞİL (o waypoint'lerden sürülüyor).
    """
    import yaml

    yol = (
        Path(__file__).resolve().parents[2]
        / "ros2_ws" / "src" / "girdap_decision" / "config" / "params.yaml"
    )
    with open(yol, "r", encoding="utf-8") as f:
        cfg = yaml.safe_load(f)
    esik = float(cfg["fsm_node"]["ros__parameters"]["shock_threshold_g"])
    assert esik <= 4.0, f"şok eşiği {esik} g — gerçekçi hiçbir temas ulaşamaz"


def test_sok_esigi_node_varsayilani_yaml_ile_AYNI(ros_context) -> None:  # noqa: ANN001
    """Drift kapısı: yaml silinse bile node aynı değere düşmeli."""
    import yaml

    yol = (
        Path(__file__).resolve().parents[2]
        / "ros2_ws" / "src" / "girdap_decision" / "config" / "params.yaml"
    )
    with open(yol, "r", encoding="utf-8") as f:
        cfg = yaml.safe_load(f)
    node = girdap.FSMNode()
    try:
        assert float(node.get_parameter("shock_threshold_g").value) == float(
            cfg["fsm_node"]["ros__parameters"]["shock_threshold_g"]
        )
    finally:
        node.destroy_node()


# --------------------------------------------------------------------------- #
# KAR-03 (2026-08-12) — BOOT kilidi teşhisi.
# Kaptanın bag'inde sistem 25 DAKİKA BOOT'ta kaldı ve hiçbir yerde
# söylenmediği için "sahte yeşil" üretti. Aşağıdaki testler, teşhisin hem
# ÜRETİLDİĞİNİ hem de İKİ ARIZAYI AYIRT ETTİĞİNİ dondurur.
# --------------------------------------------------------------------------- #


def _boot_node(ros_context, tmp_path, esik=0.05):  # noqa: ANN001, ANN201
    return _make_node(
        ros_context, tmp_path,
        extra_params=[Parameter("boot_uyari_s", value=float(esik))],
    )


def test_KAR03_BOOTta_kisa_sure_HENUZ_uyarmaz(ros_context, tmp_path) -> None:  # noqa: ANN001
    """Normal açılış birkaç saniye BOOT'tadır — orada alarm basmak yanlış olur.

    Bekçinin değeri eşiğin ÜSTÜNDE; altında sessiz kalmazsa her açılışta
    yanlış alarm üretir ve operatör kısa sürede tüm uyarıları yok saymayı
    öğrenir (asıl arıza da o gürültüde kaybolur).
    """
    node = _boot_node(ros_context, tmp_path, esik=60.0)
    try:
        node._boot_kilidi_denetle(MissionState.BOOT)
        assert node._boot_teshis == ""
        assert node._boot_uyarildi is False
    finally:
        node.destroy_node()


def test_KAR03_mavros_HIC_yayin_yapmadiysa_MAVROS_YOK(ros_context, tmp_path) -> None:  # noqa: ANN001
    """`/mavros/state` hiç gelmedi → MAVROS düğümü koşmuyor / yanlış domain."""
    import time
    node = _boot_node(ros_context, tmp_path)
    try:
        time.sleep(0.1)
        node._boot_kilidi_denetle(MissionState.BOOT)
        assert node._boot_teshis == "MAVROS-YOK", (
            f"beklenen MAVROS-YOK, gelen {node._boot_teshis!r}"
        )
    finally:
        node.destroy_node()


def test_KAR03_mavros_var_ama_FCU_kopuksa_AYRI_teshis(ros_context, tmp_path) -> None:  # noqa: ANN001
    """🔴 Ayrımın bütün değeri burada: iki arızanın ÇÖZÜMÜ farklı.

    MAVROS-YOK → launch/servis/domain'e bak.
    FCU-KOPUK  → kablo/port/baud/Pixhawk gücüne bak.
    Tek bir "MAVROS yok" mesajı operatörü yanlış yere gönderirdi. Kaptanın
    bag'indeki gerçek arıza ikincisiydi ("disconnected" ×1.695).
    """
    import time
    node = _boot_node(ros_context, tmp_path)
    try:
        st = MavState()
        st.connected = False               # MAVROS ayakta, FCU hattı ölü
        node._on_mav_state(st)
        time.sleep(0.1)
        node._boot_kilidi_denetle(MissionState.BOOT)
        assert node._boot_teshis == "FCU-KOPUK", (
            f"beklenen FCU-KOPUK, gelen {node._boot_teshis!r}"
        )
    finally:
        node.destroy_node()


def test_KAR03_BOOTtan_cikilinca_teshis_TEMIZLENIR(ros_context, tmp_path) -> None:  # noqa: ANN001
    """Arıza geçtiyse ekranda asılı kalmamalı; sayaç da baştan başlamalı.

    md 5.5.3.1 yeniden başlama hakkı: reset sonrası tekrar BOOT'a düşülürse
    süre sıfırdan sayılmalı, yoksa ikinci koşuda anında yanlış alarm basar.
    """
    import time
    node = _boot_node(ros_context, tmp_path)
    try:
        time.sleep(0.1)
        node._boot_kilidi_denetle(MissionState.BOOT)
        assert node._boot_teshis != ""
        node._boot_kilidi_denetle(MissionState.ARM)      # bağlantı kuruldu
        assert node._boot_teshis == ""
        assert node._boot_uyarildi is False
        node._boot_kilidi_denetle(MissionState.BOOT)     # tekrar BOOT
        assert node._boot_teshis == "", "sayac sifirlanmadi — aninda alarm"
    finally:
        node.destroy_node()


def test_KAR03_teshis_OPERATOR_ekranina_gidiyor(ros_context, tmp_path) -> None:  # noqa: ANN001
    """🔴 En kritik test: teşhis ROS log'unda kalmamalı, YKİ'de GÖRÜNMELİ.

    Operatör sahada `ros2 topic echo` değil Mission Planner'a bakıyor.
    Yalnız "GIRDAP BOOT" yazmak ona hiçbir şey söylemez; sebep aynı satırda
    gitmeli ve NOTICE değil ERROR seviyesinde olmalı — yoksa MP mesaj
    akışında diğer satırların arasında kaybolur.
    """
    import time
    node = _boot_node(ros_context, tmp_path)
    try:
        yollanan = []
        if node._pub_statustext is None:
            pytest.skip("statustext kapalı")
        node._pub_statustext.get_subscription_count = lambda: 1
        node._pub_statustext.publish = lambda m: yollanan.append(m)

        time.sleep(0.1)
        node._boot_kilidi_denetle(MissionState.BOOT)
        node._publish_statustext(MissionState.BOOT)

        assert yollanan, "statustext hic yollanmadi"
        msg = yollanan[-1]
        assert "TAKILDI" in msg.text and "MAVROS-YOK" in msg.text, (
            f"operatore sebep gitmiyor: {msg.text!r}"
        )
        assert len(msg.text) <= 50, "MAVLink STATUSTEXT 50 karakterle sinirli"
        from mavros_msgs.msg import StatusText
        assert msg.severity == StatusText.ERROR, (
            "BOOT kilidi NOTICE seviyesinde — MP akisinda kaybolur"
        )
    finally:
        node.destroy_node()
