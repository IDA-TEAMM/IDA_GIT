"""
Girdap İDA — mavros_bridge_node güvenlik izleme testleri (F-M.2).

Masa olayı (2026-07-12, donanim_gunlugu §10): gerçek FCU'da KASITLI disarm
("DISARM başarılı" logu) hemen ardından "FAILSAFE — beklenmedik disarm →
KILL" üretti. F14.2'nin `_expected_disarm` bayrağı TEK ATIMLIK; `_on_monitor`
sonundaki `_was_armed = _was_armed or armed` latch'i ise disarm kenarını her
tick yeniden "görür" → 2. tick'te bayrak tüketilmiş olduğundan sahte KILL.

rclpy + mavros_msgs gerektirir (girdap_deps_ws source'lu) → yoksa SKIP.
"""

from __future__ import annotations

import pytest

rclpy = pytest.importorskip("rclpy", reason="rclpy yok (.venv) — ROS ortamında koş")
mavros_msgs = pytest.importorskip(
    "mavros_msgs", reason="mavros_msgs yok — girdap_deps_ws source'la"
)

from mavros_msgs.msg import RCIn                          # noqa: E402
from mavros_msgs.msg import State as MavState              # noqa: E402

girdap = pytest.importorskip(
    "girdap_decision.mavros_bridge_node",
    reason="girdap_decision source'lanmamış (ros2_ws/install/setup.bash)",
)


@pytest.fixture(scope="module")
def ros_context():                                       # noqa: ANN201
    rclpy.init()
    yield
    rclpy.shutdown()


def _state(*, armed: bool) -> MavState:
    msg = MavState()
    msg.connected = True
    msg.armed = armed
    msg.guided = False
    msg.mode = "MANUAL"                # görev aktif değil → auto_guided devrede değil
    return msg


def test_fm2_komutlu_disarm_sonraki_ticklerde_kill_uretmez(ros_context) -> None:  # noqa: ANN001
    """Kasıtlı disarm SONRASI monitor tick'leri sahte KILL üretmemeli.

    Masa senaryosu birebir: ARM → komutlu disarm → kenar tick'i (beklenen,
    KILL yok) → SONRAKİ tick'ler (burada sahte KILL basılıyordu).
    """
    n = girdap.MavrosBridgeNode()
    try:
        # 1) ARM'lı durum gözlendi.
        n._on_state(_state(armed=True))
        n._on_monitor()
        assert n._killed is False

        # 2) Komutlu disarm (F14.2 bayrağı) + FCU disarm'ı raporladı.
        n._bridge.note_command_disarm()
        n._on_state(_state(armed=False))
        n._on_monitor()                          # kenar: beklenen → KILL yok
        assert n._killed is False

        # 3) Sonraki tick'ler — durum hâlâ disarm. Sahte KILL OLMAMALI.
        for _ in range(3):
            n._on_state(_state(armed=False))     # heartbeat taze
            n._on_monitor()
        assert n._killed is False, "kasıtlı disarm sonrası sahte FAILSAFE/KILL"
    finally:
        n.destroy_node()


def test_fm2_gercek_beklenmedik_disarm_hala_kill(ros_context) -> None:  # noqa: ANN001
    """Regresyon bekçisi: KOMUTSUZ arm→disarm hâlâ failsafe KILL üretmeli."""
    n = girdap.MavrosBridgeNode()
    try:
        n._on_state(_state(armed=True))
        n._on_monitor()
        assert n._killed is False

        n._on_state(_state(armed=False))         # komut YOK → gerçek failsafe
        n._on_monitor()
        assert n._killed is True
    finally:
        n.destroy_node()


def test_fm2_yeniden_arm_sonrasi_kenar_yine_izlenir(ros_context) -> None:  # noqa: ANN001
    """Disarm→yeniden ARM→komutsuz disarm: kenar tazelenir, failsafe yakalanır."""
    n = girdap.MavrosBridgeNode()
    try:
        n._on_state(_state(armed=True))
        n._on_monitor()
        n._bridge.note_command_disarm()
        n._on_state(_state(armed=False))
        n._on_monitor()                          # beklenen disarm ✓
        assert n._killed is False

        n._on_state(_state(armed=True))          # operatör yeniden arm etti
        n._on_monitor()
        n._on_state(_state(armed=False))         # bu kez KOMUTSUZ düştü
        n._on_monitor()
        assert n._killed is True                 # gerçek failsafe yakalandı
    finally:
        n.destroy_node()


def test_fm3_fsm_kill_gozlenince_fcu_disarm_tetiklenir(ros_context) -> None:  # noqa: ANN001
    """F-M.3: operatör kill'i (fsm servisi) bridge'e FCU disarm yaptırmalı.

    Masa bulgusu (2026-07-14, Oturum 2 / M6a): `/girdap/mission/kill` sonrası
    FSM=KILL + thrust [0,0] ama FCU ARMED kaldı — disarm yalnız bridge'in
    kendi `_trigger_kill` yolundaydı. Bridge `/girdap/mission/state`'te KILL
    gözleyince `_trigger_kill()` çağırmalı (disarm + latch).
    """
    from std_msgs.msg import String

    n = girdap.MavrosBridgeNode()
    try:
        calls = []
        orijinal = n._trigger_kill
        n._trigger_kill = lambda: (calls.append(1), orijinal())[1]  # type: ignore[method-assign]

        n._on_state(_state(armed=True))
        n._on_monitor()
        assert n._killed is False

        # FSM KILL yayınladı (operatör servisi fsm_node'dan geçti).
        n._on_mission_state(String(data="KILL"))
        assert calls, "KILL gözlendi ama _trigger_kill çağrılmadı (F-M.3)"
        assert n._killed is True, "kill latch'i set edilmedi"

        # İkinci KILL mesajı yeniden tetiklemez (idempotent).
        n._on_mission_state(String(data="KILL"))
        assert len(calls) == 1

        # Disarm kenarı sahte FAILSAFE üretmez (_killed erken dönüşü).
        n._on_state(_state(armed=False))
        n._on_monitor()
    finally:
        n.destroy_node()


def test_fm3_kill_disi_stateler_tetiklemez(ros_context) -> None:  # noqa: ANN001
    """Regresyon bekçisi: PARKUR1/BEKLEMEDE gibi state'ler disarm tetiklemez."""
    from std_msgs.msg import String

    n = girdap.MavrosBridgeNode()
    try:
        n._on_state(_state(armed=True))
        for s in ("BOOT", "ARM", "BEKLEMEDE", "PARKUR1", "TAMAMLANDI"):
            n._on_mission_state(String(data=s))
        assert n._killed is False
    finally:
        n.destroy_node()


# ----- F-M.6: FC akış hızı (SR0) isteği -----
#
# Masa Oturum 2 (2026-07-14): taze USB bağlantısında FC ~1 Hz yayınlıyor; elle
# /mavros/set_stream_rate (id 0, 50) ile 39 Hz alındı ama istek GEÇİCİ — boot
# yolunda kimse istemiyordu. Sonuç: Ekran-2 grafikleri 1 Hz basamaklı (md
# 3.3.1.1 "görüntü net değilse BAŞARISIZ") + fusion pose_timeout_s=1.0 bekçisi
# 1 Hz akışı "bayat" sayıp odom'u kesiyor + MPPI bayat pozla plan yapıyor.
# Köprü artık bağlantı kurulur kurulmaz hızı İSTİYOR (FC EEPROM'una YAZMADAN).


class _FakeClient:
    """service_is_ready=True olan sahte servis istemcisi (istekleri toplar)."""

    def __init__(self) -> None:
        self.requests: list = []

    def service_is_ready(self) -> bool:
        return True

    def call_async(self, req):                           # noqa: ANN001, ANN201
        self.requests.append(req)
        return rclpy.task.Future()


def _connected_state(*, connected: bool = True) -> MavState:
    msg = MavState()
    msg.connected = connected
    msg.armed = False
    msg.guided = False
    msg.mode = "HOLD"
    return msg


def test_stream_rate_baglantida_bir_kez_istenir(ros_context) -> None:  # noqa: ANN001
    """Bağlantı kenarında TEK istek (stream_id=0, on_off=True); 1 Hz spam yok."""
    n = girdap.MavrosBridgeNode()
    try:
        fake = _FakeClient()
        n._cli_stream = fake

        for _ in range(5):                       # 1 Hz state akışı
            n._on_state(_connected_state())

        assert len(fake.requests) == 1, "her state'te istek → 1 Hz spam"
        req = fake.requests[0]
        assert req.stream_id == 0                # STREAM_ALL
        assert req.on_off is True
        assert req.message_rate == n._stream_rate_hz > 1
    finally:
        n.destroy_node()


def test_stream_rate_yeniden_baglantida_tekrar_istenir(ros_context) -> None:  # noqa: ANN001
    """Seri kopup geri gelirse FC hızı sıfırlanır → istek TEKRARLANMALI."""
    n = girdap.MavrosBridgeNode()
    try:
        fake = _FakeClient()
        n._cli_stream = fake

        n._on_state(_connected_state())
        n._on_state(_connected_state(connected=False))
        n._on_state(_connected_state())

        assert len(fake.requests) == 2
    finally:
        n.destroy_node()


def test_stream_rate_sifirsa_devre_disi(ros_context) -> None:  # noqa: ANN001
    """stream_rate_hz=0 → istek gönderilmez (FC paramları elle yönetiliyorsa)."""
    from rclpy.parameter import Parameter

    n = girdap.MavrosBridgeNode(
        parameter_overrides=[
            Parameter("stream_rate_hz", Parameter.Type.INTEGER, 0)
        ]
    )
    try:
        fake = _FakeClient()
        n._cli_stream = fake
        n._on_state(_connected_state())
        assert fake.requests == []
    finally:
        n.destroy_node()


# --------------------------------------------------------------------------- #
# F-M.7 — boot/restart yarışı: FC hiç bağlanmadan heartbeat-KILL yok
# --------------------------------------------------------------------------- #


def _state_conn(*, connected: bool, armed: bool = False) -> MavState:
    msg = MavState()
    msg.connected = connected
    msg.armed = armed
    msg.guided = False
    msg.mode = "MANUAL"
    return msg


def test_fm7_fc_gorulmeden_heartbeat_kill_yok(ros_context) -> None:  # noqa: ANN001
    """Restart yarışı birebir (journal 2026-07-14 18:13): mavros connected=false
    state bastı, port devri >5 sn state boşluğu yarattı → KILL LATCH'LENMEMELİ.
    """
    n = girdap.MavrosBridgeNode()
    try:
        t = {"now": 0.0}
        n._now = lambda: t["now"]                 # sahte saat
        n._on_state(_state_conn(connected=False))
        t["now"] = 6.0                            # 5 s bekçi penceresi aşıldı
        n._on_monitor()
        assert n._killed is False, "FC hiç görülmeden heartbeat-KILL (F-M.7)"
    finally:
        n.destroy_node()


def test_fm7_baglanti_sonrasi_kayip_hala_kill(ros_context) -> None:  # noqa: ANN001
    """Regresyon bekçisi (M6d): FC bir kez bağlandıktan sonra 5 sn'lik state
    boşluğu ESKİSİ GİBİ heartbeat-KILL üretmeli.
    """
    n = girdap.MavrosBridgeNode()
    try:
        t = {"now": 0.0}
        n._now = lambda: t["now"]
        n._on_state(_state_conn(connected=True))
        t["now"] = 6.0
        n._on_monitor()
        assert n._killed is True, "bağlantı sonrası heartbeat kaybı KILL üretmeli"
    finally:
        n.destroy_node()


# --------------------------------------------------------------------------- #
# F-S.1 — RC donanım kill-switch
# --------------------------------------------------------------------------- #
#
# girdap_decision şimdiye kadar yalnız yazılım/servis KILL yollarını
# biliyordu (heartbeat, beklenmedik disarm, fsm servisi) — ida_topics/
# control_node.py'deki gibi companion computer'dan bağımsız bir RC donanım
# anahtarı hiç izlenmiyordu. Bu testler o boşluğu kapatan _on_rc_in'i
# doğrular.


def _rc(channels: list[int]) -> RCIn:
    msg = RCIn()
    msg.channels = channels
    return msg


def test_fs1_rc_kill_kanali_esik_alti_disarm_tetikler(ros_context) -> None:  # noqa: ANN001
    n = girdap.MavrosBridgeNode()
    try:
        calls = []
        orijinal = n._trigger_kill
        n._trigger_kill = lambda: (calls.append(1), orijinal())[1]  # type: ignore[method-assign]

        # 8 kanal, index 7 (RC kanal 8) = 900 → eşiğin (1500) altı → KILL.
        n._on_rc_in(_rc([1500, 1500, 1500, 1500, 1500, 1500, 1500, 900]))
        assert calls, "RC kill kanalı eşik altı ama _trigger_kill çağrılmadı (F-S.1)"
        assert n._killed is True
    finally:
        n.destroy_node()


def test_fs1_rc_kill_kanali_esik_ustu_tetiklemez(ros_context) -> None:  # noqa: ANN001
    n = girdap.MavrosBridgeNode()
    try:
        n._on_rc_in(_rc([1500] * 7 + [1800]))
        assert n._killed is False
    finally:
        n.destroy_node()


def test_fs1_rc_kanal_eksikse_tetiklemez(ros_context) -> None:  # noqa: ANN001
    """Kısa/eksik RCIn mesajı (kanal hiç yok) sahte KILL üretmemeli."""
    n = girdap.MavrosBridgeNode()
    try:
        n._on_rc_in(_rc([1500, 1500]))          # yalnız 2 kanal, index 7 yok
        assert n._killed is False
    finally:
        n.destroy_node()


def test_fs1_zaten_killed_iken_tekrar_tetiklemez(ros_context) -> None:  # noqa: ANN001
    """İdempotent: bir kez KILL olduktan sonra RC callback'i sessiz kalır."""
    n = girdap.MavrosBridgeNode()
    try:
        calls = []
        orijinal = n._trigger_kill
        n._trigger_kill = lambda: (calls.append(1), orijinal())[1]  # type: ignore[method-assign]

        n._on_rc_in(_rc([1500] * 7 + [900]))
        assert len(calls) == 1
        n._on_rc_in(_rc([1500] * 7 + [800]))     # hâlâ eşik altı
        assert len(calls) == 1, "zaten killed iken _trigger_kill tekrar çağrıldı"
    finally:
        n.destroy_node()


# --------------------------------------------------------------------------- #
# F-S.4: RC manuel-override
# --------------------------------------------------------------------------- #
#
# ida_topics/control_node.py'deki manual_override'ın karşılığı yoktu — pilot
# görev aktifken RC'den manuel moda geçmek isterse yazılım GUIDED için kavga
# ediyordu. Bu testler _on_rc_manual_check'in bridge state'ini doğru
# güncellediğini ve needs_mode_change()'i bastırdığını doğrular.


def test_fs4_rc_manual_esik_ustu_override_aktif_eder(ros_context) -> None:  # noqa: ANN001
    n = girdap.MavrosBridgeNode()
    try:
        # index 4 (RC kanal 5) = 1900 → eşiğin (1700) üstü → override aktif.
        n._on_rc_in(_rc([1500, 1500, 1500, 1500, 1900, 1500, 1500, 1500]))
        assert n._bridge.rc_manual_override is True
    finally:
        n.destroy_node()


def test_fs4_rc_manual_esik_alti_override_pasif(ros_context) -> None:  # noqa: ANN001
    n = girdap.MavrosBridgeNode()
    try:
        n._on_rc_in(_rc([1500, 1500, 1500, 1500, 1200, 1500, 1500, 1500]))
        assert n._bridge.rc_manual_override is False
    finally:
        n.destroy_node()


def test_fs4_manual_override_needs_mode_change_bastirir(ros_context) -> None:  # noqa: ANN001
    """Görev aktifken bile manuel override → yazılım GUIDED istemeyi bırakır."""
    from std_msgs.msg import String

    n = girdap.MavrosBridgeNode()
    try:
        n._on_mission_state(String(data="PARKUR1"))
        n._on_state(_state(armed=True))          # mode="MANUAL" != hedef GUIDED
        assert n._bridge.needs_mode_change() is True

        n._on_rc_in(_rc([1500, 1500, 1500, 1500, 1900, 1500, 1500, 1500]))
        assert n._bridge.needs_mode_change() is False, (
            "RC manuel-override aktifken yazılım hâlâ GUIDED istiyor (F-S.4)"
        )

        n._on_rc_in(_rc([1500, 1500, 1500, 1500, 1200, 1500, 1500, 1500]))
        assert n._bridge.needs_mode_change() is True   # override kalkınca eski hâl
    finally:
        n.destroy_node()


# --------------------------------------------------------------------------- #
# F-P.15 (robustness taraması, 2026-07-15) — sıkışmış set_mode isteği
# --------------------------------------------------------------------------- #


def test_fp15_sikismis_mode_istegi_zaman_asimiyla_kurtarir(ros_context) -> None:  # noqa: ANN001
    """_mode_req_pending yalnız /mavros/set_mode'un done-callback'inde
    temizlenirdi — future hiç sonuçlanmazsa (mavros restart/hang) bayrak
    SONSUZA DEK True kalır, _maybe_auto_guided() bir daha GUIDED istemezdi.
    Artık `mode_req_timeout_s` sonra sıkışmış sayılıp sıfırlanır."""
    n = girdap.MavrosBridgeNode()
    try:
        t = {"now": 0.0}
        n._now = lambda: t["now"]

        # İstek "gönderildi" ama callback HİÇ gelmedi (future asılı kaldı).
        n._mode_req_pending = True
        n._mode_req_sent_t = 0.0

        t["now"] = 2.0                            # timeout (5.0s) içinde
        assert n._mode_req_stuck() is False, "erken zaman aşımı tetiklendi"

        t["now"] = 6.0                            # 5.0s eşiği aşıldı
        assert n._mode_req_stuck() is True, (
            "sıkışmış set_mode isteği süresiz beklemede kalıyor (F-P.15)"
        )

        # _maybe_auto_guided çağrısı bayrağı temizlemeli (yeniden deneme açılsın).
        n._maybe_auto_guided()
        assert n._mode_req_pending is False, (
            "sıkışmış istek _maybe_auto_guided sonrası hâlâ True (F-P.15)"
        )
    finally:
        n.destroy_node()


# --------------------------------------------------- F-P.24: link bekçisi

def _diag(endpoint_name: str, remotes_count: int):  # noqa: ANN201
    from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue

    msg = DiagnosticArray()
    status = DiagnosticStatus()
    status.name = f"mavros_router: {endpoint_name}"
    status.values.append(KeyValue(key="Remotes count", value=str(remotes_count)))
    msg.status.append(status)
    return msg


def test_fp24_yuksek_remotes_count_uyari_basar(ros_context) -> None:  # noqa: ANN001
    """F-P.24 (2026-07-17): 2026-07-16 gerçek donanım testinde "Remotes
    count" 174'e/323'e çıktığı ELLE (/diagnostics manuel echo) bulundu —
    artık otomatik izlenip GÜRÜLTÜLÜ uyarı basılmalı."""
    n = girdap.MavrosBridgeNode()
    try:
        errors: list[str] = []
        n.get_logger().error = lambda msg, **kw: errors.append(msg)  # type: ignore[method-assign]

        n._on_diagnostics(_diag("endpoint 1000: serial:///dev/ttyUSB0:57600", 174))

        assert len(errors) == 1
        assert "Remotes count" in errors[0] and "174" in errors[0]
        assert "LINK ANORMALLİĞİ" in errors[0]
    finally:
        n.destroy_node()


def test_fp24_saglikli_remotes_count_uyari_basmaz(ros_context) -> None:  # noqa: ANN001
    n = girdap.MavrosBridgeNode()
    try:
        errors: list[str] = []
        n.get_logger().error = lambda msg, **kw: errors.append(msg)  # type: ignore[method-assign]

        n._on_diagnostics(_diag("endpoint 1000: serial:///dev/ttyUSB0:57600", 3))

        assert errors == []
    finally:
        n.destroy_node()


def test_fp24_normale_donunce_bir_kez_bilgi_loglar_tekrar_uyarmaz(ros_context) -> None:  # noqa: ANN001
    """Uyarı tekrar tekrar (her /diagnostics mesajında) spam yapmamalı —
    yalnız anormal→normal ve normal→anormal GEÇİŞLERİNDE loglanmalı."""
    n = girdap.MavrosBridgeNode()
    try:
        errors: list[str] = []
        n.get_logger().error = lambda msg, **kw: errors.append(msg)  # type: ignore[method-assign]

        n._on_diagnostics(_diag("endpoint 1000: serial:///dev/ttyUSB0:57600", 174))
        n._on_diagnostics(_diag("endpoint 1000: serial:///dev/ttyUSB0:57600", 200))
        n._on_diagnostics(_diag("endpoint 1000: serial:///dev/ttyUSB0:57600", 174))
        assert len(errors) == 1, "aynı anormallik tekrar tekrar loglanmamalı (spam)"

        n._on_diagnostics(_diag("endpoint 1000: serial:///dev/ttyUSB0:57600", 2))
        n._on_diagnostics(_diag("endpoint 1000: serial:///dev/ttyUSB0:57600", 174))
        assert len(errors) == 2, "normale dönüp tekrar bozulunca yeniden uyarmalı"
    finally:
        n.destroy_node()
