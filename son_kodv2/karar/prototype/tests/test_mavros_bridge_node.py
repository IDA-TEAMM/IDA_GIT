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

from mavros_msgs.msg import State as MavState            # noqa: E402

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
