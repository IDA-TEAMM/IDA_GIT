"""
Girdap İDA — MavrosBridge güvenlik/mod geçidi çekirdeği testleri.

Doğrulanan davranışlar (ROS'suz, saf mantık):
    - Heartbeat zaman aşımı → KILL
    - FCU bağlantısı yok → KILL
    - armed=False → DISARMED (thrust sıfır)
    - mode != GUIDED → NOT_GUIDED (cmd_vel yayınlama yok)
    - Hepsi iyi → ACTIVE (tam yetki)
    - needs_mode_change mantığı

Çalıştır: pytest prototype/tests/test_mavros_bridge.py -v
"""

from __future__ import annotations

import math

import pytest

from prototype.control.mavros_bridge import (
    ControlGate,
    GateState,
    MavrosBridge,
    MavrosBridgeConfig,
)


def _active_bridge(timeout: float = 5.0) -> MavrosBridge:
    """t=0'da tam operasyonel (armed + GUIDED + connected) köprü."""
    b = MavrosBridge(MavrosBridgeConfig(heartbeat_timeout_s=timeout))
    b.update_state(0.0, connected=True, armed=True, guided=True, mode="GUIDED")
    return b


# --------------------------------------------------------------------------- #
# Heartbeat
# --------------------------------------------------------------------------- #


def test_no_state_is_infinite_and_kill() -> None:
    b = MavrosBridge()
    assert math.isinf(b.seconds_since_update(10.0))
    assert not b.heartbeat_alive(10.0)
    gate = b.control_gate(10.0)
    assert gate.state is GateState.KILL
    assert gate.zero_thrust and not gate.allow_cmd_vel


def test_heartbeat_alive_within_timeout() -> None:
    b = _active_bridge(timeout=5.0)
    assert b.heartbeat_alive(4.9)
    assert b.control_gate(4.9).state is GateState.ACTIVE


def test_heartbeat_expired_beyond_timeout() -> None:
    b = _active_bridge(timeout=5.0)
    assert not b.heartbeat_alive(5.01)
    gate = b.control_gate(5.01)
    assert gate.state is GateState.KILL
    assert gate.zero_thrust and not gate.allow_cmd_vel


def test_heartbeat_exact_boundary_is_alive() -> None:
    # Sınır dahil (<=): tam timeout'ta hâlâ canlı sayılır.
    b = _active_bridge(timeout=5.0)
    assert b.heartbeat_alive(5.0)
    assert b.control_gate(5.0).state is GateState.ACTIVE


# --------------------------------------------------------------------------- #
# Gate durumları
# --------------------------------------------------------------------------- #


def test_not_connected_is_kill() -> None:
    b = MavrosBridge()
    b.update_state(0.0, connected=False, armed=True, guided=True, mode="GUIDED")
    gate = b.control_gate(0.1)
    assert gate.state is GateState.KILL
    assert gate.zero_thrust and not gate.allow_cmd_vel


def test_disarmed_zeros_thrust() -> None:
    b = MavrosBridge()
    b.update_state(0.0, connected=True, armed=False, guided=True, mode="GUIDED")
    gate = b.control_gate(0.1)
    assert gate.state is GateState.DISARMED
    assert gate.zero_thrust is True
    assert gate.allow_cmd_vel is False


def test_not_guided_blocks_cmd_vel_but_not_thrust() -> None:
    b = MavrosBridge()
    b.update_state(0.0, connected=True, armed=True, guided=False, mode="MANUAL")
    gate = b.control_gate(0.1)
    assert gate.state is GateState.NOT_GUIDED
    assert gate.allow_cmd_vel is False
    assert gate.zero_thrust is False          # armed → thrust zorla sıfırlanmaz


def test_active_allows_everything() -> None:
    b = _active_bridge()
    gate = b.control_gate(0.1)
    assert gate.state is GateState.ACTIVE
    assert gate.allow_cmd_vel is True
    assert gate.zero_thrust is False


def test_priority_heartbeat_over_arm() -> None:
    # Hem heartbeat ölmüş hem disarmed → heartbeat (KILL) kazanır.
    b = MavrosBridge(MavrosBridgeConfig(heartbeat_timeout_s=1.0))
    b.update_state(0.0, connected=True, armed=False, guided=True, mode="GUIDED")
    gate = b.control_gate(5.0)
    assert gate.state is GateState.KILL


# --------------------------------------------------------------------------- #
# Beklenen (komutlu) disarm — F14.2
# --------------------------------------------------------------------------- #


def test_uncommanded_disarm_is_failsafe() -> None:
    """Komut verilmeden arm→disarm görülürse failsafe (True)."""
    b = MavrosBridge()
    assert b.is_unexpected_disarm(was_armed=True, now_armed=False) is True


def test_commanded_disarm_is_not_failsafe() -> None:
    """note_command_disarm sonrası arm→disarm beklenen (False) — video güç kesme."""
    b = MavrosBridge()
    b.note_command_disarm()
    assert b.is_unexpected_disarm(was_armed=True, now_armed=False) is False


def test_expected_disarm_flag_is_single_shot() -> None:
    """Beklenen disarm bayrağı bir kez tüketilir; sonraki disarm yine failsafe."""
    b = MavrosBridge()
    b.note_command_disarm()
    assert b.is_unexpected_disarm(True, False) is False   # tüketildi
    assert b.is_unexpected_disarm(True, False) is True    # artık failsafe


def test_no_transition_is_not_disarm() -> None:
    """arm→arm veya disarm→disarm geçiş değil → failsafe değil."""
    b = MavrosBridge()
    assert b.is_unexpected_disarm(was_armed=True, now_armed=True) is False
    assert b.is_unexpected_disarm(was_armed=False, now_armed=False) is False


# --------------------------------------------------------------------------- #
# needs_mode_change
# --------------------------------------------------------------------------- #


def test_needs_mode_change_true_when_not_guided() -> None:
    """Görev AKTİFKEN mod hedeften saparsa GUIDED'e dönüş istenir."""
    b = MavrosBridge()
    b.set_mission_state("PARKUR1")
    b.update_state(0.0, connected=True, armed=True, guided=False, mode="HOLD")
    assert b.needs_mode_change() is True


def test_needs_mode_change_false_when_guided() -> None:
    b = _active_bridge()
    b.set_mission_state("PARKUR1")
    assert b.needs_mode_change() is False


def test_needs_mode_change_false_when_not_connected() -> None:
    b = MavrosBridge()
    b.set_mission_state("PARKUR1")
    b.update_state(0.0, connected=False, armed=False, guided=False, mode="HOLD")
    assert b.needs_mode_change() is False


def test_needs_mode_change_false_without_state() -> None:
    assert MavrosBridge().needs_mode_change() is False


# --------------------------------------------------------------------------- #
# F14.3 — auto-GUIDED yalnız görev aktifken (md 3.3.1/3 manuel dönüş)
# --------------------------------------------------------------------------- #


def test_auto_guided_inactive_before_mission() -> None:
    """Görev başlamadan (BOOT/ARM/BEKLEMEDE) mod zorlanmaz — operatör
    aracı RC ile başlangıç noktasına manuel sürebilmeli."""
    b = MavrosBridge()
    b.update_state(0.0, connected=True, armed=False, guided=False, mode="MANUAL")
    assert b.needs_mode_change() is False          # hiç durum bildirilmedi
    for s in ("BOOT", "ARM", "BEKLEMEDE"):
        b.set_mission_state(s)
        assert b.needs_mode_change() is False, s


def test_auto_guided_active_during_parkur() -> None:
    """PARKUR1/2/3'te mod saparsa GUIDED'e geri çekilir (otonomi korunur)."""
    for s in ("PARKUR1", "PARKUR2", "PARKUR3"):
        b = MavrosBridge()
        b.set_mission_state(s)
        b.update_state(
            0.0, connected=True, armed=True, guided=False, mode="MANUAL"
        )
        assert b.needs_mode_change() is True, s


def test_auto_guided_released_after_mission() -> None:
    """md 3.3.1/3: görev tamamlanınca manuel dönüş serbest — köprü RC'nin
    MANUAL seçimiyle kavga etmemeli. KILL'de de zorlama olmamalı."""
    b = MavrosBridge()
    b.set_mission_state("PARKUR1")
    b.update_state(0.0, connected=True, armed=True, guided=False, mode="MANUAL")
    assert b.needs_mode_change() is True           # görev sürüyor
    for s in ("TAMAMLANDI", "KILL"):
        b.set_mission_state(s)
        assert b.needs_mode_change() is False, s


# --------------------------------------------------------------------------- #
# Yardımcı erişimler
# --------------------------------------------------------------------------- #


def test_state_accessors() -> None:
    b = _active_bridge()
    assert b.is_armed() is True
    assert b.current_mode() == "GUIDED"
    assert b.last_state is not None
    # Yeni state eskisini değiştirir
    b.update_state(3.0, connected=True, armed=False, guided=False, mode="HOLD")
    assert b.is_armed() is False
    assert b.current_mode() == "HOLD"
    assert b.seconds_since_update(3.5) == pytest.approx(0.5)


def test_custom_target_mode() -> None:
    b = MavrosBridge(MavrosBridgeConfig(target_mode="OFFBOARD"))
    b.update_state(0.0, connected=True, armed=True, guided=True, mode="OFFBOARD")
    assert b.control_gate(0.1).state is GateState.ACTIVE
    assert b.needs_mode_change() is False


def test_planning_gate_expects_configured_mode_auto() -> None:
    """planning_node mode_name='AUTO' → geçit AUTO bekler, GUIDED reddeder.

    planning_node MavrosBridge'i mode_name param'ıyla kurar (GUIDED hardcoded
    değil); config'ten gelen mod aktif kabul edilmeli.
    """
    b = MavrosBridge(MavrosBridgeConfig(target_mode="AUTO"))
    b.update_state(0.0, connected=True, armed=True, guided=True, mode="AUTO")
    assert b.control_gate(0.1).state is GateState.ACTIVE       # AUTO → aktif
    b.update_state(0.0, connected=True, armed=True, guided=True, mode="GUIDED")
    assert b.control_gate(0.1).state is GateState.NOT_GUIDED   # GUIDED beklenmiyor


# ----- SR0 akış hızı isteği (F-M.6) — bağlantı kenarı -----
#
# Gerçek FC davranışı (masa Oturum 2, 2026-07-14): taze bağlantıda ArduPilot
# ~1 Hz yayınlıyor; elle set_stream_rate ile 39 Hz'e çıkıyor. Köprü bağlantı
# kurulur kurulmaz akış hızını İSTEMELİ — ama her state mesajında değil (1 Hz
# spam), yalnız bağlantının YÜKSELEN KENARINDA ve her yeniden bağlanışta.


def test_stream_rate_state_gelmeden_istenmez() -> None:
    """Hiç /mavros/state gelmediyse istek yok (FCU yok, servis de yok)."""
    b = MavrosBridge()
    assert b.should_request_stream_rate() is False


def test_stream_rate_ilk_baglantida_istenir() -> None:
    b = MavrosBridge()
    b.update_state(0.0, connected=True, armed=False, guided=False, mode="HOLD")
    assert b.should_request_stream_rate() is True


def test_stream_rate_baglanti_yokken_istenmez() -> None:
    b = MavrosBridge()
    b.update_state(0.0, connected=False, armed=False, guided=False, mode="HOLD")
    assert b.should_request_stream_rate() is False


def test_stream_rate_bir_kez_istenir_sonra_susar() -> None:
    """İstek gönderildikten sonra 1 Hz'lik state akışında TEKRAR istenmez."""
    b = MavrosBridge()
    b.update_state(0.0, connected=True, armed=False, guided=False, mode="HOLD")
    b.note_stream_rate_requested()
    assert b.should_request_stream_rate() is False
    b.update_state(1.0, connected=True, armed=False, guided=False, mode="HOLD")
    assert b.should_request_stream_rate() is False


def test_stream_rate_yeniden_baglantida_tekrar_istenir() -> None:
    """USB/seri kopup geri gelirse FC akış hızı sıfırlanır → yeniden iste."""
    b = MavrosBridge()
    b.update_state(0.0, connected=True, armed=False, guided=False, mode="HOLD")
    b.note_stream_rate_requested()
    b.update_state(1.0, connected=False, armed=False, guided=False, mode="HOLD")
    b.update_state(2.0, connected=True, armed=False, guided=False, mode="HOLD")
    assert b.should_request_stream_rate() is True


# --------------------------------------------------------------------------- #
# RC donanım kill-switch — F-S.1
# --------------------------------------------------------------------------- #


def test_rc_kill_esik_alti_pwm_aktif() -> None:
    b = MavrosBridge(MavrosBridgeConfig(rc_kill_threshold_pwm=1500))
    assert b.is_rc_kill_active(900) is True


def test_rc_kill_esik_ustu_pwm_pasif() -> None:
    b = MavrosBridge(MavrosBridgeConfig(rc_kill_threshold_pwm=1500))
    assert b.is_rc_kill_active(1800) is False


def test_rc_kill_esik_tam_sinirda_pasif() -> None:
    # <, <= değil: eşiğin TAM üstünde/eşit değeri kill saymaz (ida_topics
    # control_node.py'deki RC_KILL_THRESHOLD karşılaştırmasıyla aynı yön).
    b = MavrosBridge(MavrosBridgeConfig(rc_kill_threshold_pwm=1500))
    assert b.is_rc_kill_active(1500) is False


def test_rc_kill_none_pwm_pasif() -> None:
    """Kanal hiç gelmediyse (mesaj kısa/eksik) kill SAYILMAZ."""
    b = MavrosBridge()
    assert b.is_rc_kill_active(None) is False


def test_rc_kill_sifir_pwm_pasif() -> None:
    """PWM=0 (alıcı/kanal kaybı) geçersiz sayılır, kill'e YOL AÇMAZ."""
    b = MavrosBridge()
    assert b.is_rc_kill_active(0) is False


def test_rc_kill_ozel_esik_config() -> None:
    b = MavrosBridge(MavrosBridgeConfig(rc_kill_threshold_pwm=1000))
    assert b.is_rc_kill_active(1200) is False
    assert b.is_rc_kill_active(900) is True
