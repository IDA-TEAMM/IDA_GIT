"""
Girdap İDA — MissionFSM + fsm_node senaryo testi (rclpy bağımsız).

fsm_node'un ROS glue mantığı ince: mesaj alanlarını Observation'a bağlar ve
MissionFSM.tick() çağırır. Bu test, node'un ürettiği gözlem dizisini birebir
taklit ederek FSM karar zincirini doğrular.

Kapsam:
    1) /girdap/mission/start → BEKLEMEDE→PARKUR1 (servis akışı)
    2) Waypoint yakınsaması (odom → son wp < 1.5 m) → PARKUR1→PARKUR2
    3) start yalnız BEKLEMEDE'de etkili (Şartname 4.1)
    4) /girdap/mission/kill her durumdan → KILL
    5) last_gate_passed çıktı türetimi (PARKUR3+ evresi)
    6) Uçtan uca görev dizisi BOOT→TAMAMLANDI

Çalıştır: pytest prototype/tests/test_mission_fsm.py -v
"""

from __future__ import annotations

import math

import pytest

from prototype.fsm.mission_fsm import MissionFSM, MissionState, Observation


# --------------------------------------------------------------------------- #
# Node glue yardımcıları — fsm_node ile birebir aynı hesap
# --------------------------------------------------------------------------- #


def _dist_to_last_wp(pose_xy, last_wp) -> float:
    """fsm_node._on_odom içindeki mesafe hesabının aynısı."""
    return math.hypot(pose_xy[0] - last_wp[0], pose_xy[1] - last_wp[1])


def _advance_to_beklemede(fsm: MissionFSM) -> None:
    """BOOT → ARM → BEKLEMEDE ortak ön koşul dizisi."""
    # mavros connected=True → boot_ok
    fsm.tick(Observation(boot_ok=True))
    assert fsm.state is MissionState.ARM
    # mavros armed=True → kill_switch_off
    fsm.tick(Observation(boot_ok=True, kill_switch_off=True))
    assert fsm.state is MissionState.BEKLEMEDE


# --------------------------------------------------------------------------- #
# 1) start servisi → PARKUR1
# --------------------------------------------------------------------------- #


def test_start_service_beklemede_to_parkur1() -> None:
    """/girdap/mission/start çağrısı BEKLEMEDE→PARKUR1 geçişi yapmalı."""
    fsm = MissionFSM()
    _advance_to_beklemede(fsm)

    # Servis callback'i: request_start() + bir sonraki tick
    fsm.request_start()
    fsm.tick(Observation(boot_ok=True, kill_switch_off=True))

    assert fsm.state is MissionState.PARKUR1
    # Geçmiş doğru gerekçeyi tutmalı
    assert fsm.history[-1][1] is MissionState.PARKUR1


# --------------------------------------------------------------------------- #
# 2) Waypoint yakınsaması → PARKUR2
# --------------------------------------------------------------------------- #


def test_waypoint_convergence_parkur1_to_parkur2() -> None:
    """Son waypoint'e < 1.5 m yaklaşınca PARKUR1→PARKUR2 geçmeli."""
    fsm = MissionFSM()
    _advance_to_beklemede(fsm)
    fsm.request_start()
    fsm.tick(Observation(boot_ok=True, kill_switch_off=True))
    assert fsm.state is MissionState.PARKUR1

    last_wp = (40.0, 40.0)

    # Uzaktayken PARKUR1'de kal
    d = _dist_to_last_wp((30.0, 30.0), last_wp)      # ~14.1 m
    fsm.tick(Observation(boot_ok=True, kill_switch_off=True,
                         dist_to_last_wp_p1=d))
    assert fsm.state is MissionState.PARKUR1

    # Eşiğin hemen üstünde hâlâ PARKUR1
    d = _dist_to_last_wp((41.2, 40.0), last_wp)      # 1.2 m < 1.5 → geçiş
    assert d < 1.5
    fsm.tick(Observation(boot_ok=True, kill_switch_off=True,
                         dist_to_last_wp_p1=d))
    assert fsm.state is MissionState.PARKUR2


def test_convergence_exact_threshold() -> None:
    """Mesafe tam eşikte (≤) geçiş tetiklenmeli (sınır koşulu)."""
    fsm = MissionFSM()
    _advance_to_beklemede(fsm)
    fsm.request_start()
    fsm.tick(Observation(boot_ok=True, kill_switch_off=True))
    fsm.tick(Observation(boot_ok=True, kill_switch_off=True,
                         dist_to_last_wp_p1=1.5))     # tam eşik
    assert fsm.state is MissionState.PARKUR2


def test_mission_complete_terminates_video(  ) -> None:
    """F12.2: video senaryosu (tek parkur, çarpma yok) mission_complete ile
    TAMAMLANDI'ya varmalı — kamikaze/IMU şoku olmadan."""
    fsm = MissionFSM()
    _advance_to_beklemede(fsm)
    fsm.request_start()
    fsm.tick(Observation(boot_ok=True, kill_switch_off=True))
    assert fsm.state is MissionState.PARKUR1

    # Görev yöneticisi tüm waypoint'leri bitirdi
    fsm.tick(Observation(boot_ok=True, kill_switch_off=True,
                         mission_complete=True))
    assert fsm.state is MissionState.TAMAMLANDI


def test_mission_complete_precedes_spurious_parkur2() -> None:
    """mission_complete, dist tabanlı PARKUR1→PARKUR2 geçişinden ÖNCE
    değerlendirilmeli (görev bitince spurious P2'ye kaçmasın)."""
    fsm = MissionFSM()
    _advance_to_beklemede(fsm)
    fsm.request_start()
    fsm.tick(Observation(boot_ok=True, kill_switch_off=True))
    # Aynı anda hem yakın mesafe (P2 tetiği) HEM görev tamam → TAMAMLANDI kazanmalı
    fsm.tick(Observation(boot_ok=True, kill_switch_off=True,
                         dist_to_last_wp_p1=0.0, mission_complete=True))
    assert fsm.state is MissionState.TAMAMLANDI


def test_mission_complete_from_parkur2_also_terminates() -> None:
    """Yarışmada P2'deyken tüm waypoint biterse de TAMAMLANDI (genel terminal)."""
    fsm = MissionFSM()
    _advance_to_beklemede(fsm)
    fsm.request_start()
    fsm.tick(Observation(boot_ok=True, kill_switch_off=True))
    fsm.tick(Observation(boot_ok=True, kill_switch_off=True,
                         dist_to_last_wp_p1=1.0))      # P1→P2
    assert fsm.state is MissionState.PARKUR2
    fsm.tick(Observation(boot_ok=True, kill_switch_off=True,
                         mission_complete=True))
    assert fsm.state is MissionState.TAMAMLANDI


# --------------------------------------------------------------------------- #
# 3) start yalnız BEKLEMEDE'de
# --------------------------------------------------------------------------- #


def test_start_ignored_outside_beklemede() -> None:
    """BOOT durumundayken request_start() PARKUR1'e atlatmamalı."""
    fsm = MissionFSM()
    fsm.request_start()                               # erken/geçersiz istek
    fsm.tick(Observation(boot_ok=True))               # BOOT→ARM (start yutulmaz)
    assert fsm.state is MissionState.ARM
    # start bayrağı BEKLEMEDE'ye kadar beklemede kalır; ARM→BEKLEMEDE sonrası
    # ilk tick'te PARKUR1'e geçer (bayrak hâlâ set)
    fsm.tick(Observation(boot_ok=True, kill_switch_off=True))
    assert fsm.state is MissionState.BEKLEMEDE
    fsm.tick(Observation(boot_ok=True, kill_switch_off=True))
    assert fsm.state is MissionState.PARKUR1


# --------------------------------------------------------------------------- #
# 4) kill her durumdan
# --------------------------------------------------------------------------- #


@pytest.mark.parametrize("target", ["ARM", "BEKLEMEDE", "PARKUR1", "PARKUR2"])
def test_kill_from_any_state(target: str) -> None:
    """kill servisi çağrıldığı her durumdan KILL'e götürmeli."""
    fsm = MissionFSM()
    fsm.tick(Observation(boot_ok=True))               # ARM
    if target in ("BEKLEMEDE", "PARKUR1", "PARKUR2"):
        fsm.tick(Observation(boot_ok=True, kill_switch_off=True))
    if target in ("PARKUR1", "PARKUR2"):
        fsm.request_start()
        fsm.tick(Observation(boot_ok=True, kill_switch_off=True))
    if target == "PARKUR2":
        fsm.tick(Observation(boot_ok=True, kill_switch_off=True,
                             dist_to_last_wp_p1=1.0))
    assert fsm.state.value == target

    # kill servisi
    fsm.kill("YKİ kill servisi")
    fsm.tick(Observation())
    assert fsm.state is MissionState.KILL


# --------------------------------------------------------------------------- #
# 5) last_gate_passed çıktı türetimi
# --------------------------------------------------------------------------- #


def test_last_gate_passed_output_derivation() -> None:
    """last_gate_passed sadece PARKUR3+ evresinde True olmalı."""
    fsm = MissionFSM()
    assert fsm.last_gate_passed is False              # BOOT
    _advance_to_beklemede(fsm)
    assert fsm.last_gate_passed is False              # BEKLEMEDE
    fsm.request_start()
    fsm.tick(Observation(boot_ok=True, kill_switch_off=True))       # PARKUR1
    assert fsm.last_gate_passed is False
    fsm.tick(Observation(boot_ok=True, kill_switch_off=True,
                         dist_to_last_wp_p1=1.0))                   # PARKUR2
    assert fsm.last_gate_passed is False
    fsm.tick(Observation(boot_ok=True, kill_switch_off=True,
                         last_gate_passed_p2=True))                 # PARKUR3
    assert fsm.state is MissionState.PARKUR3
    assert fsm.last_gate_passed is True


# --------------------------------------------------------------------------- #
# 6) Uçtan uca görev dizisi
# --------------------------------------------------------------------------- #


def test_full_mission_sequence() -> None:
    """BOOT → ARM → BEKLEMEDE → P1 → P2 → P3 → TAMAMLANDI tam zincir."""
    fsm = MissionFSM()
    on = dict(boot_ok=True, kill_switch_off=True)

    fsm.tick(Observation(boot_ok=True))               # ARM
    fsm.tick(Observation(**on))                        # BEKLEMEDE
    fsm.request_start()
    fsm.tick(Observation(**on))                        # PARKUR1
    fsm.tick(Observation(**on, dist_to_last_wp_p1=1.0))  # PARKUR2
    fsm.tick(Observation(**on, last_gate_passed_p2=True))  # PARKUR3
    fsm.tick(Observation(**on, shock_detected_p3=True))    # TAMAMLANDI

    assert fsm.state is MissionState.TAMAMLANDI
    # Geçiş zinciri sırası doğru olmalı
    chain = [new.value for _, new, _ in fsm.history]
    assert chain == [
        "ARM", "BEKLEMEDE", "PARKUR1", "PARKUR2", "PARKUR3", "TAMAMLANDI",
    ]
