"""
Girdap İDA — Planning bypass (RRT* atla, düz hedef → MPPI) testleri.

use_rrt=false modu: global planlama yerine current_target doğrudan MPPI
referansı olur (mevcut poz → hedef düz çizgi). MPPI'ın çalıştığını ve kontrol
(cmd_vel eşdeğeri thrust) ürettiğini doğrular.

Çalıştır: pytest prototype/tests/test_planning_bypass.py -v
"""

from __future__ import annotations

import numpy as np

from prototype.planning.pipeline import PlanningPipeline, PlanningPipelineConfig
from prototype.planning.rrt_star import Bounds


def _pipe() -> PlanningPipeline:
    cfg = PlanningPipelineConfig(mppi_K=200, mppi_T=30)   # test için hızlı
    return PlanningPipeline(Bounds(0.0, 200.0, 0.0, 200.0), cfg)


def test_direct_reference_is_straight_line() -> None:
    p = _pipe()
    p.set_state(np.array([10.0, 10.0, 0.0, 0.0, 0.0, 0.0]))
    p.set_reference_direct(30.0, 10.0)
    # RRT* atlandı → referans = [mevcut poz, hedef]
    assert p.global_path == [(10.0, 10.0), (30.0, 10.0)]


def test_bypass_mppi_produces_control() -> None:
    p = _pipe()
    p.set_state(np.array([10.0, 10.0, 0.0, 0.0, 0.0, 0.0]))
    p.set_mission_state("PARKUR1")            # görev aktif
    p.set_reference_direct(30.0, 10.0)
    u = p.compute_control()
    assert u is not None
    assert u.shape == (2,)
    assert np.all(np.isfinite(u))


def test_bypass_no_control_when_mission_inactive() -> None:
    p = _pipe()
    p.set_state(np.array([10.0, 10.0, 0.0, 0.0, 0.0, 0.0]))
    p.set_reference_direct(30.0, 10.0)
    # mission_state BOOT (parkur dışı) → motor stop (None)
    assert p.compute_control() is None


def test_repeated_reference_preserves_warmstart() -> None:
    """Node 5 Hz'te set_reference_direct çağırır → MPPI YENİDEN YARATILMAMALI.

    F11.1: her hedef tazelemesinde yeni MPPIController = warm-start (U_nominal)
    sıfırlanır = zikzak. Aynı config'de kontrolcü korunmalı, warm-start yaşamalı.
    """
    p = _pipe()
    p.set_state(np.array([10.0, 10.0, 0.0, 0.0, 0.0, 0.0]))
    p.set_mission_state("PARKUR1")
    p.set_reference_direct(40.0, 10.0)
    ctrl_first = p._mppi
    p.compute_control()                        # warm-start birikir
    warm = p._mppi.U_nominal.copy()
    assert np.any(warm != 0.0)                 # bir şey biriktiğini doğrula

    p.set_reference_direct(40.0, 10.0)         # node'un yaptığı gibi tazele
    assert p._mppi is ctrl_first               # AYNI kontrolcü (yeniden yaratılmadı)
    assert np.array_equal(p._mppi.U_nominal, warm)   # warm-start korundu


def test_parkur_change_carries_warmstart() -> None:
    """Parkur profili değişince (P1→P2) config değişir → yeni kontrolcü, ama
    warm-start taşınmalı (soğuk başlangıç zikzağı olmasın)."""
    p = _pipe()
    p.set_state(np.array([10.0, 10.0, 0.0, 0.0, 0.0, 0.0]))
    p.set_mission_state("PARKUR1")
    p.set_reference_direct(40.0, 10.0)
    p.compute_control()
    warm = p._mppi.U_nominal.copy()
    assert np.any(warm != 0.0)
    p.set_mission_state("PARKUR2")             # ağırlık profili değişir → rebuild
    assert np.array_equal(p._mppi.U_nominal, warm)   # warm-start taşındı


def test_bypass_closed_loop_moves_toward_target() -> None:
    """Kapalı döngü: düz hedef referansıyla araç +x'e (hedefe) ilerlemeli."""
    p = _pipe()
    state = np.array([10.0, 10.0, 0.0, 0.0, 0.0, 0.0])       # ψ=0 → +x'e bakıyor
    p.set_state(state)
    p.set_mission_state("PARKUR1")
    p.set_reference_direct(40.0, 10.0)        # 30 m ileride (RRT* bypass)
    dyn = p._dyn
    for _ in range(60):                       # ~3 s (dt=0.05)
        p.set_state(state)
        u = p.compute_control()
        assert u is not None
        state = dyn.step_rk4(state, u, 0.05)
    assert state[0] > 12.0                     # hedefe doğru ilerledi (10 → >12)
    assert abs(state[1] - 10.0) < 3.0          # hattan büyük sapma yok
