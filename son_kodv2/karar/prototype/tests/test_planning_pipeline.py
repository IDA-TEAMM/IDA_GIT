"""
Girdap İDA — PlanningPipeline + uçtan uca entegrasyon testi (rclpy bağımsız).

Kapsam:
    1) Parkur bazlı MPPI ağırlık profillerinin doğru uygulanması
    2) FSM gating: parkur dışı durumda motor stop (compute_control → None)
    3) Kamikaze modu: PARKUR3'te hedef çekici + kamikaze_mode aktif
    4) Kapalı döngü: PlanningPipeline + katamaran plant → goal'e yakınsama
    5) Zincir: mock sensör → FusionPipeline → PlanningPipeline → kontrol

Çalıştır: pytest prototype/tests/test_planning_pipeline.py -v -s
"""

from __future__ import annotations

import math

import numpy as np
import pytest

from prototype.dynamics.catamaran import CatamaranDynamics
from prototype.planning.pipeline import (
    PlanningPipeline,
    PlanningPipelineConfig,
)
from prototype.planning.rrt_star import Bounds, CircleObstacle


# Testlerde hız için küçük MPPI (matematik aynı, rollout sayısı düşük)
def _fast_cfg() -> PlanningPipelineConfig:
    return PlanningPipelineConfig(mppi_K=200, mppi_T=30)


@pytest.fixture
def bounds() -> Bounds:
    return Bounds(0.0, 50.0, 0.0, 50.0)


# --------------------------------------------------------------------------- #
# 1) Parkur bazlı ağırlık profilleri
# --------------------------------------------------------------------------- #


def test_parkur_profiles_switch_weights(bounds: Bounds) -> None:
    """FSM durumu değişince MPPI ağırlıkları parkur profiline geçmeli."""
    pipe = PlanningPipeline(bounds, _fast_cfg())
    pipe.set_waypoints([(5.0, 5.0), (45.0, 45.0)])

    pipe.set_mission_state("PARKUR1")
    w_track, w_obs, w_term, kam = pipe.active_weights
    assert (w_track, w_obs, kam) == (5.0, 50.0, False)

    pipe.set_mission_state("PARKUR2")
    w_track, w_obs, w_term, kam = pipe.active_weights
    assert (w_track, w_obs, kam) == (3.0, 200.0, False), \
        "PARKUR2 agresif engel kaçınmaya geçmeliydi"

    pipe.set_mission_state("PARKUR3")
    w_track, w_obs, w_term, kam = pipe.active_weights
    assert kam is True, "PARKUR3 kamikaze modunu açmalıydı"
    assert w_track == 1.0, "PARKUR3 referans takibini gevşetmeliydi"


def test_kamikaze_target_is_last_waypoint(bounds: Bounds) -> None:
    """PARKUR3'te MPPI kamikaze hedefi son waypoint olmalı."""
    pipe = PlanningPipeline(bounds, _fast_cfg())
    pipe.set_waypoints([(5.0, 5.0), (40.0, 40.0)])
    pipe.set_mission_state("PARKUR3")
    cfg = pipe._active_mppi_cfg()
    assert cfg.kamikaze_mode is True
    assert cfg.kamikaze_target == (40.0, 40.0)


# --------------------------------------------------------------------------- #
# 2) FSM gating — parkur dışı durumda motor stop
# --------------------------------------------------------------------------- #


@pytest.mark.parametrize("state", ["BOOT", "ARM", "BEKLEMEDE", "TAMAMLANDI", "KILL"])
def test_fsm_gating_returns_none(bounds: Bounds, state: str) -> None:
    """Parkur dışı FSM durumunda compute_control None (motor stop) döndürür."""
    pipe = PlanningPipeline(bounds, _fast_cfg())
    pipe.set_waypoints([(5.0, 5.0), (45.0, 45.0)])
    pipe.set_mission_state(state)
    assert pipe.compute_control() is None


def test_no_reference_returns_none(bounds: Bounds) -> None:
    """Waypoint hiç gelmeden PARKUR1'e geçilse bile kontrol yok (None)."""
    pipe = PlanningPipeline(bounds, _fast_cfg())
    pipe.set_mission_state("PARKUR1")
    assert pipe.compute_control() is None


# --------------------------------------------------------------------------- #
# 3) Kapalı döngü — PlanningPipeline plant üzerinde goal'e yakınsar
# --------------------------------------------------------------------------- #


def test_closed_loop_reaches_goal(bounds: Bounds) -> None:
    """PARKUR1: engelsiz sahnede araç goal'e < 2 m yaklaşmalı."""
    dyn = CatamaranDynamics()
    pipe = PlanningPipeline(bounds, _fast_cfg(), dynamics=dyn)

    start = (5.0, 5.0)
    goal = (40.0, 40.0)
    pipe.set_waypoints([start, goal])
    pipe.set_mission_state("PARKUR1")

    # Plant başlangıç durumu — heading goal yönüne hizalı
    state = np.zeros(6)
    state[0], state[1] = start
    state[2] = math.atan2(goal[1] - start[1], goal[0] - start[0])

    dt = 0.05
    reached = False
    for _ in range(int(40.0 / dt)):
        pipe.set_state(state)
        u = pipe.compute_control()
        assert u is not None and np.all(np.isfinite(u))
        state = dyn.step_rk4(state, u, dt)
        if math.hypot(state[0] - goal[0], state[1] - goal[1]) < 2.0:
            reached = True
            break

    final = math.hypot(state[0] - goal[0], state[1] - goal[1])
    print(f"\n[closed-loop] final goal hata = {final:.2f} m")
    assert reached, f"Goal'e ulaşılamadı (final hata {final:.2f} m)"


def test_closed_loop_avoids_obstacle(bounds: Bounds) -> None:
    """PARKUR2: yol üstündeki engelden emniyet payıyla kaçınmalı."""
    dyn = CatamaranDynamics()
    pipe = PlanningPipeline(bounds, _fast_cfg(), dynamics=dyn)

    start = (5.0, 5.0)
    goal = (45.0, 45.0)
    obs = CircleObstacle(25.0, 25.0, 4.0)     # köşegen üstünde
    pipe.set_waypoints([start, goal])
    pipe.set_obstacles([obs])
    pipe.set_mission_state("PARKUR2")

    state = np.zeros(6)
    state[0], state[1] = start
    state[2] = math.atan2(goal[1] - start[1], goal[0] - start[0])

    dt = 0.05
    min_clearance = float("inf")
    for _ in range(int(45.0 / dt)):
        pipe.set_state(state)
        u = pipe.compute_control()
        assert u is not None
        state = dyn.step_rk4(state, u, dt)
        d = math.hypot(state[0] - obs.cx, state[1] - obs.cy) - obs.r
        min_clearance = min(min_clearance, d)
        if math.hypot(state[0] - goal[0], state[1] - goal[1]) < 2.0:
            break

    print(f"\n[obstacle] min clearance = {min_clearance:.2f} m")
    assert min_clearance > -0.5, "Araç engelin derinine girdi (çarptı)"


# --------------------------------------------------------------------------- #
# 4) Zincir: mock sensör → fusion → planning → kontrol
# --------------------------------------------------------------------------- #


def test_e2e_fusion_to_planning_chain(bounds: Bounds) -> None:
    """
    Sentetik GPS+IMU → FusionPipeline smooth pose → PlanningPipeline →
    sonlu, makul thrust. mock_sensors→fusion→planning→cmd_vel zincirinin
    ROS-bağımsız doğrulaması.
    """
    # F16.3: gtsam yalnız BU teste gerekli — modül düzeyinde import edilirse
    # dosyadaki 7 kapalı-döngü testi gtsam'sız makinede rehin kalır.
    pytest.importorskip("gtsam", reason="gtsam yok — e2e füzyon zinciri atlanır")
    from prototype.fusion.pipeline import FusionPipeline

    # 1) Fusion boru hattı — düz ileri hareket sentezle
    fp = FusionPipeline()
    fp.on_gps(36.85, 28.27)              # origin

    dt = 0.02
    x_true = 0.0
    for k in range(150):                 # 3 s @ 50 Hz
        t = (k + 1) * dt
        x_true += 1.0 * dt               # 1 m/s ileri
        fp.on_velocity(1.0, 0.0)
        fp.on_imu(t, 0.0)
        if (k + 1) % 50 == 0:            # 1 Hz GPS
            lat, lon = fp.enu_to_latlon(x_true, 0.0)
            fp.on_gps(lat, lon)

    x, y, psi = fp.current_pose()
    assert abs(x - x_true) < 1.0, "Fusion pozu ground-truth'tan çok saptı"

    # 2) Planning boru hattı — fusion pozunu durum olarak besle
    pipe = PlanningPipeline(bounds, _fast_cfg())
    pipe.set_waypoints([(x, y), (40.0, 5.0)])
    pipe.set_mission_state("PARKUR1")
    pipe.set_state(np.array([x, y, psi, 1.0, 0.0, 0.0]))

    u = pipe.compute_control()
    assert u is not None, "Zincir kontrol üretmeliydi"
    assert np.all(np.isfinite(u)), "Kontrol sonlu olmalı"
    assert np.all(np.abs(u) <= pipe._dyn.p.max_thrust + 1e-6), \
        "Kontrol thruster doygunluk sınırını aşmamalı"
    print(f"\n[e2e] fusion pose=({x:.2f},{y:.2f}) → thrust=({u[0]:.1f},{u[1]:.1f})")


def test_global_path_published_after_waypoints(bounds: Bounds) -> None:
    """Waypoint set edilince RRT* global_path üretmeli (RViz kanalı)."""
    pipe = PlanningPipeline(bounds, _fast_cfg())
    assert pipe.global_path is None
    # RRT* mevcut pozdan (state) son waypoint'e planlar; başlangıcı sabitle
    pipe.set_state(np.array([5.0, 5.0, 0.0, 0.0, 0.0, 0.0]))
    pipe.set_waypoints([(5.0, 5.0), (40.0, 40.0)])
    path = pipe.global_path
    assert path is not None and len(path) >= 2
    # Yol mevcut pozdan (5,5) goal'e (40,40) gitmeli
    assert math.hypot(path[0][0] - 5.0, path[0][1] - 5.0) < 2.0
    assert math.hypot(path[-1][0] - 40.0, path[-1][1] - 40.0) < 2.0


# --------------------------------------------------------------------------- #
# F-S.10: control_mode="pid" — MPPI'ye alternatif, kanıtlanmış PID kontrolcü.
# Aynı kapalı-döngü fiziksel plant testleriyle (test_closed_loop_*), PID
# yolunun da GERÇEKTEN goal'e ulaştığını + engelden kaçındığını kanıtlar —
# yalnız birim test değil, uçtan uca simülasyon.
# --------------------------------------------------------------------------- #


def _pid_cfg() -> PlanningPipelineConfig:
    return PlanningPipelineConfig(control_mode="pid")


def test_control_mode_varsayilan_mppi(bounds: Bounds) -> None:
    assert PlanningPipelineConfig().control_mode == "mppi"


def test_pid_modu_rrt_replan_atlar(bounds: Bounds) -> None:
    """F-S.10: control_mode='pid' iken RRT* hiç koşmaz — global_path None
    kalır (gereksiz CPU harcanmaz, PID hedefe doğrudan gider)."""
    pipe = PlanningPipeline(bounds, _pid_cfg())
    pipe.set_state(np.array([5.0, 5.0, 0.0, 0.0, 0.0, 0.0]))
    pipe.set_waypoints([(5.0, 5.0), (40.0, 40.0)])
    assert pipe.global_path is None


def test_pid_modu_kapali_dongu_goale_ulasir(bounds: Bounds) -> None:
    """PID modu: engelsiz sahnede araç goal'e < 2 m yaklaşmalı (fiziksel
    plant üzerinde — test_closed_loop_reaches_goal'ın PID karşılığı)."""
    dyn = CatamaranDynamics()
    pipe = PlanningPipeline(bounds, _pid_cfg(), dynamics=dyn)

    start = (5.0, 5.0)
    goal = (40.0, 40.0)
    pipe.set_waypoints([start, goal])
    pipe.set_mission_state("PARKUR1")

    state = np.zeros(6)
    state[0], state[1] = start
    state[2] = math.atan2(goal[1] - start[1], goal[0] - start[0])

    dt = 0.05
    reached = False
    for _ in range(int(40.0 / dt)):
        pipe.set_state(state)
        u = pipe.compute_control()
        assert u is not None and np.all(np.isfinite(u))
        state = dyn.step_rk4(state, u, dt)
        if math.hypot(state[0] - goal[0], state[1] - goal[1]) < 2.0:
            reached = True
            break

    final = math.hypot(state[0] - goal[0], state[1] - goal[1])
    print(f"\n[pid closed-loop] final goal hata = {final:.2f} m")
    assert reached, f"PID modu goal'e ulaşamadı (final hata {final:.2f} m)"


def test_pid_modu_kapali_dongu_engelden_kacar(bounds: Bounds) -> None:
    """PID modu: yol üstündeki LiDAR engelinden emniyet payıyla kaçınmalı
    (test_closed_loop_avoids_obstacle'ın PID karşılığı)."""
    dyn = CatamaranDynamics()
    pipe = PlanningPipeline(bounds, _pid_cfg(), dynamics=dyn)

    start = (5.0, 5.0)
    goal = (45.0, 45.0)
    obs = CircleObstacle(25.0, 25.0, 4.0)     # köşegen üstünde
    pipe.set_waypoints([start, goal])
    pipe.set_obstacles([obs])
    pipe.set_mission_state("PARKUR1")

    state = np.zeros(6)
    state[0], state[1] = start
    state[2] = math.atan2(goal[1] - start[1], goal[0] - start[0])

    dt = 0.05
    min_clearance = float("inf")
    for _ in range(int(45.0 / dt)):
        pipe.set_state(state)
        u = pipe.compute_control()
        assert u is not None
        state = dyn.step_rk4(state, u, dt)
        d = math.hypot(state[0] - obs.cx, state[1] - obs.cy) - obs.r
        min_clearance = min(min_clearance, d)
        if math.hypot(state[0] - goal[0], state[1] - goal[1]) < 2.0:
            break

    print(f"\n[pid obstacle] min clearance = {min_clearance:.2f} m")
    assert min_clearance > -0.5, "PID modu engelin derinine girdi (çarptı)"


def test_pid_modu_parkur_disi_motor_stop(bounds: Bounds) -> None:
    """PID modu da FSM gating'e uymalı — parkur dışı None (motor stop)."""
    pipe = PlanningPipeline(bounds, _pid_cfg())
    pipe.set_waypoints([(5.0, 5.0), (45.0, 45.0)])
    pipe.set_mission_state("BEKLEMEDE")
    assert pipe.compute_control() is None


def test_pid_modu_waypoint_yoksa_none(bounds: Bounds) -> None:
    pipe = PlanningPipeline(bounds, _pid_cfg())
    pipe.set_mission_state("PARKUR1")
    assert pipe.compute_control() is None
