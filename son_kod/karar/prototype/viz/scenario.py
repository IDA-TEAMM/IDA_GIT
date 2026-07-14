"""
Girdap İDA — Offline 2D görselleştirici: senaryo motoru (Sprint 4.5).

Sentetik bir senaryoyu (dubalar + waypoint'ler + tekne) adım adım koşturur ve
her frame için bir durum anlık görüntüsü (FrameState) üretir. ROS GEREKTİRMEZ;
mevcut prototype çekirdeklerini DOĞRUDAN çağırır — yeni algoritma yazılmaz:

    synthetic_lidar.generate_buoy_points + lidar_obstacles.detect_obstacles
        → gerçek LiDAR clustering (body-frame sentetik nokta bulutu, frame'e
          göre seed'li → deterministik)
    fusion.associate                → LiDAR + kamera bearing füzyonu (renk sınıfı)
    parkur_fsm.ParkurTransitionLogic → waypoint-index parkur geçişi
    planning.PlanningPipeline       → yerel maliyet haritası (Dosya-3) + MPPI
                                       öngörü yörüngesi (opsiyonel)

Tekne HAREKETİ basit kinematiktir (aktif waypoint'e yönel, cruise hızla ilerle)
— gerçek MPPI kontrolü değil; görsel akış için yeterli (CLAUDE.md Sprint 4.5).
MPPI yörüngesi yalnız OVERLAY olarak (opsiyonel) çizdirilir.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import List, Optional, Tuple

import numpy as np

from prototype.mission.parkur_fsm import ParkurState, ParkurTransitionLogic
from prototype.perception.fusion import (
    CameraDetection,
    FusedObstacle,
    FusionConfig,
    LidarDetection,
    associate,
)
from prototype.perception.lidar_obstacles import (
    LidarObstacleConfig,
    detect_obstacles,
)
from prototype.perception.synthetic_lidar import BuoySpec, generate_buoy_points
from prototype.planning.pipeline import (
    LocalCostGrid,
    PlanningPipeline,
    PlanningPipelineConfig,
)
from prototype.planning.rrt_star import Bounds, CircleObstacle

# ParkurState → PlanningPipeline mission_state (MPPI ağırlık profili).
_PARKUR_TO_PIPELINE = {
    ParkurState.PARKUR_1: "PARKUR1",
    ParkurState.PARKUR_2: "PARKUR2",
    ParkurState.PARKUR_3: "PARKUR3",
    ParkurState.COMPLETED: "TAMAMLANDI",
}


@dataclass(frozen=True)
class Buoy:
    """Dünya çerçevesinde bir duba — gerçek (ground-truth) rengi ile."""

    x: float
    y: float
    true_class: int          # 0=parkur_kenari, 1=engel, 2=hedef
    radius: float = 0.15


@dataclass
class VizScenario:
    """Bir görselleştirme senaryosunun statik tanımı."""

    name: str
    buoys: List[Buoy]
    waypoints: List[Tuple[float, float]]     # dünya ENU
    parkur_labels: List[int]                 # her waypoint'in parkuru (1/2/3)
    boat_start: Tuple[float, float, float]   # (x, y, ψ)
    bounds: Bounds = field(default_factory=lambda: Bounds(-5.0, 45.0, -15.0, 15.0))
    cruise_velocity: float = 1.5             # m/s (görsel kinematik)
    arrival_radius: float = 1.5              # m, waypoint'e "varıldı" eşiği
    dt: float = 0.2                          # s, frame periyodu
    max_frames: int = 400                    # güvenlik üst sınırı
    lidar_range: float = 25.0                # m, LiDAR yatay menzil
    camera_range: float = 15.0               # m, kamera etkin menzil
    show_cost_map: bool = True
    show_mppi: bool = False                  # MPPI öngörü yörüngesi overlay


@dataclass
class FrameState:
    """Tek frame'in çizilebilir durum anlık görüntüsü."""

    t: float
    boat_x: float
    boat_y: float
    boat_psi: float
    trail: List[Tuple[float, float]]
    obstacles: List[FusedObstacle]           # DÜNYA çerçevesi, füzyon sınıflı
    waypoints: List[Tuple[float, float]]
    active_wp_index: int
    parkur_state: str
    mission_phase: str                       # ACTIVE / COMPLETE
    cost_grid: Optional[LocalCostGrid] = None
    mppi_traj: Optional[np.ndarray] = None   # (M, 2) dünya XY (öngörü/düz çizgi)


def _perceive_and_fuse(
    boat_x: float,
    boat_y: float,
    boat_psi: float,
    scenario: VizScenario,
    rng: np.random.Generator,
) -> List[FusedObstacle]:
    """Dünya dubalarını body-frame'e taşı → LiDAR clustering + kamera → füzyon.

    Gerçek çekirdekleri çalıştırır:
      - LiDAR: her menzildeki duba için sentetik nokta bulutu üret →
        detect_obstacles (cKDTree clustering) → LidarDetection.
      - Kamera: FOV içindeki dubalar için bearing→bbox_cx (kalibrasyonsuz
        projeksiyon), gerçek renk sınıfı. (Görüntü render'ı bypass — HSV/CLAHE
        birim testlerde doğrulanır; viz geometrik projeksiyon kullanır.)
      - associate: bearing eşleştirme → FusedObstacle (eşleşmeyen LiDAR = unknown).
    Çıktı DÜNYA çerçevesine geri döndürülür (çizim için).
    """
    fcfg = FusionConfig()
    cos_n, sin_n = math.cos(-boat_psi), math.sin(-boat_psi)   # dünya→body

    cloud_parts: List[np.ndarray] = []
    camera_dets: List[CameraDetection] = []
    for b in scenario.buoys:
        dx, dy = b.x - boat_x, b.y - boat_y
        xb = dx * cos_n - dy * sin_n                          # body ileri (+x)
        yb = dx * sin_n + dy * cos_n                          # body sol (+y)
        dist = math.hypot(xb, yb)
        if dist > scenario.lidar_range:
            continue
        # LiDAR: body-frame duba etrafında sentetik nokta bulutu.
        cloud_parts.append(
            generate_buoy_points(BuoySpec(x=xb, y=yb, radius=b.radius), rng)
        )
        # Kamera: yalnız ileri FOV + kamera menzili.
        bearing = math.atan2(yb, xb)
        if abs(bearing) <= fcfg.camera_hfov_rad / 2.0 and dist <= scenario.camera_range:
            camera_dets.append(
                CameraDetection(
                    # F6.1: sol (+bearing) → görüntünün sol yarısı (cx<0.5)
                    bbox_cx=0.5 - bearing / fcfg.camera_hfov_rad,
                    bbox_cy=0.5,
                    class_id=b.true_class,
                    score=0.9,
                )
            )

    if cloud_parts:
        cloud = np.vstack(cloud_parts)
        lidar_circles = detect_obstacles(cloud, LidarObstacleConfig())
    else:
        lidar_circles = []
    lidar_dets = [
        LidarDetection(x=c.center_x, y=c.center_y, radius=c.radius)
        for c in lidar_circles
    ]

    fused_body = associate(lidar_dets, camera_dets, fcfg)

    # body → dünya (çizim için).
    cos_p, sin_p = math.cos(boat_psi), math.sin(boat_psi)
    fused_world: List[FusedObstacle] = []
    for f in fused_body:
        xw = boat_x + f.x * cos_p - f.y * sin_p
        yw = boat_y + f.x * sin_p + f.y * cos_p
        fused_world.append(
            FusedObstacle(
                x=xw, y=yw, radius=f.radius,
                class_id=f.class_id, score=f.score, matched=f.matched,
            )
        )
    return fused_world


def _cost_and_mppi(
    cost_pipe: PlanningPipeline,
    pipe_cfg: PlanningPipelineConfig,
    boat_state: np.ndarray,
    world_obstacles: List[FusedObstacle],
    target: Tuple[float, float],
    parkur_state: ParkurState,
    scenario: VizScenario,
) -> Tuple[Optional[LocalCostGrid], Optional[np.ndarray]]:
    """Yerel maliyet haritası (Dosya-3) + opsiyonel MPPI öngörü yörüngesi.

    - Cost map: `cost_pipe` REFERANSSIZ kullanılır (yalnız set_state/obstacles →
      _ref_path None kalır → set_obstacles RRT replan TETİKLEMEZ). local_cost_grid
      ref_path'e ihtiyaç duymaz.
    - MPPI overlay: FRAME BAŞINA TAZE pipeline (durum kuplajı yok). set_obstacles
      referanstan ÖNCE (ref_path None → RRT yok); set_reference_direct düz çizgi
      MPPI'yi mevcut engellerle kurar. Yalnız scenario.show_mppi + aktif parkur.
    """
    circles = [
        CircleObstacle(cx=o.x, cy=o.y, r=o.radius) for o in world_obstacles
    ]
    cost_pipe.set_state(boat_state)
    cost_pipe.set_obstacles(circles)
    cost = cost_pipe.local_cost_grid() if scenario.show_cost_map else None

    mppi_traj: Optional[np.ndarray] = None
    pipe_state = _PARKUR_TO_PIPELINE.get(parkur_state, "PARKUR1")
    if scenario.show_mppi and pipe_state in ("PARKUR1", "PARKUR2", "PARKUR3"):
        mppi_pipe = PlanningPipeline(scenario.bounds, pipe_cfg)
        mppi_pipe.set_state(boat_state)
        mppi_pipe.set_obstacles(circles)         # ref_path None → RRT yok
        mppi_pipe.set_reference_direct(target[0], target[1])
        mppi_pipe.set_mission_state(pipe_state)
        mppi_pipe.compute_control()              # u0 atılır (tekne kinematik)
        mppi_traj = mppi_pipe.predicted_trajectory()
    return cost, mppi_traj


def run_scenario(scenario: VizScenario) -> List[FrameState]:
    """Senaryoyu koştur → frame frame FrameState listesi (deterministik)."""
    boat_x, boat_y, boat_psi = scenario.boat_start
    parkur = ParkurTransitionLogic(scenario.parkur_labels)
    # MPPI/cost için küçük, hızlı, seed'li config (deterministik overlay).
    pipe_cfg = PlanningPipelineConfig(mppi_K=150, mppi_T=25)
    cost_pipe = PlanningPipeline(scenario.bounds, pipe_cfg)   # referanssız (cost)

    frames: List[FrameState] = []
    trail: List[Tuple[float, float]] = [(boat_x, boat_y)]
    active_idx = 0
    complete = False

    for step in range(scenario.max_frames):
        rng = np.random.default_rng(1000 + step)     # frame'e göre deterministik
        t = step * scenario.dt

        # --- perception + füzyon (dünya çerçevesi) ---
        obstacles = _perceive_and_fuse(boat_x, boat_y, boat_psi, scenario, rng)

        # --- aktif hedef + basit kinematik ---
        if active_idx < len(scenario.waypoints):
            target = scenario.waypoints[active_idx]
            dx, dy = target[0] - boat_x, target[1] - boat_y
            dist = math.hypot(dx, dy)
            boat_psi = math.atan2(dy, dx)
            if dist <= scenario.arrival_radius:
                parkur.current_waypoint_reached(active_idx)
                active_idx += 1
                if active_idx >= len(scenario.waypoints):
                    # Son waypoint parkur-3 ise kamikaze çarpma → COMPLETED.
                    if parkur.current_parkur == 3:
                        parkur.confirm_impact()
                    complete = True
            else:
                step_len = min(scenario.cruise_velocity * scenario.dt, dist)
                boat_x += step_len * math.cos(boat_psi)
                boat_y += step_len * math.sin(boat_psi)
                trail.append((boat_x, boat_y))
        else:
            complete = True

        # --- cost map + MPPI overlay ---
        boat_state = np.array([boat_x, boat_y, boat_psi, 0.0, 0.0, 0.0])
        cur_target = (
            scenario.waypoints[min(active_idx, len(scenario.waypoints) - 1)]
            if scenario.waypoints else (boat_x, boat_y)
        )
        cost, mppi_traj = _cost_and_mppi(
            cost_pipe, pipe_cfg, boat_state, obstacles, cur_target,
            parkur.state, scenario
        )
        if mppi_traj is None and scenario.waypoints:      # düz çizgi fallback
            mppi_traj = np.array([[boat_x, boat_y], list(cur_target)])

        frames.append(
            FrameState(
                t=t, boat_x=boat_x, boat_y=boat_y, boat_psi=boat_psi,
                trail=list(trail), obstacles=obstacles,
                waypoints=list(scenario.waypoints),
                active_wp_index=min(active_idx, len(scenario.waypoints) - 1)
                if scenario.waypoints else 0,
                parkur_state=parkur.state.value,
                mission_phase="COMPLETE" if complete else "ACTIVE",
                cost_grid=cost, mppi_traj=mppi_traj,
            )
        )
        if complete:
            break
    return frames


# --------------------------------------------------------------------------- #
# Örnek senaryolar
# --------------------------------------------------------------------------- #


def scenario_parkur1() -> VizScenario:
    """Parkur-1: düz waypoint takibi, birkaç kenar dubası (turuncu)."""
    return VizScenario(
        name="parkur1",
        buoys=[
            Buoy(10.0, 3.0, true_class=0),
            Buoy(20.0, -3.0, true_class=0),
            Buoy(30.0, 3.0, true_class=0),
        ],
        waypoints=[(12.0, 0.0), (24.0, 0.0), (36.0, 0.0)],
        parkur_labels=[1, 1, 1],
        boat_start=(0.0, 0.0, 0.0),
    )


def scenario_parkur2() -> VizScenario:
    """Parkur-1→2 geçişi: engelli geçiş (sarı engeller), MPPI overlay.

    labels [1,2,2]: wp0 (parkur-1 son) → PARKUR_2; sonrası engel kaçınma.
    """
    return VizScenario(
        name="parkur2",
        buoys=[
            Buoy(8.0, 2.5, true_class=0),     # turuncu kenar
            Buoy(8.0, -2.5, true_class=0),
            Buoy(15.0, 1.5, true_class=1),    # sarı engel (yolda)
            Buoy(22.0, -1.5, true_class=1),
            Buoy(30.0, 2.5, true_class=0),
        ],
        waypoints=[(10.0, 0.0), (20.0, 0.0), (32.0, 0.0)],
        parkur_labels=[1, 2, 2],
        boat_start=(0.0, 0.0, 0.0),
        show_mppi=True,
    )


def scenario_fusion() -> VizScenario:
    """Tam parkur 1→2→3 + füzyon vitrini + kamikaze tamamlanma.

    labels [1,2,3]: wp0→PARKUR_2, wp1→PARKUR_3, wp2 (hedef) varış → impact →
    COMPLETED. Yan dubalar (FOV dışı) → unknown (gri) beklenir.
    """
    return VizScenario(
        name="fusion",
        buoys=[
            Buoy(12.0, 1.0, true_class=0),    # turuncu, FOV içi
            Buoy(18.0, -1.0, true_class=1),   # sarı, FOV içi
            Buoy(26.0, 0.0, true_class=2),    # hedef (kırmızı), FOV içi
            Buoy(6.0, 8.0, true_class=0),     # yanda: LiDAR görür, kamera görmez
            Buoy(6.0, -8.0, true_class=1),    # → unknown (gri) beklenir
        ],
        waypoints=[(14.0, 0.0), (20.0, 0.0), (26.0, 0.0)],
        parkur_labels=[1, 2, 3],
        boat_start=(0.0, 0.0, 0.0),
        show_mppi=True,
    )


#: run_viz.py --scenario <ad> eşlemesi.
SCENARIOS = {
    "parkur1": scenario_parkur1,
    "parkur2": scenario_parkur2,
    "fusion": scenario_fusion,
}
