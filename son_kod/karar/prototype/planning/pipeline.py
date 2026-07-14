"""
Girdap İDA — Planlama Boru Hattı (ROS-bağımsız): RRT* global + MPPI lokal.

Layer 2 planning_node ve Layer 0 uçtan uca testleri ortak bu sınıfı kullanır.
ROS 2 mesaj tipleri yerine düz veri alır; pytest rclpy olmadan koşar.

Parkur bazlı davranış (CLAUDE.md MPPI + algoritma_tasarimlari.md §4.5, §4.7):
    PARKUR1 (Nokta Takip)  : w_track yüksek, w_obstacle düşük, sıkı takip
    PARKUR2 (Engelli Geçiş) : w_obstacle agresif (200), w_track gevşek
    PARKUR3 (Kamikaze)      : hedef Gaussian çekici (kamikaze_mode), engel
                              maliyetini ezer, w_track minimal

FSM durumu PARKUR1/2/3 dışında ise compute_control() None döndürür
(motor stop otoritesi FSM'de — Şartname 4.1 / 5.5.2.2).

Akış:
    set_state(...)          fusion smooth pose → durum vektörü
    set_waypoints(...)      görev hedefleri → RRT* replan
    set_obstacles(...)      perception engel listesi → replan tetiği
    set_mission_state(...)  FSM durumu → MPPI ağırlık profili değişimi
    compute_control()       20 Hz MPPI step → (T_left, T_right) veya None
"""

from __future__ import annotations

import logging
from dataclasses import dataclass, replace
from typing import Dict, List, Optional, Tuple

import numpy as np

_log = logging.getLogger(__name__)

from prototype.dynamics.catamaran import CatamaranDynamics
from prototype.planning.mppi import MPPIConfig, MPPIController
from prototype.planning.rrt_star import (
    Bounds,
    CircleObstacle,
    RRTStar,
    RRTStarConfig,
)

# Aracın hareket ettiği parkur durumları (bunların dışında motor stop)
_ACTIVE_STATES = ("PARKUR1", "PARKUR2", "PARKUR3")


@dataclass(frozen=True)
class ParkurProfile:
    """Bir parkura özgü MPPI ağırlık override'ları."""

    w_track: float
    w_obstacle: float
    w_terminal: float
    kamikaze_mode: bool = False
    w_kamikaze: float = 50.0


# CLAUDE.md ve algoritma_tasarimlari.md §4.5 tablosuyla birebir uyumlu.
# Parkur-3 kamikaze: hedef son waypoint'e Gaussian çekici; w_track minimal
# (referans takip zayıflar, hedefe kilitlenir).
_PARKUR_PROFILES: Dict[str, ParkurProfile] = {
    "PARKUR1": ParkurProfile(w_track=5.0, w_obstacle=50.0, w_terminal=5.0),
    "PARKUR2": ParkurProfile(w_track=3.0, w_obstacle=200.0, w_terminal=5.0),
    "PARKUR3": ParkurProfile(
        w_track=1.0, w_obstacle=50.0, w_terminal=5.0,
        kamikaze_mode=True, w_kamikaze=50.0,
    ),
}


@dataclass
class PlanningPipelineConfig:
    """Boru hattı ayarları — ROS 2 parametre arayüzünden aynı isimle gelir."""

    replan_proximity: float = 2.0        # m, RRT* replan tetiği
    # F10.2: RRT* örnekleme alanı statik bounds ∪ (start/goal ± bu pay) —
    # odom origin boot konumu olduğundan araç/hedef negatif çeyreğe düşebilir;
    # statik [0,200]² tek başına orada plan üretemez (start bounds dışı →
    # ValueError → F10.1 ölümü).
    bounds_margin_m: float = 30.0
    mppi_K: int = 1000
    mppi_T: int = 50
    mppi_dt: float = 0.05                # s (= 20 Hz kontrol adımı)
    ref_spacing: float = 0.5             # m, referans yeniden örnekleme
    # Yerel maliyet haritası (Şartname 4.2 Dosya-3) — araç merkezli pencere.
    map_width: int = 100                 # hücre
    map_height: int = 100                # hücre
    map_resolution: float = 0.5          # m/hücre → 50 m × 50 m pencere


@dataclass(frozen=True)
class LocalCostGrid:
    """Yerel maliyet haritası (araç merkezli, kuzey yukarı, ROS satır-major).

    `data`: int8, (height*width,), ROS OccupancyGrid konvansiyonu — satır 0
    güney (min y). Değerler: 0 serbest, 100 engel, -1 bilinmiyor (arena dışı).
    """

    data: np.ndarray
    width: int
    height: int
    resolution: float


class PlanningPipeline:
    """RRT* global + MPPI lokal planlayıcı — parkur bazlı ağırlık yönetimi."""

    def __init__(
        self,
        bounds: Bounds,
        cfg: Optional[PlanningPipelineConfig] = None,
        dynamics: Optional[CatamaranDynamics] = None,
        rrt_cfg: Optional[RRTStarConfig] = None,
    ) -> None:
        self._bounds = bounds
        self.cfg = cfg or PlanningPipelineConfig()
        self._dyn = dynamics or CatamaranDynamics()
        self._rrt_cfg = rrt_cfg or RRTStarConfig(use_informed=True)

        # Temel MPPI konfigürasyonu — parkur profili bunun üzerine biner
        self._base_mppi_cfg = MPPIConfig(
            K=self.cfg.mppi_K, T=self.cfg.mppi_T, dt=self.cfg.mppi_dt,
        )

        self._state = np.zeros(6)                    # [x, y, ψ, u, v, r]
        self._obstacles: List[CircleObstacle] = []
        self._waypoints: List[Tuple[float, float]] = []
        self._ref_path: Optional[List[Tuple[float, float]]] = None
        self._mission_state = "BOOT"

        self._mppi: Optional[MPPIController] = None   # referans gelince kurulur

    # ----- girdi setter'ları -----

    def set_state(self, state: np.ndarray) -> None:
        """[x, y, ψ, u, v, r] durum vektörünü güncelle."""
        self._state[:] = state

    def set_waypoints(self, waypoints: List[Tuple[float, float]]) -> None:
        """Görev hedeflerini ayarla ve global yörüngeyi (yeniden) planla."""
        self._waypoints = list(waypoints)
        if self._waypoints:
            self._global_replan()

    def set_reference_direct(self, target_x: float, target_y: float) -> None:
        """RRT* bypass (video modu) — mevcut konumdan hedefe düz çizgi referansı.

        Global planlama atlanır; referans yörünge = [mevcut poz → hedef]. MPPI
        bu düz çizgiyi horizon içinde örnekler ve engel kaçınmayı üstlenir.
        """
        sx, sy = float(self._state[0]), float(self._state[1])
        self._waypoints = [(float(target_x), float(target_y))]
        self._ref_path = [(sx, sy), (float(target_x), float(target_y))]
        self._rebuild_mppi()

    def set_obstacles(self, obstacles: List[CircleObstacle]) -> None:
        """
        Perception engel listesini güncelle.
        Yeni engel mevcut ref_path'e < replan_proximity ise RRT* yeniden koşar;
        aksi halde sadece MPPI engel listesi tazelenir.
        """
        if self._ref_path is not None and self._needs_replan(obstacles):
            self._obstacles = obstacles
            self._global_replan()
        else:
            self._obstacles = obstacles
            if self._mppi is not None:
                self._rebuild_mppi()

    def set_mission_state(self, state: str) -> None:
        """
        FSM durumunu ayarla. Parkur değiştiyse MPPI ağırlık profili değişir
        ve kontrolcü yeniden inşa edilir.
        """
        if state == self._mission_state:
            return
        prev = self._mission_state
        self._mission_state = state
        # Parkurlar arası geçişte ağırlık profili değişir
        if state in _PARKUR_PROFILES and prev != state and self._ref_path:
            self._rebuild_mppi()

    # ----- planlama iç mantığı -----

    def _needs_replan(self, new_obs: List[CircleObstacle]) -> bool:
        """ref_path'e replan_proximity + r kadar yakın yeni engel var mı?"""
        if self._ref_path is None:
            return False
        ref = np.asarray(self._ref_path)
        thr = self.cfg.replan_proximity
        for o in new_obs:
            d2 = (ref[:, 0] - o.cx) ** 2 + (ref[:, 1] - o.cy) ** 2
            if np.sqrt(d2.min()) < thr + o.r:
                return True
        return False

    def _global_replan(self) -> bool:
        """RRT* ile global yörüngeyi (start=mevcut poz, goal=son wp) hesapla.

        Başarısızlıkta (çözüm yok, start/goal engel payı içinde ya da alan
        dışında) mevcut `_ref_path` KORUNUR ve False döner — istisna asla
        dışarı sızmaz (F10.1: rclpy callback'inde yakalanmayan istisna
        planning_node'u görev ortasında öldürüyordu).
        """
        if not self._waypoints:
            return False
        start = (float(self._state[0]), float(self._state[1]))
        goal = self._waypoints[-1]
        # F10.2: örnekleme alanı = statik bounds ∪ (start/goal ± pay).
        # start/goal'in daima alan içinde kalmasını garanti eder.
        b, pay = self._bounds, self.cfg.bounds_margin_m
        bounds = Bounds(
            min(b.x_min, start[0] - pay, goal[0] - pay),
            max(b.x_max, start[0] + pay, goal[0] + pay),
            min(b.y_min, start[1] - pay, goal[1] - pay),
            max(b.y_max, start[1] + pay, goal[1] + pay),
        )
        rrt = RRTStar(bounds, self._obstacles, self._rrt_cfg)
        try:
            path = rrt.plan(start, goal)
        except ValueError as exc:
            _log.warning(
                "RRT* plan reddedildi (%s) — eski referans korunuyor", exc
            )
            return False
        if path is None:
            _log.warning("RRT* çözüm bulamadı — eski referans korunuyor")
            return False
        self._ref_path = path
        self._rebuild_mppi()
        return True

    def _active_mppi_cfg(self) -> MPPIConfig:
        """Mevcut parkur profilini temel MPPI config üzerine uygula."""
        profile = _PARKUR_PROFILES.get(self._mission_state)
        if profile is None:
            # Parkur dışı — temel config yeter (kontrol zaten yayınlanmayacak)
            return self._base_mppi_cfg
        kamikaze_target = (
            self._waypoints[-1]
            if (profile.kamikaze_mode and self._waypoints)
            else None
        )
        return replace(
            self._base_mppi_cfg,
            w_track=profile.w_track,
            w_obstacle=profile.w_obstacle,
            w_terminal=profile.w_terminal,
            kamikaze_mode=profile.kamikaze_mode,
            kamikaze_target=kamikaze_target,
            w_kamikaze=profile.w_kamikaze,
        )

    def _rebuild_mppi(self) -> None:
        """Referans/engel/parkur değiştiğinde MPPI'yi güncelle.

        Warm-start korunması (F11.1): kontrolcüyü HER çağrıda yeniden yaratmak
        U_nominal'i sıfırlar → soğuk başlangıç → zikzak (node 5-10 Hz çağırır).
        Bu yüzden:
          - Config (parkur ağırlık profili) AYNI ise → mevcut kontrolcüyü koru,
            yalnız engel + referansı güncelle (U_nominal yaşar).
          - Config DEĞİŞTİYSE (parkur geçişi) → yeni kontrolcü kur ama önceki
            U_nominal'i taşı (geçişte de soğuk başlangıç olmasın).
        """
        if self._ref_path is None:
            return
        new_cfg = self._active_mppi_cfg()
        if self._mppi is not None and new_cfg == self._mppi.cfg:
            self._mppi.set_obstacles(self._obstacles)
            self._mppi.set_reference(self._ref_path, spacing=self.cfg.ref_spacing)
            return
        prev_U = self._mppi.U_nominal.copy() if self._mppi is not None else None
        self._mppi = MPPIController(
            self._dyn, self._bounds, self._obstacles, new_cfg
        )
        self._mppi.set_reference(self._ref_path, spacing=self.cfg.ref_spacing)
        if prev_U is not None and prev_U.shape == self._mppi.U_nominal.shape:
            self._mppi.U_nominal[:] = prev_U

    # ----- kontrol -----

    def compute_control(self) -> Optional[np.ndarray]:
        """
        Tek MPPI step. Dönüş:
            (2,) [T_left, T_right] (N) — parkur aktif ve MPPI hazırsa
            None — FSM parkur dışı veya referans/kontrolcü henüz yok (motor stop)
        """
        if self._mission_state not in _ACTIVE_STATES:
            return None
        if self._mppi is None:
            return None
        return self._mppi.step(self._state)

    # ----- sorgu -----

    @property
    def global_path(self) -> Optional[List[Tuple[float, float]]]:
        return self._ref_path

    @property
    def mission_state(self) -> str:
        return self._mission_state

    @property
    def active_weights(self) -> Tuple[float, float, float, bool]:
        """(w_track, w_obstacle, w_terminal, kamikaze_mode) — test/log için."""
        c = self._active_mppi_cfg()
        return c.w_track, c.w_obstacle, c.w_terminal, c.kamikaze_mode

    def predicted_trajectory(self) -> Optional[np.ndarray]:
        """MPPI'nin son ağırlıklı-ortalama öngörü yörüngesi — (T+1, 2) dünya XY.

        Yalnız GÖRSELLEŞTİRME için (offline viz): en son compute_control()
        çağrısındaki K rollout'un softmax ağırlıklarıyla ortalaması. Kontrol
        mantığını değiştirmez, yeni algoritma değil — mevcut MPPI çıktısını
        (last_trajectories × last_weights) dışa açar. None: MPPI henüz koşmadı.
        """
        if self._mppi is None:
            return None
        trajs = self._mppi.last_trajectories       # (K, T+1, 6)
        weights = self._mppi.last_weights          # (K,)
        if trajs is None or weights is None:
            return None
        mean = np.tensordot(weights, trajs, axes=(0, 0))   # (T+1, 6)
        return mean[:, :2]

    # ----- yerel maliyet haritası (Dosya-3) -----

    def local_cost_grid(self) -> LocalCostGrid:
        """Araç merkezli yerel maliyet haritası (Şartname 4.2 Dosya-3).

        MPPI engel maliyet modeli (quadratic barrier `max(0, r_safe - d)²`)
        araç etrafındaki 50 m × 50 m pencerede değerlendirilip 0-100'e
        normalize edilir. Arena (bounds) dışı hücreler bilinmiyor (-1).
        Vektörize NumPy — 10 Hz yayım için ucuz.
        """
        w = self.cfg.map_width
        h = self.cfg.map_height
        res = self.cfg.map_resolution
        x0 = float(self._state[0])
        y0 = float(self._state[1])

        # Hücre merkezleri (dünya ENU). Satır 0 = güney (min y) — ROS konvansiyonu.
        origin_x = x0 - (w * res) / 2.0
        origin_y = y0 - (h * res) / 2.0
        cx = origin_x + (np.arange(w) + 0.5) * res           # (w,)
        cy = origin_y + (np.arange(h) + 0.5) * res           # (h,)
        gx, gy = np.meshgrid(cx, cy)                         # (h, w)

        # Engel maliyeti → occupancy [0,100]: engel içi (d ≤ r) kesin dolu (100);
        # emniyet halkasında (r < d ≤ r+margin) lineer 100→0; dışı serbest (0).
        # Birden çok engelde hücre başına maksimum alınır.
        occ = np.zeros((h, w), dtype=np.float64)             # 0 = serbest su
        if self._obstacles:
            margin = self._base_mppi_cfg.obstacle_margin
            for o in self._obstacles:
                d = np.hypot(gx - o.cx, gy - o.cy)
                if margin > 0.0:
                    contrib = np.where(
                        d <= o.r, 100.0,
                        np.where(
                            d <= o.r + margin,
                            100.0 * (o.r + margin - d) / margin,
                            0.0,
                        ),
                    )
                else:
                    contrib = np.where(d <= o.r, 100.0, 0.0)
                occ = np.maximum(occ, contrib)

        grid = np.rint(occ).astype(np.int16)                 # 0..100
        # Arena dışı → bilinmiyor (-1)
        b = self._bounds
        unknown = (gx < b.x_min) | (gx > b.x_max) | (gy < b.y_min) | (gy > b.y_max)
        grid[unknown] = -1

        return LocalCostGrid(
            data=grid.reshape(-1).astype(np.int8),           # ROS satır-major
            width=w,
            height=h,
            resolution=res,
        )
