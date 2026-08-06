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
import math
from dataclasses import dataclass, replace
from typing import Dict, List, Optional, Tuple

import numpy as np

_log = logging.getLogger(__name__)

from prototype.dynamics.catamaran import CatamaranDynamics
from prototype.planning.mppi import MPPIConfig, MPPIController
from prototype.planning.pid_controller import (
    CascadeHeadingPidController,
    PidControllerConfig,
)
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
    # Softmax sıcaklığı — parkur başına ayarlanır (ölçümler CLAUDE.md "λ").
    # λ küçük → tek rollout kararı belirler (ESS≈1, sert/gürültülü);
    # λ büyük → ağırlıklı ortalama yumuşar ama engel tepkisi zayıflar.
    lambda_: float = 10.0


# CLAUDE.md ve algoritma_tasarimlari.md §4.5 tablosuyla birebir uyumlu.
# Parkur-3 kamikaze: hedef son waypoint'e Gaussian çekici; w_track minimal
# (referans takip zayıflar, hedefe kilitlenir).
# w_terminal 5.0 → 50.0 (2026-08-02, F-M.3 lookahead benimsemesi): terminal
# hedefi artık rotanın sonu değil çapadan 15 m ileride; gradyan 2·w·d hedef
# uzaklığıyla orantılı olduğundan w telafi edilmezse ileri sürüş çöker
# (ölçüm: 5.64 → 1.07 m/s, hedefe varamıyor). ÜÇÜ BİRDEN değişir.
#
# λ 1.0 → 10.0 (2026-08-02): λ=1 softmax'ı dejenere ediyordu — ESS 2.6/1000
# (p5=1.0), yani adımların en az %5'inde MPPI ağırlıklı ortalama YAPMIYOR,
# tek rastgele örneği seçiyordu. λ=10: ESS 176 (p5 9.2), adımlar arası
# |Δu₀| RMS 9.95 → 3.44 N (2.9× yumuşak); iz sapması/açıklık/goal/hız
# değişmiyor. λ≥500'de araç hedefe hiç varamıyor (ortalama maliyet farklarını
# siler) — üst sınır orası.
# PARKUR3 λ=50 (ayrı ölçüldü): kamikaze çekicisi maliyet yayılımını büyüttüğü
# için λ=10'da ESS p5 hâlâ 1.9 (dejenere). λ=50 → p5 112, |Δu₀| 4.52 → 2.87 N
# ve en önemlisi **temas hızı 1.18 → 1.81 m/s (+%53)**: parkur-3'ün bitişi IMU
# şok eşiğiyle (shock_threshold_g=3.0, log 58'den ölçüldü) algılandığı için çarpma enerjisi görev
# tamamlama güvenilirliğidir. Bedel: yaklaşma %13 daha uzun (230 → 260 adım).
#
# 🔴 λ YENİDEN ÖLÇÜLDÜ (2026-08-06, GIRDAP_DURUM §0.8): P1/P2 10.0 → **1.0**,
# P3 50.0 → **10.0**. Yukarıdaki 02.08 gerekçesi 30 N/motor'luk HAYALİ tekneye
# aitti; log 58 tanılamasıyla aktüatör 1.455 N'a inince tablo tersine döndü —
# ağırlıklı ortalama (büyük λ) kontrolü nominale çekiyor ve tekne sürüklenmeyi
# yenemiyor. Ölçüm: 405 koşumluk σ×λ ızgarası + 162 koşumluk bozucu taraması
# (bozucu = ölçülen itkinin %30 ve %50'si), 3 sahne × 3 tohum.
#   P1/P2 (σ_u=0.485): λ=1 → 9/9 · slalom 62.5 s · bozucu %50'de 64.9 s
#                      λ=3 → 9/9 ama bozucuda 94-178 s (2-3× yavaş)
#                      λ=10 → bozucuda **6/9 VARAMADI**
#   P3 kamikaze: λ=50 → 3/3 ama TEMAS HIZI 0.134-0.154 m/s (47 s)
#                λ=10 → 3/3, temas 0.556-0.604 m/s (**4×**), 30.6 s
#                λ=1/3 → 2/3 (bir tohumda araç 0.13 m/s'e takılıyor)
# ⚠ P3'te büyük λ'nın hâlâ küçük λ'dan İYİ olması tesadüf değil: kamikaze
# çekicisi maliyet ölçeğini büyütür → λ maliyet ölçeğiyle birlikte seçilir.
# 🔴 Temas hızı P3'ün BİTİŞ ŞARTI: görev sonu IMU şokuyla algılanıyor
# (fsm_node shock_threshold_g=3.0) — 0.14 m/s'lik temas şok ÜRETMEZ.
# ⚠ Görev sonu buna BAĞLI DEĞİL: tüm waypoint'ler bitince MissionFSM
# zaten TAMAMLANDI'ya geçiyor; şok kanalı sert-çarpışma dedektörü.
_PARKUR_PROFILES: Dict[str, ParkurProfile] = {
    "PARKUR1": ParkurProfile(w_track=5.0, w_obstacle=50.0, w_terminal=50.0,
                             lambda_=1.0),
    "PARKUR2": ParkurProfile(w_track=3.0, w_obstacle=200.0, w_terminal=50.0,
                             lambda_=1.0),
    "PARKUR3": ParkurProfile(
        w_track=1.0, w_obstacle=50.0, w_terminal=50.0,
        kamikaze_mode=True, w_kamikaze=50.0, lambda_=10.0,
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
    # F-S.10: yerel kontrolcü seçimi — "mppi" (varsayılan, LiDAR cost-map +
    # RRT* global plan) | "pid" (ida_topics'in donanımda kanıtlanmış cascade
    # heading PID'i + LiDAR potansiyel-alan kaçınması — MPPI saha kalibrasyonu
    # tamamlanana kadar ya da beklenmedik davranışta düşme-güvenli yedek).
    # İkisi de AYNI FSM/parkur/güvenlik çatısı altında çalışır — yalnız
    # compute_control()'ın iç kontrolcüsü değişir.
    control_mode: str = "mppi"
    pid_cfg: PidControllerConfig = None  # type: ignore[assignment]

    # --- Saha tuning override'ları (ROS: planning_node mppi_* parametreleri) ---
    # HEPSİ None = "dokunma": MPPIConfig varsayılanı, λ'da ise PARKUR PROFİLİ
    # kazanır. Böylece varsayılanların TEK kaynağı kod olarak kalır (yaml'da
    # olmayan anahtar sessizce bir kopya varsayılanı dayatmaz — config-drift
    # taramasının dersi). Değer verilirse profili/varsayılanı EZER.
    mppi_lambda: Optional[float] = None
    mppi_sigma_u: Optional[float] = None
    mppi_obstacle_margin: Optional[float] = None
    mppi_terminal_mode: Optional[str] = None
    mppi_terminal_lookahead_m: Optional[float] = None
    mppi_ref_window_size: Optional[int] = None
    mppi_ref_window_enabled: Optional[bool] = None

    def __post_init__(self) -> None:
        if self.pid_cfg is None:
            self.pid_cfg = PidControllerConfig()

    def mppi_overrides(self) -> Dict[str, object]:
        """Verilen (None olmayan) MPPIConfig alan override'ları.

        λ BİLEREK dışarıda: parkur profilinden gelir, `_active_mppi_cfg`
        override'ı orada uygular (profil ↔ yaml önceliği tek yerde kalsın).
        """
        adlar = {
            "sigma_u": self.mppi_sigma_u,
            "obstacle_margin": self.mppi_obstacle_margin,
            "terminal_mode": self.mppi_terminal_mode,
            "terminal_lookahead_m": self.mppi_terminal_lookahead_m,
            "ref_window_size": self.mppi_ref_window_size,
            "ref_window_enabled": self.mppi_ref_window_enabled,
        }
        return {k: v for k, v in adlar.items() if v is not None}


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

        # Temel MPPI konfigürasyonu — parkur profili bunun üzerine biner.
        # Saha override'ları (verilmişse) burada uygulanır; λ profilden gelir.
        self._base_mppi_cfg = replace(
            MPPIConfig(
                K=self.cfg.mppi_K, T=self.cfg.mppi_T, dt=self.cfg.mppi_dt,
            ),
            **self.cfg.mppi_overrides(),        # type: ignore[arg-type]
        )
        if self.cfg.mppi_lambda is not None:    # parkur dışı kol için de geçerli
            self._base_mppi_cfg = replace(
                self._base_mppi_cfg, lambda_=self.cfg.mppi_lambda
            )

        self._state = np.zeros(6)                    # [x, y, ψ, u, v, r]
        self._obstacles: List[CircleObstacle] = []
        self._waypoints: List[Tuple[float, float]] = []
        self._ref_path: Optional[List[Tuple[float, float]]] = None
        self._mission_state = "BOOT"

        self._mppi: Optional[MPPIController] = None   # referans gelince kurulur
        # A3 — gereksiz RRT* koşumunu kesen durum (bkz. set_waypoints/set_obstacles)
        self._planlanan_goal: Optional[Tuple[float, float]] = None
        self._engel_imzasi_son: Optional[Tuple] = None
        self._replan_sayisi = 0          # fiilen koşan RRT* sayısı
        self._replan_atlandi = 0         # gereksiz olduğu için atlanan çağrı
        # F-S.10: PID yedek kontrolcü — cfg.control_mode="pid" iken kullanılır.
        self._pid = CascadeHeadingPidController(self.cfg.pid_cfg)

    # ----- girdi setter'ları -----

    def set_state(self, state: np.ndarray) -> None:
        """[x, y, ψ, u, v, r] durum vektörünü güncelle."""
        self._state[:] = state

    def set_waypoints(self, waypoints: List[Tuple[float, float]]) -> None:
        """Görev hedeflerini ayarla ve GEREKİYORSA global yörüngeyi yeniden planla.

        F-S.10: control_mode="pid" iken RRT*/MPPI hiç kurulmaz — PID hedefe
        doğrudan seyir eder, global path'e ihtiyaç duymaz (gereksiz CPU
        harcanmaz).

        🔴 **A3 (2026-08-06) — KOŞULSUZ REPLAN KALDIRILDI.** `planning_node`
        bu metodu `/girdap/mission/waypoints` kadansında (**5 Hz**) çağırıyor;
        her çağrıda RRT* koşuyordu. Ölçüm (P2 sahnesi, 40 s):
            **201 çağrı / 40 s = 5,0 çağrı/sn · ort 427 ms · p95 466 ms**
            → tek çekirdeğin **%215'i** (laptop), Orin Nano'da (3-5×) %640-1070.
        Üstüne `planning_node` tek-thread executor kullandığı için bu, 20 Hz
        kontrol timer'ını doğrudan bloke ediyordu.

        Yeni kural: hedef, **RRT*'ın kendi `goal_tolerance`'ı** kadar bile
        kaymadıysa mevcut plan zaten o hedefe gidiyordur (planlayıcı çözümü
        o toleransla kabul ediyor) → yeniden planlamaya GEREK YOK. Yeni bir
        eşik uydurulmadı; ölçüt planlayıcının kendi kabul yarıçapı.
        ⚠ Karşılaştırma **en son PLANLANAN** hedefe göre yapılır, bir önceki
        isteğe göre değil — yoksa 5 Hz'te birikerek kayan bir hedef (kapı
        nişanı drift'i) hiçbir zaman replan tetiklemezdi.
        """
        yeni = list(waypoints)
        if not yeni or self.cfg.control_mode == "pid":
            self._waypoints = yeni
            return
        self._waypoints = yeni
        hedef = yeni[-1]
        if self._ref_path is not None and self._planlanan_goal is not None:
            kayma = math.hypot(
                hedef[0] - self._planlanan_goal[0],
                hedef[1] - self._planlanan_goal[1],
            )
            if kayma <= self._rrt_cfg.goal_tolerance:
                self._replan_atlandi += 1
                return
        self._global_replan()

    def set_reference_direct(self, target_x: float, target_y: float) -> None:
        """RRT* bypass (video modu) — mevcut konumdan hedefe düz çizgi referansı.

        Global planlama atlanır; referans yörünge = [mevcut poz → hedef]. MPPI
        bu düz çizgiyi horizon içinde örnekler ve engel kaçınmayı üstlenir.
        """
        sx, sy = float(self._state[0]), float(self._state[1])
        self._waypoints = [(float(target_x), float(target_y))]
        self._ref_path = [(sx, sy), (float(target_x), float(target_y))]
        # A3: bu yol da bir "plan"dır — set_waypoints'in kayma ölçütü buna göre
        # çalışsın, yoksa bypass'tan RRT* koluna dönüldüğünde bayat hedefe bakar.
        self._planlanan_goal = (float(target_x), float(target_y))
        if self.cfg.control_mode != "pid":       # F-S.10: PID path'e ihtiyaç duymaz
            self._rebuild_mppi()

    def set_obstacles(self, obstacles: List[CircleObstacle]) -> None:
        """
        Perception engel listesini güncelle.
        Yeni engel mevcut ref_path'e < replan_proximity ise RRT* yeniden koşar;
        aksi halde sadece MPPI engel listesi tazelenir.

        🔴 **A3 (2026-08-06):** algı 10 Hz'te AYNI engel kümesini yeniden
        yayınlıyor. `_needs_replan` yalnız "rotaya yakın mı" diye baktığı için
        Parkur-2'de (engeller tanım gereği rotanın yanında) bu her karede
        DOĞRU dönüyor ve 10 Hz'te RRT* tetikleyebiliyordu. Artık önce
        **içerik değişti mi** bakılıyor; değişmediyse ne replan ne de MPPI
        yeniden kurulumu gerekir (ikisi de aynı veriyle aynı sonucu üretirdi).
        """
        imza = self._engel_imzasi(obstacles)
        if imza == self._engel_imzasi_son:
            self._obstacles = obstacles          # aynı içerik, nesneyi tazele
            self._replan_atlandi += 1
            return
        self._engel_imzasi_son = imza
        if self._ref_path is not None and self._needs_replan(obstacles):
            self._obstacles = obstacles
            self._global_replan()
        else:
            self._obstacles = obstacles
            if self._mppi is not None:
                self._rebuild_mppi()

    @staticmethod
    def _engel_imzasi(obstacles: List[CircleObstacle]) -> Tuple:
        """Engel kümesinin içerik imzası (sıra bağımsız, cm çözünürlüğünde).

        cm altı fark planlamayı değiştirmez (RRT* `safety_margin`=0.5 m,
        MPPI `obstacle_margin`=1.0 m); yuvarlama, algı gürültüsünün her
        karede sahte "değişti" üretmesini engeller.
        """
        return tuple(sorted(
            (round(o.cx, 2), round(o.cy, 2), round(o.r, 2)) for o in obstacles
        ))

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
        # F-S.10: parkur geçişinde PID'in heading-yumuşatma geçmişi de
        # sıfırlanır — MPPI'nin warm-start korumasıyla aynı ruh (soğuk
        # başlangıç zikzağı önlenir), ama PID durumsuz olduğundan ref_path
        # şartı yok.
        if state in _PARKUR_PROFILES and prev != state:
            self._pid.reset()

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
        self._replan_sayisi += 1
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
        self._planlanan_goal = (float(goal[0]), float(goal[1]))   # A3 ölçütü
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
            # λ: yaml/CLI override verilmişse o, yoksa PARKUR PROFİLİ kazanır.
            lambda_=(
                self.cfg.mppi_lambda
                if self.cfg.mppi_lambda is not None
                else profile.lambda_
            ),
        )

    def _rebuild_mppi(self) -> None:
        """Referans/engel/parkur değiştiğinde MPPI'yi güncelle.

        Warm-start korunması (F11.1): kontrolcüyü HER çağrıda yeniden yaratmak
        U_nominal'i sıfırlar → soğuk başlangıç → zikzak (node 5-10 Hz çağırır).
        Bu yüzden:
          - Config (parkur ağırlık profili) AYNI ise → mevcut kontrolcüyü koru,
            yalnız engel + referansı güncelle (U_nominal + çapa yaşar).
          - Config DEĞİŞTİYSE (parkur geçişi) → yeni kontrolcü kur ama eskisinin
            sıcak durumunu (U_nominal + kayan pencere çapası) devret
            (`carry_state_from`) — geçişte ne soğuk başlangıç ne de çapa
            sıfırlanmasından doğan tek adımlık tam tarama olsun.
        """
        if self._ref_path is None:
            return
        new_cfg = self._active_mppi_cfg()
        if self._mppi is not None and new_cfg == self._mppi.cfg:
            self._mppi.set_obstacles(self._obstacles)
            self._mppi.set_reference(self._ref_path, spacing=self.cfg.ref_spacing)
            return
        onceki = self._mppi
        self._mppi = MPPIController(
            self._dyn, self._bounds, self._obstacles, new_cfg
        )
        self._mppi.set_reference(self._ref_path, spacing=self.cfg.ref_spacing)
        if onceki is not None:
            self._mppi.carry_state_from(onceki)      # sıralama: referanstan SONRA

    # ----- kontrol -----

    def compute_control(self) -> Optional[np.ndarray]:
        """
        Tek kontrol adımı (MPPI ya da PID — cfg.control_mode). Dönüş:
            (2,) [T_left, T_right] (N) — parkur aktif ve kontrolcü hazırsa
            None — FSM parkur dışı veya referans/kontrolcü henüz yok (motor stop)
        """
        if self._mission_state not in _ACTIVE_STATES:
            return None
        if self.cfg.control_mode == "pid":
            return self._compute_control_pid()
        if self._mppi is None:
            return None
        return self._mppi.step(self._state)

    def _compute_control_pid(self) -> Optional[np.ndarray]:
        """F-S.10: PID yedek kontrolcü — RRT* global path gerekmez, hedefe
        (son waypoint) doğrudan seyir + LiDAR engel kaçınma."""
        if not self._waypoints:
            return None
        target = self._waypoints[-1]
        return self._pid.step(self._state, target, self._obstacles)

    # ----- sorgu -----

    @property
    def global_path(self) -> Optional[List[Tuple[float, float]]]:
        return self._ref_path

    @property
    def replan_sayaclari(self) -> Tuple[int, int]:
        """A3 teşhisi: (fiilen koşan RRT* sayısı, atlanan gereksiz çağrı).

        Sahada oran önemlidir: atlanan/koşan yükseldikçe planlayıcı boşa
        çalışmıyor demektir. Ölçüm (P2, 40 s): düzeltme öncesi 201/0.
        """
        return self._replan_sayisi, self._replan_atlandi

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
