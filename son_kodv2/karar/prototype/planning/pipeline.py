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
import time
from dataclasses import dataclass, replace
from typing import TYPE_CHECKING, Callable, Dict, List, Optional, Tuple

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

if TYPE_CHECKING:                      # yalnız tip denetimi için
    from prototype.planning.plan_isci import PlanIscisi


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
    # 🔴 F-P.9 (13.08.2026) — REPLAN FRENLERİ. Sahada ölçüldü (§0.66/§0.69):
    # `planning_node` tek thread'de koşar (`rclpy.spin`) ve RRT* AYNI thread'de
    # çalışır. Jetson'da `plan()` 100 engelle ortanca 510 ms / en kötü 1491 ms
    # sürüyor; kontrol bütçesi 100 ms. Bloklama iki yoldan birden vuruyor:
    # (a) o sürede `cmd_vel` yayınlanmıyor — ArduPilot GUIDED'da 3 sn komut
    #     gelmezse aracı DURDURUR, öncesinde ise SON komutu sürdürür (kör sürme);
    # (b) düğümün kendi abonelikleri işlenmiyor → `_last_odom_t` yaşlanıyor →
    #     KENDİ bekçisi (F-P.1) "poz bayat" deyip thrust'ı sıfırlıyor.
    #     Ölçülen kanıt: "poz 2,4 s bayat" yazarken füzyon 50 Hz yayındaydı.
    # Bu iki fren kökü çözmez (o, planlayıcıyı AYRI SÜRECE almaktır — Python
    # GIL yüzünden ayrı THREAD yetmez), ama RRT* çağrı sıklığını ~10 Hz'ten
    # ≤1 Hz'e indirir.
    # 🔑 FREN SABİT DEĞİL, SON PLANIN SÜRESİNE GÖRE: `aralık = min(katsayı ×
    # son_plan_süresi, tavan)`. Sabit bir sayı seçilmedi çünkü `plan()` maliyeti
    # sahneyle 5 kat değişiyor (Jetson ölçümü: 0 duba 173 ms · 20 duba 331 ms ·
    # 100 engel 510 ms · en kötü 1491 ms). Kural, ağır sahnede kendiliğinden
    # geri çekilir, ucuz sahnede tepkiselliği bırakır.
    #: Körlük payı katsayısı. `plan()` koşarken düğüm kördür; kör oran =
    #: T_plan/(T_plan+aralık) = 1/(1+katsayı). 3.0 → **≤ %25**. 0 → fren kapalı
    #: (eski davranış birebir).
    replan_bosluk_katsayisi: float = 3.0
    #: Frenin tavanı (s) — TAZELİK sınırı. Global rota, tekne `replan_proximity`
    #: kadar yol almadan tazelenmeli: 2,0 m ÷ **1,05 m/s** (ölçülmüş gerçek
    #: seyir hızı, CLAUDE.md dinamik log 58) = 1,9 s. Ayrıca ArduPilot'ın 3 s'lik
    #: GUIDED zaman aşımının altında kalır (aralık + T_plan < 3 s).
    replan_max_interval_s: float = 1.9
    # 🔴 F-P.10 (13.08.2026) — RRT* AYRI SÜREÇTE (bkz. `plan_isci.py`).
    # Ampirik ölçüm (bu Jetson, 10 Hz döngü, CUDA'lı ebeveyn): senkron planda
    # döngünün en kötü gecikmesi **370,7 ms**, asenkron kolda **1,1 ms** —
    # yani planlama hiç yokmuş gibi. Ayrı THREAD yetmez (GIL); ayrı SÜREÇ şart.
    #: ⚠ VARSAYILAN **KAPALI**. Prototip/çevrimdışı kullanım (viz, senaryo
    #: koşumu, birim testler) planı AYNI turda ister — asenkron kol orada
    #: belirlenimsizlik üretirdi. Üretimde `planning_node` açar (params.yaml
    #: `plan_isci_enabled: true`); açık olduğunda İLK plan yine senkrondur.
    plan_isci_enabled: bool = False
    #: İşçiden yanıt beklenecek üst süre (s) — aşılırsa işçi öldürülüp yenilenir.
    plan_isci_zaman_asimi_s: float = 5.0
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
        saat: Optional[Callable[[], float]] = None,
        isci: Optional["PlanIscisi"] = None,
    ) -> None:
        # F-P.9: replan freni "ne kadar zaman GEÇTİ" sorar → TEK YÖNLÜ saat
        # (F-S.14 kuralı: geçen süre monotonic, mutlak an duvar saati). Testler
        # ve sim, kendi saatini enjekte edebilsin diye çağrılabilir alınıyor.
        self._saat: Callable[[], float] = saat or time.monotonic
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
        #: Dosya-3 haritasına çizilecek engeller (bkz. `set_gosterim_engelleri`).
        #: None → harita kontrol listesinden çizilir (eski davranış).
        self._gosterim_engelleri: Optional[List[CircleObstacle]] = None
        self._waypoints: List[Tuple[float, float]] = []
        self._ref_path: Optional[List[Tuple[float, float]]] = None
        self._mission_state = "BOOT"

        self._mppi: Optional[MPPIController] = None   # referans gelince kurulur
        # A3 — gereksiz RRT* koşumunu kesen durum (bkz. set_waypoints/set_obstacles)
        self._planlanan_goal: Optional[Tuple[float, float]] = None
        self._engel_imzasi_son: Optional[Tuple] = None
        self._replan_sayisi = 0          # fiilen koşan RRT* sayısı
        self._replan_atlandi = 0         # gereksiz olduğu için atlanan çağrı
        # F-P.9 fren durumu: son RRT* koşumunun ANI ve SÜRESİ (ikisi de tek
        # yönlü saatte). Süre "bir sonraki koşuma ne kadar ara verilecek"i
        # belirler; ölçüm yoksa (ilk plan) fren UYGULANMAZ.
        self._son_replan_t: Optional[float] = None
        self._son_plan_suresi_s: Optional[float] = None
        self._replan_ertelendi = 0       # fren yüzünden ertelenen çağrı
        #: son bildirilen arena taşması (m) — tekrar uyarı eşiği için
        self._arena_tasmasi_m = 0.0
        #: arena dışı hedefle planlanan tur sayısı (teşhis; 0 = hep içeride)
        self.arena_tasma_sayisi = 0
        # F-P.10 asenkron planlama durumu. İşçi TEMBEL kurulur: ilk asenkron
        # ihtiyaç doğana kadar süreç başlatılmaz (her boru hattı nesnesi bir
        # süreç doğurmasın — testler yüzlerce nesne kuruyor).
        self._isci: Optional["PlanIscisi"] = isci
        self._bekleyen_goal: Optional[Tuple[float, float]] = None
        self._gonderim_t: Optional[float] = None
        self._asenkron_plan = 0          # teşhis: işçiye giden istek sayısı
        self._duz_cizgiye_dusuldu = 0    # A1: RRT* reddedip düz çizgiye düşülen tur
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
        self._obstacles = obstacles
        if (
            self._ref_path is not None
            and self._needs_replan(obstacles)
            and not self._replan_frenli()
        ):
            self._global_replan()
        elif self._mppi is not None:
            # ⚠ FREN GÜVENLİĞİ ZAYIFLATMAZ: engeller her karede MPPI'ye
            # verilir (soft ceza, 10 Hz). Ertelenen yalnız GLOBAL rotanın
            # yenilenmesidir; kaçınma katmanı tam hızda çalışmaya devam eder.
            self._rebuild_mppi()

    def set_gosterim_engelleri(
        self, obstacles: Optional[List[CircleObstacle]]
    ) -> None:
        """Dosya-3 haritasına ÇİZİLECEK engeller — kontrol yolunu ETKİLEMEZ.

        🔴 **Kaptan kararı 15.08.2026:** *"canlı haritada da 99 verileri
        olmayacak."* Ölçüm (session_20260814_153256): sınıflı akıştaki
        tespitlerin **%98,6'sı** `CLASS_UNKNOWN=99` — teslim edilen harita
        fiilen eşleşmemiş LiDAR kümelerinden oluşan gri bir bulut oluyordu.

        🔑 **Neden AYRI liste, neden `set_obstacles` süzülmüyor:** eşleşmemiş
        küme GÜVENLİK nesnesidir (füzyon sözleşmesi: *"bilinmeyeni atma"*) ve
        MPPI'nin engel torbasında KALMALIDIR — 99'lar torbadan çıkarılırsa
        kaçınma fiilen kör kalır (tespitlerin %98,6'sı). Bu yüzden kontrol
        listesi (`_obstacles`) aynen korunur, harita ayrı listeden çizilir.

        `None` verilirse (ya da hiç çağrılmazsa) harita eskisi gibi kontrol
        listesinden çizilir — geriye tam uyumlu, sınıfsız LiDAR kolunda ve
        prototip/görselleştirme kullanımında davranış birebir aynı.
        """
        self._gosterim_engelleri = obstacles

    def _replan_frenli(self) -> bool:
        """F-P.9: bu turda RRT* koşmak ERKEN mi (kör kalma payı dolmadı mı)?

        Fren yalnız ENGEL kaynaklı replan'a uygulanır — `set_waypoints`'in
        (yeni hedef) ve ilk planın yolu buradan geçmez, onlar hiç ertelenmez.
        Ölçüm yoksa (henüz plan koşmadıysa) fren YOK: soğuk başlangıçta rotayı
        geciktirmek, tam da aracın kıpırdamadığı ana denk gelirdi.
        """
        if self.cfg.replan_bosluk_katsayisi <= 0.0:      # fren kapalı
            return False
        if self._son_replan_t is None or self._son_plan_suresi_s is None:
            return False
        aralik = min(
            self.cfg.replan_bosluk_katsayisi * self._son_plan_suresi_s,
            self.cfg.replan_max_interval_s,
        )
        if (self._saat() - self._son_replan_t) >= aralik:
            return False
        self._replan_ertelendi += 1
        return True

    def _etkin_sinir(self) -> Bounds:
        """İKİ PLANLAYICININ ORTAK sınır kutusu = statik kutu ∪ (start/goal ± pay).

        🔴 **14.08.2026 — F-S.17. Bu daha önce YALNIZ RRT\\*'a uygulanıyordu**
        (F10.2), MPPI ham `self._bounds`'u alıyordu. Sonuç: iki planlayıcı
        dünyanın nerede bittiği konusunda ANLAŞMIYORDU — RRT* kutunun dışına
        rahatça yol çiziyor, MPPI `w_boundary=1000` duvarıyla o yolu takip
        etmeyi reddediyordu. Araç sessizce duruyordu; ne hata ne uyarı.

        **İki yerde ölçüldü:**
          · Sanal ölçüm (§0.86a): GN5=(90,0) kutunun 30 m dışındaydı → tekne
            `x=59,9`'da durup 900 s tavana kadar bekledi (26 m yakınında duba
            yok, itki %13). Kutu düzeltilince **900 s → 220 s**.
          · Gerçek donanım (§0.87c): 13.08 GUIDED koşumunda görev hedefi
            **(27,3, −23,7)**, `params.yaml`'daki kutu ise `y ∈ [0, 200]` →
            hedef **23,7 m dışarıda**.

        **Neden bu doğru çözüm (dış kaynak):**
          · **MPC yinelemeli uygulanabilirlik (recursive feasibility):** sert
            durum kısıtı referansı dışarıda bırakırsa problem UYGULANAMAZ olur;
            literatürün standart çaresi kısıtı yumuşatmak ya da hedefi
            uygulanabilir kümeye taşımaktır. `w_boundary=1000` adı ceza olsa da
            öteki terimlerin (w_track 5, w_terminal 50) yanında fiilen SERT.
          · **Nav2 pratiği:** hedef global costmap'in dışındaysa planlama
            "her zaman başarısız olur" ve bunu **uyarı basarak** yapar. Bizim
            eşdeğer durumumuz **sessiz** — asıl tehlike buydu.
          · **Hiyerarşik planlama literatürü:** katmanlar farklı kısıt kümesi
            kullanınca yerel kontrolcü global yolu reddeder ve sistem kilide
            girer; çözüm katmanları CASCADE etmek değil, kısıt yönetimini
            BİRLEŞTİRMEK.

        ⚠ Kutu yalnız BÜYÜR (statik kutuyla `min`/`max`); yani sınırın koruma
        işlevi kaybolmaz, yalnız "hedefin kendisi duvarın dışında" patolojisi
        imkânsızlaşır. Gerçek parkur sınırı F-S.16'da algıdan türetilecek.

        🔴 **14.08 EKİ — büyüme artık SESSİZ DEĞİL.** F-S.17 kilidi çözdü ama
        yerine ikinci bir sessizlik bıraktı: hedef **nereye düşerse düşsün**
        kutu ona uyacak şekilde genişliyor ve kimse haber vermiyordu. Yani
        bozuk bir hedef — yanlış ENU orijini, hatalı `home`, yanlış çerçevede
        yüklenmiş görev, `competition_mission.yaml`'ın 0.0/0.0 yer tutucuları —
        "araç durdu" yerine **"araç 3000 km ötedeki bir noktaya doğru yola
        çıktı"** üretir; ikincisi daha kötüdür ve teşhisi daha zordur.
        Bildirilen arenanın dışına taşan her hedef artık **BAĞIRIR**
        (ana belge madde 5: hataları bağıran mekanizmalar).

        Eşik **ayarlanabilir bir sayı değil, arenanın kendi ölçeği** (kapı
        seçimindeki disiplinin aynısı): hedef, bildirilen arenayı kendi
        genişliğinden DAHA ÇOK aşıyorsa çerçeve neredeyse kesin yanlıştır.
        Gerekçe `_arena_tasmasini_bildir`'de.
        """
        b, pay = self._bounds, self.cfg.bounds_margin_m
        xs = [b.x_min, b.x_max]
        ys = [b.y_min, b.y_max]
        noktalar = [(float(self._state[0]), float(self._state[1]))]
        if self._waypoints:
            noktalar.append(self._waypoints[-1])
        for px, py in noktalar:
            xs += [px - pay, px + pay]
            ys += [py - pay, py + pay]
        etkin = Bounds(min(xs), max(xs), min(ys), max(ys))
        self._arena_tasmasini_bildir(etkin)
        return etkin

    def _arena_tasmasini_bildir(self, etkin: Bounds) -> None:
        """Etkin kutu bildirilen arenayı aşıyorsa BAĞIR (bkz. `_etkin_sinir`).

        Ölçü **payı çıkarılmış** taşmadır: `bounds_margin_m` kadar genişleme
        tasarımın kendisidir, arıza değil. Geriye kalan sayı "hedef, bildirilen
        arenanın kaç metre dışında" demektir.

        🔑 **EŞİK NEDEN ARENANIN KENDİ BOYU (ayarlanabilir sayı DEĞİL).**
        `params.yaml`'daki kutu kendi yorumunda *"yarışma alanı temsili"* —
        yani ÖLÇÜLMÜŞ bir arena değil, yer tutucu. Onun için "arenayı 1 m aşan
        her hedef" uyarısı **her gerçek görevde** çalardı: 13.08 donanım
        koşumundaki (27,3, −23,7) hedefi bile 23,7 m dışarıdaydı ve o hedef
        DOĞRUYDU. Her koşuda çalan uyarı, operatörün uyarılara bakmayı
        bırakmasıdır — kapatmak istediğimiz sessizliğin daha kötü hâli.

        Ayırt edici olan **büyüklük mertebesi**: hedef arenayı KENDİ boyundan
        daha çok aşıyorsa artık "kenarda biraz taşmış görev" değil, **çerçeve
        hatası**dır — yanlış ENU orijini, hatalı home, yanlış çerçevede
        yüklenmiş görev ya da `competition_mission.yaml`'ın 0.0/0.0 yer
        tutucuları (bunlar Gine Körfezi'ni gösterir → binlerce km). Ölçü
        arenanın kendinden türediği için ölçek-bağımsızdır: 200 m'lik alanda da
        2 km'lik alanda da aynı mantık çalışır, elle ayar gerekmez.
        """
        b, pay = self._bounds, self.cfg.bounds_margin_m
        tasma = max(
            b.x_min - etkin.x_min, etkin.x_max - b.x_max,
            b.y_min - etkin.y_min, etkin.y_max - b.y_max,
        ) - pay
        olcek = max(b.x_max - b.x_min, b.y_max - b.y_min)
        if tasma <= olcek:
            self._arena_tasmasi_m = 0.0
            return
        self.arena_tasma_sayisi += 1
        if abs(tasma - self._arena_tasmasi_m) > 1.0:
            self._arena_tasmasi_m = tasma
            hedef = self._waypoints[-1] if self._waypoints else None
            _log.warning(
                "ARENA DIŞI HEDEF: hedef bildirilen arenanın %.0f m dışında — "
                "arenanın KENDİ boyu (%.0f m) kadarını aşıyor, yani bu bir "
                "ÇERÇEVE HATASI (hedef=%s, arena x[%.0f,%.0f] y[%.0f,%.0f]). "
                "Kutu hedefe uyacak şekilde BÜYÜTÜLDÜ (F-S.17), yani araç "
                "DURMAZ — oraya doğru yola çıkar. Yanlış ENU orijini / hatalı "
                "home / yanlış çerçevede yüklenmiş görev / 0.0-0.0 yer tutucu "
                "waypoint olabilir. GÖREVİ DOĞRULA.",
                tasma, olcek, hedef,
                b.x_min, b.x_max, b.y_min, b.y_max,
            )

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

        🔴 **A1 (2026-08-09) — REFERANS HİÇ YOKKEN DÜZ ÇİZGİYE DÜŞÜLÜR.**
        "Eski referansı koru" kuralının sessiz bir deliği vardı: soğuk
        başlangıçta korunacak referans YOKTUR. O hâlde `_ref_path` None kalır
        → `_rebuild_mppi` erken döner → `_mppi` hiç kurulmaz →
        `compute_control` **None** → node sıfır thrust basar ve **araç hiç
        kıpırdamaz**. Belirtisi yalnız logdaki bir WARN satırıdır.

        Ölçüldü (09.08, kapalı döngü, model YOKken = her duba engel):
        GN kapı ortasından **1,5 m kaçıkken** (md 5.5.2.2 bunu açıkça mümkün
        sayıyor) hedef, dubanın `safety_margin`+r = 0,65 m halkasının içinde
        kalıyor → **2001/2001 adım sıfır thrust, 0/3 GN**. Aynı kök P3'ün
        145 puanını da kilitliyordu (hedef duba `class_id=2` → engel).

        Düşülen yol yeni değil: `set_reference_direct`'in düz çizgi referansı
        (video modunda kanıtlanmış kol). Engelden kaçınmayı MPPI'nin kendi
        `obstacle_margin` cezası üstlenir — RRT*'ın sert `safety_margin`'i
        gibi ikili "geçer/geçmez" kararı vermez, yani hedef halkanın içinde
        olsa bile çözüm üretir. RRT* bir sonraki turda başarılı olursa
        yörünge normal yoluna kendiliğinden döner.
        """
        if not self._waypoints:
            return False
        start = (float(self._state[0]), float(self._state[1]))
        goal = self._waypoints[-1]
        # F10.2 → F-S.17: örnekleme alanı artık MPPI ile ORTAK (`_etkin_sinir`).
        bounds = self._etkin_sinir()
        # F-P.10: ARAÇ HAREKETTEYKEN plan AYRI SÜREÇTE koşar. İlk plan
        # (`_ref_path is None`) bilerek SENKRON kalır: görev başında araç
        # duruyor ve `cmd_vel` akışı yok — orada bloklamak zararsız, referanssız
        # kalmak zararlıdır (A1: `_ref_path` None → MPPI kurulmaz → araç
        # kıpırdamaz). İşçi meşgulse bu tur ATLANIR (senkrona DÜŞÜLMEZ; düşmek
        # tam da kaçındığımız bloklamayı geri getirirdi).
        if self._ref_path is not None:
            isci = self._plan_iscisi()
            if isci is not None:
                if isci.mesgul:
                    return False
                simdi = self._saat()
                if isci.gonder(
                    bounds, self._obstacles, self._rrt_cfg, start, goal, simdi
                ):
                    self._replan_sayisi += 1
                    self._asenkron_plan += 1
                    self._bekleyen_goal = (float(goal[0]), float(goal[1]))
                    self._gonderim_t = simdi
                    # Fren, gönderim anından işler: uçuşta istek varken
                    # yenisini üretmeye çalışmayalım.
                    self._son_replan_t = simdi
                    return False        # yol henüz yok; MEVCUT referans korunur
                # gönderilemedi (işçi düştü) → aşağıdaki senkron kol yedek

        rrt = RRTStar(bounds, self._obstacles, self._rrt_cfg)
        self._replan_sayisi += 1
        # F-P.9: koşum SÜRESİ ölçülür — bir sonraki koşumun freni bundan
        # türetilir. Ölçüm başarısız plan için de geçerlidir: bloklama
        # planın başarısına değil süresine bağlıdır (uzak hedefte RRT* hiç
        # çözüm bulamadan 1,1 s harcıyor — §0.67a).
        _t0 = self._saat()
        try:
            path = rrt.plan(start, goal)
        except ValueError as exc:
            self._plan_suresini_kaydet(_t0)
            return self._rrt_basarisiz(f"plan reddedildi ({exc})", goal)
        self._plan_suresini_kaydet(_t0)
        if path is None:
            return self._rrt_basarisiz("çözüm bulamadı", goal)
        self._ref_path = path
        self._planlanan_goal = (float(goal[0]), float(goal[1]))   # A3 ölçütü
        self._rebuild_mppi()
        return True

    def plan_isci_saglik(self) -> dict:
        """F-P.10 asenkron planlama sağlığı — dışarıdan okunabilir teşhis.

        🔴 13.08 kod incelemesi bulgusu. Mekanizma doğruydu ama GÖRÜNÜR
        değildi: işçi bir kez kurulamazsa `kullanilabilir` kalıcı olarak
        False olur, o an tek bir `logger.warning` düşer ve sonrası sessizdir.
        Boru hattı senkron kola döner — yani kontrol döngüsü yeniden 370 ms'ye
        kadar bloklanır, ki KAR-11/KAR-09 belirtilerini üreten şey tam buydu.
        Aynı şekilde `zaman_asimi` sayacı artıyordu ama kimse okumuyordu;
        her zaman aşımı 5 saniye ve işçiyi öldürüp yeniden kuruyor.

        Bu hafta üç ayrı yerde aynı desen çıktı: *arıza vardı, kod onu
        biliyordu, ama kimseye söylemiyordu.* Sayaçları dışarı açmak
        mekanizmayı değiştirmiyor, yalnız sessizliği kaldırıyor.
        """
        isci = self._isci
        return {
            "acik": bool(self.cfg.plan_isci_enabled),
            "kullanilabilir": bool(isci.kullanilabilir) if isci else None,
            "gonderilen": getattr(isci, "gonderilen", 0) if isci else 0,
            "tamamlanan": getattr(isci, "tamamlanan", 0) if isci else 0,
            "zaman_asimi": getattr(isci, "zaman_asimi", 0) if isci else 0,
            "asenkron_plan": self._asenkron_plan,
        }

    def _plan_iscisi(self) -> Optional["PlanIscisi"]:
        """F-P.10 işçisini (gerekiyorsa) kur; kapalıysa/kurulamıyorsa None.

        Tembel kurulum: süreç ancak ilk asenkron ihtiyaçta doğar. İşçi bir kez
        `kullanilabilir=False` olursa (spawn yok/izin yok) bir daha denenmez —
        boru hattı sessizce senkron kolda çalışmayı sürdürür.
        """
        if not self.cfg.plan_isci_enabled:
            return None
        if self._isci is None:
            from prototype.planning.plan_isci import PlanIscisi
            self._isci = PlanIscisi(self.cfg.plan_isci_zaman_asimi_s)
        return self._isci if self._isci.kullanilabilir else None

    def plan_sonucunu_isle(self) -> bool:
        """F-P.10: işçiden gelen planı KUR (bloklamaz). True = referans değişti.

        Düğüm bunu kontrol adımında çağırır — maliyeti bir kuyruk yoklamasıdır.
        Sonuç yoksa hiçbir şey olmaz; MPPI mevcut referansla sürmeye devam eder.
        """
        if self._isci is None or not self._isci.mesgul:
            return False
        sonuc = self._isci.sonuc_al(self._saat())
        if sonuc is None:
            return False
        yol, hata = sonuc
        if self._gonderim_t is not None:
            self._plan_suresini_kaydet(self._gonderim_t)
        goal = self._bekleyen_goal or (
            self._waypoints[-1] if self._waypoints else (0.0, 0.0)
        )
        self._bekleyen_goal = None
        self._gonderim_t = None
        if yol is None:
            return self._rrt_basarisiz(hata or "çözüm bulamadı", goal)
        self._ref_path = yol
        self._planlanan_goal = (float(goal[0]), float(goal[1]))   # A3 ölçütü
        self._rebuild_mppi()
        return True

    def kapat(self) -> None:
        """Boru hattını kapat — işçi süreci varsa düzgünce durdurulur."""
        if self._isci is not None:
            self._isci.kapat()

    def _plan_suresini_kaydet(self, t0: float) -> None:
        """F-P.9: son RRT* koşumunun anını ve süresini yaz (fren girdisi)."""
        simdi = self._saat()
        self._son_replan_t = simdi
        self._son_plan_suresi_s = max(0.0, simdi - t0)

    def _rrt_basarisiz(self, sebep: str, goal: Tuple[float, float]) -> bool:
        """RRT* bir plan üretemedi — referans İSTENEN hedefe gidiyorsa koru,
        gitmiyorsa düz çizgiye düş.

        Ayrım kasıtlı: elde, hedefe giden çalışan bir yörünge varken onu düz
        çizgiyle değiştirmek gerileme olurdu (RRT* geçen tur o yolu bir sebeple
        seçti). Ama korunan referans **başka bir hedefe** gidiyorsa "koru"
        demek, araca eski waypoint'e gitmeyi sürdürtmek demektir.

        Ölçüldü (09.08): GN kaçıkken 1. noktaya varılıyor, sonra waypoint
        ilerliyor, yeni hedefe plan kurulamıyor ve araç **1. noktanın
        yörüngesinde kalıyor → 1/3 GN**. Bayatlık ölçütü yeni değil, A3'ün
        `set_waypoints`'te kullandığı ölçütün aynısı: planlanan hedef ile
        istenen hedef arasındaki kayma > RRT*'ın kendi `goal_tolerance`'ı.
        """
        bayat = (
            self._planlanan_goal is None
            or math.hypot(
                goal[0] - self._planlanan_goal[0],
                goal[1] - self._planlanan_goal[1],
            ) > self._rrt_cfg.goal_tolerance
        )
        if self._ref_path is not None and not bayat:
            _log.warning("RRT* %s — eski referans korunuyor", sebep)
            return False
        self._duz_cizgiye_dusuldu += 1
        _log.error(
            "RRT* %s ve REFERANS YOK → düz çizgi hedefine düşülüyor "
            "(%.1f, %.1f). Araç hareket eder; engelden kaçınma yalnız MPPI'de. "
            "Sık tekrarlıyorsa hedef bir dubanın içinde olabilir.",
            sebep, goal[0], goal[1],
        )
        self.set_reference_direct(goal[0], goal[1])
        return False

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
        sinir = self._etkin_sinir()                  # F-S.17: RRT* ile AYNI kutu
        if self._mppi is not None and new_cfg == self._mppi.cfg:
            # ⚠ SICAK YOL — kontrolcü korunur (warm-start, F11.1). Sınır
            # `cfg`'nin parçası DEĞİL, ayrı bir kurucu argümanı; bu yüzden
            # burada ELLE tazelenmeli. Tazelenmezse araç ilerledikçe MPPI eski
            # kutuyla kalır ve F-S.17 yalnız parkur geçişlerinde düzelir —
            # yani hatanın en sinsi hâli geri gelir.
            self._mppi.bounds = sinir
            self._mppi.set_obstacles(self._obstacles)
            self._mppi.set_reference(self._ref_path, spacing=self.cfg.ref_spacing)
            return
        onceki = self._mppi
        self._mppi = MPPIController(
            self._dyn, sinir, self._obstacles, new_cfg
        )
        self._mppi.set_reference(self._ref_path, spacing=self.cfg.ref_spacing)
        if onceki is not None:
            self._mppi.carry_state_from(onceki)      # sıralama: referanstan SONRA

    def yeniden_basla(self) -> None:
        """Kontrolcü sıcak durumunu sıfırla — md 5.5.3.1 yeniden başlama.

        Yeniden başlamada araç fiilen başa döner; ilk turdan kalan sıcak durum
        artık YANLIŞ bir tahmin:
          - **MPPI `U_nominal`** (warm-start): ilk turun son manevrasını
            (ör. son kapıda tam dönüş) ikinci turun ilk adımına dayatır.
          - **Kayan pencere çapası** (F-M.2): referans üzerinde ilerideki bir
            noktayı gösterir; araç başa döndüğü için çapa geride kalır ve her
            adım kenar-fallback'e düşer (ölçülen bedel: adım 58 → 791 ms).
          - **PID integratörü**: ilk turun birikmiş hatası ikinci turda
            başlangıç vuruşu (windup) olarak boşalır.

        Warm-start'ı sıfırlamak F11.1'in TERSİ değil: F11.1 ardışık adımlar
        arası sürekliliği korumakla ilgili; burada süreklilik zaten koptu.
        """
        if self._mppi is not None:
            self._mppi.reset_warm_start()
        self._pid.reset()          # F-S.10 yedek kontrolcü (control_mode="pid")

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
    def compute_control_hazir(self) -> bool:
        """Kontrolcü kurulu mu — `compute_control()` komut üretebilir mi.

        A1 teşhisi: False iken node **sıfır thrust** basar ve araç kıpırdamaz.
        Ayrı bir property çünkü "referans var" (`global_path`) ile "kontrolcü
        kuruldu" aynı şey değil: `control_mode="pid"` kolunda MPPI hiç kurulmaz.
        """
        if self.cfg.control_mode == "pid":
            return bool(self._waypoints)
        return self._mppi is not None

    @property
    def duz_cizgiye_dusuldu(self) -> int:
        """A1: RRT* reddedip düz çizgi referansına düşülen tur sayısı (teşhis).

        Sahada 0'dan büyükse hedef bir dubanın payı içinde kalıyor demektir —
        araç yine de sürülür ama kaçınma yalnız MPPI cezasına kalır.
        """
        return self._duz_cizgiye_dusuldu

    @property
    def replan_sayaclari(self) -> Tuple[int, int]:
        """A3 teşhisi: (fiilen koşan RRT* sayısı, atlanan gereksiz çağrı).

        Sahada oran önemlidir: atlanan/koşan yükseldikçe planlayıcı boşa
        çalışmıyor demektir. Ölçüm (P2, 40 s): düzeltme öncesi 201/0.
        """
        return self._replan_sayisi, self._replan_atlandi

    @property
    def replan_ertelendi(self) -> int:
        """F-P.9 teşhisi: kör kalma payı dolmadığı için ertelenen replan sayısı.

        Sahada `replan_sayisi` ile birlikte okunur: ertelenen/koşan oranı
        frenin fiilen ne kadar iş kestiğini söyler. Sıfır kalıyorsa fren hiç
        devreye girmemiş demektir (sahne ucuz ya da katsayı 0).
        """
        return self._replan_ertelendi

    @property
    def son_plan_suresi_s(self) -> Optional[float]:
        """Son RRT* koşumunun süresi (s) — frenin girdisi; None = hiç koşmadı."""
        return self._son_plan_suresi_s

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
        # Harita ÇİZİM listesinden kurulur; verilmemişse kontrol listesinden
        # (eski davranış). Ayrımın gerekçesi: `set_gosterim_engelleri`.
        cizilecek = (
            self._obstacles if self._gosterim_engelleri is None
            else self._gosterim_engelleri
        )
        if cizilecek:
            margin = self._base_mppi_cfg.obstacle_margin
            for o in cizilecek:
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
        # 🔴 2026-08-07 — ARENA MASKESİ KALDIRILDI (Dosya-3 teslim denetimi).
        # Eskiden `bounds` dışı hücreler -1 (bilinmiyor) işaretleniyordu. İki
        # ayrı sebeple YANLIŞTI:
        #   1) `bounds` YARIŞMA ALANI DEĞİL, RRT*'ın ÖRNEKLEME KUTUSUDUR —
        #      üstelik `_global_replan` onu her planda start/goal ± 30 m ile
        #      genişletiyor (F10.2). Planlayıcının arama kutusunu "bilinen
        #      dünya" sanmak kategori hatasıydı.
        #   2) Ölçülen bedel: varsayılan `bounds_x/y=[0,200]` ve araç odom
        #      origin'inde (0,0) başladığı için 50×50 m pencerenin **%75'i**
        #      gri çıkıyordu → teslim edilen PNG'nin çoğu "bilinmiyor".
        # Bu haritada gerçek bir "bilinmiyor" kavramı YOK (sensör kapsama
        # alanını izlemiyoruz); olmayan bilgiyi uydurmak yerine hücreler
        # engel maliyetini taşır. -1 desteği çizici/dumper tarafında DURUYOR:
        # ileride gerçek kapsama izlenirse anlamıyla birlikte geri gelir.

        return LocalCostGrid(
            data=grid.reshape(-1).astype(np.int8),           # ROS satır-major
            width=w,
            height=h,
            resolution=res,
        )
