"""
Girdap İDA — Görev Sonlu Durum Otomatı (Mission FSM)

Şartname referansları:
    - 5.5.2.2 → Parkur 1→2→3 geçişleri TAMAMEN OTONOM. Dış komut yok.
    - 4.1     → Görev başladıktan sonra YKİ→İDA komut yasak (KILL hariç).

Durumlar ve geçişler:
    BOOT       ──boot_ok──→            ARM
    ARM        ──kill_switch_off──→    BEKLEMEDE
    BEKLEMEDE  ──YKİ "başlat"──→       PARKUR-1   (tek dış sinyal — sadece burada)
    PARKUR-1   ──son wp <1.5 m──→      PARKUR-2
    PARKUR-2   ──son duba ikilisi──→   PARKUR-3
    PARKUR-3   ──IMU şok──→            TAMAMLANDI
    *          ──kill──→               KILL       (RC kumanda + YKİ + watchdog)

Mimari notlar:
    - CLAUDE.md "aşırı mühendislik yapma" kuralı → enum + dict[State, Callbacks]
      yeterli. transitions kütüphanesi vb. eklenmez.
    - KILL geçişi ÖNCELİKLİDİR: tick() önce KILL koşulunu test eder, sonra
      durum-bazlı kuralı değerlendirir.
    - Callback üçlüsü (on_enter / on_exit / on_tick) sahaya uyarlamada karar
      modüllerinin (RRT*, MPPI, telemetri yazıcı) yaşam döngüsünü kontrol eder.
    - Tick'i besleyen Observation kasıtlı olarak ince — gerçek sahada bu alanlar
      iSAM2/MAVROS/LiDAR node'larından doldurulur. FSM sensör soyutlamaz, sadece
      kararı verir.

Çalıştırma (demo + KTR diyagramı):
    python -m prototype.fsm.mission_fsm
"""

from __future__ import annotations

import logging
import math
from dataclasses import dataclass
from enum import Enum
from pathlib import Path
from typing import Callable, Dict, List, Optional, Tuple

# NOT: matplotlib yalnızca KTR görselleştirmesi (draw_state_diagram) içindir;
# runtime ROS node'u (fsm_node) çekmesin diye modül seviyesinde import EDİLMEZ,
# fonksiyon içinde tembel import edilir. (Sistem matplotlib'i NumPy 1.x ABI'sine
# bağlı; NumPy 2.x altında modül seviyesinde import node'u çökertirdi.)

_REPO_ROOT = Path(__file__).resolve().parents[2]
_log = logging.getLogger(__name__)


# --------------------------------------------------------------------------- #
# Tipler
# --------------------------------------------------------------------------- #


class MissionState(Enum):
    """Sonlu durum kümesi — yarışma görev hattı + acil durum."""

    BOOT = "BOOT"
    ARM = "ARM"
    BEKLEMEDE = "BEKLEMEDE"
    PARKUR1 = "PARKUR1"
    PARKUR2 = "PARKUR2"
    PARKUR3 = "PARKUR3"
    TAMAMLANDI = "TAMAMLANDI"
    KILL = "KILL"


@dataclass
class Observation:
    """
    FSM tick'ine beslenen sinyaller. Gerçek sahada:
        boot_ok               — ROS 2 graph hazır + sensör presence check
        kill_switch_off       — Pixhawk RC kanalı + yazılım flag'i
        kill_switch_active    — kill onaylandı (sıfırlanmaz)
        dist_to_last_wp_p1    — RRT* hedef listesindeki son wp'ye anlık mesafe
        last_gate_passed_p2   — görev kütüphanesi (gate geometri kontrolü)
        shock_detected_p3     — IMU |a| spike eşiği (ham high-rate kanal)
        mission_complete      — görev yöneticisi TÜM waypoint'leri bitirdi
                                (video senaryosu terminal koşulu; kamikaze
                                çarpması olmadan da PARKUR*→TAMAMLANDI)
    """

    boot_ok: bool = False
    kill_switch_off: bool = False
    kill_switch_active: bool = False

    dist_to_last_wp_p1: float = math.inf
    last_gate_passed_p2: bool = False
    shock_detected_p3: bool = False
    mission_complete: bool = False


@dataclass
class StateCallbacks:
    """Bir duruma bağlı yaşam döngüsü kancaları — opsiyonel."""

    on_enter: Optional[Callable[[], None]] = None
    on_exit: Optional[Callable[[], None]] = None
    on_tick: Optional[Callable[[Observation], None]] = None


# --------------------------------------------------------------------------- #
# FSM
# --------------------------------------------------------------------------- #


class MissionFSM:
    """
    Görev otomatı.

    Tipik kullanım (sahada):
        fsm = MissionFSM()
        fsm.register(MissionState.PARKUR1, on_enter=start_p1, on_tick=run_p1)
        ...
        while running:
            obs = collect_observation()
            fsm.tick(obs)
            if fsm.state is MissionState.KILL:
                shutdown()
    """

    # PARKUR-1 → PARKUR-2 yakınsama eşiği (CLAUDE.md FSM bölümü).
    P1_TO_P2_DIST: float = 1.5  # m

    def __init__(self) -> None:
        self._state: MissionState = MissionState.BOOT
        self._callbacks: Dict[MissionState, StateCallbacks] = {
            s: StateCallbacks() for s in MissionState
        }
        self._start_requested: bool = False
        self._kill_reason: Optional[str] = None
        self._history: List[Tuple[MissionState, MissionState, str]] = []

    # ----- public erişim -----

    @property
    def state(self) -> MissionState:
        return self._state

    @property
    def last_gate_passed(self) -> bool:
        """
        Son duba ikilisi geçildi mi? (PARKUR3+ evresine girildiyse True.)
        Türetilmiş monoton ilerleme bayrağı — planning_node kamikaze evresine
        geçişi bu sinyalle de teyit edebilir (mission/state'e ek redundant kanal).
        """
        return self._state in (MissionState.PARKUR3, MissionState.TAMAMLANDI)

    @property
    def history(self) -> List[Tuple[MissionState, MissionState, str]]:
        """Sıralı (eski, yeni, gerekçe) listesi — replay/log için."""
        return list(self._history)

    def register(
        self,
        state: MissionState,
        *,
        on_enter: Optional[Callable[[], None]] = None,
        on_exit: Optional[Callable[[], None]] = None,
        on_tick: Optional[Callable[[Observation], None]] = None,
    ) -> None:
        """Bir duruma callback bağla. Önceki bağlamayı ezer."""
        self._callbacks[state] = StateCallbacks(on_enter, on_exit, on_tick)

    def request_start(self) -> None:
        """YKİ 'başlat' komutu — yalnız BEKLEMEDE'de etkili (Şartname 4.1)."""
        self._start_requested = True

    def kill(self, reason: str = "manual kill") -> None:
        """Acil durdurma — bir sonraki tick'te KILL'e geçilir."""
        self._kill_reason = reason

    # ----- ana döngü -----

    def tick(self, obs: Observation) -> MissionState:
        """
        Bir adım yürüt. Geçiş varsa on_exit/on_enter tetiklenir, ardından
        mevcut durumun on_tick'i çağrılır.

        Öncelik sırası:
            1) KILL (bayrak veya kill_switch_active)
            2) Durum-bazlı kural
            3) on_tick dispatch
        """
        # 1) KILL — her durumdan, herhangi bir tick'te
        if self._state is not MissionState.KILL and (
            self._kill_reason is not None or obs.kill_switch_active
        ):
            reason = self._kill_reason or "kill_switch_active"
            self._transition(MissionState.KILL, reason)
            self._dispatch_tick(obs)
            return self._state

        # 2) Durum-bazlı geçiş
        next_pair = self._evaluate_transition(obs)
        if next_pair is not None:
            new_state, reason = next_pair
            self._transition(new_state, reason)

        # 3) Mevcut durumun tick callback'i
        self._dispatch_tick(obs)
        return self._state

    # ----- iç -----

    def _evaluate_transition(
        self, obs: Observation
    ) -> Optional[Tuple[MissionState, str]]:
        """Tek tablo halinde geçiş kuralları. Her durum tek hedef üretir."""
        s = self._state
        if s is MissionState.BOOT and obs.boot_ok:
            return MissionState.ARM, "boot_ok"
        if s is MissionState.ARM and obs.kill_switch_off:
            return MissionState.BEKLEMEDE, "kill_switch_off"
        if s is MissionState.BEKLEMEDE and self._start_requested:
            # Tek atış: tüketilince sıfırla
            self._start_requested = False
            return MissionState.PARKUR1, "YKİ başlat"
        # Görev tamamlandı (tüm waypoint'ler) → TAMAMLANDI. Parkur/kamikaze
        # yolundan BAĞIMSIZ terminal: video senaryosu (tek parkur, çarpma yok)
        # buraya varır ve araç temiz durur (TAMAMLANDI'da compute_control None →
        # sıfır thrust). Parkur geçiş kurallarından ÖNCE değerlendirilir ki
        # görev bitince spurious PARKUR2 geçişi kazanmasın.
        if (
            s in (MissionState.PARKUR1, MissionState.PARKUR2, MissionState.PARKUR3)
            and obs.mission_complete
        ):
            return MissionState.TAMAMLANDI, "görev tamamlandı (tüm waypoint'ler)"
        if (
            s is MissionState.PARKUR1
            and obs.dist_to_last_wp_p1 <= self.P1_TO_P2_DIST
        ):
            return (
                MissionState.PARKUR2,
                f"son wp {obs.dist_to_last_wp_p1:.2f} m ≤ "
                f"{self.P1_TO_P2_DIST:.1f} m",
            )
        if s is MissionState.PARKUR2 and obs.last_gate_passed_p2:
            return MissionState.PARKUR3, "son duba ikilisi geçildi"
        if s is MissionState.PARKUR3 and obs.shock_detected_p3:
            return MissionState.TAMAMLANDI, "IMU şok algılandı"
        return None

    def _transition(self, new: MissionState, reason: str) -> None:
        old = self._state
        cb_old = self._callbacks[old]
        if cb_old.on_exit is not None:
            cb_old.on_exit()
        self._history.append((old, new, reason))
        _log.info("FSM: %s → %s (%s)", old.value, new.value, reason)
        self._state = new
        cb_new = self._callbacks[new]
        if cb_new.on_enter is not None:
            cb_new.on_enter()

    def _dispatch_tick(self, obs: Observation) -> None:
        cb = self._callbacks[self._state]
        if cb.on_tick is not None:
            cb.on_tick(obs)


# --------------------------------------------------------------------------- #
# KTR diyagramı
# --------------------------------------------------------------------------- #


def draw_state_diagram(out_path: Path) -> None:
    """Şartname 5.5.2.2 ile uyumlu görsel — KTR rapor sayfasına gider."""
    import matplotlib.pyplot as plt          # tembel: yalnız görselleştirme
    from matplotlib.patches import FancyArrowPatch, FancyBboxPatch

    fig, ax = plt.subplots(figsize=(11.0, 8.5))
    ax.set_xlim(-1, 12)
    ax.set_ylim(0, 10)
    ax.set_aspect("equal")
    ax.axis("off")

    box_w, box_h = 3.2, 0.85
    main_x = 2.5

    # (etiket, y, açıklama)
    main = [
        ("BOOT", 9.0),
        ("ARM", 8.0),
        ("BEKLEMEDE", 7.0),
        ("PARKUR-1\nNokta Takip", 5.8),
        ("PARKUR-2\nEngelli Takip", 4.4),
        ("PARKUR-3\nKamikaze", 3.0),
        ("TAMAMLANDI", 1.8),
    ]

    # Ana akış kutuları
    pos: Dict[str, Tuple[float, float]] = {}
    for label, y in main:
        rect = FancyBboxPatch(
            (main_x - box_w / 2, y - box_h / 2),
            box_w, box_h,
            boxstyle="round,pad=0.04,rounding_size=0.18",
            ec="black", fc="#cfe6ff", lw=1.4,
        )
        ax.add_patch(rect)
        ax.text(main_x, y, label, ha="center", va="center",
                fontsize=10, fontweight="bold")
        pos[label.split("\n")[0]] = (main_x, y)

    # KILL (sağda, kırmızı)
    kill_x, kill_y = 8.5, 5.0
    rect = FancyBboxPatch(
        (kill_x - box_w / 2, kill_y - box_h / 2),
        box_w, box_h,
        boxstyle="round,pad=0.04,rounding_size=0.18",
        ec="black", fc="#ffd0d0", lw=1.6,
    )
    ax.add_patch(rect)
    ax.text(kill_x, kill_y, "KILL\n(motor cut)",
            ha="center", va="center", fontsize=10, fontweight="bold")
    pos["KILL"] = (kill_x, kill_y)

    # Ana akış geçişleri
    transitions = [
        ("BOOT", "ARM", "boot_ok"),
        ("ARM", "BEKLEMEDE", "kill_switch_off"),
        ("BEKLEMEDE", "PARKUR-1", "YKİ 'başlat' (Şart. 4.1)"),
        ("PARKUR-1", "PARKUR-2", "son wp < 1.5 m"),
        ("PARKUR-2", "PARKUR-3", "son duba ikilisi geçildi"),
        ("PARKUR-3", "TAMAMLANDI", "IMU şok algılandı"),
    ]
    for src, dst, label in transitions:
        x0, y0 = pos[src]
        x1, y1 = pos[dst]
        arrow = FancyArrowPatch(
            (x0, y0 - box_h / 2),
            (x1, y1 + box_h / 2),
            arrowstyle="-|>", mutation_scale=15,
            lw=1.4, color="black",
        )
        ax.add_patch(arrow)
        ax.text(x0 + box_w / 2 + 0.1, (y0 + y1) / 2, label,
                fontsize=8.8, ha="left", va="center")

    # KILL geçişleri (kesik kırmızı, her ana durumdan)
    kill_sources = ["ARM", "BEKLEMEDE", "PARKUR-1", "PARKUR-2", "PARKUR-3"]
    for src in kill_sources:
        x0, y0 = pos[src]
        arrow = FancyArrowPatch(
            (x0 + box_w / 2, y0),
            (kill_x - box_w / 2, kill_y),
            arrowstyle="-|>", mutation_scale=10,
            lw=0.9, color="tab:red", linestyle="--",
            connectionstyle="arc3,rad=0.06",
        )
        ax.add_patch(arrow)

    # KILL açıklaması
    ax.text(
        kill_x, kill_y - 1.3,
        "Tetikler:\n"
        "• RC kumanda kill anahtarı\n"
        "• YKİ kill butonu\n"
        "• İç güvenlik watchdog\n"
        "  (sensör timeout, batarya alarmı)",
        ha="center", va="top", fontsize=8.5, color="tab:red",
    )

    ax.set_title(
        "Girdap İDA — Görev Sonlu Durum Otomatı (Şartname 5.5.2.2)",
        fontsize=12,
    )

    out_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out_path, dpi=150, bbox_inches="tight")
    plt.close(fig)


# --------------------------------------------------------------------------- #
# KTR demosu — sentetik tick zinciri
# --------------------------------------------------------------------------- #


def _demo() -> None:
    logging.basicConfig(level=logging.INFO, format="%(message)s")

    fsm = MissionFSM()

    # Sahada bu callback'ler RRT*, MPPI, telemetri yazıcı, görev kütüphanesi
    # gibi modülleri başlatır/durdurur. Demoda sadece print.
    fsm.register(
        MissionState.PARKUR1,
        on_enter=lambda: print("[fsm] PARKUR-1 başladı: nokta takip"),
        on_exit=lambda: print("[fsm] PARKUR-1 bitti"),
    )
    fsm.register(
        MissionState.PARKUR2,
        on_enter=lambda: print("[fsm] PARKUR-2 başladı: engelli geçiş"),
        on_exit=lambda: print("[fsm] PARKUR-2 bitti"),
    )
    fsm.register(
        MissionState.PARKUR3,
        on_enter=lambda: print("[fsm] PARKUR-3 başladı: kamikaze hedefleme"),
        on_exit=lambda: print("[fsm] PARKUR-3 bitti"),
    )
    fsm.register(
        MissionState.TAMAMLANDI,
        on_enter=lambda: print("[fsm] motor stop, telemetri devam"),
    )
    fsm.register(
        MissionState.KILL,
        on_enter=lambda: print("[fsm] *** KILL — tüm motorlara stop ***"),
    )

    print("\n[demo] sentetik görev senaryosu çalıştırılıyor...\n")

    # 1) Boot → Arm
    fsm.tick(Observation(boot_ok=False))
    fsm.tick(Observation(boot_ok=True))

    # 2) Kill switch off → BEKLEMEDE
    fsm.tick(Observation(kill_switch_off=True))

    # 3) YKİ "başlat" → PARKUR1
    fsm.request_start()
    fsm.tick(Observation())

    # 4) PARKUR-1: yaklaşma profili
    fsm.tick(Observation(dist_to_last_wp_p1=5.0))
    fsm.tick(Observation(dist_to_last_wp_p1=2.8))
    fsm.tick(Observation(dist_to_last_wp_p1=1.0))   # → PARKUR2

    # 5) PARKUR-2: gate geçiş
    fsm.tick(Observation(last_gate_passed_p2=False))
    fsm.tick(Observation(last_gate_passed_p2=True)) # → PARKUR3

    # 6) PARKUR-3: kamikaze çarpma şoku
    fsm.tick(Observation(shock_detected_p3=False))
    fsm.tick(Observation(shock_detected_p3=True))   # → TAMAMLANDI

    print("\n[demo] geçiş geçmişi:")
    for old, new, reason in fsm.history:
        print(f"   {old.value:11s} → {new.value:11s}  ({reason})")
    print(f"\n[demo] son durum: {fsm.state.value}")

    # KILL senaryosu — ayrı bir FSM örneğinde
    print("\n[demo] KILL senaryosu (PARKUR-2 sırasında acil durdurma)...\n")
    fsm2 = MissionFSM()
    fsm2.tick(Observation(boot_ok=True))
    fsm2.tick(Observation(kill_switch_off=True))
    fsm2.request_start()
    fsm2.tick(Observation())
    fsm2.tick(Observation(dist_to_last_wp_p1=1.0))           # → P2
    fsm2.tick(Observation(kill_switch_active=True))          # → KILL
    print(f"[demo] KILL sonrası durum: {fsm2.state.value}")

    # KTR diyagramı
    out_path = _REPO_ROOT / "docs" / "KTR" / "mission_fsm_diagram.png"
    draw_state_diagram(out_path)
    print(f"\n[demo] diyagram kaydedildi: {out_path.relative_to(_REPO_ROOT)}")


if __name__ == "__main__":
    _demo()
