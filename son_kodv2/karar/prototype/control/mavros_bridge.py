"""
Girdap İDA — MAVROS köprü karar çekirdeği (ROS-bağımsız).

Pixhawk ↔ mavros bağlantısının **güvenlik ve mod geçidi (gating)** mantığı
burada toplanır. rclpy bağımsız olduğundan pytest ile .venv altında
doğrulanabilir. Layer 2 `mavros_bridge_node` bu çekirdeği sarar:
`/mavros/state` besler, kararları servis çağrılarına (set_mode, arming) ve
KILL tetiğine çevirir. Aynı çekirdek `planning_node`'da adım-başı geçit
kararında yeniden kullanılır (DRY).

Kapsanan güvenlik kuralları (öncelik sırasıyla):
    1. Heartbeat — `heartbeat_timeout_s` içinde `/mavros/state` gelmezse
       bağlantı koptu kabul edilir → KILL.
    2. Bağlantı — FCU `connected=False` → KILL.
    3. Failsafe (arm) — `armed=False` → thruster'lar sıfırlanır.
    4. Mod geçidi — `mode != GUIDED` → cmd_vel yayınlanmaz (mavros zaten
       yok sayar; iç tutarlılık için susturulur).

Not: Bu çekirdek *durumsuzdur* (son state anlık görüntüsünü tutar). Latching
failsafe (arm True→False geçişinde tek seferlik KILL) node katmanında ele
alınır; burada `control_gate` her çağrıda güncel state'e göre güvenli kararı
döndürür — planlayıcının adım-başı susturması için idealdir.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum, auto
from typing import Optional


# F14.3: auto-GUIDED yalnız bu FSM durumlarında zorlanır. Görev öncesi
# (BOOT/ARM/BEKLEMEDE) operatör aracı RC ile konumlar, görev sonrası
# (TAMAMLANDI, md 3.3.1/3) manuel dönüş serbesttir — köprü bu evrelerde
# operatörün mod seçimiyle KAVGA ETMEZ. Değerler MissionState.value ile aynı.
MISSION_ACTIVE_STATES = frozenset({"PARKUR1", "PARKUR2", "PARKUR3"})


class GateState(Enum):
    """Kontrol geçidinin ayrık durumu."""

    KILL = auto()        # heartbeat kaybı / bağlantı yok → her şeyi durdur
    DISARMED = auto()    # armed=False → thruster sıfır (otonom sürüş yok)
    NOT_GUIDED = auto()  # GUIDED değil → cmd_vel yayınlama (manuel/hold modu)
    ACTIVE = auto()      # armed + GUIDED + canlı heartbeat → tam yetki


@dataclass(frozen=True)
class MavrosBridgeConfig:
    """Köprü güvenlik parametreleri (config/params.yaml ile override edilir)."""

    heartbeat_timeout_s: float = 5.0
    target_mode: str = "GUIDED"


@dataclass(frozen=True)
class ControlGate:
    """`control_gate` çıktısı — planlayıcı/köprü bu karara göre davranır."""

    state: GateState
    allow_cmd_vel: bool   # /mavros/setpoint_velocity yayınlansın mı?
    zero_thrust: bool     # thruster komutu sıfıra zorlansın mı?
    reason: str           # log/telemetri için kısa gerekçe


@dataclass(frozen=True)
class MavStateSnapshot:
    """Tek `/mavros/state` mesajının ilgili alanları + alınma zamanı (s)."""

    t: float
    connected: bool
    armed: bool
    guided: bool
    mode: str


class MavrosBridge:
    """MAVROS güvenlik/mod geçidi karar çekirdeği (rclpy'siz, test edilebilir)."""

    def __init__(self, config: Optional[MavrosBridgeConfig] = None) -> None:
        self._cfg = config or MavrosBridgeConfig()
        self._last: Optional[MavStateSnapshot] = None
        # Operatör/görev sonu KOMUTLU disarm bekleniyor mu? (F14.2) True iken
        # gözlenen arm→disarm geçişi FAILSAFE sayılmaz — video güç-kesme
        # gösteriminde (md 3.3.1/4) sahte KILL/hata basılmasını önler.
        self._expected_disarm: bool = False
        # F14.3: görev-aktif bayrağı — FSM durumu bildirilene kadar False,
        # yani auto-GUIDED görev başlamadan asla mod zorlamaz.
        self._mission_active: bool = False
        # F-M.6: FC akış hızı (SR0) bu bağlantı için istendi mi? Bağlantı
        # koptuğunda sıfırlanır → yeniden bağlanışta hız yeniden istenir.
        self._stream_rate_requested: bool = False

    # ----- config / durum erişimi -----

    @property
    def config(self) -> MavrosBridgeConfig:
        return self._cfg

    @property
    def last_state(self) -> Optional[MavStateSnapshot]:
        return self._last

    def update_state(
        self,
        t: float,
        connected: bool,
        armed: bool,
        guided: bool,
        mode: str,
    ) -> None:
        """Yeni `/mavros/state` mesajını kaydet (t: alınma zamanı, saniye)."""
        self._last = MavStateSnapshot(
            t=float(t),
            connected=bool(connected),
            armed=bool(armed),
            guided=bool(guided),
            mode=str(mode),
        )
        # F-M.6: bağlantı düştü → FC'nin akış hızı ayarı da uçtu sayılır.
        if not connected:
            self._stream_rate_requested = False

    def is_armed(self) -> bool:
        return self._last is not None and self._last.armed

    # ----- beklenen (komutlu) disarm — F14.2 -----

    def note_command_disarm(self) -> None:
        """Operatör/görev sonu disarm KOMUTU verildi.

        Bir sonraki arm→disarm gözlemi failsafe değil, beklenen kabul edilir.
        `mavros_bridge_node._request_disarm` bunu çağırır.
        """
        self._expected_disarm = True

    def is_unexpected_disarm(self, was_armed: bool, now_armed: bool) -> bool:
        """arm True→False geçişi beklenmedik (failsafe) mi?

        Komutlu disarm (note_command_disarm çağrılmışsa) → False, bayrak
        tüketilir. Aksi halde gerçek arm→disarm geçişi → True (failsafe).
        """
        if not (was_armed and not now_armed):
            return False
        if self._expected_disarm:
            self._expected_disarm = False        # tek atış: tüketildi
            return False
        return True

    def current_mode(self) -> Optional[str]:
        return None if self._last is None else self._last.mode

    # ----- FC akış hızı (SR0) — F-M.6 -----

    def should_request_stream_rate(self) -> bool:
        """Bu bağlantı için FC akış hızı isteği gönderilmeli mi?

        ArduPilot taze bağlantıda SR0_* parametrelerine göre yayınlar; masada
        ölçülen ~1 Hz, Ekran-2 grafiklerini basamaklandırır ve MPPI'yi bayat
        pozla besler (md 3.3.1.1 istemsiz-hareket riski). Köprü bağlantının
        YÜKSELEN KENARINDA bir kez hız ister; 1 Hz'lik state akışında istek
        tekrarlanmaz. FC parametrelerine YAZMAZ (EEPROM'a dokunulmaz) —
        yalnız oturum boyunca geçerli REQUEST_DATA_STREAM gönderilir.
        """
        return (
            self._last is not None
            and self._last.connected
            and not self._stream_rate_requested
        )

    def note_stream_rate_requested(self) -> None:
        """Akış hızı isteği gönderildi (bağlantı kopana kadar tekrarlanmaz)."""
        self._stream_rate_requested = True

    def note_stream_rate_failed(self) -> None:
        """İstek başarısız (servis hatası) → sonraki state mesajında yeniden dene."""
        self._stream_rate_requested = False

    # ----- görev-aktif bayrağı — F14.3 -----

    def set_mission_state(self, state_name: str) -> None:
        """FSM durum etiketini (/girdap/mission/state) görev-aktife çevir."""
        self._mission_active = state_name in MISSION_ACTIVE_STATES

    @property
    def mission_active(self) -> bool:
        return self._mission_active

    # ----- heartbeat -----

    def seconds_since_update(self, now: float) -> float:
        """Son state'ten bu yana geçen süre; hiç state yoksa +∞."""
        if self._last is None:
            return float("inf")
        return max(0.0, float(now) - self._last.t)

    def heartbeat_alive(self, now: float) -> bool:
        """Heartbeat zaman aşımına uğramadı mı?"""
        return self.seconds_since_update(now) <= self._cfg.heartbeat_timeout_s

    # ----- kararlar -----

    def needs_mode_change(self) -> bool:
        """GUIDED'e geçiş gerekiyor mu?

        Koşullar: görev aktif (F14.3) + bağlı + mod hedeften farklı. Görev
        aktif değilken (öncesi/sonrası/KILL) operatörün mod seçimi zorlanmaz.
        """
        if not self._mission_active:
            return False
        if self._last is None or not self._last.connected:
            return False
        return self._last.mode != self._cfg.target_mode

    def control_gate(self, now: float) -> ControlGate:
        """Güncel state'e göre güvenli kontrol kararını döndür.

        Öncelik: heartbeat > bağlantı > arm > mod. İlk ihlal eden kural
        kazanır; hiçbiri ihlal etmezse ACTIVE.
        """
        # 1) Heartbeat (hiç state yoksa da burada KILL)
        if self._last is None or not self.heartbeat_alive(now):
            dt = self.seconds_since_update(now)
            return ControlGate(
                GateState.KILL, False, True,
                f"heartbeat {dt:.1f}s > {self._cfg.heartbeat_timeout_s:.1f}s",
            )
        # 2) FCU bağlantısı
        if not self._last.connected:
            return ControlGate(GateState.KILL, False, True, "FCU baglantisi yok")
        # 3) Failsafe: arm
        if not self._last.armed:
            return ControlGate(GateState.DISARMED, False, True, "arm degil")
        # 4) Mod geçidi
        if self._last.mode != self._cfg.target_mode:
            return ControlGate(
                GateState.NOT_GUIDED, False, False,
                f"mod {self._last.mode} != {self._cfg.target_mode}",
            )
        # 5) Tam yetki
        return ControlGate(GateState.ACTIVE, True, False, "aktif")
