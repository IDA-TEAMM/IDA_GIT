"""
Girdap İDA — Parkur geçiş logic'i (Sprint 4, ROS-bağımsız çekirdek).

Yarışma görev hattı Parkur-1 → Parkur-2 → Parkur-3 → tamamlanma. Geçiş
KURALI **waypoint-index tabanlı**: her parkurun SON waypoint'ine varılınca bir
sonraki parkura geçilir (Şartname: sıralı geçiş; ⚠ duba sayısına bağlı akış
tasarlamak YASAK — sadece waypoint dizisi + parkur etiketi).

Bu katman mevcut MissionFSM'in (BOOT/ARM/.../KILL) ÜSTÜNE oturur, onu
değiştirmez: MissionFSM görev yaşam döngüsü + güvenliği, ParkurTransitionLogic
ise hangi parkurda olduğumuzu waypoint ilerlemesinden türetir.

Geçişler TEK YÖNLÜ (Şartname: tek seferde sırayla, geri dönüş yok):
    PARKUR_1 ──parkur-1 son wp──→ PARKUR_2 ──parkur-2 son wp──→ PARKUR_3
    PARKUR_3 ──IMU çarpma onayı──→ COMPLETED   (impact = Sprint 5 placeholder)

Parkur-3 tamamlanması waypoint DEĞİL, IMU çarpma tespitiyle olur (Şartname:
kamikaze). Şimdilik `confirm_impact()` dışarıdan (placeholder topic) çağrılır;
Sprint 5'te IMU şok kanalı besleyecek.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
from typing import Dict, List, Optional, Tuple

import yaml


class ParkurState(Enum):
    """Parkur ilerleme durumu (waypoint-index tabanlı)."""

    PARKUR_1 = "PARKUR_1"
    PARKUR_2 = "PARKUR_2"
    PARKUR_3 = "PARKUR_3"
    COMPLETED = "COMPLETED"


# ParkurState → parkur numarası (COMPLETED son parkurda sayılır).
_STATE_PARKUR_NO: Dict[ParkurState, int] = {
    ParkurState.PARKUR_1: 1,
    ParkurState.PARKUR_2: 2,
    ParkurState.PARKUR_3: 3,
    ParkurState.COMPLETED: 3,
}


@dataclass(frozen=True)
class WaypointInfo:
    """Bir waypoint'in parkur bağlamı."""

    index: int
    parkur: int
    is_last_of_parkur: bool


def build_waypoint_infos(parkur_labels: List[int]) -> List[WaypointInfo]:
    """Parkur etiketleri listesinden her waypoint için WaypointInfo üretir.

    `is_last_of_parkur`: o parkurun listede EN SON göründüğü index. Waypoint'ler
    parkur sırasına göre monoton dizilir (contiguous blok) varsayımı — Şartname
    sıralı geçiş; bu yüzden "son görülen index" = "o parkurun son waypoint'i".
    """
    last_index: Dict[int, int] = {}
    for i, parkur in enumerate(parkur_labels):
        last_index[parkur] = i
    return [
        WaypointInfo(index=i, parkur=parkur, is_last_of_parkur=(last_index[parkur] == i))
        for i, parkur in enumerate(parkur_labels)
    ]


def load_parkur_labels(path: str) -> List[int]:
    """Görev dosyasından parkur etiketlerini çıkarır — TEK İZOLE PARSER.

    Görev formatı değişince yalnız bu fonksiyon değişir (node'lar ve
    ParkurTransitionLogic aynı kalır). Eksik `parkur` alanı → 1 (tek parkur;
    video görevi bozulmadan çalışır).
    """
    with open(path, "r", encoding="utf-8") as fh:
        data = yaml.safe_load(fh) or {}
    return [int(w.get("parkur", 1)) for w in data.get("waypoints", [])]


class ParkurTransitionLogic:
    """Waypoint-index tabanlı parkur geçiş otomatı (tek yönlü)."""

    def __init__(self, parkur_labels: Optional[List[int]] = None) -> None:
        labels = [int(p) for p in (parkur_labels or [])]
        self._infos = build_waypoint_infos(labels)
        # Her parkurun son waypoint index'i (yalnız listede bulunan parkurlar).
        self._last_index_of: Dict[int, int] = {
            info.parkur: info.index for info in self._infos if info.is_last_of_parkur
        }
        self._state = ParkurState.PARKUR_1
        self._impact_confirmed = False
        self._history: List[Tuple[ParkurState, ParkurState, str]] = []

    # ----- sorgu -----

    @property
    def state(self) -> ParkurState:
        return self._state

    @property
    def current_parkur(self) -> int:
        """Mevcut parkur numarası (1/2/3; COMPLETED → 3)."""
        return _STATE_PARKUR_NO[self._state]

    @property
    def is_complete(self) -> bool:
        return self._state is ParkurState.COMPLETED

    @property
    def impact_confirmed(self) -> bool:
        return self._impact_confirmed

    @property
    def waypoint_infos(self) -> List[WaypointInfo]:
        return list(self._infos)

    @property
    def last_index_of_parkur(self) -> Dict[int, int]:
        return dict(self._last_index_of)

    @property
    def history(self) -> List[Tuple[ParkurState, ParkurState, str]]:
        """Sıralı (eski, yeni, gerekçe) — log/replay için."""
        return list(self._history)

    def _has_parkur(self, parkur_no: int) -> bool:
        return parkur_no in self._last_index_of

    # ----- geçiş tetikleri -----

    def current_waypoint_reached(self, index: int) -> ParkurState:
        """Bir waypoint'e varıldı sinyali. Son-of-parkur ise ileri geç.

        Tek yönlü: yalnız mevcut parkurun son waypoint'i + bir sonraki parkurun
        varlığı geçişi tetikler. Erken/geç index veya olmayan parkur → no-op
        (tek parkurlu video görevi PARKUR_1'de kalır, bozulmaz).
        """
        if self._state is ParkurState.PARKUR_1:
            if self._has_parkur(2) and index == self._last_index_of.get(1):
                self._transition(
                    ParkurState.PARKUR_2, f"Parkur-1 son waypoint (idx {index})"
                )
        elif self._state is ParkurState.PARKUR_2:
            if self._has_parkur(3) and index == self._last_index_of.get(2):
                self._transition(
                    ParkurState.PARKUR_3, f"Parkur-2 son waypoint (idx {index})"
                )
        # PARKUR_3 → COMPLETED waypoint ile DEĞİL, confirm_impact() ile.
        return self._state

    def confirm_impact(self) -> ParkurState:
        """Parkur-3 çarpma onayı (Sprint 5 IMU placeholder) → COMPLETED.

        Yalnız PARKUR_3'te etkili; erken çağrı durumu bozmaz (tek yönlü).
        """
        self._impact_confirmed = True
        if self._state is ParkurState.PARKUR_3:
            self._transition(ParkurState.COMPLETED, "IMU çarpma onaylandı")
        return self._state

    # ----- iç -----

    def _transition(self, new_state: ParkurState, reason: str) -> None:
        old = self._state
        self._history.append((old, new_state, reason))
        self._state = new_state


def _demo() -> None:
    """Sentetik geçiş zinciri — parkur_fsm mantığını gösterir."""
    labels = [1, 1, 2, 2, 3]
    logic = ParkurTransitionLogic(labels)
    print(f"parkur etiketleri: {labels}")
    print(f"parkur son index'leri: {logic.last_index_of_parkur}")
    for idx in range(len(labels)):
        before = logic.state
        logic.current_waypoint_reached(idx)
        if logic.state is not before:
            print(f"  wp {idx} → {before.value} → {logic.state.value}")
    logic.confirm_impact()
    print(f"  impact → {logic.state.value}")
    print("geçiş geçmişi:")
    for old, new, reason in logic.history:
        print(f"   {old.value:10s} → {new.value:10s}  ({reason})")


if __name__ == "__main__":
    _demo()
