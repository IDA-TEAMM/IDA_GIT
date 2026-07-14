"""
Girdap İDA — Video waypoint görev yöneticisi (ROS-bağımsız çekirdek).

Otonomi Kabiliyeti videosu senaryosu: 4 GPS waypoint dikdörtgen + başlangıca
dönüş. Bu çekirdek durum makinesini, varış/bekleme (arrival/dwell) mantığını ve
lat/lon → local ENU dönüşümünü içerir. Layer 2 `mission_manager_node` sarar:
GPS besler, `/girdap/mission/current_target` yayınlar. rclpy bağımsız →
pytest ile .venv altında doğrulanır.

Durum makinesi:
    IDLE  → ACTIVE   : start() (FSM görevi başlattığında)
    ACTIVE → DWELL   : hedefe arrival_radius_m kadar yaklaşınca
    DWELL → ACTIVE   : dwell_time_s dolunca index++ (yeni hedef)
    DWELL → COMPLETE : son waypoint'te dwell dolunca

Konum referansı current pose'a görelidir (mavros home bağımlılığı yok — video
için yeterli). update() her çağrıda güncel hedefe ENU ofsetini döndürür.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from enum import Enum, auto
from typing import List, Optional, Sequence, Tuple

# WGS-84 ekvatoral yarıçap (fusion pipeline ile aynı sabit).
_EARTH_R = 6378137.0


class MissionPhase(Enum):
    """Video görev durum makinesi evreleri."""

    IDLE = auto()       # görev başlamadı
    ACTIVE = auto()     # hedefe seyir
    DWELL = auto()      # waypoint'te bekleme
    COMPLETE = auto()   # tüm waypoint'ler tamamlandı


@dataclass(frozen=True)
class Waypoint:
    lat: float
    lon: float
    name: str = ""
    parkur: int = 1        # yarışma parkur etiketi (1/2/3); video görevi → 1


@dataclass(frozen=True)
class MissionManagerConfig:
    arrival_radius_m: float = 2.0
    dwell_time_s: float = 2.0
    cruise_velocity_mps: float = 1.0


def latlon_to_enu(
    lat0: float, lon0: float, lat: float, lon: float
) -> Tuple[float, float]:
    """(lat0,lon0) → (lat,lon) yerel ENU ofseti (east, north) metre.

    Küçük mesafelerde (video ölçeği) equirectangular yaklaşım; büyük-çember
    (haversine) mesafesiyle ~cm farkı. east = doğu (+x), north = kuzey (+y).
    """
    lat0_rad = math.radians(lat0)
    east = math.radians(lon - lon0) * _EARTH_R * math.cos(lat0_rad)
    north = math.radians(lat - lat0) * _EARTH_R
    return east, north


# --------------------------------------------------------------------------- #
# FC (MAVLink) görev listesi → Waypoint dönüşümü (T0-f)
#
# Şartname md 3.3.1(2) + md 5.5.2.2: görev YKİ'de tanımlanıp İDA'ya YÜKLENİR
# (araç üstü YAML bunu karşılamaz). QGC → Pixhawk → MAVROS zinciriyle yüklenen
# görev /mavros/mission/waypoints (mavros_msgs/WaypointList) üzerinden gelir.
# Bu çekirdek dönüşüm mavros_msgs'e bağımlı DEĞİL — Layer 2 node ilgili alanları
# çıkarıp FcMissionItem'a koyar; böylece pytest ile (mavros'suz) doğrulanır.
# --------------------------------------------------------------------------- #

# MAVLink MAV_CMD gezinme komutları — yalnız bunlar birer görev NOKTASIDIR.
# DO_*/koşul/RTL/JUMP item'larının konumu yok ya da anlamsız → atlanır.
MAV_CMD_NAV_WAYPOINT = 16
MAV_CMD_NAV_SPLINE_WAYPOINT = 82
_NAV_COMMANDS = frozenset({MAV_CMD_NAV_WAYPOINT, MAV_CMD_NAV_SPLINE_WAYPOINT})


@dataclass(frozen=True)
class FcMissionItem:
    """MAVLink görev item'ının konum alt kümesi (mavros_msgs bağımsız).

    seq     — WaypointList içindeki index; ArduPilot'ta index 0 = home konumu.
    command — MAV_CMD kodu (gezinme filtresi için).
    lat/lon — mavros_msgs/Waypoint.x_lat / .y_long (derece).
    """

    seq: int
    command: int
    lat: float
    lon: float


def fc_items_to_waypoints(
    items: Sequence[FcMissionItem],
    *,
    skip_home_seq0: bool = True,
) -> List[Waypoint]:
    """FC görev listesini MissionManager Waypoint'lerine çevirir.

    Filtreler (sırayla):
      1. skip_home_seq0 ise index 0 (ArduPilot home konumu) atlanır — gerçek
         görev noktası değildir, QGC görevin başına otomatik ekler.
      2. Gezinme komutu değilse (NAV_WAYPOINT/NAV_SPLINE_WAYPOINT dışı) atlanır.
      3. lat==lon==0 (tanımsız / DO_ item'ı) atlanır.
    Parkur etiketi FC'den GELMEZ → hepsi parkur=1 (video senaryosu tek parkur).
    """
    wps: List[Waypoint] = []
    for it in items:
        if skip_home_seq0 and it.seq == 0:
            continue
        if it.command not in _NAV_COMMANDS:
            continue
        if it.lat == 0.0 and it.lon == 0.0:
            continue
        wps.append(
            Waypoint(lat=float(it.lat), lon=float(it.lon), name=f"FC{it.seq}", parkur=1)
        )
    return wps


def farthest_waypoint_m(
    lat: float, lon: float, waypoints: Sequence[Waypoint]
) -> float:
    """Mevcut konumdan en uzak waypoint'e ENU mesafesi (m); boş liste → 0.

    F-M.1 makullük kontrolü: fix'siz/yanlış konum + gerçek koordinatlı görev
    binlerce km'lik hedef üretir (masa olayı: (0,0) → 40°K/29°D ≈ 4400 km →
    MPPI referansı 92 GB tensöre şişti). Görev başlatılmadan ÖNCE çağrılır.
    """
    best = 0.0
    for wp in waypoints:
        east, north = latlon_to_enu(lat, lon, wp.lat, wp.lon)
        d = math.hypot(east, north)
        if d > best:
            best = d
    return best


class MissionManager:
    """Video waypoint görev durum makinesi (arrival + dwell)."""

    def __init__(
        self,
        waypoints: List[Waypoint],
        config: Optional[MissionManagerConfig] = None,
    ) -> None:
        self._wps = list(waypoints)
        self._cfg = config or MissionManagerConfig()
        self._phase = MissionPhase.IDLE
        self._idx = 0
        self._dwell_start: Optional[float] = None

    # ----- kontrol -----

    def start(self) -> None:
        """IDLE → ACTIVE (waypoint varsa). Tekrar çağrı etkisiz."""
        if self._phase is MissionPhase.IDLE and self._wps:
            self._phase = MissionPhase.ACTIVE
            self._idx = 0
            self._dwell_start = None

    def update(
        self, lat: float, lon: float, now: float
    ) -> Optional[Tuple[float, float]]:
        """GPS fix + zaman → güncel hedefe ENU ofseti (east, north).

        Durum geçişlerini işler. IDLE/COMPLETE'te None döner.
        """
        if self._phase in (MissionPhase.IDLE, MissionPhase.COMPLETE):
            return None

        wp = self._wps[self._idx]
        east, north = latlon_to_enu(lat, lon, wp.lat, wp.lon)
        dist = math.hypot(east, north)

        if self._phase is MissionPhase.ACTIVE:
            if dist <= self._cfg.arrival_radius_m:
                self._phase = MissionPhase.DWELL
                self._dwell_start = now

        elif self._phase is MissionPhase.DWELL:
            assert self._dwell_start is not None
            if now - self._dwell_start >= self._cfg.dwell_time_s:
                if self._idx + 1 >= len(self._wps):
                    self._phase = MissionPhase.COMPLETE
                    return None
                self._idx += 1
                self._phase = MissionPhase.ACTIVE
                self._dwell_start = None
                wp = self._wps[self._idx]
                east, north = latlon_to_enu(lat, lon, wp.lat, wp.lon)

        return east, north

    # ----- sorgu -----

    @property
    def phase(self) -> MissionPhase:
        return self._phase

    @property
    def current_index(self) -> int:
        return self._idx

    @property
    def waypoint_count(self) -> int:
        return len(self._wps)

    @property
    def waypoints(self) -> List[Waypoint]:
        return list(self._wps)

    @property
    def current_waypoint(self) -> Optional[Waypoint]:
        if self._phase in (MissionPhase.IDLE, MissionPhase.COMPLETE):
            return None
        return self._wps[self._idx]

    @property
    def is_complete(self) -> bool:
        return self._phase is MissionPhase.COMPLETE
