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
    # 🔑 GEÇİŞ ZORUNLU — "yaklaştım" yerine "GEÇTİM" ölçütü (along-track).
    #
    # SORUN (ölçüldü, GIRDAP_DURUM §1.68): `arrival_radius_m` klasik
    # *circle of acceptance*tır — araç çembere girince "vardım" der ve sonraki
    # noktaya döner; noktanın ÜZERİNE gitmez. Şartname ise kapı geçişini
    # **"İDA'nın duba ikilisinin %100'ünü geçmiş olması"** diye tanımlıyor ve
    # algı tarafı bunu DÜZLEM AŞMA ile sayıyor
    # (`duba_gecis_navigator.PASS_EK_YOL` = ARAC_BOY 1,03 + 0,5 = **1,53 m**).
    # İki taraf aynı olayı farklı tanımlıyor; açık = 2,0 + 1,53 = **3,53 m**
    # ve DWELL de kapatmıyor (dwell boyunca hedef aynı kalıyor ama dur komutu
    # yok: ~0,5 m/s × 2 s ≈ 1 m). Ölçülen "en ileri −0,72 m" bu aralığa düşer.
    #
    # ÇÖZÜM: varışa ikinci koşul — araç, noktadan geçen ve BACAK yönüne dik
    # düzlemi aşmış olmalı: `(p − wp)·t̂ > 0`. Literatürde *along-track/travel*
    # koşulu; **ArduRover 4.3+ kendi waypoint tamamlamasını zaten böyle yapıyor**
    # (`WP_RADIUS` AUTO'da etkisiz — ArduPilot #23457). Bizdeki sürüm V4.6.3.
    #
    # ⚠ False = ESKİ DAVRANIŞ BİREBİR (varsayılan). Ölçülmeden açılmaz.
    gecis_zorunlu: bool = False
    # Kilitlenme yedeği: araç yarıçapın İÇİNDE bu kadar saniye kalıp düzlemi
    # hâlâ aşamadıysa varış yine kabul edilir ve sayaç artar (sessiz düşmez).
    # Gerekli, çünkü nokta kapının ötesinde değilse ya da araç geçemiyorsa
    # görev sonsuza kadar takılırdı. 0 = yedek YOK (takılma serbest).
    gecis_zaman_asimi_s: float = 5.0
    # 📐 GEÇİŞ PAYI (19.08.2026) — düzlemi NE KADAR aşmış sayılmalı.
    # `_gecti` ölçütü `(p − wp)·t̂ > 0` idi: REFERANS NOKTA düzlemi geçer
    # geçmez "geçti" sayılıyordu. Şartname ise **İDA'nın %100'ünün** geçmesini
    # istiyor (md: "birinci duba ikilisinin %100'ünü geçmiş olması"), algı
    # tarafı da bunu `PASS_EK_YOL` = ARAC_BOY 1,03 + 0,5 = **1,53 m** ile
    # sayıyor. İki taraf aynı olayı farklı eşikle sayarsa geçit "sayıldı" ama
    # puanlanmaz. ÖLÇÜLDÜ (12 farklı parkur süpürmesi, `gecis_supurme.py`):
    # pay 0 iken ortanca aşma **+1,32 m** — eşiğin 21 cm altında.
    # ⚠ 0.0 = ESKİ DAVRANIŞ BİREBİR.
    gecis_payi_m: float = 0.0


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


def fc_items_to_waypoints_with_seqs(
    items: Sequence[FcMissionItem],
    *,
    skip_home_seq0: bool = True,
) -> Tuple[List[Waypoint], List[int]]:
    """FC görev listesini Waypoint'lere çevirir + tutulanların FC seq'lerini döndürür.

    Filtreler (sırayla):
      1. skip_home_seq0 ise index 0 (ArduPilot home konumu) atlanır — gerçek
         görev noktası değildir, QGC görevin başına otomatik ekler.
      2. Gezinme komutu değilse (NAV_WAYPOINT/NAV_SPLINE_WAYPOINT dışı) atlanır.
      3. lat==lon==0 (tanımsız / DO_ item'ı) atlanır.
    Parkur etiketi FC'den GELMEZ → hepsi parkur=1 (video senaryosu tek parkur).

    Seq listesi F-V.8 için: FC'nin MISSION_ITEM_REACHED'i wp_seq (FC dizisi)
    verir; bizim index'e çevirmek için AYNI filtreden geçmiş eşleme gerekir —
    filtre mantığı burada tek yerde kalır (kopya = sessiz ayrışma riski).
    """
    wps: List[Waypoint] = []
    seqs: List[int] = []
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
        seqs.append(int(it.seq))
    return wps, seqs


def fc_items_to_waypoints(
    items: Sequence[FcMissionItem],
    *,
    skip_home_seq0: bool = True,
) -> List[Waypoint]:
    """Geriye-uyum sarmalayıcı — bkz. fc_items_to_waypoints_with_seqs."""
    wps, _ = fc_items_to_waypoints_with_seqs(items, skip_home_seq0=skip_home_seq0)
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
        # `gecis_zorunlu` durumu — çembere ilk girişteki yaklaşma yönü (idx=0'da
        # bacak yönü yoktur) ve zaman aşımı saati + teşhis sayaçları.
        self._giris_xy: Optional[Tuple[float, float]] = None
        self._gecis_bekleme_basi: Optional[float] = None
        self._gecis_bekleyen = 0
        self._zaman_asimiyla_varilan = 0

    # ----- kontrol -----

    def start(self) -> None:
        """IDLE → ACTIVE (waypoint varsa). Tekrar çağrı etkisiz."""
        if self._phase is MissionPhase.IDLE and self._wps:
            self._phase = MissionPhase.ACTIVE
            self._idx = 0
            self._dwell_start = None
            self._gecis_durumunu_sifirla()

    def reset(self) -> None:
        """Görevi başa al — md 5.5.3.1 yeniden başlama hakkı.

        IDLE + index 0 + dwell temiz. `start()` yeniden çağrılabilir hâle
        gelir (o metot yalnız IDLE'da etkili). COMPLETE'ten de çıkar: ikinci
        tur ilk waypoint'ten başlar.
        """
        self._phase = MissionPhase.IDLE
        self._idx = 0
        self._dwell_start = None
        self._gecis_durumunu_sifirla()

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
                if self._giris_xy is None:
                    self._giris_xy = (east, north)   # araç → nokta, giriş anı
                if self._varis_kabul(lat, lon, now):
                    self._phase = MissionPhase.DWELL
                    self._dwell_start = now
                    self._gecis_durumunu_sifirla()
            else:
                # Çemberden çıkıldı: dalga/akıntı yüzünden girip çıkan araç
                # zaman aşımını BİRİKTİRMEMELİ, yaklaşma yönü de tazelenmeli.
                self._gecis_durumunu_sifirla()

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

        return self._nisan(east, north, lat, lon)

    def _nisan(
        self, east: float, north: float, lat: float, lon: float
    ) -> Tuple[float, float]:
        """Kontrolcüye verilecek nişan (ENU ofseti) — kapıda **fly-by**.

        🔴 SABİT ÖTELEME DENENDİ VE KALDIRILDI (19.08.2026). Ölçüldü:
        `2,03 m` → red2 **+1,41 m** (eşik 1,53 ⇒ 12 cm eksik) · `2,5 m` →
        **daha kötü** (geçit 0/8, red −0,54/−0,62). Mesafeyi mesafeyle yenmek
        kırılgan: belirleyici olan öteleme değil aracın **DURDUĞU** yer.
        `arrival_radius_m` (2,0 m) içine girince nişan ayağının dibinde kalıyor,
        itki sıfıra gidiyor ve araç kapının ortasında ölüyor.

        ✅ ÇÖZÜM — **fly-by**: otopilotların *fly-over* (noktaya var ve dur) ↔
        *fly-by* (noktayı geçerken sonrakine dön) ayrımı. Kapı durulacak değil
        **geçilecek** noktadır. Çemberin içindeyken düzlem henüz aşılmadıysa
        nişan **sonraki nokta** olur; araç durmaz, kapının içinden geçer ve
        düzlemi geniş payla aşar. **Ayarlanacak sayı yok.**
        (ArduRover 4.3+ waypoint tamamlamayı zaten "geçti mi" ile tetikliyor;
        `WP_RADIUS` AUTO'da etkisiz — ArduPilot #23457.)

        ⚠ SAYIM DEĞİŞMEZ: geçiş hâlâ GERÇEK waypoint düzlemine göre sayılır.
        ⚠ Yalnız `gecis_zorunlu` açıkken; kapalıyken eski davranış birebir.

        Üç yerde fly-by UYGULANMAZ:
          1) **Parkur 3** — kamikaze noktaya VARMAYI ister, geçmeyi değil.
          2) **Parkur değiştiren nokta** — P2'nin son waypoint'i bir GEÇİT
             değil DEVİR noktasıdır; orada ileri gitmek aracı P3'ün büyük
             dubasının görüş/menzil penceresinden (kamera 69°, LiDAR ~8 m)
             çıkarabilir. Kazanılacak geçit yok, kaybedilecek nişan var.
          3) **Görevin son noktası** — ötesinde sayılacak bir şey yok.
        """
        wp = self._wps[self._idx]
        son_nokta = self._idx + 1 >= len(self._wps)
        parkur_degisiyor = (
            not son_nokta and self._wps[self._idx + 1].parkur != wp.parkur
        )
        if wp.parkur == 3 or son_nokta or parkur_degisiyor:
            return east, north
        if (
            self._cfg.gecis_zorunlu
            and math.hypot(east, north) <= self._cfg.arrival_radius_m
            and not self._gecti(lat, lon)
        ):
            sonraki = self._wps[self._idx + 1]
            return latlon_to_enu(lat, lon, sonraki.lat, sonraki.lon)
        return east, north

    def _gecis_durumunu_sifirla(self) -> None:
        """Yaklaşma yönü + zaman aşımı saati (sayaçlar KALIR — teşhis)."""
        self._giris_xy = None
        self._gecis_bekleme_basi = None

    def _varis_kabul(self, lat: float, lon: float, now: float) -> bool:
        """Yarıçapa girildi — varış SAYILSIN mı?

        `gecis_zorunlu` kapalıysa evet (eski davranış birebir). Açıksa araç
        ayrıca noktanın düzlemini aşmış olmalı; aşamıyorsa `gecis_zaman_asimi_s`
        sonunda yine kabul edilir (kilitlenme yedeği) ve SAYAÇ artar.
        """
        if not self._cfg.gecis_zorunlu:
            return True
        if self._gecti(lat, lon):
            return True
        if self._gecis_bekleme_basi is None:
            self._gecis_bekleme_basi = now
            self._gecis_bekleyen += 1
        t_asimi = self._cfg.gecis_zaman_asimi_s
        if t_asimi > 0.0 and now - self._gecis_bekleme_basi >= t_asimi:
            self._zaman_asimiyla_varilan += 1
            return True
        return False

    def _gecti(self, lat: float, lon: float) -> bool:
        """Araç, noktadan geçen ve bacak yönüne dik düzlemi AŞTI mı.

        `t̂` = bacak yönü (önceki nokta → bu nokta). İlk noktada önceki yoktur;
        o zaman aracın çembere GİRDİĞİ andaki yaklaşma yönü kullanılır — düz
        yaklaşmada ikisi aynıdır ve idx=0 tanımsız kalmaz.
        Ölçüt: `(p − wp) · t̂ > gecis_payi_m` (0 = eski davranış; şartname
        teknenin TAMAMININ geçmesini ister ⇒ dağıtımda 1,53 m).
        """
        wp = self._wps[self._idx]
        e_vw, n_vw = latlon_to_enu(lat, lon, wp.lat, wp.lon)   # araç → nokta
        if self._idx > 0:
            onceki = self._wps[self._idx - 1]
            tx, ty = latlon_to_enu(onceki.lat, onceki.lon, wp.lat, wp.lon)
        elif self._giris_xy is not None:
            tx, ty = self._giris_xy                            # giriş → nokta
        else:
            return False
        n = math.hypot(tx, ty)
        if n <= 1e-6:
            # Bacak yönü tanımsız (üst üste iki nokta) ⇒ ölçüt uygulanamaz;
            # takmamak için varış kabul edilir.
            return True
        return ((-e_vw) * tx + (-n_vw) * ty) / n > self._cfg.gecis_payi_m

    @property
    def gecis_bekleyen(self) -> int:
        """`gecis_zorunlu` yüzünden varışın ertelendiği waypoint sayısı."""
        return self._gecis_bekleyen

    @property
    def zaman_asimiyla_varilan(self) -> int:
        """Düzlem aşılamadığı için ZAMAN AŞIMIYLA kabul edilen varış sayısı."""
        return self._zaman_asimiyla_varilan

    def notify_external_reached(self, idx: int) -> bool:
        """F-V.8: dış otorite (FC MISSION_ITEM_REACHED) idx'e varıldı diyor.

        AUTO'da rover köşeyi bizim arrival_radius'a girmeden dönebilir →
        kendi varış tespitimiz takılır. FC'nin sinyali index'i İLERİ senkronlar
        (yalnız ileri: geride/aralık dışı yok sayılır — bayat mesaj koruması).
        Başlamamış (IDLE) ya da bitmiş görevde etkisiz: TRANSIENT_LOCAL
        yayıncıdan sızabilecek eski koşu mesajı görevi başlatamaz/ilerletemez.

        Dönüş: durum değiştiyse True.
        """
        if self._phase in (MissionPhase.IDLE, MissionPhase.COMPLETE):
            return False
        if idx < self._idx or idx >= len(self._wps):
            return False
        if idx + 1 >= len(self._wps):
            self._phase = MissionPhase.COMPLETE
            self._dwell_start = None
            self._gecis_durumunu_sifirla()
            return True
        self._idx = idx + 1
        self._phase = MissionPhase.ACTIVE
        self._dwell_start = None
        # Index dışarıdan atladı ⇒ eski noktanın yaklaşma yönü ve zaman aşımı
        # saati ARTIK GEÇERSİZ; taşınırsa yeni noktada yanlış düzlem sınanır.
        self._gecis_durumunu_sifirla()
        return True

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
