"""Girdap İDA — `cmd_vel` EĞİM SINIRLAYICI (ROS-bağımsız çekirdek).

14.08.2026, F-F.18 (GIRDAP_DURUM §0.99u). **Ölçümden doğdu, tahminden değil.**

🔴 NEDEN VAR — ÖLÇÜLEN ARIZA
--------------------------------
14.08 su koşumunda (`session_20260814_153256`, GUIDED, 1 879 örnek) uçuş
kontrolcüsüne gönderdiğimiz hız komutu **fiziksel olarak takip edilemez**
çıktı:

    ardışık `cmd_vel` farkı : ortanca 0,074 · %90 0,291 · **azami 0,982 m/s**
    (10 Hz'te, yani saniyede ~1 m/s'lik sıçrama)
    komut 0,4 m/s'nin altında kesintisiz kalma süresi: **ortanca 0,2 s**

Sonuç: düşük hız komutunda araç **iki katı** gidiyordu (istenen 0,22-0,25 →
gerçekleşen 0,47-0,56 m/s). Açığın kaynağı ayrıştırıldı:

    komut <0,4'te geçen süre | açık
    0-1 s                    | +0,337 m/s
    1-2 s                    | +0,105 m/s
    **2-4 s**                | **−0,047 m/s**  ← açık YOK

Yani uçuş kontrolcüsü iyi takip ediyor; **komut yerinde durmuyor.**
`ATC_SPEED_P/I` süpürmesi (60 kombinasyon, `scripts/hiz_supurme.py`) bunun
PI ayarıyla kapanmadığını gösterdi: tüm ızgarada kazanç **0,012 m/s**.

🔑 SINIRIN DEĞERİ NEREDEN GELİYOR
----------------------------------
Uydurulmadı — **iki bağımsız yol aynı sayıyı verdi**:

1. Aynı koşumun hız serisinden **doğrudan ölçüm** (0,5 s pencerede):
   hızlanma %90 = 0,46 · **%99 = 0,87-0,95** · azami 0,95-1,11 m/s²
   yavaşlama %90 = 0,47 · %99 = 0,80-0,84 · azami 0,84-1,01 m/s²
2. Uçuş kontrolcüsü hız çevrimi modelinin gerçek veriye **fit'i**
   (`hiz_supurme.py`): `a_max = 0,80 m/s²`

→ varsayılan **0,8 m/s²**. Ayrıca `ATC_ACCEL_MAX = 1,0`'ın ALTINDA kalır,
yani uçuş kontrolcüsünün kendi eğim sınırıyla yarışmaz.

⚠ AÇISAL EKSEN VARSAYILAN OLARAK **KAPALI** (`0.0`). Sebep: yaw ivmesi
**ölçülmedi**. `dynamics.yaml`'ın `inertia_z`'si CFD değeri ve dosyanın kendi
notu *"suda doğrulanmadı"* diyor — ölçülmemiş bir sayıdan sınır türetmek bu
deponun kuralına aykırı. Ölçülünce açılır.

🛟 GÜVENLİK SÖZLEŞMESİ — EN ÖNEMLİ MADDE
-----------------------------------------
`planning_node`'daki **bütün bekçiler `u = zeros(2)` yazarak durur**
(`DISARM-VEYA-KILL`, `POZ-SACMA`, `POZ-BAYAT`, `ENGEL-BAYAT`, kontrol adımı
çökmesi). Bu çekirdek o yolları **ASLA yavaşlatmamalıdır** — yoksa eğim
sınırlayıcı, deponun bütün güvenlik kapılarını sakatlar.

Bu yüzden sözleşme **çağıranda**dır: acil/bekçi kaynaklı sıfır komutu
sınırlayıcıya **hiç uğramaz** ve `sifirla()` çağrılır. Bu modül "acil mi"
karar vermez; karar bilgisi (`sebepler`, `gate.zero_thrust`) yalnız node'da
vardır ve orada verilir. Testler ikisini birden dondurur.
"""
from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class EgimSinirlayiciConfig:
    """Eksen başına ivme tavanı. `<= 0` → o eksen sınırlanmaz (kapalı)."""

    azami_ivme_mps2: float = 0.8
    azami_acisal_ivme_rps2: float = 0.0
    #: İki çağrı arası en fazla bu kadar süre "gerçek" sayılır. Daha uzun
    #: boşlukta (yığın donması, KAR-09) tek adımda kocaman bir değişime izin
    #: vermemek için Δt buraya kırpılır — sınırlayıcı boşluktan sonra ASLA
    #: bugünkü davranıştan daha gevşek olmaz.
    azami_dt_s: float = 0.5


class EgimSinirlayici:
    """`cmd_vel` bileşenlerini ivme tavanına göre yumuşatır.

    Durum: son yayınlanan (u, r) ve zamanı. `sifirla()` durumu düşürür;
    sonraki ilk çağrı hedefi **olduğu gibi geçirir** (tohumlama).

    ⚠ Tohumlama bilinçli: `sifirla()` yalnız kontrol yolu kesildiğinde
    çağrılır (geçit kapandı / acil duruş). O anda teknenin gerçek hızı
    bilinmediği için sıfırdan rampa başlatmak yanlış olurdu — araç hâlâ
    hareket hâlindeyken "0'dan başla" demek, komutu gereksiz yere kısar.
    """

    def __init__(self, config: EgimSinirlayiciConfig | None = None) -> None:
        self._cfg = config or EgimSinirlayiciConfig()
        self._son_u: float | None = None
        self._son_r: float | None = None
        self._son_t: float | None = None

    @property
    def config(self) -> EgimSinirlayiciConfig:
        return self._cfg

    def sifirla(self) -> None:
        """Durumu düşür — sonraki çağrı hedefi olduğu gibi geçirir."""
        self._son_u = None
        self._son_r = None
        self._son_t = None

    def uygula(self, hedef_u: float, hedef_r: float, simdi_s: float) -> tuple[float, float]:
        """Hedefi ivme tavanına göre kırp ve yayınlanacak (u, r) döndür."""
        hedef_u = float(hedef_u)
        hedef_r = float(hedef_r)

        if self._son_t is None or self._son_u is None or self._son_r is None:
            self._son_u, self._son_r, self._son_t = hedef_u, hedef_r, simdi_s
            return hedef_u, hedef_r

        dt = simdi_s - self._son_t
        if dt <= 0.0:                      # aynı damga / geri saat → değişme
            return self._son_u, self._son_r
        dt = min(dt, self._cfg.azami_dt_s)

        self._son_u = _kirp(self._son_u, hedef_u, self._cfg.azami_ivme_mps2 * dt)
        self._son_r = _kirp(self._son_r, hedef_r, self._cfg.azami_acisal_ivme_rps2 * dt)
        self._son_t = simdi_s
        return self._son_u, self._son_r


def _kirp(onceki: float, hedef: float, azami_degisim: float) -> float:
    """`azami_degisim <= 0` → sınır kapalı, hedef doğrudan geçer."""
    if azami_degisim <= 0.0:
        return hedef
    fark = hedef - onceki
    if fark > azami_degisim:
        return onceki + azami_degisim
    if fark < -azami_degisim:
        return onceki - azami_degisim
    return hedef
