"""F-K.1 — kontrol döngüsü süre ölçer (ROS-BAĞIMSIZ çekirdek).

Teşhis (2026-08-16): §0.12b'de kontrol döngüsünün 10 Hz yerine **~5 Hz**
koştuğu sahada ölçüldü, ama kod bunu kendi başına HİÇ ölçmüyordu — depoda
`perf_counter|elapsed|overrun` araması **0 sonuç** veriyordu. rclpy timer'ı
geç kalırsa kuyruğa girmez, **sessizce seyrelir**: tıkanma ne loglanıyor ne
sayılıyor, yalnız dışarıdan `ros2 topic hz` ile görülebiliyordu. Sahada SSH
yok (md 4.1) ⇒ görünmeyen arıza yok sayılır.

**İKİ AYRI ŞEY ölçülür ve ikisi de gerekir:**

| ölçüm | anlamı |
|---|---|
| `is_s` | callback İÇİNDE geçen süre → *bizim* maliyetimiz |
| `aralik_s` | iki callback ARASI süre → **GERÇEK kontrol hızı** |

Fark kritik: aynı executor'daki başka bir iş (yerel harita yayını, `plan()`)
bizi geciktirirse `is_s` küçük kalır ama `aralik_s` büyür. **Tekneyi durduran
şey `aralik_s`'tir** — ArduPilot GUIDED modda 3 sn komut gelmezse aracı
durdurur (ardupilot.org/dev/docs/mavlink-rover-commands.html) ve bunu
kimseye söylemez.

ROS'suz olması bilinçli: sayma mantığı rclpy olmadan test edilebilmeli,
yoksa "eklendi" demek kanıtsız kalır (bkz. `tests/test_dongu_olcumu.py`).
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Optional

# ArduPilot GUIDED komut zaman aşımı (s). Bu eşiğe YAKLAŞAN tek bir boşluk
# bile görev bitirir; yarısında uyarmak için pay bırakılır.
ARDUPILOT_DURDURMA_S = 3.0
BOSLUK_ALARM_S = 1.5

# Tek tük aşım normaldir (GC, işletim sistemi planlayıcısı) — kalıcı seyrelme
# değildir. Bu eşiklerin altında INFO, üstünde WARN yazılır.
ASIM_ORAN_ESIGI = 10.0        # %
HIZ_ORAN_ESIGI = 0.8          # gerçek/hedef


@dataclass(frozen=True)
class DonguRaporu:
    """Bir rapor penceresinin özeti."""

    gercek_hz: float
    hedef_hz: float
    is_maks_s: float
    aralik_maks_s: float
    asim: int
    ornek: int

    @property
    def asim_oran(self) -> float:
        return 100.0 * self.asim / max(1, self.ornek)

    @property
    def saglikli(self) -> bool:
        return (self.gercek_hz >= HIZ_ORAN_ESIGI * self.hedef_hz
                and self.asim_oran <= ASIM_ORAN_ESIGI)

    @property
    def bosluk_alarmi(self) -> bool:
        """Tek bir aralık ArduPilot eşiğine tehlikeli yaklaştı mı."""
        return self.aralik_maks_s > BOSLUK_ALARM_S

    def ozet(self) -> str:
        return (
            f"kontrol döngüsü: GERÇEK {self.gercek_hz:.1f} Hz "
            f"(hedef {self.hedef_hz:.0f}) "
            f"| iş maks {self.is_maks_s * 1e3:.0f} ms "
            f"| aralık maks {self.aralik_maks_s * 1e3:.0f} ms "
            f"| bütçe aşımı {self.asim}/{self.ornek} (%{self.asim_oran:.0f})"
        )

    def bosluk_mesaji(self) -> str:
        return (
            f"kontrol döngüsünde {self.aralik_maks_s:.2f} sn boşluk — "
            f"ArduPilot {ARDUPILOT_DURDURMA_S:.0f} sn'de tekneyi DURDURUR"
        )


class DonguOlcer:
    """Callback süresi + gerçek periyot sayacı; periyodik özet üretir.

    Kullanım (`planning_node._on_control_step`):

        t0 = time.perf_counter()
        try:
            ...
        finally:
            rapor = self._olcer.kaydet(t0, time.perf_counter())
            if rapor is not None:
                ...  # logla

    `kaydet` yalnız rapor penceresi dolduğunda `DonguRaporu` döner, yoksa
    `None` — çağıran her adımda log basmaz.
    """

    def __init__(self, hedef_hz: float, rapor_periyot_s: float = 10.0) -> None:
        if hedef_hz <= 0.0:
            raise ValueError("hedef_hz > 0 olmalı")
        self._butce_s = 1.0 / hedef_hz
        self._hedef_hz = hedef_hz
        self._rapor_periyot_s = rapor_periyot_s
        self._onceki_t: Optional[float] = None
        self._rapor_t: Optional[float] = None
        self._sifirla()

    def _sifirla(self) -> None:
        self._sayi = 0
        self._asim = 0
        self._is_maks = 0.0
        self._aralik_maks = 0.0
        self._aralik_top = 0.0
        self._aralik_sayi = 0

    def kaydet(self, t_giris: float, t_cikis: float) -> Optional[DonguRaporu]:
        is_s = t_cikis - t_giris
        self._sayi += 1
        if is_s > self._is_maks:
            self._is_maks = is_s
        if is_s > self._butce_s:
            self._asim += 1

        if self._onceki_t is not None:
            aralik = t_giris - self._onceki_t
            self._aralik_top += aralik
            self._aralik_sayi += 1
            if aralik > self._aralik_maks:
                self._aralik_maks = aralik
        self._onceki_t = t_giris

        # İlk çağrı yalnız zamanı kurar — tek örnekle hız hesaplamak anlamsız.
        if self._rapor_t is None:
            self._rapor_t = t_cikis
            return None
        if t_cikis - self._rapor_t < self._rapor_periyot_s:
            return None

        ort_aralik = self._aralik_top / max(1, self._aralik_sayi)
        rapor = DonguRaporu(
            gercek_hz=(1.0 / ort_aralik) if ort_aralik > 0.0 else 0.0,
            hedef_hz=self._hedef_hz,
            is_maks_s=self._is_maks,
            aralik_maks_s=self._aralik_maks,
            asim=self._asim,
            ornek=self._sayi,
        )
        self._rapor_t = t_cikis
        # Sıfırlanmazsa maks değerler tüm koşuya yapışır ve sağlığa dönmüş
        # bir sistemi hâlâ arızalı gösterir.
        self._sifirla()
        return rapor
