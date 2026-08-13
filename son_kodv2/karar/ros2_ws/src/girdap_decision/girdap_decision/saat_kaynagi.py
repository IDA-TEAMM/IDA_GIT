"""
Girdap İDA — bayatlık/zaman aşımı ölçümleri için SIÇRAMAYA BAĞIŞIK saat.

🔴 NEYİ ÇÖZÜYOR (13.08.2026 canlı arıza, §0.61): Jetson'un gerçek zaman saati
(RTC) tutmuyor; sistem yanlış saatle açılıyor ve saat sonradan (GPS'ten
`girdap-saat`, ya da ağ gelirse `systemd-timesyncd`) **tek adımda** düzeltiliyor.
Bütün düğümler bayatlığı `get_clock()` ile — yani DUVAR SAATİYLE — ölçtüğü için
o adım, "son mesajdan beri geçen süre"yi bir anda sıçrama kadar büyütüyordu:

    05:52:11  systemd-timesyncd: saat +1497,6 s adımlandı
    05:52:11  mavros_bridge: FAILSAFE — heartbeat kaybı (1497.6s) → KILL

Hat kopmamıştı: `/mavros/state` `connected: true`, uçuş kontrolcüsünde TEK BİR
failsafe mesajı bile yok, füzyon 76 ms sonra "poz kaynağı geri geldi" dedi.
Aynı saniyede heartbeat 1497,6 · GPS 1496,7 · poz 1496,8 · engel 1496,7 s
bayatladı — **hepsi aynı miktarda**, ki bu veri kaybının değil saat adımının
imzasıdır. KILL mandallı olduğu için araç yığın yeniden başlatılana kadar
durdu. Aynı imza 11.08 16:05:17'de de var (1195,6 s, yine ilk NTP adımı);
kayıttaki diğer bütün KILL'ler 5,1-5,5 s, yani gerçek.

🔑 KURAL: **iki zaman ayrı işlerdir, karıştırılmaz.**

  - "Ne kadar zaman GEÇTİ?" (bayatlık, zaman aşımı, kadans, log kısma)
    → TEK YÖNLÜ saat (`time.monotonic`). Adımlanamaz, geri gitmez.
  - "Bu olay HANGİ ANDA oldu?" (mesaj damgası, CSV zaman sütunu, dosya adı)
    → duvar saati (`get_clock().now().to_msg()`, `datetime.now`). Değişmez.

⚠ SİM ZAMANI İSTİSNASI: `use_sim_time` açıkken zamanı `/clock` sürer (Gazebo,
`ros2 bag play --clock`). Orada duvar saati adımı diye bir sorun yoktur, buna
karşılık tek yönlü saat sim durdurulduğunda/hızlandırıldığında akmaya devam
eder ve bekçiler yalancı ateşler. Bu yüzden sim zamanında `get_clock()` KALIR.

Bu modül ROS'a yalnız `node` üzerinden dokunur; `rclpy` import ETMEZ, böylece
ROS'suz ortamda da (çekirdek pytest koşumu) içe aktarılabilir.
"""

from __future__ import annotations

import time
from typing import Callable, Optional

__all__ = ["bayatlik_saati", "sim_zamani_mi", "SaatSicramaBekcisi"]


def sim_zamani_mi(node) -> bool:  # noqa: ANN001
    """Düğüm sim zamanında mı koşuyor? Parametre okunamazsa False (donanım)."""
    try:
        return bool(node.get_parameter("use_sim_time").value)
    except Exception:                       # parametre yok/henüz yok → donanım
        return False


def bayatlik_saati(node) -> Callable[[], float]:  # noqa: ANN001
    """Bayatlık/zaman aşımı ölçümü için saniye döndüren saat üreteci.

    Donanımda `time.monotonic` (saat adımına bağışık), sim zamanında
    `get_clock()` (zamanı `/clock` sürer). Düğümün kurucusunda BİR KEZ çağrılıp
    `self._saat`e alınmalı — her ölçümde parametre okumak boşuna maliyettir.

    ⚠ Dönen değerin BAŞLANGICI keyfidir (donanımda makinenin açılışından beri
    geçen süre). Yalnız FARKI anlamlıdır; mutlak an olarak yazılmaz, dosya
    adına/CSV'ye konmaz, mesaj damgası yapılmaz.
    """
    if sim_zamani_mi(node):
        return lambda: node.get_clock().now().nanoseconds * 1e-9
    return time.monotonic


class SaatSicramaBekcisi:
    """Duvar saati ile tek yönlü saat arasındaki kaymayı izler.

    Sıçrama artık KILL üretmiyor (asıl düzeltme bu), ama SESSİZ de kalmamalı:
    saatin ne zaman ve ne kadar adımlandığını bilmek, kayıt damgalarını
    yorumlarken (§0.53e "SAATSIZ" işareti) ve suda "neden o an tuhaf davrandı"
    sorusunu yanıtlarken gerekiyor. Ölçüm bedeli iki `time` çağrısı.

    Kullanım: kurucuda bir örnek, izleme timer'ında `kontrol()`; None değilse
    logla. Dönen değer saniyedir, işaretlidir (+ ileri, − geri).
    """

    def __init__(self, esik_s: float = 2.0) -> None:
        self._esik_s = float(esik_s)
        self._duvar = time.time()
        self._tek_yonlu = time.monotonic()

    def kontrol(self) -> Optional[float]:
        """Son çağrıdan bu yana duvar saati tek yönlü saatten saptı mı?"""
        duvar = time.time()
        tek_yonlu = time.monotonic()
        sapma = (duvar - self._duvar) - (tek_yonlu - self._tek_yonlu)
        self._duvar = duvar
        self._tek_yonlu = tek_yonlu
        if abs(sapma) >= self._esik_s:
            return sapma
        return None
