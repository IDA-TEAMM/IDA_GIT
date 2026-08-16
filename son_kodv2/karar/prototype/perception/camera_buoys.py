"""Kamera duba sınıf kimliği SÖZLEŞMESİ (`/perception/buoys` class_id).

🔴 16.08.2026 — HSV/CLAHE tespit hattı BURADAN KALDIRILDI.
Gerekçe: bu modül Sprint-2'de, ortada eğitilmiş model yokken yazılmış bir
yedekti (`YoloInference` mock'tu). Bugün algıyı **algı ekibi** yapıyor, tek
OAK'ı onlar tutuyor (`use_onboard_camera=false`) ⇒ bu node'a kare hiç
ulaşmıyordu. İki ayrı OpenCV hattının yan yana durması karışıklık üretiyor.
Parkur-3 hedefi için saf OpenCV tespiti artık: **`girdap-ida-p3/p3_hedef/hedef_bul.py`**
(kırmızı/yeşil/**siyah**; siyah burada HİÇ YOKTU — şartnamenin RAL 9005'i
görülemiyordu). Eşikleri 600 sentetik hedef karesiyle ölçülerek oturtuldu.

Taşınan ölçülmüş bilgi (kaybolmasın diye): CLAHE + **doygunluk germesi**
(F-P.21, 16.07 gerçek donanım: akşamüstü ışıkta duba S≈29-83 okudu, sabit eşik
120'nin altında kalıp hiç tespit edilemedi) + kırmızının iki H aralığı gerektirmesi.

Burada YALNIZ sözleşme kalıyor: `fusion.py`, `kamikaze_hedef.py` ve testler
sınıf kimliklerini buradan okuyor.
"""
from __future__ import annotations

from dataclasses import dataclass

# Sınıf kimlikleri — /perception/buoys class_id sözleşmesi
CLASS_PARKUR_KENARI = 0     # turuncu
CLASS_ENGEL = 1             # sarı
CLASS_HEDEF = 2             # Parkur-3 hedef (YOLO)
CLASS_KIRMIZI = 3           # kırmızı
CLASS_YESIL = 4             # yeşil
CLASS_KAHVERENGI = 5        # kahverengi


@dataclass
class Detection:
    """Tek tespit — bbox merkezi + boyut (piksel), sınıf, güven skoru."""

    center_x: float
    center_y: float
    width: float
    height: float
    class_id: int
    score: float
