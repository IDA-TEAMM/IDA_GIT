"""
Girdap İDA — Sentetik kamera frame üreticisi (Sprint 2 test fixture).

Gerçek OAK-D verisi yokken pipeline'ı beslemek için su arka planı üzerine
turuncu/sarı duba daireleri çizer. Senaryolar:
    - scene_camera_minimum: 2 turuncu + 1 sarı, temiz      → temel segmentasyon
    - scene_camera_orta:    3 turuncu + 2 sarı + gürültü + parlama → CLAHE testi

Çeldirici/negatif sahneler (F6.5 — dedektörün NE YAPMAMASI gerektiği):
    - scene_camera_beyaz_sosis:  parkur çevresi beyaz sosis şamandıra hattı
      (şartname md 5.5.2.1) + gerçek dubalar → sosisler ateşlememeli
    - scene_camera_turuncu_serit: dikdörtgen turuncu çeldirici (halat/parlama
      şeridi) + yuvarlak duba → F5.6 doluluk-skoru tersliğini belgeler
    - scene_camera_fov_kenari:   aynı boy duba frame ortasında + kenarda
      yarım — FOV kenarı kör noktasını belgeler
    - scene_camera_menzil_siniri: min_area_px eşiği iki yanında iki duba —
      F5.5 ≈15 m etkin menzil sınırını belgeler

Frame: 640×480 BGR uint8 (OAK-D Lite preview çözünürlüğü).
"""

from __future__ import annotations

import cv2
import numpy as np

FRAME_W, FRAME_H = 640, 480

#: BGR renkler — HSV karşılıkları camera_buoys varsayılan aralıklarının içinde.
WATER_BGR = (110, 70, 20)       # koyu deniz mavisi
ORANGE_BGR = (0, 140, 255)      # RAL 2003 yakını (OpenCV H≈16)
YELLOW_BGR = (0, 220, 255)      # RAL 1026 yakını (OpenCV H≈26)
WHITE_BGR = (245, 245, 245)     # sosis şamandıra — S≈0, HİÇBİR maskeye girmez
RED_BGR = (0, 0, 220)           # OpenCV H≈0  (kırmızı sarmalanmanın ALT ucu)
RED_MAGENTA_BGR = (30, 0, 200)  # OpenCV H≈176 (ÜST ucu — wraparound testi)
GREEN_BGR = (0, 150, 0)         # OpenCV H≈60
BROWN_BGR = (19, 49, 79)        # OpenCV H≈15 S≈194 V≈79 — turuncuyla aynı Hue,
                                 # belirgin düşük V (turuncu V-alt eşiği 120)


def _water_frame() -> np.ndarray:
    return np.full((FRAME_H, FRAME_W, 3), WATER_BGR, dtype=np.uint8)


def draw_buoy(
    frame: np.ndarray, x: int, y: int, radius: int, color_bgr: tuple
) -> None:
    """Dolu daire duba — kenar yumuşatmasız (maske testleri deterministik)."""
    cv2.circle(frame, (x, y), radius, color_bgr, thickness=-1)


def add_noise(
    frame: np.ndarray, rng: np.random.Generator, sigma: float = 8.0
) -> np.ndarray:
    """Gauss sensör gürültüsü — HSV aralıklarını bozmayacak ölçekte (σ=8)."""
    noisy = frame.astype(np.int16) + rng.normal(0.0, sigma, frame.shape)
    return np.clip(noisy, 0, 255).astype(np.uint8)


def desaturate(frame: np.ndarray, factor: float) -> np.ndarray:
    """Doygunluğu `factor` ile ölçekler (0=gri, 1=değişmez) — bulutlu/akşamüstü
    gerçek ışıkta ölçülen düşük doygunluğu (S≈29-83) taklit eder (2026-07-16
    gerçek donanım testi, sonkodv2_test1_log)."""
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV).astype(np.float32)
    hsv[:, :, 1] *= factor
    hsv = np.clip(hsv, 0, 255).astype(np.uint8)
    return cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)


def add_glare(frame: np.ndarray, strength: float = 0.35) -> np.ndarray:
    """Su parlaması taklidi: frame'i beyaza harmanla (CLAHE'nin dengeleyeceği
    düşük kontrast + yıkanmış renk)."""
    white = np.full_like(frame, 255)
    return cv2.addWeighted(frame, 1.0 - strength, white, strength, 0.0)


def scene_camera_minimum(rng: np.random.Generator) -> np.ndarray:
    """2 turuncu + 1 sarı duba, gürültü yok → 2× class 0 + 1× class 1."""
    frame = _water_frame()
    draw_buoy(frame, 160, 240, 30, ORANGE_BGR)
    draw_buoy(frame, 480, 240, 30, ORANGE_BGR)
    draw_buoy(frame, 320, 320, 25, YELLOW_BGR)
    return frame


def draw_sosis(
    frame: np.ndarray, x: int, y: int, half_w: int, half_h: int
) -> None:
    """Beyaz sosis şamandıra — yatay dolu elips (parkur çevre hattı)."""
    cv2.ellipse(
        frame, (x, y), (half_w, half_h), 0.0, 0.0, 360.0, WHITE_BGR, -1
    )


def scene_camera_beyaz_sosis(rng: np.random.Generator) -> np.ndarray:
    """Beyaz sosis hattı + 1 turuncu + 1 sarı duba (gürültü + parlama).

    Şartname md 5.5.2.1: parkur dışını BEYAZ sosis şamandıralar çevreler.
    Beklenen: yalnız 2 gerçek duba tespit edilir; beyaz (S≈0) hiçbir HSV
    maskesine girmez — parlama altında da (beyaz beyaza harmanlanır).
    """
    frame = _water_frame()
    for x in (60, 220, 380, 540):               # ufka yakın sosis hattı
        draw_sosis(frame, x, 150, 42, 12)
    draw_buoy(frame, 200, 300, 26, ORANGE_BGR)
    draw_buoy(frame, 440, 320, 24, YELLOW_BGR)
    frame = add_glare(frame)
    return add_noise(frame, rng)


def scene_camera_turuncu_serit(rng: np.random.Generator) -> np.ndarray:
    """Dikdörtgen turuncu çeldirici (halat/şerit) + yuvarlak turuncu duba.

    F5.6 belgeleme sahnesi: `score = alan/bbox_alanı` DOLULUK oranıdır —
    dikdörtgen çeldirici ≈1.0, yuvarlak gerçek duba ≈0.785 alır (TERS).
    Skor sözleşmesi düzeltilene kadar bu sahne tersliği testte görünür tutar.
    """
    frame = _water_frame()
    cv2.rectangle(frame, (80, 100), (240, 130), ORANGE_BGR, -1)   # şerit
    draw_buoy(frame, 440, 300, 26, ORANGE_BGR)
    return frame


def scene_camera_fov_kenari(rng: np.random.Generator) -> np.ndarray:
    """Aynı boy iki duba: biri frame ortasında, biri sol kenarda YARIM.

    FOV kenarı kör noktası: kenarda kırpılan dubanın görünen alanı
    min_area_px altına düşer → orta-frame'de görünen boy, kenarda görünmez.
    """
    frame = _water_frame()
    r = 9                                        # tam alan ≈254 px² > 150
    draw_buoy(frame, 320, 240, r, ORANGE_BGR)    # ortada → tespit
    draw_buoy(frame, 0, 240, r, ORANGE_BGR)      # kenarda yarım ≈127 px² → yok
    return frame


def scene_camera_menzil_siniri(rng: np.random.Generator) -> np.ndarray:
    """min_area_px=150 eşiğinin iki yanında iki turuncu duba (F5.5).

    30 cm duba ≈ 15 m'de ~150 px² (F5.5 hesabı: 533 px/rad HFOV) → r=8
    (~201 px², ~13-14 m) tespit edilir; r=6 (~113 px², ~17-18 m) edilmez.
    HSV etkin menzilinin LiDAR'dan (25 m) kısa olduğunu testte görünür tutar.
    """
    frame = _water_frame()
    draw_buoy(frame, 200, 200, 8, ORANGE_BGR)    # eşik üstü → tespit
    draw_buoy(frame, 440, 200, 6, ORANGE_BGR)    # eşik altı → görünmez
    return frame


def scene_camera_renkli(rng: np.random.Generator) -> np.ndarray:
    """1 kırmızı + 1 kırmızı(wraparound uç) + 1 yeşil + 1 kahverengi duba.

    Kırmızının OpenCV Hue'da İKİ ucu da (H≈0 ve H≈176) test edilir —
    red_mask()'ın sarmalanmayı doğru OR'ladığını doğrular.
    """
    frame = _water_frame()
    draw_buoy(frame, 120, 240, 28, RED_BGR)
    draw_buoy(frame, 280, 240, 28, RED_MAGENTA_BGR)
    draw_buoy(frame, 440, 240, 26, GREEN_BGR)
    draw_buoy(frame, 560, 340, 24, BROWN_BGR)
    return frame


def scene_camera_dusuk_isik(rng: np.random.Generator) -> np.ndarray:
    """Akşamüstü/bulutlu ışık taklidi: normal dubalar %30 doygunluğa düşürülür
    (2026-07-16 gerçek donanım testinde ölçülen S≈29-83'e denk gelir).

    Amaç: `equalize_saturation()` (F-P.21) OLMADAN bu dubalar sabit S≥120
    eşiğinin altında kalıp HİÇ tespit edilmemeli; equalize_saturation
    AÇIKKEN (varsayılan) tekrar tespit edilebilmeli.
    """
    frame = _water_frame()
    draw_buoy(frame, 160, 240, 30, ORANGE_BGR)
    draw_buoy(frame, 480, 240, 30, ORANGE_BGR)
    draw_buoy(frame, 320, 320, 25, YELLOW_BGR)
    return desaturate(frame, 0.3)


def scene_camera_orta(rng: np.random.Generator) -> np.ndarray:
    """3 turuncu + 2 sarı + gürültü + parlama → 3× class 0 + 2× class 1.

    Parlama + gürültü CLAHE ve morfoloji katmanlarını çalıştırır; dubalar
    yine tespit edilebilir kalır (renk aralıkları harmanlamaya dayanıklı).
    """
    frame = _water_frame()
    draw_buoy(frame, 100, 200, 28, ORANGE_BGR)
    draw_buoy(frame, 320, 180, 24, ORANGE_BGR)
    draw_buoy(frame, 540, 220, 30, ORANGE_BGR)
    draw_buoy(frame, 210, 340, 22, YELLOW_BGR)
    draw_buoy(frame, 430, 360, 26, YELLOW_BGR)
    frame = add_glare(frame)
    return add_noise(frame, rng)
