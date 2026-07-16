"""
Girdap İDA — Kamera duba tespiti çekirdeği (Sprint 2, ROS-bağımsız).

Pipeline: CLAHE ön işleme (su parlaması dengelenir) → HSV renk segmentasyonu
(turuncu/sarı) → kontur → bbox. YOLO katmanı mock (gerçek .pt yok); gerçek
model gelince YALNIZ YoloInference._infer_real gövdesi değişir, arayüz sabit.

Sınıflar:
    0 = parkur_kenari (turuncu duba, RAL 2003 yakını)
    1 = engel         (sarı duba, RAL 1026 yakını)
    2 = hedef         (Parkur-3 kamikaze hedefi — YOLO katmanı)
    3 = kirmizi       (kırmızı duba/işaret)
    4 = yesil         (yeşil duba/işaret)
    5 = kahverengi    (kahverengi duba/işaret — düşük-V turuncuya yakın)

Tasarım: turuncu/sarı/kırmızı/yeşil/kahverengi için renk segmentasyonu
yeterli (YOLO'ya gerek yok); YOLO yalnız hedef sınıfı için — ultralytics
lazy import, mock modda hiç yüklenmez (Jetson'a gereksiz bağımlılık
taşınmaz).

⚠ Işık koşulu notu (2026-07-16 gerçek donanım testi, sonkodv2_test1_log):
akşamüstü/bulutlu ışıkta gerçek bir turuncu/sarı dubanın ölçülen doygunluğu
(S≈29-83) sabit eşiklerin (S≥120) çok altında kaldı, hiç tespit edilmedi.
`equalize_saturation()` bunu düzeltmek için eklendi — doygunluk kanalını
yüzdelik-dilim germesiyle yeniden ölçekler, sabit eşikler farklı ışık
koşullarında da işe yarar hale gelir. Kırmızı/yeşil/kahverengi eşikleri de turuncu/sarı gibi SAHADA
gerçek nesnelerle doğrulanmalı — burada verilen varsayılanlar ilk tahmin,
kör güven duyulmamalı.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Optional, Sequence

import cv2
import numpy as np

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


@dataclass
class CameraBuoyConfig:
    """Kamera duba tespiti parametreleri (config perception.camera bloğu).

    HSV aralıkları OpenCV ölçeği: H ∈ [0,179], S/V ∈ [0,255].
    """

    hsv_orange_lo: tuple[int, int, int] = (5, 120, 120)     # RAL 2003 yakını
    hsv_orange_hi: tuple[int, int, int] = (20, 255, 255)
    hsv_yellow_lo: tuple[int, int, int] = (21, 120, 120)    # RAL 1026 yakını
    hsv_yellow_hi: tuple[int, int, int] = (35, 255, 255)
    # Kırmızı, OpenCV'nin H=0/179 sarmalanmasının İKİ ucuna da düşer — tek
    # aralık yeterli değil, iki aralık OR'lanır (bkz. red_mask()).
    hsv_red_lo1: tuple[int, int, int] = (0, 120, 70)
    hsv_red_hi1: tuple[int, int, int] = (8, 255, 255)
    hsv_red_lo2: tuple[int, int, int] = (170, 120, 70)
    hsv_red_hi2: tuple[int, int, int] = (179, 255, 255)
    hsv_green_lo: tuple[int, int, int] = (40, 80, 60)
    hsv_green_hi: tuple[int, int, int] = (85, 255, 255)
    # Kahverengi ≈ düşük-V turuncu (aynı Hue bandı, çok daha karanlık) —
    # V üst sınırı turuncunun V alt sınırının (120) altında tutulur ki iki
    # sınıf çakışmasın (ince bir orta bant hâlâ belirsiz kalabilir, bu
    # kahverengi/turuncu ayrımının doğası gereği kaçınılmaz bir sınırlama).
    hsv_brown_lo: tuple[int, int, int] = (5, 60, 40)
    hsv_brown_hi: tuple[int, int, int] = (20, 255, 115)
    clahe_clip_limit: float = 2.0       # kontrast sınırı (su parlaması dengesi)
    clahe_tile: int = 8                 # CLAHE karo ızgarası (8×8)
    # F-P.21 (2026-07-16, gerçek donanım testi): düşük ışıkta (akşamüstü/
    # bulutlu) doygunluk sabit eşiklerin altında kalıp tespiti hiç
    # tetiklememesini önlemek için doygunluk kanalı yüzdelik-dilim germesiyle
    # (percentile stretch) yeniden ölçeklenir. NOT: önce CLAHE denendi ama
    # sahne baştan sona düzgün soluksa (tüm kareye yayılmış düşük doygunluk —
    # tam olarak akşamüstü/bulutlu ışığın verdiği durum) CLAHE'nin yerel
    # kontrast artışı yetersiz kaldı (canlı ölçümde S=75→84, gereken 120+
    # değil) — global germe bunu S=75→255'e çıkardı, gerçek düzeltme bu.
    saturation_clahe: bool = True
    saturation_stretch_lo_pct: float = 1.0
    saturation_stretch_hi_pct: float = 99.0
    # Sahnede zaten en az bir doygun renk varsa (ör. parlak kırmızı/yeşil
    # duba, S≈255) germe UYGULANMAZ — aksi halde farklı renklerin DOĞAL
    # doygunluk farkı (ör. kahverengi turuncudan doğal olarak daha az
    # doygundur) germe tarafından bozulup yanlışlıkla sıfıra çökebilir.
    # Germe yalnız sahne GENELİNDE düşük doygunluksa (üst yüzdelik dilim de
    # düşükse — akşamüstü/bulutlu ışığın verdiği durum) devreye girer.
    saturation_stretch_skip_above: float = 150.0
    min_area_px: int = 150              # bu altı kontur noise
    morph_kernel_px: int = 5            # açma/kapama gürültü temizliği
    use_yolo: bool = False              # hedef sınıfı YOLO katmanı
    yolo_model_path: str = ""           # gerçek .pt yolu (boş = mock)
    # F-S.9: turuncu/sarı (class 0/1) tespiti için ALTERNATİF yol — eğitilmiş
    # genel duba lokalizatörü (ör. ida_topics/best.pt) + BU config'in AYARLANMIŞ
    # HSV eşikleriyle renk sınıflandırması. Varsayılan False → mevcut saf-HSV
    # segmentasyonu (detect_buoys) hiç değişmez, geriye dönük tam uyumlu.
    use_yolo_localizer: bool = False
    yolo_localizer_model_path: str = ""     # boş = mock (kutu üretmez)
    yolo_localizer_min_coverage: float = 0.15  # ROI'de sınıf HSV kaplama eşiği


def apply_clahe(frame_bgr: np.ndarray, cfg: CameraBuoyConfig) -> np.ndarray:
    """LAB uzayında L kanalına CLAHE — renk tonunu bozmadan kontrast dengeler."""
    lab = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2LAB)
    l_ch, a_ch, b_ch = cv2.split(lab)
    clahe = cv2.createCLAHE(
        clipLimit=cfg.clahe_clip_limit,
        tileGridSize=(cfg.clahe_tile, cfg.clahe_tile),
    )
    merged = cv2.merge((clahe.apply(l_ch), a_ch, b_ch))
    return cv2.cvtColor(merged, cv2.COLOR_LAB2BGR)


def equalize_saturation(hsv: np.ndarray, cfg: CameraBuoyConfig) -> np.ndarray:
    """Doygunluk kanalını yüzdelik-dilim germesiyle (percentile stretch)
    yeniden ölçekler (F-P.21).

    2026-07-16 gerçek donanım testinde bulundu: akşamüstü/bulutlu ışıkta
    gerçek turuncu/sarı bir dubanın ölçülen doygunluğu (S≈29-83) sabit
    `hsv_*_lo` eşiklerinin (S≥120) çok altında kaldı, hiç tespit edilmedi.
    Bu, ışığın dağınık (diffuse) olduğu koşullarda rengin kameraya daha
    "soluk" yansımasının doğrudan sonucu. Karedeki 1.-99. yüzdelik dilim
    aralığı [0,255]'e yeniden ölçeklenir — sahne baştan sona düzgün soluksa
    bile (tam olarak bu senaryo) o an mevcut en yüksek doygunluk 255'e
    yakın bir değere çekilir, sabit eşikler hem parlak güneşte hem
    bulutlu/akşamüstünde işe yarar hale gelir. `cfg.saturation_clahe=False`
    ile eski (ham) davranışa dönülebilir.
    """
    if not cfg.saturation_clahe:
        return hsv
    h_ch, s_ch, v_ch = cv2.split(hsv)
    hi = np.percentile(s_ch, cfg.saturation_stretch_hi_pct)
    if hi >= cfg.saturation_stretch_skip_above:
        # Sahnede zaten yeterince doygun bir şey var — germe uygulanmaz,
        # farklı renklerin doğal doygunluk farkı korunur.
        return hsv
    lo = np.percentile(s_ch, cfg.saturation_stretch_lo_pct)
    if hi - lo < 1.0:
        return hsv          # düz/ayırt edilemez sahne (tüm kare tek renk) — dokunma
    s_stretched = np.clip(
        (s_ch.astype(np.float32) - lo) * (255.0 / (hi - lo)), 0, 255
    ).astype(np.uint8)
    return cv2.merge((h_ch, s_stretched, v_ch))


def color_mask(
    hsv: np.ndarray,
    lo: Sequence[int],
    hi: Sequence[int],
    kernel_px: int = 5,
) -> np.ndarray:
    """HSV aralık maskesi + morfolojik açma/kapama (tuz-biber temizliği)."""
    mask = cv2.inRange(hsv, np.array(lo, np.uint8), np.array(hi, np.uint8))
    k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (kernel_px, kernel_px))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, k)
    return cv2.morphologyEx(mask, cv2.MORPH_CLOSE, k)


def red_mask(hsv: np.ndarray, cfg: CameraBuoyConfig, kernel_px: int = 5) -> np.ndarray:
    """Kırmızı — OpenCV Hue'nun 0/179 sarmalanmasının İKİ ucunu da OR'lar.

    Kırmızı hem H≈0 hem H≈179 civarında bulunur (döngüsel Hue ekseninin iki
    ucu da aynı renk) — tek bir `cv2.inRange` aralığı bunu YAKALAYAMAZ, iki
    aralık ayrı ayrı maskelenip birleştirilmeli.
    """
    m1 = cv2.inRange(
        hsv, np.array(cfg.hsv_red_lo1, np.uint8), np.array(cfg.hsv_red_hi1, np.uint8)
    )
    m2 = cv2.inRange(
        hsv, np.array(cfg.hsv_red_lo2, np.uint8), np.array(cfg.hsv_red_hi2, np.uint8)
    )
    mask = cv2.bitwise_or(m1, m2)
    k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (kernel_px, kernel_px))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, k)
    return cv2.morphologyEx(mask, cv2.MORPH_CLOSE, k)


def mask_to_detections(
    mask: np.ndarray, class_id: int, cfg: CameraBuoyConfig
) -> list[Detection]:
    """Kontur → bbox; alan filtresi + doluluk oranı skoru (daire ~0.79)."""
    contours, _ = cv2.findContours(
        mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
    )
    detections: list[Detection] = []
    for contour in contours:
        area = cv2.contourArea(contour)
        if area < cfg.min_area_px:
            continue
        x, y, w, h = cv2.boundingRect(contour)
        detections.append(
            Detection(
                center_x=x + w / 2.0,
                center_y=y + h / 2.0,
                width=float(w),
                height=float(h),
                class_id=class_id,
                score=min(1.0, float(area) / float(w * h)),
            )
        )
    return detections


class YoloInference:
    """YOLOv11n sarmalayıcı — mock ↔ gerçek model tek arayüz.

    Mock mod: sabit test bbox'ları döner (gerçek .pt yokken pipeline canlı).
    Gerçek mod: ultralytics LAZY import — .pt gelince yalnız _infer_real
    gövdesi doğrulanır/değişir, çağıran kod (node, detect_buoys) dokunulmaz.
    """

    #: Mock modun döndürdüğü sabit tespit (frame merkezli hedef dubası).
    _MOCK_BBOX_SIZE = 48.0
    _MOCK_SCORE = 0.9

    def __init__(
        self,
        model_path: str = "",
        mock: bool = True,
        mock_detections: Optional[list[Detection]] = None,
    ) -> None:
        self._model_path = model_path
        self._mock = mock or not model_path       # .pt yoksa mock'a düş
        self._mock_detections = mock_detections
        self._model: Any = None                   # lazy — mock'ta hiç yüklenmez

    @property
    def is_mock(self) -> bool:
        return self._mock

    def infer(self, frame_bgr: np.ndarray) -> list[Detection]:
        if self._mock:
            return self._infer_mock(frame_bgr)
        return self._infer_real(frame_bgr)

    def _infer_mock(self, frame_bgr: np.ndarray) -> list[Detection]:
        """Sabit test bbox'ları — frame merkezinde tek hedef dubası."""
        if self._mock_detections is not None:
            return list(self._mock_detections)
        h, w = frame_bgr.shape[:2]
        return [
            Detection(
                center_x=w / 2.0,
                center_y=h / 2.0,
                width=self._MOCK_BBOX_SIZE,
                height=self._MOCK_BBOX_SIZE,
                class_id=CLASS_HEDEF,
                score=self._MOCK_SCORE,
            )
        ]

    def _infer_real(self, frame_bgr: np.ndarray) -> list[Detection]:
        """Gerçek YOLOv11n — .pt gelince BU gövde saha verisiyle doğrulanır."""
        if self._model is None:
            from ultralytics import YOLO          # lazy — mock modda import yok
            self._model = YOLO(self._model_path)
        detections: list[Detection] = []
        for result in self._model(frame_bgr, verbose=False):
            for box in result.boxes:
                cx, cy, w, h = box.xywh[0].tolist()
                detections.append(
                    Detection(cx, cy, w, h, int(box.cls), float(box.conf))
                )
        return detections


# --------------------------------------------------------------------------- #
# F-S.9: YOLO-lokalizasyon + girdap HSV-sınıflandırma hibrit yolu.
#
# ida_topics/perception_node.py'nin kanıtlanmış yaklaşımı (gerçek eğitilmiş
# model kutuyu bulur, HSV rengi/sınıfı belirler) buraya taşındı — ama renk
# eşikleri bu modülün AYARLANMIŞ CameraBuoyConfig değerleridir (iki tarafın
# güçlü yanı birleşiyor: modelin gerçek-veri lokalizasyonu + tune edilmiş HSV).
# Varsayılan (use_yolo_localizer=False) mevcut saf-HSV yolunu hiç etkilemez.
# --------------------------------------------------------------------------- #


@dataclass
class RawBox:
    """Lokalizatörün ürettiği ham kutu — HENÜZ sınıfsız (yalnız konum+skor)."""

    center_x: float
    center_y: float
    width: float
    height: float
    score: float


class BuoyLocalizer:
    """Eğitilmiş genel duba lokalizatörü — SINIF ÜRETMEZ, yalnız kutu+skor.

    `YoloInference`'dan (hedef sınıfı için, class_id döner) kasıtlı FARKLI:
    ida_topics/best.pt modeli gibi "burada bir duba var" der, rengi/sınıfı
    `classify_roi_color` ayrıca HSV ile belirler. Mock mod (model_path boş)
    hiç kutu üretmez — testler `mock_boxes` ile enjekte eder.
    """

    def __init__(
        self,
        model_path: str = "",
        mock: bool = True,
        mock_boxes: Optional[list[RawBox]] = None,
    ) -> None:
        self._model_path = model_path
        self._mock = mock or not model_path
        self._mock_boxes = mock_boxes
        self._model: Any = None

    @property
    def is_mock(self) -> bool:
        return self._mock

    def locate(self, frame_bgr: np.ndarray) -> list[RawBox]:
        if self._mock:
            return list(self._mock_boxes) if self._mock_boxes else []
        return self._locate_real(frame_bgr)

    def _locate_real(self, frame_bgr: np.ndarray) -> list[RawBox]:
        if self._model is None:
            from ultralytics import YOLO          # lazy — mock modda import yok
            self._model = YOLO(self._model_path)
        boxes: list[RawBox] = []
        for result in self._model(frame_bgr, conf=0.15, verbose=False):
            for box in result.boxes:
                cx, cy, w, h = box.xywh[0].tolist()
                boxes.append(RawBox(cx, cy, w, h, float(box.conf)))
        return boxes


def classify_roi_color(roi_bgr: np.ndarray, cfg: CameraBuoyConfig) -> Optional[int]:
    """Bir lokalizasyon kutusunun ROI'sini girdap'ın HSV eşikleriyle sınıflar.

    inRange KAPLAMA ORANI kullanılır (ida_topics'in ortalama-H yaklaşımından
    daha sağlam — kısmi gölge/parlama tek bir pikselin ortalamayı bozmasını
    engeller). Hiçbir sınıf `yolo_localizer_min_coverage` eşiğini aşmıyorsa
    (ör. "hedef" nesnesi ya da arka plan) None — bu kutu ATLANIR.
    """
    if roi_bgr.size == 0:
        return None
    hsv = cv2.cvtColor(roi_bgr, cv2.COLOR_BGR2HSV)
    total = hsv.shape[0] * hsv.shape[1]
    if total == 0:
        return None
    hsv = equalize_saturation(hsv, cfg)
    orange_mask = cv2.inRange(
        hsv, np.array(cfg.hsv_orange_lo, np.uint8), np.array(cfg.hsv_orange_hi, np.uint8)
    )
    yellow_mask = cv2.inRange(
        hsv, np.array(cfg.hsv_yellow_lo, np.uint8), np.array(cfg.hsv_yellow_hi, np.uint8)
    )
    red_m = red_mask(hsv, cfg, kernel_px=1)
    green_mask = cv2.inRange(
        hsv, np.array(cfg.hsv_green_lo, np.uint8), np.array(cfg.hsv_green_hi, np.uint8)
    )
    brown_mask = cv2.inRange(
        hsv, np.array(cfg.hsv_brown_lo, np.uint8), np.array(cfg.hsv_brown_hi, np.uint8)
    )
    coverage = {
        CLASS_PARKUR_KENARI: float(np.count_nonzero(orange_mask)) / total,
        CLASS_ENGEL: float(np.count_nonzero(yellow_mask)) / total,
        CLASS_KIRMIZI: float(np.count_nonzero(red_m)) / total,
        CLASS_YESIL: float(np.count_nonzero(green_mask)) / total,
        CLASS_KAHVERENGI: float(np.count_nonzero(brown_mask)) / total,
    }
    best_class = max(coverage, key=coverage.get)
    if coverage[best_class] < cfg.yolo_localizer_min_coverage:
        return None
    return best_class


def _detect_buoys_yolo_localized(
    frame_bgr: np.ndarray, cfg: CameraBuoyConfig, localizer: BuoyLocalizer
) -> list[Detection]:
    """F-S.9: lokalizatör kutuyu bulur, girdap HSV eşikleri rengi/sınıfı verir."""
    balanced = apply_clahe(frame_bgr, cfg)
    h_frame, w_frame = balanced.shape[:2]
    detections: list[Detection] = []
    for box in localizer.locate(balanced):
        x1 = max(0, int(box.center_x - box.width / 2))
        y1 = max(0, int(box.center_y - box.height / 2))
        x2 = min(w_frame, int(box.center_x + box.width / 2))
        y2 = min(h_frame, int(box.center_y + box.height / 2))
        roi = balanced[y1:y2, x1:x2]
        class_id = classify_roi_color(roi, cfg)
        if class_id is None:
            continue                              # ne turuncu ne sarı — atla
        detections.append(
            Detection(box.center_x, box.center_y, box.width, box.height,
                      class_id, box.score)
        )
    return detections


def detect_buoys(
    frame_bgr: np.ndarray,
    cfg: CameraBuoyConfig,
    yolo: Optional[YoloInference] = None,
    localizer: Optional[BuoyLocalizer] = None,
) -> list[Detection]:
    """Tam pipeline: CLAHE → turuncu/sarı bbox (+ opsiyonel YOLO hedef).

    turuncu/sarı için iki yol vardır (cfg.use_yolo_localizer ile seçilir):
        False (varsayılan): saf HSV segmentasyonu (mevcut davranış, DEĞİŞMEDİ).
        True: F-S.9 hibrit — eğitilmiş lokalizatör + girdap HSV sınıflandırma.
    Boş/geçersiz girişte boş liste (defensive).
    """
    if frame_bgr is None or frame_bgr.size == 0:
        return []

    if cfg.use_yolo_localizer and localizer is not None:
        detections = _detect_buoys_yolo_localized(frame_bgr, cfg, localizer)
    else:
        balanced = apply_clahe(frame_bgr, cfg)
        hsv = cv2.cvtColor(balanced, cv2.COLOR_BGR2HSV)
        hsv = equalize_saturation(hsv, cfg)
        orange = color_mask(
            hsv, cfg.hsv_orange_lo, cfg.hsv_orange_hi, cfg.morph_kernel_px
        )
        yellow = color_mask(
            hsv, cfg.hsv_yellow_lo, cfg.hsv_yellow_hi, cfg.morph_kernel_px
        )
        red = red_mask(hsv, cfg, cfg.morph_kernel_px)
        green = color_mask(
            hsv, cfg.hsv_green_lo, cfg.hsv_green_hi, cfg.morph_kernel_px
        )
        brown = color_mask(
            hsv, cfg.hsv_brown_lo, cfg.hsv_brown_hi, cfg.morph_kernel_px
        )
        detections = mask_to_detections(orange, CLASS_PARKUR_KENARI, cfg)
        detections += mask_to_detections(yellow, CLASS_ENGEL, cfg)
        detections += mask_to_detections(red, CLASS_KIRMIZI, cfg)
        detections += mask_to_detections(green, CLASS_YESIL, cfg)
        detections += mask_to_detections(brown, CLASS_KAHVERENGI, cfg)

    if cfg.use_yolo and yolo is not None:
        detections += yolo.infer(frame_bgr)       # hedef sınıfı (Parkur-3)
    return detections
