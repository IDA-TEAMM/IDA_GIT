"""
Girdap İDA — Kamera duba tespiti çekirdeği (Sprint 2, ROS-bağımsız).

Pipeline: CLAHE ön işleme (su parlaması dengelenir) → HSV renk segmentasyonu
(turuncu/sarı) → kontur → bbox. YOLO katmanı mock (gerçek .pt yok); gerçek
model gelince YALNIZ YoloInference._infer_real gövdesi değişir, arayüz sabit.

Sınıflar:
    0 = parkur_kenari (turuncu duba, RAL 2003 yakını)
    1 = engel         (sarı duba, RAL 1026 yakını)
    2 = hedef         (Parkur-3 kamikaze hedefi — YOLO katmanı)

Tasarım: turuncu/sarı için renk segmentasyonu yeterli (YOLO'ya gerek yok);
YOLO yalnız hedef sınıfı için — ultralytics lazy import, mock modda hiç
yüklenmez (Jetson'a gereksiz bağımlılık taşınmaz).
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
    clahe_clip_limit: float = 2.0       # kontrast sınırı (su parlaması dengesi)
    clahe_tile: int = 8                 # CLAHE karo ızgarası (8×8)
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
    orange_mask = cv2.inRange(
        hsv, np.array(cfg.hsv_orange_lo, np.uint8), np.array(cfg.hsv_orange_hi, np.uint8)
    )
    yellow_mask = cv2.inRange(
        hsv, np.array(cfg.hsv_yellow_lo, np.uint8), np.array(cfg.hsv_yellow_hi, np.uint8)
    )
    orange_cov = float(np.count_nonzero(orange_mask)) / total
    yellow_cov = float(np.count_nonzero(yellow_mask)) / total
    min_cov = cfg.yolo_localizer_min_coverage
    if orange_cov < min_cov and yellow_cov < min_cov:
        return None
    return CLASS_PARKUR_KENARI if orange_cov >= yellow_cov else CLASS_ENGEL


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
        orange = color_mask(
            hsv, cfg.hsv_orange_lo, cfg.hsv_orange_hi, cfg.morph_kernel_px
        )
        yellow = color_mask(
            hsv, cfg.hsv_yellow_lo, cfg.hsv_yellow_hi, cfg.morph_kernel_px
        )
        detections = mask_to_detections(orange, CLASS_PARKUR_KENARI, cfg)
        detections += mask_to_detections(yellow, CLASS_ENGEL, cfg)

    if cfg.use_yolo and yolo is not None:
        detections += yolo.infer(frame_bgr)       # hedef sınıfı (Parkur-3)
    return detections
