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
                # F5.6 SÖZLEŞME: HSV yolunda score = DOLULUK ORANI (şekil
                # metriği), güven skoru DEĞİL — daire ≈0.785, dikdörtgen
                # şerit 1.0 (yani "daha duba" ≠ daha yüksek skor). YOLO
                # katmanı ise ağ güveni yazar. Tüketici bu alanı güven eşiği
                # olarak KULLANMAMALI (CLAUDE.md Kamera Pipeline sözleşmesi).
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


def detect_buoys(
    frame_bgr: np.ndarray,
    cfg: CameraBuoyConfig,
    yolo: Optional[YoloInference] = None,
) -> list[Detection]:
    """Tam pipeline: CLAHE → HSV → turuncu/sarı bbox (+ opsiyonel YOLO hedef).

    Boş/geçersiz girişte boş liste (defensive).
    """
    if frame_bgr is None or frame_bgr.size == 0:
        return []
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
