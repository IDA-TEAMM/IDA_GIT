"""
Girdap İDA — Kamera duba tespiti çekirdeği testleri (Sprint 2).

CLAHE → HSV segmentasyon → kontur/bbox pipeline'ı + mock YOLO sarmalayıcısı;
sentetik sahneler (scene_camera_minimum, scene_camera_orta) fixture.

Çalıştır: pytest prototype/tests/test_camera_buoys.py -v
"""

from __future__ import annotations

import sys

import cv2
import numpy as np
import pytest

from prototype.perception.camera_buoys import (
    CLASS_ENGEL,
    CLASS_HEDEF,
    CLASS_PARKUR_KENARI,
    CameraBuoyConfig,
    Detection,
    YoloInference,
    apply_clahe,
    color_mask,
    detect_buoys,
    mask_to_detections,
)
from prototype.perception.synthetic_camera import (
    FRAME_H,
    FRAME_W,
    ORANGE_BGR,
    draw_buoy,
    scene_camera_beyaz_sosis,
    scene_camera_fov_kenari,
    scene_camera_menzil_siniri,
    scene_camera_minimum,
    scene_camera_orta,
    scene_camera_turuncu_serit,
)


@pytest.fixture
def cfg() -> CameraBuoyConfig:
    return CameraBuoyConfig()


@pytest.fixture
def rng() -> np.random.Generator:
    return np.random.default_rng(42)


# ---------------------------------------------------------------- CLAHE

def test_clahe_preserves_shape_and_dtype(
    cfg: CameraBuoyConfig, rng: np.random.Generator
) -> None:
    frame = scene_camera_minimum(rng)
    out = apply_clahe(frame, cfg)
    assert out.shape == frame.shape
    assert out.dtype == np.uint8


def test_clahe_boosts_low_contrast(
    cfg: CameraBuoyConfig, rng: np.random.Generator
) -> None:
    # Parlama + gürültülü (gerçekçi) frame'de CLAHE kontrastı artırmalı.
    # NOT: tamamen gürültüsüz düz arka planda CLAHE global std'yi ARTIRMAZ
    # (düz karo düz kalır) — bu yüzden sentetik gürültülü sahne kullanılır.
    washed = scene_camera_orta(rng)               # parlama + gürültü içerir
    gray_before = cv2.cvtColor(washed, cv2.COLOR_BGR2GRAY)
    gray_after = cv2.cvtColor(apply_clahe(washed, cfg), cv2.COLOR_BGR2GRAY)
    assert gray_after.std() > gray_before.std()


# ---------------------------------------------------------------- HSV maske

def test_color_mask_orange_hits_orange_buoy(cfg: CameraBuoyConfig) -> None:
    frame = np.zeros((100, 100, 3), dtype=np.uint8)
    draw_buoy(frame, 50, 50, 20, ORANGE_BGR)
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask = color_mask(hsv, cfg.hsv_orange_lo, cfg.hsv_orange_hi)
    assert mask[50, 50] == 255                  # duba merkezi maskede
    assert mask[5, 5] == 0                      # arka plan maskede değil


def test_color_mask_yellow_ignores_orange(cfg: CameraBuoyConfig) -> None:
    frame = np.zeros((100, 100, 3), dtype=np.uint8)
    draw_buoy(frame, 50, 50, 20, ORANGE_BGR)
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask = color_mask(hsv, cfg.hsv_yellow_lo, cfg.hsv_yellow_hi)
    assert mask.sum() == 0                      # sarı maskesi turuncuyu görmez


# ---------------------------------------------------------------- kontur/bbox

def test_mask_to_detections_min_area_filter(cfg: CameraBuoyConfig) -> None:
    mask = np.zeros((100, 100), dtype=np.uint8)
    cv2.circle(mask, (30, 30), 20, 255, -1)     # alan ~1257 px² > 150
    cv2.circle(mask, (80, 80), 5, 255, -1)      # alan ~79 px² < 150 → elenir
    dets = mask_to_detections(mask, CLASS_PARKUR_KENARI, cfg)
    assert len(dets) == 1
    assert dets[0].center_x == pytest.approx(30.0, abs=1.0)
    assert dets[0].center_y == pytest.approx(30.0, abs=1.0)
    assert 0.0 < dets[0].score <= 1.0


# ---------------------------------------------------------------- mock YOLO

def test_mock_yolo_returns_fixed_target_bbox(
    rng: np.random.Generator,
) -> None:
    yolo = YoloInference()                      # .pt yok → mock
    assert yolo.is_mock
    frame = scene_camera_minimum(rng)
    dets = yolo.infer(frame)
    assert len(dets) == 1
    assert dets[0].class_id == CLASS_HEDEF
    assert dets[0].center_x == pytest.approx(320.0)   # frame merkezi (640/2)
    assert dets[0].center_y == pytest.approx(240.0)


def test_yolo_mock_never_imports_ultralytics(
    rng: np.random.Generator,
) -> None:
    # Lazy import garantisi: mock inference ultralytics'i YÜKLEMEMELİ
    YoloInference().infer(scene_camera_minimum(rng))
    assert "ultralytics" not in sys.modules


# ---------------------------------------------------------------- pipeline

def test_detect_buoys_scene_minimum(
    cfg: CameraBuoyConfig, rng: np.random.Generator
) -> None:
    dets = detect_buoys(scene_camera_minimum(rng), cfg)
    by_class: dict[int, list[Detection]] = {}
    for d in dets:
        by_class.setdefault(d.class_id, []).append(d)
    assert len(by_class.get(CLASS_PARKUR_KENARI, [])) == 2   # 2 turuncu
    assert len(by_class.get(CLASS_ENGEL, [])) == 1           # 1 sarı
    yellow = by_class[CLASS_ENGEL][0]
    assert yellow.center_x == pytest.approx(320.0, abs=3.0)
    assert yellow.center_y == pytest.approx(320.0, abs=3.0)


def test_detect_buoys_scene_orta(
    cfg: CameraBuoyConfig, rng: np.random.Generator
) -> None:
    # Gürültü + parlama altında: 3 turuncu + 2 sarı, fazladan tespit yok
    dets = detect_buoys(scene_camera_orta(rng), cfg)
    orange = [d for d in dets if d.class_id == CLASS_PARKUR_KENARI]
    yellow = [d for d in dets if d.class_id == CLASS_ENGEL]
    assert len(orange) == 3
    assert len(yellow) == 2
    assert all(0.0 < d.score <= 1.0 for d in dets)


def test_detect_buoys_empty_input(cfg: CameraBuoyConfig) -> None:
    assert detect_buoys(np.empty((0, 0, 3), dtype=np.uint8), cfg) == []


# ------------------------------------------- F6.5: çeldirici/negatif sahneler


def test_beyaz_sosis_hatti_ateslemez(
    cfg: CameraBuoyConfig, rng: np.random.Generator
) -> None:
    """Parkur çevresi BEYAZ sosis şamandıralar (md 5.5.2.1) tespit ÜRETMEMELİ.

    Beyaz S≈0 → hiçbir HSV maskesine girmez; parlama + gürültü altında da.
    Yalnız gerçek dubalar (1 turuncu + 1 sarı) kalmalı."""
    detections = detect_buoys(scene_camera_beyaz_sosis(rng), cfg)
    assert len(detections) == 2
    classes = sorted(d.class_id for d in detections)
    assert classes == [CLASS_PARKUR_KENARI, CLASS_ENGEL]
    for d in detections:
        assert d.center_y > 250          # sosis hattı (y=150) civarında tespit yok


def test_turuncu_serit_skor_tersligi_f56(
    cfg: CameraBuoyConfig, rng: np.random.Generator
) -> None:
    """F5.6 BELGELEME (açık bulgu): score doluluk oranı — dikdörtgen çeldirici
    yuvarlak GERÇEK dubadan YÜKSEK skor alır (ters). Skor sözleşmesi
    düzeltildiğinde bu test doğru davranışa GÜNCELLENMELİ (dondurmak için
    değil, tersliği görünür tutmak için var)."""
    detections = detect_buoys(scene_camera_turuncu_serit(rng), cfg)
    assert len(detections) == 2
    serit = max(detections, key=lambda d: d.width / d.height)   # 161×31 şerit
    duba = min(detections, key=lambda d: d.width / d.height)    # ~51×53 daire
    assert serit.class_id == CLASS_PARKUR_KENARI    # yanlış-pozitif: şerit "kenar dubası"
    assert serit.score > 0.9                        # dikdörtgen doluluk ≈ 1.0
    assert duba.score < 0.85                        # daire doluluk ≈ 0.785
    assert serit.score > duba.score                 # TERSLİK: çeldirici > gerçek


def test_fov_kenari_kirpilan_duba_gorunmez(
    cfg: CameraBuoyConfig, rng: np.random.Generator
) -> None:
    """FOV kenarı kör noktası: AYNI boy duba ortada tespit edilirken kenarda
    yarım kalınca görünen alanı min_area_px altına düşer → görünmez.
    Bilinen sınırlama — kenar dubası ancak yaklaşınca/FOV'a girince görünür."""
    detections = detect_buoys(scene_camera_fov_kenari(rng), cfg)
    assert len(detections) == 1
    assert abs(detections[0].center_x - 320.0) < 3   # yalnız ortadaki
    # bbox frame içinde kalmalı (sözleşme: piksel uzayı 640×480)
    d = detections[0]
    assert 0.0 <= d.center_x - d.width / 2 and d.center_x + d.width / 2 <= FRAME_W
    assert 0.0 <= d.center_y - d.height / 2 and d.center_y + d.height / 2 <= FRAME_H


def test_menzil_siniri_esik_iki_yani_f55(
    cfg: CameraBuoyConfig, rng: np.random.Generator
) -> None:
    """F5.5 BELGELEME: min_area_px=150 ⇒ HSV etkin menzili ≈15 m (LiDAR 25 m).

    Eşik üstü duba (r=8 ≈ 13-14 m) tespit edilir, eşik altı (r=6 ≈ 17-18 m)
    edilmez. 15 m ötesi renk/sınıf bilgisi YOK — geçit çifti ancak yaklaşınca
    sınıflanır; füzyonda eşleşmeyen LiDAR engeli CLASS_UNKNOWN=99 kalır."""
    detections = detect_buoys(scene_camera_menzil_siniri(rng), cfg)
    assert len(detections) == 1
    assert abs(detections[0].center_x - 200.0) < 3   # yalnız eşik üstü (r=8)
