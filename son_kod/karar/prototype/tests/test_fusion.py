"""
Girdap İDA — Kamera-LiDAR bearing füzyonu çekirdeği testleri (Sprint 3).

Bearing hesaplamaları + greedy eşleştirme (matched/unknown/double-match/
camera-only-drop) + sentetik senaryo uçtan uca doğrulaması.

Çalıştır: pytest prototype/tests/test_fusion.py -v
"""

from __future__ import annotations

import math

import numpy as np
import pytest

from prototype.perception.camera_buoys import CLASS_ENGEL, CLASS_HEDEF, CLASS_PARKUR_KENARI
from prototype.perception.fusion import (
    CLASS_UNKNOWN,
    CameraDetection,
    FusedObstacle,
    FusionConfig,
    LidarDetection,
    associate,
    bearing_from_camera,
    bearing_from_lidar,
)
from prototype.perception.synthetic_fusion import (
    scene_fusion_camera_only,
    scene_fusion_matched,
)


@pytest.fixture
def cfg() -> FusionConfig:
    return FusionConfig()


@pytest.fixture
def rng() -> np.random.Generator:
    return np.random.default_rng(42)


# ---------------------------------------------------------------- bearing

def test_bearing_from_lidar_dead_ahead_is_zero() -> None:
    assert bearing_from_lidar(LidarDetection(5.0, 0.0, 0.15)) == pytest.approx(0.0)


def test_bearing_from_lidar_matches_atan2() -> None:
    det = LidarDetection(3.0, 4.0, 0.15)
    assert bearing_from_lidar(det) == pytest.approx(math.atan2(4.0, 3.0))


def test_bearing_from_camera_center_is_zero(cfg: FusionConfig) -> None:
    det = CameraDetection(bbox_cx=0.5, bbox_cy=0.5, class_id=0, score=0.9)
    assert bearing_from_camera(det, cfg.camera_hfov_rad) == pytest.approx(0.0)


def test_bearing_from_camera_edges_are_half_hfov(cfg: FusionConfig) -> None:
    """İşaret kuralı = LiDAR ile AYNI: sol pozitif (atan2 konvansiyonu).

    Görüntünün SOL kenarındaki (bbox_cx=0) nesne fiziksel olarak SOLDA →
    bearing +hfov/2 (F6.1: eski test −hfov/2 bekleyerek ters kuralı
    donduruyordu; sentetik üreteçteki ters fonksiyon hatayı maskeliyordu).
    """
    left = CameraDetection(bbox_cx=0.0, bbox_cy=0.5, class_id=0, score=0.9)
    right = CameraDetection(bbox_cx=1.0, bbox_cy=0.5, class_id=0, score=0.9)
    assert bearing_from_camera(left, cfg.camera_hfov_rad) == pytest.approx(
        cfg.camera_hfov_rad / 2.0
    )
    assert bearing_from_camera(right, cfg.camera_hfov_rad) == pytest.approx(
        -cfg.camera_hfov_rad / 2.0
    )


def test_bearing_sign_physically_consistent_between_sensors(
    cfg: FusionConfig,
) -> None:
    """F6.1 fiziksel tutarlılık: AYNI nesne iki sensörde AYNI işareti vermeli.

    Üreteçten BAĞIMSIZ ham geometri: (5, +3)'teki duba base_link'te SOLDA
    (atan2 → +). Aynı duba görüntüde SOL yarıda görünür (ör. bbox_cx=0.25)
    → kamera bearing'i de + olmalı. Bu test var olsaydı işaret hatası ilk
    günden yakalanırdı (bulgular #3 / F5.9 / F6.1).
    """
    lidar_left = LidarDetection(x=5.0, y=3.0, radius=0.15)     # solda
    cam_left = CameraDetection(bbox_cx=0.25, bbox_cy=0.5, class_id=0, score=0.9)
    b_lidar = bearing_from_lidar(lidar_left)
    b_cam = bearing_from_camera(cam_left, cfg.camera_hfov_rad)
    assert b_lidar > 0.0 and b_cam > 0.0, (
        f"soldaki nesne iki sensörde de + bearing vermeli "
        f"(lidar={b_lidar:+.3f}, kamera={b_cam:+.3f})"
    )
    # ve sağ taraf için de simetrik
    lidar_right = LidarDetection(x=5.0, y=-3.0, radius=0.15)
    cam_right = CameraDetection(bbox_cx=0.75, bbox_cy=0.5, class_id=0, score=0.9)
    assert bearing_from_lidar(lidar_right) < 0.0
    assert bearing_from_camera(cam_right, cfg.camera_hfov_rad) < 0.0


def test_associate_left_object_with_raw_geometry(cfg: FusionConfig) -> None:
    """F6.1: üreteç kullanmadan, ham geometriyle soldaki duba eşleşmeli.

    (5, +3) → lidar bearing +0.540; fiziksel görüntü konumu
    bbox_cx = 0.5 − 0.540/1.2 = 0.05 (sol kenara yakın). Eski işaretle bu
    çift 1.08 rad ayrık görünür ve eşleşmezdi.
    """
    lidar = [LidarDetection(x=5.0, y=3.0, radius=0.15)]
    bearing = math.atan2(3.0, 5.0)                             # +0.540
    cam = [CameraDetection(
        bbox_cx=0.5 - bearing / cfg.camera_hfov_rad,
        bbox_cy=0.5, class_id=CLASS_ENGEL, score=0.9,
    )]
    fused = associate(lidar, cam, cfg)
    assert fused[0].matched is True
    assert fused[0].class_id == CLASS_ENGEL


# ---------------------------------------------------------------- associate

def test_associate_matched_scene_assigns_camera_color(
    cfg: FusionConfig, rng: np.random.Generator
) -> None:
    lidar, camera = scene_fusion_matched(rng)
    fused = associate(lidar, camera, cfg)
    assert fused[0].matched and fused[0].class_id == CLASS_PARKUR_KENARI
    assert fused[1].matched and fused[1].class_id == CLASS_ENGEL


def test_associate_lidar_only_becomes_unknown(
    cfg: FusionConfig, rng: np.random.Generator
) -> None:
    lidar, camera = scene_fusion_matched(rng)
    fused = associate(lidar, camera, cfg)
    assert fused[2].matched is False
    assert fused[2].class_id == CLASS_UNKNOWN
    assert fused[2].score == pytest.approx(0.0)
    # LiDAR pozisyonu/yarıçapı korunmalı (güvenlik — engel olarak kalmalı)
    assert fused[2].x == pytest.approx(lidar[2].x)
    assert fused[2].y == pytest.approx(lidar[2].y)


def test_associate_camera_only_is_dropped(
    cfg: FusionConfig, rng: np.random.Generator
) -> None:
    lidar, camera = scene_fusion_camera_only(rng)
    assert associate(lidar, camera, cfg) == []


def test_associate_prevents_double_match(cfg: FusionConfig) -> None:
    # İki LiDAR aynı tek kamera tespitine aday — yalnız en yakın olan eşleşir.
    lidar = [
        LidarDetection(x=5.0, y=0.0, radius=0.15),      # bearing=0.000 (yakın)
        LidarDetection(x=5.0, y=0.3, radius=0.15),      # bearing≈0.060 (uzak)
    ]
    camera = [CameraDetection(bbox_cx=0.5, bbox_cy=0.5, class_id=CLASS_ENGEL, score=0.8)]
    fused = associate(lidar, camera, cfg)
    assert fused[0].matched is True and fused[0].class_id == CLASS_ENGEL
    assert fused[1].matched is False and fused[1].class_id == CLASS_UNKNOWN  # ikinci LiDAR boşta kalır


def test_associate_beyond_tolerance_stays_unmatched(cfg: FusionConfig) -> None:
    lidar = [LidarDetection(x=5.0, y=5.0, radius=0.15)]      # bearing=π/4≈0.785
    camera = [CameraDetection(bbox_cx=0.5, bbox_cy=0.5, class_id=0, score=0.9)]  # bearing=0
    fused = associate(lidar, camera, cfg)                    # fark >> tolerance(0.15)
    assert fused[0].matched is False
    assert fused[0].class_id == CLASS_UNKNOWN


def test_associate_empty_lidar_returns_empty() -> None:
    assert associate([], [CameraDetection(0.5, 0.5, 0, 0.9)], FusionConfig()) == []


def test_associate_empty_camera_all_unknown(cfg: FusionConfig) -> None:
    lidar = [LidarDetection(5.0, 0.0, 0.15), LidarDetection(6.0, 1.0, 0.2)]
    fused = associate(lidar, [], cfg)
    assert len(fused) == 2
    assert all(f.class_id == CLASS_UNKNOWN and not f.matched for f in fused)


# ---------------------------------------------------------------- uçtan uca

def test_scene_fusion_matched_end_to_end(
    cfg: FusionConfig, rng: np.random.Generator
) -> None:
    lidar, camera = scene_fusion_matched(rng)
    fused = associate(lidar, camera, cfg)
    assert len(fused) == 3
    assert sum(f.matched for f in fused) == 2
    assert sum(f.class_id == CLASS_UNKNOWN for f in fused) == 1
    assert all(isinstance(f, FusedObstacle) for f in fused)
