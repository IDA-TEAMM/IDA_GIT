"""
Girdap İDA — Sentetik kamera-LiDAR füzyon senaryoları (Sprint 3 test fixture).

fusion.associate()'i beslemek için eşleşen/eşleşmeyen LiDAR+kamera tespit
çiftleri üretir. Kamera bbox'ları, verilen bearing_from_camera formülünün
TERSİ alınarak (bearing → bbox_cx) LiDAR bearing'iyle kasıtlı hizalanır —
gerçek kalibrasyon değil, yalnız senaryo iç-tutarlılığı.
"""

from __future__ import annotations

import numpy as np

from prototype.perception.camera_buoys import CLASS_ENGEL, CLASS_HEDEF, CLASS_PARKUR_KENARI
from prototype.perception.fusion import (
    CameraDetection,
    FusionConfig,
    LidarDetection,
    bearing_from_lidar,
)

#: Senaryo varsayılan HFOV — FusionConfig() varsayılanıyla aynı (OAK-D Lite).
_DEFAULT_HFOV = FusionConfig().camera_hfov_rad


def _cx_for_bearing(bearing: float, hfov: float) -> float:
    """bearing_from_camera'nın tersi — senaryo bbox'ını LiDAR bearing'ine kilitler.

    F6.1 ile birlikte çevrildi: sol (+bearing) nesne görüntünün SOL yarısına
    (cx<0.5) düşer — artık fiziksel olarak da tutarlı. (Eski `0.5 + b/hfov`
    soldaki dubayı görüntünün sağına koyuyordu; işaret hatasını maskeliyordu.)
    """
    return 0.5 - bearing / hfov


def scene_fusion_matched(
    rng: np.random.Generator, hfov: float = _DEFAULT_HFOV
) -> tuple[list[LidarDetection], list[CameraDetection]]:
    """3 LiDAR tespiti: A ve B kamerayla eşleşir (renkli), C yalnız LiDAR'da.

    A: tam ileride, turuncu (parkur kenarı).
    B: sağa açık, sarı (engel).
    C: LiDAR-only — karşılığı olan kamera tespiti yok → unmatched kalmalı.
    """
    lidar = [
        LidarDetection(x=5.0, y=0.0, radius=0.15),     # A: bearing=0
        LidarDetection(x=5.0, y=3.0, radius=0.15),     # B: bearing≈0.540
        LidarDetection(x=8.0, y=-6.0, radius=0.15),    # C: eşleşmesiz
    ]
    camera = [
        CameraDetection(
            bbox_cx=_cx_for_bearing(bearing_from_lidar(lidar[0]), hfov),
            bbox_cy=0.5, class_id=CLASS_PARKUR_KENARI, score=0.90,
        ),
        CameraDetection(
            bbox_cx=_cx_for_bearing(bearing_from_lidar(lidar[1]), hfov),
            bbox_cy=0.5, class_id=CLASS_ENGEL, score=0.85,
        ),
        # C'ye karşılık gelen kamera tespiti YOK (bilinçli — LiDAR-only testi)
    ]
    return lidar, camera


def scene_fusion_camera_only(
    rng: np.random.Generator,
) -> tuple[list[LidarDetection], list[CameraDetection]]:
    """LiDAR tespiti yok, tek kamera tespiti var → associate() boş dönmeli
    (3D konumu olmayan kamera-only tespit atılır)."""
    camera = [
        CameraDetection(bbox_cx=0.5, bbox_cy=0.5, class_id=CLASS_HEDEF, score=0.95)
    ]
    return [], camera
