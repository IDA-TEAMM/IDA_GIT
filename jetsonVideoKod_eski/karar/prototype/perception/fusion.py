"""
Girdap İDA — Kamera-LiDAR bearing füzyonu çekirdeği (Sprint 3, ROS-bağımsız).

LiDAR (3D, renksiz cluster) ile kamera (2D bbox, renkli) tespitlerini ORTAK
KALİBRASYON OLMADAN eşleştirir: her iki sensörün bearing'i (yatay açısı)
hesaplanır, en yakın bearing'li çift greedy olarak birleştirilir.

⚠ TASARIM SINIRI (bilinçli, Sprint 4+'a bırakıldı):
    Gerçek intrinsic/extrinsic kamera projeksiyonu YOK. `bearing_from_camera`
    kamera bbox'ının yatay konumunu HFOV ile orantılı bir açıya çevirir —
    kaba bir yaklaşım. Kamera optik çerçevesi/base_link hizası varsayımına
    bağlı bir İŞARET KURALI içerir; gerçek donanımda sol/sağ ters çıkarsa
    `bearing_from_camera` içindeki işareti çevirmek yeterli olur (çağıran
    kod / çıktı sözleşmesi değişmez — bkz. CLAUDE.md Perception bölümü).

Eşleşmeyen LiDAR tespiti GÜVENLİK NEDENİYLE atılmaz — class_id=CLASS_UNKNOWN
(99) ile korunur (MPPI cost map'te hâlâ engel olarak sayılmalı). Eşleşmeyen
kamera tespiti atılır (3D konumu yok, planning'e taşınamaz).
"""

from __future__ import annotations

import math
from dataclasses import dataclass

#: LiDAR-only (eşleşmemiş, renksiz) tespit sınıfı — güvenlik: engel olarak tut.
#: Kamera sınıfları (0=parkur_kenari, 1=engel, 2=hedef) camera_buoys'ta tanımlı;
#: bu modül yalnız kendi placeholder sınıfını (99) ekler.
CLASS_UNKNOWN = 99


@dataclass
class LidarDetection:
    """/perception/obstacle_map'ten çıkarılan tek daire engel (base_link)."""

    x: float
    y: float
    radius: float


@dataclass
class CameraDetection:
    """/perception/buoys'tan çıkarılan tek bbox (normalize görüntü uzayı)."""

    bbox_cx: float     # yatay merkez, normalize [0, 1] (0=sol kenar, 1=sağ kenar)
    bbox_cy: float
    class_id: int
    score: float


@dataclass
class FusedObstacle:
    """Birleştirilmiş çıktı — /perception/classified_obstacles ön-hali."""

    x: float
    y: float
    radius: float
    class_id: int
    score: float
    matched: bool      # True: kamera+LiDAR eşleşti, False: yalnız LiDAR


@dataclass
class FusionConfig:
    """Füzyon parametreleri (config/hardware.yaml perception.fusion bloğu)."""

    bearing_tolerance_rad: float = 0.15   # ~8.6° — eşleşme kabul eşiği
    camera_hfov_rad: float = 1.2          # OAK-D Lite yatay FOV yaklaşık değeri


def bearing_from_lidar(det: LidarDetection) -> float:
    """LiDAR cluster'ının base_link'e göre yatay açısı (rad, atan2(y,x))."""
    return math.atan2(det.y, det.x)


def bearing_from_camera(det: CameraDetection, hfov: float) -> float:
    """Bbox yatay merkezinden kaba bearing yaklaşımı (rad).

    İşaret kuralı = `bearing_from_lidar` (atan2) ile AYNI: **sol pozitif.**
    Görüntünün sol yarısındaki nesne (bbox_cx<0.5) fiziksel olarak soldadır
    → + bearing. merkez (0.5) → 0; sol kenar → +hfov/2; sağ → −hfov/2.
    (F6.1 düzeltmesi: eski `(cx−0.5)·hfov` LiDAR'a göre TERSTİ; sentetik
    üretecin ters fonksiyonu hatayı maskeliyordu.) ⚠ Kamera ters/aynalı
    monte edilirse sahada yine burası tek değişim noktasıdır.
    """
    return (0.5 - det.bbox_cx) * hfov


def _circular_diff(a: float, b: float) -> float:
    """İki açı arası [-π, π] farkı — atan2 sarımı (CLAUDE.md heading kuralı)."""
    return math.atan2(math.sin(a - b), math.cos(a - b))


def associate(
    lidar_list: list[LidarDetection],
    camera_list: list[CameraDetection],
    cfg: FusionConfig,
) -> list[FusedObstacle]:
    """Greedy en-yakın-bearing eşleştirme.

    - Her LiDAR tespiti sonuca girer (eşleşmesin bile — CLASS_UNKNOWN ile).
    - Her kamera tespiti EN FAZLA bir LiDAR'a eşlenir (double-match yok);
      eşleşmeyen kamera tespiti atılır (3D konumu yok).
    - Aday çiftler bearing farkına göre küçükten büyüğe işlenir → global
      olarak en yakın çiftler önce kilitlenir (greedy, optimal değil ama
      kalibrasyonsuz bearing-only senaryoda yeterli).
    """
    candidates: list[tuple[float, int, int]] = []
    for i, lidar_det in enumerate(lidar_list):
        lidar_bearing = bearing_from_lidar(lidar_det)
        for j, camera_det in enumerate(camera_list):
            camera_bearing = bearing_from_camera(camera_det, cfg.camera_hfov_rad)
            diff = abs(_circular_diff(lidar_bearing, camera_bearing))
            if diff <= cfg.bearing_tolerance_rad:
                candidates.append((diff, i, j))
    candidates.sort(key=lambda c: c[0])

    matched_camera_for_lidar: dict[int, int] = {}
    used_camera_idx: set[int] = set()
    for _diff, i, j in candidates:
        if i in matched_camera_for_lidar or j in used_camera_idx:
            continue                                  # biri zaten kilitlendi
        matched_camera_for_lidar[i] = j
        used_camera_idx.add(j)

    fused: list[FusedObstacle] = []
    for i, lidar_det in enumerate(lidar_list):
        if i in matched_camera_for_lidar:
            camera_det = camera_list[matched_camera_for_lidar[i]]
            fused.append(
                FusedObstacle(
                    x=lidar_det.x, y=lidar_det.y, radius=lidar_det.radius,
                    class_id=camera_det.class_id, score=camera_det.score,
                    matched=True,
                )
            )
        else:
            fused.append(
                FusedObstacle(
                    x=lidar_det.x, y=lidar_det.y, radius=lidar_det.radius,
                    class_id=CLASS_UNKNOWN, score=0.0, matched=False,
                )
            )
    return fused
