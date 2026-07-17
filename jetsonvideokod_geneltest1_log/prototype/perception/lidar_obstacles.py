"""
Girdap İDA — LiDAR engel tespiti çekirdeği (Sprint 1, ROS-bağımsız).

Pipeline: Z-passthrough + menzil filtresi → (opsiyonel voxel downsample) →
cKDTree Öklid clustering (bağlantılı bileşen) → daire engel dönüşümü
(centroid + çevrel yarıçap).

Tasarım kararları:
    - DBSCAN yerine cKDTree.query_pairs + scipy connected_components: sklearn
      bağımlılığı yok, sabit yarıçaplı bağlantılı bileşen Sprint 1 için yeter.
      (F5.3: eski saf-Python Union-Find 20k noktada ~500 ms idi → 10 Hz
      tutmazdı; sparse csgraph + vektörize gruplama ile değiştirildi.)
    - voxel_size>0 → clustering öncesi voxel downsample (Livox 20k nokta/mesaj
      için Jetson bütçesi). 0 = kapalı (çekirdek varsayılanı, davranış birebir);
      üretim değeri params.yaml/hardware.yaml'da.
    - Çıktı CircleObstacle — planning'in /perception/obstacle_map PoseArray
      sözleşmesiyle (position.{x,y}=merkez, orientation.z=yarıçap) birebir.
    - Kaynak-bağımsız: girdi düz Nx3 numpy (x,y,z) — sentetik, Livox veya
      Gazebo point cloud'u aynı yoldan geçer (replaceable design).

Kullanım:
    obstacles = detect_obstacles(points, LidarObstacleConfig())
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np
from scipy.sparse import coo_matrix
from scipy.sparse.csgraph import connected_components
from scipy.spatial import cKDTree


@dataclass
class CircleObstacle:
    """Daire engel — planning.rrt_star.CircleObstacle ile aynı semantik."""

    center_x: float
    center_y: float
    radius: float
    point_count: int        # cluster'daki nokta sayısı (kalite metriği)


@dataclass
class LidarObstacleConfig:
    """LiDAR engel tespiti parametreleri (config/hardware.yaml perception.lidar)."""

    z_min: float = 0.1              # base_link'e göre su üstü kesim (m)
    z_max: float = 3.0              # yüksek yansımaları (kuş, direk tepesi) at
    cluster_tolerance: float = 0.5  # aynı cluster'da olma mesafesi (m)
    min_cluster_size: int = 5       # bu altı noise sayılır
    max_cluster_size: int = 500     # bu üstü tek engel değil büyük yapı →
                                    # ATILMAZ, split_cell_m ızgarasıyla
                                    # bölünür (F5.4)
    split_cell_m: float = 1.0       # F5.4: bölme ızgarası XY hücre kenarı (m);
                                    # engel dairesi ≤ hücre yarı çaprazı ~0.71 m
    max_range: float = 25.0         # LiDAR yatay menzil filtresi (m)
    voxel_size: float = 0.0         # m; >0 → clustering öncesi downsample
                                    # (F5.3). 0 = kapalı. Duba r=0.15 için
                                    # 0.1 m güvenli (merkez hatası ≤ voxel).
    # F5.1: LiDAR montaj geometrisi. Ham Livox noktaları SENSÖR (livox_frame)
    # çerçevesinde gelir (z=0 = sensörün kendisi). z_min/z_max ve engel
    # merkezleri base_link (su datumu) çerçevesinde tanımlı → noktalar önce
    # to_base_link ile taşınır. Değerler SAHADA ölçülür (0 = sensör≈base_link).
    lidar_height_m: float = 0.0     # optik merkezin base_link üstü yüksekliği (m)
    lidar_pitch_rad: float = 0.0    # y-ekseni eğimi (rad; 0 = düz montaj)


def to_base_link(points: np.ndarray, cfg: LidarObstacleConfig) -> np.ndarray:
    """Nx3 sensör-çerçevesi (livox_frame) noktalarını base_link'e taşı (F5.1).

    LiDAR base_link üstünde ``lidar_height_m`` yükseklikte ve y-ekseni etrafında
    ``lidar_pitch_rad`` ile eğik monteli. Ham Livox noktalarında z=0 SENSÖRÜ
    gösterir, su yüzeyini DEĞİL; dönüşüm yapılmazsa su hizasındaki duba
    sensör-z≈-h'de kalır ve z_min ile YANLIŞLIKLA elenir (gerçek veride engel
    KAYBI). θ=0, h=0 → birim dönüşüm (sensör≈base_link; eski davranış — sentetik
    sahneler base_link'te üretildiği için mevcut testler değişmez).

    Pitch, base_link y-ekseni etrafında R_y(θ):
        x' =  cosθ·x + sinθ·z
        z' = -sinθ·x + cosθ·z + h
    ⚠ İşaret sahada doğrulanmalı (montaj/optik-çerçeve varsayımı): burun-aşağı
    eğim +θ kabul edilir; canlı testte ters çıkarsa config'de işaret çevrilir.
    """
    if points.size == 0:
        return points.reshape(0, 3)
    theta = cfg.lidar_pitch_rad
    if theta == 0.0:                                 # düz montaj: saf yükseklik ofseti
        out = points.astype(np.float64, copy=True)
        out[:, 2] += cfg.lidar_height_m
        return out
    c, s = np.cos(theta), np.sin(theta)
    x, y, z = points[:, 0], points[:, 1], points[:, 2]
    return np.column_stack(
        (c * x + s * z, y, -s * x + c * z + cfg.lidar_height_m)
    )


def filter_water_surface(
    points: np.ndarray, cfg: LidarObstacleConfig
) -> np.ndarray:
    """Nx3 (x,y,z) → Z aralığı + yatay menzil filtresi uygulanmış Nx3.

    Su yüzeyi yansımaları (z < z_min) ve menzil dışı noktalar atılır.
    """
    if points.size == 0:
        return points.reshape(0, 3)
    z = points[:, 2]
    range_xy_sq = points[:, 0] ** 2 + points[:, 1] ** 2
    mask = (
        (z >= cfg.z_min)
        & (z <= cfg.z_max)
        & (range_xy_sq <= cfg.max_range**2)     # sqrt'siz karşılaştırma
    )
    return points[mask]


def voxel_downsample(points: np.ndarray, voxel_size: float) -> np.ndarray:
    """Nx3 → voxel ızgarasında hücre-başına centroid (Mx3, M ≤ N).

    voxel_size ≤ 0 veya boş giriş → giriş birebir döner. Temsil hatası
    hücre başına ≤ voxel_size (centroid hücre içinde kalır).
    """
    if voxel_size <= 0.0 or len(points) == 0:
        return points
    keys = np.floor(points / voxel_size).astype(np.int64)
    _, inverse, counts = np.unique(
        keys, axis=0, return_inverse=True, return_counts=True
    )
    sums = np.zeros((len(counts), 3), dtype=np.float64)
    np.add.at(sums, inverse, points)
    return sums / counts[:, None]


def _split_oversized(cluster: np.ndarray, cell_m: float) -> list[np.ndarray]:
    """Aşırı büyük kümeyi XY ızgara hücrelerine böl — F5.4 "böl, atma".

    LiDAR'da nokta sayısı mesafeyle ters orantılıdır: üst sınırı aşan küme
    çoğu zaman EN YAKIN büyük engeldir (iskele, duvar, hedef dubası).
    Atmak MPPI'yi ona karşı kör eder; tek dev daire de serbest suyu kapatır.
    Bölme kayıpsızdır ve min_cluster_size alt-kümelere UYGULANMAZ — büyük
    katı cismin kenar hücresindeki 1-2 nokta gürültü değil, cismin kanıtı.
    """
    keys = np.floor(cluster[:, :2] / cell_m).astype(np.int64)
    _, inverse = np.unique(keys, axis=0, return_inverse=True)
    order = np.argsort(inverse, kind="stable")
    boundaries = np.flatnonzero(np.diff(inverse[order])) + 1
    return [cluster[idx] for idx in np.split(order, boundaries)]


def cluster_points(
    points: np.ndarray, cfg: LidarObstacleConfig
) -> list[np.ndarray]:
    """cKDTree Öklid clustering → boyut-işlenmiş cluster listesi (her biri Nx3).

    query_pairs(cluster_tolerance) komşuluk çizgesi kurar;
    scipy.sparse.csgraph.connected_components bağlantılı bileşenleri çıkarır
    (F5.3: tamamı vektörize — 20k noktada eski Python döngüsünün ~50 katı).
    |cluster| < min olanlar noise sayılıp elenir; |cluster| > max olanlar
    ELENMEZ, split_cell_m ızgarasıyla bölünür (F5.4).

    Not (tekne gövdesi): max_cluster_size'ın eski "tekne kendisi" gerekçesi
    artık bölmeyle karşılanmıyor; bugün gövde dönüşleri sensör çerçevesinde
    z_min filtresine takılıyor. F5.1 (lidar_height_m) düzeltmesi çerçeveyi
    kaydırdığında gövde görünür olursa min_range filtresi eklenmeli —
    F5.1 ile birlikte değerlendirilecek.
    """
    n = len(points)
    if n == 0:
        return []
    tree = cKDTree(points)
    pairs = tree.query_pairs(cfg.cluster_tolerance, output_type="ndarray")
    if len(pairs) == 0:
        labels = np.arange(n)                    # herkes kendi bileşeni
    else:
        adj = coo_matrix(
            (np.ones(len(pairs), dtype=np.int8), (pairs[:, 0], pairs[:, 1])),
            shape=(n, n),
        )
        _, labels = connected_components(adj, directed=False)
    # Vektörize gruplama: etikete göre sırala, sınırlardan böl
    order = np.argsort(labels, kind="stable")
    boundaries = np.flatnonzero(np.diff(labels[order])) + 1
    clusters: list[np.ndarray] = []
    for member_idx in np.split(order, boundaries):
        if len(member_idx) < cfg.min_cluster_size:
            continue                                   # noise
        if len(member_idx) <= cfg.max_cluster_size:
            clusters.append(points[member_idx])
        else:                                          # F5.4: böl, atma
            clusters.extend(
                _split_oversized(points[member_idx], cfg.split_cell_m)
            )
    return clusters


def cluster_to_obstacle(cluster: np.ndarray) -> CircleObstacle:
    """Cluster → daire engel: XY centroid + çevrel (circumscribed) yarıçap."""
    xy = cluster[:, :2]
    centroid = xy.mean(axis=0)
    radius = float(np.sqrt(((xy - centroid) ** 2).sum(axis=1)).max())
    return CircleObstacle(
        center_x=float(centroid[0]),
        center_y=float(centroid[1]),
        radius=radius,
        point_count=len(cluster),
    )


def detect_obstacles(
    points: np.ndarray, cfg: LidarObstacleConfig
) -> list[CircleObstacle]:
    """Tam pipeline: filter → (voxel) → cluster → convert. Boş girişte boş liste.

    Not: voxel_size>0 iken point_count VOXEL sayısıdır (ham nokta değil);
    min/max_cluster_size eşikleri de voxel sayısına uygulanır.
    """
    if points is None or points.size == 0:
        return []
    # F5.1: ham noktalar sensör çerçevesinde → önce base_link'e taşı, SONRA
    # z-filtre uygula (aksi halde su hizası dubaları z_min ile yanlış elenir).
    base = to_base_link(np.asarray(points, dtype=np.float64), cfg)
    filtered = filter_water_surface(base, cfg)
    filtered = voxel_downsample(filtered, cfg.voxel_size)
    return [cluster_to_obstacle(c) for c in cluster_points(filtered, cfg)]
