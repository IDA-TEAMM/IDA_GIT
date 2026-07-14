"""
Girdap İDA — Sentetik LiDAR point cloud üreticisi (Sprint 1 test fixture).

Gerçek Livox verisi yokken pipeline'ı beslemek için duba (silindirik yüzey)
ve su yüzeyi noise noktaları üretir. Senaryolar:
    - scene_minimum: 3 duba, noise yok       → temel clustering doğrulaması
    - scene_orta:    5 duba + 200 su noise'ı → Z-filtre + noise eleme testi
    - scene_yakin_duvar: iskele duvarı + 2 duba → max_cluster_size ucu
      (F6.3: duvar voxel sonrası bile üst sınırı aşar — F5.4 bölme testi)
    - scene_uzak_seyrek_duba: 24 m'de 4 nokta → min_cluster_size ucu
      (F6.3: uzak-seyrek duba bilinen kör nokta, testle belgelenir)

Koordinatlar base_link (araç merkezi) çerçevesinde, metre.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np


@dataclass
class BuoySpec:
    """Duba tanımı — TEKNOFEST parkur dubası (~30 cm çap)."""

    x: float
    y: float
    radius: float = 0.15        # duba yarıçapı (30 cm çap / 2)
    height: float = 0.5         # su üstü görünür yükseklik
    points_per_buoy: int = 40   # LiDAR'ın dubadan aldığı yaklaşık dönüş sayısı


#: LiDAR menzil ölçüm gürültüsü (m) — Livox Mid-360 ~2 cm sınıfı.
_LIDAR_NOISE_SIGMA = 0.02


def generate_buoy_points(
    spec: BuoySpec, rng: np.random.Generator
) -> np.ndarray:
    """Silindirik duba yüzeyinden rastgele nokta örnekle → Nx3 (x,y,z)."""
    theta = rng.uniform(0.0, 2.0 * np.pi, spec.points_per_buoy)
    z = rng.uniform(0.0, spec.height, spec.points_per_buoy)
    points = np.column_stack(
        (
            spec.x + spec.radius * np.cos(theta),
            spec.y + spec.radius * np.sin(theta),
            z,
        )
    )
    return points + rng.normal(0.0, _LIDAR_NOISE_SIGMA, points.shape)


def generate_water_noise(
    n_points: int, area_m: float, rng: np.random.Generator
) -> np.ndarray:
    """Su yüzeyine yakın dağınık yansımalar → Nx3.

    z ∈ [-0.05, 0.08]: Z-passthrough'un (z_min=0.1) kesmesi gereken aralık —
    filtreleme testinin negatif örneği.
    """
    half = area_m / 2.0
    return np.column_stack(
        (
            rng.uniform(-half, half, n_points),
            rng.uniform(-half, half, n_points),
            rng.uniform(-0.05, 0.08, n_points),
        )
    )


def scene_minimum(rng: np.random.Generator) -> np.ndarray:
    """3 duba, noise yok — araç merkezine göre (5,0), (10,3), (15,-2)."""
    buoys = [BuoySpec(5.0, 0.0), BuoySpec(10.0, 3.0), BuoySpec(15.0, -2.0)]
    return np.vstack([generate_buoy_points(b, rng) for b in buoys])


def generate_wall_points(
    y0: float,
    y1: float,
    x: float,
    height: float,
    n_points: int,
    rng: np.random.Generator,
) -> np.ndarray:
    """Dikey düzlem yüzey (iskele/duvar/rıhtım) örnekle → Nx3.

    x sabit, y ∈ [y0, y1], z ∈ [0, height]; LiDAR gürültüsü eklenir.
    Gerçekte nokta sayısı mesafeyle ters orantılıdır — yakın büyük yüzey
    binlerce dönüş verir (F5.4'ün tetiklendiği durum).
    """
    y = rng.uniform(y0, y1, n_points)
    z = rng.uniform(0.0, height, n_points)
    points = np.column_stack((np.full(n_points, x), y, z))
    return points + rng.normal(0.0, _LIDAR_NOISE_SIGMA, points.shape)


def scene_yakin_duvar(rng: np.random.Generator) -> np.ndarray:
    """8 m'lik iskele duvarı (x=6) + 2 kenar dubası — F6.3/F5.4 sahnesi.

    Duvar: y ∈ [-4, 4], 1 m yüksek, 4000 nokta → voxel 0.1'de ~700+ hücre,
    max_cluster_size=500'ü AŞAR. Eski davranış duvarı sessizce atardı
    (en yakın büyük engel silinir, MPPI içinden geçer); F5.4 düzeltmesi
    ızgarayla böler. Dubalar duvardan 3 m önde, ayrı cluster kalmalı.
    """
    parts = [generate_wall_points(-4.0, 4.0, 6.0, 1.0, 4000, rng)]
    parts += [
        generate_buoy_points(b, rng)
        for b in (BuoySpec(3.0, -2.0), BuoySpec(3.0, 2.0))
    ]
    return np.vstack(parts)


def scene_uzak_seyrek_duba(rng: np.random.Generator) -> np.ndarray:
    """24 m'de 4 dönüşlü tek duba — min_cluster_size alt ucu (F6.3).

    Menzil sınırına (25 m) yakın dubadan LiDAR yalnız birkaç dönüş alır;
    min_cluster_size=5 bunu noise sayar → duba GÖRÜNMEZ. Bu bilinen
    sınırlamadır; test mevcut davranışı belgeler (sessiz varsayım değil).
    """
    return generate_buoy_points(
        BuoySpec(24.0, 0.0, points_per_buoy=4), rng
    )


def scene_orta(rng: np.random.Generator) -> np.ndarray:
    """5 duba + 200 su yüzeyi noise noktası — karışık gerçekçi sahne."""
    buoys = [
        BuoySpec(4.0, 1.0),
        BuoySpec(8.0, -2.0),
        BuoySpec(12.0, 4.0),
        BuoySpec(16.0, 0.0),
        BuoySpec(20.0, -3.0),
    ]
    parts = [generate_buoy_points(b, rng) for b in buoys]
    parts.append(generate_water_noise(200, 40.0, rng))
    return np.vstack(parts)
