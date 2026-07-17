"""
Girdap İDA — LiDAR engel tespiti çekirdeği testleri (Sprint 1).

Filter → cluster → convert pipeline'ının izole ve uçtan uca doğrulaması;
sentetik sahneler (scene_minimum, scene_orta) fixture olarak kullanılır.

Çalıştır: pytest prototype/tests/test_lidar_obstacles.py -v
"""

from __future__ import annotations

import numpy as np
import pytest

# F16.2b: scipy×numpy ABI kırığı ValueError fırlatır; importorskip yalnız
# ImportError yakalar → çekirdek modülü elle kapıla (dürüst skip, hata değil).
try:
    from prototype.perception.lidar_obstacles import (
        CircleObstacle,
        LidarObstacleConfig,
        cluster_points,
        cluster_to_obstacle,
        detect_obstacles,
        filter_water_surface,
        to_base_link,
        voxel_downsample,
    )
except Exception as exc:  # ImportError VEYA ABI ValueError
    pytest.skip(
        f"lidar_obstacles import edilemedi (scipy/numpy): {exc}",
        allow_module_level=True,
    )
from prototype.perception.synthetic_lidar import (
    BuoySpec,
    generate_buoy_points,
    scene_minimum,
    scene_orta,
    scene_uzak_seyrek_duba,
    scene_yakin_duvar,
)


@pytest.fixture
def cfg() -> LidarObstacleConfig:
    return LidarObstacleConfig()


@pytest.fixture
def rng() -> np.random.Generator:
    return np.random.default_rng(42)


# ---------------------------------------------------------------- filtre

def test_filter_water_surface_removes_z_below_threshold(
    cfg: LidarObstacleConfig,
) -> None:
    points = np.array(
        [
            [5.0, 0.0, 0.05],    # su yüzeyi yansıması → atılmalı (z < 0.1)
            [5.0, 0.0, 0.3],     # duba gövdesi → kalmalı
            [5.0, 0.0, 4.0],     # yüksek yansıma → atılmalı (z > 3.0)
        ]
    )
    out = filter_water_surface(points, cfg)
    assert out.shape == (1, 3)
    assert out[0, 2] == pytest.approx(0.3)


def test_filter_water_surface_removes_out_of_range(
    cfg: LidarObstacleConfig,
) -> None:
    points = np.array(
        [
            [10.0, 0.0, 0.5],    # 10 m → kalmalı
            [30.0, 0.0, 0.5],    # 30 m > max_range=25 → atılmalı
            [20.0, 20.0, 0.5],   # √800 ≈ 28.3 m → atılmalı
        ]
    )
    out = filter_water_surface(points, cfg)
    assert out.shape == (1, 3)
    assert out[0, 0] == pytest.approx(10.0)


# ------------------------------------------------ F5.1: sensör → base_link

def test_to_base_link_identity_default() -> None:
    """h=0, θ=0 → dönüşüm birim (sensör≈base_link; eski davranış korunur)."""
    pts = np.array([[5.0, 1.0, 0.3], [10.0, -2.0, 0.5]])
    np.testing.assert_allclose(to_base_link(pts, LidarObstacleConfig()), pts)


def test_to_base_link_height_offset() -> None:
    """h>0 → sensör z'ye yükseklik eklenir; x,y değişmez (düz montaj)."""
    cfg = LidarObstacleConfig(lidar_height_m=0.6)
    pts = np.array([[5.0, 0.0, -0.6], [5.0, 0.0, -0.1]])   # sensör: su & duba tepesi
    out = to_base_link(pts, cfg)
    np.testing.assert_allclose(out[:, 2], [0.0, 0.5])       # base_link z (su datumu)
    np.testing.assert_allclose(out[:, :2], pts[:, :2])       # x,y korunur


def test_to_base_link_pitch_rotation() -> None:
    """θ=90° → y-ekseni rotasyonu: (x=1,z=0) → base_link'te (0, ., -1)."""
    cfg = LidarObstacleConfig(lidar_pitch_rad=np.pi / 2.0)
    out = to_base_link(np.array([[1.0, 0.0, 0.0]]), cfg)
    np.testing.assert_allclose(out[0], [0.0, 0.0, -1.0], atol=1e-9)


def test_f51_sensor_frame_buoys_lost_without_height(
    rng: np.random.Generator,
) -> None:
    """F5.1 BUG kanıtı: LiDAR 0.6 m yüksekte → dubalar sensör-z<0; düzeltme
    (lidar_height_m) verilmezse HEPSİ z_min ile elenir → 0 engel."""
    sensor_scene = scene_minimum(rng).copy()   # base_link (z∈[0,0.5])
    sensor_scene[:, 2] -= 0.6                   # base_link → sensör (su z=-0.6)
    lost = detect_obstacles(sensor_scene, LidarObstacleConfig(lidar_height_m=0.0))
    assert lost == []                          # bug: su hizası dubaları kaybolur


def test_f51_sensor_frame_buoys_recovered_with_height(
    rng: np.random.Generator,
) -> None:
    """F5.1 FIX: aynı sensör-çerçeve sahne + doğru lidar_height_m → 3 duba geri."""
    sensor_scene = scene_minimum(rng).copy()
    sensor_scene[:, 2] -= 0.6
    found = detect_obstacles(sensor_scene, LidarObstacleConfig(lidar_height_m=0.6))
    assert len(found) == 3                      # su datumuna taşındı → dubalar korunur


# ---------------------------------------------------------------- clustering

def test_cluster_single_buoy_returns_one_cluster(
    cfg: LidarObstacleConfig, rng: np.random.Generator
) -> None:
    points = generate_buoy_points(BuoySpec(5.0, 0.0), rng)
    clusters = cluster_points(points, cfg)
    assert len(clusters) == 1
    assert len(clusters[0]) == 40


def test_cluster_two_far_buoys_returns_two_clusters(
    cfg: LidarObstacleConfig, rng: np.random.Generator
) -> None:
    # 5 m ara >> cluster_tolerance=0.5 → iki ayrı bileşen
    points = np.vstack(
        [
            generate_buoy_points(BuoySpec(5.0, 0.0), rng),
            generate_buoy_points(BuoySpec(10.0, 0.0), rng),
        ]
    )
    assert len(cluster_points(points, cfg)) == 2


def test_cluster_below_min_size_filtered_out(
    cfg: LidarObstacleConfig, rng: np.random.Generator
) -> None:
    # 3 nokta < min_cluster_size=5 → noise sayılır
    points = generate_buoy_points(
        BuoySpec(5.0, 0.0, points_per_buoy=3), rng
    )
    assert cluster_points(points, cfg) == []


# ---------------------------------------------------------------- dönüşüm

def test_cluster_to_obstacle_centroid_correct() -> None:
    # Merkez (2, 3) etrafında simetrik 4 nokta → centroid tam (2, 3)
    cluster = np.array(
        [
            [1.0, 3.0, 0.5],
            [3.0, 3.0, 0.5],
            [2.0, 2.0, 0.5],
            [2.0, 4.0, 0.5],
        ]
    )
    obs = cluster_to_obstacle(cluster)
    assert obs.center_x == pytest.approx(2.0)
    assert obs.center_y == pytest.approx(3.0)
    assert obs.point_count == 4


def test_cluster_to_obstacle_radius_covers_all_points(
    rng: np.random.Generator,
) -> None:
    cluster = generate_buoy_points(BuoySpec(5.0, 0.0), rng)
    obs = cluster_to_obstacle(cluster)
    dists = np.sqrt(
        (cluster[:, 0] - obs.center_x) ** 2
        + (cluster[:, 1] - obs.center_y) ** 2
    )
    assert obs.radius == pytest.approx(dists.max())   # çevrel yarıçap
    assert np.all(dists <= obs.radius + 1e-12)        # tüm noktalar içeride


# ---------------------------------------------------------------- pipeline

def test_detect_obstacles_scene_minimum(
    cfg: LidarObstacleConfig, rng: np.random.Generator
) -> None:
    obstacles = detect_obstacles(scene_minimum(rng), cfg)
    assert len(obstacles) == 3
    # Her duba merkezi bir tespit merkezine ~duba yarıçapı kadar yakın olmalı
    expected = [(5.0, 0.0), (10.0, 3.0), (15.0, -2.0)]
    for ex, ey in expected:
        best = min(
            np.hypot(o.center_x - ex, o.center_y - ey) for o in obstacles
        )
        assert best < 0.3


def test_detect_obstacles_scene_orta(
    cfg: LidarObstacleConfig, rng: np.random.Generator
) -> None:
    obstacles = detect_obstacles(scene_orta(rng), cfg)
    assert len(obstacles) == 5                        # noise filtrelendi
    for obs in obstacles:
        assert isinstance(obs, CircleObstacle)
        assert obs.radius < 0.5                       # duba ölçeği, dev blob yok
        assert obs.point_count >= cfg.min_cluster_size


def test_detect_obstacles_empty_input(cfg: LidarObstacleConfig) -> None:
    assert detect_obstacles(np.empty((0, 3)), cfg) == []


# ------------------------------------------------ F5.3: eşdeğerlik + voxel

def _reference_clusters(points: np.ndarray, tol: float) -> set[frozenset[int]]:
    """O(n²) referans Öklid clustering (BFS) — hızlı implementasyonun teli."""
    n = len(points)
    d2 = ((points[:, None, :] - points[None, :, :]) ** 2).sum(axis=2)
    adj = d2 <= tol * tol
    seen = np.zeros(n, dtype=bool)
    out: set[frozenset[int]] = set()
    for s in range(n):
        if seen[s]:
            continue
        stack, comp = [s], []
        seen[s] = True
        while stack:
            i = stack.pop()
            comp.append(i)
            for j in np.flatnonzero(adj[i]):
                if not seen[j]:
                    seen[j] = True
                    stack.append(int(j))
        out.add(frozenset(comp))
    return out


def test_cluster_points_matches_bruteforce_reference(
    rng: np.random.Generator,
) -> None:
    """F5.3 emniyet ağı: hızlı clustering, O(n²) referansla AYNI bileşenleri
    bulmalı (boyut filtresi kapalı — saf bağlantılılık karşılaştırması)."""
    pts = rng.uniform(-10.0, 10.0, size=(200, 3))
    cfg = LidarObstacleConfig(min_cluster_size=1, max_cluster_size=10**9)
    got = cluster_points(pts, cfg)
    # Nokta koordinatlarından index kümelerine geri eşle
    index_of = {tuple(p): i for i, p in enumerate(map(tuple, pts))}
    got_sets = {
        frozenset(index_of[tuple(p)] for p in map(tuple, c)) for c in got
    }
    assert got_sets == _reference_clusters(pts, cfg.cluster_tolerance)


def test_voxel_downsample_zero_is_identity(rng: np.random.Generator) -> None:
    """voxel_size=0 → giriş birebir döner (çekirdek varsayılan davranışı)."""
    from prototype.perception.lidar_obstacles import voxel_downsample

    pts = rng.uniform(-5.0, 5.0, size=(50, 3))
    np.testing.assert_array_equal(voxel_downsample(pts, 0.0), pts)


def test_voxel_downsample_merges_and_preserves_centroid(
    rng: np.random.Generator,
) -> None:
    """Yoğun bulut voxel'lenince nokta sayısı düşmeli; genel centroid
    voxel boyutundan az kaymalı (temsil hatası sınırı)."""
    from prototype.perception.lidar_obstacles import voxel_downsample

    pts = rng.normal(0.0, 0.05, size=(2000, 3)) + np.array([5.0, 1.0, 0.3])
    out = voxel_downsample(pts, 0.1)
    assert len(out) < len(pts) / 4                     # ciddi seyrekleşme
    np.testing.assert_allclose(
        out.mean(axis=0), pts.mean(axis=0), atol=0.1   # ≤ voxel boyutu
    )


def test_detect_obstacles_scene_orta_with_voxel(
    rng: np.random.Generator,
) -> None:
    """Üretim config'i (voxel=0.1) sahne doğruluğunu BOZMAMALI: 5 duba,
    merkez hatası < 10 cm. (Jetson'da koşacak asıl yol budur — F6.2 dersi:
    üretimde açık olan bayrak testte de açık olmalı.)"""
    cfg = LidarObstacleConfig(voxel_size=0.1)          # diğer alanlar üretim varsayılanı
    obstacles = detect_obstacles(scene_orta(rng), cfg)
    assert len(obstacles) == 5
    for obs in obstacles:
        assert obs.radius < 0.5


# ------------------------------------------- F5.4/F6.3: küme boyutu uçları


def test_yakin_duvar_max_cluster_sinirini_asar(
    rng: np.random.Generator,
) -> None:
    """F6.3 ön koşulu: duvar, voxel SONRASI bile max_cluster_size'ı aşıyor.

    Bu geçmezse ana test (aşağıda) hiçbir şeyi kanıtlamaz — sahnenin
    üst sınırı gerçekten tetiklediği ayrıca doğrulanır (maskeleme avı)."""
    cfg = LidarObstacleConfig(voxel_size=0.1)
    filtered = voxel_downsample(
        filter_water_surface(scene_yakin_duvar(rng), cfg), cfg.voxel_size
    )
    wall_voxels = int((filtered[:, 0] > 5.0).sum())
    assert wall_voxels > cfg.max_cluster_size


def test_detect_obstacles_duvar_sessizce_silinmez(
    rng: np.random.Generator,
) -> None:
    """F5.4 ana test: en yakın büyük engel (duvar) haritadan KAYBOLMAMALI.

    Eski davranış: duvar kümesi > max_cluster_size → atılır → MPPI duvarı
    boş su sanır (Parkur-2/iskele çarpması). Yeni davranış: ızgarayla
    bölünür, duvar boyunca engel dairesi zinciri kalır."""
    cfg = LidarObstacleConfig(voxel_size=0.1)          # üretim config (F6.2)
    obstacles = detect_obstacles(scene_yakin_duvar(rng), cfg)

    wall = [o for o in obstacles if o.center_x > 5.0]
    assert wall, "duvar tamamen silinmiş — F5.4 gerilemesi"
    # Zincir duvarın iki ucuna da uzanmalı (y ∈ [-4, 4])
    ys = sorted(o.center_y for o in wall)
    assert ys[0] < -2.5 and ys[-1] > 2.5
    # Tek dev daire değil, mekânsal sınırlı parçalar (hücre yarı çaprazı + pay)
    for o in wall:
        assert o.radius <= 0.9

    # Dubalar duvara karışmadan ayrı tespit edilmeli
    buoys = [o for o in obstacles if o.center_x < 5.0]
    assert len(buoys) == 2
    for o in buoys:
        assert abs(o.center_x - 3.0) < 0.3 and o.radius < 0.5


def test_bolme_hicbir_noktayi_atmaz() -> None:
    """F5.4 sözleşmesi: bölme kayıpsızdır; alt-kümeler hücreyle sınırlı.

    min_cluster_size alt-kümelere UYGULANMAZ — büyük katı cismin kenar
    hücresindeki 1-2 nokta gürültü değil, cismin kanıtıdır."""
    xs, ys = np.meshgrid(np.arange(0.0, 4.0, 0.05), np.arange(0.0, 2.0, 0.05))
    blob = np.column_stack(
        (xs.ravel(), ys.ravel(), np.full(xs.size, 0.5))
    )                                                   # 3200 nokta, tek bileşen
    cfg = LidarObstacleConfig()                         # max=500 → bölme tetiklenir
    clusters = cluster_points(blob, cfg)
    assert len(clusters) > 1
    assert sum(len(c) for c in clusters) == len(blob)   # kayıpsız
    for c in clusters:
        xy = c[:, :2]
        extent = float(np.linalg.norm(xy.max(axis=0) - xy.min(axis=0)))
        assert extent <= np.sqrt(2.0) * cfg.split_cell_m + 1e-9


def test_uzak_seyrek_duba_min_sinirinin_altinda(
    rng: np.random.Generator,
) -> None:
    """F6.3: 24 m'deki 4 dönüşlü duba min_cluster_size=5 altında → görünmez.

    Bu BİLİNEN sınırlamayı belgeler (kamera 15 m'de zaten kör — F5.5;
    uzak algı sınırı sözleşmede). Eşik gevşetilirse aynı duba görünür —
    sınırın menzilde değil min_cluster_size'da olduğunun kanıtı."""
    pts = scene_uzak_seyrek_duba(rng)                  # tek sahne, iki config

    cfg = LidarObstacleConfig(voxel_size=0.1)
    assert detect_obstacles(pts, cfg) == []

    cfg_gevsek = LidarObstacleConfig(voxel_size=0.1, min_cluster_size=3)
    obstacles = detect_obstacles(pts, cfg_gevsek)
    assert len(obstacles) == 1
    assert abs(obstacles[0].center_x - 24.0) < 0.3
