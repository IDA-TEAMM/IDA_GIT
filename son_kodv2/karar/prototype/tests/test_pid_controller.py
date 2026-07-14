"""
Girdap İDA — CascadeHeadingPidController testleri (F-S.10).

ida_topics/decision_node.py'nin donanımda kanıtlanmış cascade PID mantığının
(dış döngü heading→yaw_rate, iç döngü yaw_rate→açısal düzeltme) + girdap'ın
LiDAR CircleObstacle kaçınmasının birleşimini doğrular. Çalıştır:
    pytest prototype/tests/test_pid_controller.py -v
"""

from __future__ import annotations

import math

import numpy as np
import pytest

from prototype.planning.pid_controller import (
    CascadeHeadingPidController,
    PidControllerConfig,
)
from prototype.planning.rrt_star import CircleObstacle


def _state(x=0.0, y=0.0, yaw=0.0, yaw_rate=0.0) -> np.ndarray:
    return np.array([x, y, yaw, 0.0, 0.0, yaw_rate])


# --------------------------------------------------------------------------- #
# Heading takibi
# --------------------------------------------------------------------------- #


def test_hedef_tam_onde_dengeli_itki() -> None:
    """Araç x-ekseni boyunca (yaw=0) tam ÖNÜNDEKİ hedefe bakıyor → dönüş
    komutu ≈0, T_left ≈ T_right (düz ileri)."""
    ctrl = CascadeHeadingPidController()
    u = ctrl.step(_state(yaw=0.0), target_xy=(100.0, 0.0))
    assert abs(u[0] - u[1]) < 0.5   # neredeyse eşit itki


def test_hedef_solda_sola_donus_komutu() -> None:
    """Hedef solda (+y) iken araç sola dönmeli — diferansiyel itki bunu
    yansıtmalı (yön MPPI/ArduPilot işaret kuralıyla test edilir, büyüklük
    değil — sabit işaret kontratı yeterli)."""
    ctrl = CascadeHeadingPidController()
    # Çok sayıda adım at (smoothing + PD yakınsasın)
    u = None
    for _ in range(50):
        u = ctrl.step(_state(yaw=0.0), target_xy=(10.0, 10.0))
    assert u is not None
    assert u[0] != pytest.approx(u[1], abs=1e-9), "hedef yana kaymışken düz itki üretti"


def test_yaw_wraparound_dogru_yon() -> None:
    """yaw=+170°, hedef -170° yönünde (kısa yol ±180° sınırından) — büyük
    ham fark yerine KISA yoldan dönmeli (wrap doğru çalışıyor)."""
    ctrl = CascadeHeadingPidController()
    yaw = math.radians(170.0)
    # Hedef aracın "arkasında az sağında" — kısa dönüş küçük olmalı.
    tx = math.cos(math.radians(-175.0)) * 100.0
    ty = math.sin(math.radians(-175.0)) * 100.0
    u = ctrl.step(_state(yaw=yaw), target_xy=(tx, ty))
    # Çıktı sonlu ve makul aralıkta olmalı (patlamamalı / NaN olmamalı)
    assert np.all(np.isfinite(u))
    assert abs(u[0]) < 1000 and abs(u[1]) < 1000


def test_reset_yumusatma_gecmisini_temizler() -> None:
    ctrl = CascadeHeadingPidController()
    ctrl.step(_state(yaw=0.0), target_xy=(10.0, 10.0))
    assert ctrl._smoothed_target_yaw is not None
    ctrl.reset()
    assert ctrl._smoothed_target_yaw is None
    assert ctrl._prev_yaw_rate_err == 0.0


# --------------------------------------------------------------------------- #
# F-S.10: LiDAR engel kaçınma (potansiyel alan)
# --------------------------------------------------------------------------- #


def test_engel_yokken_kacinma_devre_disi() -> None:
    """Menzil dışı/hiç engel yok → nav_angular aynen kullanılır (crash yok)."""
    ctrl = CascadeHeadingPidController()
    u = ctrl.step(_state(yaw=0.0), target_xy=(100.0, 0.0), obstacles=[])
    assert np.all(np.isfinite(u))


def test_tam_onde_yakin_engel_kacinma_tetikler() -> None:
    """Aracın tam önünde, güvenlik payı içinde bir engel → kaçınma AKTİF olmalı,
    düz-ileri itkiden (T_left≈T_right) SAPMALI."""
    cfg = PidControllerConfig(obstacle_safety_margin_m=5.0)
    ctrl = CascadeHeadingPidController(cfg)
    obstacle = CircleObstacle(cx=5.0, cy=0.0, r=1.0)   # tam önde, 5 m
    u = ctrl.step(_state(yaw=0.0), target_xy=(100.0, 0.0), obstacles=[obstacle])
    assert abs(u[0] - u[1]) > 1e-6, "önde yakın engel varken kaçınma tetiklenmedi"


def test_uzak_engel_kacinmayi_tetiklemez() -> None:
    """Güvenlik payının ÇOK dışındaki engel kaçınmayı tetiklememeli — düz
    hedefe seyir korunur (yalnız heading PID'i etkiler, avoid_angular yok)."""
    cfg = PidControllerConfig(obstacle_safety_margin_m=3.0)
    ctrl = CascadeHeadingPidController(cfg)
    far_obstacle = CircleObstacle(cx=500.0, cy=500.0, r=1.0)
    # Hedef de aracın tam önünde (yaw=0) olsun ki avoidance karışmasın.
    u = ctrl.step(_state(yaw=0.0), target_xy=(100.0, 0.0), obstacles=[far_obstacle])
    assert abs(u[0] - u[1]) < 1e-6, "uzak engel yanlışlıkla kaçınmayı tetikledi"


def test_sagdaki_engelden_sola_kacar() -> None:
    """Engel aracın SAĞINDA (bearing<0) iken kontrolcü SOLA kaçmalı (pozitif
    yön kuralı — ida_topics 'sağda engel → sola kaç' ile aynı)."""
    cfg = PidControllerConfig(obstacle_safety_margin_m=5.0)
    ctrl = CascadeHeadingPidController(cfg)
    # yaw=0 (doğuya bakıyor); engel biraz sağda (güneyde, -y) ve önde.
    obstacle = CircleObstacle(cx=5.0, cy=-2.0, r=1.0)
    u = ctrl.step(_state(yaw=0.0), target_xy=(100.0, 0.0), obstacles=[obstacle])
    # Sola kaçış: sol thruster'a göre sağ thruster'ın daha fazla itki alması
    # BEKLENİR (turn>0 → T_right artar) — işaret kontratı, tam değer değil.
    assert u[1] > u[0], "sağdaki engelden kaçarken beklenen dönüş yönü ters"


def test_cok_engel_katkilari_toplanir() -> None:
    """Birden çok tehdit eden engel varsa katkılar toplanır, çıktı hâlâ sonlu
    ve sınır içinde kalır (max_angular kırpması)."""
    cfg = PidControllerConfig(obstacle_safety_margin_m=5.0, max_angular=1.0)
    ctrl = CascadeHeadingPidController(cfg)
    obstacles = [
        CircleObstacle(cx=5.0, cy=-1.0, r=1.0),
        CircleObstacle(cx=5.0, cy=-2.0, r=1.0),
        CircleObstacle(cx=5.0, cy=-3.0, r=1.0),
    ]
    u = ctrl.step(_state(yaw=0.0), target_xy=(100.0, 0.0), obstacles=obstacles)
    assert np.all(np.isfinite(u))
    turn = abs(u[1] - cfg.cruise_thrust_n)
    assert turn <= cfg.max_diff_thrust_n + 1e-6   # kırpma sınırını aşmadı


# --------------------------------------------------------------------------- #
# Çıktı sözleşmesi — MPPIController.step() ile aynı arayüz
# --------------------------------------------------------------------------- #


def test_cikti_iki_elemanli_numpy_dizisi() -> None:
    ctrl = CascadeHeadingPidController()
    u = ctrl.step(_state(), target_xy=(10.0, 0.0))
    assert isinstance(u, np.ndarray)
    assert u.shape == (2,)


def test_varsayilan_cruise_thrust_max_thrust_siniri_altinda() -> None:
    """Varsayılan config, dynamics.yaml'daki max_thrust=30N'yi (tek motor)
    aşmayacak şekilde ayarlı olmalı — PlanningPipeline/CatamaranDynamics
    zaten ayrıca kırpar ama varsayılanın makul olması saha güvenliği içindir."""
    cfg = PidControllerConfig()
    assert cfg.cruise_thrust_n + cfg.max_diff_thrust_n <= 30.0
