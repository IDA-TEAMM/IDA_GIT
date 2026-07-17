"""
Girdap İDA — FusionPipeline birim testi (rclpy bağımsız).

Düz çizgi + 90° dönüş yörüngesi üretir, gürültülü IMU/velocity_body/GPS
streamleri sentezler, FusionPipeline'a besler ve smoother çıktısının
RMSE'sini ground truth'a karşı ölçer.

Hedef: iSAM2 RMSE < 0.5 m (CLAUDE.md saha gereksinimi).

Çalıştır: pytest prototype/tests/test_fusion_pipeline.py -v
"""

from __future__ import annotations

import math

import numpy as np
import pytest

# F16.2: pipeline modül düzeyinde gtsam ister — gtsam'sız makinede toplama
# hatası yerine dürüst skip (kurulum: pip install gtsam, ARM64'te kaynaktan).
pytest.importorskip("gtsam", reason="gtsam yok — iSAM2 füzyon testleri atlanır")

from prototype.fusion.pipeline import FusionPipeline, FusionPipelineConfig  # noqa: E402


# --------------------------------------------------------------------------- #
# Sentetik yörünge: düz → 90° sol dönüş → düz
# --------------------------------------------------------------------------- #


def _generate_truth(
    dt: float = 0.02,           # 50 Hz IMU adımı
    duration: float = 30.0,
    forward_speed: float = 1.0, # m/s
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """
    (truth_xy_psi, vel_body, omega_z) üretir.
        0-10 s   : düz, +x ekseni boyunca
        10-15 s  : 90° sol dönüş (omega = π/10 rad/s)
        15-30 s  : düz, +y ekseni boyunca

    Çıktılar:
        truth   : (N+1, 3) [x, y, psi]
        velbody : (N, 2)   [vx_body, vy_body]
        omega   : (N,)     yaw rate (rad/s)
    """
    n = int(duration / dt)
    times = np.arange(n) * dt

    omega = np.zeros(n)
    omega[(times >= 10.0) & (times < 15.0)] = math.pi / 10.0  # 90° / 5 s

    vx = np.full(n, forward_speed)
    vy = np.zeros(n)
    vel_body = np.column_stack([vx, vy])

    # Truth integrasyonu: dünya çerçevesinde dx = vx·cos(ψ)·dt, dy = vx·sin(ψ)·dt
    truth = np.zeros((n + 1, 3))
    for k in range(n):
        psi = truth[k, 2]
        truth[k + 1, 0] = truth[k, 0] + vx[k] * math.cos(psi) * dt
        truth[k + 1, 1] = truth[k, 1] + vx[k] * math.sin(psi) * dt
        truth[k + 1, 2] = truth[k, 2] + omega[k] * dt

    return truth, vel_body, omega


# --------------------------------------------------------------------------- #
# Sentetik sensör akışlarını boru hattından geçir
# --------------------------------------------------------------------------- #


def _run_pipeline_with_synthetic(
    truth: np.ndarray,
    vel_body: np.ndarray,
    omega: np.ndarray,
    dt: float,
    *,
    gps_period_s: float = 1.0,
    vel_sigma: float = 0.05,
    omega_sigma: float = 0.005,
    gps_sigma: float = 0.30,
    seed: int = 0,
    cfg: FusionPipelineConfig | None = None,
) -> np.ndarray:
    """
    Pipeline'ı sentetik streamle besler ve smooth poses'i (M, 3) döndürür.

    GPS lat/lon üretmek için mock origin (Marmaris koordinatlarını kullan;
    kesin değer önemli değil, projeksiyon iç tutarlı).
    """
    rng = np.random.default_rng(seed)
    fp = FusionPipeline(cfg or FusionPipelineConfig(gps_sigma_xy=gps_sigma))

    # Origin'i seed et: sıfır-fix → smoother (0,0)'da kalsın
    LAT0, LON0 = 36.85, 28.27          # Marmaris yarışma alanı civarı
    fp.on_gps(LAT0, LON0)               # ilk fix = origin, smoother değişmez

    n = vel_body.shape[0]
    gps_stride = max(1, int(round(gps_period_s / dt)))

    for k in range(n):
        t = (k + 1) * dt
        # 1) velocity_body (gürültülü, body-frame)
        vx = vel_body[k, 0] + rng.normal(0.0, vel_sigma)
        vy = vel_body[k, 1] + rng.normal(0.0, vel_sigma)
        fp.on_velocity(vx, vy)

        # 2) IMU (gyro yaw rate, gürültülü)
        wz = omega[k] + rng.normal(0.0, omega_sigma)
        fp.on_imu(t, wz)

        # 3) GPS — gps_period_s aralıklarla (k+1 tabanlı sayım)
        if (k + 1) % gps_stride == 0:
            tx = truth[k + 1, 0] + rng.normal(0.0, gps_sigma)
            ty = truth[k + 1, 1] + rng.normal(0.0, gps_sigma)
            lat, lon = fp.enu_to_latlon(tx, ty)
            fp.on_gps(lat, lon)

    return fp.all_xy_psi()


# --------------------------------------------------------------------------- #
# Testler
# --------------------------------------------------------------------------- #


def _resample_truth_to_keys(
    truth: np.ndarray, n_keys: int, dt: float
) -> np.ndarray:
    """
    Smoother her odom_period_s'te bir key üretir; truth daha sık örneklenmiş.
    Key sayısı kadar uniform örnek al ki RMSE eşleşsin.
    """
    # Truth indeksleri: 0..N. Smoother key sayısı n_keys (origin dahil).
    idxs = np.linspace(0, truth.shape[0] - 1, n_keys).astype(int)
    return truth[idxs]


def test_fusion_pipeline_rmse_under_threshold() -> None:
    """Smoother çıktısı ground-truth'a karşı RMSE < 0.5 m."""
    dt = 0.02   # 50 Hz IMU
    truth, vel_body, omega = _generate_truth(dt=dt, duration=30.0)
    smooth = _run_pipeline_with_synthetic(truth, vel_body, omega, dt)

    # Smoother key cadansı = odom_period_s = 0.1 s. Truth dt = 0.02 s.
    # Truth'u key sayısına ölçekle.
    truth_aligned = _resample_truth_to_keys(truth, smooth.shape[0], dt)

    err = np.linalg.norm(smooth[:, :2] - truth_aligned[:, :2], axis=1)
    rmse = float(np.sqrt((err ** 2).mean()))
    final_err = float(err[-1])

    print(f"\n[fusion test] keys={smooth.shape[0]}, RMSE={rmse:.3f} m, "
          f"final_err={final_err:.3f} m")

    assert rmse < 0.5, f"iSAM2 RMSE {rmse:.3f} m > 0.5 m hedefini geçti"


def test_fusion_pipeline_origin_lock_no_drift() -> None:
    """Sıfır hareket + sıfır gürültü senaryosunda smoother (0,0)'da kalmalı."""
    fp = FusionPipeline()
    fp.on_gps(36.85, 28.27)  # origin

    for k in range(50):
        t = (k + 1) * 0.02
        fp.on_velocity(0.0, 0.0)
        fp.on_imu(t, 0.0)

    x, y, psi = fp.current_pose()
    assert abs(x) < 0.05 and abs(y) < 0.05 and abs(psi) < 0.01


def test_fusion_pipeline_latlon_roundtrip() -> None:
    """ENU↔lat/lon projeksiyonu kendi kendinin tersi olmalı (<1 mm)."""
    fp = FusionPipeline()
    fp.on_gps(36.85, 28.27)

    for x_in, y_in in [(0.0, 0.0), (10.0, 5.0), (-50.0, 75.0), (200.0, -120.0)]:
        lat, lon = fp.enu_to_latlon(x_in, y_in)
        x_out, y_out = fp._latlon_to_enu(lat, lon)
        assert abs(x_out - x_in) < 1e-3
        assert abs(y_out - y_in) < 1e-3
