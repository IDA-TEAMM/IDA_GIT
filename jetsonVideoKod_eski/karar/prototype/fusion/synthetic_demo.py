"""
Girdap İDA — iSAM2 smoother sentetik veri demosu.

Akış:
    1) Katamaran 3-DOF dinamik modeliyle bilinen bir görev profili koş
       (ground-truth yörünge üret).
    2) Her adımda IMU pre-integration deltası + her saniye GPS örneği
       gürültülü olarak simüle et.
    3) Yörüngeyi üç şekilde karşılaştır:
         - Ground truth                  → referans
         - Sadece IMU dead-reckoning     → drift hattı
         - iSAM2 smoother (IMU + GPS)    → fizyon çıktısı
    4) Karşılaştırma figürünü docs/KTR/isam2_smoother_demo.png olarak kaydet.

KTR raporu için iki panel: yörünge planı + anlık konum hatası grafiği.

Çalıştırma:
    python -m prototype.fusion.synthetic_demo
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import numpy as np
import matplotlib.pyplot as plt
import gtsam

from prototype.dynamics.catamaran import CatamaranDynamics
from prototype.fusion.isam2_smoother import ISAM2Smoother, ISAM2SmootherConfig


# Repo kökü: bu dosya …/prototype/fusion/synthetic_demo.py
_REPO_ROOT = Path(__file__).resolve().parents[2]


@dataclass
class DemoConfig:
    """Sentetik demo parametreleri — sihirli sayı yok, hepsi burada."""

    dt: float = 0.05                     # entegrasyon adımı (s) ↔ IMU 20 Hz
    duration: float = 60.0               # toplam simülasyon süresi (s)
    gps_period: float = 1.0              # GPS örnekleme periyodu (s)

    # Simüle edilen sensör gürültüleri
    gps_sigma_xy: float = 0.50           # m, demoda RTK-fix DEĞİL — drama
    imu_sigma_xy: float = 0.05           # m, body-frame adım gürültüsü
    imu_sigma_psi: float = 0.01          # rad, body-frame yaw gürültüsü

    seed: int = 0
    output_path: Path = _REPO_ROOT / "docs" / "KTR" / "isam2_smoother_demo.png"


def control_profile(n_steps: int, dt: float) -> np.ndarray:
    """
    Yarışma görev profili: düz → sağa dönüş → düz → sola dönüş → düz.
    Çıktı: (n_steps, 2) [T_left, T_right] (N).
    """
    times = np.arange(n_steps) * dt
    ctrl = np.tile(np.array([12.0, 12.0]), (n_steps, 1))   # düz ileri
    ctrl[(times >= 15.0) & (times < 25.0)] = [14.0, 8.0]   # sağa
    ctrl[(times >= 40.0) & (times < 50.0)] = [8.0, 14.0]   # sola
    return ctrl


def simulate_truth(controls: np.ndarray, dt: float) -> np.ndarray:
    """3-DOF model + RK4 ile ground-truth state geçmişi: (N+1, 6)."""
    # Param'sız çağrı → configs/dynamics.yaml otomatik yüklenir
    return CatamaranDynamics().simulate(np.zeros(6), controls, dt)


def simulate_sensors(
    truth: np.ndarray,
    cfg: DemoConfig,
    rng: np.random.Generator,
) -> tuple[np.ndarray, np.ndarray]:
    """
    Ground-truth poselerden gürültülü ölçümler türet.
        imu_deltas  : (N, 3)  body-frame [Δx, Δy, Δψ] + gürültü
        gps_records : (M, 3)  [step_idx, x_meas, y_meas]
    """
    xy_psi = truth[:, [0, 1, 2]]
    n_steps = xy_psi.shape[0] - 1

    imu_deltas = np.zeros((n_steps, 3))
    sigmas = np.array([cfg.imu_sigma_xy, cfg.imu_sigma_xy, cfg.imu_sigma_psi])
    for k in range(n_steps):
        prev_pose = gtsam.Pose2(*xy_psi[k])
        next_pose = gtsam.Pose2(*xy_psi[k + 1])
        # body-frame relatif poz: prev^-1 ⊕ next
        delta = prev_pose.between(next_pose)
        noise = rng.normal(0.0, sigmas)
        imu_deltas[k] = [
            delta.x() + noise[0],
            delta.y() + noise[1],
            delta.theta() + noise[2],
        ]

    gps_stride = max(1, int(round(cfg.gps_period / cfg.dt)))
    gps_idx = np.arange(0, xy_psi.shape[0], gps_stride)
    gps_records = np.column_stack(
        [
            gps_idx.astype(float),
            xy_psi[gps_idx, 0] + rng.normal(0.0, cfg.gps_sigma_xy, gps_idx.size),
            xy_psi[gps_idx, 1] + rng.normal(0.0, cfg.gps_sigma_xy, gps_idx.size),
        ]
    )
    return imu_deltas, gps_records


def dead_reckon(pose0: gtsam.Pose2, imu_deltas: np.ndarray) -> np.ndarray:
    """Sadece IMU delta'larını topla — drift referans hattı. Çıktı: (N+1, 3)."""
    poses = [pose0]
    for d in imu_deltas:
        poses.append(poses[-1].compose(gtsam.Pose2(*d)))
    return np.array([[p.x(), p.y(), p.theta()] for p in poses])


def run_smoother(
    pose0: gtsam.Pose2,
    imu_deltas: np.ndarray,
    gps_records: np.ndarray,
    cfg: DemoConfig,
) -> np.ndarray:
    """iSAM2 inkremental: her adımda update; GPS olduğu adımda prior ekle."""
    sm = ISAM2Smoother(
        ISAM2SmootherConfig(
            odom_sigma_xy=cfg.imu_sigma_xy,
            odom_sigma_psi=cfg.imu_sigma_psi,
            gps_sigma_xy=cfg.gps_sigma_xy,
        )
    )
    sm.initialize(pose0)

    gps_at = {int(rec[0]): (rec[1], rec[2]) for rec in gps_records}

    if 0 in gps_at:
        sm.add_gps(0, *gps_at[0])
        sm.update()

    for k, delta_xytheta in enumerate(imu_deltas, start=1):
        sm.add_odometry(gtsam.Pose2(*delta_xytheta))
        if k in gps_at:
            sm.add_gps(k, *gps_at[k])
        sm.update()

    return sm.all_xy_psi()


def plot_results(
    truth: np.ndarray,
    dr: np.ndarray,
    gps_records: np.ndarray,
    smooth: np.ndarray,
    cfg: DemoConfig,
) -> tuple[float, float]:
    """KTR figürünü çiz, kaydet ve (RMSE_DR, RMSE_smoother) döndür."""
    truth_xy = truth[:, :2]
    t_arr = np.arange(truth_xy.shape[0]) * cfg.dt
    err_dr = np.linalg.norm(dr[:, :2] - truth_xy, axis=1)
    err_sm = np.linalg.norm(smooth[:, :2] - truth_xy, axis=1)

    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(12, 5))

    ax1.plot(truth_xy[:, 0], truth_xy[:, 1],
             color="black", lw=2.0, label="Ground truth")
    ax1.plot(dr[:, 0], dr[:, 1],
             color="tab:red", lw=1.0, ls="--",
             label="Dead-reckoning (sadece IMU)")
    ax1.scatter(gps_records[:, 1], gps_records[:, 2],
                s=14, color="tab:green", alpha=0.6, label="GPS ölçümleri")
    ax1.plot(smooth[:, 0], smooth[:, 1],
             color="tab:purple", lw=1.8, label="iSAM2 smoother")
    ax1.set_xlabel("x (m)")
    ax1.set_ylabel("y (m)")
    ax1.set_title("Yörünge karşılaştırması")
    ax1.axis("equal")
    ax1.grid(True, alpha=0.3)
    ax1.legend(loc="best", fontsize=9)

    ax2.plot(t_arr, err_dr, color="tab:red", lw=1.0, label="Dead-reckoning")
    ax2.plot(t_arr, err_sm, color="tab:purple", lw=1.5, label="iSAM2 smoother")
    ax2.set_xlabel("zaman (s)")
    ax2.set_ylabel("konum hatası (m)")
    ax2.set_title("Anlık konum hatası")
    ax2.grid(True, alpha=0.3)
    ax2.legend(loc="best", fontsize=9)

    rmse_dr = float(np.sqrt((err_dr ** 2).mean()))
    rmse_sm = float(np.sqrt((err_sm ** 2).mean()))
    fig.suptitle(
        f"iSAM2 sentetik demo — GPS σ={cfg.gps_sigma_xy} m, "
        f"IMU σxy={cfg.imu_sigma_xy} m, dt={cfg.dt} s   |   "
        f"RMSE: DR={rmse_dr:.2f} m → smoother={rmse_sm:.2f} m"
    )
    fig.tight_layout()

    cfg.output_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(cfg.output_path, dpi=150, bbox_inches="tight")
    plt.close(fig)
    return rmse_dr, rmse_sm


def main() -> None:
    cfg = DemoConfig()
    rng = np.random.default_rng(cfg.seed)

    n_steps = int(cfg.duration / cfg.dt)
    controls = control_profile(n_steps, cfg.dt)
    truth = simulate_truth(controls, cfg.dt)
    imu_deltas, gps_records = simulate_sensors(truth, cfg, rng)

    pose0 = gtsam.Pose2(0.0, 0.0, 0.0)
    dr_traj = dead_reckon(pose0, imu_deltas)
    smooth_traj = run_smoother(pose0, imu_deltas, gps_records, cfg)

    rmse_dr, rmse_sm = plot_results(truth, dr_traj, gps_records, smooth_traj, cfg)

    print(f"[demo] {n_steps} adım, {gps_records.shape[0]} GPS örneği")
    print(f"[demo] dead-reckoning RMSE = {rmse_dr:.2f} m")
    print(f"[demo] iSAM2 smoother  RMSE = {rmse_sm:.2f} m")
    print(f"[demo] kaydedildi: {cfg.output_path.relative_to(_REPO_ROOT)}")


if __name__ == "__main__":
    main()
