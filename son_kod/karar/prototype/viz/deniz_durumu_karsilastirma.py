"""
Girdap İDA — Deniz Durumu Karşılaştırma Görseli (KTR §4.5 ek)

İki MPPI kapalı döngü senaryosu yan yana koşturulur:
  1) Sakin su      : wave.enabled=False (referans)
  2) Deniz Durumu-2: Beaufort 3, Fx_amp=8 N, periyot=4 s (Fx_freq=0.25 Hz),
                     Mz_amp=1.5 N·m aynı periyotta

Aynı RRT* referansı, aynı engel düzeni, aynı MPPI hiperparametreleri.
Tek değişken: CatamaranDynamics içine enjekte edilen WaveDisturbance.

Çıktı: 2x2 panel
  Üst sıra : (sol) sakin trajektori + ref, (sağ) DD-2 trajektori + ref
  Alt sıra : (sol) goal hata zaman serisi, (sağ) toplam thruster efor

Çalıştırma:
    python -m prototype.viz.deniz_durumu_karsilastirma
"""

from __future__ import annotations

import math
from dataclasses import replace
from pathlib import Path
from typing import List, Tuple

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Circle

from prototype.dynamics.catamaran import (
    CatamaranDynamics,
    CatamaranParams,
    WaveDisturbance,
)
from prototype.planning.mppi import MPPIConfig, MPPIController
from prototype.planning.rrt_star import (
    Bounds,
    CircleObstacle,
    RRTStar,
    RRTStarConfig,
)

_REPO_ROOT = Path(__file__).resolve().parents[2]


# --------------------------------------------------------------------------- #
# Sahne (mppi.py demosuyla birebir aynı — karşılaştırma adil olsun)
# --------------------------------------------------------------------------- #


def _build_scenario() -> Tuple[
    Bounds, List[CircleObstacle], Tuple[float, float], Tuple[float, float]
]:
    bounds = Bounds(0.0, 50.0, 0.0, 50.0)
    obstacles = [
        CircleObstacle(15.0, 20.0, 3.0),
        CircleObstacle(25.0, 25.0, 4.0),
        CircleObstacle(35.0, 15.0, 3.0),
        CircleObstacle(20.0, 35.0, 3.0),
        CircleObstacle(35.0, 35.0, 3.5),
        CircleObstacle(10.0, 40.0, 2.5),
    ]
    return bounds, obstacles, (5.0, 5.0), (45.0, 45.0)


# --------------------------------------------------------------------------- #
# Tek senaryo koşumu
# --------------------------------------------------------------------------- #


def _run_scenario(
    label: str,
    wave: WaveDisturbance,
    ref_path: List[Tuple[float, float]],
    bounds: Bounds,
    obstacles: List[CircleObstacle],
    start: Tuple[float, float],
    goal: Tuple[float, float],
    cfg: MPPIConfig,
    max_sim_s: float = 40.0,
    goal_tol: float = 1.5,
) -> dict:
    """MPPI kapalı döngü koşumu. Wave tek değişken; diğer her şey sabit."""
    base = CatamaranParams.from_yaml()
    params = replace(base, wave=wave)
    dyn = CatamaranDynamics(params)

    ctrl = MPPIController(dyn, bounds, obstacles, cfg)
    ctrl.set_reference(ref_path, spacing=0.5)

    state = np.zeros(6)
    state[0], state[1] = start
    state[2] = math.atan2(goal[1] - start[1], goal[0] - start[0])

    n_max = int(max_sim_s / cfg.dt)
    executed = np.zeros((n_max + 1, 6))
    executed[0] = state
    controls = np.zeros((n_max, 2))
    t_sim = 0.0
    n_done = 0

    for k in range(n_max):
        u = ctrl.step(state)
        controls[k] = u
        # Wave zaman-bağımlı; gerçek aracı simüle ederken t kayar
        state = dyn.step_rk4(state, u, cfg.dt, t=t_sim)
        executed[k + 1] = state
        t_sim += cfg.dt
        n_done = k + 1

        if math.hypot(state[0] - goal[0], state[1] - goal[1]) < goal_tol:
            break

    executed = executed[: n_done + 1]
    controls = controls[: n_done]
    times = np.arange(n_done + 1) * cfg.dt
    goal_err = np.hypot(executed[:, 0] - goal[0], executed[:, 1] - goal[1])
    effort = np.abs(controls).sum(axis=1)            # |T_l| + |T_r| (N)
    final_err = float(goal_err[-1])

    print(
        f"[{label}] n={n_done} adım ({n_done * cfg.dt:.1f} s), "
        f"goal hata = {final_err:.2f} m, "
        f"ort. efor = {effort.mean():.2f} N"
    )

    return {
        "label": label,
        "executed": executed,
        "controls": controls,
        "times": times,
        "goal_err": goal_err,
        "effort": effort,
        "final_err": final_err,
    }


# --------------------------------------------------------------------------- #
# Görsel
# --------------------------------------------------------------------------- #


def _draw_traj_panel(
    ax: plt.Axes,
    title: str,
    bounds: Bounds,
    obstacles: List[CircleObstacle],
    ref_path: List[Tuple[float, float]],
    executed: np.ndarray,
    color_traj: str,
    final_err: float,
    cfg: MPPIConfig,
) -> None:
    for o in obstacles:
        ax.add_patch(Circle((o.cx, o.cy), o.r,
                            color="tab:red", alpha=0.45, zorder=2))
        ax.add_patch(Circle((o.cx, o.cy), o.r + cfg.obstacle_margin,
                            color="tab:red", fill=False, ls=":",
                            lw=0.8, alpha=0.5, zorder=2))

    ref = np.asarray(ref_path)
    ax.plot(ref[:, 0], ref[:, 1], color="tab:orange", lw=1.4, ls="--",
            label="RRT* referansı", zorder=3)
    ax.plot(executed[:, 0], executed[:, 1],
            color=color_traj, lw=2.2,
            label=f"MPPI iz (hata={final_err:.2f} m)", zorder=4)

    ax.scatter(ref[0, 0], ref[0, 1], c="tab:green", marker="X",
               s=130, zorder=5, label="Start")
    ax.scatter(ref[-1, 0], ref[-1, 1], c="black", marker="*",
               s=180, zorder=5, label="Goal")

    ax.set_xlim(bounds.x_min, bounds.x_max)
    ax.set_ylim(bounds.y_min, bounds.y_max)
    ax.set_aspect("equal")
    ax.set_xlabel("x (m)")
    ax.set_ylabel("y (m)")
    ax.set_title(title)
    ax.grid(True, alpha=0.3)
    ax.legend(loc="lower right", fontsize=8)


def _draw(
    bounds: Bounds,
    obstacles: List[CircleObstacle],
    ref_path: List[Tuple[float, float]],
    calm: dict,
    wavy: dict,
    cfg: MPPIConfig,
    wave: WaveDisturbance,
    out_path: Path,
) -> None:
    fig, axes = plt.subplots(2, 2, figsize=(13.5, 11.5))
    (ax_calm, ax_wavy), (ax_err, ax_eff) = axes

    _draw_traj_panel(
        ax_calm, "1) Sakin su (wave.enabled=False)",
        bounds, obstacles, ref_path,
        calm["executed"], "tab:purple", calm["final_err"], cfg,
    )
    _draw_traj_panel(
        ax_wavy,
        f"2) Deniz Durumu-2 (Beaufort 3) — "
        f"Fx_amp={wave.Fx_amp:.0f} N, T={1.0 / wave.Fx_freq:.1f} s",
        bounds, obstacles, ref_path,
        wavy["executed"], "tab:cyan", wavy["final_err"], cfg,
    )

    # Goal hata zaman serisi
    ax_err.plot(calm["times"], calm["goal_err"],
                color="tab:purple", lw=1.4, label="Sakin")
    ax_err.plot(wavy["times"], wavy["goal_err"],
                color="tab:cyan", lw=1.4, label="DD-2")
    ax_err.axhline(1.5, color="gray", ls=":", lw=0.8,
                   label="goal_tol = 1.5 m")
    ax_err.set_xlabel("zaman (s)")
    ax_err.set_ylabel("‖p − goal‖ (m)")
    ax_err.set_title("Goal hatası — zaman serisi")
    ax_err.grid(True, alpha=0.3)
    ax_err.legend(loc="upper right", fontsize=9)
    ax_err.set_yscale("log")

    # Kontrol efor (|T_l|+|T_r|)
    ax_eff.plot(calm["times"][:-1], calm["effort"],
                color="tab:purple", lw=1.0, alpha=0.85, label="Sakin")
    ax_eff.plot(wavy["times"][:-1], wavy["effort"],
                color="tab:cyan", lw=1.0, alpha=0.85, label="DD-2")
    ax_eff.set_xlabel("zaman (s)")
    ax_eff.set_ylabel("|T_l| + |T_r| (N)")
    ax_eff.set_title(
        f"Kontrol eforu — ort: sakin {calm['effort'].mean():.1f} N, "
        f"DD-2 {wavy['effort'].mean():.1f} N"
    )
    ax_eff.grid(True, alpha=0.3)
    ax_eff.legend(loc="upper right", fontsize=9)

    fig.suptitle(
        "MPPI Dalga Dayanıklılığı — Sakin Su vs Deniz Durumu-2",
        fontsize=13,
    )
    fig.tight_layout()

    out_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out_path, dpi=150, bbox_inches="tight")
    plt.close(fig)


# --------------------------------------------------------------------------- #
# Ana akış
# --------------------------------------------------------------------------- #


def main() -> None:
    bounds, obstacles, start, goal = _build_scenario()

    # 1) Tek RRT* yörüngesi her iki senaryo için ortak referans
    rrt = RRTStar(
        bounds, obstacles,
        RRTStarConfig(use_informed=True, seed=0, max_iter=1500),
    )
    ref_path = rrt.plan(start, goal)
    if ref_path is None:
        raise RuntimeError("RRT* çözüm bulamadı")
    print(
        f"[ref] RRT* yol: {len(ref_path)} waypoint, "
        f"cost = {rrt.best_cost:.2f} m"
    )

    # 2) MPPI hiperparametreleri (mppi.py demosuyla aynı)
    cfg = MPPIConfig(K=1000, T=50, dt=0.05, lambda_=1.0, sigma_u=5.0, seed=0)

    # 3) Senaryo A: sakin su
    wave_calm = WaveDisturbance(enabled=False)
    calm = _run_scenario(
        "sakin", wave_calm, ref_path, bounds, obstacles, start, goal, cfg,
    )

    # 4) Senaryo B: Deniz Durumu-2 / Beaufort 3
    # Periyot 4 s → Fx_freq = 0.25 Hz. Mz aynı periyotta küçük yaw bozucu.
    wave_dd2 = WaveDisturbance(
        enabled=True,
        Fx_amp=8.0, Fx_freq=0.25,
        Mz_amp=1.5, Mz_freq=0.25,
        phase=0.0,
    )
    wavy = _run_scenario(
        "DD-2", wave_dd2, ref_path, bounds, obstacles, start, goal, cfg,
    )

    # 5) Görsel
    out_path = _REPO_ROOT / "docs" / "KTR" / "deniz_durumu_karsilastirma.png"
    _draw(bounds, obstacles, ref_path, calm, wavy, cfg, wave_dd2, out_path)
    print(f"[viz] kaydedildi: {out_path.relative_to(_REPO_ROOT)}")


if __name__ == "__main__":
    main()
