"""
Girdap İDA — Offline 2D görselleştirici: çizim katmanı (Sprint 4.5).

FrameState anlık görüntülerini top-down (dünya ENU) tek bir matplotlib figürüne
çizer ve FuncAnimation ile oynatır. matplotlib TEMBEL import edilir (modül
seviyesinde değil) — böylece scenario.py'yi çeken runtime/test yolları
matplotlib'i zorunlu yüklemez (NumPy ABI tuzağı + başsız ortam).

Kullanım:
    from prototype.viz.plotter import animate, save_gif, draw_frame
    frames = run_scenario(scenario)
    animate(frames, scenario)            # pencere aç (interaktif backend)
    save_gif(frames, scenario, path)     # GIF kaydet (Pillow, Agg)
"""

from __future__ import annotations

from pathlib import Path
from typing import TYPE_CHECKING, List

import numpy as np

from prototype.viz.scenario import FrameState, VizScenario

if TYPE_CHECKING:                                   # yalnız tip denetimi
    from matplotlib.axes import Axes

# Füzyon sınıfı → renk (CLAUDE.md perception sınıf sözleşmesi).
FUSION_COLORS = {
    0: "#ff8c00",     # parkur_kenari — turuncu (RAL 2003)
    1: "#ffd11a",     # engel — sarı (RAL 1026)
    2: "#e02020",     # hedef (Parkur-3) — kırmızı
    99: "#888888",    # unknown (eşleşmeyen LiDAR) — gri
}
_CLASS_LABEL = {0: "kenar", 1: "engel", 2: "hedef", 99: "?"}


def _color_for(class_id: int) -> str:
    return FUSION_COLORS.get(class_id, FUSION_COLORS[99])


def draw_frame(ax: "Axes", state: FrameState, scenario: VizScenario) -> None:
    """Tek frame'i verilen eksene çiz (önce temizler). Başsız-güvenli."""
    from matplotlib.patches import Circle

    ax.clear()
    b = scenario.bounds
    ax.set_xlim(b.x_min, b.x_max)
    ax.set_ylim(b.y_min, b.y_max)
    ax.set_aspect("equal")
    ax.set_xlabel("Doğu (m)")
    ax.set_ylabel("Kuzey (m)")
    ax.grid(True, alpha=0.15)

    # --- yerel maliyet haritası (Dosya-3) arka plan ---
    if state.cost_grid is not None:
        g = state.cost_grid
        grid = np.asarray(g.data, dtype=float).reshape(g.height, g.width)
        masked = np.ma.masked_where(grid < 0, grid)     # -1 bilinmiyor → şeffaf
        half_w = g.width * g.resolution / 2.0
        half_h = g.height * g.resolution / 2.0
        ax.imshow(
            masked, origin="lower", cmap="Reds", vmin=0, vmax=100, alpha=0.35,
            extent=[
                state.boat_x - half_w, state.boat_x + half_w,
                state.boat_y - half_h, state.boat_y + half_h,
            ],
            zorder=0,
        )

    # --- tekne izi (soluk) ---
    if len(state.trail) >= 2:
        tr = np.asarray(state.trail)
        ax.plot(tr[:, 0], tr[:, 1], color="#3070c0", alpha=0.4, lw=1.2, zorder=1)

    # --- MPPI öngörü yörüngesi / düz çizgi referans ---
    if state.mppi_traj is not None and len(state.mppi_traj) >= 2:
        mt = np.asarray(state.mppi_traj)
        ax.plot(mt[:, 0], mt[:, 1], color="#00b0b0", ls="--", lw=1.6,
                alpha=0.9, zorder=2, label="MPPI öngörü")

    # --- waypoint'ler (yıldız; aktif olan vurgulu) ---
    for i, (wx, wy) in enumerate(state.waypoints):
        if i == state.active_wp_index:
            ax.scatter(wx, wy, marker="*", s=340, c="#1a1a1a",
                       edgecolors="#f5c518", linewidths=2.0, zorder=4)
        else:
            ax.scatter(wx, wy, marker="*", s=180, c="#bbbbbb",
                       edgecolors="#555555", linewidths=1.0, zorder=3)

    # --- engeller (füzyon renkli daireler + sınıf etiketi) ---
    for o in state.obstacles:
        col = _color_for(o.class_id)
        ax.add_patch(
            Circle((o.x, o.y), max(o.radius, 0.25), facecolor=col,
                   edgecolor="#222222", lw=0.8, alpha=0.85, zorder=5)
        )
        ax.text(o.x, o.y + max(o.radius, 0.25) + 0.4, _CLASS_LABEL.get(o.class_id, "?"),
                fontsize=7, ha="center", va="bottom", color="#333333", zorder=6)

    # --- tekne (yön oklu) ---
    ax.scatter(state.boat_x, state.boat_y, s=90, c="#103080",
               edgecolors="white", linewidths=1.2, zorder=7)
    arrow_len = 2.0
    ax.arrow(
        state.boat_x, state.boat_y,
        arrow_len * np.cos(state.boat_psi), arrow_len * np.sin(state.boat_psi),
        head_width=0.7, head_length=0.7, fc="#103080", ec="#103080", zorder=7,
    )

    # --- metin overlay: parkur + FSM + aktif WP ---
    ax.text(
        0.015, 0.975,
        f"senaryo: {scenario.name}\n"
        f"parkur: {state.parkur_state}\n"
        f"durum: {state.mission_phase}\n"
        f"aktif WP: {state.active_wp_index}\n"
        f"t = {state.t:.1f} s",
        transform=ax.transAxes, va="top", ha="left", fontsize=9,
        family="monospace",
        bbox=dict(boxstyle="round,pad=0.4", fc="white", ec="#999999", alpha=0.85),
        zorder=8,
    )
    ax.set_title(f"Girdap İDA — Offline Görselleştirici ({scenario.name})",
                 fontsize=11)


def make_figure():                                  # noqa: ANN201
    """Standart figür + eksen üret (tembel matplotlib)."""
    import matplotlib.pyplot as plt

    fig, ax = plt.subplots(figsize=(10.0, 6.0))
    return fig, ax


def animate(frames: List[FrameState], scenario: VizScenario, interval_ms: int = 60):  # noqa: ANN201
    """FrameState listesini FuncAnimation ile oynat. Anim nesnesini döndürür.

    Çağıran `plt.show()` yapmalı (interaktif backend). Anim referansı GC'ye
    yem olmasın diye döndürülür.
    """
    from matplotlib.animation import FuncAnimation

    fig, ax = make_figure()

    def _update(i: int):
        draw_frame(ax, frames[i], scenario)
        return []

    anim = FuncAnimation(
        fig, _update, frames=len(frames), interval=interval_ms,
        blit=False, repeat=False,
    )
    return fig, anim


def save_gif(
    frames: List[FrameState],
    scenario: VizScenario,
    path: Path,
    fps: int = 15,
) -> Path:
    """Animasyonu GIF olarak kaydet (Pillow writer, Agg backend — başsız).

    Dokümantasyon/KTR için. Çıktı dizini gerekiyorsa oluşturulur.
    """
    import matplotlib
    matplotlib.use("Agg", force=True)               # başsız kayıt
    import matplotlib.pyplot as plt
    from matplotlib.animation import FuncAnimation, PillowWriter

    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)

    fig, ax = make_figure()

    def _update(i: int):
        draw_frame(ax, frames[i], scenario)
        return []

    anim = FuncAnimation(fig, _update, frames=len(frames), blit=False)
    anim.save(str(path), writer=PillowWriter(fps=fps))
    plt.close(fig)
    return path
