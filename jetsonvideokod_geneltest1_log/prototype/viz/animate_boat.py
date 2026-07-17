"""
Girdap İDA — Katamaran Dinamik Model Görselleştirme
4 senaryo: düz ileri, sağa dönüş, yerinde dönüş, step input
KTR Algoritma Tasarımları bölümü için ekran görüntüsü al.
"""

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib.patches import FancyArrow
from prototype.dynamics.catamaran import CatamaranDynamics

DT = 0.05  # saniye
model = CatamaranDynamics()


def ciz_arac(ax: plt.Axes, x: float, y: float, psi: float,
             renk: str = "steelblue", alpha: float = 0.8) -> None:
    """Aracı dikdörtgen + heading oku olarak çiz."""
    uzunluk, genislik = 1.0, 0.5
    # Merkez → köşe dönüşümü
    kose = np.array([
        [-uzunluk / 2, -genislik / 2],
        [ uzunluk / 2, -genislik / 2],
        [ uzunluk / 2,  genislik / 2],
        [-uzunluk / 2,  genislik / 2],
    ])
    R = np.array([[np.cos(psi), -np.sin(psi)],
                  [np.sin(psi),  np.cos(psi)]])
    donmus = (R @ kose.T).T + np.array([x, y])
    polygon = plt.Polygon(donmus, color=renk, alpha=alpha)
    ax.add_patch(polygon)
    # Heading oku
    ax.annotate("", xy=(x + 0.7 * np.cos(psi), y + 0.7 * np.sin(psi)),
                xytext=(x, y),
                arrowprops=dict(arrowstyle="->", color="white", lw=1.5))


def sim_calistir(kontrollar: np.ndarray, n_adim: int) -> np.ndarray:
    state0 = np.zeros(6)
    return model.simulate(state0, np.tile(kontrollar, (n_adim, 1)), dt=DT)


def main() -> None:
    fig, axes = plt.subplots(2, 2, figsize=(14, 11))
    fig.suptitle("Girdap İDA — 3-DOF Katamaran Dinamik Model\nSenaryo Simülasyonları",
                 fontsize=14, fontweight="bold")
    fig.patch.set_facecolor("#1e1e2e")
    for ax in axes.flat:
        ax.set_facecolor("#2a2a3e")
        ax.tick_params(colors="white")
        ax.xaxis.label.set_color("white")
        ax.yaxis.label.set_color("white")
        ax.title.set_color("white")
        for spine in ax.spines.values():
            spine.set_edgecolor("#444466")

    # --- Senaryo 1: Düz ileri ---
    ax = axes[0, 0]
    hist = sim_calistir(np.array([15.0, 15.0]), 150)
    ax.plot(hist[:, 0], hist[:, 1], color="cyan", lw=2, label="Yörünge")
    for i in range(0, len(hist), 20):
        ciz_arac(ax, hist[i, 0], hist[i, 1], hist[i, 2])
    ax.set_title("Senaryo 1: Eşit Thrust — Düz İleri\n(Sol=15N, Sağ=15N)")
    ax.set_xlabel("x (m)"); ax.set_ylabel("y (m)")
    ax.set_aspect("equal"); ax.grid(True, alpha=0.3, color="white")
    ax.legend(facecolor="#2a2a3e", labelcolor="white")

    # --- Senaryo 2: Sağa dönüş ---
    ax = axes[0, 1]
    hist = sim_calistir(np.array([15.0, 7.0]), 200)
    ax.plot(hist[:, 0], hist[:, 1], color="orange", lw=2, label="Yörünge")
    for i in range(0, len(hist), 25):
        ciz_arac(ax, hist[i, 0], hist[i, 1], hist[i, 2], renk="darkorange")
    ax.set_title("Senaryo 2: Asimetrik Thrust — Sağa Dönüş\n(Sol=15N, Sağ=7N)")
    ax.set_xlabel("x (m)"); ax.set_ylabel("y (m)")
    ax.set_aspect("equal"); ax.grid(True, alpha=0.3, color="white")
    ax.legend(facecolor="#2a2a3e", labelcolor="white")

    # --- Senaryo 3: Yerinde dönüş ---
    ax = axes[1, 0]
    hist = sim_calistir(np.array([-12.0, 12.0]), 150)
    ax.plot(hist[:, 0], hist[:, 1], color="lime", lw=2, label="Yörünge")
    for i in range(0, len(hist), 20):
        ciz_arac(ax, hist[i, 0], hist[i, 1], hist[i, 2], renk="darkgreen")
    ax.set_title("Senaryo 3: Ters Thrust — Yerinde Dönüş\n(Sol=-12N, Sağ=+12N)")
    ax.set_xlabel("x (m)"); ax.set_ylabel("y (m)")
    ax.set_aspect("equal"); ax.grid(True, alpha=0.3, color="white")
    ax.legend(facecolor="#2a2a3e", labelcolor="white")

    # --- Senaryo 4: Step input + sönümleme ---
    ax = axes[1, 1]
    n_toplam = 300
    kontrollar = np.zeros((n_toplam, 2))
    kontrollar[50:200] = [20.0, 20.0]   # tam gas
    kontrollar[200:] = [0.0, 0.0]       # dur
    state = np.zeros(6)
    hizlar = []
    for ctrl in kontrollar:
        state = model.step_rk4(state, ctrl, DT)
        hizlar.append(state[3])          # u: ileri sürat
    zaman = np.arange(n_toplam) * DT
    ax.plot(zaman, hizlar, color="violet", lw=2)
    ax.axvspan(50 * DT, 200 * DT, alpha=0.2, color="yellow", label="Full thrust")
    ax.axvspan(200 * DT, n_toplam * DT, alpha=0.2, color="red", label="Motor off")
    ax.set_title("Senaryo 4: Step Input — Hız & Sönümleme")
    ax.set_xlabel("Zaman (s)"); ax.set_ylabel("İleri Sürat u (m/s)")
    ax.grid(True, alpha=0.3, color="white")
    ax.legend(facecolor="#2a2a3e", labelcolor="white")

    plt.tight_layout()

    # KTR için kaydet
    import os
    os.makedirs("docs/KTR", exist_ok=True)
    plt.savefig("docs/KTR/dinamik_model_senaryolar.png", dpi=150,
                bbox_inches="tight", facecolor=fig.get_facecolor())
    print("Görsel kaydedildi: docs/KTR/dinamik_model_senaryolar.png")
    plt.show()


if __name__ == "__main__":
    main()