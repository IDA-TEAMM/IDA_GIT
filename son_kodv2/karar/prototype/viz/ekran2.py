"""
Girdap İDA — Ekran-2 panel üretici (Otonomi Kabiliyeti videosu, T0).

Şartname md 3.3.1.1: videonun 2. bölmesi, İDA hareketleriyle SENKRON üç
grafik ister — (a) gerçek hız + hız setpoint, (b) gerçek heading/yaw + yaw
setpoint, (c) thrusterlardan kuvvet isteği. Girdi: telemetry_node'un yazdığı
grafik CSV'si (~/girdap_logs/grafik/grafik_<UTC>.csv, GRAPH_CSV_HEADER,
10 Hz). Çıktı: statik PNG (kontrol/rapor) ve zaman imleçli MP4 (montajda
dış kamera görüntüsünün yanına bindirilir; imleç = senkron kanıtı).

Offline montaj aracıdır — Jetson'da koşmaz, runtime yükü yok.
matplotlib TEMBEL import edilir (plotter.py deseni): ayrıştırma testleri
grafik bağımlılığı olmadan da anlamlı kalır.
"""

from __future__ import annotations

import csv
import math
import shutil
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import List, Optional, Union

import numpy as np

from prototype.telemetry.csv_logger import GRAPH_CSV_HEADER

# Panel stili — plotter.py/deniz_durumu_karsilastirma.py ile tutarlı.
_GRID_ALPHA = 0.15
_COLOR_GERCEK = "tab:blue"
_COLOR_SETPOINT = "tab:orange"
_COLOR_SOL = "tab:green"
_COLOR_SAG = "tab:red"
_CURSOR_COLOR = "black"


@dataclass
class Ekran2Data:
    """Grafik CSV'sinin panel-hazır hali. Eksik hücreler NaN (0 DEĞİL)."""

    t: np.ndarray                 # görev süresi [s], t[0] = 0
    hiz: np.ndarray               # m/s
    hiz_setpoint: np.ndarray      # m/s
    heading_deg: np.ndarray       # derece (CSV rad → çevrildi)
    yon_setpoint_deg: np.ndarray  # derece
    thrust_sol: np.ndarray        # N
    thrust_sag: np.ndarray        # N
    kaynak: str = ""              # dosya adı — panel başlığında gösterilir


def _to_float(cell: str) -> float:
    """CSV hücresi → float; boş hücre ("" = veri henüz yok) → NaN."""
    return float(cell) if cell else math.nan


def load_graph_csv(path: Union[str, Path]) -> Ekran2Data:
    """Grafik CSV'sini oku ve panel-hazır dizilere çevir.

    Header GRAPH_CSV_HEADER ile birebir doğrulanır — Dosya-2 telemetri
    CSV'si yanlışlıkla verilirse sessiz çöp grafik yerine net hata.
    """
    path = Path(path)
    with open(path, newline="", encoding="utf-8") as fp:
        reader = csv.reader(fp)
        header = next(reader, None)
        if header != GRAPH_CSV_HEADER:
            raise ValueError(
                f"{path.name}: header GRAPH_CSV_HEADER değil "
                f"(bulunan: {header}) — grafik CSV'si mi?"
            )
        rows = [row for row in reader if row]

    if not rows:
        raise ValueError(f"{path.name}: hiç veri satırı yok")

    stamps = [datetime.fromisoformat(row[0]) for row in rows]
    t0 = stamps[0]
    t = np.array([(s - t0).total_seconds() for s in stamps])
    cols = np.array(
        [[_to_float(cell) for cell in row[1:]] for row in rows]
    )  # (N, 6): hiz, hiz_sp, heading, yon_sp, thrust_sol, thrust_sag

    return Ekran2Data(
        t=t,
        hiz=cols[:, 0],
        hiz_setpoint=cols[:, 1],
        heading_deg=np.degrees(cols[:, 2]),
        yon_setpoint_deg=np.degrees(cols[:, 3]),
        thrust_sol=cols[:, 4],
        thrust_sag=cols[:, 5],
        kaynak=path.name,
    )


def break_wraps(deg: np.ndarray, threshold_deg: float = 180.0) -> np.ndarray:
    """±180° sarım sıçramasının vardığı noktayı NaN yap (kopya döner).

    Heading [-180, 180] aralığında sarılıdır; sıçrama noktası çizilirse
    panelde dikey çizgi artefaktı olur. NaN matplotlib'de çizgiyi keser.
    """
    out = deg.astype(float).copy()
    if len(out) < 2:
        return out
    jump = np.abs(np.diff(out)) > threshold_deg
    out[1:][jump] = math.nan
    return out


def find_latest_graph_csv(directory: Union[str, Path]) -> Path:
    """Dizindeki en yeni grafik CSV'si (grafik_*.csv, ada göre sıralı)."""
    directory = Path(directory).expanduser()
    candidates = sorted(directory.glob("grafik_*.csv"))
    if not candidates:
        raise FileNotFoundError(f"{directory} içinde grafik_*.csv yok")
    return candidates[-1]  # utc_timestamp() adları sıralanabilir


def make_figure(
    data: Ekran2Data,
    figsize: tuple = (8.0, 9.0),
    thrust_birim: str = "N",
):
    """3 panelli Ekran-2 figürü kur (md 3.3.1.1 sinyal sırasıyla).

    `thrust_birim`: telemetry setpoint_source ile AYNI olmalı — "girdap"
    modunda MPPI kuvvet isteği newton, "fc" modunda FC servo çıkışı yüzdedir
    (±%100). Yanlış birim etiketi grafiği yalancı yapar.
    """
    import matplotlib.pyplot as plt

    fig, (ax_hiz, ax_yon, ax_thrust) = plt.subplots(
        3, 1, sharex=True, figsize=figsize
    )

    ax_hiz.plot(data.t, data.hiz, color=_COLOR_GERCEK, label="hız")
    ax_hiz.plot(
        data.t, data.hiz_setpoint, color=_COLOR_SETPOINT,
        linestyle="--", label="hız setpoint",
    )
    ax_hiz.set_ylabel("hız (m/s)")

    ax_yon.plot(
        data.t, break_wraps(data.heading_deg),
        color=_COLOR_GERCEK, label="heading",
    )
    ax_yon.plot(
        data.t, break_wraps(data.yon_setpoint_deg),
        color=_COLOR_SETPOINT, linestyle="--", label="yön setpoint",
    )
    ax_yon.set_ylabel("heading (°)")

    ax_thrust.plot(
        data.t, data.thrust_sol, color=_COLOR_SOL, label="thrust sol"
    )
    ax_thrust.plot(
        data.t, data.thrust_sag, color=_COLOR_SAG, label="thrust sağ"
    )
    ax_thrust.set_ylabel(f"kuvvet isteği ({thrust_birim})")
    ax_thrust.set_xlabel("görev süresi (s)")

    for ax in (ax_hiz, ax_yon, ax_thrust):
        ax.grid(alpha=_GRID_ALPHA)
        ax.legend(loc="upper right", fontsize=8)

    fig.suptitle(f"Ekran-2 — {data.kaynak}" if data.kaynak else "Ekran-2")
    fig.align_ylabels()
    return fig


def save_png(
    data: Ekran2Data, path: Union[str, Path], dpi: int = 150,
    thrust_birim: str = "N",
) -> Path:
    """Statik tam-zaman-ekseni PNG (kontrol bakışı / KTR görseli)."""
    import matplotlib

    matplotlib.use("Agg", force=True)
    import matplotlib.pyplot as plt

    path = Path(path).expanduser()
    path.parent.mkdir(parents=True, exist_ok=True)
    fig = make_figure(data, thrust_birim=thrust_birim)
    fig.savefig(path, dpi=dpi, bbox_inches="tight")
    plt.close(fig)
    return path


def save_mp4(
    data: Ekran2Data,
    path: Union[str, Path],
    fps: int = 30,
    dpi: int = 100,
    thrust_birim: str = "N",
) -> Path:
    """Zaman imleçli MP4 — montajda dış kamera yanına senkron bindirme.

    İmleç t[0]→t[-1] gerçek süreyle akar: videonun i. saniyesi = görevin
    i. saniyesi. Montajda görev başlangıcı hizalanır, gerisi senkron kalır.
    3 dk × 30 fps ≈ 5400 kare; offline render birkaç dakika sürebilir.
    """
    if shutil.which("ffmpeg") is None:
        raise RuntimeError(
            "ffmpeg bulunamadı — MP4 için gerekli (apt install ffmpeg); "
            "alternatif: save_png ile statik panel"
        )

    import matplotlib

    matplotlib.use("Agg", force=True)
    import matplotlib.pyplot as plt
    from matplotlib.animation import FFMpegWriter, FuncAnimation

    path = Path(path).expanduser()
    path.parent.mkdir(parents=True, exist_ok=True)

    fig = make_figure(data, thrust_birim=thrust_birim)
    cursors = [
        ax.axvline(data.t[0], color=_CURSOR_COLOR, linewidth=1.0, alpha=0.7)
        for ax in fig.axes
    ]

    duration = float(data.t[-1] - data.t[0])
    n_frames = max(2, int(round(duration * fps)) + 1)
    frame_times = np.linspace(data.t[0], data.t[-1], n_frames)

    def _update(i: int) -> list:
        for cursor in cursors:
            cursor.set_xdata([frame_times[i], frame_times[i]])
        return cursors

    anim = FuncAnimation(
        fig, _update, frames=n_frames, interval=1000.0 / fps, blit=False
    )
    anim.save(str(path), writer=FFMpegWriter(fps=fps), dpi=dpi)
    plt.close(fig)
    return path
