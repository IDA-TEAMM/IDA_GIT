#!/usr/bin/env python3
"""
Girdap İDA — Ekran-2 panel üretici çalıştırıcı (T0 video montaj aracı).

Grafik CSV'sinden (telemetry_node, ~/girdap_logs/grafik/grafik_<UTC>.csv)
şartname md 3.3.1.1 Ekran-2 üç sinyalini çizer: hız+setpoint, heading+yön
setpoint, thruster kuvvet istekleri. ROS gerektirmez, offline koşar.

Kullanım:
    python scripts/run_ekran2.py                        # en yeni CSV → PNG
    python scripts/run_ekran2.py --csv /yol/grafik.csv  # belirli CSV
    python scripts/run_ekran2.py --mp4                  # zaman imleçli MP4
    python scripts/run_ekran2.py --mp4 --t0 12 --t1 190 # görev penceresi kırp

Varsayılan çıktı: ~/girdap_logs/viz/ekran2_<csv-adı>.png|.mp4
MP4 imleci gerçek süreyle akar — montajda dış kamera görüntüsüyle görev
başlangıcını hizala, gerisi senkron kalır (md 3.3.1.1 "senkron" şartı).
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

# Repo kökünü path'e ekle (script doğrudan çalıştırılır).
_REPO_ROOT = Path(__file__).resolve().parents[1]
if str(_REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(_REPO_ROOT))

from prototype.viz.ekran2 import (  # noqa: E402
    Ekran2Data,
    find_latest_graph_csv,
    load_graph_csv,
    save_mp4,
    save_png,
)

_DEFAULT_GRAPH_DIR = Path.home() / "girdap_logs" / "grafik"


def _trim(data: Ekran2Data, t0: float | None, t1: float | None) -> Ekran2Data:
    """Zaman penceresi kırp (görev öncesi/sonrası rölanti satırlarını at)."""
    lo = t0 if t0 is not None else float(data.t[0])
    hi = t1 if t1 is not None else float(data.t[-1])
    mask = (data.t >= lo) & (data.t <= hi)
    if not mask.any():
        raise SystemExit(f"[ekran2] pencere boş: t0={lo} t1={hi} veri dışında")
    return Ekran2Data(
        t=data.t[mask] - data.t[mask][0],
        hiz=data.hiz[mask],
        hiz_setpoint=data.hiz_setpoint[mask],
        heading_deg=data.heading_deg[mask],
        yon_setpoint_deg=data.yon_setpoint_deg[mask],
        thrust_sol=data.thrust_sol[mask],
        thrust_sag=data.thrust_sag[mask],
        kaynak=data.kaynak,
    )


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Girdap İDA Ekran-2 panel üretici (md 3.3.1.1)",
    )
    parser.add_argument(
        "--csv", default=None,
        help=f"grafik CSV yolu (varsayılan: {_DEFAULT_GRAPH_DIR} içindeki en yeni)",
    )
    parser.add_argument(
        "--mp4", action="store_true",
        help="zaman imleçli MP4 üret (varsayılan: statik PNG)",
    )
    parser.add_argument(
        "--out", default=None,
        help="çıktı yolu (varsayılan: ~/girdap_logs/viz/ekran2_<ad>.png|.mp4)",
    )
    parser.add_argument(
        "--fps", type=int, default=30, help="MP4 kare hızı (varsayılan 30)",
    )
    parser.add_argument(
        "--t0", type=float, default=None,
        help="pencere başı [s, CSV başından] — görev öncesi rölantiyi kırp",
    )
    parser.add_argument(
        "--t1", type=float, default=None, help="pencere sonu [s, CSV başından]",
    )
    args = parser.parse_args()

    csv_path = (
        Path(args.csv).expanduser() if args.csv
        else find_latest_graph_csv(_DEFAULT_GRAPH_DIR)
    )
    data = load_graph_csv(csv_path)
    print(f"[ekran2] {csv_path.name}: {len(data.t)} satır, "
          f"süre {data.t[-1] - data.t[0]:.1f} s")
    if args.t0 is not None or args.t1 is not None:
        data = _trim(data, args.t0, args.t1)
        print(f"[ekran2] pencere: {len(data.t)} satır, "
              f"süre {data.t[-1]:.1f} s")

    ext = "mp4" if args.mp4 else "png"
    out = (
        Path(args.out).expanduser() if args.out
        else Path.home() / "girdap_logs" / "viz" / f"ekran2_{csv_path.stem}.{ext}"
    )
    # --out dizin/uzantısız verilirse dosya adını İÇİNE üret; aksi hâlde
    # matplotlib sessizce "<yol>.png" yazar ve basılan yol yanıltıcı olur.
    if args.out and out.suffix == "":
        out = out / f"ekran2_{csv_path.stem}.{ext}"
    if args.mp4:
        print(f"[ekran2] MP4 render ({args.fps} fps) — uzun kayıtta birkaç "
              "dakika sürebilir...")
        path = save_mp4(data, out, fps=args.fps)
    else:
        path = save_png(data, out)
    print(f"[ekran2] kaydedildi: {path}")


if __name__ == "__main__":
    main()
