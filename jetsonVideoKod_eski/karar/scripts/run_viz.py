#!/usr/bin/env python3
"""
Girdap İDA — Offline 2D görselleştirici çalıştırıcı (Sprint 4.5).

Sentetik bir senaryoyu koşturup top-down animasyonu açar (veya GIF kaydeder).
ROS gerektirmez; prototype/ çekirdeklerini doğrudan kullanır.

Kullanım:
    python scripts/run_viz.py --scenario parkur2
    python scripts/run_viz.py --scenario fusion --save
    python scripts/run_viz.py --scenario fusion --save --out /tmp/f.gif

Senaryolar: parkur1 (nokta takip), parkur2 (engelli, 1→2), fusion (1→2→3 +
kamikaze tamamlanma + füzyon renkleri). --save çıktısı varsayılan:
    ~/girdap_logs/viz/scenario_<ad>.gif
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

# Repo kökünü path'e ekle (script doğrudan çalıştırılır).
_REPO_ROOT = Path(__file__).resolve().parents[1]
if str(_REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(_REPO_ROOT))

from prototype.viz.plotter import animate, save_gif  # noqa: E402
from prototype.viz.scenario import SCENARIOS, run_scenario  # noqa: E402


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Girdap İDA offline 2D görselleştirici",
    )
    parser.add_argument(
        "--scenario", choices=sorted(SCENARIOS), default="parkur2",
        help="çalıştırılacak senaryo (varsayılan: parkur2)",
    )
    parser.add_argument(
        "--save", action="store_true",
        help="animasyonu GIF olarak kaydet (pencere açmaz)",
    )
    parser.add_argument(
        "--out", default=None,
        help="GIF çıktı yolu (varsayılan: ~/girdap_logs/viz/scenario_<ad>.gif)",
    )
    parser.add_argument(
        "--fps", type=int, default=15, help="GIF kare hızı (varsayılan 15)",
    )
    args = parser.parse_args()

    scenario = SCENARIOS[args.scenario]()
    print(f"[viz] senaryo '{scenario.name}' koşturuluyor...")
    frames = run_scenario(scenario)
    print(f"[viz] {len(frames)} frame üretildi "
          f"(son parkur: {frames[-1].parkur_state}, {frames[-1].mission_phase})")

    if args.save:
        out = (
            Path(args.out) if args.out
            else Path.home() / "girdap_logs" / "viz" / f"scenario_{scenario.name}.gif"
        )
        path = save_gif(frames, scenario, out, fps=args.fps)
        print(f"[viz] GIF kaydedildi: {path}")
    else:
        import matplotlib.pyplot as plt

        _fig, _anim = animate(frames, scenario)
        print("[viz] pencere açıldı — kapatmak için figürü kapatın.")
        plt.show()


if __name__ == "__main__":
    main()
