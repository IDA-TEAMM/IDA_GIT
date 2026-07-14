#!/usr/bin/env python3
"""
Girdap İDA — MPPI step benchmark'ı (docs/mppi_cuda_plani.md §5, D3 ölçüm aracı).

İki backend'i AYNI sahnede ölçer; kabul kriterleri plan §5: (a) cupy ort
< 50 ms (20 Hz), hedef < 20 ms (50 Hz); (d) uzun koşuda step süresi
sürüklenmiyor (--steps 600 ≈ 60 s @ 10 Hz; tegrastats ile birlikte oku).

Kullanım (Jetson'da ÖNCE: sudo nvpmodel -m 0 && sudo jetson_clocks):
    python3 scripts/bench_mppi.py --backend numpy
    python3 scripts/bench_mppi.py --backend cupy
    python3 scripts/bench_mppi.py --backend cupy --steps 600   # sürüklenme
    python3 scripts/bench_mppi.py --K 2000 --T 50              # tune taraması

Ölçüm notu: step() çıkışı sözleşme gereği host'a döner → perf_counter GPU
senkronunu doğal içerir, ayrıca cupy event gerekmez.
"""

from __future__ import annotations

import argparse
import sys
import time
from dataclasses import dataclass
from pathlib import Path
from typing import List, Optional

import numpy as np

# Repo kökünü path'e ekle (script doğrudan çalıştırılır).
_REPO_ROOT = Path(__file__).resolve().parents[1]
if str(_REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(_REPO_ROOT))

from prototype.dynamics.catamaran import CatamaranDynamics  # noqa: E402
from prototype.planning.mppi import (  # noqa: E402
    MPPIConfig,
    MPPIController,
    _build_scenario,
)


@dataclass
class BenchResult:
    """Tek backend koşusunun özeti — times_ms yalnız ölçülen adımlar."""

    backend_name: str            # fiilen çözülen backend ("auto" sonrası)
    dtype_name: str
    n_ref: int
    n_obstacle: int
    times_ms: List[float]
    final_state: np.ndarray

    @property
    def avg_ms(self) -> float:
        return float(np.mean(self.times_ms))

    @property
    def min_ms(self) -> float:
        return float(np.min(self.times_ms))

    @property
    def max_ms(self) -> float:
        return float(np.max(self.times_ms))

    @property
    def drift_pct(self) -> Optional[float]:
        """İlk yarı → son yarı ort. değişim (%) — plan §5(d) throttle gözlemi."""
        if len(self.times_ms) < 4:
            return None
        half = len(self.times_ms) // 2
        first = float(np.mean(self.times_ms[:half]))
        last = float(np.mean(self.times_ms[-half:]))
        return 100.0 * (last / first - 1.0)


def run_bench(
    backend: str = "numpy",
    K: int = 1000,
    T: int = 50,
    steps: int = 100,
    warmup: int = 3,
    seed: int = 0,
) -> BenchResult:
    """Demo sahnesinde kapalı döngü MPPI koş, yalnız step() sürelerini ölç.

    Referans BİLEREK düz çizgi (start→goal, RRT*'sız): deterministik ve
    ~114 ref noktasıyla baskın maliyet tensörünü (K, T+1, n_ref) gerçekçi
    boyutta çalıştırır. Warmup adımları döngüde koşar (cupy'de kernel
    derleme + bellek havuzu ısınması) ama ölçüme girmez.
    """
    bounds, obstacles, start, goal = _build_scenario()
    cfg = MPPIConfig(K=K, T=T, seed=seed, backend=backend)
    dyn = CatamaranDynamics()
    ctrl = MPPIController(dyn, bounds, obstacles, cfg)
    ctrl.set_reference([start, goal], spacing=0.5)

    state = np.zeros(6)
    state[0], state[1] = start
    state[2] = np.arctan2(goal[1] - start[1], goal[0] - start[0])

    times_ms: List[float] = []
    for k in range(warmup + steps):
        t0 = time.perf_counter()
        u = ctrl.step(state)
        elapsed_ms = 1e3 * (time.perf_counter() - t0)
        if k >= warmup:
            times_ms.append(elapsed_ms)
        state = dyn.step_rk4(state, u, cfg.dt)

    assert ctrl._ref_xy is not None  # set_reference az önce çağrıldı
    return BenchResult(
        backend_name="numpy" if ctrl.xp is np else "cupy",
        dtype_name=np.dtype(ctrl._dtype).name,
        n_ref=int(ctrl._ref_xy.shape[0]),
        n_obstacle=len(obstacles),
        times_ms=times_ms,
        final_state=state,
    )


def _print_report(r: BenchResult, istenen: str, warmup: int, K: int, T: int) -> None:
    print(
        f"[bench] backend={r.backend_name} dtype={r.dtype_name} "
        f"(istenen: {istenen}) | K={K} T={T} n_ref={r.n_ref} engel={r.n_obstacle}"
    )
    print(
        f"[bench] {len(r.times_ms)} adım (warmup {warmup} hariç): "
        f"ort {r.avg_ms:.1f} ms | min {r.min_ms:.1f} | maks {r.max_ms:.1f} "
        f"→ tavan ~{1000.0 / r.avg_ms:.1f} Hz"
    )
    if r.drift_pct is not None:
        print(f"[bench] sürüklenme (ilk→son yarı ort): {r.drift_pct:+.1f}%")
    esik20 = "GEÇTİ" if r.avg_ms < 50.0 else "KALDI"
    esik50 = "GEÇTİ" if r.avg_ms < 20.0 else "KALDI"
    print(f"[bench] plan §5(a): 20 Hz (<50 ms): {esik20} | 50 Hz hedef (<20 ms): {esik50}")


def main(argv: Optional[List[str]] = None) -> int:
    ap = argparse.ArgumentParser(
        description="MPPI step benchmark'ı (docs/mppi_cuda_plani.md §5)"
    )
    ap.add_argument("--backend", choices=("numpy", "cupy", "auto"), default="numpy")
    ap.add_argument("--K", type=int, default=1000, help="rollout sayısı")
    ap.add_argument("--T", type=int, default=50, help="horizon adımı")
    ap.add_argument("--steps", type=int, default=100, help="ölçülen adım sayısı")
    ap.add_argument("--warmup", type=int, default=3, help="ölçüme girmeyen ısınma adımı")
    ap.add_argument("--seed", type=int, default=0)
    args = ap.parse_args(argv)

    r = run_bench(
        backend=args.backend,
        K=args.K,
        T=args.T,
        steps=args.steps,
        warmup=args.warmup,
        seed=args.seed,
    )
    _print_report(r, args.backend, args.warmup, args.K, args.T)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
