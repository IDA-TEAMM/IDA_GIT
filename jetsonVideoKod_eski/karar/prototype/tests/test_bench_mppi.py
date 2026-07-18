"""
bench_mppi çekirdeği (run_bench) testleri — CUDA planı §5 D3 ölçüm aracı.
Çalıştır: pytest prototype/tests/test_bench_mppi.py -v

Benchmark Jetson D3 gününde elle koşar; burada yalnız ölçüm iskeletinin
sözleşmesi sabitlenir (adım sayısı, warmup dışlama, determinizm, geçersiz
backend). Süre DEĞERLERİ makineye bağlıdır — test edilmez.
"""

import numpy as np
import pytest

from scripts.bench_mppi import run_bench


def test_bench_olcum_adedi_ve_alanlar() -> None:
    """steps kadar ölçüm, warmup dışlanır; sonuç alanları tutarlı dolar."""
    r = run_bench(backend="numpy", K=32, T=8, steps=4, warmup=1, seed=0)
    assert r.backend_name == "numpy"
    assert r.dtype_name == "float64"
    assert len(r.times_ms) == 4                     # warmup ölçüme girmez
    assert all(t > 0.0 for t in r.times_ms)
    assert r.min_ms <= r.avg_ms <= r.max_ms
    assert r.n_ref >= 2                             # düz çizgi ref örneklendi
    assert r.final_state.shape == (6,)
    assert np.all(np.isfinite(r.final_state))
    assert r.drift_pct is not None                  # 4 ölçüm → yarılar var


def test_bench_ayni_seed_deterministik() -> None:
    """Aynı seed + numpy → kapalı döngü bit-birebir (süreler hariç)."""
    r1 = run_bench(backend="numpy", K=32, T=8, steps=3, warmup=0, seed=5)
    r2 = run_bench(backend="numpy", K=32, T=8, steps=3, warmup=0, seed=5)
    np.testing.assert_array_equal(r1.final_state, r2.final_state)


def test_bench_gecersiz_backend_value_error() -> None:
    """Backend sözleşmesi mppi._resolve_backend'den aynen geçer."""
    with pytest.raises(ValueError, match="backend"):
        run_bench(backend="tpu", K=8, T=4, steps=1, warmup=0)
