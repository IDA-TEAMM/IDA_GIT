# -*- coding: utf-8 -*-
"""SIKIŞMA KURTARMASI RASTGELE DİZİYE BAĞIMLIYDI — DÜZELTİLDİ (19.08.2026)

🔴 NEDEN ÖNEMLİ — DOĞRUDAN KAPI KONUSU: "370 s donma" arızası tekne bir
KAPI DİREĞİNİN huni payında sıkışırken oluşuyordu. Kurtarma mekanizması o
yüzden var. Mekanizma çalışmazsa araç kapı önünde donar ⇒ **kapıdan geçemez**.

## BULGU (takım, RTX 4060 + CuPy 14.1.1, 19.08 gece)

| yol | dtype | u0 | max ağırlık | ESS | test |
|---|---|---|---|---|---|
| CPU (numpy) | float64 | [0.1128, 0.2955] | 0.146 | 15,2 | ✅ 5/5 |
| CPU + float32 | float32 | [0.1128, 0.2955] | 0.146 | 15,2 | ✅ |
| **GPU (cupy), DÜZELTME ÖNCESİ** | float32 | [0.1469, 0.2828] | **0.299** | **5,2** | 🔴 6/6 |

Kök neden `numpy`/`cupy`'nin aynı tohumda TAMAMEN FARKLI dizi üretmesiydi
(`MPPIController.__init__` `self._rng = self.xp.random.default_rng(seed)`
diyordu — backend'e göre numpy YA DA cupy RNG'si). Jetson `backend="auto"`
ile GPU'yu seçtiği için sahada koşan dizi tam da mekanizmanın çalışmadığı
dizidi.

## DÜZELTME (`prototype/planning/mppi.py`, commit b3caa601)
`self._rng` artık backend'den BAĞIMSIZ, HER ZAMAN `np.random.default_rng
(seed)`. `_sample_noise()` zaten sonucu `self.xp.asarray(...)` ile aktif
backend'e taşıyordu — yani CPU ve GPU artık AYNI tohumda BİREBİR AYNI
gürültü dizisini kullanıyor (yalnız dtype farklı: numpy float64, cupy
float32 — modülün kendi tasarım kararı, dokunulmadı).

## JETSON'DA GERÇEK GPU İLE DOĞRULANDI (19.08.2026, ssh 192.168.117.60)
    ESS: 5,2 → **15,2** (CPU ile birebir aynı)
    x (kaçış): donuk → **21,19 m** (CPU'daki ~21,4 m ile aynı bantta)
Aşağıdaki iki test artık TERSİNE döndü: "hâlâ bozuk" değil "düzeldi,
BÖYLE KALSIN" kilidi. Kırmızı çıkmaları yeniden GERÇEK bir regresyondur.
"""
from __future__ import annotations

import numpy as np
import pytest

cp = pytest.importorskip("cupy", reason="GPU yolu yok — bu makinede sınanamaz")


def test_iki_backend_AYNI_TOHUMDA_FARKLI_dizi_uretir():
    """Kırılganlığın DAYANAĞI — hâlâ doğru, düzeltme bunu SAKLAMADI, bu
    gerçeğin etrafından DOLAŞTI (`_rng` artık hiç cupy RNG'si çağırmıyor).
    Bu bir CuPy kusuru değil, bilinmesi gereken bir sözleşme."""
    a = np.random.default_rng(0).standard_normal(8, dtype=np.float32)
    b = cp.asnumpy(cp.random.default_rng(0).standard_normal(8, dtype=cp.float32))
    assert not np.allclose(a, b), (
        "backend'ler aynı diziyi üretiyor — bu testin dayanağı değişmiş")
    # dağılım AYNI olmalı; farklı olan yalnız DİZİ
    buyuk_a = np.random.default_rng(1).standard_normal(200_000, dtype=np.float32)
    buyuk_b = cp.asnumpy(
        cp.random.default_rng(1).standard_normal(200_000, dtype=cp.float32))
    assert abs(buyuk_a.std() - buyuk_b.std()) < 0.02
    assert abs(buyuk_a.mean() - buyuk_b.mean()) < 0.02


def test_kurtarma_GPU_yolunda_ARTIK_CALISIYOR() -> None:
    """DÜZELTME KİLİDİ — Jetson'da (gerçek CuPy) 19.08'de doğrulandı.

    Kırmızı çıkarsa REGRESYON demektir: `MPPIController._rng` yeniden
    backend'e özel hâle gelmiş olabilir — `mppi.py`daki 19.08 notuna bak.
    """
    from prototype.planning.rrt_star import Bounds
    from prototype.tests.test_sikisma_kurtarmasi import _tek_direk_sahnesi

    b = Bounds(-5.0, 60.0, -20.0, 20.0)
    pipe, dyn, state = _tek_direk_sahnesi(b, stuck_recovery_enabled=True)
    if "cupy" not in pipe._mppi.backend_adi:
        pytest.skip("bu koşumda MPPI numpy yolunda — GPU yolu sınanamıyor")
    dt = 0.1
    for _ in range(900):
        pipe.set_state(state)
        u = pipe.compute_control()
        if u is None or not np.all(np.isfinite(u)):
            break
        for _ in range(2):
            state = dyn.step_rk4(state, u, dt / 2)
    assert state[0] > 15.0, (
        f"GPU yolunda kurtarma YENİDEN ÇALIŞMIYOR (x={state[0]:.2f}) — "
        "RNG düzeltmesi (mppi.py, 19.08) bozulmuş olabilir, regresyon!")


def test_ESS_GPU_yolunda_ARTIK_CPU_ILE_AYNI() -> None:
    """Mekanizmanın DÜZELDİ imzası: ESS artık CPU ile aynı bantta (~15).

    ESS düşükse MPPI ağırlıklı ortalama yapmıyor, tek örneği seçiyor —
    CLAUDE.md λ bölümündeki bilinen tuzağın GPU yolundaki hâliydi.
    """
    from prototype.planning.rrt_star import Bounds
    from prototype.tests.test_sikisma_kurtarmasi import _tek_direk_sahnesi

    b = Bounds(-5.0, 60.0, -20.0, 20.0)
    pipe, _, state = _tek_direk_sahnesi(b, stuck_recovery_enabled=True)
    if "cupy" not in pipe._mppi.backend_adi:
        pytest.skip("numpy yolu — kıyas anlamsız")
    pipe.set_state(state)
    pipe.compute_control()
    w = np.asarray(pipe._mppi._as_numpy(pipe._mppi._last_weights))
    ess = 1.0 / float(np.sum(w ** 2))
    assert ess > 10.0, (
        f"ESS {ess:.1f} — GPU yolunda YENİDEN dejenere, RNG düzeltmesi "
        "(mppi.py, 19.08) bozulmuş olabilir, regresyon!")
