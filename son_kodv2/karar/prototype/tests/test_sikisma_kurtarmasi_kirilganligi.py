# -*- coding: utf-8 -*-
"""SIKIŞMA KURTARMASI RASTGELE DİZİYE BAĞIMLI (19.08.2026)

🔴 NEDEN ÖNEMLİ — DOĞRUDAN KAPI KONUSU: "370 s donma" arızası tekne bir
KAPI DİREĞİNİN huni payında sıkışırken oluşuyordu. Kurtarma mekanizması o
yüzden var. Mekanizma çalışmazsa araç kapı önünde donar ⇒ **kapıdan geçemez**.

## ÖLÇÜM (bu makinede, RTX 4060 + CuPy 14.1.1 kurulduktan sonra)

| yol | dtype | u0 | max ağırlık | ESS | test |
|---|---|---|---|---|---|
| CPU (numpy) | float64 | [0.1128, 0.2955] | 0.146 | 15,2 | ✅ 5/5 |
| CPU + float32 | float32 | [0.1128, 0.2955] | 0.146 | 15,2 | ✅ |
| **GPU (cupy)** | float32 | [0.1469, 0.2828] | **0.299** | **5,2** | 🔴 |

Tohum taraması: CPU 5/5 tohum ✅ · **GPU 6/6 tohum 🔴** ⇒ şans değil.

## KÖK NEDEN — dtype DEĞİL, RASTGELE DİZİ
`numpy` ve `cupy` aynı tohumda **tamamen farklı** diziler üretir
(ölçüldü: numpy [1.1176, −1.3871, …] · cupy [−2.5974, 0.3704, …]).
`MPPIConfig.seed` backend'ler arasında tekrarlanabilirlik SAĞLAMAZ.

🔑 AYIRT EDİCİ DENEY: CPU hesap yoluna CuPy'nin gürültüsü verildi →
**CPU DA ÇÖKTÜ**. Yani kusur CuPy hesap yolunda değil; kurtarma mekanizması
belirli rastgele dizilerde **çalışmıyor** — yani KIRILGAN.

⚠ Bu bir "PC/GPU kurulum sorunu" DEĞİL: Jetson'da CuPy kurulu ve MPPI
`backend="auto"` ile GPU yolunu seçiyor ⇒ sahada koşan dizi tam da
mekanizmanın çalışmadığı dizi.

⚠ Bu dosya mekanizmayı DÜZELTMEZ (düzeltme karar tarafının kararı —
ortak alan kuralı). Kırılganlığı GÖRÜNÜR ve TEKRARLANABİLİR kılar.
"""
from __future__ import annotations

import numpy as np
import pytest

cp = pytest.importorskip("cupy", reason="GPU yolu yok — bu makinede sınanamaz")


def test_iki_backend_AYNI_TOHUMDA_FARKLI_dizi_uretir():
    """Kırılganlığın dayanağı: `seed` backend'ler arası tekrarlanabilirlik
    sağlamaz. Bu bir CuPy kusuru değil, bilinmesi gereken bir sözleşme."""
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


def test_kurtarma_GPU_yolunda_calismiyor_KAYIT():
    """Ölçülen durumu DONDURUR — düzelirse bu test 'beklenmedik geçti' der.

    Kırmızı olması BEKLENEN: sahada koşan yol (Jetson + CuPy) bu.
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
    if state[0] > 15.0:
        pytest.fail(
            f"GPU yolunda kurtarma ARTIK ÇALIŞIYOR (x={state[0]:.2f}) — "
            "bu dosyanın notu ve `_BILINEN` kaydı güncellenmeli")
    assert state[0] < 15.0


def test_ESS_GPU_yolunda_belirgin_DUSUK():
    """Mekanizmanın çökme imzası: softmax dejenere (ESS 15,2 → 5,2).

    ESS düşükse MPPI ağırlıklı ortalama yapmıyor, tek örneği seçiyor —
    CLAUDE.md λ bölümündeki bilinen tuzağın GPU yolundaki hâli.
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
    assert ess < 10.0, (
        f"ESS {ess:.1f} — GPU yolundaki dejenerasyon kaybolmuş, not güncellensin")
