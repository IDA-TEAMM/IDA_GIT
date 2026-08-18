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


def test_MPPI_gurultusu_backendden_BAGIMSIZ():
    """✅ 19.08 DÜZELTİLDİ (`b3caa601`): `_rng` artık HER ZAMAN numpy.

    Kütüphanelerin kendi PRNG'leri hâlâ farklı dizi üretir (aşağıda
    gösteriliyor) — düzeltme, MPPI'nin artık ONLARA BAĞLI OLMAMASI.
    Ölçüldü: u0 CPU [0.11280112, 0.29550353] ↔ GPU [0.11281674, 0.2954855]
    ve ESS 15,2 ↔ 15,2 (önce 15,2 ↔ 5,2 idi).
    """
    import prototype.planning.mppi as M
    from prototype.dynamics.catamaran import CatamaranDynamics
    from prototype.planning.rrt_star import Bounds

    # kütüphane PRNG'leri hâlâ farklı — düzeltmenin gerekçesi bu
    a = np.random.default_rng(0).standard_normal(8, dtype=np.float32)
    b = cp.asnumpy(cp.random.default_rng(0).standard_normal(8, dtype=cp.float32))
    assert not np.allclose(a, b)

    # ama MPPI artık ikisinde de AYNI diziyi kullanıyor
    ciktilar = []
    for backend in ("numpy", "cupy"):
        c = M.MPPIController(
            CatamaranDynamics(), Bounds(-50, 50, -50, 50), [],
            M.MPPIConfig(K=64, T=10, seed=3, backend=backend))
        c.set_reference([(0.0, 0.0), (10.0, 5.0)])
        ciktilar.append(np.asarray(c.step(np.zeros(6))))
    assert np.allclose(ciktilar[0], ciktilar[1], atol=1e-4), (
        f"CPU {ciktilar[0]} ↔ GPU {ciktilar[1]} — gürültü uyumsuzluğu geri gelmiş")
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


def test_kurtarma_GPU_yolunda_COK_GEC_tetikleniyor():
    """🔑 RNG düzeltmesinden SONRA kalan fark — asıl teşhis burada.

    ESS artık iki yolda da 15,2 (dejenerasyon KAPANDI) ama kurtarma hâlâ
    çalışmıyor. Ölçülen sebep TETİKLEME ZAMANI:

        CPU : ilk tetik  101. adım · 2 kez · x_son 21,63 m  ✅ kurtuldu
        GPU : ilk tetik  777. adım · 1 kez · x_son  5,35 m  🔴 sıkışık

    Yani GPU'da araç uzun süre "durgun" SAYILMIYOR — eşiğin hemen üstünde
    titreyip yerinde kalıyor, kurtarma geç açılıyor ve 90 s yetmiyor.
    Mekanizma kaotik DEĞİL (başlangıç 1e-5 bozulunca CPU'da yine kurtuluyor)
    ⇒ kalan fark CuPy hesap yolunda sistematik.
    """
    from prototype.planning.rrt_star import Bounds
    from prototype.tests.test_sikisma_kurtarmasi import _tek_direk_sahnesi

    b = Bounds(-5.0, 60.0, -20.0, 20.0)
    pipe, dyn, state = _tek_direk_sahnesi(b, stuck_recovery_enabled=True)
    if "cupy" not in pipe._mppi.backend_adi:
        pytest.skip("numpy yolu — kıyas anlamsız")
    dt = 0.1
    ilk_tetik = None
    for i in range(900):
        pipe.set_state(state)
        u = pipe.compute_control()
        if u is None or not np.all(np.isfinite(u)):
            break
        for _ in range(2):
            state = dyn.step_rk4(state, u, dt / 2)
        if pipe._kurtarma_aktif and ilk_tetik is None:
            ilk_tetik = i
    if state[0] > 15.0:
        pytest.fail(
            f"GPU yolunda kurtarma ARTIK ÇALIŞIYOR (x={state[0]:.2f}, "
            f"ilk tetik {ilk_tetik}) — bu dosyanın notu güncellenmeli")
    assert ilk_tetik is None or ilk_tetik > 300, (
        f"tetik {ilk_tetik}. adımda — CPU'daki 101 ile kıyaslanabilir hale "
        "gelmiş, teşhis değişti")
