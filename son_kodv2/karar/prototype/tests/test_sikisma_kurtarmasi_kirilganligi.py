# -*- coding: utf-8 -*-
"""SIKIŞMA KURTARMASI RASTGELE DİZİYE BAĞIMLIYDI (19.08.2026) — İKİ KATMANLI ARIZA

🔴 NEDEN ÖNEMLİ — DOĞRUDAN KAPI KONUSU: "370 s donma" arızası tekne bir
KAPI DİREĞİNİN huni payında sıkışırken oluşuyordu. Kurtarma mekanizması o
yüzden var. Mekanizma çalışmazsa araç kapı önünde donar ⇒ **kapıdan geçemez**.

## KATMAN 1 — RNG UYUMSUZLUĞU (takım, RTX 4060 + CuPy 14.1.1, 19.08 gece)

| yol | dtype | u0 | max ağırlık | ESS | test |
|---|---|---|---|---|---|
| CPU (numpy) | float64 | [0.1128, 0.2955] | 0.146 | 15,2 | ✅ 5/5 |
| CPU + float32 | float32 | [0.1128, 0.2955] | 0.146 | 15,2 | ✅ |
| **GPU (cupy), DÜZELTME ÖNCESİ** | float32 | [0.1469, 0.2828] | **0.299** | **5,2** | 🔴 6/6 |

Kök neden `numpy`/`cupy`'nin aynı tohumda TAMAMEN FARKLI dizi üretmesiydi
(`MPPIController.__init__` `self._rng = self.xp.random.default_rng(seed)`
diyordu). DÜZELTME (`mppi.py`, commit b3caa601): `_rng` artık backend'den
BAĞIMSIZ, HER ZAMAN `np.random.default_rng(seed)`.

**Jetson'da gerçek CuPy ile doğrulandı:** ESS 5,2 → 15,2 (CPU ile birebir),
u0 CPU [0.11280112,0.29550353] ↔ GPU [0.11281674,0.2954855].

## KATMAN 2 — RNG DÜZELTİLDİKTEN SONRA KALAN FARK: TETİKLEME ZAMANI (takım)

ESS iki yolda da 15,2 (dejenerasyon kapandı) ama kurtarma GPU'da HÂLÂ geç
kalıyordu:

    CPU : ilk tetik  101. adım · 2 kez · x_son 21,63 m  ✅ kurtuldu
    GPU : ilk tetik  777. adım · 1 kez · x_son  5,35 m  🔴 sıkışık

Sebep: `_sikisma_kurtarmasini_guncelle` ANLIK hızı eşikle karşılaştırıyordu;
CPU/GPU'nun float64/float32 farkı zamanla birikip anlık hızı eşiğin HEMEN
üstünde titreştiriyordu — tek bir üst-eşik okuma durgunluk sayacını
sıfırlıyordu (kaotik değil: CPU'da başlangıç 1e-5 bozulunca AYNI gecikme
deseni çıkıyordu — yani CuPy hesap yolundaki sistematik ufak farka karşı
YAPISAL bir kırılganlıktı, rastgele bir şans değil).

## KATMAN 2 DÜZELTMESİ (`pipeline.py`, 19.08 gece, ikinci tur)
`_sikisma_kurtarmasini_guncelle` artık ANLIK hız yerine TÜMSEK (tumbling)
PENCERE boyunca NET yer değiştirmeye bakıyor — MPPI'nin kendi ufku kadar
süren pencerenin sonunda net mesafe eşik-mesafenin altındaysa "durgun"
sayılır. Tek bir titreşimli anlık okumaya bağışık, çünkü ölçüt zaten bir
SÜRE boyunca biriktirilen konum farkı. Giriş/çıkış AYNI ölçütle simetrik.

⚠ Bu ikinci düzeltme bu commit'te YALNIZ CPU'da doğrulandı (mutasyon
testiyle). Jetson/GPU doğrulaması AYRI adımda yapılacak — aşağıdaki iki
test o yüzden HÂLÂ "KAYIT" (kırmızı beklenir) formunda tutuluyor; gerçek
donanımda doğrulanınca "DÜZELDİ" kilidine çevrilecek (bkz. katman 1'in
kendi geçmişindeki aynı desen).
"""
from __future__ import annotations

import numpy as np
import pytest

cp = pytest.importorskip("cupy", reason="GPU yolu yok — bu makinede sınanamaz")


def test_MPPI_gurultusu_backendden_BAGIMSIZ():
    """✅ 19.08 DÜZELTİLDİ (`b3caa601`), Jetson'da GERÇEK CuPy ile doğrulandı.

    Kütüphanelerin kendi PRNG'leri hâlâ farklı dizi üretir (aşağıda
    gösteriliyor) — düzeltme, MPPI'nin artık ONLARA BAĞLI OLMAMASI.
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


def test_kurtarma_GPU_yolunda_tumsek_pencere_SONRASI():
    """KATMAN 2 (tümsek pencere) GPU sınaması — 19.08'de KİLİDE ÇEVRİLDİ.

    Docstring'in beklediği koşum yapıldı (Jetson/Orin, CuPy 13.6.0, GPU yolu
    açık) ve sonuç netleşti — ama önce KATMAN 3 gerekti:

    🔴 Tümsek pencere TEK BAŞINA yetmiyordu, çünkü pencerenin uzunluğu
    `self._saat()` ile ölçülüyor (varsayılan `time.monotonic` = DUVAR saati),
    eşiği ise `mppi_T * mppi_dt` = 2,5 s **SİMÜLASYON** ufku. Sahne kendi
    saatini enjekte etmediği için iki zaman ekseni karışıyordu: 900 adımlık
    döngü hızlı bir makinede 2,5 s duvar saatinden ÖNCE bitiyor ⇒ pencere HİÇ
    kapanmıyor ⇒ kurtarma HİÇ tetiklenmiyor.

    ÖLÇÜLDÜ — üç makinenin üçü de TEK bu nedenle açıklanıyor:
      · hızlı makine (RTX 4060, GPU)   → %100 kırmızı, sayaç 0, x=5,12
      · aynı makine, CPU (daha YAVAŞ)  → %100 yeşil
      · Jetson/Orin (arada)            → %17 kararsız
    🔑 Kusurlu hâldeki x=5,12 ve sayaç=0, kurtarmanın KAPALI olduğu kolun
    BİREBİR aynısı ⇒ mekanizma sessizce devre dışıydı.

    Düzeltme (`_tek_direk_sahnesi` sim saatini enjekte ediyor) sonrası
    Jetson GPU yolunda ölçülen: **sayaç 7 · x = 21,52 m**. Artık kilit.
    """
    from prototype.planning.rrt_star import Bounds
    from prototype.tests.test_sikisma_kurtarmasi import _tek_direk_sahnesi

    b = Bounds(-5.0, 60.0, -20.0, 20.0)
    pipe, dyn, state, saat = _tek_direk_sahnesi(b, stuck_recovery_enabled=True)
    if "cupy" not in pipe._mppi.backend_adi:
        pytest.skip("bu koşumda MPPI numpy yolunda — GPU yolu sınanamıyor")
    dt = 0.1
    ilk_tetik = None
    for i in range(900):
        pipe.set_state(state)
        u = pipe.compute_control()
        if u is None or not np.all(np.isfinite(u)):
            break
        for _ in range(2):
            state = dyn.step_rk4(state, u, dt / 2)
        saat.ilerlet(dt)
        if pipe._kurtarma_aktif and ilk_tetik is None:
            ilk_tetik = i
    print(f"\n[tümsek pencere sonrası] x_son={state[0]:.2f} ilk_tetik={ilk_tetik} "
          f"kurtarma_sayaci={pipe._kurtarma_sayaci}")
    assert pipe._kurtarma_sayaci > 0, (
        f"GPU yolunda kurtarma HİÇ tetiklenmedi (sayaç 0, x={state[0]:.2f}) — "
        "durgunluk penceresi duvar saatine düşmüş olabilir; sahne `saat` "
        "enjekte ediyor mu? (19.08 kök nedeni)"
    )
    assert state[0] > 15.0, (
        f"GPU yolunda kurtarma tetiklendi ({pipe._kurtarma_sayaci} kez) ama "
        f"araç yine sıkışık kaldı (x={state[0]:.2f})"
    )
