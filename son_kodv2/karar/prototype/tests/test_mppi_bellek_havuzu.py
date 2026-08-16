"""F-F.21 — cupy bellek havuzu temizliği (ROS'SUZ).

Ölçüm bağlamı: GIRDAP_DURUM §1.04. 14.08 koşumunda RAM 14:00→14:05 arası
**+2 GB** sıçradı, 50 dakika o seviyede kaldı ve 14:53'te **tüm yığın ~10
saniye dondu** (MAVROS IMU dahil → makinenin kendisi). Kök neden: cupy'nin
varsayılan havuzu blokları işletim sistemine geri vermiyor ve Jetson'da GPU
belleği **sistem RAM'inin kendisi**.

Bu testler cupy GEREKTİRMEZ — sahte bir modülle davranış dondurulur.
"""
from __future__ import annotations

import numpy as np
import pytest

from prototype.dynamics.catamaran import CatamaranDynamics
from prototype.planning.mppi import MPPIConfig, MPPIController
from prototype.planning.rrt_star import Bounds


class _SahteHavuz:
    def __init__(self) -> None:
        self.serbest_cagrisi = 0

    def free_all_blocks(self) -> None:
        self.serbest_cagrisi += 1

    def used_bytes(self) -> int:
        return 111

    def total_bytes(self) -> int:
        return 222


class _SahteCupy:
    """`xp` yerine geçer; yalnız testin dokunduğu yüzeyi taşır."""

    __name__ = "cupy"

    def __init__(self) -> None:
        self.havuz = _SahteHavuz()
        self.sabit_havuz = _SahteHavuz()

    def get_default_memory_pool(self):          # noqa: ANN201
        return self.havuz

    def get_default_pinned_memory_pool(self):   # noqa: ANN201
        return self.sabit_havuz


def _kontrolcu(**kw) -> MPPIController:
    return MPPIController(
        CatamaranDynamics(),
        Bounds(-1000.0, 1000.0, -1000.0, 1000.0),
        [],
        MPPIConfig(K=8, T=4, backend="numpy", **kw),
    )


def test_cupy_yolunda_havuz_serbest_birakilir() -> None:
    k = _kontrolcu()
    k.xp = _SahteCupy()
    k._bellek_havuzunu_serbest_birak()
    assert k.xp.havuz.serbest_cagrisi == 1
    assert k.xp.sabit_havuz.serbest_cagrisi == 1, "sabitlenmiş havuz atlandı"
    assert k._bellek_temizleme_sayaci == 1


def test_numpy_yolunda_HICBIR_SEY_yapilmaz() -> None:
    """numpy'de havuz yok; kod oraya dokunmaya çalışırsa çöker."""
    k = _kontrolcu()
    assert getattr(k.xp, "__name__", "") == "numpy"
    k._bellek_havuzunu_serbest_birak()          # patlamamalı
    assert k._bellek_temizleme_sayaci == 0


def test_bayrak_kapaliyken_temizlenmez() -> None:
    """False → eski davranış birebir (A/B ölçümü için)."""
    k = _kontrolcu(bellek_havuzu_temizle=False)
    k.xp = _SahteCupy()
    k._bellek_havuzunu_serbest_birak()
    assert k.xp.havuz.serbest_cagrisi == 0


def test_surucu_hatasi_kontrol_dongusunu_DUSURMEZ() -> None:
    """🛟 Bu bir iyileştirme; havuz çağrısı patlarsa MPPI ölmemeli."""
    class _Patlayan(_SahteCupy):
        def get_default_memory_pool(self):      # noqa: ANN201
            raise RuntimeError("surucu hatasi")

    k = _kontrolcu()
    k.xp = _Patlayan()
    k._bellek_havuzunu_serbest_birak()          # sessizce geçmeli
    assert k._bellek_temizleme_sayaci == 0


def test_havuz_olcumu_teshis_icin_okunabilir() -> None:
    """Sayı bantta/logda görünmezse tekrar edip etmediği sonradan anlaşılamaz."""
    k = _kontrolcu()
    assert k.bellek_havuzu_bayt() == (0, 0)     # numpy
    k.xp = _SahteCupy()
    assert k.bellek_havuzu_bayt() == (111, 222)


def test_TAM_TARAMA_dususunde_havuz_temizlenir() -> None:
    """🔑 Asıl sözleşme: kenar-fallback (yüzlerce MB'lik tam tarama) olduğunda
    havuz BOŞALTILMALI. 14.08'de tek bir fallback havuzun tepe değerini kalıcı
    yükseltip sistemi dondurdu."""
    # Fallback yolunu tam MPPI koşumuyla tetiklemek pahalı; sözleşmeyi
    # KAYNAK ÜZERİNDEN donduruyoruz — çağrı silinirse ya da fallback bloğunun
    # dışına taşınırsa test kırmızı olur.
    import inspect

    kaynak = inspect.getsource(MPPIController._trajectory_cost)
    assert "_bellek_havuzunu_serbest_birak" in kaynak, (
        "tam tarama düşüşünde havuz temizleme çağrısı KALDIRILMIŞ — "
        "F-F.21 geri geldi (§1.04)"
    )
    # ve çağrı fallback bloğunun içinde olmalı, sonunda değil
    i_fb = kaynak.index("_ref_window_fallbacks += 1")
    i_tmz = kaynak.index("_bellek_havuzunu_serbest_birak")
    assert i_tmz > i_fb, "temizleme çağrısı fallback bloğunun dışına taşınmış"
