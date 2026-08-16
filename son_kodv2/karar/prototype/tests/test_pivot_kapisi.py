"""F-F.20 — pivot kapısı çekirdek testleri (ROS'SUZ).

Ölçüm bağlamı: GIRDAP_DURUM §1.01. Eşikler teknenin kendi `WP_PIVOT_ANGLE=60`
ayarından ve ArduPilot'un "10° içinde devam et" kuralından gelir; testler bu
iki sayıyı ve **histerezisi** dondurur.
"""
from __future__ import annotations

import math

import pytest

from prototype.control.pivot_kapisi import (
    PivotKapisi,
    PivotKapisiConfig,
    pivot_itkisi,
)


def _kapi(tetik: float = 60.0, birak: float = 10.0, ufuk: float = 3.0):
    return PivotKapisi(
        PivotKapisiConfig(tetik_derece=tetik, birak_derece=birak, ufuk_m=ufuk)
    )


def _yol(*noktalar):
    return list(noktalar)


def test_buyuk_yon_hatasinda_acilir() -> None:
    """Araç doğuya bakıyor, hedef kuzeyde (90°) → pivot."""
    k = _kapi()
    aktif, hata = k.guncelle(0.0, 0.0, 0.0, _yol((0.0, 10.0)))
    assert aktif is True
    assert math.degrees(hata) == pytest.approx(90.0)


def test_kucuk_yon_hatasinda_acilmaz() -> None:
    """30° hata → 60° eşiğinin altında, normal sürüş sürer."""
    k = _kapi()
    aktif, _ = k.guncelle(0.0, 0.0, 0.0, _yol((10.0, 5.77)))   # ~30°
    assert aktif is False


def test_HISTEREZIS_ara_bolgede_ACIK_kalir() -> None:
    """🔑 Tek eşik olsaydı araç 60° sınırında 10 Hz'te açılıp kapanır ve
    düzeltmek istediğimiz salınımın AYNISI üretilirdi."""
    k = _kapi()
    assert k.guncelle(0.0, 0.0, 0.0, _yol((0.0, 10.0)))[0] is True     # 90° → aç
    # 30°: tetik eşiğinin ALTINDA ama bırakma eşiğinin ÜSTÜNDE → hâlâ dönmeli
    assert k.guncelle(0.0, 0.0, 0.0, _yol((10.0, 5.77)))[0] is True
    assert k.aktif is True


def test_birakma_esiginde_kapanir() -> None:
    k = _kapi()
    k.guncelle(0.0, 0.0, 0.0, _yol((0.0, 10.0)))                       # aç
    aktif, _ = k.guncelle(0.0, 0.0, 0.0, _yol((10.0, 0.9)))            # ~5°
    assert aktif is False


def test_sifir_tetik_kapiyi_TAMAMEN_kapatir() -> None:
    """0 → eski davranış birebir (A/B ölçümü için)."""
    k = _kapi(tetik=0.0)
    assert k.guncelle(0.0, 0.0, 0.0, _yol((-10.0, 0.0)))[0] is False   # 180°


def test_referans_yoksa_DONULMEZ() -> None:
    """Neye döneceğini bilmeden dönmek, kör sürmenin dönen hâli olurdu."""
    k = _kapi()
    assert k.guncelle(0.0, 0.0, 0.0, None) == (False, None)
    assert k.guncelle(0.0, 0.0, 0.0, []) == (False, None)


def test_ufuk_noktasi_yakin_gurultuyu_atlar() -> None:
    """Hemen yanı başındaki nokta yön açısını gürültüye çevirir; kapı
    referansın >= ufuk_m ilerisindeki İLK noktasına bakmalı."""
    k = _kapi(ufuk=3.0)
    # ilk iki nokta çok yakın ve ileriyi göstermiyor; asıl yön 3 m sonra
    aktif, hata = k.guncelle(
        0.0, 0.0, 0.0, _yol((0.1, 0.0), (1.0, 0.0), (0.0, 5.0))
    )
    assert aktif is True
    assert math.degrees(hata) == pytest.approx(90.0)


def test_hedefe_cok_yakinken_kapi_ACILMAZ() -> None:
    """Varışta atan2 gürültüye döner — dönmeye çalışmak yalpa üretirdi."""
    k = _kapi(ufuk=3.0)
    assert k.guncelle(0.0, 0.0, 0.0, _yol((0.0, 0.2)))[0] is False


def test_sifirla_kapiyi_dusurur() -> None:
    k = _kapi()
    k.guncelle(0.0, 0.0, 0.0, _yol((0.0, 10.0)))
    assert k.aktif is True
    k.sifirla()
    assert k.aktif is False


# --------------------------------------------------------------------------- #
# pivot_itkisi — işaret ve "ileri bileşen sıfır" sözleşmesi
# --------------------------------------------------------------------------- #


def test_pivot_itkisinin_ILERI_bileseni_SIFIR() -> None:
    """`linear.x ∝ (u0+u1)` → toplam sıfır olmalı, yoksa pivot ilerler."""
    for hata in (+1.0, -1.0):
        u = pivot_itkisi(hata, 1.455)
        assert sum(u) == pytest.approx(0.0)


def test_pivot_itkisi_DOGRU_YONE_donduruyor() -> None:
    """`angular.z ∝ (u1-u0)`; pozitif hata (hedef solda) → pozitif yaw."""
    u_sol = pivot_itkisi(+0.5, 1.455)
    assert u_sol[1] - u_sol[0] > 0
    u_sag = pivot_itkisi(-0.5, 1.455)
    assert u_sag[1] - u_sag[0] < 0


def test_pivot_itkisi_buyuklugu_korunur() -> None:
    u = pivot_itkisi(1.0, 1.455)
    assert abs(u[0]) == pytest.approx(1.455)
    assert abs(u[1]) == pytest.approx(1.455)
