"""PARKUR-3 TEMAS NİŞANI + engel muafiyeti (19.08.2026) — ROS'suz.

🔴 Neden var: nişan hedefin ÜSTÜNE kurulunca araç hedefe VARMIYOR (ölçüldü:
dağıtım ayarıyla 8 yaklaşma geometrisinde temas 1/8; kalanlarda araç hedefin
0,33-1,29 m önünde sıfır itkiyle park ediyor). Şartname md 5.5.2.5 tamamlama
şartı **fiziksel temas** ⇒ Parkur-3 puanı (İHA'sız 100) gidiyordu.

Bu dosyanın ASIL işi P1/P2'yi korumak: her iki mekanizma da yalnız PARKUR3'te
ve yalnız KİLİT varken devreye girer.
"""
from __future__ import annotations

import math

import pytest

from prototype.mission.hedef_secim import (
    HEDEF_YARICAP_M, MUAFIYET_YARICAP_M, Hedef, TemasNisani, temas_havucu,
    temas_muafiyeti,
)


class _Engel:
    """`CircleObstacle` ile uyumlu en küçük arayüz (duck-typing)."""

    def __init__(self, cx: float, cy: float, r: float = 0.32) -> None:
        self.cx, self.cy, self.r = cx, cy, r


# ───────────────────────── havuç TÜRETMESİ ─────────────────────────
def test_havuc_UC_OLCULEN_buyuklukten_turer():
    """0,32 (şartname Ø640) + 1,04/2 (burun) + 2,00 (varış) = 2,84 m."""
    assert temas_havucu(1.04, 2.0) == pytest.approx(2.84)


def test_havuc_VARIS_YARICAPINI_ASAR():
    """🔑 Pazarlıksız: havuç varış yarıçapının ALTINDA kalırsa terminal gradyan
    temastan ÖNCE ölür (kapı ölçümü: %31 ↔ %62,5)."""
    for arrival in (1.0, 2.0, 3.0):
        assert temas_havucu(1.04, arrival) > arrival


# ───────────────────────── nişan: hedefin ARKASI ─────────────────────────
def test_nisan_hedefin_ARKASINA_konur():
    n = TemasNisani(2.84)
    hedef = Hedef(10.0, 0.0, 1)
    x, y = n.nisan(hedef, (0.0, 0.0))
    assert (x, y) == pytest.approx((12.84, 0.0))


def test_nisan_hedeften_HAVUC_kadar_uzakta():
    n = TemasNisani(2.84)
    hedef = Hedef(6.0, 8.0, 2)                     # 10 m, çapraz
    x, y = n.nisan(hedef, (0.0, 0.0))
    assert math.hypot(x - 6.0, y - 8.0) == pytest.approx(2.84)


def test_EKSEN_KILIT_ANINDA_DONAR():
    """🔴 Ölçüm: dönen eksen 1/5 temas ↔ donmuş eksen 4/5.

    Araç yana kaydıkça nişan hedefin ETRAFINDA dönmemeli — yoksa araç onu
    kovalayıp hedefin YANINDAN geçer ve yanında park eder.
    """
    n = TemasNisani(3.0)
    hedef = Hedef(10.0, 0.0, 1)
    ilk = n.nisan(hedef, (0.0, 0.0))               # eksen +x'te donar
    # Araç 8 m yana kaysın: ham doğrultu tamamen değişti
    sonra = n.nisan(hedef, (0.0, 8.0))
    assert sonra == pytest.approx(ilk)


def test_sifirla_EKSENI_BIRAKIR():
    n = TemasNisani(3.0)
    hedef = Hedef(10.0, 0.0, 1)
    n.nisan(hedef, (0.0, 0.0))
    assert n.eksen is not None
    n.sifirla()
    assert n.eksen is None
    # yeni angajman farklı yönden gelebilir
    x, y = n.nisan(hedef, (10.0, -10.0))
    assert y > 0.0


def test_hedefin_USTUNDEYSE_oteleme_YAPILMAZ():
    """Yön tanımsız — sıfıra bölme yerine ham hedef döner."""
    n = TemasNisani(3.0)
    hedef = Hedef(5.0, 5.0, 1)
    assert n.nisan(hedef, (5.0, 5.0)) == pytest.approx((5.0, 5.0))


def test_taze_konum_kullanilir_eksen_KORUNUR():
    """Hedef konumu tazelenince nişan onunla kayar, eksen aynı kalır."""
    n = TemasNisani(2.0)
    ilk = n.nisan(Hedef(10.0, 0.0, 1), (0.0, 0.0))
    assert ilk == pytest.approx((12.0, 0.0))
    sonra = n.nisan(Hedef(10.5, 0.3, 1), (1.0, 0.4))
    assert sonra == pytest.approx((12.5, 0.3))      # eksen hâlâ +x


# ───────────────────── engel muafiyeti (Nav2 deseni) ─────────────────────
def test_KILITLI_hedefin_engel_kaydi_CIKARILIR():
    engeller = [_Engel(25.0, 0.0), _Engel(25.0, 4.0)]
    kalan = temas_muafiyeti(engeller, (25.0, 0.0))
    assert [(o.cx, o.cy) for o in kalan] == [(25.0, 4.0)]


def test_KILIT_YOKSA_liste_BIREBIR_ayni():
    """P1/P2 koruması: kilit yoksa hiçbir engel düşmez."""
    engeller = [_Engel(1.0, 2.0), _Engel(3.0, 4.0)]
    kalan = temas_muafiyeti(engeller, None)
    assert [(o.cx, o.cy) for o in kalan] == [(1.0, 2.0), (3.0, 4.0)]


def test_muafiyet_yaricapi_BASKA_HEDEFI_asla_kapsamaz():
    """🔑 Geometrik KANIT: iki Ø0,64 m duba merkezleri 0,64 m'den yakın
    olamaz ⇒ yarıçap = ÇAP seçildiği için başka hedef muaf tutulamaz.
    (TS3: yanlış temas 100 → 50 → 5.)
    """
    assert MUAFIYET_YARICAP_M == pytest.approx(2 * HEDEF_YARICAP_M)
    komsu = _Engel(25.0 + MUAFIYET_YARICAP_M + 1e-9, 0.0)
    assert temas_muafiyeti([komsu], (25.0, 0.0)) == [komsu]


def test_muafiyet_sinirinda_ELENIR():
    """Sınır davranışı donduruldu: yarıçap İÇİNDE kalan çıkarılır."""
    icteki = _Engel(25.0 + MUAFIYET_YARICAP_M - 0.01, 0.0)
    assert temas_muafiyeti([icteki], (25.0, 0.0)) == []


def test_hedef_yaricapi_SARTNAMEDEN():
    """Şartname s.18: hedef duba Ø640 mm."""
    assert HEDEF_YARICAP_M == pytest.approx(0.640 / 2)
