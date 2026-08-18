# -*- coding: utf-8 -*-
"""ÖLÜ BANT — kapı kadrajdan çıkınca geçiş tetiği ateşlenebiliyor mu?

🔴 DOĞURAN VAKA (18.08.2026): geçiş fazına giriş sabit `PASS_KAYIP_Z = 3,2 m`
eşiğine bağlıydı. Kapı genişliğine göre çok daha uzakta kadrajdan çıkar
(`b/tan(HFOV/2)`), bu yüzden **W > ~4,4 m** olan hiçbir kapı o eşiğe inemiyor
ve geçiş HİÇ sayılmıyordu.

Sanal gölde ölçüldü: 483 karede kapı **138 kez kuruldu**, geçiş tetiği
**0 kez** ateşlendi; kurulan kapının en yakın orta menzili **3,91 m**.
A/B: 12 m ✗ · 6 m ✗ · 4 m ✓ (ilk kez GECIS fazı).
🔑 Sahada da aynı belirti vardı (kaptan: *"realde de gate'den geçmiyordu"*)
— yani bu bir sim artefaktı DEĞİL.
"""
from __future__ import annotations

import math

import pytest

from girdap_ida_algi import gecit_mantik as gm


def _kayip(W: float) -> float:
    return gm.fov_kayip_menzili(W / 2.0)


def test_kayip_menzili_GEOMETRI_ile_birebir():
    """z = b / tan(HFOV/2) — saf pinhole, ayarlanmış sabit değil."""
    for W in (2.0, 4.0, 6.0, 12.0, 40.0):
        beklenen = (W / 2.0) / math.tan(gm.HFOV_RAD / 2.0)
        assert _kayip(W) == pytest.approx(beklenen, rel=1e-12)


def test_genis_kapi_UZAKTA_kaybolur_ESKI_esik_yakalayamazdi():
    """Ölü bandın kendisini dondurur: eski 3,2 m sabiti neden yetmiyordu."""
    assert _kayip(4.0) < 3.2, "4 m kapı eski eşikle de yakalanıyordu"
    assert _kayip(6.0) > 3.2, "6 m kapı eski eşiğin ALTINA inebiliyor?"
    assert _kayip(12.0) > 3.2
    kritik = 2.0 * 3.2 * math.tan(gm.HFOV_RAD / 2.0)
    assert 4.3 < kritik < 4.5, f"kritik genişlik {kritik:.2f} m beklenenden uzak"


@pytest.mark.parametrize("W", [2.0, 4.0, 6.0, 12.0, 40.0])
def test_HER_genislikte_geometrik_kayip_YAKALANIR(W):
    """🔑 Asıl kazanım: ölçüt ölçek-bağımsız — 2 m de 40 m de çalışır."""
    b = W / 2.0
    assert gm.fov_kaybi_mi(_kayip(W), b), f"{W} m kapı hâlâ ölü bantta"


@pytest.mark.parametrize("W", [2.0, 6.0, 12.0])
def test_UZAKTA_kaybolan_kapi_gecis_SAYILMAZ(W):
    """Tespit düşmesi ≠ geçiyoruz. Beklenen menzilin çok üstünde kaybolan
    kapı için tetik ateşlenmemeli — yoksa sahte geçiş üretilir."""
    b = W / 2.0
    assert not gm.fov_kaybi_mi(_kayip(W) * 3.0, b)


def test_pay_sinirda_calisir_ama_sinirsiz_DEGIL():
    """Pay tespit gecikmesini karşılar; keyfî büyümez."""
    b = 3.0
    z = gm.fov_kayip_menzili(b)
    assert gm.fov_kaybi_mi(z * gm.FOV_KAYIP_PAYI * 0.99, b)
    assert not gm.fov_kaybi_mi(z * gm.FOV_KAYIP_PAYI * 1.02, b)
    assert 1.0 <= gm.FOV_KAYIP_PAYI <= 1.5


def test_bozuk_girdide_IDDIA_ETMEZ():
    """Ölçemediğimizde 'geçtik' demeyiz (kör sayım yok)."""
    assert not gm.fov_kaybi_mi(None, 3.0)
    assert not gm.fov_kaybi_mi(4.0, None)
    assert not gm.fov_kaybi_mi(-1.0, 3.0)
    assert gm.fov_kayip_menzili(0.0) == 0.0
    assert gm.fov_kayip_menzili(-2.0) == 0.0


def test_dugum_ARTIK_sabit_esigi_KULLANMIYOR():
    """`PASS_KAYIP_Z` tarihsel kayıt olarak duruyor; karar yolunda OLMAMALI."""
    # ⚠ Ham metin araması YETMEZ: değişikliği ANLATAN yorum satırlarında da
    # `PASS_KAYIP_Z` geçiyor ve test onları eşleyip boşuna kırmızı yanıyordu
    # (ölçüldü). Aranan şey ÇALIŞAN kod; bu yüzden AST'ten okunur.
    import ast
    import inspect
    import textwrap

    from girdap_ida_algi import duba_gecis_navigator as nav
    agac = ast.parse(textwrap.dedent(inspect.getsource(nav.DubaNavigator)))
    adlar = {n.id for n in ast.walk(agac) if isinstance(n, ast.Name)}
    nitelikler = {n.attr for n in ast.walk(agac) if isinstance(n, ast.Attribute)}
    assert "PASS_KAYIP_Z" not in adlar, (
        "sabit metre eşiği karar yoluna geri döndü — ölü bant geri gelir")
    assert "fov_kaybi_mi" in nitelikler, "geometrik ölçüt karar yolunda yok"
