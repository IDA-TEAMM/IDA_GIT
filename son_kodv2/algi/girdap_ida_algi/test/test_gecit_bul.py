"""`DubaNavigator.gecit_bul()` — P1/P2'nin KALBİ, kamerasız senaryo testleri.

🔴 NEDEN YAZILDI (2026-08-10 derin tarama): saf katman `gecit_mantik.py`
%100 kapsanmıştı (16/16 fonksiyon), ama **puanı üreten seçim mantığı**
`gecit_bul()` node içinde ve kapsaması **SIFIR**dı. Şartname G tanımı:
*"FARKLI Karşılıklı KENAR Dubaları Arasından Geçiş Sayısı"* — yanlış çift
seçmek `(G1/KD1)×10` ve `(G2/KD2)×40`'ı doğrudan düşürür, üstelik yanlış
kapıya sürmek Ç1/Ç2 çarpma cezası doğurur.

YÖNTEM: `gecit_bul` `self`'ten yalnız `dubalar`, `kenar_cls`, `engel_cls`,
`_tani`, `_f_norm` ve `_menzil_saglikli`/`get_logger` kullanıyor ⇒ sahte bir
`self` ile **ROS ve kamera olmadan** çağrılabiliyor. Gerçek fonksiyonun
kendisi koşuyor — kopya mantık yok.
"""
import math
import os
import sys
import types

import pytest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

pytest.importorskip("depthai", reason="depthai kurulu değil")
_rclpy = pytest.importorskip("rclpy", reason="rclpy kurulu değil")

from girdap_ida_algi import gecit_mantik as gm  # noqa: E402
from girdap_ida_algi import duba_gecis_navigator as dgn  # noqa: E402


def _duba(cls, x, z, w=None):
    """Gerçek `Duba` veri sınıfı. w verilmezse stereo Z ile TUTARLI genişlik
    üretilir (pinhole: w = f·0,30/z) ki `_menzil_saglikli` elemesin."""
    if w is None:
        w = gm.DUBA_CAP_M * gm.odak_px(1.0) / z
    return dgn.Duba(cls=cls, x=x, z=z, conf=0.9, cx=0.5, cy=0.5, w=w)


def _sahte_node(dubalar, kenar_cls=0, engel_cls=1):
    """`gecit_bul`'un ihtiyaç duyduğu asgari `self` — ROS'suz."""
    ns = types.SimpleNamespace(
        dubalar=dubalar, kenar_cls=kenar_cls, engel_cls=engel_cls,
        _f_norm=gm.odak_px(1.0),
        _tani={"dar": 0, "dizili": 0, "arada_duba": 0, "menzil_celiski": 0,
               "mono_menzil": 0, "menzil_yok": 0},
    )
    ns._menzil_saglikli = lambda d: dgn.DubaNavigator._menzil_saglikli(ns, d)
    ns.get_logger = lambda: types.SimpleNamespace(
        warn=lambda *a, **k: None, info=lambda *a, **k: None,
        error=lambda *a, **k: None)
    return ns


def _bul(dubalar):
    return dgn.DubaNavigator.gecit_bul(_sahte_node(dubalar))


# --------------------------------------------------------------------------
# S1 — NORMAL: karşılıklı iki kenar dubası, geçilebilir genişlik → KAPI VAR
def test_karsilikli_kenar_ciftinden_kapi_kurulur():
    g = _bul([_duba(0, -1.5, 5.0), _duba(0, +1.5, 5.0)])
    assert g is not None
    a, b = g
    assert {round(a.x, 1), round(b.x, 1)} == {-1.5, 1.5}


# S2 — DAR: gövde (0,78) + 2×yarıçap sığmıyor → REDDEDİLİR
def test_govde_sigmayan_cift_reddedilir():
    ns = _sahte_node([_duba(0, -0.25, 5.0), _duba(0, +0.25, 5.0)])
    assert dgn.DubaNavigator.gecit_bul(ns) is None
    assert ns._tani["dar"] == 1, "dar sayacı artmalı (sessiz ret görünür olsun)"


# S3 — DİZİLİ: iki duba arka arkaya (kapı değil, ardışık kapı direkleri)
def test_ardisik_dizili_cift_kapi_sayilmaz():
    ns = _sahte_node([_duba(0, 0.0, 4.0), _duba(0, 0.2, 8.0)])
    assert dgn.DubaNavigator.gecit_bul(ns) is None
    assert ns._tani["dizili"] == 1


# S4 — ARADA DUBA: çiftin ortasında üçüncü kenar dubası → koridorun iki ayrı
# tarafını "karşılıklı ikili" sanma hatası (A-5)
def test_arada_ucuncu_duba_varsa_cift_reddedilir():
    ns = _sahte_node([_duba(0, -2.0, 5.0), _duba(0, +2.0, 5.0),
                      _duba(0, 0.0, 5.0)])
    g = dgn.DubaNavigator.gecit_bul(ns)
    if g is not None:                     # dar bir çift seçilmişse o da geçerli
        a, b = g
        assert abs(a.x - b.x) < 4.0
    assert ns._tani["arada_duba"] >= 1


# S5 — MENZİL TAVANI: uzaktaki kapı seçilmez.
# 🪤 İLK HÂLİ TOOTHLESS'TI (10.08 mutasyon testi yakaladı): mesafe
# `dgn.GECIT_MAX_MESAFE + 4.0` diye yazılmıştı ⇒ sabit 8→99 yapılsa test
# mesafesi de büyüyor, test ASLA kırmızıya dönmüyordu. Artık MUTLAK 30 m:
# kameramız 20 m'de zaten hiçbir şey görmüyor (10.08 menzil ölçümü), yani
# 30 m'lik bir kapının seçilmesi her koşulda hatadır. Tavan meşru sebeple
# büyütülürse (ölçüm 11-15 m'de %94 diyor) bu test yine geçer.
def test_uzak_kapi_secilmez():
    assert _bul([_duba(0, -1.5, 30.0), _duba(0, +1.5, 30.0)]) is None


# S5b — SINIR DAVRANIŞI: tavanın hemen ALTINDAKİ kapı seçilmeli
def test_tavanin_hemen_altindaki_kapi_secilir():
    z = dgn.GECIT_MAX_MESAFE - 0.5
    assert _bul([_duba(0, -1.5, z), _duba(0, +1.5, z)]) is not None


# S6 — EN YAKIN KAPI: iki geçerli kapı varsa yakın olan seçilir
def test_iki_kapidan_en_yakin_secilir():
    g = _bul([_duba(0, -1.5, 6.5), _duba(0, +1.5, 6.5),
              _duba(0, -1.5, 3.0), _duba(0, +1.5, 3.0)])
    assert g is not None
    assert (g[0].z + g[1].z) / 2.0 == pytest.approx(3.0, abs=0.3)


# S7 — 🔴 SARI ENGELDEN KAPI KURULMAZ (şartname: geçit yalnız KENAR×KENAR).
# Kurulursa tekne engellerin arasından geçmeye çalışır ⇒ Ç2 çarpma.
def test_sari_engel_ciftinden_kapi_kurulmaz():
    assert dgn.ENGEL_YEDEK is False, "saha yedeği açıksa bu senaryo değişir"
    assert _bul([_duba(1, -1.5, 5.0), _duba(1, +1.5, 5.0)]) is None


# S8 — KARIŞIK: bir kenar + bir engel → kenar çifti olmadığı için kapı YOK
def test_kenar_engel_karisimi_kapi_vermez():
    assert _bul([_duba(0, -1.5, 5.0), _duba(1, +1.5, 5.0)]) is None


# S9 — SINIF İNDEKSLERİ TERS ÇÖZÜLMÜŞSE de mantık kenar'ı izler
def test_sinif_indeksleri_ters_ise_kenar_takip_edilir():
    ns = _sahte_node([_duba(1, -1.5, 5.0), _duba(1, +1.5, 5.0)],
                     kenar_cls=1, engel_cls=0)
    assert dgn.DubaNavigator.gecit_bul(ns) is not None


# S10 — BOŞ / TEK duba → çökmeden None
@pytest.mark.parametrize("dubalar", [[], [_duba(0, 0.0, 5.0)]])
def test_yetersiz_duba_none_doner(dubalar):
    assert _bul(dubalar) is None


# S11 — 🔴 MENZİL ÇELİŞKİSİ: stereo Z ile bbox genişliği uyuşmuyorsa duba
# elenmeli (uzak/kısmen görünen duba → hayalet kapı üretmesin)
def test_menzil_celiskili_duba_elenir():
    kotu = _duba(0, +1.5, 5.0, w=gm.DUBA_CAP_M * gm.odak_px(1.0) / 1.5)
    ns = _sahte_node([_duba(0, -1.5, 5.0), kotu])
    assert dgn.DubaNavigator.gecit_bul(ns) is None
    assert ns._tani["menzil_celiski"] >= 1


# S12 — MONO menzilli duba çapraz kontrolden MUAF (aynı sayıyı kendisiyle
# karşılaştırmak anlamsız — 08.08 düzeltmesi)
def test_mono_kaynakli_duba_capraz_kontrolden_muaf():
    a = _duba(0, -1.5, 5.0, w=0.001)      # stereo ile çelişen genişlik
    b = _duba(0, +1.5, 5.0, w=0.001)
    a.kaynak = b.kaynak = "mono"
    ns = _sahte_node([a, b])
    assert dgn.DubaNavigator.gecit_bul(ns) is not None
    assert ns._tani["menzil_celiski"] == 0
