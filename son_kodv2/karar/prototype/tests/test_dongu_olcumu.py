"""F-K.1 — kontrol döngüsü süre ölçer testleri (rclpy GEREKMEZ).

Neden bu testler var: `DonguOlcer`, sahada tıkanmayı görünür kılan TEK kanal.
Sahada SSH yok (md 4.1) — bu sayaç yanlış sayarsa operatör yanlış karar verir.
Her test, sahada karşılaşacağımız bir durumu dondurur.
"""
from __future__ import annotations

import pytest

from prototype.control.dongu_olcer import DonguOlcer


def _kosu(olcer, periyot_s, is_s, sure_s, t0=100.0):
    """`sure_s` boyunca `periyot_s` aralıkla, her biri `is_s` süren adımlar.

    Dönen: üretilen raporların listesi.
    """
    raporlar = []
    t = t0
    while t - t0 < sure_s:
        r = olcer.kaydet(t, t + is_s)
        if r is not None:
            raporlar.append(r)
        t += periyot_s
    return raporlar


def test_saglikli_dongu_gercek_hizi_dogru_verir():
    """10 Hz hedefte 10 Hz koşuyorsak: 10.0 Hz ve sağlıklı."""
    r = _kosu(DonguOlcer(10.0), periyot_s=0.10, is_s=0.02, sure_s=11.0)
    assert r, "rapor hic uretilmedi"
    assert r[-1].gercek_hz == pytest.approx(10.0, abs=0.05)
    assert r[-1].saglikli
    assert not r[-1].bosluk_alarmi


def test_YARI_HIZDA_kosarsa_saglikli_DEGIL():
    """§0.12b'nin ta kendisi: 10 Hz hedef, 5 Hz gerçek.

    Bu testin koruduğu davranış tüm hata avının çıkış noktası — Eyüp'ün
    "gate görüyor ama adam akıllı gidemiyor" tarifi.
    """
    r = _kosu(DonguOlcer(10.0), periyot_s=0.20, is_s=0.02, sure_s=11.0)
    assert r[-1].gercek_hz == pytest.approx(5.0, abs=0.05)
    assert not r[-1].saglikli, "yari hizda kosarken saglikli deniyor"
    assert "GERÇEK 5.0 Hz" in r[-1].ozet()


def test_butce_asimi_sayilir():
    """İş süresi bütçeyi aşarsa sayaç artmalı — Orin'de MPPI 343 ms senaryosu."""
    r = _kosu(DonguOlcer(10.0), periyot_s=0.35, is_s=0.34, sure_s=11.0)
    assert r[-1].asim_oran == pytest.approx(100.0)
    assert not r[-1].saglikli
    assert "bütçe aşımı" in r[-1].ozet()


def test_IS_kucukken_ARALIK_buyuk_olabilir():
    """🔑 İki ölçümün ayrı tutulmasının sebebi.

    Aynı executor'daki başka bir iş bizi geciktirirse `is_s` küçük kalır ama
    gerçek hız yarıya düşer. Yalnız `is_s` ölçülseydi bu arıza GÖRÜNMEZDİ —
    ve tekneyi durduran şey tam olarak bu.
    """
    r = _kosu(DonguOlcer(10.0), periyot_s=0.25, is_s=0.005, sure_s=11.0)
    assert r[-1].is_maks_s < 0.01, "is suresi kucuk olmali"
    assert r[-1].asim == 0, "butce asimi YOK — is ucuz"
    assert not r[-1].saglikli, "ama gercek hiz 4 Hz; saglikli denemez"
    assert r[-1].gercek_hz == pytest.approx(4.0, abs=0.05)


def test_UZUN_BOSLUK_ardupilot_alarmi():
    """1,5 sn'yi aşan tek bir boşluk bile alarm — ArduPilot 3 sn'de tekneyi
    durduruyor ve bunu kimseye söylemiyor."""
    o = DonguOlcer(10.0)
    o.kaydet(100.0, 100.02)
    o.kaydet(102.0, 102.02)                       # 2 sn boşluk
    r = _kosu(o, periyot_s=0.10, is_s=0.02, sure_s=11.0, t0=102.1)
    assert r[0].bosluk_alarmi
    assert "ArduPilot" in r[0].bosluk_mesaji()
    assert "3 sn" in r[0].bosluk_mesaji()


def test_kisa_boslukta_alarm_YOK():
    """Eşik altı gecikme alarm üretmemeli — yoksa uyarı gürültüye boğulur
    ve gerçek olay gözden kaçar."""
    r = _kosu(DonguOlcer(10.0), periyot_s=0.30, is_s=0.02, sure_s=11.0)
    assert not r[-1].bosluk_alarmi


def test_sayaclar_rapordan_sonra_SIFIRLANIR():
    """Sıfırlanmazsa maks değerler tüm koşuya yapışır ve sağlığa dönmüş bir
    sistemi hâlâ arızalı gösterir."""
    o = DonguOlcer(10.0)
    _kosu(o, periyot_s=0.20, is_s=0.30, sure_s=11.0)              # kötü pencere
    r2 = _kosu(o, periyot_s=0.10, is_s=0.02, sure_s=11.0, t0=200.0)
    assert r2[-1].saglikli, "duzelmis dongu hala arizali gorunuyor"
    assert r2[-1].is_maks_s < 0.05, "onceki pencerenin maks'i yapismis"


def test_ilk_cagri_rapor_uretmez():
    """Tek örnekle hız hesaplamak anlamsız; ilk çağrı yalnız zamanı kurar."""
    assert DonguOlcer(10.0).kaydet(100.0, 100.02) is None


def test_rapor_periyodundan_once_susar():
    """Her adımda log basmak sahada journal'ı boğar."""
    o = DonguOlcer(10.0, rapor_periyot_s=10.0)
    assert _kosu(o, periyot_s=0.10, is_s=0.02, sure_s=5.0) == []


def test_gecersiz_hedef_hz_reddedilir():
    """0 ya da negatif hedef sessizce sıfıra bölmemeli."""
    with pytest.raises(ValueError):
        DonguOlcer(0.0)
    with pytest.raises(ValueError):
        DonguOlcer(-10.0)
