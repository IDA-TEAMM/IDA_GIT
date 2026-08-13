"""FC parametre öz-denetimi — ölümcül sapmaları bul, gerisini sus.

13.08'de Pixhawk'a bağlanıldığında **39 parametre** değişmiş bulundu.
Parametreleri belirlemek takımda başkasının görevi ve her testten sonra
güncelleniyor; farkı elle ayıklamak yarım saat sürdü. Bu testler denetimin
o günün GERÇEK verisiyle doğru davrandığını donduruyor.
"""

from __future__ import annotations

from prototype.control.param_denetimi import OLUMCUL, Bulgu, denetle, ozet

#: 13.08.2026 16:00'da Pixhawk'ta GERÇEKTEN bulunan değerler.
GERCEK_13_08 = {
    "INS_POS1_X": 0.0, "INS_POS1_Y": 0.0, "INS_POS1_Z": 0.0,
    "FS_ACTION": 0.0, "ARMING_CHECK": 1.0, "BATT_MONITOR": 0.0,
    "SR2_EXT_STAT": 10.0, "SR2_EXTRA1": 10.0, "SR2_POSITION": 10.0,
}

DOGRU = {ad: b.deger for ad, b in OLUMCUL.items()}


def test_13_agustos_gercek_sapmasi_YAKALANIYOR() -> None:
    """O günün gerçek Pix durumu → tam 5 ölümcül bulgu."""
    b = denetle(GERCEK_13_08)
    adlar = {x.ad for x in b}
    assert adlar == {
        "INS_POS1_X", "INS_POS1_Y", "INS_POS1_Z", "FS_ACTION", "BATT_MONITOR"
    }, f"beklenmeyen bulgu kumesi: {adlar}"


def test_dogru_yapilandirmada_SESSIZ() -> None:
    """🔑 Yanlış alarm, bu bekçinin en büyük düşmanı: uyarı listesi uzarsa
    okunmaz ve gerçek sapma gürültüde kaybolur."""
    assert denetle(DOGRU) == []


def test_akis_hizi_FAZLASI_sorun_degil() -> None:
    """SR2_* beklenenden BÜYÜK olabilir — fazla veri zarar değil, az veri
    zarardır (PAR-04: 0,17 Hz'de oturumun %86'sı KILL'de geçti)."""
    d = dict(DOGRU); d["SR2_EXT_STAT"] = 20.0; d["SR2_POSITION"] = 50.0
    assert denetle(d) == []


def test_akis_hizi_AZLIGI_yakalaniyor() -> None:
    d = dict(DOGRU); d["SR2_EXT_STAT"] = 0.17      # PAR-04'te ölçülen değer
    assert [x.ad for x in denetle(d)] == ["SR2_EXT_STAT"]


def test_okunamayan_parametre_bulgu_sayilir() -> None:
    """Sessizce atlamak, sapmayı gizlemekle aynı şey — okunamadı da bir bulgudur."""
    d = dict(DOGRU); d["FS_ACTION"] = None
    b = denetle(d)
    assert len(b) == 1 and b[0].okunan is None and "okunamadı" in b[0].sebep


def test_LOG_parametreleri_BILEREK_listede_yok() -> None:
    """🔑 Ölçüt: "yanlışken görev başarısız olur". LOG_* yanlışsa yalnız
    TEŞHİS kaybederiz, görev yürür — listeye girerse uyarı gürültüsü artar
    ve asıl sapmalar okunmaz hale gelir."""
    assert not any(ad.startswith("LOG_") for ad in OLUMCUL)
    assert not any(ad in OLUMCUL for ad in ("WP_SPEED", "CRUISE_SPEED"))


def test_baglanti_parametreleri_listede_YOK() -> None:
    """SERIAL*_BAUD en ölümcül olan — ama yanlışsa MAVROS hiç bağlanmaz ve bu
    denetim zaten koşamaz. Hat kendi kanıtıdır; listeye koymak anlamsız."""
    assert not any("BAUD" in ad for ad in OLUMCUL)


def test_statustext_50_karakteri_asmiyor() -> None:
    """MAVLink STATUSTEXT sınırı."""
    assert len(ozet(denetle(GERCEK_13_08))) <= 50
    assert ozet([]) == "GIRDAP FC PARAM OK"
