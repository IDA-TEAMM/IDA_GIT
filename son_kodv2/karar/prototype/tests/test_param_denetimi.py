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
    "GPS1_POS_X": -0.035, "GPS1_POS_Y": 0.16, "GPS1_POS_Z": -0.365,
    "MOT_THR_MIN": 0.0,
    "FS_ACTION": 0.0, "ARMING_CHECK": 1.0, "BATT_MONITOR": 0.0,
    "FRAME_CLASS": 2.0, "SERVO1_FUNCTION": 74.0, "SERVO3_FUNCTION": 73.0,
    "BATT_VOLT_MULT": 5.091626, "SERIAL2_BAUD": 921.0, "ARMING_REQUIRE": 1.0,
    "SR2_EXT_STAT": 10.0, "SR2_EXTRA1": 10.0, "SR2_POSITION": 10.0,
}

DOGRU = {ad: b.deger for ad, b in OLUMCUL.items()}


def test_13_agustos_gercek_sapmasi_YAKALANIYOR() -> None:
    """O günün gerçek Pix durumu → tam 6 ölümcül bulgu.

    ⚠ 16.08: bulgu kümesinden `BATT_MONITOR` DÜŞTÜ — sayı 7'den 6'ya indi.
    Sebep fixture değil BEKLENTİ: kaptan kararıyla batarya izleme kapatıldı
    (PM06 canlıda 0,007 V / 0,01 A okuyor, `BATT_MONITOR=3` iken
    `PreArm: Battery 1 unhealthy` arm'ı engelliyordu). 13.08'in gerçek
    değeri zaten 0'dı; artık beklenen de 0 → sapma yok. Batarya izleme
    yeniden açılırsa beklenti 3'e döner ve bu küme tekrar 7 olur.

    ⚠ 13.08 akşamı liste 9'dan 13'e çıkarıldı: `GPS1_POS_X/Y/Z` ve
    `MOT_THR_MIN` eksikti. Eksik olmalarının bedeli somut — bu dördü ölümcül
    listede olmadığı için REFERANS DOSYASINA bozuk dökümden dondular
    (`GPS1_POS_Y=+0.16` işareti ters, `MOT_THR_MIN=0`). Yani denetleyici
    onları hem yakalamıyor hem de yanlış değeri "doğru" diye saklıyordu.
    🔑 Ders: ölümcül liste yalnız uyarı üretmiyor, REFERANSIN NEYİ KORUYACAĞINI
    da belirliyor — listeye almadığın her değer bozuk hâliyle donabilir.
    """
    b = denetle(GERCEK_13_08)
    adlar = {x.ad for x in b}
    assert adlar == {
        "INS_POS1_X", "INS_POS1_Y", "INS_POS1_Z", "GPS1_POS_Y",
        "MOT_THR_MIN", "FS_ACTION",
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


def test_TELEM2_baud_listede_TELEM1_DEGIL() -> None:
    """🔑 14.08'de gerekçe DEĞİŞTİ, çünkü denetleyici yer değiştirdi.

    Önce şöyle yazmıştım: *"`SERIAL*_BAUD` listeye girmez; yanlışsa MAVROS
    hiç bağlanmaz ve denetim zaten koşamaz — hat kendi kanıtıdır."* O gerekçe
    **düğüm tarafındaki** denetleyici içindi (MAVROS'tan `param/get` okuyordu).

    Denetleyici artık LAPTOPTA ve `.param` DOSYASI okuyor; dosyayı Mission
    Planner'dan **telemetri radyosu** (SERIAL1) üzerinden alıyorsun. Yani
    `SERIAL2_BAUD` (TELEM2 = Jetson hattı) bozukken bile denetim çalışır —
    ve o bozukken MAVROS hiç bağlanmadığı için otonominin tamamı ölür,
    üstelik Mission Planner sorunsuz göründüğü için fark edilmez.
    ⇒ TELEM2 listede OLMALI.

    SERIAL1 (telemetri radyosu) listede DEĞİL: o bozuksa dökümü zaten
    alamazsın, yani kendi kanıtıdır.
    """
    assert "SERIAL2_BAUD" in OLUMCUL, "TELEM2 baud'u izlenmeli"
    assert "SERIAL1_BAUD" not in OLUMCUL, "SERIAL1 kendi kanitidir"


def test_statustext_50_karakteri_asmiyor() -> None:
    """MAVLink STATUSTEXT sınırı."""
    assert len(ozet(denetle(GERCEK_13_08))) <= 50
    assert ozet([]) == "GIRDAP FC PARAM OK"
