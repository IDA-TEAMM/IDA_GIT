"""
`prototype/telemetry/ariza_bildirici.py` testleri — ROS GEREKTİRMEZ.

Bu nöbetçiler üç şeyi donduruyor (üçü de sahada pahalıya patlayan cinsten):
  1. Telsize çıkan her satır **50 karakteri aşmaz** (MAVLink taşanı sessizce
     keser → operatör yarım kod okur).
  2. Bildirici **telsizi doldurmaz** — aynı arıza sürerken tazeleme
     periyodundan sık göndermez (§10.1: 868 MHz hattı dolunca uçuş
     kontrolcüsü komut kabul etmiyordu).
  3. Seviye sayıları `mavros_msgs/StatusText` ile **aynı** kalır (mavros
     kuruluysa karşılaştırılır; yoksa test gerekçeli atlanır).
"""

from __future__ import annotations

import pytest

from prototype.telemetry.ariza_bildirici import (
    ARIZALAR,
    AZAMI_UZUNLUK,
    ENGEL_BOS,
    ENGEL_YOK,
    KONTROL_HATA,
    ONEK,
    POZ_YOK,
    SEVIYE_CRITICAL,
    SEVIYE_ERROR,
    SEVIYE_NOTICE,
    SEVIYE_WARNING,
    TEMIZ_METNI,
    ArizaBildirici,
)


# ----------------------------------------------------------- kod defteri


def test_her_satir_50_karaktere_sigar() -> None:
    """MAVLink STATUSTEXT sınırı — taşan kısım SESSİZCE kesilir.

    Yeni bir arıza kodu eklenirken en kolay yapılan hata metni uzun
    yazmaktır; kesilme hata vermez, operatör yarım satır görür.
    """
    for tanim in ARIZALAR:
        satir = tanim.statustext()
        assert len(satir) <= AZAMI_UZUNLUK, (
            f"{tanim.kod}: satır {len(satir)} karakter "
            f"(sınır {AZAMI_UZUNLUK}) — {satir!r}"
        )
    assert len(TEMIZ_METNI) <= AZAMI_UZUNLUK


def test_kodlar_benzersiz_ve_onekli() -> None:
    """Aynı kod iki kez tanımlanırsa öncelik sırası anlamsızlaşır."""
    kodlar = [t.kod for t in ARIZALAR]
    assert len(kodlar) == len(set(kodlar)), f"tekrar eden kod: {kodlar}"
    for tanim in ARIZALAR:
        assert tanim.statustext().startswith(f"{ONEK} "), (
            f"{tanim.kod}: satır '{ONEK}' önekiyle başlamıyor — Mission "
            "Planner akışında teknenin mesajı uçuş kontrolcüsününkinden "
            "ayırt edilemez"
        )


def test_satirlar_ascii() -> None:
    """Mission Planner akışında Türkçe harfler bozuk görünebiliyor."""
    for tanim in ARIZALAR:
        satir = tanim.statustext()
        assert satir.isascii(), f"{tanim.kod}: ASCII olmayan karakter: {satir!r}"


def test_seviyeler_gecerli() -> None:
    for tanim in ARIZALAR:
        assert tanim.seviye in (
            SEVIYE_CRITICAL,
            SEVIYE_ERROR,
            SEVIYE_WARNING,
        ), f"{tanim.kod}: beklenmedik seviye {tanim.seviye}"


def test_seviye_sabitleri_mavros_ile_ayni() -> None:
    """Çekirdek mavros'suz koşsun diye sayılar elle yazıldı — ayrışmasınlar."""
    try:
        from mavros_msgs.msg import StatusText
    except ImportError:  # pragma: no cover - ROS'suz çekirdek job'ı
        pytest.skip("mavros_msgs yok (ROS'suz ortam) — karşılaştırma atlandı")

    assert SEVIYE_CRITICAL == StatusText.CRITICAL
    assert SEVIYE_ERROR == StatusText.ERROR
    assert SEVIYE_WARNING == StatusText.WARNING
    assert SEVIYE_NOTICE == StatusText.NOTICE


# ----------------------------------------------------------- gönderim kararı


def test_ariza_yokken_sessiz() -> None:
    """Açılışta hiç arıza yoksa telsize hiçbir şey gitmez."""
    b = ArizaBildirici(tazeleme_s=20.0)
    assert b.gonderilecek(0.0) is None
    assert b.gonderilecek(100.0) is None


def test_ariza_basinca_hemen_gonderilir() -> None:
    b = ArizaBildirici(tazeleme_s=20.0)
    b.bildir(ENGEL_YOK)
    gonderim = b.gonderilecek(0.0)
    assert gonderim is not None
    metin, seviye = gonderim
    assert "ENGEL-YOK" in metin
    assert seviye == SEVIYE_ERROR


def test_ayni_ariza_tazeleme_periyodundan_once_TEKRARLANMAZ() -> None:
    """Telsiz bütçesi — her tick göndermek hattı doldurur (§10.1)."""
    b = ArizaBildirici(tazeleme_s=20.0)
    b.bildir(ENGEL_YOK)
    assert b.gonderilecek(0.0) is not None
    for t in (0.1, 1.0, 5.0, 19.9):
        assert b.gonderilecek(t) is None, f"t={t}: erken tekrar"
    assert b.gonderilecek(20.0) is not None, "tazeleme periyodu doldu, tekrarlamalı"


def test_daha_kritik_ariza_hemen_one_gecer() -> None:
    """Öncelik seviyeye göre; aynı anda iki arıza varsa kritik olan görünür."""
    b = ArizaBildirici(tazeleme_s=20.0)
    b.bildir(ENGEL_BOS)                       # WARNING
    ilk = b.gonderilecek(0.0)
    assert ilk is not None and "ENGEL-BOS" in ilk[0]

    b.bildir(KONTROL_HATA)                    # CRITICAL — beklemeden geçmeli
    ikinci = b.gonderilecek(0.5)
    assert ikinci is not None, "kritik arıza tazeleme periyodunu beklememeli"
    assert "KONTROL" in ikinci[0]
    assert ikinci[1] == SEVIYE_CRITICAL


def test_kritik_dusunce_alttaki_ariza_gorunur() -> None:
    """Üstteki arıza temizlenince operatör alttakini görmeli — susmamalı."""
    b = ArizaBildirici(tazeleme_s=20.0)
    b.bildir(KONTROL_HATA)
    b.bildir(ENGEL_BOS)
    assert b.gonderilecek(0.0) is not None

    b.temizle(KONTROL_HATA)
    gonderim = b.gonderilecek(1.0)
    assert gonderim is not None
    assert "ENGEL-BOS" in gonderim[0]


def test_hepsi_temizlenince_bir_kez_ariza_yok_basilir() -> None:
    """Operatör arızanın DÜŞTÜĞÜNÜ de görmeli; ama tekrar tekrar değil."""
    b = ArizaBildirici(tazeleme_s=20.0)
    b.bildir(POZ_YOK)
    assert b.gonderilecek(0.0) is not None

    b.temizle(POZ_YOK)
    gonderim = b.gonderilecek(1.0)
    assert gonderim is not None
    assert gonderim[0] == TEMIZ_METNI
    assert gonderim[1] == SEVIYE_NOTICE

    # "ariza yok" TEKRARLANMAZ — telsizi boşuna doldurmasın.
    assert b.gonderilecek(100.0) is None
    assert b.gonderilecek(1000.0) is None


def test_ayarla_kosulu_dogrudan_gecirir() -> None:
    b = ArizaBildirici(tazeleme_s=0.0)
    b.ayarla(ENGEL_YOK, aktif=True)
    assert b.aktif_kodlar == frozenset({ENGEL_YOK.kod})
    b.ayarla(ENGEL_YOK, aktif=False)
    assert b.aktif_kodlar == frozenset()


def test_tazeleme_kapaliyken_yalniz_degisimde_gonderilir() -> None:
    b = ArizaBildirici(tazeleme_s=0.0)
    b.bildir(ENGEL_YOK)
    assert b.gonderilecek(0.0) is not None
    assert b.gonderilecek(10_000.0) is None       # tazeleme kapalı


def test_metin_azami_uzunlukta_kesilir() -> None:
    """Sınır aşılırsa bile telsize taşan satır çıkmaz (savunma katmanı)."""
    b = ArizaBildirici(tazeleme_s=20.0, azami_uzunluk=10)
    b.bildir(ENGEL_YOK)
    gonderim = b.gonderilecek(0.0)
    assert gonderim is not None
    assert len(gonderim[0]) == 10


def test_negatif_tazeleme_reddedilir() -> None:
    with pytest.raises(ValueError):
        ArizaBildirici(tazeleme_s=-1.0)
