"""Parkur-3 sayısal renk kodu sözleşmesi — ROS gerekmez.

🔴 Bu tablo İKİ REPODA yaşıyor (burada + `girdap-iha-plaka/plaka/cikis.py`).
Ayrışırsa İHA "3" der, İDA başka rengi avlar ve bu SESSİZ olur ⇒ yanlış hedefe
angajman (TS3: 1 yanlış temas 100→50, 2 yanlış 100→5).
"""
from __future__ import annotations

import pytest

from prototype.mission.kamikaze_hedef import HedefRengiHatasi
from prototype.mission.renk_kodu import KOD_RENK, kod_to_renk, renk_to_kod


def test_kod_tablosu_IHA_tarafiyla_AYNI() -> None:
    """İHA `cikis.py`: {None:0, kirmizi:1, yesil:2, siyah:3} — birebir aynı olmalı."""
    assert KOD_RENK == {0: None, 1: "kirmizi", 2: "yesil", 3: "siyah"}


@pytest.mark.parametrize("kod,beklenen", [
    (0, None), (1, "kirmizi"), (2, "yesil"), (3, "siyah"),
])
def test_kod_renge_cevriliyor(kod, beklenen) -> None:
    assert kod_to_renk(kod) == beklenen


def test_UCUS_KONTROLCUSU_float_verir_kabul_edilir() -> None:
    """`SCR_USER1` bir FLOAT parametredir: 3.0 gelir, 3 değil."""
    assert kod_to_renk(3.0) == "siyah"
    assert kod_to_renk(1.0) == "kirmizi"
    assert kod_to_renk(0.0) is None


def test_bilinmeyen_kod_HATA_verir_sessizce_hedefsiz_kalmaz() -> None:
    """🔴 Sessizce 'hedef yok'a düşmek, operatörün yanlış değer girdiğini
    GİZLERDİ — yanlış girildiği ANLAŞILMALI."""
    for kot in (4, -1, 99):
        with pytest.raises(HedefRengiHatasi):
            kod_to_renk(kot)


def test_ONDALIK_kod_HATA_verir() -> None:
    """2.5 bir renk değil, yazım hatası — yuvarlayıp 'yeşil' demek tehlikeli."""
    with pytest.raises(HedefRengiHatasi):
        kod_to_renk(2.5)


def test_sayi_olmayan_HATA_verir() -> None:
    with pytest.raises(HedefRengiHatasi):
        kod_to_renk("siyah")          # ad değil KOD bekleniyor
    with pytest.raises(HedefRengiHatasi):
        kod_to_renk(None)


@pytest.mark.parametrize("ad,kod", [
    ("kirmizi", 1), ("KIRMIZI", 1), ("red", 1),
    ("yesil", 2), ("YEŞİL", 2), ("green", 2),
    ("siyah", 3), ("SİYAH", 3), ("black", 3),
    ("", 0), (None, 0),
])
def test_ad_koda_cevriliyor_TURKCE_BUYUK_HARF_dahil(ad, kod) -> None:
    """Türkçe İ tuzağı burada da çözülmüş olmalı (`_anahtarla` paylaşılıyor)."""
    assert renk_to_kod(ad) == kod


def test_gidis_donus_tutarli() -> None:
    for kod, ad in KOD_RENK.items():
        assert renk_to_kod(ad) == kod


def test_TURUNCU_ve_SARI_koda_cevrilemez() -> None:
    """Hedef olarak seçilemezler (turuncu=kapı dubası, sarı=engel)."""
    for yasak in ("turuncu", "sari", "sarı", "orange", "yellow"):
        with pytest.raises(HedefRengiHatasi):
            renk_to_kod(yasak)


# ───────── 13.08 kusur avı: geçici hata sonrası YENİDEN DENEME ─────────
from prototype.mission.renk_kodu import RenkUygulamaDurumu    # noqa: E402


def test_GECICI_HATA_sonrasi_YENIDEN_DENENIR() -> None:
    """🔴🔴 BULUNAN KUSURUN BEKÇİSİ.

    Köprünün ilk hâli okunan kodu UYGULAMADAN ÖNCE önbelleğe yazıyordu. Hedef
    node'un parametre servisi o anda hazır değilse (açılışta çok muhtemel),
    uygulama atlanıyor ve bir sonraki yoklamada kod "değişmedi" görünüp erken
    dönülüyordu ⇒ **renk bir daha HİÇ uygulanmıyordu** — belirtisiz, logsuz,
    Parkur-3 sessizce sıfır.
    """
    d = RenkUygulamaDurumu()

    renk, yeni = d.kod_geldi(3.0)
    assert (renk, yeni) == ("siyah", True)
    # ... uygulama BAŞARISIZ oldu (servis hazır değil) → uygulandi() ÇAĞRILMADI
    assert d.bekleyen == "siyah"

    # aynı kod tekrar okundu: log için "yeni değil" ama HÂLÂ uygulanmalı
    renk2, yeni2 = d.kod_geldi(3.0)
    assert renk2 == "siyah", "geçici hatadan sonra yeniden denenmiyor!"
    assert yeni2 is False, "aynı kod tekrar tekrar loglanmamalı"
    assert d.bekleyen == "siyah"


def test_uygulandiktan_SONRA_tekrar_denenmez() -> None:
    d = RenkUygulamaDurumu()
    d.kod_geldi(2.0)
    d.uygulandi("yesil")
    assert d.bekleyen is None
    assert d.kod_geldi(2.0) == (None, False)      # iş yok
    assert d.uygulanan == "yesil"


def test_kod_DEGISIRSE_yeni_renk_uygulanir() -> None:
    """Operatör yanlış yazıp düzeltirse yeni değer geçmeli."""
    d = RenkUygulamaDurumu()
    d.kod_geldi(1.0)
    d.uygulandi("kirmizi")
    renk, yeni = d.kod_geldi(3.0)
    assert (renk, yeni) == ("siyah", True)
    assert d.bekleyen == "siyah"


def test_kod_0_beklemeyi_TEMIZLER() -> None:
    """Operatör değeri sıfırlarsa bekleyen uygulama düşer (hedef atanmamış)."""
    d = RenkUygulamaDurumu()
    d.kod_geldi(3.0)
    assert d.bekleyen == "siyah"
    renk, _ = d.kod_geldi(0.0)
    assert renk is None and d.bekleyen is None


def test_gecersiz_kod_beklemeyi_BOZMAZ() -> None:
    """Geçersiz kod hata fırlatır; önceden bekleyen renk kaybolmamalı."""
    d = RenkUygulamaDurumu()
    d.kod_geldi(3.0)
    with pytest.raises(HedefRengiHatasi):
        d.kod_geldi(7.0)
    assert d.bekleyen == "siyah", "geçersiz kod bekleyen rengi düşürdü"
