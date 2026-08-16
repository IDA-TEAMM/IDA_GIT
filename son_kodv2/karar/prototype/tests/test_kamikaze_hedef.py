"""Parkur-3 hedef renk mekanizması testleri — 12'lik madde #4, md 5.5.3.1.

Belgenin kapatma ölçütü iki yarım: (a) parametre var, (b) o renk `class_id=2`
(hedef) olarak işaretleniyor. Buradaki testler (b)'yi ve aktarım zamanlaması
kuralını donduruyor. ROS'suz + cv2'siz koşar.
"""

from __future__ import annotations

from dataclasses import dataclass

import pytest

from prototype.mission.kamikaze_hedef import (
    CLASS_ENGEL,
    CLASS_HEDEF,
    CLASS_KAHVERENGI,
    CLASS_KIRMIZI,
    CLASS_PARKUR_KENARI,
    CLASS_YESIL,
    DEGISTIRILEBILIR_DURUMLAR,
    RENK_SINIFLARI,
    SECILEBILIR_SINIFLAR,
    HedefRengiHatasi,
    degistirilebilir_mi,
    hedef_isaretle,
    renk_to_class,
)


@dataclass
class _T:
    """`camera_buoys.Detection`in test ikizi (yalnız class_id gerekli)."""

    class_id: int


# --------------------------------------------------------------- renk → sınıf


def test_hakemin_soyleyebilecegi_renkler_cozuluyor() -> None:
    assert renk_to_class("kirmizi") == CLASS_KIRMIZI
    assert renk_to_class("kırmızı") == CLASS_KIRMIZI      # Türkçe karakterli
    assert renk_to_class("  KIRMIZI  ") == CLASS_KIRMIZI  # boşluk + büyük harf
    assert renk_to_class("red") == CLASS_KIRMIZI
    assert renk_to_class("yesil") == CLASS_YESIL
    assert renk_to_class("kahverengi") == CLASS_KAHVERENGI


def test_bos_deger_HEDEF_ATANMAMIS_demek() -> None:
    """Varsayılan hâl hareketsiz olmalı: hakem konuşmadan hiçbir şey değişmez."""
    assert renk_to_class(None) is None
    assert renk_to_class("") is None
    assert renk_to_class("   ") is None


def test_bilinmeyen_renk_SESSIZCE_gecmiyor() -> None:
    """Yazım hatası Parkur-3'ü sessizce kaybettirmemeli — açık hata versin."""
    with pytest.raises(HedefRengiHatasi) as e:
        renk_to_class("mor")
    # Operatöre ne yazabileceği söylenmeli
    assert "kirmizi" in str(e.value) and "yesil" in str(e.value)


def test_TURUNCU_hedef_olarak_REDDEDILIYOR() -> None:
    """🔴 Turuncu = kenar dubası. Hedefe taşınırsa kapılar kaybolur VE tekne
    kapı dubalarına sürer (gate_follower `edge_buoy_class_id`'ye bakıyor).
    """
    with pytest.raises(HedefRengiHatasi) as e:
        renk_to_class("turuncu")
    assert "kenar" in str(e.value).lower()


def test_SARI_hedef_olarak_REDDEDILIYOR() -> None:
    """Sarı = engel; engele nişan almak istemiyoruz."""
    with pytest.raises(HedefRengiHatasi) as e:
        renk_to_class("sari")
    assert "engel" in str(e.value).lower()


def test_yasak_renkler_TANINIYOR_ama_secilemiyor() -> None:
    """Bilinmeyen renk ile yasak renk AYRI hatalar — mesajları da ayrı."""
    assert RENK_SINIFLARI["turuncu"] == CLASS_PARKUR_KENARI
    assert RENK_SINIFLARI["sari"] == CLASS_ENGEL
    assert CLASS_PARKUR_KENARI not in SECILEBILIR_SINIFLAR
    assert CLASS_ENGEL not in SECILEBILIR_SINIFLAR
    assert CLASS_HEDEF not in SECILEBILIR_SINIFLAR   # hedefi hedefe taşıma


# ------------------------------------------------------------- yeniden etiketleme


def test_secilen_renk_CLASS_HEDEF_e_tasiniyor() -> None:
    """Belgenin (b) yarısı: o renk sınıfı `class_id=2` olarak işaretlenir."""
    tespitler = [
        _T(CLASS_PARKUR_KENARI), _T(CLASS_KIRMIZI), _T(CLASS_ENGEL),
        _T(CLASS_KIRMIZI), _T(CLASS_YESIL),
    ]
    n = hedef_isaretle(tespitler, renk_to_class("kirmizi"))
    assert n == 2
    assert [t.class_id for t in tespitler] == [
        CLASS_PARKUR_KENARI, CLASS_HEDEF, CLASS_ENGEL, CLASS_HEDEF, CLASS_YESIL
    ]


def test_diger_siniflara_DOKUNULMUYOR() -> None:
    """Kenar (0) ve engel (1) korunmalı — Parkur-1/2 onlara bağlı."""
    tespitler = [_T(CLASS_PARKUR_KENARI), _T(CLASS_ENGEL), _T(CLASS_KAHVERENGI)]
    hedef_isaretle(tespitler, renk_to_class("yesil"))   # yeşil YOK sahnede
    assert [t.class_id for t in tespitler] == [
        CLASS_PARKUR_KENARI, CLASS_ENGEL, CLASS_KAHVERENGI
    ]


def test_hedef_atanmamissa_hicbir_sey_degismez() -> None:
    tespitler = [_T(CLASS_KIRMIZI), _T(CLASS_YESIL)]
    assert hedef_isaretle(tespitler, None) == 0
    assert [t.class_id for t in tespitler] == [CLASS_KIRMIZI, CLASS_YESIL]


def test_bos_sahne_cokmez() -> None:
    assert hedef_isaretle([], renk_to_class("kirmizi")) == 0


def test_yasak_sinif_dogrudan_verilirse_de_reddediliyor() -> None:
    """`renk_to_class`ı atlayıp doğrudan sınıf verilse bile kapı kapalı."""
    with pytest.raises(HedefRengiHatasi):
        hedef_isaretle([_T(CLASS_PARKUR_KENARI)], CLASS_PARKUR_KENARI)


# ------------------------------------------------- md 5.5.3.1 aktarım zamanı


def test_hareket_ONCESI_degistirilebilir() -> None:
    for d in ("BOOT", "ARM", "BEKLEMEDE"):
        ok, _ = degistirilebilir_mi(d)
        assert ok, d


def test_HAREKET_BASLADIKTAN_SONRA_REDDEDILIYOR() -> None:
    """🔴 md 5.5.3.1'in yasakladığı tam durum. Kod bunu imkânsız kılmalı —
    operatörün "dokunmamayı hatırlamasına" bırakılmamalı.
    """
    for d in ("PARKUR1", "PARKUR2", "PARKUR3"):
        ok, neden = degistirilebilir_mi(d)
        assert not ok, d
        assert "5.5.3.1" in neden


def test_KILL_ve_TAMAMLANDI_izinli_yeniden_baslama_hakki() -> None:
    """md 5.5.3.1 yeniden başlama hakkı: hareket bittikten sonra renk
    yeniden verilebilmeli (bkz. `/girdap/mission/reset`).
    """
    for d in ("KILL", "TAMAMLANDI"):
        ok, _ = degistirilebilir_mi(d)
        assert ok, d


def test_durum_bilinmiyorsa_izin_veriliyor() -> None:
    """FSM henüz yayın yapmadıysa hareket de başlamamıştır. Tersi (bilinmiyorken
    reddetmek) koşu sabahı ayar yapılmasını engellerdi.
    """
    ok, neden = degistirilebilir_mi(None)
    assert ok and "bilinmiyor" in neden


def test_durum_adi_bosluk_kucuk_harf_toleransli() -> None:
    assert degistirilebilir_mi("  beklemede ")[0] is True
    assert degistirilebilir_mi("  parkur1 ")[0] is False


# --------------------------------------------------- SÜRÜKLENME KAPILARI (CI)


def test_sinif_sozlesmesi_camera_buoys_ILE_AYNI() -> None:
    """`kamikaze_hedef` sınıf sabitlerini bilerek KOPYALIYOR (cv2 çekmemek
    için). İki taraf ayrışırsa hedef yanlış sınıfa taşınır ve bu sessiz olur —
    bu test o sürüklenmeyi CI'da kırmızıya çevirir.

    cv2 yoksa dürüst skip: kopya yine de `camera_buoys`ın sözleşmesi.
    """
    cb = pytest.importorskip(
        "prototype.perception.camera_buoys",
        reason="cv2 yok — sınıf sözleşmesi karşılaştırması atlanıyor",
    )
    assert CLASS_PARKUR_KENARI == cb.CLASS_PARKUR_KENARI
    assert CLASS_ENGEL == cb.CLASS_ENGEL
    assert CLASS_HEDEF == cb.CLASS_HEDEF
    assert CLASS_KIRMIZI == cb.CLASS_KIRMIZI
    assert CLASS_YESIL == cb.CLASS_YESIL
    assert CLASS_KAHVERENGI == cb.CLASS_KAHVERENGI


def test_durum_adlari_MissionState_ILE_AYNI() -> None:
    """`DEGISTIRILEBILIR_DURUMLAR` string tutuyor (FSM'i import etmemek için).
    FSM durum adları değişirse kapı sessizce yanlış tarafa açılır.
    """
    from prototype.fsm.mission_fsm import MissionState

    tum = {s.value for s in MissionState}
    assert DEGISTIRILEBILIR_DURUMLAR <= tum, (
        f"FSM'de olmayan durum adi: {DEGISTIRILEBILIR_DURUMLAR - tum}"
    )
    # Hareket hâlindeki üç parkur durumu izinli listede OLMAMALI
    assert {"PARKUR1", "PARKUR2", "PARKUR3"}.isdisjoint(
        DEGISTIRILEBILIR_DURUMLAR
    )
    # Ve FSM'in her durumu ya izinli ya yasak sayılmış olmalı (yeni bir durum
    # eklenirse burada fark edilsin)
    beklenen_yasak = tum - DEGISTIRILEBILIR_DURUMLAR
    assert beklenen_yasak == {"PARKUR1", "PARKUR2", "PARKUR3"}, (
        f"FSM'e yeni durum eklenmis: {beklenen_yasak}. Hedef rengi o durumda "
        f"degistirilebilir mi? DEGISTIRILEBILIR_DURUMLAR'i gozden gecir."
    )
