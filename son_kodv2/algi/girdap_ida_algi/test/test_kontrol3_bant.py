"""KONTROL 3'ün `--bant` kipi — kamera GEREKMEZ, donanım GEREKMEZ.

🔴 NEDEN (17.08.2026, pahalıya öğrenildi): KONTROL 3 kadrajda gerçek duba
istiyordu. O şart yüzünden kapı 13.08'den 17.08'e kadar **hiç koşulmadı** ve
16.08'de dağıtılan bozuk blob (`6df2d644`, `--scale_values` düşmüş) teknede üç
oturum kaldı. 17.08'de ölçüldü: **o blob bu testten KALIRDI** (takas/normal
0,862). Kapı vardı, çalışıyordu — yalnız açılamıyordu.

Burada kilitlenen iki şey:
  1. `_kayit_bgr_mi` — ölçüm ARACININ kendisini doğrulayan kapı. Klasör RGB
     kaydedilmişse turuncu duba mavi okunur, normal taraf çöker ve betik
     **doğru blob'a "ters" damgası vurur**. Bu, tam olarak avladığımız sınıftan
     bir sessiz katil.
  2. `_bant_kareleri_yukle` — dağıtımdaki `keepAspectRatio(False)` ile aynı,
     yani SIKIŞTIRMA. Letterbox'a kayılırsa ölçülen kare, dağıtımın gördüğü
     kare olmaktan çıkar (§5: aynı model aynı video, kırp+sıkıştır 480 tespit ↔
     letterbox 338).

Sentetik dizi kullanmak burada kuralı çiğnemez: ölçülen şey **modelin
davranışı değil**, saf bir renk-istatistiği fonksiyonu.
"""
import importlib.util
import os
import sys

import numpy as np
import pytest

_KOK = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
_BETIK = os.path.join(_KOK, "scripts", "kontrol3_kanal_sirasi.py")

cv2 = pytest.importorskip("cv2", reason="opencv kurulu değil")


@pytest.fixture(scope="module")
def k3():
    spec = importlib.util.spec_from_file_location("kontrol3_bant", _BETIK)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def _duba_karesi(bgr) -> np.ndarray:
    """Ortasında doygun renk lekesi olan kare (gerisi nötr gri = doygunluk 0)."""
    kare = np.full((64, 64, 3), 128, dtype=np.uint8)
    kare[20:44, 20:44] = bgr
    return kare


# RAL 2003 turuncu ve RAL 1026 sarının kabaca BGR karşılıkları.
_TURUNCU_BGR = (20, 110, 245)
_SARI_BGR = (30, 220, 250)


def test_bgr_kaydedilmis_klasor_KABUL_EDILIR(k3):
    kareler = [_duba_karesi(_TURUNCU_BGR), _duba_karesi(_SARI_BGR)]
    bgr_mi, oran = k3._kayit_bgr_mi(kareler)
    assert bgr_mi is True
    assert oran > 0.5


def test_RGB_kaydedilmis_klasor_REDDEDILIR(k3):
    """Aynı dubalar, yanlış düzende kaydedilmiş — kapı bunu yakalamalı.

    Yakalamazsa: normal taraf çöker, takas tarafı yükselir ve betik DOĞRU
    blob'a 'kanallar ters' der. Bu testin kırmızıya dönmesi = o senaryonun
    geri gelmesi.
    """
    kareler = [_duba_karesi(_TURUNCU_BGR[::-1]), _duba_karesi(_SARI_BGR[::-1])]
    bgr_mi, oran = k3._kayit_bgr_mi(kareler)
    assert bgr_mi is False
    assert oran < 0.5


def test_doygun_renk_YOKSA_karar_verilmez(k3):
    """Gri set: 'BGR' DEMEZ — çağıran KARARSIZ döner, 'GEÇTİ' değil."""
    kareler = [np.full((64, 64, 3), 128, dtype=np.uint8)]
    bgr_mi, oran = k3._kayit_bgr_mi(kareler)
    assert oran == 0.0
    assert bgr_mi is False


def test_kareler_SIKISTIRILIR_letterbox_YOK(k3, tmp_path):
    """4:3 kare kare NN girişine sıkıştırılmalı — şerit (letterbox) OLMAMALI."""
    from girdap_ida_algi import duba_gecis_navigator as dgn

    # Dağıtımdaki RGB çıkışıyla aynı en-boy: 1352x1014.
    kare = np.zeros((1014, 1352, 3), dtype=np.uint8)
    kare[:, :] = _TURUNCU_BGR
    cv2.imwrite(str(tmp_path / "kare_0001.jpg"), kare)

    secilen, kareler = k3._bant_kareleri_yukle(str(tmp_path), 1)
    assert len(kareler) == 1
    assert kareler[0].shape == (dgn.NN_GIRIS, dgn.NN_GIRIS, 3)
    # Letterbox olsaydı üst/alt şeritler SİYAH kalırdı; sıkıştırmada kalmaz.
    assert kareler[0][0, :, :].max() > 0, "üst şerit siyah — letterbox'a kayılmış"
    assert kareler[0][-1, :, :].max() > 0, "alt şerit siyah — letterbox'a kayılmış"


def test_bos_dizin_SESSIZ_GECMEZ(k3, tmp_path):
    with pytest.raises(SystemExit):
        k3._bant_kareleri_yukle(str(tmp_path), 8)


def test_bant_ve_blob_secenekleri_DURUYOR(k3):
    """Sözleşme: seçenekler kaybolursa kapı yine donanıma bağımlı hâle gelir."""
    kaynak = open(_BETIK, encoding="utf-8").read()
    assert '"--bant"' in kaynak
    assert '"--blob"' in kaynak
    assert "BANT_DIZIN" in kaynak


def test_karar_esikleri_DEGISMEDI(k3):
    """`--bant` karar mantığını DEĞİŞTİRMEZ — aynı `_karar`, aynı eşikler."""
    assert k3.TAKAS_TOLERANS == 0.25
    assert k3.MIN_TESPIT == 3
    # 17.08 ölçümü: yeni blob 306/71, bozuk blob 73604/63444.
    assert k3._karar(306, 71)[0] == k3.GECTI
    assert k3._karar(73604, 63444)[0] == k3.KALDI
