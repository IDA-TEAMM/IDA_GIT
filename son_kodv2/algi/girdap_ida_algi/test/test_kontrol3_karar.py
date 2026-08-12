"""KONTROL 3'ün KARAR mantığı — donanımsız koşar.

🔴 NEDEN: `kontrol3_kanal_sirasi.py`'nin kendisi ancak kamerayla koşabilir, ama
**yanlış karar verme riski** kameradan bağımsız. Bu betiğin çıktısı "512
dağıtılsın mı" sorusunun cevabı olacak; eşik/sıra hatası ya sahte alarm üretir
ya da **gerçek arızayı maskeler**.

Tanı kodu da koddur (09.08 dersi: `durum_log`'daki tanı satırı node'u öldürmüştü).
"""
import importlib.util
import os

import pytest

_KOK = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
_BETIK = os.path.join(_KOK, "scripts", "kontrol3_kanal_sirasi.py")


@pytest.fixture(scope="module")
def k3():
    """Betiği modül olarak yükle — depthai/kamera GEREKMEZ (karar saf fonksiyon)."""
    spec = importlib.util.spec_from_file_location("kontrol3", _BETIK)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def test_temiz_gecis(k3):
    """Normal taraf görüyor, takas tarafı görmüyor = blob DOĞRU."""
    kod, _ = k3._karar(normal=20, ters=0)
    assert kod == k3.GECTI


def test_kucuk_takas_sizintisi_gecer(k3):
    """Takas tarafında birkaç tespit olması normal (gürültü) — kapı %25."""
    kod, _ = k3._karar(normal=20, ters=4)
    assert kod == k3.GECTI


def test_takas_esigi_asilinca_kalir(k3):
    kod, mesaj = k3._karar(normal=20, ters=6)
    assert kod == k3.KALDI
    assert "reverse_input_channels" in mesaj


def test_kadrajda_duba_yoksa_KARARSIZ_kalir_KALDI_demez(k3):
    """🪤 Kuru ortam tuzağı: tespit yokluğu BAŞARISIZLIK sayılmamalı.

    'Modeli kuru ortamda test etmek anlamsız' (10.08): suda %95,98, masada
    kararsız. Burada KALDI denseydi sağlam bir blob sahada reddedilirdi.
    """
    for normal in (0, 1, 2):
        kod, mesaj = k3._karar(normal=normal, ters=0)
        assert kod == k3.KARARSIZ, f"normal={normal}"
        assert "TEKRAR KOŞ" in mesaj


def test_ters_kanal_az_tespitle_bile_KALDI_der(k3):
    """🔴 REGRESYON: normal az + takas çok = kanallar TERS, KARARSIZ DEĞİL.

    İlk sürümde `normal < MIN_TESPIT` kontrolü önce geliyordu ⇒ bu tablo
    'KARARSIZ, dubayı göster tekrar koş' diye okunuyordu. Operatör aynı sonucu
    alıp durur, **gerçek arıza maskelenirdi** — üstelik tam da tespit etmek
    istediğimiz arıza (recall %96,8 → %43, hata basılmadan).
    """
    kod, mesaj = k3._karar(normal=1, ters=9)
    assert kod == k3.KALDI
    assert "TERS" in mesaj


def test_ikisi_de_sifir_KARARSIZ(k3):
    kod, _ = k3._karar(normal=0, ters=0)
    assert kod == k3.KARARSIZ


def test_cikis_kodlari_ayrik(k3):
    """Kabuk betiği bu üç kodu ayırt edebilmeli (0 = geçti sözleşmesi)."""
    assert k3.GECTI == 0
    assert len({k3.GECTI, k3.KALDI, k3.KARARSIZ}) == 3
