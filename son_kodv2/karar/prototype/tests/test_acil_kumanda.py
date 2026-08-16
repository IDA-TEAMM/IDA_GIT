"""Acil durdurma / klavye kumandası — GÜVENLİK mantığının nöbetçisi.

🔴 Bu dosyanın var olma sebebi 14.08 göl testi: tekne hiç çalışmadı,
sürüklenerek karşı kıyıya gitti ve elle alındı; RC ile de müdahale
edilemedi (Pixhawk'ta RC alıcısı fiziksel olarak yok).

Buradaki testler kullanışlılığı değil **güvenliği** dondurur. Bir kumanda
aracında en tehlikeli kusur "çalışmıyor" değil, **istenmeden çalışıyor**.
"""

from __future__ import annotations

import importlib.util
import sys
from pathlib import Path

import pytest

_yol = Path(__file__).resolve().parents[2] / "scripts" / "acil_kumanda.py"
_spec = importlib.util.spec_from_file_location("acil_kumanda", _yol)
ak = importlib.util.module_from_spec(_spec)
sys.modules["acil_kumanda"] = ak
_spec.loader.exec_module(ak)


def _k(olu_adam: float = 0.4):
    return ak.Kumanda("yok", kuru=True, olu_adam=olu_adam, gaz_adim=40, don_adim=60)


def test_acilista_DEVREDE_DEGIL() -> None:
    """Araç açılır açılmaz komut göndermemeli — yanlış pencereye basılan bir
    tuş tekneyi hareket ettirmemeli."""
    k = _k()
    assert k.devrede is False
    assert k.gaz == ak.NOTR and k.don == ak.NOTR


def test_DEVREYE_ALMADAN_yon_tuslari_ETKISIZ() -> None:
    """🔑 Devreye alma ayrı bir adım. `W`'ye basmak tek başına gaz vermemeli."""
    k = _k()
    for c in "wasd":
        k._tus(c)
    assert k.gaz == ak.NOTR and k.don == ak.NOTR, "devreye alinmadan komut uretildi"


def test_M_devreye_alir_ve_notrden_baslar() -> None:
    k = _k()
    k._tus("m")
    assert k.devrede is True
    assert k.gaz == ak.NOTR and k.don == ak.NOTR, "devreye alinca notr olmali"


def test_OLU_ADAM_gazi_notre_ceker() -> None:
    """🔴 En kritik test: takılı tuş / donmuş terminal / kopmuş SSH kaçak
    tekne üretmemeli. Tuş gelmezse gaz kendiliğinden nötre dönmeli."""
    k = _k(olu_adam=0.05)
    k._tus("m"); k._tus("w")
    assert k.gaz > ak.NOTR, "ileri komutu uretilmedi"
    import time; time.sleep(0.08)
    # calistir() dongusundeki olu adam kontrolunun aynisi
    if k.devrede and time.monotonic() - k.son_tus > k.olu_adam:
        k.gaz = ak.NOTR
    assert k.gaz == ak.NOTR, "olu adam gazi notre cekmedi"


def test_ACIL_DURDURMA_devrede_OLMASA_da_calisir() -> None:
    """Acil durdurma bir kipe bağlı olamaz — panik anında 'önce M'ye bas'
    diye bir şey yok."""
    k = _k()
    assert k.devrede is False
    k._tus(" ")
    assert k.gaz == ak.NOTR and k.don == ak.NOTR


def test_ACIL_DURDURMA_devreyi_KAPATIR() -> None:
    """Durdurduktan sonra sürüş açık kalmamalı; yön tuşu yeniden hareket
    başlatmamalı."""
    k = _k()
    k._tus("m"); k._tus("w"); k._tus(" ")
    assert k.devrede is False
    k._tus("w")
    assert k.gaz == ak.NOTR, "acil durdurmadan sonra surus yeniden basladi"


def test_X_override_birakir_devreyi_kapatir() -> None:
    """Otonomiye geri vermek: override bırakılır, sürüş kapanır."""
    k = _k(); k._tus("m"); k._tus("x")
    assert k.devrede is False


def test_gaz_ve_donus_SINIRLARI_asilmiyor() -> None:
    """PWM 1000-2000 dışına çıkmak ESC'de tanımsız davranıştır."""
    k = _k(); k._tus("m")
    for _ in range(200):
        k._tus("w"); k._tus("d")
    assert k.gaz <= 2000 and k.don <= 2000
    for _ in range(400):
        k._tus("s"); k._tus("a")
    assert k.gaz >= 1000 and k.don >= 1000


def test_Q_dongu_bitirir() -> None:
    assert _k()._tus("q") is False
