"""KONTROL 3'ün DONANIM yolu — `pipeline_kur()` sözleşmesi. Kamera GEREKMEZ.

🔴 NEDEN (12.08, Jetson'da ölçüldü): `kontrol3_kanal_sirasi.py` cihazda
koşturulunca daha ilk adımda `TypeError` ile düştü — `pipeline_kur()`'un dönüşü
bir `dai.Pipeline` sanılıp `dai.Device(...)` ile İKİNCİ kez açılmaya
çalışılıyordu. Oysa `pipeline_kur()` cihazı KENDİSİ açar ve
`(dev, det_q, rgb_q, siniflar)` döner (duba_gecis_navigator:551-555).

🪤 Tuzağın asıl dersi: `test_kontrol3_karar.py`'nin yedi testi de YEŞİLDİ, çünkü
hepsi `_karar()`'ı — saf fonksiyonu — tutuyordu. "Kamerayla koşar" diye test
dışı bırakılan yol, betiğin **hiç koşmamış** olan tek yoluydu. Kamera burada
sahteleniyor; kilitlenen şey görüntü değil, iki modül arasındaki **sözleşme**.
"""
import importlib.util
import os
import sys

import numpy as np
import pytest

_KOK = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
_BETIK = os.path.join(_KOK, "scripts", "kontrol3_kanal_sirasi.py")
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

pytest.importorskip("depthai", reason="depthai kurulu değil")

from girdap_ida_algi import duba_gecis_navigator as dgn  # noqa: E402


@pytest.fixture(scope="module")
def k3():
    spec = importlib.util.spec_from_file_location("kontrol3", _BETIK)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


class _SahteKare:
    def __init__(self, kare):
        self._kare = kare

    def getCvFrame(self):
        return self._kare


class _SahteTespit:
    detections = (object(), object())


class _SahteKuyruk:
    """`tryGet()` çağrıldıkça sırayla verir; bitince None (kamera sustu)."""

    def __init__(self, ogeler):
        self._ogeler = list(ogeler)

    def tryGet(self):
        return self._ogeler.pop(0) if self._ogeler else None


class _SahteCihaz:
    """`with dev:` ile kapanmalı — cihaz açık kalırsa sıradaki açış çakışır."""

    def __init__(self):
        self.kapandi = False

    def __enter__(self):
        return self

    def __exit__(self, *_):
        self.kapandi = True
        return False


def _sahte_kur(monkeypatch, kare_sayisi=3):
    cihaz = _SahteCihaz()
    kareler = [_SahteKare(np.zeros((dgn.NN_GIRIS, dgn.NN_GIRIS, 3), np.uint8))
               for _ in range(kare_sayisi)]
    q_det = _SahteKuyruk([_SahteTespit() for _ in range(kare_sayisi)])
    q_rgb = _SahteKuyruk(kareler)
    monkeypatch.setattr(dgn, "pipeline_kur",
                        lambda: (cihaz, q_det, q_rgb, ["kenar_dubasi", "engel_dubasi"]))
    return cihaz


def test_pipeline_kur_dorttuslu_donusu_dogru_acilir(k3, monkeypatch):
    """🔴 REGRESYON: dönüş `(dev, det_q, rgb_q, siniflar)` olarak AÇILMALI.

    Eskiden tek değer sanılıp `dai.Device(pipeline, ...)`'a veriliyordu ⇒
    `TypeError: incompatible constructor arguments` ⇒ KONTROL 3 cihazda hiç
    koşmadı. Sözleşme değişirse (sıra ya da eleman sayısı) burası kırmızı olur.
    """
    cihaz = _sahte_kur(monkeypatch, kare_sayisi=3)

    kareler, cihaz_tespit = k3._kareleri_topla(3, 5.0)

    assert len(kareler) == 3
    assert cihaz_tespit == [2, 2, 2]          # her karede 2 kutu
    assert cihaz.kapandi, "cihaz `with` ile kapanmadı — sıradaki açış çakışır"


def test_kamera_susarsa_zaman_asimi_atar_asilmaz(k3, monkeypatch):
    """🪤 Kamera kare vermezse betik ASILMAMALI — sahada 'dondu mu' ayırt edilemez."""
    _sahte_kur(monkeypatch, kare_sayisi=1)

    with pytest.raises(TimeoutError, match="girdap-algi"):
        k3._kareleri_topla(5, 0.5)            # 5 kare istendi, 1 tane var


def test_kayit_aktif_degilse_acilir(k3, monkeypatch):
    """passthrough (NN giriş karesi) yalnız `KAYIT_AKTIF` iken bağlanıyor."""
    monkeypatch.setattr(dgn, "KAYIT_AKTIF", False)
    _sahte_kur(monkeypatch, kare_sayisi=1)

    k3._kareleri_topla(1, 5.0)

    assert dgn.KAYIT_AKTIF is True
