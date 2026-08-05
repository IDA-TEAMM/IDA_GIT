#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""oak_baglanti.py — kamerasız testler.

USB ioctl'i ve gerçek cihaz burada test EDİLMEZ (donanım gerekir); test edilen
şey ETRAFINDAKİ MANTIK: kaç kez denenir, ne zaman reset atılır, hangi hata
yukarı fırlatılır, sıcaklık nasıl sınıflandırılır. Bunlar sahada yanlış
davranırsa kamera hiç açılmaz ya da aşırı ısınma sessizce geçer.

Koşum:  python3 -m pytest girdap_ida_algi/test/test_oak_baglanti.py -q
"""
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from girdap_ida_algi import oak_baglanti as ob  # noqa: E402


# ───────────────────────────────── dayanikli_ac ───────────────────────────
def test_ilk_denemede_acilirsa_reset_atilmaz(monkeypatch):
    resetler = []
    monkeypatch.setattr(ob, "usb_reset", lambda *a, **k: resetler.append(1) or True)
    dev = ob.dayanikli_ac(lambda: "CIHAZ", deneme=4)
    assert dev == "CIHAZ"
    assert resetler == []          # boş yere USB resetlemek cihazı kilitleyebilir


def test_kilitlenirse_reset_atip_yeniden_dener(monkeypatch):
    resetler = []
    monkeypatch.setattr(ob, "usb_reset", lambda *a, **k: (resetler.append(1), True)[1])
    monkeypatch.setattr(ob.time, "sleep", lambda s: None)

    denemeler = {"n": 0}

    def acici():
        denemeler["n"] += 1
        if denemeler["n"] < 3:
            raise RuntimeError("X_LINK_DEVICE_NOT_FOUND")
        return "CIHAZ"

    assert ob.dayanikli_ac(acici, deneme=4) == "CIHAZ"
    assert denemeler["n"] == 3
    assert len(resetler) == 2       # her başarısız denemeden sonra bir reset


def test_deneme_tukenirse_SON_hatayi_firlatir(monkeypatch):
    monkeypatch.setattr(ob, "usb_reset", lambda *a, **k: True)
    monkeypatch.setattr(ob.time, "sleep", lambda s: None)

    def acici():
        raise RuntimeError("son hata")

    with pytest.raises(RuntimeError, match="son hata"):
        ob.dayanikli_ac(acici, deneme=3)


def test_son_denemeden_sonra_bosuna_reset_atmaz(monkeypatch):
    """Deneme bittiyse reset atmanın faydası yok; cihazı boşuna sarsma."""
    resetler = []
    monkeypatch.setattr(ob, "usb_reset", lambda *a, **k: (resetler.append(1), True)[1])
    monkeypatch.setattr(ob.time, "sleep", lambda s: None)
    with pytest.raises(RuntimeError):
        ob.dayanikli_ac(lambda: (_ for _ in ()).throw(RuntimeError("x")), deneme=3)
    assert len(resetler) == 2       # 3 deneme → aralarda 2 reset, sonda YOK


def test_reset_basarisizsa_yine_de_dener(monkeypatch):
    """USB reset başarısız olsa bile (kural yok/cihaz yok) açılış denenmeye devam
    etmeli — kilit bazen kendiliğinden açılıyor."""
    monkeypatch.setattr(ob, "usb_reset", lambda *a, **k: False)
    monkeypatch.setattr(ob.time, "sleep", lambda s: None)
    n = {"i": 0}

    def acici():
        n["i"] += 1
        if n["i"] < 2:
            raise RuntimeError("kilit")
        return "CIHAZ"

    assert ob.dayanikli_ac(acici, deneme=3) == "CIHAZ"


def test_log_fonksiyonu_cagrilir(monkeypatch):
    monkeypatch.setattr(ob, "usb_reset", lambda *a, **k: True)
    monkeypatch.setattr(ob.time, "sleep", lambda s: None)
    satirlar = []
    n = {"i": 0}

    def acici():
        n["i"] += 1
        if n["i"] < 2:
            raise RuntimeError("kilit")
        return "CIHAZ"

    ob.dayanikli_ac(acici, deneme=3, kaydet=satirlar.append)
    assert any("açılamadı" in s for s in satirlar)
    assert any("reset" in s.lower() for s in satirlar)


# ─────────────────────────────── sicaklik_durumu ──────────────────────────
@pytest.mark.parametrize("c,beklenen", [
    (25.0, "normal"),
    (68.9, "normal"),      # 11 FPS'te ölçülen iç mekân platosu
    (84.9, "normal"),
    (85.0, "uyari"),       # eşik DÂHİL
    (94.9, "uyari"),
    (95.0, "kritik"),      # eşik DÂHİL
    (125.0, "kritik"),     # gözlenen çökme sıcaklığı
])
def test_sicaklik_esikleri(c, beklenen):
    assert ob.sicaklik_durumu(c) == beklenen


def test_sicaklik_okunamazsa_alarm_uretmez():
    """Ölçemediğimiz şey için 'kritik' demek yanlış alarmdır; None → normal."""
    assert ob.sicaklik_durumu(None) == "normal"


def test_esikler_disaridan_verilebilir():
    assert ob.sicaklik_durumu(70.0, uyari=60.0, kritik=80.0) == "uyari"
    assert ob.sicaklik_durumu(90.0, uyari=60.0, kritik=80.0) == "kritik"


def test_kritik_esik_cip_anma_sinirinin_ALTINDA():
    """Çip anma sınırı 105 °C, gözlenen çökme 125 °C. Kritik eşiğimiz bunların
    altında olmalı ki müdahale için zaman kalsın."""
    assert ob.SICAKLIK_KRITIK < 105.0
    assert ob.SICAKLIK_UYARI < ob.SICAKLIK_KRITIK


# ───────────────────────────────── vpu_sicakligi ──────────────────────────
def test_vpu_sicakligi_okur():
    class SahteDev:
        def getChipTemperature(self):
            return type("T", (), {"average": 66.5})()

    assert ob.vpu_sicakligi(SahteDev()) == pytest.approx(66.5)


def test_vpu_sicakligi_hatada_None_doner():
    class BozukDev:
        def getChipTemperature(self):
            raise RuntimeError("cihaz kapandı")

    assert ob.vpu_sicakligi(BozukDev()) is None


# ───────────────────────────────── usb_dugum_yolu ─────────────────────────
def test_usb_dugum_yolu_cihaz_yoksa_None(monkeypatch):
    monkeypatch.setattr(ob.glob, "glob", lambda p: [])
    assert ob.usb_dugum_yolu() is None


def test_usb_reset_cihaz_yoksa_False(monkeypatch):
    monkeypatch.setattr(ob, "usb_dugum_yolu", lambda: None)
    assert ob.usb_reset() is False
