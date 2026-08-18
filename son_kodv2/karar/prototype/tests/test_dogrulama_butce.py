# -*- coding: utf-8 -*-
"""BÜTÇE DEĞİŞMEZLERİ testleri + DAĞITIM CONFIG'İNİN DENETİMİ."""
from __future__ import annotations

import pathlib

import pytest

from prototype.dogrulama import butce

_HW = (pathlib.Path(__file__).resolve().parents[2]
       / "ros2_ws/src/girdap_decision/config/hardware.yaml")


# ────────────────────────────── B1 ──────────────────────────────
def test_B1_saglikli_periyot_SESSIZ():
    assert butce.B1.olc(0.105, 0.100).ihlal is False


def test_B1_KAR11_sarkmasini_yakalar():
    """Gerçek bant bulgusu: 10 Hz döngü 1.062 ms'e çıkmıştı."""
    assert butce.B1.olc(1.062, 0.100).ihlal is True


def test_B1_nominal_gecersizse_IHLAL():
    assert butce.B1.olc(0.1, 0.0).ihlal is True


# ────────────────────────────── B2 ──────────────────────────────
def test_B2_damga_duzeltmesi_ONCESI_bile_gecerdi():
    """Dürüstlük: 202 ms, ArduPilot'un 3 s eşiğini AŞMIYOR.

    Damga düzeltmesinin gerekçesi B2 değildi — füzyon eşleşmesiydi
    (202,4 ms = sync_slop_s'in 2,02 katı). Kurallar birbirinin yerine geçmez.
    """
    assert butce.B2.olc(0.2024).ihlal is False


def test_B2_esigi_asan_gecikmeyi_yakalar():
    assert butce.B2.olc(3.5).ihlal is True


# ────────────────────────────── B4 ──────────────────────────────
def test_B4_algi_zinciri_SAGLIKLI():
    """Algı düğümü 15 Hz döner, NN 8 FPS üretir ⇒ kuyruk birikmez."""
    assert butce.B4.olc(15.0, 8.0).ihlal is False


def test_B4_yavas_tuketici_yakalanir():
    assert butce.B4.olc(5.0, 10.0).ihlal is True


# ══════════════════════ B3 — DAĞITIM CONFIG DENETİMİ ══════════════════════
def test_merdiven_CONFIGDEN_okunuyor_elle_yazilmiyor():
    """Sayılar testte sabit olsaydı config değişince sessizce bayatlardı."""
    m = butce.dagitim_merdiveni(str(_HW))
    assert m, "hardware.yaml'dan hiç bekçi süresi okunamadı"
    assert all(isinstance(v, float) and v > 0 for _, v in m)


def test_B3_ters_merdiveni_yakalar():
    """Duyarlılık — sentetik ters merdiven."""
    assert butce.B3.olc([("a", 1.0), ("b", 4.0)]).ihlal is True


def test_B3_duzgun_merdiven_SESSIZ():
    """Özgüllük — en dıştaki halka eşiğin altındaysa alarm YOK."""
    assert butce.B3.olc([("a", 1.0), ("b", 2.5)]).ihlal is False


@pytest.mark.xfail(
    strict=True,
    reason=(
        "🔴 CANLI İHLAL (18.08 bulundu): heartbeat_timeout_s = 5,0 s, "
        "ArduPilot komut kesme eşiği 3,0 s ⇒ marj −2,0 s. 3-5 sn arasında "
        "tekne DURMUŞ hâldeyken yığın 'sağlıklı' der. Düzeltme karar "
        "tarafının canlı ayarı — kararı verilince bu işaret KALDIRILACAK "
        "(strict=True: düzeltilince test 'beklenmedik geçti' diye uyarır)."
    ),
)
def test_DAGITIM_CONFIGI_B3_saglamali():
    m = butce.dagitim_merdiveni(str(_HW))
    s = butce.B3.olc(m)
    assert s.ihlal is False, f"B3 marjı {s.marj:+.2f} s"


def test_ArduPilot_esigi_BIZIM_ayarimiz_DEGIL():
    """Sözleşme: 3 s uçuş kontrolcüsünün davranışı; config'ten okunmaz,
    ona GÖRE ayarlanırız. Değeri değiştiren biri bu testi görsün."""
    assert butce.ARDUPILOT_KOMUT_KESME_S == 3.0


def test_her_butce_kuralinin_KAYNAGI_yazili():
    for k in butce.KURALLAR:
        assert len(k.kaynak) > 10, f"{k.ad}: kaynak yetersiz"
