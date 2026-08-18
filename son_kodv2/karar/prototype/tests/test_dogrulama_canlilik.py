# -*- coding: utf-8 -*-
"""CANLILIK (sınırlı) değişmezleri — hepsinin bir SÜRE SINIRI olmalı."""
from __future__ import annotations

import math

import pytest

from prototype.dogrulama import canlilik as C
from prototype.dogrulama.butce import ARDUPILOT_KOMUT_KESME_S


# ═══════════ TASARIM KAPISI: saf canlılık İZLENEBİLİR DEĞİL ═══════════
def test_her_canlilik_kuralinin_SURE_SINIRI_var():
    """🔑 Saf canlılık ('er geç olacak') sonlu izde ihlal edilmiş sayılamaz.

    Yalnız SINIRLI canlılık ('T süre içinde olacak') izlenebilir. Bu test,
    her kuralın süre sınırlı bir imzası olduğunu donduruyor — biri sınırsız
    yazılırsa (izlenemez hâle gelirse) kırmızı yanar.
    """
    import inspect

    #: Sınır iki biçimde ifade edilebilir ve İKİSİ DE geçerlidir:
    #:   · mutlak tavan  (`tavan_s`)                       — C1, C2, C5
    #:   · göreli tavan  (`nominal_periyot_s × tolerans`)  — C3
    #: Aranan şey isim değil, sınırın **açıkça parametre olması**: sabit
    #: gömülü bir sayı, türetilebilirliği yok eder.
    SINIR_ISARETLERI = ("tavan", "tolerans", "sinir")
    for k in C.KURALLAR:
        p = list(inspect.signature(k._fn).parameters)
        assert any(i in ad for ad in p for i in SINIR_ISARETLERI), (
            f"{k.ad}: süre sınırı parametre DEĞİL — izlenemez canlılık")
        assert k.birim == "s", f"{k.ad}: sınırlı canlılık saniye cinsinden olmalı"
        # Sınırın bir VARSAYILANI olmalı ki kaynağı koda gömülü kalsın
        assert k._fn.__defaults__, f"{k.ad}: sınırın türetilmiş varsayılanı yok"


def test_sinirlar_TURETILMIS_kaynagi_yazili():
    for k in C.KURALLAR:
        assert len(k.kaynak) > 20, f"{k.ad}: kaynak yetersiz"


def test_C1_siniri_ArduPilot_esiginden_geliyor():
    """Sınır bizim seçimimiz değil — eyleyicinin davranışı."""
    assert C.c1_itki_sifir_kalmasin.__defaults__[0] == ARDUPILOT_KOMUT_KESME_S


# ───────────────────────────── C1 sessiz felç ─────────────────────────────
def test_C1_kisa_sifir_SESSIZ():
    """Manevra sırasında anlık sıfır itki normaldir."""
    assert C.C1.olc(0.2).ihlal is False


def test_C1_KAR04_sessiz_felci_yakalar():
    """21.000+ mesajın tamamı [0,0] — araç hiç tahrik almadı."""
    assert C.C1.olc(300.0).ihlal is True


def test_C1_tam_esikte_ihlal_DEGIL():
    assert C.C1.olc(ARDUPILOT_KOMUT_KESME_S).ihlal is False


# ──────────────────────── C2 durum ilerlemesi ────────────────────────
def test_C2_KAR03_BOOT_kilitlenmesini_yakalar():
    """25 dakika BOOT'ta kalındı, bu sürede 10 Hz komut yayınlandı."""
    assert C.C2.olc(25 * 60.0, "BOOT").ihlal is True


def test_C2_BEKLEMEDE_MUAF_yoksa_her_kosuda_yanar():
    """🔑 ÖZGÜLLÜK: YKİ komutu beklerken saatlerce durmak DOĞRU davranıştır.

    Muafiyet olmasaydı kural her koşuda yanardı ve hiçbir şey söylemezdi
    (09.08 `mono_menzil` dersi).
    """
    assert C.C2.olc(3 * 3600.0, "BEKLEMEDE").ihlal is False


@pytest.mark.parametrize("durum", ["TAMAMLANDI", "KILL", "beklemede"])
def test_C2_terminal_durumlar_da_MUAF(durum):
    assert C.C2.olc(10 * 3600.0, durum).ihlal is False


def test_C2_normal_parkur_ilerlemesi_SESSIZ():
    assert C.C2.olc(120.0, "PARKUR1").ihlal is False


# ────────────────────────── C3 topic akışı ──────────────────────────
def test_C3_sinir_topicin_KENDI_kadansindan_turer():
    """🔑 Sabit '5 s' eşiği yanlış olurdu: 50 Hz IMU için felaket,
    1 Hz görev yayını için normal."""
    assert C.C3.olc(0.10, 0.02).ihlal is True    # IMU 50 Hz, 5 periyot sessiz
    assert C.C3.olc(0.10, 1.00).ihlal is False   # 1 Hz topic, aynı sessizlik


def test_C3_PAR04_state_dususunu_yakalar():
    """/mavros/state 2 Hz'den 0,17 Hz'e düşmüştü ⇒ oturumun %86'sı KILL."""
    assert C.C3.olc(6.0, 0.5).ihlal is True


def test_C3_ALG05_lidar_cokusunu_yakalar():
    """LiDAR 5 saatte 39 mesaj."""
    assert C.C3.olc(60.0, 0.1).ihlal is True


def test_C3_saglikli_akis_SESSIZ():
    assert C.C3.olc(0.11, 0.10).ihlal is False


# ────────────────────────── C5 temiz kapanış ──────────────────────────
def test_C5_PAR10_sonlanmamis_dosyayi_yakalar():
    """14 bag'in 13'ü sonlandırılmamıştı. Aynı sınıf Dosya-1 mp4'ü de vurur
    (moov atomu yazılmazsa oynatılamaz ⇒ 5 ceza puanı)."""
    assert C.C5.olc(3.0, dosyalar_sonlandi=False).ihlal is True


def test_C5_temiz_kapanis_SESSIZ():
    assert C.C5.olc(3.0, dosyalar_sonlandi=True).ihlal is False


def test_C5_asiri_uzun_kapanis_yakalanir():
    assert C.C5.olc(25.0, dosyalar_sonlandi=True).ihlal is True


def test_C5_siniri_SERVIS_DOSYASINDAN():
    """TimeoutStopSec=20 — uydurulmuş değil."""
    assert C.KAPANIS_TAVANI_S == 20.0


# ═════════════════ ÖZGÜLLÜK: sağlıklı koşum sessiz mi ═════════════════
def test_SAGLIKLI_KOSUM_hicbir_canlilik_kurali_yakmaz():
    assert C.C1.olc(0.3).ihlal is False
    assert C.C2.olc(90.0, "PARKUR2").ihlal is False
    assert C.C3.olc(0.12, 0.10).ihlal is False
    assert C.C5.olc(2.0, True).ihlal is False
