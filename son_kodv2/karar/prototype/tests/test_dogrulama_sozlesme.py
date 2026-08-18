# -*- coding: utf-8 -*-
"""SÖZLEŞME DEĞİŞMEZLERİ testleri — REP-103/105 dayanaklı."""
from __future__ import annotations

import math

import pytest

from prototype.dogrulama import sozlesme as S


# ───────────────────────────── S1 damga yaşı ─────────────────────────────
def test_S1_saglikli_yas_SESSIZ():
    """Ölçülen gerçek yaş (çekim→yayın) 0,2 s bandındaydı."""
    assert S.S1.olc(0.2024).ihlal is False


def test_S1_ALG06_bayat_damgayi_yakalar():
    """Gerçek bant bulgusu: algı damgaları 56 yıl bayattı."""
    assert S.S1.olc(56 * 365 * 86400).ihlal is True


def test_S1_GELECEKTEN_damgayi_da_yakalar():
    """Saat sıçraması iki yöne de gider — negatif yaş da ihlal."""
    assert S.S1.olc(-0.5).ihlal is True


def test_S1_tavanda_SINIRDA_gecer():
    assert S.S1.olc(2.0).ihlal is False
    assert S.S1.olc(2.01).ihlal is True


# ───────────────────────────── S2 alan sözleşmesi ─────────────────────────
def test_S2_gecerli_siniflar_SESSIZ():
    assert S.S2.olc([0, 1, 2, 99]).ihlal is False


def test_S2_sozlesme_disi_sinifi_yakalar_ve_SAYAR():
    s = S.S2.olc([0, 7, 42])
    assert s.ihlal is True and s.marj == pytest.approx(-2.0)


def test_S2_bos_liste_ihlal_DEGIL():
    """'Duba yok' ile 'sözleşme bozuk' farklı şeyler."""
    assert S.S2.olc([]).ihlal is False


def test_S2B_E1_regresyonunu_yakalar():
    """640×480 yayınlayıp 1280×720 varsayılması P1+P2'yi sessizce sıfırlıyordu."""
    assert S.S2B.olc(640, 480).ihlal is True
    assert S.S2B.olc(1280, 720).ihlal is False


# ───────────────────────────── S3 QoS uyumu ──────────────────────────────
def test_S3_RELIABLE_abone_BEST_EFFORT_yayinci_IHLAL():
    """Tek gerçek sessiz-ölüm yönü budur."""
    assert S.S3.olc(("BEST_EFFORT", "VOLATILE"), ("RELIABLE", "VOLATILE")).ihlal is True


def test_S3_ters_yon_UYUMLU():
    """BEST_EFFORT abone + RELIABLE yayıncı çalışır (best-effort'a düşer)."""
    assert S.S3.olc(("RELIABLE", "VOLATILE"), ("BEST_EFFORT", "VOLATILE")).ihlal is False


def test_S3_mavros_state_GERCEK_ciftimiz_UYUMLU():
    """MAVROS kaynağından doğrulandı: StateQoS = RELIABLE + TRANSIENT_LOCAL.
    Bizim abonemiz RELIABLE + VOLATILE ⇒ daha gevşek ⇒ uyumlu."""
    assert S.S3.olc(("RELIABLE", "TRANSIENT_LOCAL"),
                    ("RELIABLE", "VOLATILE")).ihlal is False


def test_S3_TRANSIENT_LOCAL_abone_VOLATILE_yayinci_IHLAL():
    assert S.S3.olc(("RELIABLE", "VOLATILE"),
                    ("RELIABLE", "TRANSIENT_LOCAL")).ihlal is True


def test_S3_tanimsiz_deger_IHLAL_sayilir():
    assert S.S3.olc(("SACMA", "VOLATILE"), ("RELIABLE", "VOLATILE")).ihlal is True


# ──────────────────────── S5 çerçeve sözleşmesi ──────────────────────────
def test_S5_beklenen_cerceve_SESSIZ():
    assert S.S5.olc("base_link", ["base_link"]).ihlal is False


def test_S5_yanlis_cerceveyi_yakalar():
    """03.08 canlı hatası: gövde koordinatı dünya sanılıyordu."""
    assert S.S5.olc("map", ["base_link"]).ihlal is True


def test_S5_bos_cerceve_IHLAL():
    assert S.S5.olc("", ["base_link"]).ihlal is True


def test_S5C_REP105_garantilerini_dogru_ayiriyor():
    """REP-105: odom sürekli (sıçramaz) · map sıçrayabilir."""
    assert S.S5C.olc("odom", True).ihlal is False
    assert S.S5C.olc("map", True).ihlal is True
    # İnterpolasyon yoksa map de sorun değil
    assert S.S5C.olc("map", False).ihlal is False


def test_REP105_cerceve_kumeleri_AYRIK():
    assert not (S.SUREKLI_CERCEVELER & S.SICRAYABILIR_CERCEVELER)


# ═════════════════ CANLI SÖZLEŞME BOŞLUĞU — 18.08 bulgusu ═════════════════
@pytest.mark.xfail(
    strict=True,
    reason=(
        "🔴 SÖZLEŞME BOŞLUĞU: `/girdap/fusion/odom` adı 'odom' ama "
        "frame_id='map' ile yayınlanıyor (fusion_node.py:677). REP-105'e göre "
        "`map` AYRIK SIÇRAYABİLİR; oysa `planning_node._poz_damgada` iki poz "
        "arasında İNTERPOLASYON yapıyor — sıçrama üzerinden interpolasyon "
        "var olmayan bir poz uydurur. İki çözüm yolu var ve KARAR verilmedi: "
        "(a) çerçeve 'odom' olacak (süreklilik garanti edilir), ya da "
        "(b) tüketici sıçramayı algılayıp interpolasyonu kesecek. "
        "Karar verilince bu işaret kaldırılır."
    ),
)
def test_FUSION_ODOM_interpolasyona_uygun_cerceve_yayinlamali():
    yayinlanan_cerceve = "map"          # fusion_node.py:677 — kaynaktan
    planlama_interpolasyon_yapiyor = True   # planning_node._poz_damgada
    assert S.S5C.olc(yayinlanan_cerceve, planlama_interpolasyon_yapiyor).ihlal is False


def test_her_sozlesme_kuralinin_KAYNAGI_yazili():
    for k in S.KURALLAR:
        assert len(k.kaynak) > 10, f"{k.ad}: kaynak yetersiz"
