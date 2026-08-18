# -*- coding: utf-8 -*-
"""KURAL MOTORU testleri — nicel doğrulama çekirdeği (ROS'suz).

İki yönlü sınama pazarlıksız (göl şartnamesinin kabul ölçütü):
  · DUYARLILIK — kuralı çiğneyen girdide **kırmızı** yanmalı
  · ÖZGÜLLÜK  — sağlıklı girdide **sessiz** kalmalı
Her koşuda yanan alarm alarm değildir (09.08 `mono_menzil` dersi).
"""
from __future__ import annotations

import math

import pytest

from prototype.dogrulama import Kural, Sonuc, Tur, bir_ara, degil, her_zaman, ve, veya
from prototype.dogrulama import fizik


def _k(ad="X", marj=1.0, birim="m", tur=Tur.DEGISMEZ):
    return Kural(ad, "test", "test kaynağı", birim, lambda: marj, tur)


# ───────────────────────────── çekirdek semantik ──────────────────────────
def test_marj_pozitif_gecti_negatif_ihlal():
    assert _k(marj=0.3).olc().ihlal is False
    assert _k(marj=-0.3).olc().ihlal is True
    assert _k(marj=0.0).olc().ihlal is False, "tam sınır ihlal DEĞİL"


def test_NaN_IHLAL_sayilir():
    """'Ölçemedim' asla 'iyi' demek değildir — bilinmeyen güvenli sayılmaz."""
    assert _k(marj=math.nan).olc().ihlal is True


def test_ve_MIN_alir():
    r = ve(_k("A", 0.5).olc(), _k("B", 0.2).olc(), _k("C", 0.9).olc())
    assert r.marj == pytest.approx(0.2)
    assert r.baglam["belirleyen"] == "B", "en zayıf halka raporlanmalı"


def test_veya_MAX_alir():
    r = veya(_k("A", -0.5).olc(), _k("B", 0.2).olc())
    assert r.marj == pytest.approx(0.2)


def test_degil_ISARET_cevirir():
    assert degil(_k(marj=0.4).olc()).marj == pytest.approx(-0.4)


def test_her_zaman_bir_kez_ihlal_YETER():
    """G (always): pencerede tek ihlal tüm pencereyi düşürür."""
    pencere = [_k(marj=v).olc() for v in (0.5, 0.4, -0.1, 0.6)]
    assert her_zaman(pencere).ihlal is True


def test_bir_ara_bir_kez_saglanma_YETER():
    """F (eventually): canlılık kuralları bunun üstüne kurulur."""
    pencere = [_k(marj=v).olc() for v in (-0.5, -0.4, 0.1, -0.6)]
    assert bir_ara(pencere).ihlal is False


# ────────────────────────────── koruma kapıları ────────────────────────────
def test_FARKLI_BIRIM_birlestirilemez():
    """min(0,3 m, 0,2 s) hesaplanır ama ANLAMSIZDIR."""
    with pytest.raises(ValueError, match="FARKLI BİRİMLER"):
        ve(_k("A", 0.3, "m").olc(), _k("B", 0.2, "s").olc())


def test_KAYNAKSIZ_kural_KURULAMAZ():
    """🔑 Uydurulmuş eşiğin sisteme sızmasını engelleyen kapı."""
    with pytest.raises(ValueError, match="kaynak"):
        Kural("Z", "test", "", "m", lambda: 1.0)


def test_normalize_OLCEKSIZ_kuralda_REDDEDILIR():
    with pytest.raises(ValueError, match="olcek"):
        k = _k(); k.normalize(k.olc())


def test_normalize_BOYUTSUZLASTIRIR_ve_kiyas_ACILIR():
    a = Kural("A", "-", "k", "m", lambda: 0.30, olcek=0.10)
    b = Kural("B", "-", "k", "s", lambda: 0.05, olcek=0.50)
    n = ve(a.normalize(a.olc()), b.normalize(b.olc()))
    assert n.marj == pytest.approx(0.1), "b daha dar paylı ⇒ o belirlemeli"
    assert n.baglam["belirleyen"] == "B"


def test_kural_PATLARSA_ihlal_doner_COKMEZ():
    """Gözlemcinin kendisi görevi düşüremez (NASA RV uyarısı) — ama susmaz."""
    k = Kural("P", "-", "k", "m", lambda: 1 / 0)
    s = k.olc()
    assert s.ihlal is True and "hata" in s.baglam


# ═══════════════════ FİZİK: eşikler TÜRETİLİYOR mu ═══════════════════
def test_tavanlar_DINAMIK_MODELDEN_turetiliyor():
    """🔑 Sayılar yazılı DEĞİL. Tekne değişirse eşik kendiliğinden değişir."""
    from prototype.dynamics.catamaran import CatamaranDynamics

    p = CatamaranDynamics().p
    u, r, a = fizik.tavanlar(p)
    assert u == pytest.approx(-2 * p.max_thrust / p.Xu)
    assert r == pytest.approx(-p.max_thrust * p.thruster_spacing / p.Nr)
    assert a == pytest.approx(2 * p.max_thrust / p.mass)


def test_tavanlar_TEKNE_DEGISINCE_degisir():
    """Eşik sabit yazılsaydı bu test geçmezdi — mutasyon kapısı."""
    from dataclasses import replace

    from prototype.dynamics.catamaran import CatamaranDynamics

    p = CatamaranDynamics().p
    guclu = replace(p, max_thrust=p.max_thrust * 2)
    assert fizik.tavanlar(guclu)[0] == pytest.approx(2 * fizik.tavanlar(p)[0])


def test_turetilen_tavan_SIMULASYONLA_tutuyor():
    """Analitik ↔ sayısal: 120 s tam gaz gerçekten u_max'e oturmalı."""
    import numpy as np

    from prototype.dynamics.catamaran import CatamaranDynamics

    d = CatamaranDynamics()
    u_max, _, _ = fizik.tavanlar(d.p)
    st = np.zeros(6)
    u = np.array([d.p.max_thrust, d.p.max_thrust])
    for _ in range(6000):
        st = d.step_rk4(st, u, 0.02)
    assert st[3] == pytest.approx(u_max, abs=0.01)


# ═════════════ DUYARLILIK: gerçek arızalar yakalanıyor mu ═════════════
def test_F1_KAR06_isinlanmasini_yakalar():
    """Gerçek bant bulgusu: 25 ms'de 6,54 m = 261,6 m/s."""
    assert fizik.F1.olc(6.54, 0.0, 0.025).ihlal is True


def test_F5_ALG02_kendi_govdesini_yakalar():
    """Gerçek bant bulgusu: en yakın 'engel' 1,3 mm — LiDAR gövdeyi görüyor."""
    assert fizik.F5.olc(0.0013).ihlal is True


def test_F4_KAR05_gecersiz_pozu_yakalar():
    assert fizik.F4.olc(math.nan, 1.0).ihlal is True
    assert fizik.F4.olc(math.inf, 0.0).ihlal is True


def test_F2_asiri_hizi_yakalar():
    u_max, _, _ = fizik.tavanlar()
    assert fizik.F2.olc(u_max * fizik.CEVRESEL_PAY + 0.5).ihlal is True


# ═════════════ ÖZGÜLLÜK: sağlıklı veride SESSİZ mi ═════════════
def test_SAGLIKLI_kosum_hicbir_kurali_yakmaz():
    """Her koşuda yanan alarm, alarm değildir."""
    u_max, r_max, _ = fizik.tavanlar()
    seyir = 0.62  # kaptanın göl bandından ölçtüğü gerçek seyir hızı
    assert fizik.F1.olc(seyir * 0.1, 0.0, 0.1).ihlal is False
    assert fizik.F2.olc(seyir).ihlal is False
    assert fizik.F2R.olc(r_max * 0.8).ihlal is False
    assert fizik.F4.olc(12.3, -4.5, 0.7).ihlal is False
    assert fizik.F5.olc(3.0).ihlal is False


def test_TAM_GAZ_bile_ihlal_DEGIL():
    """Fiziksel tavanda koşmak ihlal olmamalı — yoksa kural her turda yanar."""
    u_max, r_max, _ = fizik.tavanlar()
    assert fizik.F2.olc(u_max).ihlal is False
    assert fizik.F2R.olc(r_max).ihlal is False


def test_her_kuralin_KAYNAGI_yazili():
    """Sözleşme: kaynaksız eşik yok."""
    for k in fizik.KURALLAR:
        assert len(k.kaynak) > 10, f"{k.ad}: kaynak yetersiz"
