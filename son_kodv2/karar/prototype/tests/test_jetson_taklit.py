# -*- coding: utf-8 -*-
"""JETSON TAKLİDİ — kaynak zarfı sözleşmesi.

🔴 NEDEN: göl bu masaüstünde koşuyor (i7-13620H, 16 iş parçacığı, 4,9 GHz);
saha hedefi **Orin Nano 8GB Super** (6× A78AE @1,5 GHz, 8 GB PAYLAŞIMLI).
Göl bu farkı üretmezse "10 Hz kontrol" varsayımı sahada çöker ve göl bunu
HABER VERMEZ — KAR-11'in yaşandığı sınıf tam budur.
"""
from __future__ import annotations

import io
import pathlib
import re
import subprocess

import pytest

_KOK = pathlib.Path(__file__).resolve().parents[2]
_BETIK = _KOK / "scripts/jetson_taklit.sh"
_METIN = io.open(_BETIK, encoding="utf-8").read()


def test_betik_SOZDIZIMI_temiz():
    r = subprocess.run(["bash", "-n", str(_BETIK)], capture_output=True)
    assert r.returncode == 0, r.stderr.decode()


def test_donanim_zarfi_GERCEK_SPEC(): 
    """Orin Nano 8GB Super: 6 çekirdek · 8 GB paylaşımlı (kaynaktan)."""
    assert "GIRDAP_JETSON_CEKIRDEK:-6}" in _METIN
    assert "GIRDAP_JETSON_BELLEK_MB:-8192}" in _METIN


def test_yavaslatma_orani_OLCUMDEN_turetildi():
    """🔑 Katsayı UYDURULMADI: gerçek Jetson 128 ms ↔ bu makine 51,2 ms.

    Kaynak `hatalar/karar.md` KAR-11 — oturum başlangıç periyotları
    117/111/145/141 ms. Sabit yazılmış keyfi bir sayı olsaydı bu test
    kaynağı gösteremezdi.
    """
    assert "GIRDAP_JETSON_KAT:-2.51}" in _METIN
    assert "KAR-11" in _METIN
    for kanit in ("117", "111", "145", "141", "51,2", "128"):
        assert kanit in _METIN, f"{kanit} kanıtı betikte yok"


def test_rakip_yuk_KALIBRE_edilmis_formulle_degil():
    """🪤 İlk sürüm `N = çekirdek × (1 − 1/KAT)` diyordu ⇒ N=4, ölçülen
    yalnız 65,4 ms (hedef 128). Rakip süreçler çekirdekleri PAYLAŞIYOR.
    Kalibrasyon tablosu betikte yazılı olmalı ki sonraki kişi formüle
    dönmesin."""
    for n, ms in (("N=6", "74,1"), ("N=10", "133,8"), ("N=14", "173,8")):
        assert n in _METIN and ms in _METIN, f"{n} kalibrasyon satırı yok"
    assert "0.9" in _METIN, "kalibre edilmiş bölen yok"


def test_KAT_2_51_icin_10_surec_uretiliyor():
    """Kalibrasyonun sayısal karşılığı — formül değişirse yakalanır."""
    m = re.search(r"round\(\$CEKIRDEK \* \(\$KAT - 1\) / ([\d.]+)\)", _METIN)
    assert m, "yük formülü bulunamadı"
    bolen = float(m.group(1))
    assert round(6 * (2.51 - 1) / bolen) == 10, "KAT=2,51 → 10 süreç olmalı"


def test_SINIRLAR_acikca_yazili():
    """Abartma yasağı: taklit CPU zarfını üretir, GPU/VPU'yu ETMEZ."""
    for sinir in ("GPU", "VPU", "BANT GENİŞLİĞİ"):
        assert sinir in _METIN, f"'{sinir}' sınırı yazılmamış"
    assert "Jetson'ın kendisinde ölçülür" in _METIN


def test_yuk_surecleri_TEMIZLENIYOR():
    """Sızan spin süreci makineyi kalıcı yavaşlatır — hayalet düğüm dersi."""
    assert "trap temizle EXIT INT TERM" in _METIN
    assert 'kill "$p"' in _METIN


def test_bellek_TAVANI_sessiz_OOM_yerine_GORUNUR_hata():
    """Orin'de bellek CPU+GPU ortak; MPPI tensörü N=2000'de 1,6 GB."""
    assert "ulimit -v" in _METIN
    assert "MemoryError" in _METIN or "GÖRÜNÜR" in _METIN
