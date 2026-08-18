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


def test_yuk_gol_omru_boyunca_YASAR_ama_temizlenebilir():
    """🪤 TRAP TASARIM KUSURU (18.08, ölçümle bulundu).

    `gol_kos.sh` düğümleri ARKA PLANDA başlatıp hemen dönüyor. `trap ... EXIT`
    ile rakip yükü sarmalayıcı çıkarken öldürüyorduk ⇒ **14 düğüm koşuyor ama
    yük süreci 0** — yani "Jetson zarfında koştu" sanılan koşum aslında
    MASAÜSTÜ hızındaydı. Sessiz ve tamamen yanıltıcı bir ölçüm.

    Doğrusu: yük gölün ömrü boyunca yaşar, ayrı betikle öldürülür.
    Test iki yönü de dondurur: EXIT'te ÖLMEZ, ama INT/TERM'de ve
    `jetson_yuk_dur.sh` ile ölür (sızan spin makineyi kalıcı yavaşlatır).
    """
    assert "trap temizle INT TERM" in _METIN
    assert "trap temizle EXIT" not in _METIN, "EXIT trap'i zarfı öldürür"
    assert 'kill "$p"' in _METIN
    assert "jetson_yuk.pids" in _METIN, "PID'ler kalıcı dosyaya yazılmıyor"

    dur = _KOK / "scripts/jetson_yuk_dur.sh"
    assert dur.exists(), "yükü durduran betik yok"
    metin = io.open(dur, encoding="utf-8").read()
    assert "jetson_yuk.pids" in metin and "pkill" in metin


def test_set_e_KULLANILMIYOR():
    """Sarmalayıcı `pgrep`/`kill` gibi 'bulamadım'da sıfırdan farklı dönen
    komutlar çağırıyor; `set -e` ile betik HİÇ ÇIKTI VERMEDEN ölüyordu."""
    assert not any(l.strip() == "set -e" for l in _METIN.splitlines())
    assert "`set -e` KULLANILMIYOR" in _METIN


def test_bellek_tavani_UYGULANMIYOR_ve_SEBEBI_yazili():
    """🪤 `ulimit -v` YANLIŞ ARAÇ — ölçüldü, kabuk exit 144 ile ölüyordu.

    `-v` SANAL adres alanını (VSZ) sınırlar, fiziksel kullanımı değil.
    numpy/DDS/CUDA büyük sanal alan ayırır ama çoğunu kullanmaz ⇒ 8 GB VSZ
    tavanı gerçek 8 GB RAM'i taklit ETMEZ, yalnız süreci erken öldürür.

    Test iki şeyi birden dondurur: (a) tavan uygulanmıyor, (b) sebebi ve
    doğru araçlar yazılı — biri "eksik" sanıp geri eklemesin.
    """
    import re
    # Etkin `ulimit -v` satırı OLMAMALI (yorumda geçmesi serbest)
    etkin = [l for l in _METIN.splitlines()
             if l.strip().startswith("ulimit -v")]
    assert not etkin, f"etkin ulimit -v satırı var: {etkin}"
    assert "SANAL adres alanını" in _METIN, "sebep yazılmamış"
    assert "cgroup" in _METIN, "doğru araç önerilmemiş"
    assert "R1" in _METIN, "kalan boşluk (kaynak tükenmesi kuralı) not edilmemiş"
