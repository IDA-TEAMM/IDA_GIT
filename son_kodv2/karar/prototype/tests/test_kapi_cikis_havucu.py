# -*- coding: utf-8 -*-
"""KAPI ÇIKIŞ HAVUCU — araç kapıda frenlemesin (19.08.2026).

🔴 ÖLÇÜLEN ARIZA: araç kapı ortasına varıyor ve düzlemi aşamıyor. İmza her
redde aynıydı: yanal **0,00 m** (nişan kusursuz) · ileri **−0,2 … −7 m**
(gerekli > +1,53 m). Yani algı ve nişan doğru; araç kapıya varınca duruyor.

MEKANİZMA — üç büyüklük uyumsuzdu:
    çıkış havucu           1,03 m   (`hull_length_m`)
    MPPI terminal ufku     3,00 m   (`terminal_lookahead_m`)
    geçişin sayılma şartı  1,53 m   (algı `PASS_EK_YOL` = gövde + 0,5)
MPPI'de AYRI HIZ TERİMİ YOK; seyir hızını terminal maliyetin gradyanı
(2·w·d) belirler. Hedef 1,03 m ötedeyken gradyan küçülüp itki sıfıra gider.

✅ DEĞER FİZİKTEN TÜRÜYOR: 1,03 + 0,50 + 2,00 (varış yarıçapı) = **3,53 m**
⇒ araç nişan çemberine girdiği anda düzlemi ZATEN aşmış olur.

📊 ÖLÇÜM (`kapi_orani.py`, 12'şer koşum, ROS'suz):
      1,03 m → kapı geçme %31,2 · en iyi %37,5
      3,53 m → kapı geçme %62,5 · ortanca %75,0 · en kötü %37,5
    Dağılımlar örtüşmüyor; gövde payı 0,06 → 0,24 m, çarpma 0/12 ↔ 0/12.
"""
from __future__ import annotations

import pytest

from prototype.mission.gate_follower import Gate, GateFollowerConfig


def test_havuc_GECIS_SARTINDAN_buyuk():
    """Havuç, geçişin sayılma şartını (gövde + 0,5) AŞMALI.

    Aksi hâlde araç nişanına varsa bile düzlemi geçmiş olmaz — arızanın
    tam tanımı budur.
    """
    cfg = GateFollowerConfig()
    pass_ek_yol = cfg.hull_length_m + 0.5      # algı tarafındaki şartla aynı
    assert cfg.cikis_havucu > pass_ek_yol, (
        f"havuç {cfg.cikis_havucu} ≤ geçiş şartı {pass_ek_yol} — "
        "araç kapı ortasında frenler")


def test_havuc_VARIS_YARICAPINI_de_kapsiyor():
    """Araç nişan çemberine (2,0 m) girdiğinde düzlem ZATEN aşılmış olmalı.

    Havuç < yarıçap + şart ise araç 'vardım' deyip durur ve geçiş sayılmaz.
    """
    cfg = GateFollowerConfig()
    varis_yaricapi = 2.0                        # params.yaml `arrival_radius_m`
    gerekli = cfg.hull_length_m + 0.5 + varis_yaricapi
    assert cfg.cikis_havucu >= gerekli - 1e-9, (
        f"havuç {cfg.cikis_havucu} < {gerekli:.2f} — araç çembere girince "
        "durur, düzlem aşılmaz")


def test_havuc_MPPI_ufkundan_kisa_DEGIL():
    """Havuç `terminal_lookahead_m`'den kısaysa MPPI hedefe yaklaşırken
    frenler (maliyette ayrı hız terimi yok, gradyan 2·w·d)."""
    cfg = GateFollowerConfig()
    mppi_lookahead = 3.0                        # params.yaml
    assert cfg.cikis_havucu >= mppi_lookahead, (
        f"havuç {cfg.cikis_havucu} < MPPI ufku {mppi_lookahead} — araç "
        "kapıya varmadan yavaşlar")


def test_havuc_ASIRI_BUYUK_degil():
    """Üst sınır: havuç kapı aralığından büyükse nişan bir SONRAKİ kapının
    ötesine düşer ve aradaki kapı atlanır.

    Şekil 3'ten ölçülen aralık/genişlik oranı 0,80; 8 m'lik en dar kapıda
    aralık ≈ 6,4 m. Havuç bunun yarısını aşmamalı.
    """
    cfg = GateFollowerConfig()
    en_dar_aralik = 8.0 * 0.80
    assert cfg.cikis_havucu <= en_dar_aralik / 2.0 + 0.5, (
        f"havuç {cfg.cikis_havucu} çok büyük — sonraki kapıyı atlayabilir")


def test_surus_noktasi_HAVUCU_kullaniyor():
    """Kapı çıkış noktası cfg'deki havuçla hesaplanmalı — sabit gömülü
    değer ya da env kancası kalmamalı."""
    import ast
    import inspect
    import textwrap

    from prototype.mission import gate_follower as gf
    kaynak = inspect.getsource(gf)
    assert "_HAVUC_M" not in kaynak, "geçici env kancası kodda kalmış"
    agac = ast.parse(kaynak)
    kullanim = [n for n in ast.walk(agac)
                if isinstance(n, ast.Attribute) and n.attr == "cikis_havucu"]
    assert len(kullanim) >= 1, "havuç hiçbir yerde kullanılmıyor"
    # ⚠ Config'e AYARLANABİLİR ALAN eklenmemeli (§0.0d kuralı, sınıfın kendi
    # uyarısı): havuç TÜRETİLİR, alan değildir.
    from dataclasses import fields
    from prototype.mission.gate_follower import GateFollowerConfig as _C
    assert "cikis_havuc_m" not in {f.name for f in fields(_C)}, (
        "havuç config alanı olarak eklenmiş — türetilmiş property olmalı")


def test_cikis_noktasi_duzlemi_ASIYOR():
    """Uçtan uca: araç kapı ortasındayken çıkış noktası düzlemin ötesinde
    ve geçiş şartını sağlayacak kadar uzakta olmalı."""
    cfg = GateFollowerConfig()
    kapi = Gate(
        left=(-5.0, 10.0), right=(5.0, 10.0), midpoint=(0.0, 10.0),
        normal=(0.0, 1.0),
    )
    hedef = kapi.surus_noktasi((0.0, 10.0), 5.0, cfg.cikis_havucu)
    ileri = (hedef[1] - kapi.midpoint[1])       # normal +y yönünde
    assert ileri >= cfg.hull_length_m + 0.5, (
        f"çıkış noktası düzlemi yalnız {ileri:.2f} m aşıyor")
