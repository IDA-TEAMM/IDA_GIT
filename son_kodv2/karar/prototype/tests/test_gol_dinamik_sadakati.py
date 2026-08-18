# -*- coding: utf-8 -*-
"""GÖL DİNAMİK SADAKATİ — plant modeli gerçeği temsil ediyor mu?

🔴 18.08 denetimi (literatür: "zayıf plant modeli SIL'de en yaygın tuzak;
kararsız kontrolü kararlı gösterebilir"). Sanal göl `CatamaranDynamics`i HİÇ
kullanmıyordu; kendi birinci mertebe HIZ modeli vardı. Sapma ÖLÇÜLDÜ:

    ivme          0,234 ↔ 0,985 m/s²   → göl **4,2× fazla**
    zaman sabiti  4,76  ↔ 0,80  s      → göl **5,9× çevik**
    dönüş tavanı  0,289 ↔ 0,800 rad/s  → göl **2,8× hızlı**
"""
from __future__ import annotations

import io
import pathlib

import pytest

_KOK = pathlib.Path(__file__).resolve().parents[2]
_SG = io.open(_KOK / "scripts/sanal_gol.py", encoding="utf-8").read()
_GOL = io.open(_KOK / "scripts/gol_kos.sh", encoding="utf-8").read()


def test_gercek_dinamik_kipi_VAR():
    """Plant, MPPI'nin kullandığı modelin TA KENDİSİ olabilmeli."""
    assert 'declare_parameter("gercek_dinamik"' in _SG
    assert "CatamaranDynamics" in _SG
    assert "step_rk4" in _SG


def test_VARSAYILAN_KAPALI_eski_davranis_korunur():
    assert 'declare_parameter("gercek_dinamik", False)' in _SG
    assert "GIRDAP_GERCEK_DINAMIK:-0" in _GOL


def test_SAPMA_TABLOSU_kodda_yazili():
    """Sayılar kaybolmasın: sonraki kişi neden gerektiğini görsün."""
    for k in ("4,2×", "5,9×", "2,8×"):
        assert k in _SG, f"{k} sapma kaydı yok"


def test_bool_parametresi_KABUKTA_normalize_ediliyor():
    """🪤 rclpy tip katı: `-p x:=1` INTEGER gelir, düğüm ÖLÜR.
    (Betiğin YON0 için zaten öğrendiği ders.)"""
    assert "_GD=true" in _GOL and "_GD=false" in _GOL


def test_durum_dizisi_METOTLA_cakismiyor():
    """🪤 `self._durum` sınıfın `_durum()` METODUNU gölgeliyordu ⇒
    `'numpy.ndarray' object is not callable` ile TÜM zincir sessizce durdu."""
    assert "self._dyn_durum" in _SG
    assert "self._durum = np.zeros" not in _SG


def test_model_uyusmazligi_SINANABILIR():
    """Aynı modeli hem planlayıcı hem tesis kullanırsa 'model uyuşmazlığı'
    sınıfı sınanamaz — bozucu parametresi o boşluğu kapatır."""
    assert 'declare_parameter("gercek_dinamik_bozucu"' in _SG
    assert "GIRDAP_DINAMIK_BOZUCU" in _GOL


def test_cmd_vel_ITKIYE_geri_cevriliyor():
    """`planning_node` itkiyi hıza çeviriyor (`hedef_u = 2T/|Xu|`);
    tesis GERÇEK ikinci mertebe dinamiği koşabilmek için tersini yapmalı."""
    assert "abs(p.Xu) / 2.0" in _SG
    assert "p.thruster_spacing" in _SG


def test_basit_model_SAPMASI_olculdu_ve_yazili():
    """Eski model silinmedi (varsayılan), ama sapması belgeli."""
    assert "zayıf plant modeli" in _SG
    assert "Birinci mertebe" in _SG or "Basit model" in _SG
