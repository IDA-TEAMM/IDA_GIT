"""MPPI davranış simülasyonunun ÜÇ KAPISI — 17.08 donmasının nöbetçileri.

🔴 NEDEN VAR (GIRDAP_DURUM §1.47): `scripts/mppi_davranis_sim.py`'nin üç
eşzamanlı koşumu 17.08.2026 15:19'da Jetson'ı dondurdu (her süreç 2,4 GB
yerleşik bellek, `Free swap = 0kB`, elle kapatma). Sebep bir MPPI ayarı
değil **koşum biçimiydi**; düzeltme de kod kapılarıdır. Bu dosya o kapıları
dondurur — biri kaldırılırsa süit kırmızı olur, sessizce geri gelmez.
"""
from __future__ import annotations

import importlib.util
import os
import sys
from pathlib import Path

import pytest

_KOK = Path(__file__).resolve().parents[1]          # .../karar/prototype
_BETIK = _KOK.parent / "scripts" / "mppi_davranis_sim.py"


@pytest.fixture(scope="module")
def sim():
    """Betiği modül olarak yükle (paket değil — `kapi_orani` deseninin aynısı)."""
    if not _BETIK.exists():                          # pragma: no cover
        pytest.skip(f"betik yok: {_BETIK}")
    sys.path.insert(0, str(_KOK.parent))
    spec = importlib.util.spec_from_file_location("mppi_davranis_sim", _BETIK)
    modul = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(modul)
    return modul


# --- Kapı ① tek örnek kilidi -------------------------------------------------

def test_kilit_ikinci_koşumu_ENGELLER(sim, tmp_path):
    """İkinci koşum SystemExit ile durur — donmanın doğrudan sebebi buydu."""
    yol = tmp_path / "sim.kilit"
    sim._tek_ornek_kilidi(yol)
    with pytest.raises(SystemExit) as hata:
        sim._tek_ornek_kilidi(yol)
    assert "ZATEN KOŞUYOR" in str(hata.value)


def test_kilit_dosyasi_acik_TUTULUR(sim, tmp_path):
    """Kilit dosyası kapatılırsa `flock` düşer → kapı sessizce açılır."""
    sim._tek_ornek_kilidi(tmp_path / "sim2.kilit")
    assert sim._kilit_dosyasi is not None
    assert not sim._kilit_dosyasi.closed


# --- Kapı ② bellek kafesi ----------------------------------------------------

def test_kafes_tavani_ve_takas_yasagi_GECER(sim, monkeypatch):
    cagri = {}
    monkeypatch.setattr(sim, "_kafes_kurulabilir", lambda: True)
    monkeypatch.setattr(sim.os, "execvpe",
                        lambda d, k, o: cagri.update(komut=k, ortam=o))
    monkeypatch.delenv(sim._KAFES_ISARETI, raising=False)
    sim._kafese_gir("2G")
    komut = cagri["komut"]
    assert "MemoryMax=2G" in komut, "bellek tavanı geçmiyor"
    assert "MemorySwapMax=0" in komut, "takas yasağı geçmiyor — donma tam buradan"
    assert cagri["ortam"][sim._KAFES_ISARETI] == "1", "yeniden başlatma döngüsü riski"


def test_kafes_isareti_varsa_YENIDEN_BASLATMAZ(sim, monkeypatch):
    """Kafesin içindeyken tekrar kafese girmeye çalışmak sonsuz döngü olurdu."""
    monkeypatch.setattr(sim, "_kafes_kurulabilir", lambda: True)
    monkeypatch.setattr(sim.os, "execvpe",
                        lambda *a: pytest.fail("kafes içinde yeniden başlatıldı"))
    monkeypatch.setenv(sim._KAFES_ISARETI, "1")
    sim._kafese_gir("2G")


def test_kafes_yok_denince_ve_kurulamayinca_KOSUM_OLMEZ(sim, monkeypatch, capsys):
    """Araç teşhis aracı — kafes kurulamıyorsa uyarır, koşumu engellemez."""
    monkeypatch.setattr(sim.os, "execvpe",
                        lambda *a: pytest.fail("kafes kurulamazken çağrıldı"))
    monkeypatch.delenv(sim._KAFES_ISARETI, raising=False)
    sim._kafese_gir("yok")                                  # açık kapatma
    monkeypatch.setattr(sim, "_kafes_kurulabilir", lambda: False)
    sim._kafese_gir("2G")                                   # kurulamıyor
    assert "KURULAMADI" in capsys.readouterr().out, "sessiz geçmemeli"


# --- Kapı ③ hesap yolu (varsayılan işlemci) ----------------------------------

def test_varsayilan_hesap_yolu_ISLEMCI(sim, monkeypatch):
    """Ekran kartı bağlamı süreç başına ~8,5 GB sanal alan → varsayılan olamaz."""
    monkeypatch.delenv("CUDA_VISIBLE_DEVICES", raising=False)
    ad = sim._hesap_yolunu_sec(gpu=False)
    assert os.environ["CUDA_VISIBLE_DEVICES"] == ""
    assert "işlemci" in ad


def test_gpu_acik_istekse_ORTAMA_DOKUNULMAZ(sim, monkeypatch):
    monkeypatch.setenv("CUDA_VISIBLE_DEVICES", "0")
    ad = sim._hesap_yolunu_sec(gpu=True)
    assert os.environ["CUDA_VISIBLE_DEVICES"] == "0"
    assert "ekran kartı" in ad


# --- Kapı ④ öldürme sırası ---------------------------------------------------

def test_oom_onceligi_yigindan_YUKSEK(sim, monkeypatch):
    """Bellek biterse ilk bu betik ölsün, `girdap-karar` değil."""
    yazilan = {}
    monkeypatch.setattr(sim.Path, "write_text",
                        lambda self, m: yazilan.update(yol=str(self), deger=m))
    sim._oom_onceligini_yukselt()
    assert yazilan["yol"] == "/proc/self/oom_score_adj"
    assert int(yazilan["deger"]) >= 500


def test_oom_yazimi_basarisizsa_KOSUM_OLMEZ(sim, monkeypatch):
    def patla(self, metin):
        raise OSError("izin yok")
    monkeypatch.setattr(sim.Path, "write_text", patla)
    sim._oom_onceligini_yukselt()          # sessizce geçmeli
