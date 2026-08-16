"""Dosya-1 birleştirme aracı — segmentler TEK mp4'e inmeli (md 4.2).

🔴 Şartname taraması (11.08.2026, doğrulanmış PDF sha256 09116afe…):
   md 4.2 "Veriler **3 dosya** olacak şekilde teslim edilecektir."
   Biz çökme dayanımı için 120 sn'lik segment yazıyoruz (20 dk tur ⇒ 10 dosya).
   md 5.5.4.3.5: teslim edilmeyen her dosya için **5 ceza puanı**, karaya
   alımdan itibaren **20 dakika** içinde.
"""
import importlib.util
import os
import subprocess
import sys

import pytest

_BETIK = os.path.join(os.path.dirname(__file__), "..", "..", "scripts",
                      "dosya1_birlestir.py")
_spec = importlib.util.spec_from_file_location("dosya1_birlestir", _BETIK)
db = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(db)

cv2 = pytest.importorskip("cv2")
np = pytest.importorskip("numpy")

_FFMPEG = subprocess.run(["which", "ffmpeg"], capture_output=True).returncode == 0


def _segment_yaz(yol, kare=6, w=160, h=120):
    vw = cv2.VideoWriter(yol, cv2.VideoWriter_fourcc(*"mp4v"), 2.0, (w, h))
    for _ in range(kare):
        vw.write(np.full((h, w, 3), 40, np.uint8))
    vw.release()
    return kare


def test_saglam_segment_kare_sayisi_okunur(tmp_path):
    y = str(tmp_path / "seg_0001.mp4")
    n = _segment_yaz(y)
    assert db.kare_sayisi(y) == n


def test_bozuk_segment_None_doner(tmp_path):
    """Ani güç kesintisinde moov atomu yazılmaz — dosya açılamaz.

    `finally` bu senaryoda ÇALIŞMAZ (SIGKILL / fiş çekme yakalanamaz), yani
    son segmentin bozuk olması BEKLENEN durumdur. Araç bunu fark edip
    ATLAMALI; tek bozuk segment yüzünden teslim dosyası hiç üretilememesi
    kabul edilemez.
    """
    y = tmp_path / "seg_0002.mp4"
    y.write_bytes(b"\x00\x00\x00\x18ftypmp42")     # moov YOK
    assert db.kare_sayisi(str(y)) is None


@pytest.mark.skipif(not _FFMPEG, reason="ffmpeg yok")
def test_bozuk_segment_atlanir_kalanlar_birlesir(tmp_path, monkeypatch, capsys):
    otr = tmp_path / "session_20260820_141500"
    otr.mkdir()
    toplam = _segment_yaz(str(otr / "seg_0001.mp4"))
    toplam += _segment_yaz(str(otr / "seg_0002.mp4"))
    (otr / "seg_0003.mp4").write_bytes(b"\x00\x00\x00\x18ftypmp42")   # BOZUK

    monkeypatch.setattr(sys, "argv",
                        ["x", "--kayit-dizin", str(tmp_path),
                         "--oturum", "session_20260820_141500"])
    assert db.main() == 0

    cikti = otr / "Dosya1_islenmis_kamera.mp4"
    assert cikti.exists(), "TEK dosya üretilmeli (md 4.2)"
    assert db.kare_sayisi(str(cikti)) == toplam, "sağlam segmentlerin TAMAMI girmeli"
    assert "BOZUK" in capsys.readouterr().out, "bozuk segment sessizce atlanmamalı"


@pytest.mark.skipif(not _FFMPEG, reason="ffmpeg yok")
def test_hic_saglam_segment_yoksa_hata(tmp_path, monkeypatch):
    otr = tmp_path / "session_20260820_150000"
    otr.mkdir()
    (otr / "seg_0001.mp4").write_bytes(b"bozuk")
    monkeypatch.setattr(sys, "argv",
                        ["x", "--kayit-dizin", str(tmp_path),
                         "--oturum", "session_20260820_150000"])
    assert db.main() == 1, "üretilemiyorsa 0 dönüp 'oldu' izlenimi VERMEMELİ"


def test_segment_yoksa_hata(tmp_path, monkeypatch):
    (tmp_path / "session_20260820_160000").mkdir()
    monkeypatch.setattr(sys, "argv", ["x", "--kayit-dizin", str(tmp_path)])
    assert db.main() == 1


# ── OTURUM SEÇİMİ: isim sırası YALAN SÖYLEYEBİLİR (2026-08-13) ────────────
# 🔴 Oturum dizini adı (`session_%Y%m%d_%H%M%S`) algı node'u AÇILIRKEN üretilir.
# Jetson'ın RTC pili yok ve BAYAT saatle açıldığı iki kez ölçüldü; saati
# `girdap-saat-gec.service` ancak GPS fix'ten sonra kuruyor. Sonuç: GERÇEK
# koşunun dizini geçmiş bir tarihle isimlenip, prova dizini isim sıralamasında
# "daha yeni" görünüyor. Eski `sorted(...)[-1]` provayı seçiyordu ⇒ teslim
# edilen Dosya-1 yanlış koşu ⇒ md 4.2 / md 5.5.4.3.5 = 5 CEZA PUANI.
def _oturum(kok, ad, mtime=None):
    d = kok / ad
    d.mkdir()
    (d / "seg_0001.mp4").write_bytes(b"x")
    if mtime is not None:
        os.utime(d, (mtime, mtime))
    return d


def test_bayat_saatli_oturum_ADI_yerine_DISKTEKI_son_secilir(tmp_path):
    import time as _t
    _oturum(tmp_path, "session_20260820_120000", mtime=_t.time() - 86400)  # prova
    gercek = _oturum(tmp_path, "session_20260813_010000")                  # koşu
    secilen = db.en_son_oturum([str(p) for p in tmp_path.glob("session_*")])
    assert os.path.basename(secilen) == gercek.name, (
        "isim sırasına güvenilirse PROVA seçilir — teslim yanlış koşu olur")


def test_celiskide_SESSIZ_kalinmaz(tmp_path, capsys):
    import time as _t
    _oturum(tmp_path, "session_20260820_120000", mtime=_t.time() - 86400)
    _oturum(tmp_path, "session_20260813_010000")
    db.en_son_oturum([str(p) for p in tmp_path.glob("session_*")])
    cikti = capsys.readouterr().out
    assert "ÇELİŞİYOR" in cikti, "tanı kodu susmamalı — kıyıda fark edilmeli"


def test_celiski_yoksa_gurultu_YAPMAZ(tmp_path, capsys):
    import time as _t
    _oturum(tmp_path, "session_20260820_120000", mtime=_t.time() - 86400)
    yeni = _oturum(tmp_path, "session_20260820_150000")
    secilen = db.en_son_oturum([str(p) for p in tmp_path.glob("session_*")])
    assert os.path.basename(secilen) == yeni.name
    assert "ÇELİŞİYOR" not in capsys.readouterr().out, (
        "her zaman yanan uyarı uyarı değildir (09.08 dersi)")
