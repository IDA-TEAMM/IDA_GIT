"""USB teslim toplayıcı testleri (md 4.2 / md 5.5.4.3.5).

Her test bir EMNİYET ya da CEZA iddiasını dondurur.
"""

from __future__ import annotations

from pathlib import Path

import pytest

from prototype.teslim.toplayici import (
    KALEMLER, Bulgu, kalemleri_bul, kopyala, manifest_metni, rapor_metni,
    topla_ve_yaz,
)


def _kur(kok: Path, *, kamera=True, lidar=True, telemetri=True, harita=True,
         oturum="oturum_20260807_143000") -> Path:
    """Sahte ~/girdap_logs ağacı."""
    if kamera:
        d = kok / "kamera" / oturum
        d.mkdir(parents=True)
        (d / "seg_0000.mp4").write_bytes(b"K" * 2048)
    if lidar:
        d = kok / "lidar" / oturum
        d.mkdir(parents=True)
        (d / "Dosya1b_lidar_kumeleme.mp4").write_bytes(b"L" * 4096)
    if telemetri:
        d = kok / "telemetry"
        d.mkdir(parents=True)
        (d / "telemetri_20260807_143000.csv").write_text("zaman,lat\n1,2\n")
    if harita:
        d = kok / "local_map" / oturum
        d.mkdir(parents=True)
        (d / "Dosya3_lokal_harita.mp4").write_bytes(b"H" * 1024)
        (d / "png_yedek").mkdir()
        (d / "png_yedek" / "frame_00000.png").write_bytes(b"P" * 128)
    return kok


def test_tum_kalemler_bulunur(tmp_path):
    logs = _kur(tmp_path / "logs")
    bulgular = {b.tanim.anahtar: b for b in kalemleri_bul(logs)}
    for a in ("kamera", "lidar", "telemetri", "harita"):
        assert bulgular[a].bulundu, f"{a} bulunamadı"


def test_EKSIK_ZORUNLU_kalem_raporlanir_ve_ceza_hesaplanir(tmp_path):
    """🔑 Betiğin ASIL değeri bu: eksik dosyayı SAHADA söylemek.

    md 5.5.4.3.5 — teslim edilmeyen tanımlı dosya başına 5 ceza.
    """
    logs = _kur(tmp_path / "logs", lidar=False)     # LiDAR mp4 üretilmemiş
    usb = tmp_path / "usb"
    usb.mkdir()
    rapor, bulgular = topla_ve_yaz(logs, usb)

    assert not rapor.basarili
    assert any("LiDAR" in ad for ad in rapor.eksik_zorunlu)
    assert rapor.tahmini_ceza == 5
    metin = (rapor.hedef / "RAPOR.txt").read_text(encoding="utf-8")
    assert "EKSİK ZORUNLU" in metin and "5 puan" in metin


def test_USB_de_HICBIR_SEY_silinmez(tmp_path):
    """Yanlış USB takılsa bile veri kaybı olmamalı."""
    logs = _kur(tmp_path / "logs")
    usb = tmp_path / "usb"
    usb.mkdir()
    kisisel = usb / "kisisel_belgeler"
    kisisel.mkdir()
    (kisisel / "onemli.txt").write_text("dokunma")
    onceki = usb / "GIRDAP_TESLIM_20260101_000000"
    onceki.mkdir()
    (onceki / "eski.txt").write_text("eski kosum")

    topla_ve_yaz(logs, usb, zaman_damgasi="20260807_150000")

    assert (kisisel / "onemli.txt").read_text() == "dokunma"
    assert (onceki / "eski.txt").read_text() == "eski kosum"
    assert (usb / "GIRDAP_TESLIM_20260807_150000").is_dir()


def test_ayni_ada_UZERINE_YAZMAZ(tmp_path):
    logs = _kur(tmp_path / "logs")
    usb = tmp_path / "usb"
    usb.mkdir()
    hedef = usb / "GIRDAP_TESLIM_X"
    (hedef / "lidar").mkdir(parents=True)
    (hedef / "lidar" / "Dosya1b_lidar_kumeleme.mp4").write_bytes(b"ESKI")

    kopyala(kalemleri_bul(logs), usb, zaman_damgasi="X")

    assert (hedef / "lidar" / "Dosya1b_lidar_kumeleme.mp4").read_bytes() == b"ESKI"
    assert (hedef / "lidar" / "Dosya1b_lidar_kumeleme_1.mp4").exists()


def test_YALNIZ_EN_YENI_OTURUM_kopyalanir(tmp_path):
    """Teslim TEK koşuya aittir; eski oturumlar hakemi yanıltır ve süre yer."""
    logs = tmp_path / "logs"
    _kur(logs, oturum="oturum_20260807_120000")
    _kur(logs, kamera=False, telemetri=False, harita=False,
         oturum="oturum_20260807_150000")            # daha yeni lidar

    b = {x.tanim.anahtar: x for x in kalemleri_bul(logs)}["lidar"]
    assert len(b.dosyalar) == 1
    assert "150000" in str(b.dosyalar[0])

    hepsi = {x.tanim.anahtar: x for x in kalemleri_bul(logs, hepsi=True)}["lidar"]
    assert len(hepsi.dosyalar) == 2


def test_dogrulama_ve_manifest(tmp_path):
    logs = _kur(tmp_path / "logs")
    usb = tmp_path / "usb"
    usb.mkdir()
    rapor, _ = topla_ve_yaz(logs, usb)
    assert rapor.basarili and not rapor.bozuk
    man = (rapor.hedef / "MANIFEST.txt").read_text(encoding="utf-8")
    assert man.startswith("# sha256")
    assert "Dosya1b_lidar_kumeleme.mp4" in man
    # her satır: sha256(64) + boyut + yol
    satir = [s for s in man.splitlines() if "lidar" in s][0]
    assert len(satir.split()[0]) == 64


def test_YETERSIZ_ALANDA_HIC_kopyalamaz(tmp_path, monkeypatch):
    """Yarım teslim, hiç teslimden KÖTÜDÜR: dosya var sanılır."""
    import shutil as _sh
    logs = _kur(tmp_path / "logs")
    usb = tmp_path / "usb"
    usb.mkdir()

    class _Az:
        free = 10                                   # 10 bayt

    monkeypatch.setattr(_sh, "disk_usage", lambda p: _Az)
    rapor = kopyala(kalemleri_bul(logs), usb)
    assert rapor.kopyalanan == 0
    assert any("YETERSİZ ALAN" in u for u in rapor.uyarilar)
    assert rapor.eksik_zorunlu, "eksik kalemler raporlanmalı"


def test_bos_log_agacinda_cokmez(tmp_path):
    usb = tmp_path / "usb"
    usb.mkdir()
    rapor, bulgular = topla_ve_yaz(tmp_path / "yok", usb)
    assert rapor.kopyalanan == 0
    zorunlu = [t.ad for t in KALEMLER if t.zorunlu]
    assert set(rapor.eksik_zorunlu) == set(zorunlu)
    assert rapor.tahmini_ceza == 5 * len(zorunlu)


def test_png_yedegi_ZORUNLU_DEGIL(tmp_path):
    """PNG yedeği teslim sözleşmesinde yok → eksikse ceza hesabına girmez."""
    logs = _kur(tmp_path / "logs")
    for p in (logs / "local_map").rglob("*.png"):
        p.unlink()
    usb = tmp_path / "usb"
    usb.mkdir()
    rapor, _ = topla_ve_yaz(logs, usb)
    assert rapor.basarili
    assert rapor.tahmini_ceza == 0
