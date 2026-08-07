"""USB teslim toplayıcı testleri (md 4.2 / md 5.5.4.3.5).

Her test bir EMNİYET ya da CEZA iddiasını dondurur.
"""

from __future__ import annotations

from pathlib import Path

import pytest

from prototype.teslim.toplayici import (
    KALEMLER, MANIFEST_ADI, RAPOR_ADI, Bulgu, kalemleri_bul, kopyala,
    manifest_metni, rapor_metni, topla_ve_yaz,
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
        (d / "lidar_kumeleme.mp4").write_bytes(b"L" * 4096)
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
    # Kalem adı ŞARTNAMEDEN gelir ("Diğer Otonomi Sensörleri Veri Seti"),
    # bizim uydurduğumuz "Dosya 1b" gibi bir addan DEĞİL.
    assert any("Diğer Otonomi Sensörleri" in ad for ad in rapor.eksik_zorunlu)
    assert rapor.tahmini_ceza == 5
    metin = (usb / RAPOR_ADI).read_text(encoding="utf-8")
    assert "EKSİK ZORUNLU" in metin and "5 puan" in metin


def test_USB_de_HICBIR_SEY_silinmez(tmp_path):
    """Yanlış USB takılsa bile veri kaybı olmamalı."""
    logs = _kur(tmp_path / "logs")
    usb = tmp_path / "usb"
    usb.mkdir()
    kisisel = usb / "kisisel_belgeler"
    kisisel.mkdir()
    (kisisel / "onemli.txt").write_text("dokunma")
    topla_ve_yaz(logs, usb, zaman_damgasi="20260807_150000")

    assert (kisisel / "onemli.txt").read_text() == "dokunma"
    # Kokte SARTNAMEDEKI adlar durur, sarmalayici klasor UYDURULMAZ
    assert (usb / "Dosya1_Otonomi_Sensorleri_Veri_Seti").is_dir()
    assert (usb / "Diger_Otonomi_Sensorleri_Veri_Seti").is_dir()
    assert (usb / "Dosya2_Arac_Telemetri_Verisi.csv").is_file()
    assert (usb / "Dosya3_Lokal_Harita_Cost_Map_Engel_Haritasi.mp4").is_file()


def test_onceki_kosum_ARSIVLENIR_silinmez(tmp_path):
    """md 5.5.3.1 — 1 yeniden baslama hakki var; ayni USB'ye iki kosum duser.

    Hakemin kokte gordugu DAIMA son kosum olmali; eskisi silinmemeli.
    """
    logs = _kur(tmp_path / "logs")
    usb = tmp_path / "usb"
    usb.mkdir()
    eski = usb / "Diger_Otonomi_Sensorleri_Veri_Seti"
    eski.mkdir()
    (eski / "ilk_kosum.mp4").write_bytes(b"ESKI")

    kopyala(kalemleri_bul(logs), usb, zaman_damgasi="X")

    ars = usb / "onceki_kosum_X" / "Diger_Otonomi_Sensorleri_Veri_Seti"
    assert (ars / "ilk_kosum.mp4").read_bytes() == b"ESKI", "eski kosum silindi"
    yeni = usb / "Diger_Otonomi_Sensorleri_Veri_Seti"
    assert (yeni / "lidar_kumeleme.mp4").exists()
    assert not (yeni / "ilk_kosum.mp4").exists(), "kokte iki kosum karisti"


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
    man = (usb / MANIFEST_ADI).read_text(encoding="utf-8")
    assert man.startswith("# sha256")
    assert "lidar_kumeleme.mp4" in man
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


def test_mp4_YOK_ama_PNG_yedegi_VARSA_toplanir_ve_BAGIRIR(tmp_path):
    """🔴 Codec açılamamışsa kareler PNG'de olur; toplanmazsa teslim TAMAMEN
    kaybolur (mp4 zaten yok). Rapor da çevirmeyi hatırlatmalı."""
    logs = _kur(tmp_path / "logs", lidar=False)          # mp4 üretilememiş
    png = logs / "lidar" / "oturum_20260807_143000" / "lidar_kumeleme_png"
    png.mkdir(parents=True)
    (png / "kare_00000.png").write_bytes(b"P" * 256)
    usb = tmp_path / "usb"
    usb.mkdir()

    rapor, bulgular = topla_ve_yaz(logs, usb)

    hedef = usb / "Diger_Otonomi_Sensorleri_PNG_YEDEK_mp4e_cevrilecek"
    assert (hedef / "kare_00000.png").exists(), "PNG yedeği USB'ye alınmadı"
    metin = (usb / RAPOR_ADI).read_text(encoding="utf-8")
    assert "ÇEVİRMEDEN TESLİM ETME" in metin


def test_PNG_yedegi_OTOMATIK_mp4e_cevriliyor(tmp_path):
    """🔑 Teslim zincirindeki SON insan adımı kaldırıldı.

    Codec yoksa kaydedici PNG'ye düşüyor; şartname mp4 istiyor. Dönüşüm elle
    bırakılsaydı 20 dakikalık teslim penceresinde atlanırdı (o akışta durup
    dosya okuyan bir adım YOK) → 5 ceza. USB takma anına bağlandı.
    """
    import shutil as _sh
    if _sh.which("ffmpeg") is None:
        pytest.skip("ffmpeg yok")
    from PIL import Image
    import numpy as np
    from prototype.mapping.bev_renderer import FPS_ISARET_ADI

    logs = _kur(tmp_path / "logs", lidar=False)          # mp4 ÜRETİLEMEMİŞ
    png = logs / "lidar" / "oturum_20260807_143000" / "lidar_kumeleme_png"
    png.mkdir(parents=True)
    (png / FPS_ISARET_ADI).write_text("2\n", encoding="utf-8")
    rng = np.random.default_rng(0)
    for i in range(6):
        Image.fromarray(
            rng.integers(0, 255, (64, 64, 3), dtype=np.uint8), mode="RGB"
        ).save(png / f"kare_{i:05d}.png")

    usb = tmp_path / "usb"
    usb.mkdir()
    rapor, bulgular = topla_ve_yaz(logs, usb)

    # 1) kaynakta mp4 oluştu (Jetson'da da kalıyor)
    assert (png.parent / "lidar_kumeleme.mp4").is_file()
    # 2) USB'de ŞARTNAMEDEKİ adla duruyor
    assert (usb / "Diger_Otonomi_Sensorleri_Veri_Seti"
            / "lidar_kumeleme.mp4").is_file()
    # 3) artık EKSİK değil, ceza yok
    assert rapor.basarili and rapor.tahmini_ceza == 0
    # 4) USB kökü temiz: PNG_YEDEK klasörü OLUŞMADI
    assert not (usb / "Diger_Otonomi_Sensorleri_PNG_YEDEK_mp4e_cevrilecek"
                ).exists()
    # 5) PNG kaynağı SİLİNMEDİ
    assert (png / "kare_00000.png").exists()
    # 6) rapor ne olduğunu söylüyor
    metin = (usb / RAPOR_ADI).read_text(encoding="utf-8")
    assert "OTOMATİK mp4'e çevrildi" in metin


def test_ffmpeg_YOKSA_PNG_yolu_calismaya_devam_eder(tmp_path, monkeypatch):
    """ffmpeg kurulu değilse teslim yine de kurtarılabilir olmalı."""
    import shutil as _sh
    logs = _kur(tmp_path / "logs", lidar=False)
    png = logs / "lidar" / "oturum_20260807_143000" / "lidar_kumeleme_png"
    png.mkdir(parents=True)
    (png / "kare_00000.png").write_bytes(b"P" * 256)
    usb = tmp_path / "usb"
    usb.mkdir()

    monkeypatch.setattr(_sh, "which", lambda ad: None)   # ffmpeg YOK
    rapor, _ = topla_ve_yaz(logs, usb)

    assert (usb / "Diger_Otonomi_Sensorleri_PNG_YEDEK_mp4e_cevrilecek"
            / "kare_00000.png").exists(), "PNG yedeği USB'ye alınmadı"
    assert any("ffmpeg KURULU DEĞİL" in u for u in rapor.uyarilar)


def test_YABANCI_dosyalar_teslime_GIRMEZ(tmp_path):
    """🔑 Klasöre başka bir yazıcı bulaşırsa teslim kirlenmemeli.

    Eski `ida_topics/local_map_node` aynı `~/girdap_logs/local_map/` altına
    `.pgm`+`.yaml` yazıyordu (07.08'de `local_map_eski/`'ye ayrıldı). Ayrım
    kodda yapıldı ama toplayıcı da kendi başına dayanıklı olmalı: bu test
    HERHANGİ bir yabancı yazıcıya karşı sözleşmeyi dondurur — teslime yalnız
    beklenen uzantılar girer.
    """
    logs = _kur(tmp_path / "logs")
    otr = logs / "local_map" / "oturum_20260807_143000"
    (otr / "map_20260807_143000.pgm").write_bytes(b"P5\n1 1\n255\n\x00")
    (otr / "map_20260807_143000.yaml").write_text("image: map.pgm\n")
    (logs / "local_map" / "map_eski.pgm").write_bytes(b"P5\n")
    usb = tmp_path / "usb"
    usb.mkdir()

    rapor, _ = topla_ve_yaz(logs, usb)

    kopyalanan = {p.suffix for p in usb.rglob("*") if p.is_file()}
    assert ".pgm" not in kopyalanan, "yabancı .pgm teslime girdi"
    assert ".yaml" not in kopyalanan, "yabancı .yaml teslime girdi"
    assert rapor.basarili
