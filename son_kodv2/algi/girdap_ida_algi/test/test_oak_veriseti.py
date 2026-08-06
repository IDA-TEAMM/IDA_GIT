#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""oak_veriseti_topla.py'nin KAMERASIZ test edilebilen parçaları.

Kamera yolu (DepthAI pipeline) burada test EDİLMEZ — o, OAK takılıyken
canlı smoke testiyle doğrulanır. Burada saf fonksiyonlar var:
benzer-kare filtresi, klasör/data.yaml kurulumu, indeks devamlılığı.

Koşum:  python3 -m pytest girdap_ida_algi/test/test_oak_veriseti.py -q
"""
import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "scripts"))
import oak_veriseti_topla as ov  # noqa: E402


# ------------------------------------------------------------ kucult_gri
def test_kucult_gri_tek_kanala_ve_sabit_ene_indirger():
    frame = np.zeros((1080, 1920, 3), dtype=np.uint8)
    kucuk = ov.kucult_gri(frame)
    assert kucuk.ndim == 2                      # gri
    assert kucuk.shape[1] == ov.KUCUK_EN        # en sabit
    assert kucuk.shape[0] == 90                 # 16:9 oranı korunur (160x90)


def test_kucult_gri_bos_karede_hata_verir():
    with pytest.raises(ValueError):
        ov.kucult_gri(np.zeros((0, 0, 3), dtype=np.uint8))


# ------------------------------------------------------- yeterince_farkli
def test_ilk_kare_her_zaman_kaydedilir():
    simdiki = ov.kucult_gri(np.full((1080, 1920, 3), 100, dtype=np.uint8))
    assert ov.yeterince_farkli(None, simdiki, esik=3.0) is True


def test_ayni_sahne_atlanir():
    """Tekne dururken gelen neredeyse-aynı kare KAYDEDİLMEMELİ."""
    a = np.full((1080, 1920, 3), 100, dtype=np.uint8)
    b = a.copy()
    b[::4, ::4] = 101                            # çok küçük gürültü
    assert ov.yeterince_farkli(ov.kucult_gri(a), ov.kucult_gri(b), esik=3.0) is False


def test_sahne_degisince_kaydedilir():
    """Tekne ilerleyip sahne değişince kare KAYDEDİLMELİ."""
    a = np.full((1080, 1920, 3), 60, dtype=np.uint8)
    b = np.full((1080, 1920, 3), 160, dtype=np.uint8)   # ort. fark 100 >> 3
    assert ov.yeterince_farkli(ov.kucult_gri(a), ov.kucult_gri(b), esik=3.0) is True


def test_esik_sifir_filtreyi_kapatir():
    a = np.full((1080, 1920, 3), 100, dtype=np.uint8)
    assert ov.yeterince_farkli(ov.kucult_gri(a), ov.kucult_gri(a), esik=0.0) is True


def test_kismi_degisim_esigin_altinda_kalabilir():
    """Karenin küçük bir köşesi değişirse ortalama fark eşiği aşmaz (istenen)."""
    a = np.full((1080, 1920, 3), 100, dtype=np.uint8)
    b = a.copy()
    b[:100, :100] = 255                          # ~%0.5 alan
    assert ov.yeterince_farkli(ov.kucult_gri(a), ov.kucult_gri(b), esik=3.0) is False


# --------------------------------------------------------- klasor/indeks
def test_klasor_hazirla_yolo_duzenini_kurar(tmp_path):
    images, labels = ov.klasor_hazirla(tmp_path, "kare")
    assert images.is_dir() and labels.is_dir()
    assert (tmp_path / "README.txt").exists()
    data = (tmp_path / "data.yaml").read_text(encoding="utf-8")
    assert "0: kenar_dubasi" in data             # 🔴 sözleşme sırası
    assert "1: engel_dubasi" in data


def test_klasor_hazirla_mevcut_data_yaml_i_EZMEZ(tmp_path):
    (tmp_path / "data.yaml").write_text("elle duzenlenmis", encoding="utf-8")
    ov.klasor_hazirla(tmp_path, "kare")
    assert (tmp_path / "data.yaml").read_text(encoding="utf-8") == "elle duzenlenmis"


def test_sonraki_index_bos_klasorde_1(tmp_path):
    images, _ = ov.klasor_hazirla(tmp_path, "kare")
    assert ov.sonraki_index(images, "kare") == 1


def test_sonraki_index_kaldigi_yerden_devam_eder(tmp_path):
    """Servis restart olunca üzerine YAZMAMALI (F-M.9 dersi: kopma sonrası devam)."""
    images, _ = ov.klasor_hazirla(tmp_path, "kare")
    for n in (1, 2, 17):
        (images / f"kare_{n:05d}.jpg").write_bytes(b"x")
    assert ov.sonraki_index(images, "kare") == 18


def test_sonraki_index_alakasiz_dosyalari_saymaz(tmp_path):
    images, _ = ov.klasor_hazirla(tmp_path, "kare")
    (images / "baska_00099.jpg").write_bytes(b"x")
    (images / "kare_00003.jpg").write_bytes(b"x")
    assert ov.sonraki_index(images, "kare") == 4


# ------------------------------------------- v2 sensör modu (v3'ten taşıma)
def test_v2_cozunurluk_varsayilan_12mp_ispscale_tam_43():
    """ÖLÇÜLDÜ (05.08): THE_1440X1080 enum'da VAR ama bu cihazda 0 kare üretiyor
    (sessizce). Çalışan 4:3 yolu: THE_12_MP + ispScale 1/3 = 1352×1014 @ 9,9 FPS,
    tam FOV. Enum'da olmak ≠ çalışmak — bu test seçimi kilitler."""
    import depthai as dai
    mod, isp, gercek = ov.v2_sensor_cozunurlugu(1352, 1014)
    assert mod is dai.ColorCameraProperties.SensorResolution.THE_12_MP
    assert isp == (1, 3)
    assert gercek == (1352, 1014)


def test_v2_cozunurluk_eski_istek_1440x1080_calisan_moda_duser():
    """Eski servis/komut satırları --res 1440x1080 diyor; bunlar kırılmamalı.
    O sensör modu bu cihazda kare üretmediği için en yakın 4:3'e eşlenir —
    çağıran gerçek boyutu manifest'e yazar."""
    _, isp, gercek = ov.v2_sensor_cozunurlugu(1440, 1080)
    assert isp == (1, 3)
    assert gercek == (1352, 1014)          # istenen DEĞİL, üretilebilen


def test_v2_cozunurluk_desteklenmeyen_deger_net_hata_verir():
    """v2'de keyfi çözünürlük YOK (v3'teki requestOutput esnekliği kayboldu).
    Sahada okunacak hata: desteklenen liste + önerilen mod."""
    with pytest.raises(ValueError) as e:
        ov.v2_sensor_cozunurlugu(1000, 700)
    assert "1352x1014" in str(e.value)


# --------------------------------- indeks çakışması (2026-08-05'te YAŞANDI)
def test_sonraki_index_kareler_silinse_bile_manifestten_devam_eder(tmp_path):
    """🔴 Gerçek olay: kareler kopyalanıp images/ temizlendi → sayaç 1'e
    sıfırlandı → yeni kareler eski manifest satırlarının adlarını aldı →
    146 dosya adı iki farklı oturum/zamanla manifest'te. O kareler için
    "hangi ışıkta çekildi" cevapsız kalır. Manifest silinenlerin izini
    tuttuğu için sayaç ondan da beslenmeli."""
    images, _ = ov.klasor_hazirla(tmp_path, "kare")
    manifest = tmp_path / ov.MANIFEST_ADI
    fh = ov.manifest_ac(tmp_path)
    for n in (1, 2, 146):
        fh.write(ov.manifest_satiri(f"kare_{n:05d}.jpg", "20260805_140134",
                                    1_754_400_000.0, 1440, 1080, 10))
    fh.close()
    # images/ BOŞ (kareler taşındı) — eskiden 1 dönerdi, üzerine yazardı
    assert ov.sonraki_index(images, "kare", manifest) == 147


def test_sonraki_index_disk_manifestten_ileriyse_diski_kullanir(tmp_path):
    """Manifest yarım yazılmış olabilir (güç kesintisi) — büyük olan kazanır."""
    images, _ = ov.klasor_hazirla(tmp_path, "kare")
    (images / "kare_00200.jpg").write_bytes(b"x")
    fh = ov.manifest_ac(tmp_path)
    fh.write(ov.manifest_satiri("kare_00005.jpg", "s", 1_754_400_000.0, 1440, 1080, 10))
    fh.close()
    assert ov.sonraki_index(images, "kare", tmp_path / ov.MANIFEST_ADI) == 201


def test_sonraki_index_manifest_yoksa_veya_bozuksa_cokmez(tmp_path):
    """Denizde müdahale yok: bozuk manifest toplamayı DURDURMAMALI."""
    images, _ = ov.klasor_hazirla(tmp_path, "kare")
    (images / "kare_00007.jpg").write_bytes(b"x")
    assert ov.sonraki_index(images, "kare", tmp_path / "yok.csv") == 8
    (tmp_path / "bozuk.csv").write_bytes(b"\xff\xfe yarim satir\nkare_00050.jpg,")
    assert ov.sonraki_index(images, "kare", tmp_path / "bozuk.csv") == 51


# ------------------------------------------------ cihaz bekleme (saha kritik)
def test_cihaz_zaten_varsa_hemen_doner():
    assert ov.cihaz_bekle(lambda: ["oak"], timeout_s=30, uyku_s=0, log=lambda *a: None) is True


def test_cihaz_sonradan_takilirsa_yakalanir():
    """Boot'ta USB geç enumere olursa / kamera sonradan takılırsa toplayıcı ölmemeli."""
    denemeler = iter([[], [], ["oak"]])
    bulucu = lambda: next(denemeler, ["oak"])  # noqa: E731
    assert ov.cihaz_bekle(bulucu, timeout_s=30, uyku_s=0, log=lambda *a: None) is True


def test_cihaz_hic_gelmezse_timeout_ile_False():
    assert ov.cihaz_bekle(lambda: [], timeout_s=0.05, uyku_s=0.01,
                          log=lambda *a: None) is False


def test_timeout_sifir_beklemeden_doner():
    assert ov.cihaz_bekle(lambda: [], timeout_s=0, uyku_s=99,
                          log=lambda *a: None) is False


# --------------------------------------------------- oran (deploy geometrisi)
def test_43_cozunurlukler_uyumlu():
    """Deploy 4:3 çalışıyor (1352x1014 → 416x416 SIKIŞTIRMA, letterbox payı 0)."""
    assert ov.oran_uyumlu(1440, 1080) is True
    assert ov.oran_uyumlu(640, 480) is True
    assert ov.oran_uyumlu(1920, 1440) is True


def test_169_cozunurluk_UYUMSUZ():
    """16:9 istemek sensörün altını/üstünü kırpar → veri seti deploy'a uymaz."""
    assert ov.oran_uyumlu(1920, 1080) is False
    assert ov.oran_uyumlu(1280, 720) is False


def test_varsayilan_res_43():
    assert ov.oran_uyumlu(*ov.res_ayristir("1440x1080")) is True


def test_oran_bozuk_girdide_cokmez():
    assert ov.oran_uyumlu(640, 0) is False


# ------------------------------------------------------------------ disk
def test_bos_disk_gb_pozitif(tmp_path):
    assert ov.bos_disk_gb(tmp_path) > 0


# ------------------------------------------------------------------- res
def test_res_ayristir_gecerli():
    assert ov.res_ayristir("1920x1080") == (1920, 1080)


def test_res_ayristir_gecersiz():
    import argparse
    with pytest.raises(argparse.ArgumentTypeError):
        ov.res_ayristir("1080p")


# ------------------------------------------- oturum / manifest (deniz oturumu)
# NEDEN: denizde PC yok, ekran yok. Kareler tek klasöre birikecek ve hangi
# oturumdan/saatten geldikleri SONRADAN sorulacak. Manifest bunun tek kaydı.

def test_saat_guvenilir_gecmis_tarihte_False():
    """RTC pilsiz Jetson boot'ta geride açılır (bu projede ~2 ay ölçüldü)."""
    assert ov.saat_guvenilir_mi(0.0) is False              # 1970
    assert ov.saat_guvenilir_mi(1_600_000_000.0) is False  # 2020


def test_saat_guvenilir_makul_tarihte_True():
    assert ov.saat_guvenilir_mi(ov.SAAT_ALT_SINIR) is True
    assert ov.saat_guvenilir_mi(ov.SAAT_ALT_SINIR + 86400) is True


def test_oturum_kimligi_siralanabilir_bicim():
    """Kimlik dizgi sıralamasıyla zaman sırası aynı olmalı (klasörde okunur)."""
    a = ov.oturum_kimligi(ov.SAAT_ALT_SINIR)
    b = ov.oturum_kimligi(ov.SAAT_ALT_SINIR + 3600)
    assert len(a) == len("20260101_000000")
    assert a < b


def test_manifest_satiri_alan_sayisi_baslikla_ayni():
    satir = ov.manifest_satiri("kare_00001.jpg", "20260804_204500",
                               ov.SAAT_ALT_SINIR, 1440, 1080, 20)
    assert satir.endswith("\n")
    assert len(satir.strip().split(",")) == len(ov.MANIFEST_BASLIK.split(","))


def test_manifest_satiri_supheli_saati_isaretler():
    satir = ov.manifest_satiri("kare_00001.jpg", "x", 0.0, 1440, 1080, 20)
    assert satir.strip().split(",")[3] == "0"
    saglam = ov.manifest_satiri("kare_00001.jpg", "x", ov.SAAT_ALT_SINIR,
                                1440, 1080, 20)
    assert saglam.strip().split(",")[3] == "1"


def test_manifest_ac_yeni_dosyaya_baslik_yazar(tmp_path):
    fh = ov.manifest_ac(tmp_path)
    fh.close()
    icerik = (tmp_path / ov.MANIFEST_ADI).read_text(encoding="utf-8")
    assert icerik.strip() == ov.MANIFEST_BASLIK


def test_manifest_ac_mevcut_dosyaya_BASLIK_TEKRARLAMAZ(tmp_path):
    """Servis restart olunca (USB kopması) manifest bozulmamalı — EKLEMELİ."""
    fh = ov.manifest_ac(tmp_path)
    fh.write(ov.manifest_satiri("kare_00001.jpg", "o1", ov.SAAT_ALT_SINIR,
                                1440, 1080, 20))
    fh.close()
    fh2 = ov.manifest_ac(tmp_path)          # ikinci koşu
    fh2.write(ov.manifest_satiri("kare_00002.jpg", "o2", ov.SAAT_ALT_SINIR,
                                 1440, 1080, 20))
    fh2.close()
    satirlar = (tmp_path / ov.MANIFEST_ADI).read_text(encoding="utf-8").splitlines()
    assert satirlar[0] == ov.MANIFEST_BASLIK
    assert len(satirlar) == 3                    # başlık + 2 kare
    assert satirlar.count(ov.MANIFEST_BASLIK) == 1
