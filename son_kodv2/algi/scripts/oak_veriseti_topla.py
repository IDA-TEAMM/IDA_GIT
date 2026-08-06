#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
GİRDAP İDA — OAK-D Lite (DepthAI v2) YOLO VERİ SETİ TOPLAYICI
=============================================================
Canlı RGB akışından YOLO eğitimi için ETİKETLENMEYE HAZIR .jpg kareler kaydeder.
Video dosyası DEĞİL — doğrudan kareler.

İki mod, tek pencere:
  • MANUEL   : SPACE / S  → o anki kareyi kaydet (en iyi kareleri SEN seç)
  • OTOMATİK : A ile aç/kapa → her --interval saniyede bir kare kaydeder
               (tekneyle gezerken elini kullanmadan çeşitli kare toplamak için)

Çıktı düzeni (YOLO'ya hazır):
  <out>/
    images/    kare_00001.jpg ...     (kaydedilen kareler)
    labels/    (BOŞ — labelImg / Roboflow / CVAT ile .txt etiketler buraya gelir)
    data.yaml  (YOLO config — sınıf sırası GİRDAP sözleşmesine göre KİLİTLİ, aşağıya bak)
    README.txt (etiketleme + eğitim notları)
    manifest.csv (her kare: dosya, oturum, ISO zaman, saat güvenilir mi, çözünürlük, fps)

Aynı klasöre tekrar çalıştırırsan numaradan DEVAM eder (üzerine YAZMAZ);
manifest.csv de eklemeli yazılır → birden çok deniz oturumu sonradan ayrıştırılır.

DENİZ OTURUMU (PC YOK, EKRAN YOK): systemd servisiyle açılışta kendi başına
başlar — bkz. scripts/girdap-veriseti.service ve docs/veriseti_deniz_oturumu.md.
⚠ Algı node'u ile AYNI ANDA çalışamaz (tek OAK, tek süreç açabilir).

🔴 SINIF SIRASI (değiştirme — üç yerde birden değişmesi gerekir):
   0 = kenar_dubasi (turuncu RAL 2003)   1 = engel_dubasi (sarı RAL 1026)
   Sebep: karar yığınının `/perception/buoys` sözleşmesi "0"=kenar, "1"=engel;
   ayrıca algı kodu sınıfı İSİMDEN çözüyor (`_sinif_indeksleri_coz`, "kenar"/"engel"
   alt dizgisini arar). İsim/sıra bozulursa model sessizce yanlış sınıf yayınlar.
   Aynı sıra: data.yaml + eğitim + NN Archive export'u (416x416).

🔴 ÇÖZÜNÜRLÜK 4:3 OLMALI (varsayılan 1352x1014):
   OAK-D Lite CAM_A = IMX214, native 4:3. Deploy node'u 12MP + ispScale(1,3) =
   1352x1014 (tam 4:3) kare üretip 416x416 NN'e **SIKIŞTIRARAK** veriyor
   (setPreviewKeepAspectRatio(False) → letterbox şeridi YOK, letterbox payı 0).
   16:9 istersen depthai varsayılanı CROP olduğu için sensörün altı/üstü kesilir
   → veri seti deploy'dan DAR dikey FOV'la toplanır, model sahada görmediği
   açıları görür.

Kullanım:
  python3 oak_veriseti_topla.py                                   # önizlemeli
  python3 oak_veriseti_topla.py --no-preview --interval 2.0       # ekransız (suda/servis)
  python3 oak_veriseti_topla.py --min-fark 0                      # benzerlik filtresi KAPALI

Kontroller (önizleme penceresi açıkken):
  SPACE / S : kareyi kaydet         A : otomatik toplamayı aç/kapa
  Q / ESC   : çık

DepthAI **v2 (2.30.0.0)** API'si — 2026-08-05'te v3'ten taşındı. Sistem tek
sürümde: v3 firmware'inde mono kameralar açılmıyor (stereo %0 ÖLÇÜLDÜ), algı
node'u derinliğe muhtaç. Bu toplayıcı yalnız RGB kullandığı için v3'te de
koşuyordu; iki sürümü yan yana yaşatmamak için o da v2'ye alındı.
🔴 Bu dosya v3'te ÇALIŞMAZ (ColorCamera/XLinkOut/getOutputQueue = v2 idiomu).
Sadece RGB (YOLO 2D için derinlik gerekmez → stereo açılmaz, daha az yük).
"""
import argparse
import re
import shutil
import sys
import time
from pathlib import Path

import cv2
import depthai as dai

# Ortak OAK bağlantı katmanı (USB2'ye zorlama + kilit kurtarma + termal denetim).
# Betik systemd'den MUTLAK yolla çalıştığı için paket dizinini sys.path'e ekliyoruz
# (testlerin `scripts`i eklemesiyle aynı desen).
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "girdap_ida_algi"))
from girdap_ida_algi import oak_baglanti as ob  # noqa: E402
import numpy as np

# ---------------------------------------------------------------- sabitler
# Sınıf sırası tek yerde: data.yaml + README aynı kaynaktan yazılır.
SINIFLAR = ["kenar_dubasi", "engel_dubasi"]

# Benzerlik ölçümü bu küçük griye indirgenmiş karede yapılır (ucuz + gürültüye dayanıklı).
KUCUK_EN = 160

# 🔴 EN/BOY ORANI = 4:3 — pazarlıksız. Gerekçe (2026-08-04 doğrulaması):
#   • OAK-D Lite CAM_A = IMX214, native 4208x3120 = 4:3 (HFOV 69°).
#   • Deploy node'u (duba_gecis_navigator.py) 12MP + ispScale(1,3) = 1352x1014
#     = 4:3 kare üretir, NN'e 416x416 **SIKIŞTIRARAK** girer
#     (setPreviewKeepAspectRatio(False)). Yani model DEPLOY'da 4:3 sahnenin
#     1:1'e ezilmiş halini görüyor — eğitim de aynı ön işlemeyle yapılmalı.
#   • 16:9 (1920x1080) istersek depthai varsayılanı CROP (resizeMode=0) olduğu için
#     sensörün ALTINI/ÜSTÜNÜ keser → veri seti dar dikey FOV'la toplanır, deploy'da
#     karenin altında beliren yakın dubalar eğitimde HİÇ görülmemiş olur.
SENSOR_ORANI = 4.0 / 3.0


# ------------------------------------------------------------------ yardımcılar
def res_ayristir(s: str):
    """'1920x1080' -> (1920, 1080). Hatalıysa anlaşılır mesajla düşer."""
    m = re.fullmatch(r"\s*(\d+)\s*[xX*]\s*(\d+)\s*", s)
    if not m:
        raise argparse.ArgumentTypeError(f"--res '1920x1080' biçiminde olmalı, gelen: {s!r}")
    return int(m.group(1)), int(m.group(2))


def cihaz_bekle(bulucu, timeout_s: float, uyku_s: float = 2.0, log=print) -> bool:
    """OAK görünene kadar bekle. Saf/enjekte edilebilir (kamerasız test edilir).

    NEDEN: boot'ta USB geç enumere olabilir ya da kamera sonradan takılır.
    Script hemen exit(1) verirse systemd birkaç denemede pes eder ve SAHADA
    (monitör/SSH yok) tek kare toplanmadan koşu biter — sessiz başarısızlık.

    `bulucu()` cihaz listesi döndürür. timeout_s <= 0 → beklemeden tek deneme.
    """
    t0 = time.monotonic()
    yazildi = False
    while True:
        if bulucu():
            return True
        if timeout_s <= 0 or (time.monotonic() - t0) >= timeout_s:
            return False
        if not yazildi:
            log(f"[…] OAK görünmüyor — {timeout_s:g} sn'ye kadar bekleniyor "
                f"(kamerayı şimdi takabilirsin)")
            yazildi = True
        time.sleep(uyku_s)


def oran_uyumlu(w: int, h: int, tolerans: float = 0.02) -> bool:
    """İstenen çözünürlük deploy geometrisiyle (4:3) uyumlu mu? Saf fonksiyon."""
    if h <= 0:
        return False
    return abs((w / h) - SENSOR_ORANI) <= tolerans


def kucult_gri(frame, en: int = KUCUK_EN):
    """Kareyi benzerlik karşılaştırması için küçük gri görüntüye indir.

    Saf fonksiyon (kamerasız test edilebilir). Dalga/ışık gürültüsü küçültmede
    zaten ortalanır; asıl sahne değişimi kalır.
    """
    h, w = frame.shape[:2]
    if w <= 0 or h <= 0:
        raise ValueError("boş kare")
    yeni = (en, max(1, int(round(h * en / w))))
    kucuk = cv2.resize(frame, yeni, interpolation=cv2.INTER_AREA)
    if kucuk.ndim == 3:
        kucuk = cv2.cvtColor(kucuk, cv2.COLOR_BGR2GRAY)
    return kucuk.astype(np.float32)


def yeterince_farkli(onceki, simdiki, esik: float) -> bool:
    """Bu kare son KAYDEDİLEN kareden yeterince farklı mı?

    Saf fonksiyon. `onceki is None` → ilk kare, her zaman kaydedilir.
    `esik <= 0` → filtre kapalı (her kare geçer).
    Ölçüt: küçültülmüş gri kareler arası ORTALAMA MUTLAK FARK (0-255).
    """
    if esik <= 0 or onceki is None:
        return True
    if onceki.shape != simdiki.shape:
        return True
    return float(np.mean(np.abs(onceki - simdiki))) >= esik


def bos_disk_gb(yol: Path) -> float:
    """Verilen yolun bulunduğu diskteki boş alan (GB)."""
    return shutil.disk_usage(str(yol)).free / (1024 ** 3)


# --------------------------------------------------------- oturum / manifest
# NEDEN MANIFEST: dosya adı yalnız sıra numarası taşıyor (kare_00001.jpg) —
# YOLO araçları bunu sever, DEĞİŞTİRME. Ama deniz oturumunda hangi karenin
# hangi saatte/hangi koşuda toplandığını bilmek ŞART: veri seti çeşitliliği
# (farklı ışık/saat) ancak böyle denetlenebilir ve tek bir klasöre biriken
# birden çok oturum sonradan ayrıştırılabilir. Çözüm: adı bozmadan yanına
# manifest.csv yazmak.
MANIFEST_ADI = "manifest.csv"
MANIFEST_BASLIK = "dosya,oturum,iso_zaman,saat_guvenilir,genislik,yukseklik,fps"

# Jetson'da RTC pili yoksa saat boot'ta GERİDE açılır (bu projede ölçüldü:
# ~2 ay). Kare yine toplanır ama zaman etiketi yalan olur — sessiz kalmasın.
SAAT_ALT_SINIR = 1767225600.0        # 2026-01-01T00:00:00Z


def saat_guvenilir_mi(epoch_s: float, alt_sinir: float = SAAT_ALT_SINIR) -> bool:
    """Sistem saati makul mü? (RTC'siz Jetson boot'ta geçmişte açılabilir)"""
    return float(epoch_s) >= alt_sinir


def oturum_kimligi(epoch_s: float) -> str:
    """Bu koşunun kimliği: '20260804_204500' (yerel saat).

    Aynı klasöre birden çok deniz oturumu birikeceği için gerekli.
    """
    return time.strftime("%Y%m%d_%H%M%S", time.localtime(epoch_s))


def manifest_satiri(dosya: str, oturum: str, epoch_s: float,
                    w: int, h: int, fps: int) -> str:
    """Tek karenin manifest satırı (CSV, sonunda \\n)."""
    iso = time.strftime("%Y-%m-%dT%H:%M:%S", time.localtime(epoch_s))
    guv = "1" if saat_guvenilir_mi(epoch_s) else "0"
    return f"{dosya},{oturum},{iso},{guv},{int(w)},{int(h)},{int(fps)}\n"


def manifest_ac(out: Path):
    """manifest.csv'yi ekleme modunda aç; yeni dosyaya başlık yaz."""
    yol = out / MANIFEST_ADI
    yeni = not yol.exists()
    fh = yol.open("a", encoding="utf-8")
    if yeni:
        fh.write(MANIFEST_BASLIK + "\n")
        fh.flush()
    return fh


def sonraki_index(images_dir: Path, prefix: str, manifest_yolu: Path = None) -> int:
    """images/ VE manifest.csv'deki en büyük {prefix}_NNNNN indeksinin bir fazlası
    (yeniden çalıştırınca kaldığı yerden devam et, üzerine yazma).

    🔴 NEDEN MANIFEST DE OKUNUYOR (2026-08-05'te sahada yaşandı):
    Eskiden yalnız images/ taranıyordu. Kareler diske kopyalanıp images/
    temizlenince sayaç 1'e SIFIRLANDI ve yeni kareler eski manifest
    satırlarının adlarını aldı → 146 dosya adı manifest'te İKİ kez, iki farklı
    oturum/zaman ile. Sonuç: o kareler için "hangi saatte/ışıkta çekildi"
    sorusunun iki cevabı var → ışık/saat çeşitliliği analizi (veri setinin
    asıl değeri) o kareler için yapılamaz. Manifest EKLEMELİ yazıldığı için
    silinen karelerin izi orada durur; sayacı ondan da beslemek çakışmayı
    kökten keser. Kare atmak beklenen bir işlem ("bol topla, fazlasını at").
    """
    en_buyuk = 0
    desen = re.compile(rf"{re.escape(prefix)}_(\d+)\.jpg$")
    for p in images_dir.glob(f"{prefix}_*.jpg"):
        m = desen.search(p.name)
        if m:
            en_buyuk = max(en_buyuk, int(m.group(1)))

    # Manifest bozuk/yarım olsa bile toplama DURMAMALI (denizde müdahale yok).
    if manifest_yolu is not None and manifest_yolu.exists():
        try:
            with manifest_yolu.open("r", encoding="utf-8", errors="replace") as fh:
                for satir in fh:
                    m = desen.search(satir.split(",", 1)[0].strip())
                    if m:
                        en_buyuk = max(en_buyuk, int(m.group(1)))
        except OSError:
            pass
    return en_buyuk + 1


# ─────────────── v2 sensör modları — ÖLÇÜLDÜ, tahmin DEĞİL (2026-08-05) ───────
# depthai v2'de ColorCamera keyfi çözünürlük ALMAZ (v3'teki `requestOutput((w,h))`
# esnekliği yok): sensör modu sabit listeden seçilir, sonra ISP ile ölçeklenir.
#
# 🔴 BU CİHAZDA (OAK-D Lite / IMX214) HANGİ MOD GERÇEKTEN KARE ÜRETİYOR —
#    enum'da olması çalıştığı anlamına GELMİYOR. 8 sn'lik tarama sonucu:
#      THE_1440X1080  → 0 kare (SESSİZ; hata bile vermiyor)   🔴
#      THE_1352X1012  → 0 kare                                 🔴
#      THE_2024X1520  → RuntimeError                           🔴
#      THE_1080_P / 1200_P / 800_P / 720_P → hepsi 1920×1080 (16:9)
#      THE_12_MP → 4056×3040 (4:3) · THE_13_MP → 4208×3120 · THE_4_K → 3840×2160
#    Luxonis `depthai-core#712` aynı sınıf sorun: desteklenmeyen sensör
#    çözünürlüğü OAK-D-Lite'ta fatal/sessiz hata veriyor.
#
# 🔑 SEÇİM: THE_12_MP + ispScale 1/3 → **1352×1014, TAM 4:3, 9,9 FPS** (ölçüldü;
#    istenen 10 birebir geliyor, 30 istenirse 18,6). Tam sensör okunup ISP'de
#    küçültüldüğü için **KIRPMA YOK → tam FOV**. Ham 12MP'yi USB'den göndermek
#    2 FPS'e düşürüyordu; darboğaz sensör değil USB'ydi, ispScale onu çözüyor.
#    Akış 1352×1014×1,5×10 = 20,6 MB/s (eski 1440×1080'in 23,3'ünden AZ).
# ⚠️ 16:9 modları (1920×1080) sensörü DİKEY KIRPAR → veri seti deploy'dan dar
#    FOV'la toplanır, model sahada öğrendiği ölçeği bulamaz. Veri seti için ❌.
#
# Değerler: (istenen_w, istenen_h) → (sensör_modu, ispScale veya None, gerçek_boyut)
V2_COZUNURLUKLER = {
    (1352, 1014): ("THE_12_MP", (1, 3), (1352, 1014)),   # ← VARSAYILAN: 4:3, tam FOV
    (1440, 1080): ("THE_12_MP", (1, 3), (1352, 1014)),   # eski istek → en yakın 4:3
    (1403, 1040): ("THE_13_MP", (1, 3), (1403, 1040)),   # 4:3 DEĞİL (1,349)
    (4056, 3040): ("THE_12_MP", None,   (4056, 3040)),   # ham 12MP — USB2'de ~2 FPS
    (1920, 1080): ("THE_1080_P", None,  (1920, 1080)),   # 16:9 — veri seti için ❌
    (3840, 2160): ("THE_4_K",   None,   (3840, 2160)),
}


def v2_sensor_cozunurlugu(w: int, h: int):
    """(w,h) → (sensör_modu_enum, ispScale|None, gerçek_boyut).

    ⚠️ Dönen `gerçek_boyut` istenenden FARKLI olabilir (ISP ölçekleme) — çağıran
    manifest'e GERÇEK boyutu yazmalı, istenen değil.
    Desteklenmeyen değerde ValueError; mesaj sahada okunacak şekilde yazıldı.
    """
    import depthai as _dai
    kayit = V2_COZUNURLUKLER.get((int(w), int(h)))
    if kayit is None:
        secenek = ", ".join(f"{a}x{b}" for a, b in sorted(V2_COZUNURLUKLER))
        raise ValueError(
            f"{w}x{h} bu cihazda v2 ile ÜRETİLEMİYOR. Desteklenen: {secenek}. "
            f"Veri seti için önerilen: 1352x1014 (12MP + ispScale 1/3 = tam 4:3, "
            f"tam FOV, ölçülen 9,9 FPS)."
        )
    ad, isp, gercek = kayit
    return getattr(_dai.ColorCameraProperties.SensorResolution, ad), isp, gercek


def klasor_hazirla(out: Path, prefix: str):
    """images/ + labels/ + data.yaml + README.txt oluştur (varsa dokunma)."""
    images = out / "images"
    labels = out / "labels"
    images.mkdir(parents=True, exist_ok=True)
    labels.mkdir(parents=True, exist_ok=True)

    isim_satirlari = "".join(f"  {i}: {ad}\n" for i, ad in enumerate(SINIFLAR))

    data_yaml = out / "data.yaml"
    if not data_yaml.exists():
        data_yaml.write_text(
            "# GİRDAP İDA — YOLO veri seti config\n"
            "# 🔴 SINIF SIRASI KİLİTLİ — değiştirme:\n"
            "#    karar yığını sözleşmesi /perception/buoys: \"0\"=kenar, \"1\"=engel\n"
            "#    algı kodu sınıfı İSİMDEN çözer (\"kenar\"/\"engel\" alt dizgisi).\n"
            "#    Aynı sıra NN Archive export'unda da geçerli olmalı (416x416).\n"
            "# val: eğitimden ÖNCE ayrı bir bölme yap (Roboflow/CVAT böler);\n"
            "#      aşağıdaki 'val: images' sadece taslak — aynı kalırsa metrikler yalan söyler.\n"
            "path: .\n"
            "train: images\n"
            "val: images\n"
            "names:\n" + isim_satirlari,
            encoding="utf-8",
        )

    readme = out / "README.txt"
    if not readme.exists():
        readme.write_text(
            "GİRDAP İDA — YOLO veri seti (OAK-D Lite ile toplandı)\n"
            "=====================================================\n\n"
            "images/  : kaydedilen kareler (.jpg)\n"
            "labels/  : YOLO etiketleri (.txt) — HER görselle AYNI adta, örn:\n"
            "           images/kare_00001.jpg  ->  labels/kare_00001.txt\n"
            "data.yaml: sınıf isimleri + train/val yolları\n\n"
            "SINIF SIRASI (KİLİTLİ):\n"
            "  0 = kenar_dubasi  (turuncu RAL 2003, parkur kenarı)\n"
            "  1 = engel_dubasi  (sarı RAL 1026, engel)\n"
            "  Bu sıra karar yığınının /perception/buoys sözleşmesiyle aynıdır ve\n"
            "  NN Archive export'unda da AYNI kalmalıdır. Parkur-3 hedef dubaları\n"
            "  eklenirse 2,3,4 olarak SONA eklenir (mevcut sıra bozulmaz).\n\n"
            "Etiketleme (birini seç):\n"
            "  • Roboflow (web)  — kolay, otomatik böler/augment eder\n"
            "  • CVAT (web/self) — çok görsel için hızlı\n"
            "  • labelImg (yerel): pip install labelImg && labelImg images/ classes.txt labels/\n"
            "    (labelImg'de 'YOLO' formatını seç)\n\n"
            "YOLO .txt satır biçimi:  <sinif_id> <x_merkez> <y_merkez> <en> <boy>\n"
            "  (hepsi 0-1 aralığında, görsel genişlik/yüksekliğine göre normalize)\n\n"
            "İyi veri seti ipuçları:\n"
            "  • Çeşitlilik: farklı mesafe/açı/ışık/arka plan.\n"
            "  • Toplayıcı benzer kareleri --min-fark eşiğiyle zaten eler.\n"
            "  • Sınıf başına dengeli sayı.\n\n"
            "🔴 ÖN İŞLEME = SIKIŞTIRMA (stretch), LETTERBOX DEĞİL — pazarlıksız:\n"
            "  Deploy 4:3 kareyi (1352x1014) 416x416'ya EZİYOR\n"
            "  (setPreviewKeepAspectRatio(False)). Ultralytics'in eğitim varsayılanı\n"
            "  ise LETTERBOX'tır (en-boy korur, gri şerit ekler). İkisi ayrışırsa\n"
            "  model eğitimde YUVARLAK, sahada ~1,33x DİKEY UZAMIŞ duba görür →\n"
            "  sistematik geometri kayması, uzak/küçük dubada belirgin. BELİRTİ VERMEZ.\n\n"
            "  Çözüm (en temiz): kareleri eğitimden ÖNCE 416x416'ya EZ. Kare girdide\n"
            "  Ultralytics'in letterbox'ı hiçbir şey yapmaz (r=1.000, dolgu YOK).\n"
            "  YOLO etiketleri normalize (0-1) olduğu için yeniden boyutlandırma\n"
            "  etiket değerlerini DEĞİŞTİRMEZ.\n"
            "  • Roboflow kullanıyorsan: Preprocessing > Resize > 'Stretch to' 416x416\n"
            "    ('Fit within' DEĞİL — o dolgu ekler = letterbox).\n"
            "  • Eğitim: yolo detect train data=data.yaml model=yolo11n.pt imgsz=416\n"
            "    (mimari 06.08.2026'da ölçülerek seçildi: v8n 21,6 / v11n 19,9 FPS,\n"
            "     fark %8; v11n'in +2,2 mAP'i tercih edildi — darboğaz menzil.)\n"
            "  • Export: 416x416, düz .blob, 4 SHAVE, superblob KAPALI.\n",
            encoding="utf-8",
        )
    return images, labels


def hud_yaz(frame, satirlar, org=(10, 26)):
    """Sol üste yarı saydam arka planlı çok satırlı bilgi yaz."""
    x, y = org
    for i, (metin, r) in enumerate(satirlar):
        yy = y + i * 26
        (tw, th), _ = cv2.getTextSize(metin, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 2)
        cv2.rectangle(frame, (x - 6, yy - th - 6), (x + tw + 6, yy + 6), (0, 0, 0), -1)
        cv2.putText(frame, metin, (x, yy), cv2.FONT_HERSHEY_SIMPLEX, 0.6, r, 2)


# ------------------------------------------------------------------------- main
def main():
    ap = argparse.ArgumentParser(
        description="OAK-D Lite (DepthAI v2) ile YOLO veri seti (kare) toplayıcı.",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    ap.add_argument("--out", type=str, default="~/girdap_veriseti",
                    help="Çıktı klasörü (images/ + labels/ + data.yaml burada oluşur)")
    ap.add_argument("--res", type=res_ayristir, default="1352x1014",
                    help="Kayıt çözünürlüğü (GENİŞxYÜKSEK). Varsayılan 1352x1014 = "
                         "12MP + ispScale 1/3: TAM 4:3, KIRPMA YOK (tam FOV), "
                         "ölçülen 9,9 FPS. 4:3 OLMALI — deploy (416x416'ya "
                         "SIKIŞTIRMA) ile aynı FOV. 16:9 verirsen sensörün altı/üstü kırpılır, "
                         "veri seti deploy'a uymaz. 1440x1080 istenirse otomatik "
                         "1352x1014'e düşer (o sensör modu bu cihazda kare üretmiyor).")
    ap.add_argument("--fps", type=int, default=10,
                    help="Kamera sensör FPS'i. Varsayılan 10 = 23,3 MB/s @1440x1080; "
                         "20 (46,7 MB/s) USB2'de ÇÖKTÜĞÜ ölçüldü — yükseltme.")
    ap.add_argument("--interval", type=float, default=2.0,
                    help="OTOMATİK modda kaç saniyede bir kare denensin")
    ap.add_argument("--zorunlu-aralik", type=float, default=10.0,
                    help="KALP ATIŞI: bu kadar saniyedir hiç kare kaydedilmediyse "
                         "benzerlik filtresini AŞ ve yine de kaydet (0 = kapalı). "
                         "Filtre uzaktaki dubayı göremediği için gerekli.")
    ap.add_argument("--min-fark", type=float, default=3.0,
                    help="Benzer kare filtresi: son kaydedilenle ortalama mutlak fark "
                         "(0-255) bu değerin altındaysa kare ATLANIR. 0 = filtre kapalı")
    ap.add_argument("--max-kare", type=int, default=0,
                    help="Bu oturumda en fazla kaç kare (0 = sınırsız). Sınıra gelince temiz çıkar")
    ap.add_argument("--min-bos-gb", type=float, default=5.0,
                    help="Diskte bu kadar GB'den az boş kalırsa temiz çıkar")
    ap.add_argument("--prefix", type=str, default="kare",
                    help="Dosya adı ön eki (kare_00001.jpg)")
    ap.add_argument("--jpg-quality", type=int, default=95, help="JPEG kalitesi (0-100)")
    ap.add_argument("--cihaz-bekle", type=float, default=30.0,
                    help="OAK görünmüyorsa kaç saniye beklensin (0 = bekleme, hemen çık). "
                         "Serviste yüksek tut: boot'ta geç enumere olur / sonradan takılır")
    ap.add_argument("--no-preview", action="store_true",
                    help="Ekransız çalış (SSH/servis): sadece OTOMATİK topla, Ctrl+C ile dur")
    ap.add_argument("--auto-start", action="store_true",
                    help="Açılışta OTOMATİK mod zaten AÇIK başlasın")
    args = ap.parse_args()

    out = Path(args.out).expanduser().resolve()
    w, h = args.res
    images_dir, _ = klasor_hazirla(out, args.prefix)
    idx = sonraki_index(images_dir, args.prefix, out / MANIFEST_ADI)
    headless = args.no_preview
    auto = headless or args.auto_start   # ekransızsa otomatik şart (klavye yok)

    print(f"[i] Çıktı klasörü : {out}")
    print(f"[i] Çözünürlük    : {w}x{h} @ {args.fps} FPS, JPEG kalite {args.jpg_quality}")
    print(f"[i] Başlangıç no  : {idx:05d}  (klasörde zaten {idx-1} kare varsa devam)")
    print(f"[i] Benzer filtre : min-fark={args.min_fark:g}"
          f"{' (KAPALI)' if args.min_fark <= 0 else ''}"
          f" | kalp atışı={args.zorunlu_aralik:g}s"
          f"{' (KAPALI)' if args.zorunlu_aralik <= 0 else ''}")
    print(f"[i] Sınırlar      : max-kare={args.max_kare or 'yok'} | "
          f"min-boş-disk={args.min_bos_gb:g} GB (şu an {bos_disk_gb(out):.1f} GB boş)")
    print(f"[i] Mod           : {'EKRANSIZ (otomatik)' if headless else 'önizlemeli'}"
          f"  |  otomatik={'AÇIK' if auto else 'kapalı'} ({args.interval:g}s)")
    print(f"[i] Sınıflar      : " + ", ".join(f"{i}={a}" for i, a in enumerate(SINIFLAR)))
    oturum = oturum_kimligi(time.time())
    manifest = manifest_ac(out)
    print(f"[i] Oturum        : {oturum}  (manifest: {out / MANIFEST_ADI})")
    if not saat_guvenilir_mi(time.time()):
        print(f"\n[!!] SİSTEM SAATİ ŞÜPHELİ: {time.strftime('%Y-%m-%d %H:%M')}\n"
              f"     Jetson'da RTC pili yoksa saat boot'ta geride açılır. Kareler\n"
              f"     yine toplanır ve manifest'e saat_guvenilir=0 yazılır, ama\n"
              f"     ışık/saat çeşitliliği analizi bu oturumda YAPILAMAZ.\n"
              f"     Denize açılmadan önce saati düzelt: sudo date -s '...'\n", flush=True)
    if not oran_uyumlu(w, h):
        print(f"\n[!!] UYARI: {w}x{h} oranı {w/h:.3f} — deploy 4:3 (1.333) çalışıyor.\n"
              f"     Sensör (IMX214) 4:3; 4:3 dışı istekte kare KIRPILIR ve bu veri setiyle\n"
              f"     eğitilen model sahada farklı FOV görür. Önerilen: --res 1440x1080\n", flush=True)

    # --- OAK görünene kadar bekle (boot'ta geç enumere olabilir) ---
    if not cihaz_bekle(dai.Device.getAllAvailableDevices, args.cihaz_bekle):
        print(f"\n[HATA] {args.cihaz_bekle:g} sn içinde OAK-D bulunamadı.")
        print("       • USB'ye takılı ve enerji alıyor mu? (lsusb'de 03e7 görünmeli)")
        print("       • Başka bir program (algı node'u) kamerayı tutuyor olabilir.")
        sys.exit(1)

    # --- DepthAI v2 pipeline (RGB-only) ---
    # 🔴 SÜRÜM KARARI (2026-08-05, Eyüp: "depthai v2'ye yükselt"): sistem tek
    # sürümde — **depthai 2.30.0.0**. Sebep v3'ün firmware hatası: v3.7.1/3.8.0'da
    # mono kameralar açılmıyor → STEREO %0 (ölçüldü). v3'te RGB çalıştığı için bu
    # toplayıcı v3'te de koşuyordu, ama algı node'u derinliğe muhtaç ve v2'de her
    # şey çalışıyor (stereo 29,7 FPS). İki sürümü yan yana yaşatmak yerine tek
    # sürüme inildi. ⇒ BU DOSYA ARTIK v3'TE ÇALIŞMAZ (v3'e dönülürse geri taşı).
    #
    # 🔴 USB2'ye ZORLANIYOR (2026-08-05 ölçümü): bu Jetson'da SuperSpeed linki
    # `tegra-xusb`ın U1/U2 pazarlığında çöküyor → cihaz bootlanıp ROM'a düşüyor
    # (`X_LINK_DEVICE_NOT_FOUND`). HIGH'a zorlanınca 5/5 açılış; otomatik
    # pazarlıkta ~6 denemede 1. Ayrıntı: girdap_ida_algi/oak_baglanti.py
    # Kilit gelirse `dayanikli_ac` sudo'suz USB reset atıp yeniden dener —
    # denizde fişe kimse uzanamayacağı için bu ZORUNLU.
    try:
        cozunurluk, isp_scale, gercek = v2_sensor_cozunurlugu(w, h)
        if (gercek[0], gercek[1]) != (w, h):
            print(f"[!] {w}x{h} bu cihazda doğrudan YOK → en yakın 4:3 mod "
                  f"kullanılıyor: {gercek[0]}x{gercek[1]} (tam FOV, kırpma yok).")
            w, h = gercek                     # manifest ve akış hesabı GERÇEĞİ göstersin
        pipeline = dai.Pipeline()
        cam = pipeline.create(dai.node.ColorCamera)
        cam.setBoardSocket(dai.CameraBoardSocket.CAM_A)
        cam.setResolution(cozunurluk)
        if isp_scale:
            # Tam sensör okunur, ISP'de küçültülür → KIRPMA YOK, tam FOV korunur.
            # Ham 12MP'yi USB'den göndermek FPS'i 2'ye düşürüyordu (ölçüldü).
            cam.setIspScale(*isp_scale)
        cam.setFps(args.fps)
        xout = pipeline.create(dai.node.XLinkOut)
        xout.setStreamName("rgb")
        # `isp` çıkışı = setIspScale sonrası boyut (ÖLÇÜLDÜ: 1352×1014 @ 9,9 FPS).
        # ⚠️ Kare gelmemesi durumunda ilk şüpheli çıkış portu DEĞİL, SENSÖR MODUdur:
        # 05.08'de `isp` de `video` de `preview` de 0 kare verdi — üçünün de altında
        # THE_1440X1080 vardı ve o mod bu cihazda hiç kare üretmiyor (sessizce).
        cam.isp.link(xout.input)
        # v2'de cihaz pipeline ile birlikte açılır (v3'teki pipeline.start() YOK).
        dev = ob.dayanikli_ac(
            lambda: dai.Device(pipeline, dai.UsbSpeed.HIGH),
            kaydet=lambda m: print(f"[!] {m}", flush=True),
        )
        rgb_q = dev.getOutputQueue("rgb", maxSize=4, blocking=False)
    except Exception as e:                                  # cihaz yok / çözünürlük hatası
        print("\n[HATA] Kamera pipeline başlatılamadı.")
        print(f"       Sebep: {type(e).__name__}: {e}")
        print("       • OAK-D Lite USB'ye takılı ve enerji alıyor mu?  (lsusb'de 03e7 görünmeli)")
        print("       • Başka bir program (algı node'u) kamerayı tutuyor olabilir.")
        print(f"       • depthai sürümü v2 mi? (bu dosya v2 API'si kullanır: "
              f"{getattr(dai, '__version__', '?')})")
        sys.exit(1)

    # --- USB link hızı: sessiz çökme tuzağı (2026-08-05'te bu Jetson'da ölçüldü) ---
    # ⚠ 05.08 AKŞAMI DÜZELTİLDİ: burada eskiden "link SUPER değilse UYAR" yazıyordu.
    # Artık HIGH'ı BİLEREK istiyoruz (USB3 bu platformda kararsız), o yüzden asıl
    # risk link hızı değil BANT GENİŞLİĞİ: USB2'nin pratik sınırı ~35-40 MB/s.
    # Ölçüm (1440x1080 NV12): 10 fps=23,3 · 15 fps=35,0 · 20 fps=46,7 MB/s —
    # 20 fps'te cihaz ÇÖKTÜ (crash dump), 15 fps sığdı. Denizde ekran/SSH YOK,
    # bu yüzden hesap journal'a yazılır: kıyı kontrolünde tek bakışta görülsün.
    USB2_GUVENLI_MBS = 30.0        # 35-40 tavanına pay bırakan eşik
    try:
        usb_hiz = dev.getUsbSpeed()          # v2: cihazın kendisinden (v3'te pipeline'dan)
        ad = str(usb_hiz).rsplit(".", 1)[-1]
        akis_mbs = w * h * 1.5 * args.fps / 1e6           # NV12 = 1,5 bayt/piksel
        print(f"[i] USB link      : {ad}  (akış ≈ {akis_mbs:.1f} MB/s)  [HIGH istendi]")
        if ad in ("LOW", "FULL"):
            print(f"\n[!!] USB 1.x LİNKİ ({ad}) — bu hızda kare akmaz.\n"
                  f"     Kablo/port arızalı; USB'yi çıkar tak.\n", flush=True)
        elif akis_mbs > USB2_GUVENLI_MBS:
            onerilen = int(USB2_GUVENLI_MBS * 1e6 / (w * h * 1.5))
            print(f"\n[!!] AKIŞ ÇOK YÜKSEK: {akis_mbs:.1f} MB/s — USB2 pratik sınırı ~35-40.\n"
                  f"     Bu ayarda cihazın oturum ortasında ÇÖKTÜĞÜ ölçüldü ve sahada\n"
                  f"     bunu görecek kimse yoktur. Öneri: --fps {onerilen} veya altı.\n",
                  flush=True)
    except Exception as e:      # hız okunamazsa toplama durmasın — kare > tanılama
        print(f"[!] USB hızı okunamadı ({type(e).__name__}: {e}) — toplama devam ediyor.")

    # --- VPU sıcaklığı: cihazda OTOMATİK KISMA YOK, doğrudan çöküyor ---
    # Luxonis: çip anma sınırı 105 °C, gözlenen çökme 125 °C; OAK-D *Lite* küçük
    # soğutuculu (azami ortam ~40 °C). Deniz oturumu güneş altında ve saatler
    # sürebilir → sıcaklık periyodik loglanır, eşik aşılırsa uyarı basılır.
    _sicaklik_dev = None
    try:
        _sicaklik_dev = dev              # v2: cihaz zaten elimizde
        _c0 = ob.vpu_sicakligi(_sicaklik_dev)
        if _c0 is not None:
            print(f"[i] VPU sıcaklık  : {_c0:.1f} °C  "
                  f"(uyarı {ob.SICAKLIK_UYARI:.0f} / kritik {ob.SICAKLIK_KRITIK:.0f})")
    except Exception:
        pass

    if not headless:
        pencere = "GIRDAP veri seti — SPACE=kaydet  A=otomatik  Q=cik"
        cv2.namedWindow(pencere, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(pencere, min(w, 1280), int(min(w, 1280) * h / w))

    def kaydet(frame):
        """Kareyi tam çözünürlükte diske yaz + manifest satırını ekle."""
        nonlocal idx
        yol = images_dir / f"{args.prefix}_{idx:05d}.jpg"
        cv2.imwrite(str(yol), frame, [cv2.IMWRITE_JPEG_QUALITY, int(args.jpg_quality)])
        # Manifest HER karede flush'lanır: denizde güç kesilirse en fazla son
        # satır kaybolur, o ana kadarki oturum kaydı sağlam kalır.
        # 🔴 GERÇEK kare boyutu yazılır, istenen değil: ISP ölçekleme yüzünden ikisi
        # ayrışabiliyor (1440x1080 istenip 1352x1014 alınması gibi). Manifest
        # veri setinin tek doğrusu — istenen değeri yazmak sessiz yalan olurdu.
        _h, _w = frame.shape[:2]
        manifest.write(manifest_satiri(yol.name, oturum, time.time(), _w, _h, args.fps))
        manifest.flush()
        idx += 1
        return yol

    kaydedilen = 0
    atlanan = 0                # benzerlik filtresinin elediği kare sayısı
    kalp_atisi = 0             # filtre elediği hâlde "zorunlu aralık" yüzünden alınan kare
    onceki_kucuk = None        # son KAYDEDİLEN karenin küçük gri hâli
    son_auto = 0.0
    son_kayit_yaz = 0.0        # ekranda "KAYDEDILDI" flaşı için zaman damgası
    son_ozet = time.monotonic()
    dur_sebep = None
    t_fps, sayac, fps_txt = time.monotonic(), 0, "..."
    son_sicaklik_t = 0.0       # VPU sıcaklığı en son ne zaman okundu
    son_sicaklik_durum = "normal"
    SICAKLIK_OKUMA_PERIYODU = 60.0   # sn — NE SIKLIKTA okunacağı (sıcaklık DEĞERİ değil).
                                 # Eşikler oak_baglanti.py: uyarı 85, kritik 95 °C.

    try:
        # v2: cihaz kapanana kadar dön (v3'teki `pipeline.isRunning()` YOK —
        # 2026-08-05 taşımasında bu satır atlandı ve servis tam burada çöktü:
        # AttributeError, cihaz açılmış/USB HIGH/sıcaklık okunmuş haldeyken).
        while not dev.isClosed():
            msg = rgb_q.tryGet()
            if msg is None:
                if not headless and cv2.waitKey(1) & 0xFF in (ord("q"), 27):
                    break
                if headless:
                    time.sleep(0.003)      # ekransızken CPU'yu boşa döndürme (Jetson yük/termal)
                continue
            frame = msg.getCvFrame()
            simdi = time.monotonic()

            # --- VPU termal denetimi (cihazda otomatik kısma YOK) ---
            # Güneş altındaki uzun oturumda ısınma sonradan gelir; açılışta bir kez
            # bakmak yetmez. Durum DEĞİŞTİĞİNDE basılır → journal spam'i olmaz.
            if _sicaklik_dev is not None and simdi - son_sicaklik_t >= SICAKLIK_OKUMA_PERIYODU:
                son_sicaklik_t = simdi
                _c = ob.vpu_sicakligi(_sicaklik_dev)
                _durum = ob.sicaklik_durumu(_c)
                if _durum != son_sicaklik_durum:
                    son_sicaklik_durum = _durum
                    if _durum == "kritik":
                        print(f"\n[!!] VPU {_c:.1f} °C — KRİTİK. Çip anma sınırı 105 °C,\n"
                              f"     gözlenen çökme 125 °C ve cihaz KENDİNİ KISMIYOR.\n"
                              f"     Kamerayı gölgele; toplama sürüyor ama çökme riski var.\n",
                              flush=True)
                    elif _durum == "uyari":
                        print(f"[!] VPU {_c:.1f} °C — uyarı eşiği aşıldı "
                              f"({ob.SICAKLIK_UYARI:.0f} °C). Gölge/hava akışı kontrol et.",
                              flush=True)
                    else:
                        print(f"[i] VPU {_c:.1f} °C — normale döndü.", flush=True)

            # anlık FPS
            sayac += 1
            if simdi - t_fps >= 1.0:
                fps_txt = f"{sayac / (simdi - t_fps):.1f} FPS"
                t_fps, sayac = simdi, 0

            kaydet_bu_kare = False
            zorla = False              # manuel kayıt: benzerlik filtresini uygulama

            # otomatik mod: interval dolduysa, kare boş/siyah değilse ve öncekinden
            # yeterince farklıysa kaydet
            if auto and (simdi - son_auto) >= args.interval:
                son_auto = simdi
                if frame.mean() > 1.0:          # açılıştaki simsiyah kareyi atla
                    if yeterince_farkli(onceki_kucuk, kucult_gri(frame), args.min_fark):
                        kaydet_bu_kare = True
                    elif (args.zorunlu_aralik > 0
                          and simdi - son_kayit_yaz >= args.zorunlu_aralik):
                        # KALP ATIŞI — benzerlik filtresi elese bile bu kadar süredir
                        # hiç kayıt yoksa kareyi yine de al.
                        # NEDEN (2026-08-05 ölçümü, tahmin değil): filtre TÜM karenin
                        # ortalama mutlak farkına bakıyor. 1440x1080'de 30 cm'lik duba
                        # 10 m'de ~31x52 px = karenin %0,1'i → ortalama farka katkısı
                        # yalnız 0,11, eşik ise 2,0. Yani UZAKTAKİ DUBA KADRAJA
                        # GİRDİĞİNDE KARE ELENİR — tam da en değerli kare. Masada
                        # ölçüldü: sahne durunca 61 sn boyunca tek kare alınmadı.
                        # Denizde müdahale yok; "hiç kare almama" hâli kabul edilemez.
                        kaydet_bu_kare = True
                        kalp_atisi += 1
                    else:
                        atlanan += 1

            if not headless:
                disp = frame.copy()
                hud_yaz(disp, [
                    (f"{fps_txt} | {w}x{h}", (0, 255, 0)),
                    (f"Kaydedilen: {kaydedilen}  Atlanan: {atlanan}  Sonraki: {idx:05d}",
                     (0, 255, 255)),
                    (f"OTOMATIK: {'ACIK' if auto else 'kapali'} ({args.interval:g}s)",
                     (0, 255, 0) if auto else (170, 170, 170)),
                ])
                if simdi - son_kayit_yaz < 0.35:
                    cv2.rectangle(disp, (0, 0), (disp.shape[1] - 1, disp.shape[0] - 1),
                                  (0, 255, 0), 8)
                    cv2.putText(disp, "KAYDEDILDI", (disp.shape[1] // 2 - 130, 60),
                                cv2.FONT_HERSHEY_SIMPLEX, 1.2, (0, 255, 0), 3)
                cv2.putText(disp, "SPACE=kaydet  A=otomatik  Q=cik",
                            (10, disp.shape[0] - 14), cv2.FONT_HERSHEY_SIMPLEX,
                            0.6, (255, 255, 255), 2)
                cv2.imshow(pencere, disp)

                k = cv2.waitKey(1) & 0xFF
                if k in (ord("q"), 27):
                    break
                elif k in (ord("s"), ord("S"), 32):     # 32 = SPACE
                    kaydet_bu_kare, zorla = True, True
                elif k in (ord("a"), ord("A")):
                    auto = not auto
                    son_auto = simdi
                    print(f"[i] Otomatik toplama {'AÇILDI' if auto else 'kapatıldı'}")

            if kaydet_bu_kare:
                # sınır kontrolleri — sınıra gelince TEMİZ çık (systemd yeniden başlatmasın)
                if args.max_kare and kaydedilen >= args.max_kare:
                    dur_sebep = f"--max-kare {args.max_kare} sınırına ulaşıldı"
                    break
                if bos_disk_gb(out) < args.min_bos_gb:
                    dur_sebep = f"disk {args.min_bos_gb:g} GB'nin altına indi"
                    break

                yol = kaydet(frame)
                kaydedilen += 1
                onceki_kucuk = kucult_gri(frame)
                son_kayit_yaz = simdi
                if headless or kaydedilen % 10 == 0 or zorla:
                    print(f"[+] {kaydedilen:4d} kare  ->  {yol.name}", flush=True)

            # ekransızken 60 sn'de bir özet (journal'dan sonradan okunur)
            if headless and simdi - son_ozet >= 60.0:
                son_ozet = simdi
                print(f"[=] özet: {kaydedilen} kayıt ({kalp_atisi} kalp atışı) / "
                      f"{atlanan} benzer-atlandı / "
                      f"{fps_txt} / disk {bos_disk_gb(out):.1f} GB boş", flush=True)

    except KeyboardInterrupt:
        print("\n[i] Ctrl+C — kapatılıyor.")
    finally:
        # v2: cihazı kapat (v3'teki pipeline.stop() karşılığı). Cihaz teardown'da
        # çökebiliyor (3/3 gözlendi) → manifest'i HER HÂLDE kapat, son satır gitmesin.
        try:
            dev.close()
        except Exception:
            pass
        manifest.close()
        if not headless:
            cv2.destroyAllWindows()

    if dur_sebep:
        print(f"\n[!] DURDURULDU: {dur_sebep}")
    print(f"\n[✓] Bitti. Bu oturumda {kaydedilen} kare kaydedildi, "
          f"{atlanan} benzer kare atlandı.")
    print(f"    Toplam klasörde ~{idx-1} kare: {images_dir}")
    print(f"    Sonraki adım — etiketle: README.txt (Roboflow/CVAT/labelImg).")


if __name__ == "__main__":
    main()
