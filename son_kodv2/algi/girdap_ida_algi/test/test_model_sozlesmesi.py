"""Model ↔ kod sözleşmesi: `NN_GIRIS` · teslim edilen blob · `FPS_UYARI_ESIK`.

🔴 NEDEN: bu üç sayı **sessizce** bozulabilen türden — hiçbiri çalışma anında
hata basmaz, üçü de doğrudan puana bağlıdır.

1. `NN_GIRIS` ↔ `models/config.json` girişi. Ayrı düşerlerse preview bir
   boyutta, blob başka boyutta olur ⇒ **çöp tespit**. Node'un kendi
   `_blob_denetle()`'si bunu açılışta yakalar ama **yalnız UYARIR** (tanı kodu
   node'u öldürmemeli — bilinçli tasarım), yani sahada belirti vermez.
   12.08'de 416 → 512 geçişinde blob ve sabit AYNI commit'te gitti; bu test
   ikisinin bir daha ayrılmamasını kilitliyor.

2. `FPS_UYARI_ESIK < FPS`. 12.08'de yakalanan tuzak: `FPS` 11 → 8 indirildi,
   eşik 8.0'da kalsaydı `olculen_fps < 8.0` koşulu **her karede** yanardı.
   Bizde bunun yazılı dersi var (09.08, `mono_menzil`): *"her zaman yanan
   alarm alarm değildir"* — sürekli yanan bir uyarı, gerçek bir düşüşü
   görünmez kılar.

3. Blob **4 shave**. Fazlası cihazda *"compiled for N shaves, only 4
   available"* ile **HİÇ yüklenmez** (hız kaybı değil — model yok).
"""
import json
import os
import sys

import pytest

_KOK = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

pytest.importorskip("depthai", reason="depthai kurulu değil")

from girdap_ida_algi import duba_gecis_navigator as dgn  # noqa: E402

_BLOB = os.path.join(_KOK, "models", "yolo11n_duba_rvc2.blob")
_CONFIG = os.path.join(_KOK, "models", "config.json")


def test_config_girisi_nn_giris_ile_ayni():
    """`models/config.json` girişi ↔ koddaki `NN_GIRIS` — İKİSİ BİRLİKTE değişir."""
    with open(_CONFIG, encoding="utf-8") as f:
        cfg = json.load(f)
    sekil = cfg["model"]["inputs"][0]["shape"]        # [1, 3, H, W] (NCHW)
    assert sekil[2] == sekil[3] == dgn.NN_GIRIS, (
        f"config.json girişi {sekil} ↔ NN_GIRIS {dgn.NN_GIRIS} — uyuşmuyor. "
        "Blob ve sabit AYNI commit'te değişmeli; ayrı kalırsa node yalnız "
        "UYARIR (öldürmez) ve sahada çöp tespit üretir."
    )


def test_blob_basligi_nn_giris_ile_ayni():
    """Gerçek blob başlığı — config.json değil, **teslim edilen dosyanın kendisi**."""
    import depthai as dai

    blob = dai.OpenVINO.Blob(_BLOB)
    boyutlar = [list(g.dims) for g in blob.networkInputs.values()]
    assert any(dgn.NN_GIRIS in b for b in boyutlar), (
        f"blob girişi {boyutlar} içinde NN_GIRIS {dgn.NN_GIRIS} yok"
    )


def test_blob_dort_shave():
    """12MP tam-FOV + stereo açıkken NN'e kalan bütçe 4; fazlası HİÇ yüklenmez."""
    import depthai as dai

    blob = dai.OpenVINO.Blob(_BLOB)
    assert blob.numShaves <= 4, (
        f"numShaves={blob.numShaves} > 4 — cihaz blob'u REDDEDER "
        "('compiled for N shaves, only 4 available'), model hiç yüklenmez."
    )


def test_fps_uyari_esigi_hedefin_altinda():
    """Eşik hedefin ÜSTÜNDE olursa alarm her karede yanar ⇒ alarm olmaktan çıkar."""
    assert dgn.FPS_UYARI_ESIK < dgn.FPS, (
        f"FPS_UYARI_ESIK={dgn.FPS_UYARI_ESIK} ≥ FPS={dgn.FPS} — uyarı SÜREKLİ "
        "yanar. Ders (09.08, mono_menzil): her zaman yanan alarm alarm değildir."
    )


def test_fps_uyari_esigi_gercek_dususu_yakalayabilecek_kadar_yuksek():
    """Çok düşük eşik de körlük: gerçek bir çöküşü hiç bildirmez."""
    assert dgn.FPS_UYARI_ESIK >= dgn.FPS * 0.6, (
        f"FPS_UYARI_ESIK={dgn.FPS_UYARI_ESIK}, hedefin %60'ının altında — "
        "boru hattı yarı hıza düşse bile uyarı çıkmaz."
    )


def test_sinif_isimleri_cozulebiliyor():
    """Node sınıfı İSİMDEN çözer; isimler bozulursa yedek sabitlere düşer ⇒
    turuncu↔sarı SESSİZCE takas olabilir (Ç2)."""
    with open(_CONFIG, encoding="utf-8") as f:
        cfg = json.load(f)
    isimler = cfg["model"]["heads"][0]["metadata"]["classes"]
    kenar, engel, isimle = dgn._sinif_indeksleri_coz(isimler)
    assert isimle, f"sınıflar {isimler} isimden çözülemedi — yedek sabitlere düşer"
    assert kenar != engel
