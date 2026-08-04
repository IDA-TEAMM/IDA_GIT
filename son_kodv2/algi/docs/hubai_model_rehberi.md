# HubAI Model Üretim Rehberi — Gazebonew.pt → yolo11n_duba_rvc2.tar.xz

> Amaç: eğitilmiş YOLO modelimizi (`~/girdap_yolo/Gazebonew.pt`, YOLOv11n)
> OAK-D Lite'ın VPU'sunda koşacak **RVC2 NN Archive**'a (.tar.xz) çevirmek.
> ⏳ Zamanlama: **video SONRASI** iş (videoda algı yok — md 3.3); bu rehber
> o gün hazır olsun diye yazıldı.
>
> Nerede yapılır: **PC'de** (model dosyası + internet burada). Jetson'da değil.
> Kaynak: Luxonis resmî dokümanları (2026-07-11'de teyit edildi; linkler sonda).

## 0. Bilinmesi gereken üç gerçek

1. **Model .pt olarak doğrudan yükleniyor** — ONNX'e çevirmek GEREKMEZ
   (desteklenen YOLO sürümlerinden olmalı; v5–v11 destekli, bizimki v11 ✓).
2. **Sınıf sırası modelin içinde `{0: 'Engel Dubasi', 1: 'Kenar Dubasi'}`**
   (data.pkl'den birebir teyitli — bekleyen_girdiler §B1b). Koddaki eski
   sabitlerin TERSİ; kod artık isimden çözüyor ama üretilen arşivin
   config.json'ı MUTLAKA doğrulanacak (adım 4).
3. **416×416 + 6 shave:** giriş boyutu arşivin içinde tanımlanır (kod
   ayarlamaz); shave=6 çünkü VPU'da stereo derinlik de koşuyor (varsayılan
   8, stereo'ya yer bırakmaz).

## 1. HubAI hesabı + API anahtarı (bir kez)

1. <https://hub.luxonis.com> → hesap aç (ücretsiz).
2. Hesap ayarlarından **API Key** üret, kopyala.
3. PC'de:
   ```bash
   pip install --user hubai-sdk
   export HUBAI_API_KEY="<anahtarın>"     # kalıcı istersen ~/.bashrc'ye
   ```

## 2. Çevirme (tek Python betiği)

```python
import os
from hubai_sdk import HubAIClient

client = HubAIClient(api_key=os.getenv("HUBAI_API_KEY"))
r = client.convert.RVC2(
    path="/home/eyup/girdap_yolo/Gazebonew.pt",
    yolo_version="yolov11",
    yolo_input_shape=[416, 416],          # saha bandı 10-14 FPS bu boyutla
    yolo_class_names=["Engel Dubasi", "Kenar Dubasi"],  # SIRA = modelin sırası!
    number_of_shaves=6,                   # stereo ile birlikte koşacak
)
print("indirilen arşiv:", r.downloaded_path)
```

⚠️ `yolo_class_names` sırası modelin GERÇEK sırasıdır (0=Engel, 1=Kenar).
Ters yazarsan kod isimden çözdüğü için yine tersine döner ve **Parkur-2
sessizce bozulur** — adım 4'teki doğrulama bu yüzden pazarlıksız.

> SDK sorun çıkarırsa B planı: hub.luxonis.com web arayüzünden model yükle →
> RVC2'ye convert et → .tar.xz indir (aynı ayarlar). Eski tools.luxonis.com
> yolu aşamalı kapanıyor; HubAI resmi yol.

## 3. Adlandır + parmak izi

```bash
mv <indirilen> yolo11n_duba_rvc2.tar.xz
sha256sum yolo11n_duba_rvc2.tar.xz        # çıktıyı models/README.md tablosuna işle
```

## 4. DOĞRULAMA (Jetson'a taşımadan önce, PC'de)

```bash
tar -xJf yolo11n_duba_rvc2.tar.xz -O config.json | python3 -m json.tool | head -40
```
Bakılacaklar:
- `classes` (heads[0].metadata altında): **["Engel Dubasi", "Kenar Dubasi"]**
  — bu sırayla. Farklıysa DUR, adım 2'yi kontrol et.
- `n_classes: 2` (80 görürsen stok COCO'yu çevirmişsin — yanlış dosya!).
- Giriş boyutu 416×416.

## 5. Jetson'a taşı + saha teyidi

```bash
# USB bellekle taşı (WiFi yok):
mkdir -p /home/girdap/models
cp /media/.../yolo11n_duba_rvc2.tar.xz /home/girdap/models/
```
1. `python3 scripts/duba_kamera_test.py` → görüntüde kutular; terminalde
   `Model sınıf sırası: [...]` logunu OKU (turuncu dubaya KENAR demeli).
2. FPS 10-14 bandında mı bak.
3. 🗑️ **`scripts/kamera_goruntu_test.py`'yi SİL** (modelsiz dönemin geçici
   testiydi — bekleyen_girdiler §B madde 5).
4. `ros2 launch girdap_ida_algi algi.launch.py` → `/perception/buoys` yayını.

## Kaynaklar

- [HubAI ile RVC2 çevirme (DepthAI v3 resmî doküman)](https://docs.luxonis.com/software-v3/ai-inference/conversion/rvc-conversion/online/hubai/)
- [HubAI SDK (GitHub)](https://github.com/luxonis/hubai-sdk)
- [HubAI Model Conversion genel](https://docs.luxonis.com/cloud/hubai/features/model-conversion/)
- [YOLO basit çevirme aracının legacy olduğu notu](https://docs.luxonis.com/software-v3/ai-inference/conversion/rvc-conversion/online/yolo/)
