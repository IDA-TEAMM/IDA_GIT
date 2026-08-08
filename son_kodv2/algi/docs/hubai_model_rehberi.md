# Model Üretim Rehberi — eğitilmiş .pt → OAK-D Lite'ta koşan .blob

> Amaç: kendi veri setimizle eğittiğimiz YOLO modelini OAK-D Lite'ın VPU'sunda
> koşacak hâle getirmek.
> Nerede: **PC'de** (model + internet burada), Jetson'da değil.
> Son güncelleme: **2026-08-06** — bu sürümdeki sayıların tamamı ya gerçek
> cihazda ölçüldü ya da birinci kaynaktan doğrulandı (bkz. §7).

---

## 0. Pazarlıksız dört kısıt (hepsi ölçüldü/doğrulandı)

| # | Kısıt | Neden |
|---|---|---|
| 1 | **shave = 4** | Deploy boru hattında (12MP RGB tam-FOV + stereo) NN'e yalnız **4 shave + 4 CMX** kalıyor. 6-shave blob cihazda *"compiled for 6 shaves, only 4 available"* ile **YÜKLENMEDİ** (05.08, kamerada). |
| 2 | **superblob = KAPALI** (düz `.blob` üret) | Jetson'da **depthai 2.30.0.0** var; bu sürümde **SuperBlob API'si ve NNArchive YOK** (`depthai-core@v2.30.0` ve `depthai-python@v2.30.0.0` kaynağından doğrulandı). Superblob gelirse teknede açılmaz ve **sahada düzeltilemez**. |
| 3 | **giriş 416×416** | Koddaki `NN_GIRIS` ile birlikte değişir. Bu boyutta ölçülen tavan 12,2 FPS, deploy 11. |
| 4 | **ön işleme = SIKIŞTIRMA (stretch), letterbox DEĞİL** | Deploy `setPreviewKeepAspectRatio(False)` kullanıyor → 4:3 kare 416×416'ya eziliyor. **Eğitim de aynı ön işlemeyle yapılmalı**, yoksa model eğitimde yuvarlak / sahada ~1,33× dikey uzamış duba görür. (Ultralytics varsayılanı letterbox'tır — bilerek değiştirilmeli.) |

🔴 **HubAI'nin iki varsayılanı bizim için YANLIŞ:**
`number_of_shaves` varsayılanı **8**, `superblob` varsayılanı **True**
(`hubai-sdk/docs/available_parameters.md`). İkisi de elle değiştirilmezse
model sahada hiç açılmaz. Bu rehberin en kritik satırı budur.

### Mimari: YOLO11n (2026-08-06'da ölçülerek seçildi)
Aynı cihazda, 4 shave, 416×416, iki koşu: **v8n 21,6 FPS · v11n 19,9 FPS**
(fark yalnız %8). v11n +2,2 mAP getiriyor (39,5 vs 37,3) ve bizim darboğazımız
hız değil **menzil/küçük nesne** → **YOLO11n seçildi**.
⚠️ **YOLO26 KULLANILAMAZ:** cihazda çözümlemesi depthai **v3.6.0+** istiyor,
biz 2.30'dayız (v3'e dönmek bu cihazda stereo'yu öldürüyor).
ℹ️ Shave ölçümü: 4→19,9 · 6→21,8 · **8→14,3 FPS** — *çok shave hızlandırmaz,
yavaşlatır* (Luxonis: optimal ≈ mevcudun yarısı).

---

## 1. Sınıf isimleri — değiştirme
Veri seti `data.yaml`'ı şunu yazıyor (`scripts/oak_veriseti_topla.py:77`):
```yaml
names:
  0: kenar_dubasi     # turuncu RAL 2003 — parkur kenarı
  1: engel_dubasi     # sarı RAL 1026    — engel
```
Algı node'u sınıfı **isimden** çözüyor (`_sinif_indeksleri_coz`): isimde
`kenar` / `engel` alt dizgisi arıyor. 06.08'de gerçek `config.json` ile test
edildi — ters sırayı bile doğru çözüyor.
🔴 İsimleri İngilizceye çevirirsen (`orange_buoy` vb.) çözüm **yedek sabitlere
düşer** ve turuncu/sarı yer değiştirebilir → sarı engel "geçit" sanılır,
Parkur-2 sessizce kaybedilir. Node bu durumda ERROR logluyor ama teknede kimse
okumaz. **İsimleri olduğu gibi bırak.**

---

## 2. Eğitim

### 2a. 🔴 ROBOFLOW KULLANIYORSAK — iki ayar hayati
**(1) Preprocessing → Resize → `Stretch to` 416×416.** `Fit within` DEĞİL.
Roboflow'un kendi tanımı: *"**Stretch to** stretches your images to a preferred
pixel-by-pixel dimension with images being square and distorted, but no source
image data is lost"* — deploy'daki `setPreviewKeepAspectRatio(False)` ile
**birebir aynı işlem**. `Fit within` en-boy koruyup **dolgu** ekler (letterbox)
→ eğitim ile deploy geometrisi ayrışır.

**(2) Eğitimi Roboflow'un kendi "Train"ine YAPTIRMA — ağırlık indiremeyebilirsin.**
Roboflow Train ile eğitilen modelin `.pt` ağırlıklarını indirmek **ücretli
plan** gerektiriyor (*"Customers on premium plans can now download weights from
models trained with Roboflow Train"*). Ağırlık olmadan blob üretemeyiz.
Ayrıca Roboflow'un OAK dağıtımı kendi çıkarım sunucusundan/API anahtarından
geçiyor → **yarışmada internet ve WiFi yok (md 4.1)**, o yol bize kapalı.
✅ **Doğru kullanım: Roboflow'u ETİKETLEME + veri seti dışa aktarma için kullan**
(YOLOv11 formatı), eğitimi kendin yap (Colab ücretsiz GPU ya da yerel) →
elinde `best.pt` olur → §3'teki dönüşüme girer.

### 2b. Kendi eğitimimiz
```bash
yolo detect train data=data.yaml model=yolo11n.pt imgsz=416 rect=False
```
- `model=yolo11n.pt` — başlangıç ağırlığı (`~/Desktop/yolo11n.pt`).
- 🔴 **Ön işleme deploy ile aynı olmalı (stretch).** Ultralytics varsayılanı
  **letterbox**'tır. En temiz çözüm: **kareleri eğitimden önce 416×416'ya EZ**
  (Roboflow'daki `Stretch to` ile aynı şey). Sebebi ölçüldü:
  ```
  1352×1014 -> letterbox -> içerik 416×312 + üstte/altta 52 px gri şerit
  416×416   -> letterbox -> dolgu YOK (r = 1.000)   ← kare girdide letterbox etkisiz
  ```
  ⇒ Kareler zaten 416×416 ise Ultralytics'in letterbox'ı **hiçbir şey yapmaz** ve
  eğitim geometrisi deploy ile birebir olur.
- ✅ **Etiketler değişmez:** YOLO etiketleri normalize (0-1) olduğu için saf
  yeniden boyutlandırma değerleri etkilemez — `.txt` dosyaları aynen kopyalanır.
- Alternatif: deploy'a `ImageManip` letterbox eklemek. **İkisi birlikte değişir** —
  biri değişip diğeri kalırsa geometri kayar.

---

## 3. Çevirme — İKİ YOL

### Yol A (06.08'de UÇTAN UCA ÇALIŞTIRILDI, tercih edilen)
Bu yol gerçekten koşuldu ve ürettiği blob **gerçek kamerada** ölçüldü.

```bash
# 1) Kafa ameliyatı + ONNX (yerelde)
git clone --recursive https://github.com/luxonis/tools.git && cd tools
python3 -m venv venv
venv/bin/pip install torch torchvision --index-url https://download.pytorch.org/whl/cpu
PIP_CONSTRAINT=constraints.txt PIP_BUILD_CONSTRAINT=constraints.txt venv/bin/pip install .
venv/bin/tools /yol/best.pt --imgsz "416" --use-rvc2
#    -> shared_with_container/outputs/<ad>_<tarih>/{<ad>.onnx, <ad>.tar.xz}
#       (.tar.xz içinde ONNX + config.json — VPU'da KOŞMAZ, adım 2 şart)
```
```python
# 2) ONNX -> 4 shave .blob (blobconverter, bulut)
import blobconverter
p = blobconverter.from_onnx(
    model="<ad>.onnx",
    data_type="FP16",
    shaves=4,                                  # 🔴 4
    version="2022.1",
    optimizer_params=["--scale_values=[255,255,255]", "--mean_values=[0,0,0]"],
)
```
```bash
# 3) config.json'ı blob'un YANINA koy (sınıf isimleri oradan okunuyor)
tar -xJf <ad>.tar.xz config.json
mv <blob> yolo11n_duba_rvc2.blob
```
⚠️ `tools --help` cyclopts sürüm uyumsuzluğundan çöker — aracın kendisi çalışır.

### Yol B — HubAI SDK
```python
import os
from hubai_sdk import HubAIClient
client = HubAIClient(api_key=os.getenv("HUBAI_API_KEY"))
r = client.convert.RVC2(
    path="/yol/best.onnx",          # SDK örneği ONNX alıyor
    name="yolo11n-duba",
    compress_to_fp16=True,
    number_of_shaves=4,             # 🔴 varsayılan 8 — MUTLAKA 4 yap
    superblob=False,                # 🔴 varsayılan True — MUTLAKA False (v2.30 okuyamaz)
    yolo_input_shape=[416, 416],    # varsayılan [640,640]
    yolo_class_names=["kenar_dubasi", "engel_dubasi"],   # SIRA = data.yaml sırası
)
```
`yolo_version` genelde **omit edilir** (otomatik algılama); yalnız algılama
başarısız olursa yazılır. Kurulum: `pip install --user hubai-sdk` +
hub.luxonis.com'dan API Key.

---

## 4. 🔴 DOĞRULAMA — Jetson'a taşımadan ÖNCE, PC'de
Bu adım pazarlıksız: yanlış blob teknede **düzeltilemez** (superblob'dan shave
seçme imkânı v2.30'da yok, yeniden dönüşüm internet ister).

```python
import depthai as dai
b = dai.OpenVINO.Blob("yolo11n_duba_rvc2.blob")
print(b.numShaves, b.numSlices)          # -> 4 4   (cihaz gerekmez)
print({n: v.dims for n, v in b.networkInputs.items()})   # -> 416x416x3
```
```bash
python3 -c "
import json;c=json.load(open('config.json'))
h=c['model']['heads'][0]['metadata']
print(h['classes'], h['n_classes'])"
```
Beklenen:
- `numShaves == 4` — değilse **DUR**, adım 3'ü tekrarla
- `classes == ['kenar_dubasi','engel_dubasi']`, `n_classes == 2`
  (80 görürsen stok COCO'yu çevirmişsin — yanlış dosya)
- giriş 416×416

### 4b. Sınıf çökmesi testi (atlanmamalı)
Luxonis'in bildirdiği gerçek vaka: YOLO11n + 416×416 + **iki benzer sınıf** →
GPU'da ayırıyor, cihazda ikisini birbirine karıştırıyor. Sebep blob
sıkıştırması, mimariye özgü **değil** — bizde turuncu↔sarı aynı riski taşıyor.
Luxonis'in önerdiği 1:1 kıyas: NN'e giren kareyi `detectionNetwork.passthrough`
çıkışından kaydet → **aynı kareyi** PC'de orijinal `.pt` ile koştur → sınıfları
karşılaştır. Turuncu duba cihazda "engel" çıkıyorsa sahaya çıkma.

---

## 5. Jetson'a taşı + saha teyidi
```bash
mkdir -p /home/girdap/models
cp yolo11n_duba_rvc2.blob config.json /home/girdap/models/    # USB ile (WiFi yok)
```
1. `python3 scripts/duba_kamera_test.py` → kutular görünmeli; terminalde
   **`Model sınıf sırası: [...]` logunu OKU** — turuncu dubaya `kenar` demeli.
2. FPS **11 civarında** mı (ölçülen tavan 12,2). 8'in altına düşerse node uyarır.
3. `ros2 launch girdap_ida_algi algi.launch.py` → `/perception/buoys` akıyor mu.

---

## 6. Sık yapılan üç hata
1. **Varsayılanlarla çevirmek** → 8 shave + superblob → teknede hiç açılmaz.
2. **config.json'ı unutmak** → sınıf isimleri okunamaz → yedek sabitlere düşer →
   turuncu/sarı yer değiştirebilir (sessiz puan kaybı).
3. **Eğitimde letterbox, deploy'da stretch** → geometri kayması, uzak dubada belirgin.

## 7. Bu rehberdeki sayılar nereden geliyor
- shave 4 / 6-shave reddi: 05.08.2026, bu cihazda ölçüldü
- 4/6/8 shave FPS (19,9 / 21,8 / 14,3): 06.08.2026, iki koşu, PC + gerçek kamera
- v8n vs v11n (21,6 / 19,9): 06.08.2026, iki koşu, tek değişken mimari
- mAP 39,5 / 37,3: Ultralytics resmî COCO tablosu
- superblob/NNArchive yokluğu: `depthai-core@v2.30.0`, `depthai-python@v2.30.0.0` kaynağı
- HubAI varsayılanları (8 / True): `luxonis/hubai-sdk` `docs/available_parameters.md`
- YOLO26'nın v3.6+ gereksinimi: Luxonis forum, 2026-04-17 ve 2026-05-16

## Kaynaklar
- [RVC2 donanım (SHAVE/CMX)](https://docs.luxonis.com/hardware/platform/rvc/rvc2)
- [RVC2 performans tablosu](https://docs.luxonis.com/software/ai-inference/performance/)
- [luxonis/tools (v5–v12 + yolo26, `--imgsz` varsayılan 416)](https://github.com/luxonis/tools)
- [HubAI SDK](https://github.com/luxonis/hubai-sdk)
- [Model dönüşümü (shave↔hız doğrusal değil)](https://docs.luxonis.com/en/latest/pages/model_conversion/)

---

## 8. ✅ KURU PROVA — bu zincir 2026-08-06'da uçtan uca KOŞTURULDU
Gerçek veri seti gelmeden zincirde sürpriz kalmasın diye, sentetik 2 sınıflı
küçük bir veri setiyle tüm adımlar denendi (model kalitesi amaç değildi):

| Adım | Sonuç |
|---|---|
| Sentetik veri (80 train / 20 val, `kenar_dubasi`+`engel_dubasi`) | ✅ |
| `yolo detect train … model=yolo11n.pt imgsz=416` (8 epoch, CPU, ~47 sn) | ✅ mAP50 0,913 |
| `tools best.pt --imgsz "416" --use-rvc2` | ✅ ONNX + config.json |
| `blobconverter.from_onnx(..., shaves=4)` | ✅ blob 5,7 MB |
| §4 doğrulama: `numShaves` | ✅ **4** |
| §4 doğrulama: `classes` / `n_classes` | ✅ `['kenar_dubasi','engel_dubasi']` / **2** |
| Algı node'unun sınıf çözümü (gerçek fonksiyonlar koşuldu) | ✅ kenar=0, engel=1, **isimden** |
| Gerçek OAK-D Lite'ta çalıştırma | ✅ **20,64 FPS** (4 shave, 416×416) |

📌 Yan bulgu: 2 sınıflı modelimiz stok 80 sınıflı v11n'den **bir tık hızlı**
(20,64 vs 19,96 FPS) — sınıf sayısı azaldıkça tespit kafası hafifliyor.
⇒ Gerçek modelde de 11 FPS hedefi için pay var.
