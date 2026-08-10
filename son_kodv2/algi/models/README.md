# Modeller

> ## ✅ 2026-08-10 — YARIŞMA MODELİ BURADA, REPODA
> `yolo11n_duba_rvc2.blob` + `config.json` **git'e dâhil edildi** (`.gitignore`'da
> istisna). Gerekçe: yarışma alanında **internet YOK** (md 4.1) ve `blobconverter`
> **bulut** ⇒ blob sahada üretilemez; ayrıca *"teknede hangi blob koşuyor?"*
> sorusunun izlenebilir cevabı olmalı.
>
> **Jetson'a kurulum:** iki dosyayı `/home/girdap/models/` altına kopyala
> (`MODEL_BLOB` oraya bakıyor, `config.json` blob'un **yanında** olmalı).

## Bu blob nereden geldi (provenance)

```
kaynak .pt : girdap_EGITIM_HATTI/runs/detect/girdap_ince3_1053/weights/last.pt
             (best.pt DEGIL — olculdu: Ultralytics mAP'e bakiyor, bizim metrigimiz
              sinif hatasi; son epoch'lar 52 -> 47 iyilestirdi)
zincir     : ana egitim 208 ep (en iyi 158)  ->  dikissiz ton kopyalariyla
             ince ayar 162 ep (en iyi 132)
egitim     : YOLO11n · imgsz 416 · 300 ep tavan/patience · hsv_h 0.015 · hsv_s 0.35
             scale 0.5 · perspective 0 · flipud 0 · degrees 0 · negatif kare %8
veri       : 7.546 train (416x416 EZILMIS) + 2.087 valid · sizintisiz blok bolme
export     : tools --imgsz 416 --use-rvc2  ->  ONNX
             blobconverter FP16 · shaves=4 · OpenVINO 2022.1
             optimizer_params: --scale_values=[255,255,255] --mean_values=[0,0,0]
                               --reverse_input_channels        <-- 🔴 ZORUNLU
sha256 blob  : 5726819a101eb4f62dd8ad65cdd302f980b349c0fa448190264d412031871b9c
sha256 config: 79f6e174bc9f175a580b03068d74202d78a50c5276ac1bec017a3d5ae0660d0c
```

### 🔴 `--reverse_input_channels` NEDEN ZORUNLU
Ultralytics modeli **RGB** bekler (`augment.py Format._format_img -> img[::-1]`),
`ColorCamera` ise **BGR** gönderir (`duba_gecis_navigator.py:406`). Arada çeviren
olmazsa kanallar ters gider. **Ölçüldü (PC, .pt ile, 600 kare):**
doğru sıra recall **%96,8** ↔ ters sıra **%43,0**. Luxonis dokümanı da bu iki
çözümü veriyor; biz (A)'yı seçtik çünkü kamera BGR kalınca **şartname Dosya-1
mp4'ü** (kutu çizimli kayıt) doğru renkte olur.
⚠️ **(A) ve (B) birlikte uygulanmaz** — çift çevirme geri alır.
🚨 Saha kurtarma: blob yeniden üretilemezse `setColorOrder(RGB)` tek satırlık,
internetsiz alternatiftir.

## Ölçülen başarım (2.087 karelik sızıntısız valid)
| val | recall | küçük <12px | sınıf hatası | yanlış kutu |
|---|---|---|---|---|
| temiz | **%97,7** | %95,9 | 8 | 155 |
| soluk (UV/pus) | %96,9 | %94,4 | — | — |
| yarışma tonu (dikişsiz benzetim) | **%96,8** | %95,2 | **47** | 231 |

Mesafeye göre (yarışma tonu): 4,5-7,5 m **%98,6** · 7,5-10 m **%99,1** ·
10-13 m %98,2 · 13 m+ %89,7.
⚠️ **Val aynı oturumdan** (tek göl, tek gün) ⇒ bu sayılar **ÜST SINIR**.

## Cihazda yapılacak kabul testi
```bash
python3 ~/girdap_EGITIM_HATTI/3_kabul_testi.py \
    --blob models/yolo11n_duba_rvc2.blob --config models/config.json
```
1. `numShaves == 4` ✅ (PC'de geçti) — fazlası model **hiç yüklenmez**
2. `config.json` sınıf isimleri kanonik ✅ (PC'de geçti)
3. 🔬 **Cihazda:** passthrough'dan kare al → aynı kareyi PC'de `.pt` ile koştur.
   Tespit sayısı yarı yarıya düşüyorsa **kanal sırası ters**.

## Kodun beklediği (🔴 2026-08-05'te DEĞİŞTİ — depthai v2'ye geçiş)

`duba_gecis_navigator.py`:
```python
MODEL_BLOB = "/home/girdap/models/yolo11n_duba_rvc2.blob"   # Jetson yolu
```

depthai **v2'de NNArchive yüklenemez** (2.30.0.0'dan doğrulandı) → model
**`.blob`** olarak verilir ve sınıf isimleri blob'un **yanındaki
`config.json`**'dan okunur (NNArchive config formatı:
`model.heads[0].metadata.classes`). HubAI/modelconverter çıktısı NNArchive
`tar.xz` ise ikisi de içinden çıkar:

```bash
cd /home/girdap/models
tar -xJf yolo11n_duba_rvc2.tar.xz     # → *.blob + config.json + buildinfo.json
mv *.blob yolo11n_duba_rvc2.blob      # MODEL_BLOB adıyla eşleştir
```

Saha benchmark'ı: deploy **11 FPS**, ölçülen tavan **12,2** (416×416 + stereo
birlikte, VPU sınırı — 05.08.2026 ölçümü).

## 🔴 Export/eğitim kısıtları (ölçüldü, pazarlıksız)

1. **≤4 SHAVE** derle (🔴 05.08 kamerada ölçüldü): 12MP RGB modu (tam 4:3 FOV)
   6 CMX dilimi yiyor → NN'e **4 shave** kalıyor; 6-shave blob yüklenmedi.
   4-shave ile 10,9 FPS @ 11, tavan 19,8 — hedef rahat. (Eski "≤7" bütçesi
   v3 + küçük RGB çıkışı dönemindendi.)
   🔴 **Blob dönüşüm anında 4 shave'e derlenmeli — sonradan değiştirilemez.**
   `getBlobWithNumShaves(4)` bir **v3 API'sidir; kurulu depthai 2.30.0.0'da
   SuperBlob ve NNArchive YOK** (`depthai-core@v2.30.0` +
   `depthai-python@v2.30.0.0` kaynağından doğrulandı 06.08.2026) ⇒ superblob
   gelirse teknede açılamaz ve sahada telafi edilemez. HubAI kullanılıyorsa
   `superblob=False` (varsayılan True!) + `number_of_shaves=4` (varsayılan 8!).
   ℹ️ Ölçüm 06.08: 4→19,9 · 6→21,8 · **8→14,3 FPS** — çok shave yavaşlatıyor
   (Luxonis: optimal ≈ mevcudun yarısı). 4 ile 6 arası fark yalnız %9; 12MP
   tam-FOV tercihinin bedeli bu, takas lehimize.
2. **Giriş 416×416** ve `NN_GIRIS` sabitiyle birlikte değişir.
3. **Ön işleme = SIKIŞTIRMA (stretch), letterbox DEĞİL:** deploy'da preview
   `keepAspectRatio(False)` ile tam kare 416×416'ya sıkıştırılıyor (Luxonis
   resmî YOLO deseni; bu cihazda ölçülen desen de bu). Eğitimde de aynı ön
   işleme kullanılmalı — Ultralytics varsayılanı letterbox'tır, fark modelin
   sahada gördüğü geometriyi kaydırır. Eğitim günü karar: ya eğitimde stretch,
   ya deploy'a ImageManip letterbox eklenir (İKİSİ BİRLİKTE değişir).
4. YOLOv11 **anchor-free** → `setAnchors` çağrılmaz (kodda da yok).

## Eldeki artefaktlar

| Dosya | Ne | Sınıflar | Not |
|---|---|---|---|
| `~/girdap_yolo/Gazebonew.pt` | **Gerçek eğitilmiş duba modeli** | `{0: Engel Dubasi, 1: Kenar Dubasi}` | 5.478.490 B, 06.04.2026. **Tek kopya, yedeksiz.** Adı simülasyon verisi şüphesi doğuruyor. |
| `~/Desktop/416x416yolov11n.tar.xz` | ❌ **stok COCO YOLO11n** — duba modeli DEĞİL | 80 COCO sınıfı | 6-shave blob + config.json içeriyor; **yalnız pipeline smoke testi için**. `/home/girdap/models/`e KOYMA — A-1 "çözüldü" sanılır, sahada COCO ile çıkılır. |

## Sınıf sırası — dikkat

Eğitilmiş modelin sırası (`0=Engel, 1=Kenar`) koddaki yedek sabitlerin
(`KENAR_CLASS=0, ENGEL_CLASS=1`) **tersidir**. Bu yüzden kod indeksleri
sabitten değil, config.json'daki **isimlerden** çözüyor
(`_sinif_indeksleri_coz`). Yeni bir model konurken:

1. Node'u başlat, `Model sınıf sırası: [...] → kenar=..., engel=...` logunu **oku**.
2. İsimlerde `kenar`/`engel` geçmiyorsa çözüm yedeğe düşer — model yanlış ya
   da isimler değişmiş demektir; sarı/turuncu YER DEĞİŞTİREBİLİR (P2 gider).

## Yeni model koyarken

### ⭐ Tek komut (2026-08-08 — elle yapılan üç adımın yerine)

```bash
cd son_kodv2/algi
./scripts/model_uret.sh /yol/best.pt          # .pt → 4 shave blob + config.json
```
Betik sırayla: luxonis/tools ile kafa ameliyatı + ONNX → `blobconverter`
(**shaves=4**, FP16, 2022.1) → `config.json`'ı blob'un yanına çıkarır →
`scripts/model_dogrula.py` ile **shave · giriş · sınıf isimleri**ni denetler.
Denetim düşerse betik "tekneye TAŞIMA" deyip durur (yanlış blob sahada
düzeltilemez: v2.30'da superblob yok, yeniden dönüşüm internet ister).

Doğrulamayı tek başına da koşabilirsin:
```bash
python3 scripts/model_dogrula.py /home/girdap/models/yolo11n_duba_rvc2.blob
```
⚠️ Betiğin ilk çalıştırması araç zincirini kurar (torch, ~1 GB, internet).
Kurulum yeri `~/girdap_model_araclari` (`GIRDAP_ARACLAR` ile değiştirilebilir).
**2026-08-08'de bu makinede kurulu ve stok `yolo11n.pt` ile uçtan uca
koşturuldu** — üretilen blob `numShaves=4`, giriş `416×416×3`; doğrulama stok
COCO modelini (80 sınıf, isimlerde `kenar`/`engel` yok) beklendiği gibi
REDDETTİ.

### Elle (betik kullanılmıyorsa)

1. NNArchive `tar.xz`'yi `/home/girdap/models/`e koy, SHA256'sını tabloya işle.
2. `tar -xJf` ile blob + config.json çıkar; blob'u `MODEL_BLOB` adına taşı.
3. `python3 -c "import json;print(json.load(open('config.json'))['model']['heads'][0]['metadata']['classes'])"`
   ile sınıf sırasını doğrula.
4. Masa testi: `scripts/duba_kamera_test.py` (deploy pipeline'ının aynası).
