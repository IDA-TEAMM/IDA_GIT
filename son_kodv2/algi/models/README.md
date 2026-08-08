# Modeller

Bu klasöre model dosyaları konur. Büyük oldukları için git'e dahil edilmez
(`.gitignore`).

> ⚠️ **A-1 hâlâ AÇIK:** `MODEL_BLOB`'un işaret ettiği
> `/home/girdap/models/yolo11n_duba_rvc2.blob` bu makinede **yok** — gerçek
> duba modeli veri seti → eğitim → export zincirini bekliyor. Ayrıntı:
> [`../docs/bekleyen_girdiler.md`](../docs/bekleyen_girdiler.md) §B1.

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

1. NNArchive `tar.xz`'yi `/home/girdap/models/`e koy, SHA256'sını tabloya işle.
2. `tar -xJf` ile blob + config.json çıkar; blob'u `MODEL_BLOB` adına taşı.
3. `python3 -c "import json;print(json.load(open('config.json'))['model']['heads'][0]['metadata']['classes'])"`
   ile sınıf sırasını doğrula.
4. Masa testi: `scripts/duba_kamera_test.py` (deploy pipeline'ının aynası).
