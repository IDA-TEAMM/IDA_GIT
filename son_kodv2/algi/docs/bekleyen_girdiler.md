# Bekleyen Girdiler — henüz elimizde olmayan her şey

> **Ne bu?** Kod yazılabilir durumda ama **dışarıdan bir bilgi/dosya/ölçüm
> beklediği için** kapatılamayan işlerin tam listesi. Her madde: *ne bekleniyor,
> kimden, neyi bloke ediyor, geldiğinde tam olarak ne yapılacak.*
>
> Amaç: parça geldiği an düşünmeden uygulamak. "Sonra bakarız" demeyelim diye.
>
> Son güncelleme: 2026-07-10 · İlgili: [`girdap_decision_bulgular.md`](girdap_decision_bulgular.md),
> karar deposu denetimi `EyupEker1/girdap-decision → docs/kod_denetimi.md`

**Öncelik:** 🔴 sahaya çıkışı bloke eder · 🟠 yarışma çıktısını/puanı etkiler ·
🟡 kalite/risk · ⚪ iyileştirme

---

## A. Mekanik ekipten — montaj ölçüleri

### 🔴 A1. Livox Mid-360 montaj yüksekliği `h` (su hattından, metre)

**Bu, şu an sahaya çıkışı bloke eden tek numaralı eksik.**

**Neden:** Karar deposu denetiminde bulundu (F5.1). `perception_lidar_node`
LiDAR noktalarını **hiç dönüştürmeden** işliyor ve `z ∈ [0.1, 3.0]` filtresini
uyguluyor. Filtre `z`'yi *su hattından* sanıyor, ama veri *sensör
çerçevesinde* geliyor. LiDAR su hattından `h` metre yukarıdaysa 50 cm'lik bir
duba `z ∈ [−h, −h+0.5]` aralığında görünür:

| `h` | Filtreden geçen duba noktası |
|---|---|
| 0.4 m | %20 |
| **> 0.4 m** | **%0 — duba tamamen silinir** |

Katamaran güverte/direk montajında `h` neredeyse kesin olarak > 0.4 m.
Sonuç: `/perception/obstacle_map` **boş**, MPPI dubaların içinden geçer,
**Parkur-2 biter.**

**Ne lazım:** `h` (m, su hattından LiDAR optik merkezine) + `x`, `y` ofseti.

**Geldiğinde ne yapılacak:**
1. `base_link`'i **su hattında** tanımla, `hardware.launch.py:237`'deki
   `_static_tf("base_link", "livox_frame")` çağrısına gerçek `--z h` ver.
2. `perception_lidar_node`'a TF dönüşümü ekle (`tf2_ros.Buffer` +
   `do_transform_cloud`), **veya** hızlı yama olarak `lidar_height_m`
   parametresi ekleyip filtreyi `z ∈ [z_min − h, z_max − h]` yap.
3. `synthetic_lidar.py`'ı gerçek çerçeveye taşı (dubaları `z ∈ [−h, −h+0.5]`
   üret) — yoksa testler hatayı maskelemeye devam eder.

**Ölçüm bitmeden Parkur-2 suda denenmemeli.**

### 🔴 A2. `base_link` orijini nerede?

**Neden:** Her şey buna bağlı — LiDAR z-filtresi, kamera bearing'i, MPPI engel
mesafeleri. Şu an `hardware.launch.py`'deki üç static TF de identity (0,0,0),
yani "tüm sensörler base_link'te" deniyor; bu fiziksel olarak yanlış.

**Ne lazım:** Tek cümlelik karar — `base_link` = su hattı + tekne merkezi mi,
güverte mi, IMU mu? (Arkadaşa sorulan 5 sorudan biri.)

**Geldiğinde:** üç static TF'i gerçek ölçülerle doldur; `perception_lidar_node`
ve füzyon TF okumaya başlasın (şu an **hiçbir node `tf2` okumuyor** — denetim
F3.4/F5.2).

### 🟠 A3. OAK-D Lite montaj konumu ve açısı

**Neden:** Kamera-LiDAR bearing füzyonu iki sensörün de `base_link` orijininde
olduğunu varsayıyor. Aralarında ~0.5 m ofset varsa 5 m'deki bir dubada ~6°
bearing hatası doğar — `bearing_tolerance_rad = 0.15` (≈8.6°) eşiğinin yarısı
tek başına yenir (denetim F3.4).

**Ne lazım:** yükseklik, ileri ofset, **pitch açısı** (aşağı eğik mi?), yaw.

**Geldiğinde:** `_static_tf("base_link", "oak_frame")` doldur; füzyona gerçek
projeksiyon ekle (`bearing_from_camera` yerine intrinsic+extrinsic ışın izdüşümü).

### 🟠 A4. Kamera yağmur muhafazası + muhafaza arkası odak

**Neden:** Muhafaza camı odak düzlemini kaydırır; OAK-D Lite'ın AF varyantında
otomatik odak cama kilitlenebilir. Kodda `RGB_SABIT_FOKUS` opsiyonu **hazır**
ama değeri ölçülmedi.

**Geldiğinde:** muhafaza arkasından masa testi → net odak değerini
`RGB_SABIT_FOKUS`'a yaz, sabitle.

### 🟡 A5. Tekne gerçek genişliği + thruster geometrisi

**Neden:**
- Genişlik → MPPI `obstacle_margin` (şu an `0.5 m`, tekne 0.75 m en varsayımıyla
  dar olabilir — denetim, `mppi.py:88`).
- Thruster yerleşimi → `prototype/dynamics/catamaran.py` diferansiyel tahrik modeli.

**Geldiğinde:** `obstacle_margin`'i sahada ölç; dinamik modelin tork kolunu güncelle.

### 🟡 A6. IMU (Pixhawk) konumu → `imu_link`

Kamikaze çarpma algılama (`shock_threshold_g = 5.0`) IMU'nun tekne merkezinden
uzaklığına duyarlı (dönme ivmesi kolu). Parkur-3 işi, ertelendi.

---

## B. Model ve eğitim verisi

### 🔴 B1. Dağıtılabilir duba NN Archive **bu makinede yok** — ve Desktop'taki arşiv stok COCO

**2026-07-10'da dosyaların içine bakılarak doğrulandı. Önceki varsayımlar yanlıştı.**

| Beklenen | Gerçek |
|---|---|
| `models/yolo11n_duba_rvc2.tar.xz` | `models/` **boş** (yalnız `.gitkeep` + README) |
| Kod: `MODEL_NNARCHIVE = "/home/girdap/models/yolo11n_duba_rvc2.tar.xz"` | Bu bir **Jetson** yolu (`/home/girdap`) — bu makinede yok. Jetson'da var mı: **DOĞRULANMADI** |
| `models/README.md`: HubAI `ida-buoy-yolo11n` | Bu isimde yerel dosya **yok** |

**Desktop'ta bulunan tek NN Archive duba modeli DEĞİL:**
`/home/eyup/Desktop/oakdlite/416x416yolov11n.tar.xz` (4.800.688 B,
SHA256 `a87b573b764e76fb3168114a057f5c134a3139b99dd9c287aa6f351da4377318`)
→ `config.json`: **`n_classes: 80`**, `classes: [person, bicycle, car, … toothbrush]`
→ **stok COCO YOLO11n.** Yanındaki `yolo11n.pt` stok Ultralytics ağırlığıyla
byte-byte aynı. Çöp kutusundaki 3 arşiv de 80 sınıf COCO.

⚠️ Bu arşiv yanlışlıkla Jetson'a atılırsa tekne `person`/`boat`/`orange`
tespit eder; `class 0 = person`, `class 1 = bicycle`.

**Gerçek eğitilmiş model:** `/home/eyup/girdap_yolo/Gazebonew.pt`
(5.478.490 B, 6 Nisan 2026) — sınıflar `Engel Dubasi`, `Kenar Dubasi`.
Tek kopya, alakasız klasörde, git dışı, yedeksiz. Adındaki "Gazebo" simülasyon
verisiyle mi eğitildiğini düşündürüyor — **gerçek saha görüntüsüyle eğitilip
eğitilmediği belirsiz.**

**Yapılacak:**
1. **ŞİMDİ:** `Gazebonew.pt`'yi iki ayrı yere yedekle (harici disk + bulut).
2. **Jetson'a bağlanınca:** `/home/girdap/models/yolo11n_duba_rvc2.tar.xz` var mı?
   Varsa `tar -xJf … -O config.json` ile `classes` sırasını oku ve buraya yaz.
3. Yoksa: `Gazebonew.pt` → HubAI ile RVC2 NN Archive üret (416×416, 6 shave) —
   **adım adım rehber: [`hubai_model_rehberi.md`](hubai_model_rehberi.md)**;
   SHA256'yı `models/README.md`'ye işle. (Üretim VİDEO SONRASINA planlı.)
4. `Gazebonew.pt`'nin eğitim verisini tespit et (Gazebo mu, saha mı?).
5. 🗑️ **Model Jetson'a konduğu gün `scripts/kamera_goruntu_test.py`'yi SİL** —
   modelsiz dönem için GEÇİCİ kamera testi (başlığında da yazıyor); asıl test
   `duba_kamera_test.py`. Karışıklık olmasın diye buraya işlendi (Eyüp istedi).

### 🔴 B1b. Model sınıf sırası sabitlerin **TERSİ** — kodda savunma eklendi

`Gazebonew.pt` içindeki ham `names` sözlüğü:

```python
{0: 'Engel Dubasi', 1: 'Kenar Dubasi'}
```

Kod ise `KENAR_CLASS = 0`, `ENGEL_CLASS = 1` diyordu — **ters**. Bu sırayla
sahaya çıkılsaydı geçit tespiti (karşılıklı iki **turuncu kenar** çifti) aslında
iki **sarı engeli** çift sanardı ve `/perception/buoys`'a ters `class_id`
yayınlanırdı. Hiçbir istisna atılmaz; yalnızca Parkur-2 kaybedilirdi.

`sdn.getClasses()` zaten çağrılıyordu ama **sadece loglanıyordu**, remap için
kullanılmıyordu.

**Yapıldı (bu oturumda):** `_sinif_indeksleri_coz()` eklendi — indeksler artık
NN Archive'ın sınıf **isimlerinden** çözülüyor (`"kenar"` / `"engel"` alt dizgi
eşleşmesi). Sabitler yalnız isimler okunamazsa yedek olarak devreye giriyor ve
o durumda gürültülü `ERROR` basılıyor. Çözülen sıra sabitlerden farklıysa `WARN`.
Beş senaryo ile doğrulandı (gerçek sıra, ters sıra, stok COCO, boş, tek sınıf).

**Yine de:** dağıtılan arşivin sınıf sırası Jetson'da `getClasses` logundan
**okunup teyit edilmeli** (B1 madde 2).

### 🟠 B2. Eğitim verisi eksikleri (Parkur-2 yanlış-pozitif azaltma)

- **Beyaz sosis duba negatifi** — şartname: parkur dışını çevreleyen beyaz sosis
  dubalar var, model bunları görmedi.
- **Bayraklı armut duba örnekleri** — kenar/engel dubalarında bayrak var
  (şartname Şekil, "yükseklik bayrak dahil değildir"), eğitim setinde bayraklı
  örnek yeterli mi?

**Geldiğinde:** veriyi ekle, yeniden eğit, yeni `.tar.xz` üret, SHA256 güncelle.

### 🟡 B3. Hedef duba sınıfı (Parkur-3) — **ERTELENDİ**

Şartname md 5.5.2.1: **3 ayrı hedef dubası**, 640 mm × 950 mm, renkler
**RAL 9005 (siyah), RAL 3026 (kırmızı), RAL 6037 (yeşil)**. Angaje edilecek renk
görev başlamadan yüklenir; başladıktan sonra aktarım yasak.

Mevcut modelde hedef sınıfı **yok**. İki yol: (a) 3 rengi ayrı sınıf olarak
yeniden eğit, (b) büyük-duba tespiti + bbox içi HSV renk ayrımı.

**Not:** İHA bu işi devralamaz — md 5.5.3.1 İHA yalnız **kıyı** tarafında
uçabilir, deniz üstü uçuş Parkur-3'ü başarısız kılar. İHA yalnız kıyıdaki renk
**plakasını** okur; denizdeki doğru renkli dubayı bulmak **İDA kamerasının işi.**

Odak şu an P1+P2 olduğu için bekliyor. Açılınca karar verilecek.

### 🟡 B4. Gerçek `.pt` karar deposuna verilecekse — sınıf eşleme tablosu şart

Denetim F5.7: karar deposundaki `camera_buoys.YoloInference._infer_real`,
`int(box.cls)` değerini **doğrudan** sözleşme `class_id`'si yapıyor. Bizim
modelde sınıflar `{0: kenar, 1: engel}`; sözleşmede `2 = hedef` bekleniyor.
`use_yolo=true` açılırsa HSV'nin ürettiği 0/1 tespitlerinin üstüne **çift
tespit** basar.

**Geldiğinde:** `_infer_real`'e açık `model_class → class_id` tablosu ekle.

---

## C. Takım arkadaşından (karar yazılımı)

### 🟠 C1. Cevap bekleyen 5 soru

`docs/mppi_entegrasyon_notu.md` §6:
1. `/girdap/fusion/odom` frekansı ve çerçevesi (odom mu, map mi?)
2. `camera_hfov_rad = 1.2` teyidi (OAK-D Lite RGB gerçek HFOV)
3. `/perception/buoys_3d` isteniyor mu? (stereo 3D bizde hazır — bearing
   füzyonu yerine doğrudan kullanılabilir, F5.9'daki işaret hatasını da
   tamamen bypass eder)
4. Mock kamera node'u gerçek donanımda kapatılacak mı? (denetim F3.1)
5. `numpy>=1.26,<2` pini (denetim F2.2)

### 🟠 C2. Denetim bulgularının uygulanması

`EyupEker1/girdap-decision → docs/kod_denetimi.md` — Faz 1-5 tamam, 18 faza
kadar sürüyor. Kritikler: F5.1 (LiDAR z-filtresi), F2.1 (`package.xml` eksik
bağımlılıklar), F4.1 (Dosya-2 göreli yol).

---

## D. Ölçüm / kalibrasyon (bizde, donanım gerektirir)

### 🟠 D1. Letterbox `_LB_PAY = 0.125` masa testi

**Neden:** 640×480 kare → 416×416 NN girişine LETTERBOX ile oturuyor; dikey
siyah şeritlerin payı varsayım. Yanlışsa tüm bbox'lar dikey kayar → Dosya-1
overlay videosunda kutular dubaların üstüne oturmaz.

**Nasıl:** `scripts/duba_kamera_test.py` — bbox'ları `sdn.passthrough` karesine
ham normalize koordinatla çizer. Kutular şeritlerle tam oturuyorsa `0.125`
doğru; dikey kaymışsa `_LB_PAY = 0.0`.

**Daha iyisi (varsayımı tamamen kaldırır):** DepthAI 3.6.1'de
`ImgDetections.getTransformation().remapPointTo()` var — elle formül yerine
kesin remap. Donanımsız yazılabilir.

### 🟠 D2. Dosya-1 kaydedicinin FPS etkisi

Passthrough frame + `cv2.VideoWriter` ekstra USB bandı yer. Şu an 10–14 FPS
(tipik ~11.6) bandındayız, `FPS_UYARI_ESIK = 8`. Kaydedici açıkken ölç.

### 🟡 D3. MPPI gerçek süresi (Jetson, CPU)

Karar deposu `control_rate_hz = 20.0` (50 ms) diyor ama kendi yorumu CPU'da
K=1000 rollout için ~100 ms ölçmüş (denetim F4.2). Jetson'da gerçek süreyi ölç;
CUDA portu yapılana kadar `control_rate_hz`'i gerçekçi değere çek.

### 🟡 D4. GPS RTK gürültüsü

`params.yaml: gps_sigma_xy = 0.30` (mock değeri). RTK fix'te ~0.05 olmalı.
Saha ölçümüyle güncelle.

---

## E. Yarışma günü gelecek veriler

| Ne | Ne zaman | Nereye |
|---|---|---|
| Görev noktaları (dd.ddddddd, USB, hakem çadırı) | Teknik kontrol öncesi/sonrası | `config/competition_mission.yaml`, parkur etiketleriyle |
| Parkur-3 angajman **rengi** | Görev başlamadan (başladıktan sonra **yasak**) | Model/HSV eşiği — B3'e bağlı |
| Video günü GPS köşe koordinatları | Video çekimi öncesi | `config/video_mission.yaml` (prosedür karar deposu `CLAUDE.md`'de) |

---

## Şimdi yapılabilecekler (hiçbir şey beklemiyor)

1. **B1 — `Gazebonew.pt` yedeği. Bugün.** Tek kopya; kaybolursa yeniden eğitim
   günler alır ve elimizde başka duba modeli yok.
2. **D1'in yazılım kısmı** — `getTransformation().remapPointTo()` ile kesin
   letterbox remap; masa testi belirsizliğini koddan siler.
3. **C2** — denetimin kalan fazları (6→18).
4. Karar deposunda **A1'den bağımsız** düzeltmeler: F2.1 `package.xml`,
   F4.1 Dosya-2 yolu, F2.2 numpy pini.

## Sahaya çıkmadan cevaplanması ZORUNLU üç soru

Bunlar bilinmeden Parkur-2 suda denenmemeli:

1. **Livox montaj yüksekliği `h`?** (A1) — bilinmezse `obstacle_map` boş kalabilir.
2. **Jetson'daki NN Archive'ın sınıf sırası ne?** (B1) — `getClasses` logunu oku.
3. **`base_link` nerede?** (A2) — her geometrik hesap buna dayanıyor.
