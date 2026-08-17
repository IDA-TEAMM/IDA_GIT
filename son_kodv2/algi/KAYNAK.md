# algi — GİRDAP görüntü işleme katmanı (son_kodv2'nin algı ayağı)

**Kaynak repo:** github.com/EyupEker1/girdap-ida-algi — commit `8771fa9`
(17.08.2026 senkronu. ⚠️ 16-17.08 gecesi iş **bu aynaya** yapılmıştı ve kaynak
`b3b3fea`'da donmuştu — 23 dosya ayrışmıştı; kaynağa geri alındı
[`girdap-ida-algi@61f507f`], sonra normal yön yeniden kuruldu. Kural 7 iki
yönlüdür: ayna güncel varsayılmaz, **kaynak da** güncel varsayılmaz.)
**Kopya tarihi:** 2026-08-07 (bu klasör kaynağın **birebir aynası**; burada
düzenleme yapılmaz, kaynak repoda yapılıp buraya yeniden kopyalanır)
**Sorumlu:** Eyüp (görüntü işleme)

Bu klasör son_kodv2'nin **kamera algısı**dır. `karar/` (Sude/Yahya) karar/görev
katmanı, burası tespit katmanı. Kimse kimsenin dosyasına dokunmuyor; bağlantı
aşağıdaki topic sözleşmesi üzerinden.

> ℹ️ **06.08 (akşam) güncellemesi — saat güvenilirliği:** zaman damgaları artık
> **çekirdeğin senkron bayrağından** (adjtimex/STA_UNSYNC) doğrulanıyor, sadece
> "tarih makul mü" diye bakılmıyor. Sebebi ölçülü bir arıza: Jetson 06.08'de
> ~15 saat bayat saatle açıldı ve 18 kare **dünün tarihiyle "güvenilir"**
> damgalandı. Karar tarafını ilgilendiren yanı: **Dosya-1 (md 4.2) zaman
> etiketleri** aynı kör noktayı taşıyordu (geçersiz dosya = 5 ceza puanı).
> Yöntem **ağ gerektirmez**; ağsız ~8,9 saat sonra bayrak düşer (çekirdek
> `NTP_PHASE_LIMIT`), bu bilinçli bir yanlış-negatiftir. `manifest.csv`
> **şeması değişmedi** (7 kolon) — `saat_guvenilir` alanının anlamı güçlendi.
> Ayrıntı: `girdap_ida_algi/saat.py`.

> ℹ️ **06.08 güncellemesi:** bu klasör 05.08'deki hâlinde donmuştu (depthai v3
> API'si, FPS 12). Bu sürümle **komple değiştirildi**. Aradaki en kritik fark:
> kod artık **depthai v2 (2.30.0.0)** kullanıyor — eski kopya bu Jetson'da
> stereo üretemiyordu.

## Bu katman ne yapar
OAK-D Lite'ın **içindeki** Myriad X VPU'da YOLO (416×416) koşturur; Jetson
CPU'suna tespit maliyeti ~sıfırdır (ölçüldü: %4,9). Aynı pipeline'da stereo
derinlik gelir.

| Topic | Tip | Anlamı |
|---|---|---|
| `/perception/buoys` | `vision_msgs/Detection2DArray` | bbox, class_id "0"=kenar "1"=engel — **piksel uzayı 1280×720** (aşağıya bak) |
| `/perception/buoys_3d` | `geometry_msgs/PoseArray` | stereo 3D duba konumu (bonus; `/perception/obstacle_map` sözleşmesiyle aynı kodlama) |
| `/perception/gate_count` | `std_msgs/Int32` | geçilen **farklı** geçit sayısı |
| `/perception/gate_target` | `geometry_msgs/PoseStamped` | geçidin ötesindeki hedef (karar tarafı isterse kullanır) |

Ayrıca **Dosya-1** (md 4.2) mp4 kaydını üretir: bbox + sınıf overlay'li,
her karesi zaman etiketli, ≥1 Hz (kodda 2 Hz, 120 sn'lik segmentler).

## 🔴 Bunu çalıştırırken karar launch'ında ŞUNLAR gerekiyor

1. **`use_onboard_camera:=false`** — ✅ 04.08'de varsayılanı `false` yapıldı
   (`IDA_GIT@5955d30`). Açık kalırsa `perception_camera_node` de
   `/perception/buoys`'a basar → iki publisher, karışık bbox uzayı.
2. **`oakd_driver_node` açılmamalı** — ✅ 04.08'de kendi `with_oak_driver`
   bayrağına ayrıldı (varsayılan `false`). Açılırsa USB cihazını o alır ve
   **bu node kamerayı hiç açamaz** (tek OAK, tek süreç).
3. **`camera_image_width_px`/`height_px` = 1280×720 kalmalı** — bu node bbox'ı
   bilerek o uzayda yayınlıyor. Değiştirilecekse **ikisi birlikte** değişmeli.

## 🔴 Neden bbox 1280×720 uzayında (gerçek kare 1352×1014 olduğu hâlde)
`perception_fusion_node` bbox merkezini kendi `camera_image_width_px`
parametresine **bölerek** normalize ediyor; mesaj görüntü boyutunu taşımıyor.
O parametre üç yerde birden 1280×720 (`hardware.launch.py:211` ·
`hardware.yaml:207` · `params.yaml:228`). Biz 640 uzayında yayınlarsak bearing
sessizce kayar: kare **ortasındaki** duba +17°'de görünür (tolerans 8,6°),
sağ kenardaki duba merkezde görünür → karenin sağ %75'indeki hiçbir tespit
LiDAR kümesine eşleşmez → sınıf bilgisi düşer → `gate_follower` kenar dubası
göremez → geçitten geçilmez (P1: G1/KD1≥0,5 · P2: ≥2 ikili).
Hiçbir hata basılmaz. Bu yüzden yayın uzayı gerçek kare boyutundan ayrıldı
(`gecit_mantik.bbox_piksel`). Yatay eksen ön işlemeden etkilenmediği için
ölçek yeterli, HFOV (~69°) değişmiyor.

## ⚠️ `/perception/gate_passed` bilerek KAPALI
`GATE_PASSED_YAYINLA = False`. Sebep: `fsm_node._on_gate_passed`
(`fsm_node.py:383-386`) gelen **herhangi** bir True'yu `last_gate_passed_p2`
yapıyor, `mission_fsm` bunu görünce PARKUR2 → PARKUR3 (kamikaze) geçiyor. Yani
her geçitte basılan sinyal Parkur-2'yi **ilk geçitte** bitirir: P2 tamamlanmaz
(md 5.5.2.4 "en az 2 duba ikilisi + son görev noktası"), (G2/KD2)×40 gider,
ödül sıralaması (en az P1+P2) kaybedilir. Algı hangi geçidin **sonuncu**
olduğunu bilemez (KD çalışma anında bilinmiyor; şartname "duba sayılarına göre
akış tasarlanmaması" diyor).
**Karar tarafında P2→P3 geçişi waypoint ilerlemesinden sürülürse** bu bayrak
True yapılabilir. Yerine dürüst sinyal: `/perception/gate_count`.

## 🔴 MODEL: düz `.blob` + `config.json` — NN Archive `.tar.xz` DEĞİL
```
/home/girdap/models/yolo11n_duba_rvc2.blob      ← düz blob, 4 SHAVE
/home/girdap/models/yolo11n_duba_rvc2.json      ← sınıf isimleri (config.json)
```
Kurulu **depthai 2.30.0.0'da `NNArchive` ve `SuperBlob` API'si YOK** (sürüm
etiketlerinden doğrulandı). HubAI/web dönüştürücü `.tar.xz` superblob üretir →
**açılıp içinden düz blob çıkarılmalı**, yoksa node açılmaz.

| Kısıt | Değer | Neden |
|---|---|---|
| SHAVE | **4** (pazarlıksız) | 12MP tam-FOV RGB + stereo CMX yiyor; 6-shave blob *"only 4 available"* ile **yüklenmedi** (05.08 kamerada). Fazla shave = model HİÇ açılmaz, yavaşlama değil |
| Giriş | 416×416 | blob bu boyutla derlenir, kod ile birlikte değişir |
| Ön işleme | **SIKIŞTIRMA (stretch)** | deploy `setPreviewKeepAspectRatio(False)`; **eğitim de aynı olmalı** (Ultralytics varsayılanı letterbox) |
| Mimari | **YOLO11n** | 06.08'de gerçek cihazda ölçüldü: v8n 21,6 / v11n 19,9 FPS = %8 fark; v11n'in +2,2 mAP'i tercih edildi (darboğaz hız değil menzil) |

⚠️ **YOLO26 kullanılamaz** — depthai v3.6+ parsing istiyor, biz 2.30'dayız.
⚠️ Export günü pazarlıksız 2 kontrol: `dai.OpenVINO.Blob(yol).numShaves == 4`
ve passthrough 1:1 sınıf ayrımı testi (**turuncu↔sarı çökmesi** riski — blob
sıkıştırmasından kaynaklanır, sessizdir).
Tarif: `docs/hubai_model_rehberi.md` (uçtan uca kuru provadan geçti, 06.08).

## 🗑️ Veri seti toplama — 2026-08-16'da REPODAN KALDIRILDI
**NE:** Toplayıcı (`scripts/oak_veriseti_topla.py`), systemd unit'i
(`scripts/girdap-veriseti.service`), kıyı kontrol listesi
(`docs/veriseti_deniz_oturumu.md`), Jetson kartı, açılış modu belgesi, planı ve
manifest testleri silindi (Eyüp kararı).

**NEDEN:** Veri seti toplandı ve model eğitildi; iş bitti. Kalan tek işlevi
riskti: **tek OAK var**, toplayıcı boot'ta kamerayı önce kaparsa algı node'u
**hiç açılamaz** ⇒ P1+P2 = 0, belirti vermeden (md 4.1). Yarışmaya günler kala
repoda duran `--veriseti-servis` kolu, yanlışlıkla koşulduğunda algıyı
**devre dışı bırakıyordu**.

**GERİ ALINIRSA NE KIRILIR:** Yeni bir göl/deniz oturumu gerekirse toplayıcı
git geçmişinden (bu commit'ten önceki hâl) geri alınır. Birlikte dönmesi
gerekenler: `jetson_kur.sh --veriseti-servis` kolu, `girdap-algi.service`
içindeki `Conflicts=girdap-veriseti.service` satırı ve `test_saat.py`'deki
`test_manifest_*` blokları — bunlar tek başlarına anlamsız, hep birlikte
anlamlılar.

🔴 **KALINTI:** Eski kurulumlarda `/etc/systemd/system/girdap-veriseti.service`
hâlâ duruyor olabilir. Denetim `bash scripts/jetson_kontrol.sh` içinde;
temizlik: `sudo rm /etc/systemd/system/girdap-veriseti.service && sudo systemctl daemon-reload`.

## Testler (donanım GEREKMEZ)
```bash
cd son_kodv2/algi && PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest -q   # 196 passed
```
⚠️ Jetson'da `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1` **şart**: ROS Humble'ın
`launch_testing` eklentisi buradaki pytest sürümüyle uyumsuz ve süiti
`INTERNALERROR` ile düşürüyor (tek bir test bile koşmadan). Bayrak olmadan
"testler kırık" sanılır — kırık olan eklenti.
```bash
# 236 -> 196: 16.08'de veri seti toplayıcısıyla birlikte 40 test kaldırıldı
# (test_oak_veriseti.py'nin 37'si + test_saat.py'deki 3 `test_manifest_*`).
```

## 🔴 ÇALIŞTIRMA — bu node'u KİMSE otomatik başlatmıyor, farkında olun

```bash
ros2 launch girdap_ida_algi algi.launch.py     # elle
```

**Karar tarafının `hardware.launch.py`'si bu node'u BAŞLATMIYOR** — 1.010
satırda `girdap_ida_algi` hiçbir `Node()` çağrısında geçmiyor. Bu bir kusur
değil, bilinçli iş bölümü: bize **yol açıyorlar** (`use_onboard_camera=false`,
`with_oak_driver=false`) ama başlatmıyorlar. Sonuç: kimse elle başlatmazsa
`/perception/buoys` **hiç akmaz**, füzyon tüm LiDAR kümelerini
`CLASS_UNKNOWN` bırakır ve `gate_follower` ham GPS'e düşer — **hiçbir hata
basılmadan**. (Fusion'ın `sync_watchdog_s` WARN'ı var ama sahada kimse
journal okumaz.)

**Yarışmada elle başlatmak da mümkün değil:** md 5.5.3.1 — YKİ'de yalnız YKİ
arayüzü, başlatma komutu kablodan verilmez; md 4.1 — WiFi/BT kapalı, yani SSH
yok. ⇒ **Tek geçerli yol systemd autostart.**

```bash
bash scripts/jetson_kur.sh --servis        # unit'i kurar + enable eder
systemctl is-enabled girdap-algi           # "enabled" görmeden sahaya çıkma
bash scripts/jetson_kontrol.sh             # 6b bloğu unit'i ayrıca denetler
```

⚠️ **Paketimiz karar tarafının `ros2_ws/src`'inde DEĞİL** (bu klasör ayrı) →
tek `colcon build` ile derlenmiyor. `jetson_kur.sh` bizim paketi
`~/ros2_ws/src/girdap-ida-algi` altına klonlayıp ayrıca derliyor; teknede
koşan kopya **odur**, bu ayna klasör değildir.

🔴 **07.08'de bulunan ve düzeltilen 3 başlatma hatası** (hepsi "yarışma günü
telafisi yok" sınıfıydı, ayrıntı `girdap-ida-algi@605d713`):
1. `girdap-algi.service`'te **`WorkingDirectory` yoktu** → boot'ta cwd `/`
   olur, depthai önbelleğini cwd'ye göreli yazdığı için `/.cache` = Permission
   denied → **node hiç açılmazdı**. Elle çalıştırınca görünmeyen, yalnız
   boot'ta çıkan arıza (veri seti servisinde 05.08'de birebir yaşanmıştı).
2. `jetson_kur.sh` **`depthai>=3.6` kuruyordu** → betiği koşan Jetson'ı v3'e
   yükseltip **stereo'yu öldürüyordu**. Artık `==2.30.0.0` pinli ve idempotent.
3. `jetson_kur.sh --servis` **servisi hiç kurmuyordu**: 6/6 adımındaki
   `MODEL_NNARCHIVE` grep'i (kod artık `MODEL_BLOB`) eşleşmeyip `set -e`
   altında betiği öldürüyordu; `--servis` bloğuna ulaşılmıyordu.

Gereken son girdi: yukarıdaki **blob + config.json** — henüz üretilmedi (veri
seti bekleniyor). Model olmadan node ilk satırda ölür.

📌 **Karar tarafından beklediğimiz bir şey yok**, ama bilinsin diye:
`/perception/gate_count`, `/perception/gate_target`, `/perception/buoys_3d`
topic'lerinin şu an **abonesi yok**. Özellikle `buoys_3d`, LiDAR düşerse
kamera-only engel haritası olarak kullanılabilirdi
(`/perception/obstacle_map` ile aynı kodlama).

## 🔴 DONANIM BULGULARI (ÖLÇÜM, tahmin değil)

**0) depthai SÜRÜMÜ 2.30.0.0 — v3'e DÖNÜLMEZ.** v3 firmware'i bu cihazda
mono/stereo'yu açamıyor (stereo **%0**); v2'de stereo 29,7 FPS, YOLO+derinlik
18,1 FPS — üstelik USB2 linkinde. v3'e dönmek hem bu node'u hem veri seti
toplayıcısını kırar.

**1) OAK USB2'ye ZORLANIYOR — `dai.Device(pipeline, dai.UsbSpeed.HIGH)`.**
Bu Jetson'da (L4T R36.5) SuperSpeed linki `tegra-xusb`ın U1/U2 güç durumu
pazarlığında çöküyor → `X_LINK_DEVICE_NOT_FOUND`. Kernel logu 2 saatte 100×
*"Disable of device-initiated U1 failed"* + error -71; hataların **tamamı**
SuperSpeed yolunda, high-speed yolunda sıfır.
⇒ HIGH'a zorlanınca **5/5 açılış** (otomatik pazarlıkta ~6 denemede 1).
Bant genişliği kaybı **yok**: pipeline ~15 MB/s, USB2 tavanı ~35-40 MB/s.
📌 06.08 kararı: **USB3 kovalanmayacak** — kazancı yok, bozuk olan yol zaten
SuperSpeed. USB portlarını bounce eden systemd betiği **EKLENMEYECEK** (aynı
donanımda kanıtlı zararlı çıktı).

**2) Kilit yazılımdan açılıyor** — `girdap_ida_algi/oak_baglanti.py`,
`usb_reset()` sudo'suz `USBDEVFS_RESET` atıyor (udev `MODE=0666`), cihaz
0,5 sn'de dönüyor. Teknede fişe erişim olmayacağı için bu **zorunlu**.
⚠️ Kalan risk: **controller** hang'inde yazılımsal kurtarma yok (JetPack 6.2'de
oto-kurtarma kaldırılmış) → Jetson'a güç kes-ver gerekir.

**3) `setDepthAlign(CAM_A)` yanında `setOutputSize()` ZORUNLU.** Tek başına
derinliği RGB çözünürlüğüne (1920×1080 = 4,1 MB/kare) ölçekliyor, USB'yi
dolduruyor: **8,1 FPS**. `setOutputSize(640,400)` ile **14,7 FPS**.

**4) RGB modu: `THE_12_MP` + `setIspScale(1,3)` = 1352×1014.**
`THE_1440X1080` bu cihazda **sessizce 0 kare** üretiyor (enum'da olmak ≠
çalışmak). 12MP tam 4:3 = kırpma yok, veri setiyle aynı çerçeve.

**5) FPS = 11.** Boru hattı tavanı 12,2 ölçüldü (YOLO+stereo birlikte);
11 = tavanın %10 altı. Termal plato 20 dk'da **68-69 °C**, kısma yok.
🔴 Ama cihazda **otomatik termal kısma YOK** (çip 105 °C anma, gözlenen çökme
125 °C) ve bu ölçüm **iç mekân**. Güneş etkin ortamı +10-20 °C itiyor ⇒
**gölgelik ŞART** (FPS düşürmekten ~10 kat etkili). Güvenlik ağı kodda:
`sicaklik_durumu()` uyarı 85 / kritik 95 °C.

Ölçüm araçları (tekrar çalıştırılabilir):
`scripts/oak_derinlik_termal_testi.py` · `scripts/duba_kamera_test.py`
(deploy pipeline'ının aynası) · `scripts/depthai_api_denetimi.py` (AST ile
v2/v3 kaçak çağrı avcısı — taşımalarda ÖNCE bu koşulur).

## 🚀 Jetson kurulumu
Teknedeki Jetson'a kurulum sırası: **`JETSON-KURULUM.md`** (11 adım, her adımın gerekçesiyle).
🔴 En kritik: servis `ROS_DOMAIN_ID=42` ile kalkmalı — karar yığını 42'de, aksi hâlde iki taraf birbirini HİÇ görmez.
