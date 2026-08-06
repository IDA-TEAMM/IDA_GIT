# algi — GİRDAP görüntü işleme katmanı (son_kodv2'nin algı ayağı)

**Kaynak repo:** github.com/EyupEker1/girdap-ida-algi — commit `ee3d623`
**Kopya tarihi:** 2026-08-06 (bu klasör kaynağın **birebir aynası**; burada
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

## Veri seti toplama (deniz oturumu — PC/EKRAN YOK)
Model dosyası henüz yok; veri seti denizde toplanacak, dosyalar sonradan
alınacak. Toplayıcı ekransız çalışır ve **açılışta kendi başına başlar**
(reboot testi 05.08'de geçti):

```bash
scripts/oak_veriseti_topla.py      # 1352×1014 4:3 (deploy FOV'u ile AYNI), manifest.csv'li
scripts/girdap-veriseti.service    # systemd; boot'ta kalkar
docs/veriseti_deniz_oturumu.md     # kıyı kontrol listesi (ATLANMAZ)
```
⚠️ Toplayıcı ile algı node'u **aynı anda çalışamaz** (tek OAK).
🔴 **Yarışma günü `sudo systemctl disable girdap-veriseti`** — yoksa boot'ta
kamerayı kapar ve algı node'u açılamaz (md 4.1: WiFi/BT kapalı, görüntü karaya
aktarılmaz).

## Testler (donanım GEREKMEZ)
```bash
cd son_kodv2/algi && python3 -m pytest -q          # 116 passed (kokten de calisir)
```

## Çalıştırma
```bash
ros2 launch girdap_ida_algi algi.launch.py
```
Gereken: yukarıdaki **blob + config.json** — henüz üretilmedi (veri seti
bekleniyor). Model olmadan node açılmaz.

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
