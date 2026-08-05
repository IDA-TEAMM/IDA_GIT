# algi — GİRDAP görüntü işleme katmanı (son_kodv2'nin algı ayağı)

**Kaynak repo:** github.com/EyupEker1/girdap-ida-algi — commit `91aecfc`
**Kopya tarihi:** 2026-08-05 (önce `algi_kamera/` idi, `son_kodv2/algi/`e taşındı)
**Sorumlu:** Eyüp (görüntü işleme)

Bu klasör son_kodv2'nin **kamera algısı**dır. `karar/` (Sude) karar/görev
katmanı, burası tespit katmanı. Kimse kimsenin dosyasına dokunmuyor; bağlantı
aşağıdaki topic sözleşmesi üzerinden.

## Bu katman ne yapar
OAK-D Lite'ın **içindeki** Myriad X VPU'da YOLOv11n (416×416) koşturur; Jetson
CPU'suna tespit maliyeti ~sıfırdır. Aynı pipeline'da stereo derinlik gelir.

| Topic | Tip | Anlamı |
|---|---|---|
| `/perception/buoys` | `vision_msgs/Detection2DArray` | bbox, class_id "0"=kenar "1"=engel — **piksel uzayı 1280×720** (aşağıya bak) |
| `/perception/buoys_3d` | `geometry_msgs/PoseArray` | stereo 3D duba konumu (bonus; `/perception/obstacle_map` sözleşmesiyle aynı kodlama) |
| `/perception/gate_count` | `std_msgs/Int32` | geçilen **farklı** geçit sayısı |
| `/perception/gate_target` | `geometry_msgs/PoseStamped` | geçidin ötesindeki hedef (karar tarafı isterse kullanır) |

Ayrıca **Dosya-1** (md 4.2) mp4 kaydını üretir: bbox + sınıf overlay'li,
her karesi zaman etiketli, ≥1 Hz.

## 🔴 Bunu çalıştırırken karar launch'ında ŞUNLAR gerekiyor

1. **`use_onboard_camera:=false`** — varsayılanı `true` (F-P.22, 17.07). Açık
   kalırsa `perception_camera_node` de `/perception/buoys`'a basar → iki
   publisher, karışık bbox uzayı.
2. **`oakd_driver_node` açılmamalı** — `with_drivers:=true` TEK bayrak olarak
   Livox + OAK sürücüsü + `kamera_kayit_node`'u birlikte açıyor
   (`hardware.launch.py:866-895`). Livox'a ihtiyaç var ama OAK sürücüsü USB
   cihazını açınca **bu node kamerayı hiç açamaz** (tek OAK, tek süreç).
   → Sürücü bayrağının ayrılması gerekiyor (karar tarafının dosyası).
3. **`camera_image_width_px`/`height_px` = 1280×720 kalmalı** — bu node artık
   bbox'ı bilerek o uzayda yayınlıyor (aşağı bak). Değiştirilecekse ikisi
   birlikte değişmeli.

## 🔴 Neden bbox 1280×720 uzayında (gerçek kare 640×480 olduğu hâlde)
`perception_fusion_node` bbox merkezini kendi `camera_image_width_px`
parametresine **bölerek** normalize ediyor; mesaj görüntü boyutunu taşımıyor.
O parametre üç yerde birden 1280×720 (`hardware.launch.py:211` ·
`hardware.yaml:207` · `params.yaml:228`). Biz 640 uzayında yayınlarsak bearing
sessizce kayar: kare **ortasındaki** duba +17°'de görünür (tolerans 8,6°),
sağ kenardaki duba merkezde görünür → karenin sağ %75'indeki hiçbir tespit
LiDAR kümesine eşleşmez → sınıf bilgisi düşer → `gate_follower` kenar dubası
göremez → geçitten geçilmez (P1: G1/KD1≥0,5 · P2: ≥2 ikili).
Hiçbir hata basılmaz. Bu yüzden yayın uzayı gerçek kare boyutundan ayrıldı
(`gecit_mantik.bbox_piksel`). Yatayda letterbox tam FOV koruduğu için ölçek
yeterli, HFOV (~69°) değişmiyor.

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

## Veri seti toplama (deniz oturumu — PC/EKRAN YOK)
Model dosyası henüz yok; veri seti denizde toplanacak, dosyalar sonradan
alınacak. Toplayıcı ekransız çalışır ve **açılışta kendi başına başlar**:

```bash
scripts/oak_veriseti_topla.py      # 4:3 (deploy FOV'u ile aynı), manifest.csv'li
scripts/girdap-veriseti.service    # systemd; Conflicts=girdap-algi
docs/veriseti_deniz_oturumu.md     # kıyı kontrol listesi (ATLANMAZ)
```
⚠️ Toplayıcı ile algı node'u **aynı anda çalışamaz** (tek OAK).

## Testler (donanım GEREKMEZ)
```bash
cd son_kodv2/algi && python3 -m pytest girdap_ida_algi/test/ -q   # 78 passed
```

## Çalıştırma
```bash
ros2 launch girdap_ida_algi algi.launch.py
```
Gereken: `~/models/yolo11n_duba_rvc2.tar.xz` (NN Archive) — **henüz üretilmedi**,
veri seti toplama + eğitim sürüyor. Model olmadan node açılmaz.

## 🔴 2026-08-05 — donanım bulguları (ÖLÇÜM, tahmin değil)

**1) OAK USB2'ye ZORLANIYOR — `dai.Device(dai.UsbSpeed.HIGH)`.**
Bu Jetson'da (L4T R36.5) SuperSpeed linki `tegra-xusb`ın U1/U2 güç durumu
pazarlığında çöküyor: cihaz firmware'i yükleyip USB3'e geçiyor, link dağılıyor,
ROM'a düşüyor → `X_LINK_DEVICE_NOT_FOUND`. Kernel logu 2 saatte 100×
*"Disable of device-initiated U1 failed"* + error -71; hataların **tamamı**
SuperSpeed yolunda, high-speed yolunda sıfır. NVIDIA forumunda aynı platformda
aynı belirti kayıtlı (Luxonis `xusb-tegra`'yı işaret ediyor, konu çözümsüz).
⇒ HIGH'a zorlanınca **5/5 açılış** (otomatik pazarlıkta ~6 denemede 1).
Bant genişliği kaybı **yok**: pipeline ~15 MB/s, USB2 tavanı ~35-40 MB/s.

**2) Kilit yazılımdan açılıyor** — `girdap_ida_algi/oak_baglanti.py`,
`usb_reset()` sudo'suz `USBDEVFS_RESET` atıyor (udev `MODE=0666`), cihaz
0,5 sn'de dönüyor. Teknede fişe erişim olmayacağı için bu **zorunlu**.

**3) `setDepthAlign(CAM_A)` yanında `setOutputSize()` ZORUNLU.** Tek başına
derinliği RGB çözünürlüğüne (1920×1080 = 4,1 MB/kare) ölçekliyor, USB'yi
dolduruyor: **8,1 FPS**. `setOutputSize(640,400)` ile **14,7 FPS**.

**4) Termal — cihazda OTOMATİK KISMA YOK.** Luxonis: çip anma sınırı 105 °C,
gözlenen çökme 125 °C; OAK-D **Lite** küçük soğutuculu (azami ortam ~40 °C).
Ölçüm (11 FPS, derinlik+YOLO açık, 20 dk): VPU 63,5 → 68,7 °C, tepe **69,3**,
plato oturdu, FPS 11,00 sabit. ⇒ İç mekânda pay bol; **güneş altında değil** —
etkin ortamı +10-20 °C ittiği için **gölgelik şart**. Güvenlik ağı kodda:
`sicaklik_durumu()` (uyarı 85 °C, kritik 95 °C), toplayıcı 60 sn'de bir okuyor.

Ölçüm aracı: `scripts/oak_derinlik_termal_testi.py` (tekrar çalıştırılabilir).
