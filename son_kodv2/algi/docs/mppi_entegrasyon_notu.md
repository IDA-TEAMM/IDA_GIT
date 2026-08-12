# girdap-decision Entegrasyon Notu (Algı ↔ Karar Sözleşmesi)

> **Revizyon (2026-07-10):** Bu notun önceki sürümü karar tarafının **Nav2
> `nav2_mppi_controller`** kuracağı varsayımıyla yazılmıştı. Karar reposu
> yayınlandı ([vistastris/girdap-decision](https://github.com/vistastris/girdap-decision))
> ve gerçek mimari farklı çıktı: **Nav2 yok** — arkadaş kendi RRT* + MPPI
> (NumPy) + iSAM2/GTSAM + FSM yığınını yazmış. `/goal_pose` diye bir arayüz
> o repoda geçmiyor. Bu not baştan yazıldı; eski Nav2 içeriği (costmap YAML,
> inflation hesabı, bt_navigator) tamamen kaldırıldı.

## 1. Sorumluluk dağılımı

```
   BİZ (girdap-ida-algi)                  ARKADAŞ (girdap-decision)
┌──────────────────────────┐         ┌─────────────────────────────────┐
│ OAK-D Lite               │         │ perception_fusion_node          │
│  VPU: YOLO11n 512x512    │ buoys   │  (kamera+LiDAR bearing füzyonu) │
│  + StereoDepth   11 FPS  │────────►│         │                       │
│                          │         │         ▼                       │
│ duba_gecis_navigator     │ gate_   │ planning_node (RRT* + MPPI)     │
│  MOD="algi_yayin"        │ passed  │ fsm_node / mission_manager      │
│  geçit seç + odom'la     │────────►│         │                       │
│  geçişi DOĞRULA          │         │         ▼                       │
└──────────────────────────┘         │ /mavros/setpoint_velocity/...   │
                                     └─────────────────────────────────┘
```

| Sorumluluk | Kim |
|---|---|
| Duba tespiti (kamera, gerçek YOLO + stereo 3D) | **Biz** |
| Geçit seçimi + odometriyle geçiş doğrulama (`gate_passed`) | **Biz** |
| LiDAR engel tespiti (`/perception/obstacle_map`) | Arkadaş (Mid-360 pipeline'ı onda) |
| Sensör füzyonu, global/lokal planlama, MPPI, FSM, MAVROS | Arkadaş |
| Hedef/hız komutu üretimi | **Sadece arkadaş** (tek dümen kuralı) |

Onun `perception_camera_node`'u mock-YOLO + HSV ile geçici yazılmış; bizim
node'umuz o mock'un **gerçek karşılığıdır**. Aynı anda ikisi birden
`/perception/buoys` yayınlamamalı — sahada onun kamera node'u kapatılır.

## 2. Yayınladığımız topic'ler (`MOD = "algi_yayin"`)

### `/perception/buoys` — `vision_msgs/Detection2DArray`

Onun `perception_camera_node` şemasıyla birebir:

- `header.frame_id = "oak_rgb"`, stamp = yayın anı
- `bbox.center.position.{x,y}`, `bbox.size_{x,y}` — 🔴 **1280×720 piksel uzayı**
  (04.08'de 640×480'den ÇIKARILDI — E-1: füzyon `camera_image_width_px=1280`
  okuyor; 640 yayınlamak bearing'i sessizce ~17° kaydırıyor ve P1+P2'yi
  sıfırlıyordu. `IDA_GIT@d7c5b56`. İki taraf **birlikte** değişir.)
- `results[0].hypothesis.class_id` **string**: `"0"`=parkur kenarı (turuncu),
  `"1"`=engel (sarı); `results[0].hypothesis.score` = güven
- Boş dizi de yayınlanır (taze kare + tespit yok bilgisi, füzyon senkronu için)

**Ön işleme notu (letterbox DEĞİL, SIKIŞTIRMA):** deploy `setPreviewKeepAspect
Ratio(False)` ile 4:3 kareyi (1352×1014) 512×512'ye **eziyor** → üst/alt şerit
oluşmuyor → çıkarılacak pay YOK, kodda `self._lb_pay = 0.0`. Normalize bbox
her iki eksende doğrudan ölçekleniyor. ⚠️ Eğitim de **aynı** ön işlemeyle
(stretch) yapılmalı — Ultralytics varsayılanı letterbox'tır.

### `/perception/gate_passed` — `std_msgs/Bool`

Onun `fsm_node`'unun parkur geçiş kanalı. Yalnız **odometriyle doğrulanmış**
geçişte bir kez `True` basılır: geçit çizgisi tespit anında sabit çerçeveye
kilitlenir, araç çizgiyi `PASS_EK_YOL` kadar aşınca geçiş sayılır. Poz
kaynağı **TF değil** — girdap-decision TF yayınlamadığı için
`/girdap/fusion/odom` (nav_msgs/Odometry) aboneliği kullanılır.

### `/perception/buoys_3d` — `geometry_msgs/PoseArray` (BONUS/öneri)

OAK stereo'dan **gerçek 3D duba konumu**, `base_link` çerçevesinde,
`/perception/obstacle_map` ile aynı şema (`position.{x,y}` = merkez,
`orientation.z` = yarıçap [m], `orientation.w = 1`). Arkadaşın bearing
füzyonu, kamerada 3D olmadığı varsayımıyla yazıldı — bizde var. İsterse bu
topic'i doğrudan MPPI engel girişine besleyebilir veya füzyonda LiDAR'la
çapraz doğrulama yapabilir. **Karar onun; biz sadece yayınlıyoruz.**

### Yayınlamadıklarımız

- `/perception/obstacle_map` — LiDAR node'unun çıkışı, ona aittir.
- `/goal_pose`, `cmd_vel` — bu modda ASLA (tek dümen kuralı).

## 3. Bizim taraftaki modlar

| MOD | Ne yapar | Durum |
|---|---|---|
| `algi_yayin` | Yukarıdaki 3 topic; komut basmaz | **VARSAYILAN (Plan A)** |
| `mppi_hedef` | `/goal_pose` (Nav2 varsayımı) | Arşiv — ancak Nav2 kurulursa |
| `dogrudan_surus` | MAVROS'a TwistStamped | Plan B saha yedeği |

## 4. Kurulum ve birlikte koşturma

```bash
sudo apt install ros-humble-vision-msgs        # Detection2DArray için
mkdir -p ~/girdap_ws/src && cd ~/girdap_ws/src
git clone git@github.com:EyupEker1/girdap-ida-algi.git
git clone https://github.com/vistastris/girdap-decision.git
cd ~/girdap_ws
colcon build --symlink-install \
  --packages-select girdap_ida_algi girdap_decision
source install/setup.bash
# Karar stack'i (onun launch'ı) + bizim algı node'u:
ros2 launch girdap_decision hardware.launch.py   # (onun README'sine bak)
ros2 run girdap_ida_algi duba_gecis_navigator
```

> ⚠️ **numpy uyarısı (ona iletilecek):** girdap-decision `requirements.txt`
> `numpy>=1.26` diyor, üst sınır yok → temiz kurulumda numpy 2.x gelir ve
> ROS Humble + scipy + matplotlib zincirini kırar (`_ARRAY_API not found`).
> Düzeltme: `numpy>=1.26,<2`.

## 5. Test merdiveni

1. **Masa (ROS'suz):** `python3 scripts/duba_kamera_test.py` — kutular doğru
   yerde mi (ön işleme stretch; dikey şerit düzeltmesi YOK, pay 0).
2. **Topic doğrulama:** node açıkken
   `ros2 topic echo /perception/buoys --once` — class_id `"0"/"1"` string mi,
   bbox **0-1280 / 0-720** aralığında mı (E-1 sonrası uzay).
3. **Füzyon smoke:** onun `perception_fusion_node` + bizim buoys →
   `/perception/classified_obstacles`'da sınıflı engel çıkıyor mu
   (LiDAR yoksa onun `mock_sensors`'ü kullanılabilir).
4. **gate_passed simülasyonu:** `/girdap/fusion/odom`'a sahte ilerleyen odom
   bas, önüne sentetik geçit tespiti ver → çizgi aşımında tek `True` gelmeli.
5. **Saha:** tam yığın, önce onun `use_isam2=false` video modu.

## 6. Arkadaştan beklenen cevaplar

1. `/girdap/fusion/odom` hangi frekansta ve hangi çerçevede? (Geçiş
   doğrulamamız buna abone.)
2. `camera_hfov_rad=1.2` OAK-D Lite'a göre mi? (Gerçek HFOV ~69° ≈ 1.20 rad
   — tesadüfen tutuyor, teyit etsin.)
3. `buoys_3d`'yi kullanacak mı? Kullanacaksa bearing füzyonundaki kamera
   dalını atlamak ister mi?
4. Sahada onun `perception_camera_node`'u kapalı mı olacak (çift yayın olmasın)?
5. numpy pini (madde 4 uyarısı) düzeltilecek mi?

## 7. Sorun giderme

| Belirti | Muhtemel neden |
|---|---|
| Füzyon hiç eşleşme bulmuyor | bearing işareti ters (onun `bearing_from_camera` docstring'i) veya bizim bbox x ölçeği yanlış |
| bbox'lar dikeyde kaymış | ön işleme ayrışmış: deploy stretch ama blob letterbox'la eğitilmiş (ya da tersi) — ikisi birlikte değişmeli |
| `gate_passed` hiç gelmiyor | `/girdap/fusion/odom` yayında değil (zaman aşımı logda "poz yok" der) |
| `vision_msgs` import hatası | `ros-humble-vision-msgs` kurulu değil |
| İki kamera tespiti kaynağı | onun mock kamera node'u da açık — kapat |
