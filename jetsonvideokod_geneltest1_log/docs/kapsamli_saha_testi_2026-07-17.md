# Kapsamlı Saha Testi — Video Hazırlığı (2026-07-17, canlı donanım)

> Jetson + Pixhawk (TELEM2/FTDI `/dev/pixhawk`) + GPS fix, dışarıda. Servis
> `girdap-karar` restart @19:13 (Pixhawk bağlandıktan SONRA). Salt-okunur test —
> ARM/motor komutu YOK. Tüm ölçümler canlı ROS_DOMAIN_ID=42.

## Özet: karar-yazılım sahada TAM ÇALIŞIYOR. Boşluklar sensör bring-up + operasyon.

---

## A. Yazılım regresyonu
- İzole (domain 99): **340 passed / 2 skipped** (2 skip = MPPI CUDA, beklenen).
- Canlı yığınla aynı domain'de (42) koşunca 3 node testi fail etti — **DDS çakışması**
  (canlı mission_manager/fsm/video-e2e node'ları test kopyalarıyla çarpışıyor).
  İzole domain 99'da 21/21 geçti → **gerçek regresyon YOK, ortam artefaktı.**
  *Ders: canlı yığın açıkken suite'i farklı domain'de koş.*

## B. Video mod config ✅
`use_isam2:false · use_rrt:false · use_mppi:true · mission_source:fc` — doğru.

## C. Donanım bağlantısı
| Bileşen | Durum |
|---|---|
| Pixhawk | ✅ `/dev/pixhawk`→ttyUSB0 (FTDI DU0EFEA7, TELEM2) |
| FC bağlantı | ✅ `connected:true` · HEARTBEAT ~8 sn (busy YOK) |
| FC arm/mod | ⚠️ `armed:true` · mode HOLD · guided:false |
| GPS | ✅ fix_type 4 (DGPS) · 20 uydu · lat/lon gerçek |
| LiDAR (Livox) | 🔴 topic var, **Publisher 0** — sürücü başlatılmamış |
| Kamera (OAK-D) | 🔴 bağlı değil — `/oak` yok, camera_node yok |

## D. Node envanteri ✅
mavros(+plugins), mavros_bridge, planning, fsm, mission_manager, fusion,
telemetry, local_map, perception_lidar, perception_fusion + static_tf'ler UP.
Eksik: **perception_camera** (kamera yok).

## E. Canlı topic akış hızları
| Topic | Hz | Değerlendirme |
|---|---|---|
| /girdap/fusion/odom | 9.99 | ✅ EKF pozu gerçek (x=1.53,y=-2.29) |
| /girdap/fusion/pose | 9.99 | ✅ |
| /mavros/local_position/pose | 7.37 | ✅ TELEM2 (57600) |
| /girdap/control/thrust | 10.0 | ✅ değer [0,0] (görev yok, doğru) |
| /girdap/map/local (Dosya-3) | 9.95 | ✅ |
| IMU / GPS | 7.2 | ✅ 5 Hz tabanın üstünde |
| /girdap/mission/current_target | — | ⚠️ FC görevi BOŞ (OPS-1, doğru) |
| /mavros/.../cmd_vel | — | ⚠️ HOLD'da tutuluyor (mod savaşı yok, doğru) |
| /livox/lidar | 0 | 🔴 sürücü yok |
| /perception/obstacle_map | 0 | 🔴 LiDAR verisi yok |

## F. Şartname md 4.2 teslim dosyaları
| Dosya | Durum |
|---|---|
| Dosya-1a (kamera mp4) | 🔴 kamera yok |
| Dosya-1b (LiDAR mp4) | 🔴 LiDAR verisi yok (sürücü başlatılmalı) |
| **Dosya-2 (telemetri CSV)** | ✅ **canlı, gerçek GPS/IMU, format birebir, fsync'li** |
| **Dosya-3 (cost map PNG)** | ✅ **1824 kare, aktif yazıyor** |
| Ekran-2 (grafik CSV) | ✅ yazıyor (setpoint/thrust görev başlayınca dolar) |

## G. Güvenlik ✅
KILL servisi `/girdap/mission/kill` + `/mavros/cmd/arming` + `/mavros/set_mode` var.
⚠️ FC şu an ARMED (HOLD) — test dışı disarm önerilir.

## H. Sürüş zinciri kapısı
cmd_vel HOLD'da bilinçli tutuluyor (B1: AUTO/HOLD'da FC ile kavga etmesin).
**Tam sürüş testi için:** FC'ye görev yükle + GUIDED/AUTO + arm → tekne HAREKET
eder. Bu ayrı, kontrollü adım (bu testte YAPILMADI, güvenlik).

---

## Sonuç ve kalan işler

**Karar-yazılım video için hazır:** bağlantı, GPS, EKF poz→fusion→MPPI→cost map,
Dosya-2/Dosya-3 hepsi canlı gerçek veriyle çalışıyor. Yazılım yeşil (340/2).

**Video için kalan (yazılım DEĞİL, sensör/operasyon):**
1. 🔴 **Livox sürücüsünü başlat** (hardware.launch'ta bilinçli yok — ayrı bring-up).
   → Dosya-1b + engel tespiti. Livox IP/ağ + `ros2 launch` sensör bring-up.
2. 🔴 **OAK-D kamera bağla** → Dosya-1a (bbox overlay). Kamera + camera_node.
3. ⚠️ **FC görevi yükle + GUIDED** → sürüş zinciri (cmd_vel) tam testi (tekne hareket).
4. ⚠️ Test dışı **disarm**.
5. FC-OLAY-2 (gaz kanalı + RC kalibrasyon) → FC ekibi.
