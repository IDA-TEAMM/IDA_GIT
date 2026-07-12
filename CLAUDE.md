# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Proje

Bu ROS2 Humble workspace'i, **TEKNOFEST 2026** için geliştirilen **IDA/Girdap** insansız su üstü aracının (USV) yazılımını içerir (Takım 989124, Alt Alan B). Aktif geliştirme `src/ida_topics_yeni/` altındaki `ida_topics` paketinde yapılır — repodaki diğer her şey ya vendored ROS2/Gazebo bağımlılığı ya da referans amaçlı simülasyon paketidir.

Proje şu anda Gazebo simülasyonundan gerçek donanıma (Pixhawk/ArduPilot, F9P GPS, OAK-D Lite kamera, Livox Mid-360 LiDAR) geçiş aşamasındadır.

## Repo yapısı

- `src/ida_topics_yeni/` — **tek aktif paket** (`ament_python`, paket adı `ida_topics`). Tüm USV node'ları burada.
- `src/girdap_yenimodel/` — Gazebo simülasyon modeli (`ament_cmake`, paket adı `Girdap`): `urdf/`, `meshes/`, `launch/`, `config/`, `textures/`. Artık gerçek donanım hedeflendiği için referans niteliğindedir.
- `src/ament/`, `src/ros2/`, `src/ros/`, `src/osrf/`, `src/eProsima/`, `src/eclipse-cyclonedds/`, `src/eclipse-iceoryx/`, `src/gazebo-release/`, `src/gazebo_ros_pkgs/`, `src/ros-tooling/`, `src/ros-planning/`, `src/ros-perception/`, `src/ros2-rust/`, `src/rqt*/`, `src/vision_msgs/`, `src/vision_opencv/` — `ros2.repos`'tan çekilmiş vendored ROS2/Gazebo çekirdek paketleri (git submodule benzeri, `git status`'ta `m` olarak görünürler). Bunlara **dokunma** — proje mantığı içermezler, sadece build bağımlılığıdır.
- `build/`, `install/`, `log/` — colcon çıktı dizinleri, git'e dahil değil (`.gitignore`).

## Komutlar

Bu proje neredeyse tamamen bir **Docker container içinde** çalıştırılır (container: `ros2_final`, host `~/ros2_ws` ↔ container `/root/ros2_ws` volume mount ile senkron). `colcon build` sadece `ida_topics` paketini ROS2 index'ine kaydetmek için gerekir; asıl node'lar `ros2 run` yerine doğrudan `python3` ile çalıştırılır (aşağıya bakın).

```bash
# Sadece ida_topics paketini derle (container içinde)
colcon build --packages-select ida_topics
source install/setup.bash

# Tüm sistemi başlat (11 node + MAVROS, container içinde çalıştırılmalı)
bash src/ida_topics_yeni/sistem_baslat.sh
# ya da host'tan:
docker exec -it ros2_final bash -c "bash /root/ros2_ws/src/ida_topics_yeni/sistem_baslat.sh"

# Tek bir node'u elle test etmek için (çoğu node ros2 run ile kayıtlı değil, bkz. aşağıdaki not)
python3 -u src/ida_topics_yeni/ida_topics/<node_adi>.py --ros-args -p <param>:=<deger>

# Syntax kontrolü (bu projede kullanılan tek "test" şekli)
python3 -m py_compile src/ida_topics_yeni/ida_topics/*.py
```

**Önemli:** `setup.py`'deki `entry_points` yalnızca `sensor_node`, `perception_node`, `control_node`, `decision_node` içerir — bu 4'ü `ros2 run ida_topics <node>` ile çalıştırılabilir. Diğer 7 node (2 driver hariç tüm driver'lar, `telemetri_node`, `kamera_kayit_node`, `local_map_node`) kayıtlı değildir, sadece doğrudan `python3 dosya.py` ile çalıştırılabilir — `sistem_baslat.sh` de bu yüzden hepsini `python3 -u` ile başlatır, `ros2 run` kullanmaz. Yeni bir node eklerken bu tutarsızlığı akılda tut (entry_points'e eklemek isteğe bağlı, script zaten doğrudan çalıştırıyor).

## Mimari: 11-node pipeline

`sistem_baslat.sh` node'ları şu sırayla başlatır (Pixhawk/F9P GPS portlarını otomatik algılar, yoksa test moduna düşer):

```
1. gps_imu_driver_node  ─┐
2. oakd_driver_node      ├─→ donanım driver katmanı (gerçek sensör → standart ROS2 topic)
3. livox_driver_node    ─┘
4. sensor_node            → sensör toplama/timeout izleme, /sensor/* + /diagnostics yayınlar
5. perception_node        → YOLO11 + HSV renk filtresi ile duba tespiti
6. MAVROS (mavros_node)   → Pixhawk/ArduPilot köprüsü
7. decision_node          → waypoint navigasyon + cascade PID + engel kaçınma → cmd_vel
8. control_node           → MAVROS entegrasyonlu güvenlik/aktüasyon katmanı (RC kill-switch, watchdog)
9. telemetri_node         → şartname zorunlu: 1Hz CSV kaydı
10. kamera_kayit_node     → şartname zorunlu: YOLO bbox overlay'li MP4 kaydı
11. local_map_node        → şartname zorunlu: LiDAR tabanlı 2D OccupancyGrid + PGM kaydı
```

Katmanlı tasarımın önemli noktası: **driver node'ları** (1-3) gerçek donanımı standart ROS2 sensör topic'lerine (`/gps/fix`, `/imu/data`, `/camera/image_raw`, `/lidar/scan`, `/lidar/points`) çeviriyor — bu aynı topic'ler daha önce Gazebo `ros_gz_bridge` tarafından besleniyordu, yani `sensor_node`den sonraki hiçbir node'un sim mi gerçek donanım mı olduğunu bilmesi gerekmiyor.

Katman akışı:
- **Sensör katmanı** (driver'lar → `sensor_node`): ham sensör verisini `/sensor/*` altında yeniden yayınlar, timeout/sağlık izlemesi yapar, `/diagnostics` üretir.
- **Algı katmanı** (`perception_node`): `/root/best.pt` YOLO11 modeli (container-içi, repo'da yok) + HSV eşiği (turuncu H=10-34, sarı H=35-55) ile `/perception/orange_buoys`, `/perception/yellow_buoys` üretir.
- **Karar katmanı** (`decision_node`): sabit `WAYPOINTS` listesi + cascade PID (dış döngü heading→yaw_rate, iç döngü yaw_rate→thrust) + yakın dubalardan kaçınma mantığı. Pozisyon kaynağı `/mavros/local_position/odom` (MAVROS EKF, BEST_EFFORT QoS gerektirir).
- **Kontrol katmanı** (`control_node`): `/cmd_vel`'i alıp MAVROS'a (`/mavros/setpoint_velocity/cmd_vel`) iletir; RC kanal 8 (idx 7) kill-switch, RC kanal 5 (idx 4) manuel override, 2s cmd_vel watchdog'u içerir.
- **Kayıt katmanı** (`telemetri_node`, `kamera_kayit_node`, `local_map_node`): şartname gereği zorunlu loglama/kayıt, `/tmp/{telemetri,kamera,local_map}/` altına zaman damgalı dosyalar yazar.

**Not:** `decision_node.py` eskiden Gazebo-özel `/model/Girdap/cmd_vel` topic'ine yayın yapıyordu, `control_node.py` ise `/cmd_vel`'i dinliyordu (decision_node çıktısı control_node'a ulaşmıyordu). Bu 2026-07-12'de düzeltildi — artık `decision_node.py` de `/cmd_vel`'e yayın yapıyor.

## Kritik teknik detaylar

- **MAVROS QoS:** MAVROS topic'leri (`/mavros/*`) BEST_EFFORT yayınlar. rclpy varsayılanı RELIABLE'dır ve uyumsuzluk sessizce veri kaybına yol açar (hata fırlatmaz). Yeni bir `/mavros/*` subscriber'ı eklerken mutlaka `QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT)` kullan (örnek: `decision_node.py`).
- **`mavros_msgs.msg.VfrHud`** doğru sınıf adıdır, `VFR_HUD` değil — yanlış yazarsan import hatası alırsın.
- **`decision_node.py`'de `cmd.angular.z = -angular`** kasıtlı işaret düzeltmesidir (ArduPilot yaw yönü uyumu için), bug değildir — kaldırma.
- **`livox_driver_node.py`'de `host_ip` parametresi `'0.0.0.0'` olmalı** — belirli bir arayüz IP'si UDP socket bağlama hatası verir.
- **Opsiyonel donanım portları:** F9P GPS portu boşsa (`GPS_PORT=""`), `sistem_baslat.sh`'daki gibi `-p port:=...` argümanını hiç geçirme — boş string parametre node başlatmasını bozar.
- **`config_ekf.yaml`** (`robot_localization` EKF ayarları) Gazebo simülasyonu içindir; gerçek donanımda MAVROS'un kendi EKF'i kullanılıyor.
- **`ROS_DOMAIN_ID=42`** hem laptop hem Jetson `~/.bashrc`'sinde hem de `sistem_baslat.sh` içinde (`source /opt/ros/humble/setup.bash`'ten ÖNCE) export edilmeli — Jetson↔laptop topic keşfinin çalışması buna bağlı.
