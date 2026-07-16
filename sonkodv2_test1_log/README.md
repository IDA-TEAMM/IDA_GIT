# son_kodv2 — İlk Gerçek Donanım Testi Log'u (Test 1)

**Tarih:** 2026-07-16
**Ortam:** Gerçek tekne donanımı (Pixhawk 6C, Livox Mid-360, OAK-D Lite, Holybro
H-RTK F9P Rover+Base) — Jetson yerine geçici olarak laptop kullanıldı (Jetson
o an Kaptan'ın video-günü koduyla meşguldü, dokunulmadı).
**Kapsam:** `girdap_decision` (`son_kodv2/karar/ros2_ws/src/girdap_decision`)
paketinin SITL dışında, **ilk kez gerçek sensör/FC donanımıyla** uçtan uca
canlı testi.

---

## 1. Kurulum Özeti

- Donanım tekneye monte edilmiş durumda, tek tek söküp masada test edilemedi.
- Jetson yerine bu oturumdaki laptop "karar bilgisayarı" rolü üstlendi:
  Livox ethernet → laptop `enp3s0`, OAK-D → USB3, Pixhawk → önce
  TELEM2/FTDI (`/dev/ttyUSB0`), sonra sorun nedeniyle USB-C direkt
  (`/dev/ttyACM1`) bağlantısına geçildi (bkz. Bölüm 3).
- `docker exec` ile `ros2_final` container'ı üzerinden çalışıldı
  (`--network host`, `/dev` bind mount, `privileged: true` — host'taki
  gerçek cihazları (ttyUSB/ttyACM/USB) sorunsuz gördü).
- RC kalibrasyonu QGroundControl üzerinden yapıldı: throttle ters yönü
  düzeltildi (`RC3_REVERSED`), iki ESC ayrı çıkışlara (`SERVO1_FUNCTION`,
  `SERVO3_FUNCTION`) bağlandı, bench testi için geçici `RCINx` passthrough
  denendi, sonra gerçek otonom kontrol için `ThrottleLeft`(73)/`ThrottleRight`(74)
  fonksiyonlarına geri alındı.
- GPS: kapalı alanda fix alınamadı, açık alana çıkıldı. H-RTK F9P Base
  ünitesi ayrı bir bilgisayara USB ile bağlanıp QGC üzerinden Survey-in
  yapıldı, RTK düzeltmesi teknedeki telemetri radyosu üzerinden Rover'a
  aktarılmaya çalışıldı (kısmen çalıştı, bkz. Bölüm 3).

---

## 2. Bulunan ve Düzeltilen Gerçek Kod Hataları

İkisi de TDD ile düzeltildi: önce regresyon testi yazıldı → gerçekten
FAIL olduğu doğrulandı → düzeltme yapıldı → test PASS oldu →
`prototype/tests/` tam takımı (`test_hardware_launch_config.py` dahil)
**385 passed, 4 skipped** (skip'ler yalnız GPU/cupy testleri) ile
0 regresyon doğrulandı.

### F-S.2 — `with_drivers:=true` hiçbir şey yapmıyordu (dead code)

- **Belirti:** `hardware.launch.py`'yi `with_drivers:=true` ile başlatınca
  `livox_driver_node`, `oakd_driver_node`, `kamera_kayit_node` node
  listesinde HİÇ görünmedi.
- **Kök sebep:** `driver_nodes` listesi launch dosyasında inşa ediliyordu
  (`IfCondition(with_drivers)` ile), ama fonksiyonun sonundaki
  `return LaunchDescription([...])` çağrısına hiç eklenmemişti —
  bayrağın kendisi ve koşul mantığı doğruydu, sadece son listeye
  unutulmuştu.
- **Düzeltme:** `*driver_nodes,` satırı `LaunchDescription` listesine
  eklendi (`decision_nodes`'dan hemen önce).
- **Test:** `test_with_drivers_node_lari_launch_descriptiona_eklenir`
  (`generate_launch_description()`'ı çağırıp dönen entity'ler arasında
  `livox_driver_node`/`oakd_driver_node`/`kamera_kayit_node`
  executable'larının gerçekten var olduğunu doğrular).
- **Canlı doğrulama:** Düzeltme sonrası gerçek Livox 10 Hz nokta bulutu,
  gerçek OAK-D görüntüsü, gerçek kamera kaydı (mp4) üretildi.

### F-P.20 — `mavros_node` çökünce hiç geri gelmiyordu

- **Belirti:** Gerçek testte FTDI/TELEM2 seri bağlantısı bir anlığına
  koptu, `mavros_node` şu hatayla çöktü:
  ```
  mavconn: serial0: receive: End of file
  terminate called after throwing 'std::system_error' — Resource deadlock avoided
  process has died [pid ..., exit code -6]
  ```
  Bizim node'larımız (`planning_node`, `mission_manager_node`) veri
  kesilince **doğru** davrandı (F-P.1/F-P.4 stale-guard'ları thrust'ı
  sıfırladı) ama sistem kendini hiç toparlamadı — mavros bir daha hiç
  dönmedi.
- **Kök sebep:** `mavros`'un kendi `apm.launch` → `node.launch` zincirinde
  `respawn_mavros` diye bir argüman VAR (varsayılan `false`), ama
  `node.launch`'taki gerçek `<node>` etiketine hiç bağlanmamış —
  argüman tanımlı, geçiriliyor, ama hiçbir yerde kullanılmıyor
  (mavros apt paketinin kendi hatası, F-S.2 ile aynı sınıf bug, bizim
  kodumuzda değil, `/opt/ros/humble/share/mavros/launch/node.launch`'ta).
  Canlı testte `respawn_mavros:=true` geçilmesine rağmen ikinci çökmede
  mavros_node bir daha hiç dönmediği doğrulandı.
- **Düzeltme:** `apm.launch`'ın `IncludeLaunchDescription`'ı tamamen
  bypass edilip `mavros_node` bizim launch dosyamızda doğrudan
  `Node(package="mavros", executable="mavros_node", respawn=True,
  respawn_delay=2.0, ...)` ile açıldı; `fcu_url`/`gcs_url`/`tgt_system=1`/
  `tgt_component=1`/`fcu_protocol="v2.0"` + `apm_pluginlists.yaml` +
  `apm_config.yaml` parametreleri manuel olarak aynen kopyalandı.
- **Test:** `test_mavros_respawn_true` (`generate_launch_description()`
  çıktısında `executable=="mavros_node"` olan bir `Node` bulup
  `respawn=True` olduğunu doğrular).
- **Canlı doğrulama:** Düzeltme sonrası mavros bir kez daha aynı
  bağlantı kesintisini yaşadı, bu sefer öldürülmeden kısa sürede
  reconnect döngüsüne girdi (`link[1000] trying to reconnect...`),
  process ölmedi.

---

## 3. Donanım/Saha Katmanı Bulguları (kod hatası DEĞİL — insan kararı gerekiyor)

### 3.1 TELEM2/FTDI hattı vs. USB-C — çelişkili kanıt

- Test sırasında Pixhawk'ın TELEM2 portundan FTDI adaptörle laptop'a
  bağlantı (`/dev/ttyUSB0`, F-M.9'un önerdiği yöntem) **kullanılamaz**
  durumdaydı: `mavros_router` diagnostics'inde "Remotes count" 174'e,
  sonra 323'e çıktı (sağlıklı bir tekli-araç hattında 1-3 olmalı), RTT
  8+ saniyeye fırladı, `WP:`/`GF: timeout` hataları sürekli tekrarladı,
  bir kere de `mavros_node` gerçekten çöktü (bkz. F-P.20).
- Bu sırada teknenin üzerindeki telemetri radyosu (868 MHz, QGC'nin yer
  istasyonu bağlantısı için) de aynı anda takılıydı. En güçlü hipotez:
  ArduPilot'un dahili MAVLink yönlendiricisi, radyo hattındaki RF
  paraziti/girişimi bizim temiz kablolu portumuza da köprülüyordu
  (QGC de sürekli "Comm Lost" alıyordu, aynı kötü RF ortamının kanıtı).
- **Pixhawk'ın USB-C portuna doğrudan geçilince** (o oturumda
  `/dev/ttyACM1` olarak numaralandı — dikkat: `ttyACM0` DEĞİL, birden
  fazla `ttyACM*` düğümü belirir, en son oluşanı seçilmeli) sorun
  **anında düzeldi**: Remotes count 3'e düştü, RTT ms seviyesine indi,
  `local_position/pose` düzensiz 1.6-3 Hz'den kararlı **10.9 Hz**'e çıktı.
- **Çelişki:** Bu, 2026-07-14 tarihli F-M.9 kararıyla (USB-C soketi
  fiziksel olarak aralıklı kopuyor diye TELEM2/FTDI'ye kalıcı geçiş
  yapılmıştı) ters düşüyor gibi görünüyor. Aslında iki bulgu da doğru
  olabilir — F-M.9'un bulgusu fiziksel konnektör aşınması, bu testin
  bulgusu ise TELEM2/FTDI+radyo kombinasyonunun kendi RF/link
  sorunu. **Takımın karar vermesi gerekiyor:** hangi hat kalıcı olarak
  kullanılacak, yoksa radyonun RF ortamı mı (girişim kaynağı, anten,
  menzil) ayrıca araştırılacak?

### 3.2 RTK kaybı

- Yukarıdaki sorunu çözmek için telemetri radyosu Pixhawk'tan söküldü.
  Bu, Base'in RTCM düzeltmesinin Rover'a ulaşma yolunu da kesti — GPS
  RTK'lı hassasiyetten (covariance ~0.0002, cm seviyesi) standalone
  hassasiyete (~0.37-0.6, ~0.6-0.8 m) düştü.
- Bu oturumda **hem temiz kablolu bağlantı hem RTK'yı aynı anda** elde
  eden bir konfigürasyon bulunamadı. Muhtemel çözüm: radyonun RF
  sorunu kaynağında çözülürse (bkz. 3.1) ikisi birden çalışabilir.

### 3.3 Kamera HSV duba tespiti gerçek ışıkta çalışmadı

- Kameraya gerçek turuncu/sarı renkte dubalar tutuldu (pembe/magenta
  renkli bir duba da denendi — o HAKLI OLARAK tespit edilmedi, çünkü
  HSV filtreleri sadece turuncu/sarı için tasarlı, bu bir hata değil).
- Ama gerçek turuncu/sarı duba da tespit edilmedi. Kameradan canlı kare
  çekilip piksel analizi yapıldı: dubanın ölçülen doygunluğu (S≈29-83,
  0-255 skalası) `prototype/perception/camera_buoys.py`'deki
  `CameraBuoyConfig` varsayılanlarının gerektirdiği **S≥120** eşiğinin
  çok altında (akşamüstü/bulutlu ışık koşulları, muhtemelen eşikler
  daha parlak/güneşli koşullar için ayarlanmıştı).
- Canlı testte eşik geçici olarak S≥30'a düşürülüp `perception_camera_node`
  bağımsız olarak yeniden başlatıldı (not: node HSV parametrelerini
  yalnız `__init__`'te bir kez okuyor, `ros2 param set` çalışan node'u
  ETKİLEMİYOR — değişiklik için node'un yeniden başlatılması şart) —
  **yine de tespit olmadı**. Muhtemel bileşik sebep: düşük doygunluk +
  kareda dubanın küçük görünmesi (uzak mesafe) + `min_area_px=150`
  + morfolojik filtrelemenin az sayıda geçen pikseli daha da eleyip
  eşiğin altına düşürmesi.
- **Hiçbir HSV parametresi kalıcı olarak koda yazılmadı** — bu, rastgele
  tahminle "düzeltilecek" bir şey değil, sahada sistematik mesafe/ışık
  taraması ile kalibre edilmeli. Video/yarışma günü öncesi mutlaka
  tekrar test edilmeli.

### 3.4 GUIDED mod geçişi bazen reddedildi

- Gürültülü link döneminde `/mavros/set_mode` çağrıları `mode_sent=True`
  dönmesine rağmen FC gerçek modda `HOLD`'da kaldı (muhtemelen EKF
  sağlık kontrolü GUIDED'a izin vermiyordu). USB-C'ye geçilip link
  sakinleşince sorun kendiliğinden geçti. Kesin kök sebep izole
  edilmedi — muhtemelen sadece genel link/EKF istikrarsızlığının bir
  belirtisiydi, ayrı bir sorun olmayabilir.

---

## 4. Gerçek Donanımla Doğrulanan Bileşenler (✅ sağlıklı)

| Bileşen | Sonuç |
|---|---|
| MAVROS bağlantısı (USB-C, `ttyACM1`) | ✅ connected, mode=GUIDED elde edildi |
| IMU (`/mavros/imu/data`) | ✅ ~11 Hz stabil, gerçek yerçekimi+açısal hız |
| GPS (`/mavros/global_position/global`) | ✅ gerçek fix (standalone + bir ara RTK) |
| LiDAR (Livox, `/livox/lidar`) | ✅ 10.0 Hz, std dev ~0, gerçek nokta bulutu |
| Engel tespiti (`/perception/obstacle_map`) | ✅ çalışıyor (0 engel — açık alanda doğru) |
| Fusion / EKF pose passthrough | ✅ 10.0 Hz, mükemmel stabilite |
| Kamera görüntüsü (OAK-D, `/oak/rgb/image_raw`) | ✅ ~8-11 Hz, gerçek görüntü doğrulandı (kare kaydedilip görsel incelendi) |
| FSM (`/girdap/parkur/state`) | ✅ `PARKUR_1` doğru varsayılan |
| RC girişi (`/mavros/rc/in`) | ✅ gerçek kanal verisi, rssi=255 |
| RC kill-switch / manuel-override donanımı | ✅ RC kalibrasyonu tamamlandı, kanal eşleşmesi doğrulandı |
| Telemetri CSV — Dosya-2 (`telemetry_node`) | ✅ gerçek dosya, gerçek veri yazıldı |
| Local_map PNG — Dosya-3 (`local_map_node`) | ✅ yüzlerce gerçek kare üretildi |
| Kamera kaydı mp4 — Dosya-1 (`kamera_kayit_node`) | ✅ gerçek dosya (~26 MB) üretildi |
| Mission upload (FC, `WaypointPush`) | ✅ FC gerçek görevi kabul etti (`mission_manager_node` algıladı) |
| mavros_node çökme sonrası dayanıklılık | ✅ F-P.20 düzeltmesiyle doğrulandı (öldürülmedi, reconnect oldu) |
| `with_drivers:=true` sürücü başlatma | ✅ F-S.2 düzeltmesiyle doğrulandı |

---

## 5. Tekrar Test Edilmesi Gerekenler (açık maddeler)

1. **MPPI/`cmd_vel` gerçek çıktısı** — mission FC'ye yüklendi ve kabul
   edildi, ama `/girdap/mission/current_target` hiç yayınlanmadı, bu
   yüzden `planning_node`'un gerçek MPPI thrust çıktısı gözlemlenemedi.
   Kök sebep tam izole edilemedi (muhtemelen mission upload sonrası
   GPS'in birkaç saniyeliğine bayatlamasıyla ilgili, F-P.4 guard'ı
   tetiklenip hedef yayınını durdurmuş olabilir). **Sonraki testte:**
   mission upload'dan sonra GPS/pose'un tekrar stabilize olmasını
   bekleyip current_target'ın gerçekten hesaplanıp hesaplanmadığı
   ayrıca izlenmeli.
2. **Gerçek ARM + GUIDED ile fiziksel hareket testi** — bu oturumda
   araç bir ara armed oldu ama gerçek motor/hareket testi (tekne
   suya girip gerçekten navigasyon yapması) yapılmadı. Pervaneler
   takılıyken bu, güvenli bir alanda ve kill-switch hazır şekilde
   yapılmalı.
3. **Kamera HSV kalibrasyonu** (bkz. 3.3) — sahada sistematik mesafe/ışık
   taraması ile S/V eşiklerinin yeniden ayarlanması gerekiyor.
4. **TELEM2/FTDI vs. USB-C link kararı** (bkz. 3.1) — takımın hangi
   bağlantı yöntemini kalıcı kullanacağına karar vermesi, ya da
   radyonun RF sorununun ayrıca araştırılması gerekiyor.
5. **RTK + temiz link'in birlikte çalıştığı bir konfigürasyon** henüz
   bulunamadı (bkz. 3.2) — 4. maddenin çözümüne bağlı.
6. **Perception fusion** (`perception_fusion_node`, LiDAR+kamera bearing
   eşleştirmesi) gerçek eşleşen bir duba ile hiç test edilmedi (kamera
   tespiti çalışmadığı için fusion'ın girdisi hiç oluşmadı).

---

## 6. Değişen Dosyalar (bu test turunda düzeltilen kod)

- `son_kodv2/karar/ros2_ws/src/girdap_decision/launch/hardware.launch.py`
  (F-S.2 + F-P.20 düzeltmeleri)
- `son_kodv2/karar/prototype/tests/test_hardware_launch_config.py`
  (`test_with_drivers_node_lari_launch_descriptiona_eklenir`,
  `test_mavros_respawn_true` testleri eklendi)

Tam test takımı sonucu (düzeltmeler sonrası): **385 passed, 4 skipped**
(skip'ler yalnız cupy/GPU-only MPPI testleri, beklenen).
