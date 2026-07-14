# CLAUDE.md — Girdap İDA Karar Algoritması

> AI asistanları (Claude, Copilot vb.) için proje bağlamı, kısıtlar, öncelikler.
> Yeni konuşmada önce bu dosyayı oku.
> Kimliğin: Sen sıradan bir kod asistanı değilsin. İnsansız Deniz Araçları (İDA) ve robotik otonomi üzerine uzmanlaşmış, 15 yıllık endüstri tecrübesine sahip Kıdemli (Senior) bir ROS 2 ve C++, Python Yazılım Mimarı'sın.
> Kodlama Felsefen: Asla "şimdilik çalışsın yeter" (spagetti) tarzı kod yazmazsın. Yazdığın her kod modüler, SOLID prensiplerine uyan, bakımı kolay ve obje odaklı (OOP) mimaridedir.
> Performans Takıntın: Geliştirdiğin sistemlerin kısıtlı işlemcilerde (Jetson) çalışacağını bilirsin. Bu yüzden yazdığın C++17 kodlarında Eigen matris işlemlerini optimize eder, pointer'ları doğru yönetir ve memory leak (hafıza sızıntısı) ihtimalini sıfıra indirirsin.
> İletişim Tarzın: Junior (acemi) yazılımcılar gibi gereksiz, uzun ve sıkıcı açıklamalar yapmazsın. Bir mühendis gibi doğrudan mimari kararlarını açıklar, temiz kodu verir ve sadece kritik optimizasyon noktalarını kısa bir yorum satırıyla belirtirsin.
> Kodu bana teslim etmeden önce; mantık hatalarına (logic bugs), null pointer eşleşmelerine, dizi sınır aşımlarına (segmentation fault) ve Eigen/GTSAM kaynaklı olası memory leak (hafıza sızıntısı) durumlarına karşı kendi içinde sessiz bir kod incelemesi (Code Review) yap.
> Eğer terminalden bir derleme (colcon build) veya CMake hatası iletirsem, asla rastgele tahmin yürütme. Hatayı satır satır analiz et, kök nedeni (root cause) mühendise yakışır kısalıkta açıkla ve doğrudan kesin çözümü içeren kod bloğunu ver.

## ⚡ İLK YAPILACAKLAR (Öncelik Sırasıyla)

> En üstü = bu hafta. KTR son teslim: **20.05.2026**.

1. **Ortamı kur (1-2 gün):** Windows tarafında VSCode + Python 3.11 + venv.
   VMware Workstation üzerinde Ubuntu 22.04 LTS + ROS 2 Humble + GTSAM 4.2 +
   Eigen 3.4 + CMake + GCC 11. VMware'ya en az 6 GB RAM, 4 vCPU, 60 GB disk.
2. **Repo iskeleti (1 gün):** Aşağıdaki "Klasör Yapısı" şemasına göre oluştur.
   `git init`, `.gitignore`, `pyproject.toml`, `requirements.txt`, `README.md`,
   bu `CLAUDE.md`. Her modülün `tests/` klasörü olsun.
3. **Python prototip — basit dinamik model (2-3 gün):** İDA'nın 3-DOF
   katamaran kinematik+dinamik modeli (x, y, ψ + sürat u, v, r). Wave/wind
   bozucu yok, ileride eklenir. `pytest` ile birim test.
4. **iSAM2 prototipi (3-4 gün):** GTSAM Python binding ile sentetik GPS+IMU
   verisi üzerinde Pose2 + Between + Prior factor grafiği kur. Smooth çıktıyı
   matplotlib ile görselleştir. Gerçek sensör değil, simülasyon.
5. **RRT* prototipi (2-3 gün):** 2D düzlemde noktasal engellerle çalışan
   sıradan RRT*. Sonra Informed RRT* iyileştirmesi. Görselleştirme şart.
6. **MPPI prototipi (4-5 gün):** İlk önce **CPU**, NumPy ile vektörize.
   Adım 3'teki dinamik model üzerinde 1000 yörünge, 30 step horizon. CUDA'ya
   geçiş Jetson testi öncesinde son adım.
7. **FSM iskeleti (1-2 gün):** Python `transitions` veya kendi enum tabanlı
   makinen. Parkur 1→2→3 geçiş kuralları (1.5 m yakınsama eşiği).
8. **KTR Algoritma Tasarımları bölümünü yaz (3-4 gün):** Şablonun 4. bölümü
   (25 puan, en yüksek). Pseudo-kod + akış diyagramı ile her parkur için
   algoritma akışını anlat. Prototiplerden ekran görüntüsü ekle.

---

## 📋 Proje Bağlamı

- **Yarışma:** TEKNOFEST 2026 İnsansız Deniz Aracı Yarışması
- **Takım:** Girdap (ID: 989124)
- **Senin Rolün:** Karar/Planlama yazılımı — iSAM2 (GTSAM) + RRT* + MPPI
- **Hedef Donanım (sahada):**
  - Görev bilgisayarı: NVIDIA Jetson Orin Nano 8GB Super
  - Uçuş kontrolcüsü: Pixhawk 6C + PM07
  - LiDAR: Livox Mid-360
  - Kamera: Luxonis OAK-D Lite (stereo + Myriad X VPU)
  - GPS: Holybro H-RTK F9P (Rover + Base)
  - Telemetri: MicoAir (frekans teyidi BEKLİYOR)
- **Platform:** Çift gövdeli katamaran, 4× 2838 sualtı thruster
  (diferansiyel tahrik), 4S7P 28× Samsung INR21700-50S batarya

### Kritik Tarihler

| Tarih | Olay |
|---|---|
| 20.05.2026 17:00 | KTR son teslim (max 30 sayfa) |
| 08.06.2026 | KTR sonuçları |
| 21.07.2026 17:00 | Otonomi Kabiliyeti videosu son teslim |
| 27.07.2026 | Finalist takımlar |
| 30 Eylül - 4 Ekim 2026 | TEKNOFEST yarışma günleri |

---

## 🛠️ Geliştirme Ortamı

- **Tüm geliştirme:** Ubuntu 22.04 VM (VMware/VirtualBox) + Python 3.11 +
  ROS 2 Humble + GTSAM. Windows tarafında VSCode Remote-SSH ile bağlan, kod
  VM'de yaşar. Tek ortam = daha az hata.
- **Sahada deploy:** Jetson Orin Nano (Ubuntu 22.04 + JetPack 6 + CUDA 12).
- **Sürüm kilidi (DEĞİŞTİRME):** Ubuntu 22.04 LTS + ROS 2 Humble. Sebep: JetPack 6
  Ubuntu 22.04 tabanlı, MAVROS Humble apt paketi olgun, TYF raporunda Humble
  yazılı (KTR ile tutarlılık şart). Jazzy/24.04 cazip görünür — kullanma.
- **Haberleşme yığını:** MAVLink 2.0 (Pixhawk ↔ Jetson) + ROS 2 (Jetson içi
  mesajlaşma). Köprü: `mavros` (ros-humble-mavros + mavros-extras). Pixhawk
  telemetri portu → USB/UART → mavros → ROS 2 topic'leri (`/mavros/state`,
  `/mavros/global_position/global`, `/mavros/imu/data`, `/mavros/setpoint_*`).
- **GPU notu:** ⚠️ VMware GPU passthrough yok. CUDA/MPPI GPU testi sadece
  Jetson'da. Geliştirme/sınama CPU sürümüyle ilerle, Jetson'da son hız ölçümü.

### Bağımlılıklar

**Python prototip:** `numpy>=1.26`, `scipy>=1.11`, `matplotlib>=3.8`, `gtsam>=4.2`, `pytest>=7.4`, `pyyaml`, `tqdm`

**Ubuntu C++:** `libgtsam-dev` (4.2+, Boost+TBB+Eigen3 gerekli), `libeigen3-dev` (3.4+), `ros-humble-desktop`, `ros-humble-mavros`, `ros-humble-mavros-extras`, `cmake>=3.22`, `gcc-11`. CUDA Toolkit sadece Jetson'da.

**MAVROS GeographicLib veri seti:** kurulum sonrası bir kez çalıştır:
```bash
sudo /opt/ros/humble/lib/mavros/install_geographiclib_datasets.sh
```

---

## 🎯 Geliştirme Stratejisi (3 Katman)

```
Layer 0  Python prototip (Ubuntu VM)    → algoritma matematik doğrulama
   ↓
Layer 1  C++ standalone (Ubuntu VM)     → üretim kalitesi, birim test
   ↓
Layer 2  ROS 2 Humble node (Ubuntu VM)  → mesaj akışı, simülasyon entegrasyonu
   ↓
Sahada   Jetson Orin Nano                → GPU MPPI, gerçek sensör
```

**Kural:** Bir alt katmana atlamadan önce üst katman yeşil olmalı (testler
geçmeli, görselleştirme makul görünmeli). Atlama, hata bulmayı imkansız kılar.

---

## 📐 Algoritma Mimarisi (Pipeline)

```
GPS (1 Hz) ──┐
IMU (~100 Hz)─┼─→ iSAM2 (GTSAM) ──→ Smooth pose+velocity ──┐
LiDAR (10 Hz)─┘                                              │
                                                              ↓
Görev waypoint'leri ──→ RRT* (global) ──→ Referans yörünge ──┐
                                                              ↓
LiDAR engel haritası ──→ Cost map (≥1 Hz) ───────────────────┤
                                                              ↓
                                MPPI (10 Hz CPU; CUDA→50 Hz hedef) ──→ (u_l, u_r)
                                                              ↓
                                            Cascade PID ──→ ESC (4× thruster)

         ┌──────────────────────────────┐
         │  FSM: BOOT → ARM → BEKLEME    │
         │  → P1 → P2 → P3 → TAMAMLANDI  │
         └──────────────────────────────┘
                  (mod yöneticisi)
```

---

## 🧮 iSAM2 / GTSAM (Sensör Füzyonu)

- **Amaç:** GPS gürültüsünden ve dalga sarsıntısından arındırılmış pürüzsüz
  poz+hız çıktısı (Deniz Durumu-2 dayanıklılığı).
- **Kütüphane:** GTSAM 4.2+ (`requirements.txt` fiilen `gtsam>=4.3a0`
  pinler — F2.4; Python binding `gtsam` pip paketi mevcut).
- **Faktörler:**
  - `GPSFactor` veya `PriorFactor<Pose2>` — RTK GPS düzeltmesi
  - `BetweenFactor<Pose2>` — IMU pre-integration adımı
  - `PriorFactor<Pose2>` — başlangıç poze sabitleme
- **Pose2 mu Pose3 mu?** İlk prototipte Pose2 (yüzey aracı, roll/pitch küçük).
  KTR'de "3D" gerekçesi sorulursa Pose3'e geçiş kolay.
- **Inkremental güncelleme:** Sadece etkilenen düğümler hesaplanır
  (`isam2.update(new_factors, new_values)`).
- **Tuning:** GPS gürültü modeli (~2 cm RTK fix), IMU bias rastgele yürüyüşü.
  Gerçek değerleri saha testinden ölçeceksin.

---

## 🌳 RRT* (Global Planlama)

- **Amaç:** Yarışma alanındaki waypoint'ler arasında asimptotik optimal,
  engelsiz yörünge.
- **Uzay:** 2D (x, y) deniz yüzeyi; ψ (heading) MPPI'ye bırak.
- **Steering:** Doğrusal segment yeterli (deniz yüzeyi engelsiz, dubaları
  MPPI hallediyor). Dubins gerek yok.
- **Sample bias:** %10-20 goal-biased.
- **İyileştirme:** Informed RRT* (ilk çözüm bulunduktan sonra elips
  içinde örnekle) → daha hızlı yakınsama.
- **Yeniden planlama tetiği:** Lokal cost map'te yeni engel + global rota
  bu engele <2 m → RRT* yeniden çalıştır.
- **Çıkış formatı:** `List[Tuple[x, y]]` waypoint zinciri → MPPI referansı.
- **Performans:** Tek iş parçacığı CPU yeterli, gerçek zamanlı kısıt yok
  (MPPI gerçek zamanlı koşar).

---

## 🎮 MPPI (Lokal Planlama / Engel Kaçınma)

- **Amaç:** Anlık engel kaçınma + diferansiyel tork çıktısı, dalga
  bozucularına dayanıklı.
- **Hyperparametreler (saha testinde tune edilecek):** K=1000 yörünge,
  T=50 step (horizon 2.5 s @ dt=0.05 s), λ=1.0, Σ_u sürat ve heading
  için ayrı ayar.
- **Maliyet:** `cost = w1·yörünge_sapma + w2·engel_yakınlık + w3·heading_hata
  + w4·kontrol_efor + w5·sınır_dışı_ceza`. Parkur-1: w2 düşük. Parkur-2:
  w2 yüksek, engel haritası girer. Parkur-3: hedef duba **negatif maliyet**
  (çekici), engel maliyetini ezer.
- **Gerçek zamanlı kısıt (GÜNCEL):** bugün CPU ~100 ms/iter →
  `control_rate_hz: 10.0` (params.yaml, F4.2). 50 Hz (20 ms) ancak CUDA
  portuyla; TYF raporu Jetson median 17.6 ms — saha testinde doğrula.
- **CPU vs GPU:** NumPy CPU sürümü ~100 ms, prototip için yeter. CUDA
  (CuPy/raw kernel) Jetson testinde gerekli.
- **Çıktı:** ilk kontrol adımı (u_l, u_r) → Cascade PID iç döngüsüne.

---

## 🔄 FSM (Görev Yöneticisi)

```
[BOOT] ──ros_init──→ [ARM] ──kill_switch_off──→ [BEKLEMEDE]
                                                     │
                                                     ↓ (YKİ "başlat" komutu)
                                              [PARKUR-1: Nokta Takip]
                                                     │
                                                     ↓ (son waypoint <1.5 m)
                                              [PARKUR-2: Engelli Takip]
                                                     │
                                                     ↓ (son duba ikilisi geçildi)
                                              [PARKUR-3: Kamikaze]
                                                     │
                                                     ↓ (IMU ani ivme tespiti)
                                              [TAMAMLANDI: motor stop]
         (F12.2: görev yöneticisi TÜM waypoint'leri bitirince de her
          PARKUR* durumundan doğrudan TAMAMLANDI'ya geçilir — video terminali)
```

- **Geçişler dış komut almaz** (Şartname 5.5.2.2). Tamamen otonom.
- **Acil durum:** her durumdan `KILL` durumuna RC kumanda + YKİ kill butonu.
- **Implementasyon:** Python `enum.Enum` + `dict[State, Callable]` yeterli.
  Aşırı mühendislik yapma.

### Parkur Geçiş Katmanı (Sprint 4 — waypoint-index tabanlı)

> MissionFSM'in (yukarıda) ÜSTÜNE oturan, onu DEĞİŞTİRMEYEN paralel katman.
> MissionFSM = görev yaşam döngüsü + güvenlik; ParkurTransitionLogic = hangi
> parkurdayız (waypoint ilerlemesinden türetilir).

```
mission_manager: waypoint'e varış (ACTIVE→DWELL)
  → /girdap/mission/waypoint_reached (Int32 index)
       ↓
fsm_node: ParkurTransitionLogic.current_waypoint_reached(index)
  PARKUR_1 ─(parkur-1 son wp)→ PARKUR_2 ─(parkur-2 son wp)→ PARKUR_3
  PARKUR_3 ─(/girdap/parkur/impact = IMU çarpma, Sprint 5)→ COMPLETED
       ↓
  → /girdap/parkur/state (String)
```

- **Çekirdek:** `prototype/mission/parkur_fsm.py` (ROS-bağımsız, pytest).
  `ParkurTransitionLogic` waypoint parkur etiketlerinden her parkurun SON
  index'ini hesaplar; o index'e varılınca sonraki parkura **tek yönlü** geçer.
- **⚠ Şartname:** geçiş **waypoint-index + parkur etiketi** ile; duba sayısına
  bağlı akış tasarlamak YASAK. Parkur-2→3 şimdilik waypoint tabanlı;
  gate-passing detection sonraki sprint (mevcut `/perception/gate_passed`
  MissionFSM'de ayrı kanal olarak zaten var).
- **Parkur-3 tamamlanma:** waypoint DEĞİL, IMU çarpma. `confirm_impact()`
  şimdilik `/girdap/parkur/impact` (Bool) placeholder'ından; **Sprint 5** IMU
  şok kanalını buraya bağlayacak.
- **Parkur etiketi VARSAYIMI:** görev dosyasındaki her waypoint'te `parkur`
  alanı (1/2/3), parkur bloğu monoton (contiguous) dizilir. Parser TEK izole
  fonksiyon `parkur_fsm.load_parkur_labels` — gerçek görev formatı gelince
  yalnız orası değişir (çekirdek + node + çıktı sözleşmesi sabit).
- **Görev dosyaları:** `config/competition_mission.yaml` (parkur etiketli) ↔
  `config/video_mission.yaml` (etiketsiz → hepsi parkur 1). `hardware.yaml`
  `mission.mission_file` seçer; `hardware.launch` HEM mission_manager HEM
  fsm_node'a AYNI dosyayı geçer (waypoint index'leri hizalı). Tek-parkurlu
  (video) görevde parkur katmanı PARKUR_1'de kalır, bozulmaz.

---

## 🛰️ MAVLink + ROS 2 Köprüsü (mavros)

- **Akış:** Pixhawk 6C ↔ (USB/UART) ↔ Jetson ↔ `mavros` node ↔ ROS 2.
- **Subscribe edeceğin topic'ler:** `/mavros/state` (mod, armed),
  `/mavros/global_position/global` (NavSatFix), `/mavros/global_position/local`
  (Odometry ENU), `/mavros/imu/data` (Imu),
  `/mavros/local_position/velocity_body` (TwistStamped).
- **Publish edeceğin topic'ler:**
  `/mavros/setpoint_velocity/cmd_vel_unstamped` (Twist) — PID dış döngü çıkışı.
  Acil durum için `/mavros/rc/override`.
- **Mod/arm:** `/mavros/set_mode` → `GUIDED`, `/mavros/cmd/arming`.
- **Frame:** Pixhawk içi NED, mavros çıktıları ENU. iSAM2/MPPI ENU'da kalsın.
- **Şartname 4.1:** Görev başladıktan sonra YKİ→İDA komut yasak. mavros tek
  yönlü telemetri yayını yap, komut akışı kapat.

---

## 👁️ Perception (B Kategorisi)

> Sprint 1 = LiDAR engel tespiti (tamam). Sprint 2 = kamera duba tespiti
> (tamam, mock YOLO). Sprint 3 = kamera-LiDAR bearing füzyonu (tamam).

### LiDAR Pipeline (Sprint 1)

```
/livox/lidar (PointCloud2) → Z-passthrough + menzil filtresi
  → cKDTree Öklid clustering (query_pairs + Union-Find, sklearn YOK)
  → CircleObstacle (centroid + çevrel yarıçap)
  → /perception/obstacle_map (PoseArray)
```

- **Çekirdek:** `prototype/perception/lidar_obstacles.py` (ROS-bağımsız,
  pytest). Node: `perception_lidar_node` — yalnız topic isimlerine bağlı
  (replaceable design: kaynak = gerçek Livox / sentetik / Gazebo).
- **Filtre:** base_link'e göre `z ∈ [z_min=0.1, z_max=3.0]` m (su yüzeyi
  yansıması + yüksek yansıma kesimi), yatay menzil ≤ 25 m.
- **Clustering:** `cluster_tolerance=0.5` m komşuluk, `5 ≤ |cluster| ≤ 500`
  boyut filtresi (altı noise, üstü tekne gövdesi). Parametreler
  `hardware.yaml perception.lidar` bloğu → launch-arg override edilebilir.

### `/perception/obstacle_map` Sözleşmesi (PLACEHOLDER)

`geometry_msgs/PoseArray`, frame `base_link`, kaynak stamp korunur:
- `position.{x,y}` = engel merkezi (cluster centroid)
- `orientation.z` = **çevrel yarıçap (m)** — quaternion DEĞİL, bilinçli hack;
  `orientation.w = 1.0`. planning_node `abs(orientation.z)` okur.
- Custom msg (girdap_msgs) gerekirse sonra; şimdilik downstream'le birebir.

### Sentetik Test Sahneleri (`prototype/perception/synthetic_lidar.py`)

- `scene_minimum(rng)`: 3 duba, noise yok — temel clustering doğrulaması.
- `scene_orta(rng)`: 5 duba + 200 su yüzeyi noise noktası — filtre + eleme
  testi (400 nokta → 5 engel, merkez hatası < 5 cm).
- Duba modeli: silindirik yüzey (r=0.15 m, h=0.5 m, 40 nokta, σ=2 cm).

### Kamera Pipeline (Sprint 2)

```
/oak/rgb/image_raw (Image) → image_codec (cv_bridge YOK) → CLAHE (LAB-L)
  → HSV segmentasyon: turuncu maskesi → class 0, sarı → class 1
  → kontur → bbox (+ opsiyonel YOLO katmanı → class 2 hedef)
  → /perception/buoys (vision_msgs/Detection2DArray)
```

- **Çekirdek:** `prototype/perception/camera_buoys.py`; node:
  `perception_camera_node` (kaynak-bağımsız, yalnız topic adına bağlı).
- **Sınıflar:** `class_id` (string): `"0"`=parkur_kenari (turuncu, RAL 2003),
  `"1"`=engel (sarı, RAL 1026), `"2"`=hedef (Parkur-3, YOLO katmanı).
  (Şartname md 5.5.2.1 — eski "RAL 2008/1003" etiketi YANLIŞTI; F17.1.)
- **Mock YOLO:** gerçek `.pt` yok → `YoloInference` mock modda sabit test
  bbox'ı döner. Gerçek model gelince `perception.camera.yolo_model_path`
  parametresi verilir — kod yolu aynı; ultralytics **lazy import** (mock
  modda hiç yüklenmez). Replace = yalnız `_infer_real` doğrulaması.
- **cv_bridge KULLANMA:** apt cv_bridge boost modülü numpy 1.x ABI'siyle
  derli → pip numpy 2.x'te `_ARRAY_API not found` + KeyError. Yerine
  `girdap_decision/image_codec.py` (bgr8/rgb8 ↔ numpy, ~15 satır).
- **Header:** kaynak `frame_id` + stamp korunur (bbox görüntü pikseli
  uzayında; base_link projeksiyonu Sprint 3 fusion'ın işi).
- **Sentetik sahneler** (`synthetic_camera.py`, 640×480 BGR):
  `scene_camera_minimum` (2 turuncu + 1 sarı, temiz),
  `scene_camera_orta` (3 turuncu + 2 sarı + gürültü + parlama → CLAHE testi).
- HSV aralıkları dizi parametre — yalnız `params.yaml`'da; skaler ayarlar
  (`clahe_clip_limit`, `min_area_px`, `use_yolo`, ...) `hardware.yaml
  perception.camera` → launch-arg (`perception.camera.*`).

### Füzyon Pipeline (Sprint 3)

```
/perception/obstacle_map (PoseArray, LiDAR 3D)  ──┐
                                                    ├─→ ApproximateTimeSynchronizer
/perception/buoys (Detection2DArray, kamera 2D) ──┘        (sync_slop_s)
                                                    ↓
                              bearing-based greedy eşleştirme (kalibrasyon YOK)
                                                    ↓
                    /perception/classified_obstacles (vision_msgs/Detection3DArray)
```

- **Çekirdek:** `prototype/perception/fusion.py`; node: `perception_fusion_node`
  (kaynak-bağımsız, iki topic adına bağlı, `message_filters` ile sync).
- **Association — bearing-based (kalibrasyon YOK):** gerçek intrinsic/extrinsic
  kamera projeksiyonu yok. LiDAR cluster bearing'i (`atan2(y,x)`) ile kamera
  bbox yatay merkezinin HFOV'a orantılı kaba bearing'i (`(bbox_cx-0.5)*hfov`)
  karşılaştırılır; `bearing_tolerance_rad` içindeki en yakın çift greedy
  eşleşir. **Gerçek projeksiyona geçiş:** yalnız `bearing_from_camera`
  fonksiyonu değişir (kamera intrinsic/extrinsic ile tam 3D ışın izdüşümü) —
  `associate()` ve çıktı sözleşmesi (Detection3DArray) SABİT kalır.
- **⚠ İşaret kuralı sahada doğrulanmalı:** `bearing_from_camera`'nın sol/sağ
  yönü kamera montaj/optik-çerçeve varsayımına dayanır; gerçek testte ters
  çıkarsa yalnız o fonksiyondaki işaret çevrilir (bkz. modül docstring'i).
- **Bilinmeyen sınıf (class_id=99):** eşleşmeyen LiDAR tespiti GÜVENLİK
  nedeniyle ATILMAZ — `CLASS_UNKNOWN=99` ile engel olarak korunur (MPPI cost
  map'te hâlâ engel sayılmalı). Eşleşmeyen kamera tespiti ise 3D konumu
  olmadığı için atılır.
- **Zaman senkronizasyonu:** `message_filters.ApproximateTimeSynchronizer`,
  tolerans `sync_slop_s` (~0.1 s). Çıktı stamp'i LiDAR mesajının stamp'i
  (3D konumun kaynağı olduğu için referans alınır).
- **`camera_image_width/height_px`:** `/perception/buoys` piksel-uzayı bbox'ını
  normalize etmek için GEÇİCİ sabit (OAK-D Lite preview 640×480); gerçek
  `CameraInfo` entegrasyonu Sprint 4+.

```
girdap-decision/
├── CLAUDE.md, README.md, pyproject.toml, requirements.txt, .gitignore
├── prototype/              ← Layer 0: Python
│   ├── dynamics/           # 3-DOF katamaran modeli
│   ├── fusion/             # iSAM2 wrapper
│   ├── planning/           # rrt_star.py, mppi.py (NumPy CPU)
│   ├── fsm/, viz/, tests/
├── cpp/                    ← Layer 1: C++ (PLANLANDI — henüz YAZILMADI)
│   ├── CMakeLists.txt, include/girdap/
│   ├── src/                # fusion/, planning/, fsm/
│   └── tests/              # GoogleTest
├── ros2_ws/src/girdap_decision/   ← Layer 2: ROS 2 (sonra)
├── data/                   ← log, kalibrasyon, görev noktaları
└── docs/KTR/, docs/algorithms/
```

---

## 🧪 Test Stratejisi

- **Birim test:** Python `pytest`, C++ `GoogleTest`. Algoritmaların izole
  parçaları (örn. RRT* collision check) ayrı test edilir.
- **Senaryolar:** (1) Tek waypoint engelsiz, (2) 4 waypoint dikdörtgen
  (Otonomi videosu senaryosu), (3) Statik engel + waypoint zinciri,
  (4) Hareketli engel/akıntı, (5) Parkur-3 kamikaze çoklu hedef.
- **Simülasyon:** İlk başta 2D matplotlib simülatörü yeter. Gazebo Layer 2'ye
  gelince devreye girer.
- **Sensör sürücüleri (2026-07-14):** ida_topics paketi ARTIK BU REPODA
  (`ros2_ws/src/ida_topics_yeni`): Livox UDP sürücüsü (SDK'sız, doğrulanmış
  IP 192.168.117.100/port 56301), OAK-D depthai sürücüsü, Dosya-1 kamera
  kayıt node'u, gps_imu MAVROS köprüsü. `with_drivers:=true` ile açılır.
- **CI:** `.github/workflows/ci.yml` (F16.4 kapandı) — GitHub Actions'ta
  ROS'suz çekirdek job: Python 3.10 + `pip install -r requirements.txt` +
  `pytest prototype/tests/`. Node testleri rclpy/mavros yokluğunda gerekçeli
  skip (F16.2 kapılaması). ROS'lu TAM koşu hâlâ elle (README test bölümü).

---

## 🖼️ Görselleştirme (Offline 2D — Sprint 4.5)

> Gazebo/RViz değil; algoritma davranışını top-down gözle görmek için sade
> matplotlib animasyonu. ROS GEREKTİRMEZ — `prototype/` çekirdeklerini doğrudan
> çağırır, deterministik, test-driven.

```
prototype/viz/scenario.py  → senaryo koştur (kinematik + gerçek çekirdekler)
                             → List[FrameState]
prototype/viz/plotter.py   → draw_frame / animate / save_gif (matplotlib tembel)
scripts/run_viz.py         → CLI: --scenario {parkur1,parkur2,fusion} [--save]
```

- **Kullanım:** `python scripts/run_viz.py --scenario fusion` (pencere) veya
  `--save` (GIF → `~/girdap_logs/viz/scenario_<ad>.gif`).
- **Çekirdek kullanımı (yeni algoritma YOK, sadece görselleştirme):**
  `synthetic_lidar` + `lidar_obstacles.detect_obstacles` (gerçek clustering,
  frame'e göre seed'li) → `fusion.associate` (renk sınıfı) →
  `parkur_fsm.ParkurTransitionLogic` (parkur geçişi) → `planning.PlanningPipeline`
  (yerel cost map Dosya-3 + MPPI öngörü yörüngesi).
- **Tekne hareketi basit kinematik** (aktif waypoint'e yönel, cruise hızla
  ilerle) — gerçek MPPI kontrolü değil, görsel akış için yeterli. MPPI yörüngesi
  yalnız OVERLAY (`PlanningPipeline.predicted_trajectory()` = last_trajectories ×
  softmax ağırlık ortalaması); `show_mppi=False` senaryolarda düz-çizgi fallback.
- **Kamera projeksiyonu:** görüntü render'ı bypass (HSV/CLAHE birim testlerde);
  viz FOV geometrik projeksiyonuyla `CameraDetection` üretir → FOV içi dubalar
  renkli, yan/arka (yalnız LiDAR) → `unknown` (gri). Bearing füzyon davranışını
  doğrudan gösterir.
- **RViz (B kategorisi) sonraki adım:** gerçek ROS topic'lerini (obstacle_map,
  buoys, classified_obstacles, cost map) RViz'de canlı göstermek Layer 2 işi;
  bu offline viz saha öncesi algoritma doğrulaması + KTR görseli üretir.

---

## 📝 Kodlama Standartları

- Yorumlar + commit mesajları: **Türkçe** (rapor/sunum tutarlılığı için).
- Identifier'lar: İngilizce snake_case (Python), snake_case değişken +
  PascalCase sınıf (C++).
- Python: type hint zorunlu (`mypy --strict` yeşil). Format: `black` + `ruff`.
- C++17 minimum, ROS 2 Humble (rclcpp). Format: `clang-format` Google stili.
- Sihirli sayı yok — tüm parametreler `ros2_ws/src/girdap_decision/config/*.yaml`'da.
- Logging: `logging` modülü (Python), `RCLCPP_INFO/WARN/ERROR` (ROS 2).
  `print()` sadece tek seferlik debug için, commit'leme.

---

## 🚫 Şartname Yasakları (Madde 4.1)

> İhlal = diskalifiye. Kod yazarken aklında bulundur.

- **Frekans:** 2.4-2.8 GHz **YASAK**, 5.15-5.85 GHz **YASAK**, hücresel
  (4G/LTE) yasak. Tüm bilgisayarlarda dahili WiFi kapalı.
- **Görüntü aktarımı:** YKİ veya yer tarafına analog/dijital görüntü
  aktarımı **yok** (FPV gözlük dahil).
- **YKİ'de işleme yok:** Otonomi/görüntü/sensör yazılımı YKİ'de **olamaz**.
  Tüm yazılım araç üstünde koşar.
- **Görev başladıktan sonra komut yasak** (acil motor kesme hariç).

---

## 📤 Çıktı Formatları (Madde 4.2)

Görev bitiminden 20 dk içinde teslim. Her gecikmiş dosya 5 ceza puanı.
(Not — F17.6: şartname md 4.2 tek "Dosya 1" sayar; "1a/1b" bu repoya özgü
iç adlandırmadır, resmi yazışmada kullanma.)

| Dosya | İçerik | Frekans | Format |
|---|---|---|---|
| 1a | Kamera (bbox + sınıf overlay) | ≥1 Hz | mp4 (zaman etiketli) |
| 1b | Diğer sensör (örn. LiDAR cluster) | ≥1 Hz | mp4 (zaman etiketli) |
| 2 | Telemetri: lat, lon, hız, roll, pitch, heading, hız_setpoint, yön_setpoint | ≥1 Hz | csv (header satırlı) |
| 3 | Lokal harita / cost map / engel haritası | ≥1 Hz | png seri / rosbag / numpy |

**Karar algoritmasının sorumluluğu:** Dosya 2 ve Dosya 3'ün üretimi. Tasarımına
en başından entegre et — sonradan eklemek acı verici.

### Dosya 3 (Yerel Harita) — Uygulama

- **Amaç:** Şartname 4.2 Dosya-3 — lokal harita / cost map / engel haritası,
  ≥1 Hz png seri. Görev bitiminden 20 dk içinde teslim (gecikme = 5 ceza puanı).
- **Üretim zinciri:**
  - `planning_node` → `/girdap/map/local` (`nav_msgs/OccupancyGrid`, **10 Hz**,
    frame `base_link`, 100×100, çözünürlük 0.5 m → araç merkezli 50 m pencere,
    origin (-25, -25), kuzey yukarı). Değer: engel maliyeti 0-100 (engel içi
    100, emniyet halkasında lineer 100→0, dışı 0); arena dışı **-1** (bilinmiyor).
  - `local_map_node` → bu topic'i **1 Hz** dinler, grayscale PNG serisine döker.
- **Grayscale eşleme:** OG 0 → PNG 255 (beyaz=serbest), OG 100 → PNG 0
  (siyah=engel), OG -1 → PNG 128 (gri=bilinmiyor), arası lineer. ROS satır 0
  güney olduğundan PNG kuzey-yukarı için dikey çevrilir.
- **Çıktı yolu:** `~/girdap_logs/local_map/session_YYYYMMDD_HHMMSS/frame_00000.png`
  (5 basamak zero-pad, boot'ta yeni oturum dizini).
- **Kod:** dönüşüm/dosya mantığı `prototype/mapping/local_map.py` (ROS-bağımsız,
  pytest); cost grid `PlanningPipeline.local_cost_grid`. Bağımlılık: Pillow.

---

## 🎬 Video Modu (Otonomi Kabiliyeti videosu)

> **Amaç:** Otonomi Kabiliyeti videosu gerçek suda; senaryo = DİKDÖRTGEN
> oluşturan TAM 4 GPS waypoint — son noktada görev otonom TAMAMLANIR,
> başlangıca dönüş MANUELDİR (md 3.3.1(3); 5. "dönüş" noktası EKLENMEZ —
> F-V.4). Karmaşık füzyon/planlama katmanları **bypass** edilip en sade
> güvenilir zincir kullanılır. Yarışma günü tam stack açılır.

**Bypass gerekçesi:** iSAM2 tuning ve RRT* saha kalibrasyonu zaman ister; video
için MAVROS'un kendi EKF'i (poz) + düz waypoint referansı + MPPI (engel kaçınma
+ diferansiyel tork) + cmd_vel yeterli ve daha az kırılgan. Dosya-2/Dosya-3
deliverable'ları her iki modda da üretilir.

**Config flag'leri — `config/hardware.yaml` `algorithm` bloğu:**
```yaml
algorithm:
  use_isam2: false   # video: /mavros/local_position/pose pass-through
  use_rrt:   false   # video: current_target doğrudan MPPI referansı
  use_mppi:  true    # her iki modda lokal kontrolcü
```
- `use_isam2=false` → `fusion_node` GTSAM'ı hiç yüklemez, EKF pozunu iletir.
- `use_rrt=false` → `planning_node` global planı atlar, `mission_manager`'ın
  `/girdap/mission/current_target`'ını düz çizgi MPPI referansı yapar.

**Video sonrası → yarışma modu:** `hardware.yaml`'da `use_isam2: true`,
`use_rrt: true` yap. Kod yolu aynı; sadece flag değişir (params.yaml
varsayılanları zaten tam stack). Bypass çekirdekleri (`PosePassthrough`,
`set_reference_direct`) yerinde kalır, dokunulmaz.

**Görev tanımı:** `config/video_mission.yaml` — `mission_manager_node` boot'ta
okur, GPS ile haversine/ENU hedef üretip 5 Hz `current_target` yayınlar.
Durum makinesi: IDLE→ACTIVE→DWELL→ACTIVE→…→COMPLETE (arrival_radius + dwell).

**Koordinat doldurma prosedürü (göl kenarı, video günü):**
1. Aracı P1'e koy; telefon GPS veya H-RTK ile lat/lon oku (≥7 ondalık).
2. Dikdörtgeni saat yönünde P1→P2→P3→P4 gez, her köşeyi ölç ve yaz.
3. Görev TAM 4 nokta — `P1_return` EKLEME (dönüş manuel, md 3.3.1(3); F-V.4).
4. `home_ref` runtime'da ilk arm'da set edilir (0.0 kalabilir — home bağımlılığı
   yok, hedefler mevcut poza göre relatiftir).
5. `arrival_radius_m`, `dwell_time_s`, `cruise_velocity_mps` sahaya göre ayarla.

---

## ⚠️ Tuzaklar / Dikkat Edilecekler

- **GTSAM Windows'ta sancılı.** Python binding kolay (`pip install gtsam`)
  ama C++ source build saatler alır. Bu yüzden C++ tarafı Ubuntu VM'de.
- **VMware'da CUDA yok.** MPPI GPU sürümünü VM'de test edemezsin. CPU
  sürümünü olgunlaştır, Jetson'da CUDA portu son adım.
- **Pixhawk NED, ROS 2/mavros ENU.** mavros çevirir ama iç hesabında
  tutarlı kal — bir kere karar ver, dökümante et.
- **Heading sürekliliği:** ψ ∈ [-π, π] sıçraması MPPI maliyetini bozar.
  `atan2(sin(Δψ), cos(Δψ))` ile farkı al.
- **iSAM2 graf büyümesi:** uzun görevde graf büyür, RAM şişer. Marginal-out
  veya sliding window düşün (yarışma 20 dk, ~kabul edilebilir).
- **MPPI ilk iterasyon kararsız.** Warm-start yoksa rastgele kontrol → araç
  zikzak. Önceki kontrol dizisini kaydır + yeni rastgele step ekle.
- **Yeniden başlama hakkı 1 kere** (puan sıfırlanır). Algoritma "soft restart"
  desteklesin (state reset + iSAM2 reinit).

---

## 📚 Anahtar Referanslar

- Williams (2017) *MPPI*, IEEE CSM 37(2). Kaess (2012) *iSAM2*, IJRR 31(2).
  Karaman & Frazzoli (2011) *RRT\* optimal motion planning*, IJRR 30(7).
- Fossen (2011) *Marine Craft Hydrodynamics* — katamaran dinamik temel.
- GTSAM tutorials: <https://gtsam.org/tutorials/>
- Nav2 MPPI Controller (referans implementasyon):
  <https://docs.nav2.org/configuration/packages/configuring-mppic.html>

---

## 🆘 Claude/AI Asistanla Çalışma Kuralları

- **Önce küçük çalışan örnek iste.** "MPPI yaz" yerine "MPPI rollout
  fonksiyonunu 30 satırda yaz, sentetik veriyle test et".
- **Hata aldığında tam stack trace + minimal repro paylaş.** Tahmin ettirme.
- **Kütüphane sürümünü söyle** — "GTSAM" değil "GTSAM 4.2 Python binding".
- **KTR'ye yarayan görselleştirme iste** — her algoritma için matplotlib
  ekran görüntüsü → rapora doğrudan girer.
- **Şartname referansı ile sınırla** — "Bu özelliği eklememe gerek var mı?"
  sorusunu önce şartnameye sor, sonra Claude'a.

---

*Son güncelleme: 26.04.2026 — Her sprint sonunda gözden geçir.*
