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
  - Telemetri: RFD868x (868 MHz)
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

### Sağlamlaştırma (2026-08-01) — outlier reddi + keyframe throttle

> Üç kapı eklendi: **robust GPS**, **fix-kalitesi sigma'sı**, **key throttle**.
> Hepsi geri uyumlu — eski davranış tek bayrakla geri gelir.

**1. Robust GPS (Huber M-estimator)** — `isam2_smoother.py`

- Tek kötü fix (multipath, RTK kaybı) saf Gauss modelinde kare-hatayla
  cezalandırıldığı için TÜM çözüm penceresini kendine çeker.
- `gps_robust_enabled: true` (varsayılan) → GPS prior'unun gürültü modeli
  `noiseModel.Robust.Create(mEstimator.Huber.Create(gps_huber_k), base)`
  ile sarılır. `gps_huber_k = 1.345` (σ katı) literatür standardı: Gauss
  gürültüde en küçük kareler verimliliğinin %95'i korunur.
- **Ölçülen etki** (40 key düz çizgi, 20. keye **10 m** outlier):
  | | maks. sapma | RMS |
  |---|---|---|
  | robust KAPALI | 4.541 m | 0.823 m |
  | robust AÇIK | 0.113 m | 0.020 m |
  → **40× iyileşme**. Boru hattı seviyesinde (lat/lon→ENU, 30 s senaryo):
  6.244 m → 0.224 m. Outlier YOKKEN iki mod arasında fark ~0 (Huber temiz
  veride bedelsiz).
- `gps_robust_enabled: false` → eski saf `Diagonal.Sigmas` modeli birebir.
- ⚠ Kernel YALNIZ GPS'e uygulanır; odometri (IMU) outlier üretmez.

**2. Fix kalitesine göre sigma** — `gps_quality.py` + `fusion_node`

- `NavSatFix.status.status` okunur (eskiden yalnız `< 0` kontrolü vardı,
  fix tipi tamamen yok sayılıyordu → RTK ile tek-nokta çözüm aynı ağırlıkta):

  | status | anlam | σ (m) |
  |---|---|---|
  | `STATUS_GBAS_FIX` (2) | RTK fixed | 0.05 |
  | `STATUS_SBAS_FIX` (1) | SBAS düzeltmeli | 0.50 |
  | `STATUS_FIX` (0) | tek nokta | 2.50 |
  | `STATUS_NO_FIX` (-1) | fix yok | **add_gps ÇAĞRILMAZ** + WARN |

- Eşikler `hardware.yaml` → `fusion.gps_sigma_by_status`; launch-arg
  override: `fusion.gps_sigma_by_status` düzleştirilip
  `fusion.gps_sigma_gbas_fix:=0.02` gibi verilir.
- Çekirdek (`prototype/fusion/gps_quality.py`) **ROS-bağımsız** — NavSatStatus
  sabitleri düz int; rclpy'siz pytest'te koşar.
- Bilinmeyen pozitif status → tablodaki EN KÖTÜMSER σ (sessizce RTK sanma).
- ⚠ `mock_sensors` artık fix durumunu **enjekte ettiği gürültüden** türetir
  (`_status_for_sigma`); sabit "status=2 (RTK)" derken 0.30 m gürültü basmak
  smoother'a σ=0.05 dedirtirdi (36× fazla güven).

**3. Keyframe throttle** — `pipeline.py`

- **Teşhis:** key kadansı `odom_period_s`'e bağlıydı → 20 dk'lık yarışma
  görevinde **11 416 key**. Graf ve her `calculateEstimate()` bununla
  doğru orantılı büyür.
- `keyframe_rate_hz: 5.0` (varsayılan) key üretimine tavan koyar; ara IMU
  adımları **Pose2 kompozisyonuyla** biriktirilip **tek `BetweenFactor`**
  olur. `<= 0` → throttle kapalı (eski kadans).
- **Ölçülen** (20 dk, IMU 50 Hz + GPS 1 Hz): 11 416 key / 17.8 s füzyon CPU
  → **6 000 key / 4.0 s** (4.5× ucuz). Aynı senaryoda iki ayarın ürettiği
  poz farkı **< 1 cm** — bilgi kaybı yok.
- Biriktirme eski koddan **daha doğru**: eski `_flush` yalnız SON hız
  örneğini periyotla çarpıyordu (ZOH); artık her IMU alt adımı entegre edilir.
- Odometri σ'sı `√(Δt/odom_period_s)` ile ölçeklenir (`add_odometry(
  sigma_scale=...)`) — yoksa daha az/uzun faktör zinciri aynı sürede sahte
  güven kazanıp GPS'i bastırırdı. Throttle kapalıyken katsayı tam 1.0.

**Açık bulgu — `calculateEstimate()` sıcak yolda O(N)**

`_flush()` her key'de argümansız `self._isam.calculateEstimate()` çağırıyor;
bu TÜM değişkenler için Values üretir. Ölçüm: N=500 → 0.015 ms, N=4500 →
0.121 ms (doğrusal). Görev boyunca toplam maliyet O(N²). Throttle bunu 4.5×
küçülttü ama **kök neden duruyor** — `current_pose()` yalnız son key'i
istiyor. Düzeltme adayı: tek-key `calculateEstimate(key)` + `all_poses()`
için ayrı tam sorgu. `all_poses()`/`all_xy_psi()` sıcak yolda DEĞİL (yalnız
offline demo + testler).

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
  + w4·kontrol_efor + w5·sınır_dışı_ceza + w6·terminal`. Parkur-1: w2 düşük.
  Parkur-2: w2 yüksek, engel haritası girer. Parkur-3: hedef duba **negatif
  maliyet** (çekici), engel maliyetini ezer.
  ⚠ Maliyette **ayrı hız terimi YOK** — seyir hızını fiilen terminal terimin
  gradyanı (2·w_terminal·d) belirler. `terminal_mode`/`w_terminal`/`lambda_`
  değiştirirken bunu hatırla (F-M.3 ve λ bölümleri).
- **Gerçek zamanlı kısıt (GÜNCEL):** bugün CPU ~100 ms/iter →
  `control_rate_hz: 10.0` (params.yaml, F4.2). 50 Hz (20 ms) ancak CUDA
  portuyla; TYF raporu Jetson median 17.6 ms — saha testinde doğrula.
- **CPU vs GPU:** NumPy CPU sürümü ~100 ms, prototip için yeter. CUDA
  (CuPy/raw kernel) Jetson testinde gerekli.
- **Çıktı:** ilk kontrol adımı (u_l, u_r) → Cascade PID iç döngüsüne.

### Kayan Referans Penceresi (F-M.2, 2026-08-02) — maliyet tensörü küçültme

> Takip maliyetinin en yakın-nokta araması TÜM referansta değil, çapa
> (anchor) etrafındaki dilimde yapılır. Geri uyumlu: tek bayrakla eski
> tam tarama geri gelir.

**Teşhis:** `_trajectory_cost` her adımda `d2 = (K, T+1, n_ref)` tam tensörünü
kuruyordu. Yarışma ölçeğinde (K=1000, T+1=51, n_ref=2048 — F-M.1 tavanı, 1 km
rota @0.5 m) **102 M eleman**: float32 389 MiB / float64 779 MiB, üstelik `dx`,
`dy` ve kare geçicileri aynı anda canlı → tepe kullanım ~3-4×. Orin Nano'nun
**8 GB PAYLAŞILAN** (CPU+GPU) belleğinde hem kapasite hem hız darboğazı.
(CLAUDE.md'deki "~100 ms/iter" ölçümü n_ref≈114'lük demo sahnesine aitti;
gerçek rota uzunluğunda maliyet doğrusal büyüyordu.)

**Çözüm:** Tekne referans üzerinde **monoton ilerler** → önceki adımın en yakın
indeksi (`_ref_anchor_idx`) etrafındaki dilim yeter:
`lo = max(0, çapa − size/4)`, `hi = min(n_ref, çapa + size)` — asimetrik
(geriye az, ileriye çok). `MPPIConfig`:

| parametre | varsayılan | anlamı |
|---|---|---|
| `ref_window_size` | `100` | ileri pencere derinliği (**nokta**; 0.5 m aralıkta 50 m ileri + 12.5 m geri) |
| `ref_window_enabled` | `True` | `False` → eski tam tarama (regresyon/A-B ölçümü) |

**Ölçülen** (K=1000, T=50, n_ref=2001, numpy float64, Ryzen CPU, rota ortasından):

| | step süresi | d2 elemanı | float32 | float64 |
|---|---|---|---|---|
| tam tarama | 736 ms | 102 051 000 | 389 MiB | 779 MiB |
| pencereli | **58 ms** | **6 375 000** | 24 MiB | 49 MiB |
| kazanç | **12.6×** | **16.0×** | −365 MiB | −730 MiB |

Boru hattı seviyesinde (`PlanningPipeline`, RRT* referansı n_ref=632, 400 adım,
10 Hz engel tazeleme): **258.6 ms → 61.1 ms (4.2×)**, p95 279 → 68 ms, 0
fallback, son konum birebir aynı. (Kazanç oranı n_ref ile büyür: pencere sabit
125 nokta tarar, tam tarama rota uzunluğuyla doğrusal.)

Kapalı döngüde (dönüşlü rota, engelli, 745 adım) iki mod **bit-birebir**:
maks |Δu₀| = 0.0 N, goal hatası 1.443 m (ikisinde de), iz sapması maks 5.846 m
/ RMS 1.769 m (ikisinde de). Beklenen: gerçek argmin pencere içinde kaldığı
sürece maliyet değişmez; dışına savrulan rollout'un takip maliyeti yalnız
BÜYÜR (min kısıtlı küme üzerinde) — zaten cezalandırılması gereken rollout.

**Kenar güvenliği:** en yakın nokta pencerenin **yapay** kenarına yapışırsa
(referansın gerçek uçları sayılmaz — yoksa rota başında/hedefte her adım
tetiklenirdi) o adım **tam taramaya düşer**, çapa yeniden bulunur, WARN
loglanır (`_ref_window_fallbacks` sayacı; log 1. ve her 50. olayda — 20 Hz'te
sel olmasın). Ölçülen bedel: fallback adımı **791 ms** (≈ tam tarama), sonraki
adımlar 58 ms'e döner. Ne zaman tetiklenir:
- ~~Parkur geçişinde yeni kontrolcü kurulması~~ → **kapatıldı** (aşağıya bak):
  `carry_state_from` çapayı da taşıyor, geçişte fallback yok.
- Tekne >12.5 m geri giderse, poz sıçrarsa (GPS/iSAM2 reinit).
- Gerçek yeniden planlamada tetiklenmez: RRT* ve video bypass'ı referansı
  **teknenin mevcut pozundan** başlatır → çapa 0 zaten doğru.

**⚠ Çapa sıfırlama kuralı:** `set_reference` çapayı yalnız referans GERÇEKTEN
değiştiyse sıfırlar. Boru hattı (`PlanningPipeline._rebuild_mppi`) her engel
tazelemesinde AYNI yolla `set_reference` çağırır (5-10 Hz); koşulsuz sıfırlama
pencereyi sürekli rotanın başına atar (U_nominal'in F11.1 dersinin referans
karşılığı). **Ölçülen fark** — yukarıdaki 400 adımlık boru hattı koşusunda
koşulsuz sıfırlama: ort **124.7 ms** (kazanç 4.2× → 2.1×), **112/400 adımda**
fallback, **p95 300.8 ms** — yani en kötü %5 tam taramadan bile KÖTÜ (pencere
denemesi + tam tarama üst üste). Davranış her üç kolda birebir aynı; fark
tamamen hız/gecikme.

**⚠ Terminal maliyet pencereden BAĞIMSIZ** — hedef ASLA dilimin ucu değil
(bkz. F-M.3); dilimin ucu hedef sanılırsa araç pencere sınırında takılır.

### Parkur Geçişinde Sıcak Durum Taşıma (2026-08-02)

`MPPIController.carry_state_from(other)` — ağırlık profili değişince
(`_rebuild_mppi`) YENİ kontrolcü kurulur; eskisinin **U_nominal**'i (warm-start,
F11.1) **ve kayan pencere çapası** (F-M.2) devredilir. Çapa yalnız referans
birebir aynıysa taşınır (`set_reference`'tan SONRA çağrılmalı).

**Ölçülen** (boru hattı, K=1000, n_ref=632, PARKUR1→PARKUR2 geçişi):

| | geçiş adımı | fallback | maks \|ΔU_nominal\| |
|---|---|---|---|
| taşıma yok | 304.5 ms | 1 | 30.0 N (doygunluktan sıfıra sıçrama) |
| taşıma var | **62.2 ms** (= normal adım) | 0 | **0.0 N** |

---

## 🎯 MPPI Terminal Maliyeti (F-M.3, 2026-08-02)

> `terminal_mode` = `"global"` (VARSAYILAN, eski davranış) | `"lookahead"`.
> Ölçüm bulguları aşağıda — **mod değiştirirken `w_terminal` de değişmeli.**

**Ölçülen maliyet bileşeni tablosu** (300 m rota, 150 m'de rotanın üstünde
2 m yarıçaplı engel, tekne 8 m geride 4 m/s, K=1000, PARKUR2 profili). Softmax
`S_min` çıkardığı için ORTALAMA değil **STANDART SAPMA** ayırt edicidir:

| bileşen | ortalama | std | std payı |
|---|---|---|---|
| `w_obstacle` | 1 623 | **868** | **79.1%** |
| `w_terminal` | **113 763** | 192 | 17.5% |
| `w_track` | 57 | 29 | 2.7% |
| `w_control` | 53 | 6 | 0.6% |
| `w_heading` | 2 | 1.5 | 0.1% |
| `w_boundary` | 0 | 0 | 0% |

- **Terminal ortalamada 70× baskın, ayırt edicilikte DEĞİL.** "Uzun referans
  terminal farkı engel maliyetini eziyor" hipotezi ölçümde doğrulanmadı: engel
  terimi silindiğinde u₀ **25.3 N** değişiyor (PARKUR1 profilinde 5.8 N) —
  engel softmax'ta net görünüyor.
- **Asıl bulgu — softmax dejenere: ESS = 1.0 / 1000.** λ=1.0 maliyet ölçeğine
  göre çok küçük; tek rollout w=1.0 alıyor, MPPI fiilen "en iyi rastgele
  örneği seç"e düşüyor (ortalama alma özelliği kayıp). λ=1000'de ESS 740/1000
  ama bu sefer engelin u₀'a etkisi 0.34 N'a düşüyor — λ ayrı bir tune ekseni,
  saha testinde ölçülmeli.
- **Sayısal (Jetson float32):** `w·d²` uzun rotada dev sabit ofset üretir.
  858 m hedefte ULP 0.25, 3.9 km'de **ULP 8.0** iken `w_track` std'si 8.6 —
  yani uzun rotada ince terimler kuantalanıyor. Lookahead terminali 113 763 →
  321 (w=5) / 3 141 (w=50) ölçeğine indirir.

**⚠ Lookahead'e geçerken w_terminal telafisi ZORUNLU.** Terminal `w·d²`'nin
gradyanı `2·w·d` — hedef uzaklığıyla orantılı. Maliyette **ayrı hız terimi
yok**; uzak hedefin gradyanı fiilen seyir hızı kontrolcüsü görevi görüyor.
Ölçüm (200 m rota, engelli, PARKUR2):

| mod | w_terminal | seyir hızı | goal hatası | min açıklık | iz sapma RMS |
|---|---|---|---|---|---|
| global | 5 | 5.64 m/s | 1.42 m | +0.08 m | 0.67 m |
| lookahead | 5 | **1.07 m/s** | **102.6 m (varamadı)** | +0.53 m | 0.11 m |
| lookahead | 50 | 5.31 m/s | 1.49 m | +0.21 m | 0.56 m |
| lookahead | 100 | 5.52 m/s | 1.43 m | +0.16 m | 0.57 m |

Telafisiz lookahead'in "daha iyi açıklığı" (+0.53 m) kazanç değil **yavaşlığın
yan etkisi**. Eşleşen hızda açıklık farkı +0.08 → +0.21 m ile sınırlı.

**Lookahead'in gerçek kazancı — DÖNÜŞLÜ rotada köşe kesmeme** (eşleşen hız):

| mod (w_terminal) | iz sapma RMS | iz sapma maks | min açıklık | adım süresi |
|---|---|---|---|---|
| global (5) | 1.66 m | 4.61 m | +0.22 m | 53.0 ms |
| lookahead (50) | **0.52 m** | **1.89 m** | +0.29 m | 56.4 ms |

**`terminal_lookahead_m` seçimi:** ≥ seyir_hızı × horizon (T·dt) olmalı.
Ölçüm (w=50): 10 m → hız 3.85 m/s'de sınırlanıyor (fren); 15 m → 5.31 m/s;
25 m → 5.89 m/s; 40 m → açıklık +0.19 → −0.12 m'ye bozuluyor (global'e
yaklaşıyor). **15-25 m** aralığı makul.

**✅ BENİMSENDİ (2026-08-02):** `terminal_mode="lookahead"` varsayılan +
`_PARKUR_PROFILES` üçünde de `w_terminal=50.0`. **İKİSİ AYRILMAZ** — biri
diğeri olmadan değiştirilirse araç ya sürünür (w=5) ya köşe keser (global,
w=50 gereksiz). `test_terminal_mode_varsayilan_lookahead` ikisini birden
donduruyor. ⚠ Göl testinde doğrulanacak.

**Boru hattı doğrulaması** (300 adım, K=1000, RRT* referansı + yol kenarında
engel; eski↔yeni varsayılanlar):

| varsayılan seti | iz sapma RMS | maks | seyir hızı | adım süresi |
|---|---|---|---|---|
| eski (global, w=5, margin 0.5) | 0.98 m | 1.68 m | 4.98 m/s | 65.1 ms |
| **yeni** (lookahead, w=50, margin 1.0) | **0.20 m** | **0.50 m** | 4.65 m/s | 65.3 ms |

İz takibi **4.9× daha iyi**, bedel %7 hız. Kayan pencere değişmezliği korunuyor
(pencere açık/kapalı son konum birebir aynı).

---

## 🛟 Emniyet Payları — `obstacle_margin` ↔ `safety_margin` (F9.2 kapandı)

> **İkisi eşit DEĞİL, sıralı:** RRT* `safety_margin` **0.5 m** (HARD kısıt,
> bu payın içinden yol geçmez) ≤ MPPI `obstacle_margin` **1.0 m** (SOFT
> quadratic barrier). Üst sınırı Parkur-2 geçidi belirler.

**Açık alanda margin taraması** (tekne yarı genişliği **0.375 m** — gövde
0.75 m, `kod_denetimi` F9.2; açıklık = engel YÜZEYİNE mesafe):

| `obstacle_margin` | min açıklık | gövde payı | iz sapma RMS | goal hatası |
|---|---|---|---|---|
| 0.5 (eski) | +0.08 m | **−0.29 m → ÇARPMA** | 0.67 m | 1.42 m |
| **1.0 (benimsendi)** | +0.54 m | **+0.17 m** | 0.76 m | 1.47 m |
| 1.5 | +0.95 m | +0.58 m | 0.87 m | 1.46 m |

Yaklaşık kural: **açıklık ≈ margin − 0.45 m**. Hız ve varış performansı
değişmiyor, bedel iz sapmasında ~0.1 m.

**⚠ ÜST SINIR — Parkur-2 geçidi (net açıklık ~1.35 m, duba r=0.15 m → merkez
hattından duba YÜZEYİNE 0.675 m).** Geçitten geçiş ölçümü:

| MPPI `obstacle_margin` | geçitten geçti mi | geçitte yanal sapma | duba yüzeyine açıklık |
|---|---|---|---|
| 0.3 / 0.5 / 0.675 / **1.0** | ✅ evet | 0.02-0.04 m (tam orta) | +0.64…+0.67 m |
| 1.5 | ❌ **HAYIR** — geçit önünde durdu (x=38, geçit x=40) | — | — |

1.5 m'de ceza bölgesi geçidin tamamını kaplıyor; MPPI geçitten geçmeyi
etraftan dolaşmaktan pahalı buluyor → **görev ihlali**. 1.0 m güvenli tarafta.

| RRT* `safety_margin` | 10 seed'de plan | ort yol | yorum |
|---|---|---|---|
| 0.3 (eski) / **0.5** | 10/10 | **60.00 m** | geçidin TAM içinden (düz) |
| 0.675 | 10/10 | 60.08 m | sapmaya başlıyor |
| 1.0 | 10/10 | 60.14 m | ⚠ "başarılı" ama geçidin **ETRAFINDAN** — sessiz görev ihlali |

RRT* asla hata vermez, sadece geçidi es geçer — bu yüzden payı MPPI'ninkiyle
eşitlemek YANLIŞ olurdu. **0.3 → 0.5** yükseltildi (0.3 m tekne yarı
genişliğinin altındaydı; global yol gövdenin sığmayacağı kadar yakın
planlanabiliyordu). F10.1 ret yarıçapı 0.45 → 0.65 m; geçit merkezinde tekne
duba merkezine 0.825 m'de kaldığı için hâlâ güvenli.

Geçidin fiziği: merkez hattında gövde payı 0.675 − 0.375 = **0.30 m/yan** —
bunu hiçbir parametre büyütemez, geçit dar. Deniz Durumu-2 sürüklenmesi burada
gerçek risk; saha testinde ölç.

---

## 🚪 Kapı Takibi (gate following) — Layer 2 BAĞLANTISI (2026-08-03)

> Çekirdek `prototype/mission/gate_follower.py` 27.07'de yazılmıştı (20 test)
> ama **hiçbir node'a bağlı değildi** — ham GN doğrudan MPPI'ye gidiyordu.
> Bu bölüm o bağlantıdır.

**Neden gerekli (md 5.5.2.2):** Parkur-1/2 puanı GPS noktasına basmaktan DEĞİL,
karşılıklı KENAR dubası ikilisinin **arasından geçmekten** gelir; üstelik
hakemin verdiği nokta *"doğrudan iki kenar dubasının arasında bir nokta
olmayabilir"* ve dubalar önceden haritalanamaz. Ham GN'ye yönelmek puan
kaybettirir.

**Bağlantı yeri: `planning_node`** (mission_manager değil — gövde→dünya dönüşümü
için ψ gerekir, o da yalnız burada var):

```
/perception/classified_obstacles (Detection3DArray, base_link)
        │  class_id == edge_buoy_class_id (0, turuncu KENAR)
        ├────────────────────────────► _edge_buoys (dünya ENU)
        │                                      │
        │  diğer sınıflar (sarı=1, UNKNOWN=99…)│
        └──► CircleObstacle → MPPI             │
                                               ▼
current_target / waypoints (ham GN) ──► _refine_target() ──► GateFollower
                                               │
                          kapı varsa: ORTA NOKTA │ kapı yoksa: ham GN (fallback)
                                               ▼
                                    set_reference_direct / set_waypoints
```

- **Turuncu duba ENGEL DEĞİLDİR.** Engel torbasında bırakılırsa
  `obstacle_margin`=1.0 m'lik ceza halkası geçidin (net açıklık ~1.35 m) içini
  kaplar → MPPI kapıdan geçmeyi etraftan dolanmaktan pahalı bulur (yukarıdaki
  "Emniyet Payları" ölçümü: 1.5 m'de geçitten **hiç** geçmiyor). Bu yüzden
  ayıklama ZORUNLU, tercih değil.
- **Sınıflı topic aktığında `obstacle_map` susar** (`use_classified_obstacles`)
  — aksi halde kapı dubaları sınıfsız yoldan engel olarak geri sızardı.
- **CLASS_UNKNOWN=99 engel KALIR** (füzyon sözleşmesinin güvenlik kuralı).
- **Geriye tam uyumlu:** kapı görünmüyorken çekirdek ham GN'ye düşer → kapısız
  senaryoda davranış birebir aynı. `gate_following_enabled=false` tamamen kapatır.
- **Parkur geçişinde `reset()`** — Parkur-1'in son kapısına kilitliyken
  Parkur-2'ye geçilirse eski hedef taşınmaz.
- Saha teşhisi: `/girdap/planning/gate` (PoseStamped, kilitli kapı ortası;
  kontrol yolu DEĞİL, RViz'de "kapı görüyor muyuz" göstergesi).
- Saha yüzeyi: `planning.gate_*` launch-arg'ları (params.yaml ↔ hardware.yaml ↔
  `_GATE_DEFAULTS`, drift'i `test_planning_config_drift.py` bağlar).

**🔴 KAPI GENİŞLİĞİ ÖNCEDEN BİLİNEMEZ — bant bir KALİBRASYON değil SÜZGEÇ.**
Şartname üç yerde söylüyor: *"kenar dubaları arasındaki mesafeler yarışma alanına
göre değişkenlik gösterecektir"* · *"Dubalar arasındaki mesafeler … yarışma alanına
göre belirlenecektir"* · *"kenar dubaları ve engeller de deniz şartlarından dolayı
yer değiştirebilir"* (→ koşu **sırasında** bile sabit değil). Parkur önceden
görülemez, önceden haritalama zaten yasak.
- Bu yüzden bant bilerek geniş: **1.0 – 20.0 m**. Dar bant, gerçek kapıyı
  **sessizce reddeder** — en kötü arıza biçimi (özellik açık görünür, hiçbir şey
  yapmaz, puan kaybedilir). Ayırt etmeyi geometri yapar: yalnız turuncu adaydır,
  ikisi de önde/menzilde, "yan yana" (`pair_depth_tol`), orta nokta kurs
  çizgisine en yakın.
- **Sessiz ret kapanı:** turuncu duba görülüyor ama hiçbir çift banda girmiyorsa
  `planning_node` 5 s'de bir **ölçülen gerçek mesafeleri** WARN'lar →
  `planning.gate_width_max:=14` ile sahada anında düzeltilir.
- Şartnamedeki tek kesin sayı: **duba çapı 30 cm, yükseklik 50 cm**; Şekil 3
  açıkça *"temsili"*.

**⚠ `obstacle_margin` üst sınırının dayanağı ZAYIF.** Yukarıdaki "Emniyet Payları"
tablosundaki *"Parkur-2 geçidi net açıklık ~1.35 m"* sayısı `kod_denetimi.md`
F10.1'de **kaynaksız** geçiyor — şartnamede karşılığı YOK ve şartname mesafelerin
alana göre değişeceğini söylüyor. Ölçümler (1.0 geçer, 1.5 geçmez) o varsayılan
geçit için geçerli; gerçek geçit daha genişse 1.0 fazlasıyla güvenli, **daha
darsa 1.0 da geçidi kapatabilir**. Sahada ilk kapı görüldüğünde
`/girdap/planning/gate` ile teyit et.

### 🔴 Aynı turda bulunan CANLI HATA — engel frame'i dönüştürülmüyordu

`/perception/obstacle_map` `perception_lidar_node`'da açıkça
`frame_id="base_link"` ile yayınlanıyor (**gövde** çerçevesi, x=ileri), ama
`planning_node._on_obstacles` koordinatları **olduğu gibi** `PlanningPipeline`'a
veriyordu — oysa boru hattı **dünya** çerçevesinde çalışır (`set_state` odom
mutlak pozu, RRT* start=mutlak poz, MPPI maliyeti rollout dünya konumlarını
engel koordinatlarıyla karşılaştırır).

- **Etki:** araç origin'de ve ψ=0 iken tesadüfen doğru; **başka her durumda**
  engeller hem döndürülmemiş hem ötelenmemiş yanlış yere düşüyordu → var
  olmayan engelden kaçınma + gerçek engelin üstüne sürme.
- **Neden gözden kaçtı:** mission topic'leri (`current_target`, `waypoints`)
  `latlon_to_enu` ile **ENU-hizalı öteleme** ofseti taşır — orada yalnız odom
  xy eklemek DOĞRU. İki sözleşme aynı sanılmış.
- **Düzeltme:** `planning_node._body_to_world()` (ψ ile döndür + ötele); hem
  `obstacle_map` hem `classified_obstacles` bundan geçer.
- ⚠ **Frame kuralı:** perception topic'leri GÖVDE, mission topic'leri ENU-hizalı
  ÖTELEME. Yeni bir kaynak eklerken hangisi olduğunu yaz.

---

## 🌡️ MPPI λ (softmax sıcaklığı) — BENİMSENDİ (2026-08-02)

> λ artık `ParkurProfile.lambda_` ile parkur başına: **PARKUR1/2 = 10.0**,
> **PARKUR3 = 50.0** (eskiden global `MPPIConfig` 1.0). ⚠ Göl testinde doğrula.

Dönüşlü engelli rota, PARKUR2 profili (benimseme sonrası ayarlar), K=1000:

| λ | ESS ort | ESS p5 | sapma RMS | maks | açıklık | goal | hız | **\|Δu₀\| RMS** | ms |
|---|---|---|---|---|---|---|---|---|---|
| **1 (mevcut)** | **2.6** | **1.0** | 0.61 | 2.34 | +0.72 | 1.42 | 5.26 | **9.95 N** | 61.8 |
| **10** | 176 | 9.2 | 0.63 | 2.35 | +0.73 | 1.47 | 5.23 | **3.44 N** | 62.1 |
| 50 | 462 | 216 | 0.69 | 2.40 | +0.75 | 1.44 | 4.29 | 2.64 N | 64.2 |
| 100 | 661 | 530 | 0.72 | 2.39 | +0.69 | 1.42 | 3.38 | 2.46 N | 63.5 |
| 500 | 954 | 942 | 1.00 | 3.01 | +6.10 | **69.2 ✗** | 1.39 | 2.18 N | 68.7 |
| 1000 | 987 | 983 | 1.07 | 2.76 | +28.6 | **91.6 ✗** | 0.89 | 2.15 N | 67.5 |

- **λ=1 dejenere:** ESS 2.6/1000, p5 = 1.0 → adımların en az %5'inde MPPI
  ağırlıklı ortalama YAPMIYOR, tek rastgele örneği seçiyor. Sonuç: adımlar
  arası |Δu₀| RMS **9.95 N** — ±30 N eyleyicide her 50 ms'de ~%33 sıçrama.
  ("MPPI ilk iterasyon kararsız / zikzak" tuzağının ölçülmüş hâli.)
- **λ=10 tatlı nokta:** |Δu₀| 9.95 → 3.44 N (**2.9× yumuşak**), iz sapması /
  açıklık / goal / hız **pratikte değişmiyor**. Bedava kazanç.
- λ≥50'de seyir hızı düşmeye başlıyor (5.23 → 4.29 → 3.38); λ≥500'de araç
  **hedefe hiç varamıyor** — ağırlıklı ortalama maliyet farklarını siliyor,
  kontrol nominale çöküyor. λ≥500'deki "yüksek açıklık" kazanç değil, aracın
  engele hiç yaklaşamamasının yan etkisi.
- λ'nın adım süresine etkisi yok (62-69 ms, gürültü içinde).

**PARKUR3 (kamikaze) ayrı ölçüldü** — hedef 50 m ileride, yolda 2 dikkat
dağıtıcı engel, K=1000:

| λ | temas | **temas hızı** | \|Δu₀\| RMS | ESS ort | ESS p5 | adım |
|---|---|---|---|---|---|---|
| 1 | 0.25 m ✓ | 0.97 m/s | 8.34 N | 3.1 | 1.0 | 234 |
| 10 | 0.30 m ✓ | 1.18 m/s | 4.52 N | 199 | **1.9** | 230 |
| **50** | 0.28 m ✓ | **1.81 m/s** | 2.87 N | 475 | **112** | 260 |
| 100 | 0.23 m ✓ | 1.48 m/s | 2.39 N | 679 | 438 | 327 |

PARKUR3'te λ=10 **hâlâ dejenere** (ESS p5 = 1.9): kamikaze çekicisi
(`w_kamikaze`·(T+1) ≈ 2550) maliyet yayılımını büyüttüğü için aynı λ daha sert
softmax demek — λ maliyet ÖLÇEĞİYLE birlikte seçilmeli. λ=50: p5 112,
**temas hızı +%53 (1.18 → 1.81 m/s)**. Bu metrik kritik çünkü Parkur-3'ün
bitişi IMU şok eşiğiyle algılanıyor (`fsm_node shock_threshold_g = 5.0`) —
yavaş temas = algılanmayan görev sonu. Bedel: yaklaşma %13 uzun (230 → 260
adım). λ=100 daha da yumuşak ama temas hızı geri düşüyor (1.48).

**Boru hattı doğrulaması** (300 adım, K=1000, PARKUR1, dağıtılan varsayılanlar):

| | konum | hız | iz sapma RMS | **\|Δu₀\| RMS** | ms |
|---|---|---|---|---|---|
| λ=1 (önceki) | (67.78, 13.62) | 4.65 m/s | 0.20 m | 9.51 N | 68.6 |
| **λ=10 (profil)** | (67.07, 13.49) | 4.61 m/s | 0.19 m | **4.80 N** | 69.3 |

Kontrol yumuşaklığı **2.0×**, takip/hız/süre değişmedi.

---

## 🎛️ MPPI Saha Tuning Yüzeyi (ROS parametreleri, 2026-08-02)

> Göl/yarışma gününde **yeniden derlemeden** ayarlanır. Üç katman:
> `config/params.yaml` (node varsayılanı) → `config/hardware.yaml` `planning:`
> bloğu (saha kökü) → `hardware.launch` CLI argümanı (anlık deneme).

| ROS parametresi | varsayılan | sınır / not |
|---|---|---|
| `mppi_lambda` | `0.0` | **nöbetçi**: 0 → parkur profili kazanır (P1/P2=10, P3=50). >0 → üçünü de ezer |
| `mppi_sigma_u` | `5.0` | N, kontrol gürültüsü σ |
| `mppi_obstacle_margin` | `1.0` | m, SOFT ceza. ⚠ 1.5 Parkur-2 geçidini kapatır |
| `mppi_terminal_mode` | `lookahead` | `global` = eski davranış; geçersiz değer → WARN + varsayılan (node ölmez) |
| `mppi_terminal_lookahead_m` | `15.0` | m; ≥ seyir_hızı × horizon (T·dt) |
| `mppi_ref_window_size` | `100` | nokta (0.5 m aralıkta 50 m ileri) |
| `mppi_ref_window_enabled` | `true` | `false` → tam tarama (16× yavaş, A/B) |

```bash
ros2 launch girdap_decision hardware.launch.py --show-args   # açıklamalarıyla listeler
ros2 launch girdap_decision hardware.launch.py planning.mppi_lambda:=50.0
```

- **Öncelik:** launch-arg > `hardware.yaml planning:` > `params.yaml` > kod
  varsayılanı. Bir anahtar yaml'dan silinirse kod varsayılanı (λ'da parkur
  profili) kazanır — yaml sessizce bir kopya varsayılan dayatmaz.
- **⚠ Drift kapısı:** aynı varsayılan üç yerde yaşıyor (`MPPIConfig`,
  `_MPPI_DEFAULTS`, iki yaml). `test_planning_config_drift.py` (ROS'suz, launch
  dosyasını `ast` ile okur) değerleri ve anahtar kümelerini bağlar — biri
  değişip diğeri kalırsa CI kırmızı. ROS bilinmeyen yaml anahtarını SESSİZCE
  atar; yazım hatası (`mppi_lambdaa`) bu testle yakalanır.

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
  → HSV segmentasyon: turuncu→0, sarı→1, kırmızı→3, yeşil→4, kahverengi→5
  → kontur → bbox (+ opsiyonel YOLO katmanı → class 2 hedef)
  → /perception/buoys (vision_msgs/Detection2DArray)
```

- **Çekirdek:** `prototype/perception/camera_buoys.py`; node:
  `perception_camera_node` (kaynak-bağımsız, yalnız topic adına bağlı).
- **Sınıflar:** `class_id` (string): `"0"`=parkur_kenari (turuncu, RAL 2003),
  `"1"`=engel (sarı, RAL 1026), `"2"`=hedef (Parkur-3, YOLO katmanı),
  `"3"`=kırmızı, `"4"`=yeşil, `"5"`=kahverengi (2026-07-17 eklendi — parkurda
  bu renklerin de bulunduğu bulundu).
  (Şartname md 5.5.2.1 — eski "RAL 2008/1003" etiketi YANLIŞTI; F17.1.)
  ⚠ Kırmızı/yeşil/kahverengi eşikleri turuncu/sarı gibi SAHADA henüz
  doğrulanmadı — ilk tahmin, kör güvenilmemeli (bkz. camera_buoys.py
  docstring).
- **F-P.21 — ışık koşulu dayanıklılığı (2026-07-16/17 gerçek donanım
  testi):** akşamüstü/bulutlu ışıkta gerçek bir turuncu/sarı dubanın ölçülen
  doygunluğu (S≈29-83) sabit eşiklerin (S≥120) çok altında kalıp hiç tespit
  edilmedi. `equalize_saturation()` (yüzdelik-dilim germe, yalnız sahne
  GENELİNDE düşük doygunsa devreye girer — `saturation_clahe` param ile
  açık/kapalı) bunu düzeltir; `perception.camera.*` HSV parametreleri hâlâ
  sahada gerçek nesnelerle doğrulanmalı.
- **F-P.22 — algı kaynağı sessizce yok olabilir:** `use_onboard_camera`
  VARSAYILAN artık `true` (eskiden `false` — varsayım: `/perception/buoys`'u
  algı ekibinin AYRI paketi, `girdap-ida-algi`, üretir; ama gerçek donanım
  testinde o paket bu ortamda hiç yoktu, hiçbir şey `/perception/buoys`'u
  üretmedi, sessizce fark edilmeden kaldı). ⚠ Algı ekibinin kendi OAK node'u
  DA çalışacaksa `use_onboard_camera:=false` ver — ikisi aynı anda açılırsa
  hem topic çakışır hem OAK-D USB cihazını iki süreç aynı anda açamaz.
  `perception_fusion_node`'un sync bekçisi artık iki girdiden biri (LiDAR ya
  da kamera) HİÇ akmıyorsa da WARN basar (öncesinde yalnız "ikisi de aktı
  ama eşleşmedi" durumunu yakalıyordu).
- **Mock YOLO:** gerçek `.pt` yok → `YoloInference` mock modda sabit test
  bbox'ı döner. Gerçek model gelince `perception.camera.yolo_model_path`
  parametresi verilir — kod yolu aynı; ultralytics **lazy import** (mock
  modda hiç yüklenmez). Replace = yalnız `_infer_real` doğrulaması.
  `use_yolo`/`use_yolo_localizer` VARSAYILAN artık `true` (2026-07-17 —
  gerçek yarışma kararı, model_path boş kaldığı sürece güvenli mock'a düşer).
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
  gelince devreye girer (arkadaşın üstleniyor).
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
  **Kısmen kapatıldı (2026-08-01):** `fusion.keyframe_rate_hz` throttle'ı
  20 dk görevi 11 416 → 6 000 key'e indirdi. Kalan O(N) kaynağı: her
  flush'taki tam `calculateEstimate()` (bkz. iSAM2 bölümü "Açık bulgu").
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
