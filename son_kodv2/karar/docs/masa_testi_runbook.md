# Masa Testi Runbook'u — Jetson + Pixhawk + RFD868 + QGC

> Video runbook'unun (§2 "gün öncesi") uygulama kılavuzu: her test için
> KOMUT + beklenen ÇIKTI (PASS kriteri, koddan birebir log satırları) +
> FAIL ise ne yapılacağı. Sırayla koş: her test bir sonrakinin ön koşulu.
>
> ⚠️ **GÜVENLİK: tüm masa testleri PERVANELER SÖKÜLÜ yapılır.** Arm/KILL
> testleri motorlara gerçek komut basar.
>
> Yazım: 2026-07-11, commit `c696989`'a karşı doğrulandı.

## Terminal hazırlığı

Kurulum **girdap-ida-algi reposundaki rehberle** yapıldıysa
(`docs/jetson_kurulum_rehberi.md` — önerilen yol) her şey `~/.bashrc`'de
hazır: tek workspace `~/ros2_ws`, iki paket birlikte derli, repo yolu
`~/ros2_ws/src/girdap-decision`. Yeni terminal açmak yeter; elle gerekirse:

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
export PYTHONPATH=$HOME/ros2_ws/src/girdap-decision:$PYTHONPATH
```
(Alternatif eski yerleşim `~/girdap-decision` + kendi ros2_ws'i —
`docs/jetson_deployment.md`; hangisiyse yolları ona göre oku.)

---

## ⚡ M0-ÖNCESİ — FC'ye GÜÇ VERMEDEN ZORUNLU KONTROL (OLAY 2026-07-12)

> Masa oturumu kapanışından SONRA FC, hafızasındaki test görevini RC/AUTO
> geçişiyle KENDİ BAŞINA tam güçte koştu (donanim_gunlugu §OLAY). Bu blok
> kapanana kadar her güç verişte, PERVANELER SÖKÜLÜ olarak:

1. **RC mod anahtarının konumuna bak** — AUTO'da OLMASIN (CH5 kalibrasyonda
   atlamalıydı; FC ekibi FLTMODE düzenini yapana kadar güvenme).
2. **FC görev hafızasını sil:** QGC Plan → Remove All (ya da yığın açıksa
   `ros2 service call /mavros/mission/clear mavros_msgs/srv/WaypointClear`).
   Sahte koordinatlı görev FC'de durduğu sürece her AUTO geçişi = tam-yol kaçış.
3. **`BRD_SAFETY_DEFLT=1` geri yaz** (masa için 0 yapılmıştı) ya da emniyet
   düğmesine erişimi çöz — FC ekibi kararı.
4. Test bitince bataryayı TAKILI BIRAKMA.

## M0 — Jetson yazılım sağlığı (donanımsız)

```bash
cd ~/ros2_ws/src/girdap-decision
python3 -m pytest prototype/tests/ -q
```
- **PASS:** `259 passed, 2 skipped` (F-M.1 `dff52af` + F-M.2 `3931220`
  sonrası taban — önce `git pull`! PC/GPU'suz karşılığı 257/4; skip'ler
  makine tipine göre gerekçeli — GPU'lu Jetson'da cupy'siz-fallback 2'lisi,
  GPU'suz makinede cupy parite + gerçek-RNG 2'lisi. M8/cupy kurulumu sayıyı
  artık DEĞİŞTİRMEZ; Jetson'da cupy zaten kurulu, 2026-07-11).
- **FAIL:** hangi modül import hatası veriyorsa `jetson_deployment.md`'nin
  ilgili adımı eksik (gtsam/scipy/mavros_msgs). Suite yeşil olmadan donanım
  testine GEÇME.

## M1 — Pixhawk ↔ Jetson (USB, MAVROS)

> ✅ **GEÇTİ 2026-07-12** — AMA USB'den DEĞİL: Pixhawk USB-C soketi arızalı
> (descriptor -32, FC ekibinde; günlük §2). Çalışan yol **TELEM2 → FTDI**:
> `fcu_url:=serial:///dev/ttyUSB0:57600`. IMU bu hatta ~10 Hz (57600 tavanı;
> MAVROS `set_stream_rate` ile yükseltildi — ~50 Hz istenirse USB tamiri ya
> da SR2 paramları). Soket tamir edilene kadar aşağıdaki tüm M adımlarında
> fcu_url override'ını kullan. Ayrıntı: donanim_gunlugu_2026-07-12.md §9.

Pixhawk USB'yi tak (`ls /dev/ttyACM*` → ACM0 bekleniyor; farklıysa
`fcu_url:=serial:///dev/ttyACM1:57600` override et).

```bash
# T1 (yığın):
ros2 launch girdap_decision hardware.launch.py
# T2 (kontrol):
ros2 topic echo /mavros/state --once
ros2 topic hz /mavros/imu/data
```
- **PASS:** `connected: true`; IMU akıyor (USB: ~50 Hz / TELEM2 57600:
  ~10 Hz kabul); T1 logunda
  `mavros_bridge aktif (heartbeat=5.0s, ...)`.
- **FAIL:** `connected: false` → kablo/port/baud (57600) veya FC boot
  bekle; `dialout` grubu üyeliği (`jetson_deployment.md`).

## M2 — QGC ↔ RFD868x ↔ Pixhawk (kablosuz YKİ)

RFD868x biri Pixhawk TELEM portunda, eşi laptopta USB. QGC'yi aç.
- QGC otomatik bağlanmazsa: Application Settings → Comm Links → seri link
  ekle (RFD'nin COM portu, 57600) → Connect.
- **PASS (3 alt madde):**
  1. QGC HUD'da telemetri canlı (mod, batarya, GPS).
  2. QGC'den **Arm** → M1'deki `ros2 topic echo /mavros/state` içinde
     `armed: true` görünür (komut RFD→FCU→USB→MAVROS zincirini doğrular).
     Hemen Disarm et.
  3. QGC'den mod değiştir (ör. Manual→Hold) → `/mavros/state` `mode:`
     alanında değişim görünür.
- **FAIL:** RFD LED'leri eşleşme durumunu gösterir; baud/NetID eşleşmesini
  FC ekibiyle kontrol et. **Not:** `gcs_url: ""` DOĞRU — QGC mavros
  üzerinden değil, radyodan doğrudan FCU'ya bağlanır.

## M3 — FSM zinciri: BOOT → ARM → BEKLEMEDE

Yığın açıkken (M1):
```bash
ros2 topic echo /girdap/mission/state
```
- Başlangıçta `BOOT`; MAVROS bağlanınca `ARM`; **QGC'den arm edince**
  (pervaneler sökülü!) `BEKLEMEDE`.
- **PASS:** üç durum sırayla görüldü. Disarm etme — M4/M5 devamı.
- **FAIL:** `ARM`'da takılıyorsa `/mavros/state.armed` gelmiyor (M1/M2'ye dön).

## M4 — Görev yükleme: QGC → Pixhawk → mission_manager (fc modu)

Yığını fc moduyla başlat (T1'i kapatıp):
```bash
ros2 launch girdap_decision hardware.launch.py mission_source:=fc
```
- Boot logunda: `mission_source=fc — /mavros/mission/waypoints bekleniyor`.
- QGC Plan ekranında 4 köşe + kapanış noktası çiz → **Upload**.
- **PASS:** T1 logunda
  `FC görevi alındı: N item → M waypoint (arrival=2.0 m, dwell=2.0 s)`
  (N≈6: home+4köşe+kapanış; M≈5 — home index0 atlanır).
- **FAIL — `... gezinme waypoint'i yok` uyarısı:** liste geldi ama filtre
  hepsini eledi → `skip_home_seq0` ters olabilir (T0-f tek anahtarı) ya da
  QGC spline/farklı komut tipi üretti. `ros2 topic echo
  /mavros/mission/waypoints --once` ile ham listeyi görüp `command`
  alanlarını oku (16=NAV_WAYPOINT beklenir).
- **FAIL — hiç log yok:** `ros-humble-mavros-msgs` kurulu mu; topic latched
  geldi mi (`--qos-durability transient_local` ile echo dene).

## M5 — 🎬 Başlatma tetiği: QGC'den mod → GUIDED (md 3.3.1/3)

Ön koşul: M3 (BEKLEMEDE'de, armed) + M4 (görev yüklü). Pervaneler SÖKÜLÜ.
```bash
# T2'de izle:
ros2 topic echo /girdap/mission/state &
ros2 topic hz /girdap/control/thrust
```
- QGC'den modu **GUIDED**'a çevir.
- **PASS (3 alt madde):**
  1. T1 logunda: `YKİ mod komutu (MANUAL→GUIDED) — görev başlatıldı (md 3.3.1/3)`
     ve ardından `görev başlatıldı (FSM aktif parkur)`.
  2. `/girdap/mission/state` → `PARKUR1`.
  3. `/girdap/control/thrust` ~10 Hz yayına başlar (MPPI çalışıyor).
- **Kenar kuralı testi (negatif):** disarm + yığını yeniden başlat; araç
  ZATEN GUIDED'dayken arm et → görev BAŞLAMAMALI (BEKLEMEDE'de kalır).
  Sonra HOLD'a al, tekrar GUIDED → başlamalı.
- **FAIL:** mod adı farklı olabilir — `/mavros/state.mode` gerçek dizgiyi
  gösterir (ArduRover'da "GUIDED" beklenir); gerekirse
  `params.yaml fsm_node.start_on_mode` değerini ona eşitle.

## M6 — KILL zinciri + failsafe (F14, gerçek FCU'da İLK KEZ)

Ön koşul: M5 sonrası görev aktif (thrust yayında), pervaneler sökülü.
1. **Komutlu KILL:**
   ```bash
   ros2 service call /girdap/mission/kill std_srvs/srv/Trigger {}
   ```
   **PASS:** logda `*** KILL — motorlar durduruluyor ***` + `DISARM başarılı`
   (F14.1: KILL artık FCU'yu da disarm eder) → QGC'de Disarmed görünür;
   thrust yayını sıfır komuta düşer.
   **Not:** KILL kalıcı latch (F14.4) — devam için yığını yeniden başlat.
2. **Kontrollü disarm ≠ failsafe (F14.2, video güç-kesme provası):**
   görev aktif DEĞİLKEN arm et, sonra:
   ```bash
   ros2 service call /girdap/bridge/disarm std_srvs/srv/Trigger {}
   ```
   **PASS:** `DISARM başarılı` var; `FAILSAFE — beklenmedik disarm → KILL`
   YOK — disarm'dan SONRAKİ birkaç monitor tick'ini de izle (2026-07-12
   masasında hata tam burada, kenardan BİR TICK SONRA çıkmıştı; kök neden
   `_was_armed` latch'iydi, F-M.2 ile düzeltildi `3931220` — bu adım artık
   düzeltmenin gerçek-FCU teyidi).
3. **Heartbeat kaybı:** yığın koşarken Pixhawk USB'sini çek.
   **PASS:** ~5 sn içinde KILL loglanır. ⚠ Bu senaryoda disarm FCU'ya
   ULAŞAMAZ (hat kopuk — F14.1a bilinen sınır): FCU tarafı emniyeti
   ArduPilot'un KENDİ failsafe'idir → **FC ekibiyle FS_GCS/FS_THR/FS_ACTION
   parametrelerini bu masada teyit et.** USB'yi geri tak, yeniden başlat.

## M7 — Telemetri çıktıları + Ekran-2 (Dosya-2 provası)

Yığın birkaç dakika koştuktan sonra (M5 sırasında birikir):
```bash
ls -lh ~/girdap_logs/telemetry/ ~/girdap_logs/grafik/
python3 ~/ros2_ws/src/girdap-decision/scripts/run_ekran2.py          # en yeni CSV → PNG
ls ~/girdap_logs/viz/
```
- **PASS:** iki dizinde de büyüyen CSV; `run_ekran2` PNG üretir; PNG'de
  hız/heading/thrust panelleri M5 koşusundaki hareketi gösterir (thrust
  paneli DOLU olmalı — T0-g'nin kanıtı).
- **FAIL:** CSV yoksa telemetry_node loguna bak (F15.1 mutlak yol düzeltmesi
  bunu çözmüştü — regresyon bildir).

## M8 — (bonus, Jetson elindeyken) CUDA Faz A + D3 ölçümü

```bash
sudo nvpmodel -m 0 && sudo jetson_clocks                 # ÖLÇÜMDEN ÖNCE
python3 -m pip install --user cupy-cuda12x
python3 -c "import cupy; print(cupy.cuda.runtime.getDeviceCount(), cupy.__version__)"
cd ~/ros2_ws/src/girdap-decision
python3 -m pytest prototype/tests/test_mppi.py -q        # parite testi artık koşar
python3 scripts/bench_mppi.py --backend numpy
python3 scripts/bench_mppi.py --backend cupy
python3 scripts/bench_mppi.py --backend cupy --steps 600 # sürüklenme (~60 s)
free -m                                                   # CUDA context RAM
```
- **Kabul (plan §5):** cupy ort < 50 ms (20 Hz), hedef < 20 ms; parite testi
  geçer; ek RAM < 1 GB; 600 adımda sürüklenme büyümüyor (`tegrastats` ile).
- Sonuca göre `control_rate_hz` kararı (10 → 20 → sahada doğrula → 50).
  Ayrıntı: `docs/mppi_cuda_plani.md`.

---

## Sonuç kaydı (masa günü doldur)

| Test | Sonuç | Not / ölçüm (2026-07-12 akşam — günlük §10) |
|---|---|---|
| M0 suite | ☑ PASS | 250/2 |
| M1 MAVROS | ☑ PASS | TELEM2 ttyUSB0:57600, connected=true, IMU 10.4 Hz |
| M2 QGC/RFD | ⏭ ERTELENDİ | QGC laptop yok; QGC'nin ARM64 sürümü yok (Jetson'a kurulamaz) |
| M3 FSM | ☑ PASS | BOOT→ARM→BEKLEMEDE; ön koşullar: BRD_SAFETY_DEFLT=0 + RC kalibrasyonu (günlük §10) |
| M4 fc görev | ☑ PASS | skip_home_seq0 = DOĞRU (5 item → 4 wp, komut=16, latched ✓; mavros mission/push ile) |
| M5 GUIDED tetiği | 🟡 KISMİ | FCU GPS'siz GUIDED REDDEDİYOR ("Flight mode change failed") — açık alanda tekrar; /girdap/mission/start ile PARKUR1 ✓ |
| M6 KILL/failsafe | ✗ KOŞULAMADI | planning_node F-M.1 OOM çöküşü (92 GB, home_ref=0,0 + uzak wp); F-M.2: kasıtlı disarm yine FAILSAFE→KILL bastı |
| M7 telemetri | 🟡 KISMİ | boot'tan üretim ✓ (Dosya-3 741+ kare); dolu-veri provası görev koşusuyla |
| M8 CUDA | ⏭ | M5 tam geçmeden anlamsız (mock ön-kontrol 9.90 Hz PASS — günlük §9) |

Hepsi PASS → suda prova (T0-h) → çekim günü (`video_gunu_runbook.md`).
FAIL çıkan her madde için log çıktısını kaydet (foto/kopya) — kod tarafında
karşılığını buluruz.
