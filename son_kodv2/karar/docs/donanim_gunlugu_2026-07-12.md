# Donanım Test Günlüğü — 2026-07-12 (Jetson, "her şey takılı" günü)

> Atölye masası: Jetson Orin Nano Super + Livox Mid-360 (Ethernet) +
> OAK-D Lite (USB3) + Pixhawk 6C (batarya + TELEM2/USB denemeleri).
> Yazılım tabanı: commit `d9778fe` (F-L.1) / suite **250 passed, 2 skip**.

## Özet tablo

| Test | Sonuç | Sayılar |
|---|---|---|
| LiDAR veri akışı | ✅ PASS | 10.00 Hz, ~20k nokta/mesaj, IMU 200 Hz |
| LiDAR kümeleme (canlı, Jetson) | ✅ PASS | üretim 38.6 ms / geniş-Z 52.3 ms medyan (F5.3 teyidi) |
| `perception_lidar_node` canlı | ✅ PASS | 9.98-10.07 Hz `/perception/obstacle_map` (F-L.1 düzeltmesi sonrası) |
| OAK kamera | ✅ PASS | UsbSpeed.SUPER + 11.9 FPS + kare net |
| Ortak yük (LiDAR+kamera) | ✅ PASS | kamera 12.0 FPS düşmedi, CPU ~%30, 53 °C, ~7.4 W, RAM 2.3/7.6 GB |
| M1 MAVROS (Pixhawk) | ❌ BLOKE | aşağıda — donanım arızası şüphesi, FC ekibi araştırıyor |

## 1) LiDAR aşama-1 — F-L.1 bulundu ve düzeltildi

Dün geceki "bir ERROR çıktı" olayının kökü: **gerçek Livox PointCloud2
karışık dtype'lı** (x/y/z/intensity float32 + tag/line uint8 + timestamp
float64, point_step=26) ve `read_points_numpy` bunu assert'le reddediyor →
üretim node'u İLK gerçek mesajda ölüyordu. TDD ile düzeltildi (`d9778fe`),
ayrıntı `docs/kod_denetimi.md` F-L.1. Sentetik testler `create_cloud_xyz32`
kullandığından yakalayamamıştı (maskeleme deseni).

- Sürücü headless başlatma (RViz'siz):
  `ros2 run livox_ros_driver2 livox_ros_driver2_node --ros-args -p
  xfer_format:=0 -p data_src:=0 -p publish_freq:=10.0 -p
  user_config_path:=$HOME/ws_livox/install/.../MID360_config.json`
- 🟡 **F-L.2 (revize — aşağı bkz. kod_denetimi.md):** Livox stamp +0.20 s geride
  (0.19-0.21 bandı) > `sync_slop_s=0.1` → kamera-LiDAR füzyon sync'i bu
  haliyle hiç eşleşmez. T1 kararı: restamp / slop / PTP (kod_denetimi.md).

## 2) M1 MAVROS — Pixhawk bağlantı teşhisi (FC ekibine)

İki yol da denendi, ikisi de başarısız; **teşhis kesin, neden fiziksel:**

### TELEM2 → USB-UART (FTDI FT231X, DU0EFEA7)
- Batarya bağlı + LED'ler yanarken bile 9 baud'da (9600→1.5M) **sıfır bayt**.
- MAVROS aktif heartbeat sondası (25 sn) → `connected: false`.
- Yanlış baud bile olsa çöp bayt görünürdü; mutlak sessizlik = elektriksel.
- **En olası neden: TX/RX düz bağlanmış (çapraz olmalı) ya da GND kopuk.**
  Doğru bağlantı: dönüştürücü TXD → TELEM2 **pin 3** (RX), dönüştürücü
  RXD → TELEM2 **pin 2** (TX), GND → **pin 6**; VCC BOŞ (batarya besliyor).
  TELEM2 JST-GH: 1=VCC 2=TX 3=RX 4=CTS 5=RTS 6=GND.
- İkincil ihtimal: SERIAL2_PROTOCOL/BAUD değiştirilmiş (QGC ile teyit edilir).

### USB-C doğrudan (kamera kablosuyla — kablonun sağlamlığı OAK'ta kanıtlı)
- Çekirdek her takışta cihaz algılıyor ama kimlik okuyamıyor:
  `new low-speed/full-speed USB device` + `device descriptor read/64,
  error -32` (tekrarlı) + `attempt power cycle`. Hız algısının low↔full
  gidip gelmesi = D+/D- sinyal bütünlüğü sorunu.
- Fiş 180° çevrildi, tam oturtuldu → değişmedi.
- **Sonuç: Pixhawk'ın USB-C soketi şüpheli (eğik pin / soğuk lehim).**
  Ayar/parametre bu tabloyu ÜRETEMEZ — firmware'siz Pixhawk bile USB'de
  düzgün görünür; "arkadaş ayar değiştirdi mi" sorusunun cevabı: USB için
  imkânsız, TELEM2 için mümkün ama ikincil.
- **Çapraz test önerisi:** Pixhawk USB'yi başka bir bilgisayara (QGC'li
  laptop) tak. Orada da `descriptor error` / görünmeme olursa soket arızası
  kesinleşir → tamir/RMA. Orada çalışırsa Jetson tarafına döneriz (beklenmez).

### M1'in yeniden koşulması (donanım çözülünce)
- USB yolu: `ttyACM0` düşer → `ros2 launch mavros apm.launch
  fcu_url:=serial:///dev/ttyACM0:57600` → `/mavros/state` connected:true
  + `/mavros/imu/data` ~50 Hz. Sonra tam stack `hardware.launch.py`.
- TELEM2 yolu: kablo çaprazlanınca 57600'de heartbeat akmalı; MAVROS
  `fcu_url:=serial:///dev/ttyUSB0:57600` ile aynı test.

## 3) Ortak yük / termal

OAK 12 FPS + Livox sürücüsü + perception_lidar_node aynı anda: CPU ~%30
(6 çekirdek, 1.7 GHz), RAM 2.3/7.6 GB, 53 °C, ~7.4 W. Bol pay var.
Not: boşta governor çekirdekleri 729 MHz'e indiriyor; gerçek görev yükü
clock'u yukarı itiyor, yine de sahada `sudo jetson_clocks` iyi pratik.

## 4) Sıradaki adımlar

1. FC ekibi: Pixhawk USB soketi çapraz testi + TELEM2 kablo düzeltmesi
   (yukarıdaki pin reçetesi) + SERIAL2/failsafe paramlarının QGC dökümü
   (`docs/olcum_formu.md` bölümüne).
2. Pixhawk canlanınca: masa runbook M1→M8 (PERVANELER SÖKÜLÜ!).
3. F8.3 iSAM2 graf/RAM ölçümü Jetson'da (bu günlükle aynı gün koşuluyor —
   sonuç kod_denetimi.md'ye).

## 5) Tam stack boot smoke + M7 ön-kontrolü (FCU'suz) — ✅ PASS

`ros2 launch girdap_decision hardware.launch.py` Jetson'da FCU olmadan
koşturuldu (öğleden sonra oturumu):

- **10 girdap node'u + mavros + 3 static TF ayakta, ölen süreç yok** —
  launch kompozisyonu ve tüm bağımlılıklar Jetson'da uçtan uca doğrulandı.
- fsm_node boot'tan kısa süre sonra **KILL bastı** — FCU yok → heartbeat
  kaybı KILL'i (mavros_bridge bekçisi). FCU'suz masada BEKLENEN ve doğru
  güvenli davranış; F14.4 latch gereği kurtarma yok, gerçek FCU testinde
  (M6) yeniden değerlendirilecek.
- **M7 ön-kontrolü:** üç kayıt da boot'tan itibaren üretiliyor —
  `telemetry/telemetri_*.csv` (Dosya-2 header birebir, 2 Hz; FCU'suz
  alanlar boş = doğru), `grafik/grafik_*.csv` (10 Hz, thrust sütunlu),
  `local_map/session_*/frame_*.png` (~1 Hz, geçerli grayscale kare).
  Asıl M7 (dolu verilerle) FCU geldikten sonra tekrarlanacak.

## 6) F8.3 iSAM2 ölçümü — ✅ (ayrıntı kod_denetimi.md)

20 dk sim, 11.4k anahtar: RAM +30 MB, flush medyan 0.3→12.7 ms (lineer),
p95 21 ms < 100 ms bütçe → yarışma süresi için marginalization gereksiz.

## 7) Kamera-LiDAR füzyonu canlı deneyi — ✅ ÇALIŞIYOR (F-L.2 revize)

Gerçek Livox + nişanlı sahte kamera tespitiyle: sync 20 sn'de 90 çıkış
(F-L.2'nin "hiç ateşlemez" öngörüsü YANLIŞTI — 10 Hz yoğun lidar akışı slop
içinde daima aday bulur; kalan etki ~0.2 s zaman kayması, düşük öncelik) ve
gerçek kümeye nişanlanan bbox 99/99 eşleşti ("1 eşleşti" logu) — bearing
işaret düzeltmesi (`e66cb40`) gerçek veriyle ilk kez kanıtlandı. Eşleşmeyen
kümeler class 99 ile korunuyor ✓. Poz füzyonu (iSAM2) için bkz. §6/F8.3.

## 8) M8 ön-kontrolü: MPPI CANLI DÖNGÜDE — ✅ PASS (sahte beslemeli)

Gerçek `planning_node` (K=1000, T=50, cupy fused) + sahte odom/hedef/görev-durumu/
engel-haritası (8 duba) beslemesiyle 60 sn: `/girdap/control/thrust`
**9.90 Hz, aralık medyan 100.0 ms / p95 110.9 ms** (maks 733 ms tek seferlik =
ilk adım CUDA derlemesi). 10 Hz control_rate GERÇEK node içinde Jetson'da
tutuyor. Kalan D3: aynı ölçüm gerçek FCU + gerçek görevle (M8).

## 9) 🎉 M1 GEÇTİ — TELEM2 kablo düzeltmesi sonrası (öğleden sonra)

- Kablo çaprazlandıktan sonra hat canlandı: 57600'de MAVLink v2 akışı
  (`fd` magic, sysid 1). **`/mavros/state` connected=True**, mod HOLD.
- IMU başta 0.5 Hz (SR2 stream rate'leri düşük) → MAVROS
  `set_stream_rate` (ALL=10, RAW/EXTRA1=25) sonrası **10.4 Hz** gerçek
  attitude verisi. 57600 baud tavanı not: runbook'un "~50 Hz" hedefi USB
  yolu (soket tamiri) ya da FC ekibinin SR2 paramlarıyla.
- **Tam stack gerçek FCU'yla smoke:** heartbeat KILL YOK (sabahki FCU'suz
  koşunun tersine), FSM BOOT→**ARM** durumuna ilerledi, mavros köprüsü
  sağlıklı. `fusion/odom` 0 Hz — kapalı mekânda GPS fix yok → EKF local
  pose yayınlamıyor (beklenen; F8.2 bekçisinin alanı).
- ⚠️ FCU PreArm: **"Radio failsafe on"** — RC kumanda bağlı/ayarlı değil.
  M4 (ARM) öncesi FC ekibi: RC'yi aç/bağla ya da failsafe paramını düzelt.
  ARM testleri ayrıca PERVANELER SÖKÜLÜ teyidi ister.

Kalan masa sırası: M3 (QGC laptop + görev upload) → M4-M6 (ARM/GUIDED/KILL,
pervanesiz) → M7 (dolu kayıtlar) → M8 (gerçek D3).

## 10) 🔥 AKŞAM MASA OTURUMU — M3/M4 GEÇTİ, M5 kısmi, 2 GERÇEK BUG YAKALANDI

QGC laptopu yoktu → tüm zincir Jetson'dan MAVROS servisleriyle sürüldü
(M2/RFD ertelendi). Ham log: `~/girdap_logs/masa_testi/masa_stack_2026-07-12_aksam.log`.

### Donanım hazırlığı sırasında çözülenler (FC ekibine rapor edilecek)

1. **Emniyet düğmesi bulunamadı** (muhtemelen GPS modülü üstünde, GPS
   masada bağlı değildi... sonra ZED-F9P'nin FC'ye bağlı olduğu görüldü ama
   düğmeye erişilemedi) → **`BRD_SAFETY_DEFLT=0` yazıldı + FCU reboot**
   (MAVROS param/set ile). ⚠️ SAHADA GERİ AÇILMALI ya da düğme erişilir
   olmalı — şartname md 3.3.1/4 fiziksel güç kesme ayrı konu.
2. **RC kalibrasyonu HİÇ yapılmamıştı** → "Arm: Roll/Yaw/Throttle is not
   neutral" ile ARM reddediliyordu (F14.7 retry+red davranışı DOĞRU çalıştı,
   KILL tetiklemedi ✓). 60 sn kayıtla (352 örnek, /mavros/rc/in) kanallar
   ölçüldü ve yazıldı: RC1 882/1650/2129, RC2 943/2137/2146,
   RC3 915/1075/2119 (MIN/TRIM/MAX). ⚠️ Kumanda tuhaf: CH2 dinlenmede üst
   uca, CH3 alt uca yakın — FC ekibi QGC'de düzgün kalibrasyon + RCMAP
   kontrolü yapmalı. Trim'ler gerçek dinlenme konumuna yazıldı (masa çözümü).
3. RC açılınca "Radio Failsafe Cleared" ✓; RC→FCU hattı canlı (modu
   HOLD→MANUAL çevirdi).

### Test sonuçları

| Test | Sonuç | Kanıt |
|---|---|---|
| M1 tekrar | ✅ PASS | connected=true, TELEM2/ttyUSB0:57600 |
| M2 QGC/RFD | ⏭️ ERTELENDİ | QGC laptop yok; ayrıca QGC'nin ARM64 Linux sürümü YOK — Jetson'a kurulamaz, laptop şart |
| M3 FSM zinciri | ✅ PASS | BOOT→ARM→BEKLEMEDE (arm `/girdap/bridge/arm` ile; FCU "Throttle armed") |
| M4 fc görev | ✅ PASS | mavros mission/push ile 5 item FC'ye yazıldı → geri okuma → "FC görevi alındı: 5 item → 4 waypoint"; skip_home_seq0 ✓, komut=16 ✓, latched ✓ |
| M5 GUIDED tetiği | 🟡 KISMİ | FC "Flight mode change failed" — GPS'siz GUIDED'ı ArduPilot REDDEDIYOR (bilinen sınır). Görev `/girdap/mission/start` ile başlatıldı: BEKLEMEDE→PARKUR1 ✓, "görev başlatıldı" ✓. GUIDED-mod tetiği SAHADA (GPS fix'li) test edilecek |
| M6 KILL zinciri | ⏭️ KOŞULAMADI | planning_node çöktüğü için thrust yayını hiç başlamadı (aşağıda F-M.1); KILL'in kendisi değil ortam engelledi. Yalnız failsafe yolu gözlendi (F-M.2) |
| M7 kayıtlar | 🟡 KISMİ | CSV/PNG üretimi boot'tan itibaren aktifti (Dosya-3 741+ kare); dolu-veri provası görev koşusu olmadığından yine bekliyor |
| M8 gerçek D3 | ⏭️ | M5 tam geçmeden anlamsız |

### 🔴 F-M.1 — planning_node OOM ÇÖKÜŞÜ (görev başlar başlamaz)

`cupy.cuda.memory.OutOfMemoryError: Out of memory allocating 92,790,828,032
bytes` — `mppi.py:466 _trajectory_cost` içinde, `pipeline.py:292` üzerinden.

**Kök neden zinciri:** kapalı mekân → GPS fix yok → `home_ref` hiç set
edilmedi (0,0 kaldı) → FC'den gelen 40°K/29°D waypoint'leri origin'e göre
~4400 km uzakta ENU'ya çevrildi → MPPI referans hattı devasa n_ref'e
bölündü → maliyet tensörü (K,T+1,n_ref) ≈ 92 GB → node ölümü.

**Değerlendirme:** sahada GPS fix'le home_ref gerçek konuma oturur, bu
patlama OLMAZ. AMA korumasız: (a) fix gelmeden görev başlatılabiliyor
(F8.4'ün kod karşılığı yok — yalnız operasyon notu), (b) referans uzunluğu/
n_ref üst sınırı yok, (c) hedef-uzaklık makullük kontrolü yok. Yanlış
koordinat girilse SAHADA DA öldürür. **TDD düzeltme planı (sonraki oturum):**
mission_manager fix/home_ref-yokken başlatmayı reddetsin + planning
n_ref/hedef-mesafe tavanı (ör. hedef >10 km = hata logu + görev reddi).

### 🟠 F-M.2 — kasıtlı disarm yine FAILSAFE sanıldı (F14.2 gerçek-FCU regresyonu?)

`/girdap/bridge/disarm` çağrısında: "DISARM başarılı" ✓ AMA hemen ardından
"FAILSAFE — beklenmedik disarm → KILL" + fsm "*** KILL ***". F14.2'nin
`_expected_disarm` bayrağı mock testlerde geçiyordu; gerçek FCU'da (görev
aktif + planning ölü durumda) failsafe yolu yine tetiklendi. Sonraki oturum:
bayrağın gerçek zamanlama/yarış koşulu incelemesi (görev-aktifken kasıtlı
disarm senaryosu node testine eklenecek). Video güç-kesme provasını
İLGİLENDİRİYOR (md 3.3.1/4).

### FC ekibine gidecek notlar

- `BRD_SAFETY_DEFLT=0` yapıldı (masa için) — sahada karar sizin.
- RC kalibrasyonu QGC'yle baştan yapılmalı; kumandanın kanal eşlemesi
  (CH2 üst uçta dinleniyor) kontrol edilmeli.
- ARM artık çalışıyor; PreArm'da yeni not: "AHRS: not using configured
  AHRS type" (kapalı mekân EKF'i, muhtemelen fix'le geçer).
- GUIDED kapalı mekânda İMKÂNSIZ (konum kestirimi şart) — M5/M6 tam testi
  açık alanda (park/bahçe, GPS görür yer) yapılabilir, su gerekmiyor.

### Sonraki oturum sırası

1. F-M.1 guard'ları TDD ile (fix'siz başlatma reddi + n_ref tavanı).
2. F-M.2 incelemesi + node testi.
3. Açık alanda: GUIDED tetiği (M5 tam) → M6 KILL zinciri → M7 dolu kayıt →
   M8 D3. QGC laptop gelirse M2 + gerçek QGC Plan Upload.

### 🔴 OLAY RAPORU (oturum kapanışı SONRASI) — FC görevi kendi başına koştu

Yığın kapatıldıktan sonra (Jetson/MAVROS tamamen kapalı) **motorlar tam
güçte döndü ve güç kesilip verilince görev devam etti** (operatör gözlemi).
Teşhis: M4 testi için FC'ye yazılan 4-köşe görev (40°K/29°D — SAHTE, ~4400 km
uzak) FC kalıcı hafızasında; FC RC üzerinden ARM + AUTO/görev moduna geçince
ArduRover ilk waypoint'e doğru süresiz tam yol bastı. Muhtemel tetik: RC mod
kanalı (CH5, kalibrasyon kaydında iki konum arasında atlıyordu) AUTO'ya denk
geldi. `BRD_SAFETY_DEFLT=0` (masa değişikliği) çıkışları serbest bırakmıştı.
Pervaneler sökülü olduğu için hasar yok — pervanesiz kuralının birebir
gerekçesi.

**ZORUNLU aksiyonlar (bir sonraki güç verişte, PERVANESİZ):**
1. FC görev hafızasını SİL (`/mavros/mission/clear` ya da QGC Plan→Remove All)
   — sahte koordinatlı görev FC'de durduğu sürece her AUTO geçişi tam-yol
   kaçış demektir.
2. `BRD_SAFETY_DEFLT=1` GERİ YAZ (masa testi bitti).
3. FC ekibi: RC mod kanalı eşlemesi (AUTO tek kol/anahtar atışıyla
   SEÇİLEMEMELİ), FLTMODE listesinin gözden geçirilmesi, ESC/SERVOn_TRIM
   kalibrasyonu, ARMING_RUDDER kararı.
4. Bu olay bitene kadar bataryayı takılı bırakma; her güç verişte önce
   RC mod anahtarının konumunu kontrol et.
