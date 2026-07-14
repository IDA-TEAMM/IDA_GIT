---
name: donanim-test-plani
description: "Kod bitti (2026-07-11) — bundan sonra NE test edilecek: masa M1-M8 madde madde, her testin amacı/PASS kriteri/riski; suda prova; video; LiDAR aşamaları; ölçüm bağımlılıkları"
metadata: 
  node_type: memory
  type: project
  originSessionId: 8bddde26-5f67-4ec4-b782-f0ea1f57b7a0
---

2026-07-11 gecesi itibarıyla bloke olmayan kod işi KALMADI (CUDA Faz B dahil
— [[girdap-decision-entegrasyon]]). Bundan sonrası doğrulama. Resmî prosedür:
`girdap-decision/docs/masa_testi_runbook.md` (M0-M8) + `docs/video_gunu_runbook.md`.
Bu memory "neden ve neye dikkat" katmanı. İlgili: [[girdap-ida-proje-durumu]],
[[bekleyen-girdiler-isaret]], [[sartname-ida-2026]].

## ✅ OTURUM 2 MASA REGRESYONU + M6 TAMAM (2026-07-14 öğleden sonra, Jetson)

test-plani.md Oturum 2'nin TÜM masa adımları geçti (batarya YOK — FC Jetson-USB
beslemeli, motor rayı güçsüz; pervane durumu bu yüzden kritik değildi):
- **M0 ✓** suite 265/2 → F-M.3 düzeltmesi sonrası **YENİ TABAN 267/2**.
- **M1 ✓ USB YOLU:** `fcu_url=serial:///dev/ttyACM0:57600` (USB-C tamirli, Pixhawk
  6C iki ACM açar, 0 kullanılır). ⚠️ IMU boot'ta 1 Hz gelir — `/mavros/set_stream_rate`
  (stream_id 0, rate 50) İSTENMELİ → **~39 Hz** (TELEM2'de 10.4'tü). Runbook'a işlenecek.
- **M4 ✓** arm → BOOT→ARM→BEKLEMEDE.
- **F-M.1 fix CANLI DOĞRULANDI ✓:** FC'ye 5-item görev push + fix YOK + start →
  FSM PARKUR1'e geçiyor AMA mission_manager "geçerli GPS fix yok — görev
  başlatılmıyor (F-M.1)" WARN'ıyla TUTUYOR; planning HAYATTA, OOM YOK.
- **M6a ✓ (F-M.3 bulundu+düzeltildi):** servis-KILL FCU'yu disarm ETMİYORDU
  (disarm yalnız bridge'in heartbeat yolundaydı; bridge mission/state KILL'i
  yalnız F14.3 geçidi için okuyordu). TDD: 2 test kırmızı→yeşil; düzeltme
  `_on_mission_state`'te KILL→`_trigger_kill()`. Canlı teyit: kill → FSM KILL +
  thrust [0,0] + FCU disarm (state topic 1 Hz → teyit birkaç sn gecikebilir).
  Runtime lokal commit `8050ceb` (repo emekli, push yok) → girdap-video `98b5386` PUSH'LU.
- **M6b ✓** kasıtlı disarm → "DISARM başarılı", sahte FAILSAFE YOK (F-M.2 fix gerçek FCU'da ilk teyit).
- **M6c ✓** MANUAL'da 12 sn sıfır GUIDED isteği (F14.3 doğru).
- **M6d ✓** USB çek → 5.1 sn'de "FAILSAFE — heartbeat kaybı → KILL" + FSM KILL + thrust [0,0].
- **M7 ön ✓** telemetry/grafik/local_map üçü de üretiyor. **OPS-1 uygulandı ✓** (clear + `waypoints: []`).
- **🟡 YENİ F-M.4 (T1, defterde):** fix'siz PARKUR1'de bridge 10 Hz "GUIDED mod
  isteği" spam'i (sahada fix ARM önce geleceği için video blokeri değil).
- **KALAN:** Oturum 3 açık alan (GPS fix → M5 tam GUIDED tetiği + M2 QGC/RFD + M7
  dolu + M8 D3) → suda prova → çekim. Tek donanım blokeri: **MicoAir LR868 telemetri çifti** kurulumu ("RFD868x" BAYAT, bkz. [[haberlesme-frekans-uyum]]) + RC bandı yasal banda çekilecek.

### ✅ SR0 → F-M.6 TDD DÜZELTİLDİ (2026-07-14 akşam) — canlı doğrulama boot provasında

Kök sorun kodda kanıtlandı ve YAZILIM tarafında çözüldü (FC paramlarına DOKUNULMADI):
köprü bağlantı kenarında `/mavros/set_stream_rate` (STREAM_ALL, **10 Hz**, oturumluk)
istiyor. Etki zinciri deftere yazıldı (F-M.6): 1 Hz → (a) Ekran-2 basamaklı,
(b) fusion `pose_timeout_s=1.0` bekçisi odom'u KESİYOR, (c) planning pozun yaşına
bakmadan 10 Hz MPPI koşuyor. **Alt sınır 5 Hz.** İstek yalnız USB/SERIAL0'ı etkiler
(868 telemetri ayrı port → QGC hattına yük binmez). Aynı oturumda B1/B2 (AUTO video)
de yazıldı → [[video-auto-donusu]]. Suite YENİ TABAN **282/2**; girdap-video `aedf6ae`.
⏳ KALAN: **BOOT PROVASI** (servis restart/reboot = sudo, Eyüp) → taze bağlantıda
Hz + tegrastats CPU + stamp jitter ölçümü. Servis şu an ESKİ kodla çalışıyor.
🧪 Test izolasyon kuralı: servis açıkken node testleri `ROS_DOMAIN_ID=77` gibi İZOLE
domain'de koşulmalı (42'de canlı yayın testin verisini ezer — sahte FAIL).

### 🔴 (ESKİ) AÇIK KONU: FC stream hızları (SR0) — karar ertelendi (Eyüp, 2026-07-14)
- Sorun: taze USB bağlantısında FC ~1 Hz yayınlıyor; bugün elle `set_stream_rate`
  (id 0, 50) ile 39 Hz alındı AMA istek GEÇİCİ — servis/boot yolunda kimse istemiyor.
  Etki: Ekran-2 grafikleri 1 Hz basamaklı + planning bypass pozu bayat (F8.2 bekçisi
  1.0 s eşikte sahte tetik riski) → VİDEO kalitesini doğrudan etkiler.
- Araştırma teyitli (2026-07-14): Pixhawk 6C TELEM portları 921600'e kadar çıkar,
  10 Hz probleme girmez; gerçek kısıt SERIALx_BAUD. USB'de (SERIAL0) bant kısıtı YOK
  → SR0_EXTRA1=25/POSITION=10/RAW_SENS=25/EXT_STAT=5/EXTRA2=10 güvenli reçete hazır.
- **Eyüp kararı: FC parametrelerine dokunulmadı ("onlara kalsın").** Plan: önce
  BOOT PROVASI (Jetson reboot → servis kendiliğinden kalkıyor mu + akış Hz ölçümü,
  dokunmadan). 1 Hz teyitlenirse ölçüm deftere yazılıp karar FC ekibine/Eyüp'e:
  ya SR0 yazılır ya açılışta bizim taraf ister (küçük kod, T0'da istenmez).
  SUDA PROVADAN ÖNCE ÇÖZÜLMELİ. Boot provası bu kayıt anında henüz KOŞULMADI.

## 🔥 AKŞAM MASA OTURUMU SONUÇLARI (2026-07-12 gece, commit `e70a5a4`)

QGC laptopu yoktu → her şey Jetson'dan MAVROS servisleriyle sürüldü.
Tam rapor: `docs/donanim_gunlugu_2026-07-12.md` §10 + runbook sonuç tablosu.

- ✅ **M3 GEÇTİ:** BOOT→ARM→BEKLEMEDE ("Throttle armed"). Yol açan işler:
  (a) emniyet düğmesi bulunamadı → `BRD_SAFETY_DEFLT=0` + FCU reboot
  (⚠️ sahada FC ekibi karar versin); (b) **RC kalibrasyonu hiç yapılmamıştı**
  → 60 sn kayıtla MIN/TRIM/MAX yazıldı (RC1 882/1650/2129, RC2 943/2137/2146,
  RC3 915/1075/2119 — trim=gerçek dinlenme; kumanda tuhaf: CH2 üst uçta
  dinleniyor, FC ekibi QGC'yle düzgün kalibrasyon+RCMAP kontrolü yapmalı).
  F14.7 retry+red davranışı gerçek FCU'da DOĞRU çalıştı (KILL basmadı).
- ✅ **M4 GEÇTİ:** mavros mission/push ile 5 item → FC → geri okuma →
  "FC görevi alındı: 5 item → 4 waypoint". skip_home_seq0 DOĞRU, komut=16,
  latched ✓ — T0-f'in tüm saha varsayımları teyit.
- 🟡 **M5 KISMİ:** ArduPilot GPS'siz GUIDED'ı REDDEDİYOR ("Flight mode
  change failed") — "M3-M6 fix'siz koşar" varsayımı M5 için YANLIŞTI.
  GUIDED-mod tetiği AÇIK ALANDA (GPS fix'li; su gerekmez) test edilecek.
  `/girdap/mission/start` ile görev başladı: BEKLEMEDE→PARKUR1 ✓.
- 🔴 **F-M.1 GERÇEK BUG:** görev başlayınca planning_node **92 GB cupy OOM**
  ile öldü (`mppi.py:466 _trajectory_cost`). Kök: GPS yok → home_ref (0,0)
  kaldı → 40°K/29°D wp'ler ~4400 km ENU → n_ref devasa → (K,T+1,n_ref)
  patladı. Sahada fix'le olmaz AMA korumasız: fix'siz görev başlatılabiliyor
  (F8.4'ün kod karşılığı yok) + n_ref/hedef-mesafe tavanı yok. **TDD düzeltme
  = sonraki oturumun 1. işi.**
- 🟠 **F-M.2:** kasıtlı `/girdap/bridge/disarm`'da "DISARM başarılı" SONRASI
  yine "FAILSAFE — beklenmedik disarm → KILL" bastı (görev aktif + planning
  ölüyken). F14.2 mock'ta geçiyordu; gerçek FCU'da bayrak zamanlaması/yarış
  incelenecek. Video güç-kesme provasını ilgilendirir.
- M6 koşulamadı (planning ölünce thrust hiç akmadı), M7 kısmi (boot üretimi ✓),
  M2 ertelendi (⚠️ QGC'nin ARM64 Linux sürümü YOK — Jetson'a kurulamaz,
  x86 laptop şart). Ham log: `~/girdap_logs/masa_testi/masa_stack_2026-07-12_aksam.log`.

## 🔴 OLAY (kapanış SONRASI, `eae9d9d`): FC sahte görevi KENDİ koştu

Yığın kapalıyken motorlar TAM GÜÇ döndü, güç kesilip verilince görev devam
etti — FC hafızasındaki M4 test görevi (40°K/29°D sahte, ~4400 km) RC/AUTO
üzerinden ARM olup koştu (muhtemel tetik CH5 mod kanalı; BRD_SAFETY_DEFLT=0
çıkışları açmıştı). Pervanesiz kural sayesinde hasar yok. **BİR SONRAKİ GÜÇ
VERİŞTE ZORUNLU (pervanesiz): (1) FC görevini SİL (/mavros/mission/clear),
(2) BRD_SAFETY_DEFLT=1 GERİ, (3) FC ekibi RC mod kanalı/FLTMODE incelemesi.**

## 🎯 BİR SONRAKİ OTURUM PLANI (revize: 2026-07-12 gece)

0. **ÖNCE OLAY TEMİZLİĞİ:** mission clear + BRD_SAFETY_DEFLT=1 (yukarıda).
1. **F-M.1 TDD düzeltmesi** (fix/home_ref-yokken görev reddi + n_ref/mesafe tavanı).
2. **F-M.2 incelemesi** + görev-aktifken kasıtlı disarm node testi.
3. **Açık alan mini-oturumu** (GPS görür yer, su gerekmez): GUIDED tetiği
   (M5 tam) → M6 KILL zinciri → M7 dolu kayıt → M8 D3.
4. QGC laptop (x86) temin → M2 + gerçek QGC Plan Upload provası.

## ESKİ PLAN (2026-07-12 gün sonu itibarıyla — kısmen uygulandı)

### Ön şartlar (oturum başlamadan hazır olsun)
1. **PERVANELER SÖKÜLÜ** — M4-M6 pazarlıksız şartı.
2. **RC kumanda bağlı/açık** — FCU PreArm "Radio failsafe on" basıyor; RC
   gelmeden ya da FC ekibi FS paramını ayarlamadan ArduPilot ARM ETMEZ.
3. **QGC kurulu laptop** (M2/M3/M5) + **MicoAir LR868 telemetri çifti** (M2; "RFD868x" bayat, [[haberlesme-frekans-uyum]]).
4. Pixhawk beslemesi (batarya) + **fcu_url=serial:///dev/ttyUSB0:57600**
   (USB-C soketi arızalı — FC ekibi tamir edene kadar TELEM2 yolu).

### Test sırası (masa runbook'la birlikte yürüt)
1. **M1 hızlı tekrar (5 dk):** stack aç (fcu_url override) → connected:true.
2. **M2 QGC↔RFD↔Pixhawk:** kablosuz YKİ hattı; MAVROS'la (TELEM2) birlikte
   yaşamalı — iki ayrı kanal.
3. **M3 görev yükleme:** QGC Plan 4 köşe Upload → `mission_source:=fc` →
   WaypointList latched geldi mi → çeviri logu. Saha varsayım teyitleri:
   ArduRover home=index0, komut tipi=16.
4. **M4 ARM:** `/girdap/bridge/arm` → BOOT→ARM→BEKLEMEDE (bugün ARM'a kadar
   geldi, armed=false'la durdu — RC şartı).
5. **M5 GUIDED tetiği:** QGC'den mod→GUIDED → "görev başlatıldı" logu
   (kenar tetikli: önce ARM sonra GUIDED).
6. **M6 KILL zinciri (EN KRİTİK):** kill→sıfır thrust+FCU disarm (F14.1);
   kasıtlı disarm≠failsafe (F14.2); manuel modda auto_guided sussun (F14.3);
   kablo çek→heartbeat KILL latch. FC'nin kendi FS paramları dökümü.
7. **M7 dolu kayıtlar:** görev koşusu sonrası CSV/PNG DOLU mu (boş-alan
   üretimi bugün doğrulandı) + `run_ekran2.py` gerçek veriyle.
8. **M8 gerçek D3:** MPPI gerçek görev zincirinde (bugünkü mock: 9.90 Hz)
   + tegrastats termal.

### Kapalı mekân kısıtı
GPS fix yok → EKF local pose yok → `fusion/odom` sessiz (bugün görüldü,
BEKLENEN). M3-M6 fix'siz de koşar; M7/M8'in dolu-veri kısmı ve gerçek odom
için açık alan/suda prova gerekir. F8.4: ARM'dan önce fix bekle.

### Dış girdi bekleyenler (oturumdan bağımsız kuyruk)
- USB-C soket tamiri (FC ekibi) → sonrasında USB'den ~50 Hz IMU'ya dönüş.
- NN Archive (video SONRASI üretilecek) → getClasses sınıf-sırası logu,
  letterbox `_LB_PAY`, Dosya-1 mp4 FPS etkisi, füzyonun GERÇEK kamera
  tespitiyle testi (bugünkü kanıt nişanlı-sahte bbox'laydı).
- Mekanik `h` → F5.1 (üreteç+testle AYNI commit) + gerçek duba haritası
  + min_range değerlendirmesi. `olcum_formu.md` dolu mu SOR.
- F-L.2 restamp kararı — duba testi günü.
- Suda prova (T0-h) → video çekimi (SON TESLİM 21.07 17:00 — ELEME).

## ⚠️ Genel uyarılar (her test gününde geçerli)

- **Pervaneler SÖKÜLÜ** — arm/KILL testleri motorlara gerçek komut basar.
- **F14 bulgularının HİÇBİRİ gerçek FCU görmedi** (mock `armed=True/GUIDED` ile
  test edildi). KILL→disarm (F14.1), kasıtlı-disarm≠failsafe (F14.2),
  auto_guided (F14.3) düzeltmeleri M4-M6'da İLK KEZ gerçek Pixhawk'ta koşacak
  — masa testlerinde bir şeyler çıkması NORMAL, çıkarsa TDD ile düzeltilir.
- Her FAIL'de runbook'un kendi FAIL satırına bak; yoksa log + `docs/kod_denetimi.md`.

## A) Masa testleri (M1-M8) — Pixhawk USB + RFD çifti + QGC laptopu gerekir

- ~~M0 yazılım sağlığı~~ ✅ geçti (suite 250/2, 2026-07-12).
- ⚠️ **M1 İLK DENEME BLOKE (2026-07-12):** Jetson'da ttyACM YOK; tek seri
  cihaz ttyUSB0 = FTDI FT231X (seri DU0EFEA7) ve 57600-921600 TÜM
  baud'larda SIFIR bayt (MAVROS connected:false, VER timeout). Teşhis:
  Pixhawk ya KAPALI ya bu FTDI'ın ucunda değil (FT231X büyük ihtimal RFD868x
  modemi — o QGC laptopuna gidecekti). Eyüp'ten fiziksel kontrol: Pixhawk
  güç LED'i, FTDI kablosunun iki ucu, Pixhawk USB-C→Jetson doğrudan kablo.
  Pixhawk USB'den takılınca ttyACM0 çıkmalı → `ros2 launch mavros
  apm.launch fcu_url:=serial:///dev/ttyACM0:57600` + /mavros/state probu.
- ✅ **M1 GEÇTİ (2026-07-12 öğleden sonra, TELEM2 üzerinden):** kablo
  çaprazlanınca hat canlandı — connected=True, mod HOLD; IMU stream-rate
  isteğiyle 10.4 Hz (57600 tavanı; ~50 Hz için USB tamiri ya da SR2).
  Tam stack gerçek FCU'yla: heartbeat KILL YOK, FSM BOOT→ARM ilerledi.
  fusion/odom kapalı mekânda 0 Hz (GPS fix yok → EKF pose yok, beklenen).
  fcu_url artık `serial:///dev/ttyUSB0:57600` (USB-C soketi hâlâ arızalı).
  ⚠️ PreArm "Radio failsafe on" — M4 öncesi FC ekibi RC'yi bağlasın/ayarlasın.
- ✅ **MPPI CANLI DÖNGÜ ÖN-KONTROLÜ GEÇTİ (M8'in sahte-beslemeli hâli):**
  gerçek planning_node + sahte odom/hedef/engel → thrust 9.90 Hz, medyan
  100.0 ms / p95 111 ms (ilk adım 733 ms = CUDA derleme). Gerçek D3 = M8.
- **M2 — QGC↔RFD868x↔Pixhawk:** kablosuz YKİ hattı (videonun 1. şartı,
  md 3.3.1/1). QGC, MAVROS'la AYRI kanal (RFD 868 MHz vs USB) — ikisi
  birlikte yaşamalı; `gcs_url=""` doğru, değiştirme.
- **M3 — Görev yükleme (T0-f zinciri):** QGC Plan'dan 4 köşe Upload →
  `mission_source:=fc` → `/mavros/mission/waypoints` latched geldi mi →
  `fc_items_to_waypoints` doğru çevirdi mi. SAHA TEYİDİ GEREKEN VARSAYIMLAR:
  ArduRover home=index0 (`skip_home_seq0` ters çıkarsa tek anahtar),
  komut tipi=16 (NAV_WAYPOINT), WaypointList'in latched gelmesi.
- **M4 — ARM + BEKLEMEDE:** `/girdap/bridge/arm` → FSM BOOT→ARM→BEKLEMEDE
  (armed + killswitch off şartı, fsm_node armed'ı /mavros/state'ten okur).
  F14.7'nin pre-arm retry'ı gerçekte tetiklenmemeli.
- **M5 — GUIDED-mod başlatma tetiği (T0-j):** QGC'den mod→GUIDED çevir →
  log: "YKİ mod komutu … görev başlatıldı". KENAR TETİKLİ: önce ARM sonra
  GUIDED; boot'ta zaten GUIDED ise BAŞLATMAMALI (gerekirse HOLD'a al, geri
  dön). Bu md 3.3.1(3) "tek komutla başlatma"nın ta kendisi.
- **M6 — KILL/disarm zinciri (EN KRİTİK GÜVENLİK TESTİ):**
  `/girdap/mission/kill` → sıfır thrust + FCU disarm (F14.1 düzeltmesi);
  kasıtlı `/girdap/bridge/disarm` → FAILSAFE ALARMI BASMAMALI (F14.2);
  manuel modda auto_guided kavga etmemeli (F14.3); heartbeat kesilince
  (USB çek) KILL latch. Ayrıca FC'nin KENDİ failsafe paramları
  (FS_ACTION vb.) FC ekibinden teyit — bizim yazılım ölürse son savunma o.
- ✅ **BOOT SMOKE + M7 ÖN-KONTROLÜ GEÇTİ (2026-07-12, FCU'suz):**
  hardware.launch Jetson'da tam kadro ayakta (10 node + mavros + 3 TF, ölen
  yok); Dosya-2 CSV (2 Hz, header birebir) + grafik CSV (10 Hz, thrust'lı) +
  local_map PNG (~1 Hz) boot'tan itibaren ÜRETİLİYOR. FCU'suz beklenen
  davranış: heartbeat-kaybı KILL'i basıyor (F14.4 latch — M6'da gerçek
  FCU'yla yeniden bakılacak). Asıl M7 dolu verilerle FCU sonrası.
  Günlük: docs/donanim_gunlugu_2026-07-12.md (`ec37f87`).
- **M7 — Kayıt dosyaları:** görev koşusu sonrası `~/girdap_logs/telemetry`
  (Dosya-2 CSV, header şartname birebir), `grafik` (Ekran-2 10 Hz),
  `local_map` (Dosya-3 PNG ≥1 Hz) DOLU mu; `run_ekran2.py` PNG/MP4 üretiyor
  mu. Boot'tan itibaren üretim — systemd yok, elle launch (bilinçli).
- ⚠️ MPPI durumu (2026-07-12): canlı kontrol döngüsünde HENÜZ koşmadı —
  boot smoke'ta planning_node ayaktaydı ama görev başlamadan compute_control
  MPPI step çağırmıyor. Bugünkü kanıt yalnız suite (kapalı-döngü + CUDA
  fused parite testleri Jetson GPU'da yeşil). Gerçek D3 ölçümü = M8, FCU şart.
- **M8 — Performans:** MPPI gerçek yığın içinde (bench değil, canlı node)
  control_rate tutuyor mu — Faz B sonrası beklenti RAHAT 10 Hz (step 9 ms);
  isterse 20 Hz denemesi. F8.3: iSAM2 graf/RAM büyümesi (yarışma 20 dk
  simülasyonu; video modunda isam2 kapalı, bu T1 ölçümü). tegrastats ile
  termal gözlem.

## B) Kamera masa testleri (OAK — donanım hazır, model beklemiyor olanlar)

- ✅ §2.6 modelsiz akış: 12.0 FPS + USB SUPER (USB-C YÖN tuzağı — rehber
  §5/10; MUHAFAZAYA ALMADAN ÖNCE `getUsbSpeed()` → SUPER teyidi ŞART).
  2026-07-12 tekrar teyit: SUPER + 11.9 FPS + kare net (headless test).
- ✅ **ORTAK YÜK TESTİ GEÇTİ (2026-07-12):** OAK 12 FPS + LiDAR sürücüsü +
  perception_lidar_node (10 Hz) AYNI ANDA — kamera FPS düşmedi, CPU ~%30
  (6 çekirdek), RAM 2.3/7.6 GB, 53°C, ~7.4 W. Orin çok rahat. Not:
  governor boşta çekirdekleri 729 MHz'e düşürüyor; gerçek koşuda yük
  clock'u yukarı iter, yine de `sudo jetson_clocks` sahada iyi pratik.
- **Letterbox `_LB_PAY=0.125` doğrulaması** (model gelince,
  `duba_kamera_test.py`): bbox'lar passthrough karesine oturuyor mu; kutular
  dikey kaymışsa kodda `_LB_PAY=0.0`. `/perception/buoys` sözleşmesinin
  (640×480 piksel) saha teyidi.
- **Dosya-1 kaydedici FPS etkisi:** `KAYIT_AKTIF` açıkken ekstra stream
  USB bandından yer — FPS düşüşü ölçülecek (şartname md 4.2: ≥1 Hz mp4
  zorunlu, 5 ceza puanı).
- **Sınıf sırası (🔴):** model Jetson'a gelince İLK İŞ `getClasses` logu —
  "Model sınıf sırası: [...]" satırı okunacak (0=Engel/1=Kenar bekleniyor,
  [[yolo-model-durumu]]); `_sinif_indeksleri_coz` isimden çözer ama LOG
  TEYİDİ pazarlıksız. Muhafaza arkası odak testi de bu döneme.

## C) LiDAR (Livox Mid-360) — iki aşamalı, tarih serbest

### ✅ AŞAMA 1 BİTTİ (2026-07-12, "her şey takılı" test günü)

- **Dün geceki gizemli ERROR ÇÖZÜLDÜ = F-L.1 GERÇEK BUG:** gerçek Livox
  PointCloud2 KARIŞIK dtype'lı (x/y/z/intensity float32 + tag/line uint8 +
  timestamp float64, point_step=26) → `read_points_numpy` field_names'ten
  bağımsız "tüm alanlar aynı tip" assert'iyle ölüyor. Üretim
  `perception_lidar_node` de aynı çağrıyı kullanıyordu → İLK gerçek mesajda
  node ölümü (Parkur-2 biterdi). TDD ile düzeltildi (`d9778fe` push'lu):
  `read_points`+`structured_to_unstructured`; test üreteci gerçek sürücü
  şemasını taklit ediyor. Suite YENİ TABAN **250/2**. Sentetikler
  `create_cloud_xyz32` kullandığından maskeleme deseninin yeni örneğiydi.
- **Ölçümler (Jetson, canlı Mid-360):** akış 10.00 Hz / ~20k nokta / IMU
  200 Hz; kümeleme üretim 38.6 ms / geniş-Z 52.3 ms medyan (F5.3 Jetson
  teyidi ✓); node canlı 9.98-10.07 Hz obstacle_map. Statik IP kalıcı
  görünüyor (enP8p1s0'da hazırdı). Sürücü headless: `ros2 run
  livox_ros_driver2 livox_ros_driver2_node` + xfer_format:=0 paramları
  (rviz launch'a gerek yok).
- 🟡 **F-L.2 REVİZE (canlı deneyle, 2026-07-12 öğleden sonra):** "sync hiç
  ateşlemez" YANLIŞTI — gerçek Livox + sahte buoys deneyi: sync 90/20 sn
  çıkış verdi (10 Hz yoğun lidar akışı slop içinde daima aday bulur).
  Gerçek etki: çiftler ~0.2 s zaman kaymalı (dönüşte bearing ~0.06 rad <
  tol 0.15) → DÜŞÜK öncelik; restamp kararı T1'de. **Eşleştirme canlı
  KANITLANDI:** gerçek kümeye nişanlı bbox 99/99 class "0" eşleşti,
  eşleşmeyenler class 99 korunuyor — bearing düzeltmesi (e66cb40) gerçek
  veriyle ilk teyit (`50004e9`).
- Kalan aşama-2: `h` ölçüsü gelince F5.1 + gerçek duba testi (değişmedi).

### Eski durum notu (2026-07-11 gecesi)

- ✅ **Config tuzağı düzeltildi:** `MID360_config.json` src kopyası 117.x'e
  düzeltilmişti ama launch'ın okuduğu INSTALL kopyası eski IP'lerde
  (192.168.1.5/1.12) kalmıştı → install'a cp ile senkronlandı, ikisi de
  host=192.168.117.50 / lidar=192.168.117.100. (Lidar flash'ı eski IP'de
  çıkarsa yedek plan: Jetson'a 192.168.1.5/24 ekle, 192.168.1.12'yi pingle,
  config'i o çifte geri hizala.)
- ✅ **`~/lidar_bench_gecici.py` yazıldı (GEÇİCİ, repoya girmez):** node
  yolunu birebir taklit (read_points_numpy→detect_obstacles), 100 mesaj,
  üretim + geniş-Z (z=[-10,10]) çift config, stamp Δ (F7.1) raporu.
  Smoke: rastgele 20k noktada Jetson 98 ms. Üretim config'de 0 engel
  atölyede NORMAL (F5.1 z-filtresi masada eler) — gerçek yük = geniş-Z.
- Test sırası verildi: statik IP (`sudo ip addr add 192.168.117.50/24 dev
  enP8p1s0`) → ping 117.100 → `source ~/ws_livox/install/setup.bash &&
  ros2 launch livox_ros_driver2 rviz_MID360_launch.py` → `topic hz` ~10 Hz
  → bench → `ros2 run girdap_decision perception_lidar_node` →
  kamera_goruntu_test.py ile ORTAK yük testi (tegrastats izle) →
  Pixhawk takılıysa mini M1+M7 (hardware.launch, /mavros/state, girdap_logs).
- ~~⏳ ERROR~~ ✅ ÇÖZÜLDÜ: ERROR = F-L.1 (yukarıda).

1. **`h` ölçüsü GELMEDEN yapılabilir:** veri akışı (`/livox/lidar` topic,
   Ethernet+güç), kümelemenin Jetson ms'si (F5.3 sonrası x86 53 ms →
   Jetson'da ölç), F5.4 böl-atma + F7.3 depth=1 canlı doğrulama, F7.1
   stamp sözleşmesi (Livox saat kaynağı vs Jetson saati — sync_watchdog
   WARN basıyor mu). Eyüp kararı 2026-07-11: bu aşama atölyede "öylesine"
   yapılacak, duba verisi bilinçli ertelendi.
   **Eski kurulum KONTROL EDİLDİ (2026-07-11):** `~/ws_livox` içinde
   livox_ros_driver2 DERLİ hazır; `.bashrc` yalnız ros2_ws source'luyor →
   gölgeleme YOK, kullanmak için `source ~/ws_livox/install/setup.bash`.
   Config (`MID360_config.json`): host=192.168.117.50, lidar=192.168.117.100
   → Jetson eth'e statik `192.168.117.50/24` ver, `ping 192.168.117.100`.
   ⚠️ LAUNCH SEÇİMİ: bizim perception_lidar_node PointCloud2 bekler →
   `rviz_MID360_launch.py` (xfer_format=0) KULLAN; `msg_MID360_launch.py`
   CustomMsg basar (xfer_format=1), node onu OKUMAZ.
2. **`h` GELDİKTEN sonra:** F5.1 düzeltmesi yazılır (lidar_height_m,
   üreteç+testlerle AYNI commit — F6.2 reçetesi; min_range de birlikte
   değerlendirilecek) → gerçek duba yerleştirip `/perception/obstacle_map`
   doğru merkez/yarıçap veriyor mu. `h` bilinmeden duba testi ANLAMSIZ
   (filtre dubaları eler, harita boş — Parkur-2 biterdi).

## D) Suda prova (T0-h) → video çekimi (son teslim 21.07 17:00, ELEME)

- 4 GPS köşe dikdörtgen: koordinat doldurma prosedürü CLAUDE.md'de; QGC'den
  yükle (dosyadan DEĞİL — md 3.3.1/2). `hardware.yaml` video modunda
  (use_isam2/rrt=false) teyit.
- **F8.4 operasyon notu: ARM'DAN ÖNCE GPS fix bekle** (ilk fix hareketteyken
  gelirse origin kayar).
- İzlenecekler: istemsiz hareket YOK (4 kök de kapandı: F11.1 warm-start,
  F12.1 sahte geçiş, F12.2 terminal, F8.1 twist — ama gerçek suda İLK KEZ),
  4. noktada TAMAMLANDI + sıfır thrust, güvenlik anahtarı gösterimi
  (md 3.3.1/4 FİZİKSEL şart), kapak/su almazlık (mekanik).
- Çekim: Ekran-1 QGC kaydı + Ekran-2 `run_ekran2.py --mp4` + Ekran-3 dış
  kamera; ≥720p, 2-5 dk, KESİNTİSİZ tek gösterim; YouTube liste dışı → KYS.
  RC bandı 2.4 GHz OLMAMALI, WiFi kapalı (`rfkill block` — çekimden önce).

## E) Test edilemeyecek / başka güne ait

- Parkur-3 hedef duba (T2 — model kararı bekliyor, bilinçli askıda).
- Upstream PR (arkadaş dönünce; tüm düzeltmeler fork'ta).
- Yarışma modu flag'leri (use_isam2/rrt=true) — video SONRASI dönemin işi;
  o gün F4.7 notuna bak (3 flag değişir).
