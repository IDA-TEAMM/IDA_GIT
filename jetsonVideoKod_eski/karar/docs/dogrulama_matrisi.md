# Doğrulama Matrisi — ne test edildi, ne bekliyor (2026-07-12 itibarıyla)

> Soru: "yazdığımız kodların hangisi gerçekten çalışıyor?" Seviyeler:
> **suite** = birim/entegrasyon testi (250 passed / 2 skip, Jetson) ·
> **bench** = ölçüm scripti · **canlı** = gerçek donanım/veriyle koştu ·
> **saha** = suda/görevde. Kaynaklar: docs/kod_denetimi.md +
> docs/donanim_gunlugu_2026-07-12.md + docs/masa_testi_runbook.md.

## ✅ Canlı donanımla KANITLI (2026-07-12)

| Bileşen | Seviye | Kanıt |
|---|---|---|
| LiDAR sürücü + `perception_lidar_node` | **canlı** | 10 Hz obstacle_map, 20k nokta; F-L.1 düzeltmesi sonrası |
| LiDAR kümeleme (F5.3 scipy+voxel) | **canlı bench** | 38.6/52.3 ms medyan @20k nokta — 10 Hz bütçesinde |
| OAK kamera akışı | **canlı** | UsbSpeed.SUPER, 11.9-12.0 FPS, kare net |
| Kamera-LiDAR füzyonu (sync+bearing eşleştirme) | **canlı*** | sahte-ama-nişanlı bbox 99/99 eşleşti; sync 90/20 sn; class-99 koruma ✓ (*kamera tespiti sahteydi — gerçeği model bekliyor) |
| Bearing işaret düzeltmesi (`e66cb40`) | **canlı** | gerçek küme bearing'iyle eşleşme — ilk gerçek-veri teyidi |
| iSAM2 poz füzyonu çekirdeği | **bench (Jetson)** | F8.3: 20 dk sim, 11.4k anahtar, flush p95 21 ms, RAM +30 MB |
| Tam stack launch kompozisyonu | **canlı (FCU'suz)** | 10 node + mavros + 3 TF ayakta, ölen yok |
| Dosya-2 telemetri CSV üretimi | **canlı (boş alan)** | boot'tan 2 Hz, header şartname birebir |
| Ekran-2 grafik CSV (10 Hz, thrust'lı) | **canlı** | 823+ satır; thrust=0 satırları GERÇEK (KILL sıfır-thrust) |
| Dosya-3 local_map PNG (≥1 Hz) | **canlı** | session dizininde ~1 Hz geçerli kare |
| `run_ekran2.py` (video montaj aracı) | **canlı** | BUGÜNÜN gerçek CSV'sinden 3 panel PNG üretti; NaN'da sahte sıfır yok |
| Heartbeat-kaybı KILL bekçisi | **canlı (kısmi)** | FCU'suz boot'ta KILL bastı — tetik çalışıyor; FCU'lu tam zincir M6'da |
| Ortak yük (LiDAR+kamera birlikte) | **canlı** | kamera FPS düşmedi; CPU ~%30, 53 °C, 7.4 W |

## 🟡 Suite/bench'te yeşil, CANLI DOĞRULAMASI BEKLEYEN

| Bileşen | Suite kanıtı | Canlı testi ne bekliyor |
|---|---|---|
| **MPPI** (CUDA fused, 9.0 ms step) | kapalı-döngü + parite testleri GPU'da yeşil; dünkü bench 9 ms | **M8**: canlı kontrol döngüsü — görev başlamadan step çağrılmıyor → FCU şart |
| MAVROS köprüsü + ARM/disarm/mod (F14'ler) | mock FCU testleri | **M1-M6**: Pixhawk (USB soketi arızası çözülünce) |
| Görev FC'den okuma (T0-f) | duck-typed node testleri | **M3**: QGC Plan Upload + latched WaypointList teyidi |
| GUIDED-mod başlatma tetiği (T0-j) | 4 çekirdek testi | **M5**: QGC'den gerçek mod değişimi |
| F-M.1 guard'ları (fix'siz/uzak-hedef görev reddi + n_ref tavanı) | 6 TDD testi (`dff52af`) | açık alanda fix'li normal başlatmanın etkilenmediği (M5) |
| F-M.2 düzeltmesi (kasıtlı disarm ≠ failsafe) | 3 node testi, masa logu kırmızıda üredi (`3931220`) | **M6/2**: gerçek FCU'da disarm sonrası tick'lerde KILL yok |
| FSM parkur geçişleri (F12.1/F12.2) | FSM testleri | görev koşusu (suda prova) |
| RRT* (F10.1/F10.2 düzeltmeli) | 6 deterministik test | T1 — videoda kapalı (`use_rrt=false`) |
| iSAM2 GERÇEK GPS/IMU ile | — | FCU + açık gökyüzü (videoda bypass) |

## 🔴 HİÇ test edilemeyen (girdi eksik)

| Bileşen | Eksik girdi |
|---|---|
| YOLO gerçek duba tespiti + sınıf sırası + letterbox `_LB_PAY` | NN Archive (video sonrası üretilecek — [[bekleyen_girdiler]] §B) |
| Dosya-1 kamera mp4 kaydedici (bbox overlay) | model (overlay için tespit gerekir) |
| F5.1 lidar_height_m + gerçek duba haritası | mekanik `h` ölçüsü (olcum_formu.md) |
| QGC↔RFD↔Pixhawk kablosuz hat (M2) | RFD çifti kurulumu + QGC laptop + Pixhawk |
| Suda davranış (istemsiz hareket 4 kökü, temiz duruş) | suda prova (T0-h) |

## 🎯 Bir sonraki oturum planı (gün sonu güncellemesi)

**GÜN SONU NOTU:** M1 öğleden sonra GEÇTİ (TELEM2 kablo çaprazlaması —
`fcu_url:=serial:///dev/ttyUSB0:57600`, connected=true, IMU 10.4 Hz) ve
MPPI gerçek planning_node içinde sahte beslemeyle 9.90 Hz tuttu (günlük
§8-9). Sıradaki oturum:

1. **Ön şart:** pervaneler SÖKÜLÜ · RC kumanda bağlı (PreArm "Radio
   failsafe on" çözülmeden ARM olmaz) · QGC laptop + RFD çifti ·
   fcu_url=ttyUSB0:57600 (USB-C soketi tamirde).
2. **Sıra:** M1 hızlı tekrar → M2 (QGC↔RFD) → M3 (QGC'den görev upload,
   `mission_source:=fc`, home=index0/komut=16 teyidi) → M4 (ARM→BEKLEMEDE)
   → M5 (GUIDED tetiği) → **M6 KILL zinciri (en kritik: F14.1/2/3 ilk kez
   gerçek FCU'da)** → M7 (dolu kayıtlar + run_ekran2) → M8 (MPPI gerçek D3
   + tegrastats).
3. **Kapalı mekân kısıtı:** GPS fix yok → fusion/odom sessiz (bugün
   görüldü, beklenen); M7/M8 dolu-veri kısmı açık alan ister. F8.4: ARM'dan
   önce fix bekle.
4. **Ayrı kuyruk:** USB-C soket tamiri (FC) · NN Archive (video sonrası:
   sınıf sırası + letterbox + Dosya-1 FPS + gerçek füzyon) · mekanik `h`
   (F5.1 + duba haritası + min_range; olcum_formu.md) · F-L.2 restamp
   kararı · suda prova → video (21.07 17:00 ELEME).

## Bugünün açık maddeleri

- **Pixhawk USB-C soketi arızalı şüphesi** (descriptor -32) + TELEM2 kablo
  düzeltmesi — FC ekibinde (günlük §2'de reçeteler).
- F-L.2 (Livox stamp +0.2 s): sync ateşliyor, etki yalnız zaman kayması —
  düşük öncelik, restamp kararı T1.
