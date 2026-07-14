---
name: ida-git-gomulu-revizyon
description: "Gömülü ekibin IDA_GIT yığını kendi kopyamızda (EyupEker1/IDA_GIT) 20 fazlı denetim/revizyonda — upstream DOKUNULMAZ; Faz 1-18 + 20-PC bitti, kalan yalnız Faz 19 (Jetson gerçek cihaz) + rapor iletimi"
metadata: 
  node_type: memory
  type: project
  originSessionId: 614f4432-b279-4bb2-abdc-48995516a1c1
---

Eyüp'ün isteği (2026-07-13): gömülü ekip arkadaşının reposu **IDA-TEAMM/IDA_GIT** (public,
"embedded-software-team") başka repoya alınıp, **arkadaşın reposuna hiç dokunmadan**, bizim
kodlara ([[girdap-ida-proje-durumu]] + [[girdap-decision-entegrasyon]]) ve **şartnameye**
([[sartname-once-kural]]) göre revize edilip Jetson Orin Nano'ya kurulacak. girdap-decision
denetim metodolojisinin aynısı, **20 faz**.

## Mekanizma (2026-07-13 kuruldu)
- Kopya: **github.com/EyupEker1/IDA_GIT (private)** — tam geçmişle push'landı.
- Lokal klon: **/home/eyup/IDA_GIT** — `origin`=EyupEker1 (çalışma), `upstream`=IDA-TEAMM
  (salt okunur; ASLA push edilmez). Git kimliği lokal: Team GIRDAP (Algi).
- Bulgu dokümanı: **`docs/kod_denetimi.md`** (20 faz planı + bulgular, 🔴🟠🟡⚪ + dosya:satır
  + şartname maddesi; her faz sonunda commit+push origin). Faz 1 commit: `804f5f1`.

## Yığın ne (upstream @ `938580b`, 2026-07-12)
11-node ROS2 Humble pipeline, **Docker container** (`ros2_final`) içinde, sim→donanım geçişi:
sürücüler (gps_imu=MAVROS köprüsü, oakd depthai **v2**, livox ham UDP) → sensor_node →
perception (ultralytics YOLO `/root/best.pt` + bbox-içi HSV renk) → decision (sabit
WAYPOINTS + kaskad PID + görüntü-uzayı kaçınma → /cmd_vel) → control (/mavros/
setpoint_velocity, RC ch8 kill / ch5 manuel) → 3 kayıt node'u (Dosya-1/2/3, hepsi /tmp'ye).
Aktif paket `src/ida_topics_yeni` ≈2057 satır; `src/girdap_yenimodel` = tekne URDF/mesh.
**SIFIR test** (tek "test" py_compile). Takım 989124 Alt Alan B (aynı takım, gömülü ekip).

## Faz 1 ✅ bulgu özeti (V1-V10, ayrıntı docs/kod_denetimi.md)
- 🔴 **V1 sürüm/ortam:** depthai **v2** API (bizim Jetson 3.7.1 v3 — aynı python ortamında
  bir arada OLAMAZ) · ultralytics→numpy≥2 ([[jetson-surum-pinleri]] 1.26.4 pinini kırar) ·
  Docker varsayımı (bizim kurulum çıplak) · `ROS_DOMAIN_ID=42` (bizim yığın default domain
  → birbirini göremez).
- 🔴 **V2 OAK sahipliği:** oakd_driver cihazı doğrudan açıyor — bizim navigatorla USB
  exclusive çakışması (girdap-decision F3.1'in birebiri).
- 🔴 **V3 LiDAR 2D dilim SENSÖR çerçevesinde** (F5.1 deseni): /lidar/scan z∈[−0.15,+0.15]
  sensörden; Livox h yüksekte → duba dilim dışı → local_map/Dosya-3 boş (5 ceza). 3D bulut
  karar yolunda hiç kullanılmıyor — kaçınma yalnız kamera bbox. (Eyüp'ün "livox 3d ama
  burda 2d" tespiti doğru.)
- 🔴 **V4 görev kaynağı:** sabit Gazebo WAYPOINTS — md 3.3.1(2)+5.5.2.2 YKİ'den yükleme yok
  (bizim mission_source:=fc çözümü var).
- 🟠 V5 renk kanalı takası (oakd bgr8 yayını + perception RGB2BGR Gazebo varsayımı → gerçek
  kamerada turuncu/sarı HSV bozuk) · 🟠 V6 Dosya-1/2/3 /tmp'ye (reboot=kayıt kaybı, md 4.2)
  · 🟠 V7 RC kill DISARM etmiyor (F14.1 aynısı) · 🟡 V8 ileri hız `linear.y=-0.5` (Gazebo
  eksen kalıntısı) · 🟡 V9 /perception/objects'e yalnız turuncu.
- 🟡 **V10 DEĞERLİ donanım gerçekleri** (Faz 14'te bizim repolara taşınacak): Livox IP
  192.168.117.100, veri portu 56301 (tcpdump'lı), host 192.168.117.50/24; F9P yalnız
  Pixhawk GPS1 (bağımsız UART yok → GPS/IMU tek yol MAVROS); **Pixhawk ttyACM0 doğrudan
  USB ÇALIŞIYOR** (bizim "USB-C soketi arızalı" şüphesiyle çelişki — çapraz teyit);
  RC ch8 kill / ch5 mod.

## Entegrasyon fırsatları (Faz 2 girdisi)
Livox UDP driver → `/livox/lidar` remap köprüsü (livox_ros_driver2 gelene kadar; PointField
şeması Faz 6'da, F-L.1 dersi) · URDF sensör ofsetleri → [[bekleyen-girdiler-isaret]] §A
base_link/h sorularına CAD cevabı (Faz 13) · RC kill izleme deseni → bizim KILL zincirine
T1 güçlendirmesi · kamera_kayit bizim Dosya-1 kaydediciyle ÇİFT — tekilleştirilecek.

## Faz planı (20): 1✅ harita+hijyen · 2 rol/çakışma matrisi (EYÜP KARARI) · 3 bağımlılık ·
4-12 node denetimleri (script, oakd+perception, livox, gps+sensor, decision, control,
telemetri, kamera_kayit, local_map) · 13 URDF ofsetleri · 14 doküman+donanım gerçekleri
taşıma · 15 sözleşme hizası · 16 kayıt tekilleştirme · 17 test altyapısı (0→pytest) ·
18 Jetson uyumu (Docker'sız/v3/numpy/domain) · 19 Jetson kurulum+masa · 20 kapanış
(CI+upstream raporu+matris).

**Öncelik:** T1 işi — videoyu (21.07, girdap-decision yığını) bloke ETMİYOR.

## Faz 2 ✅ KARAR (Eyüp onayı 2026-07-13, commit `e107196`+`d52c22a`)
Çakışma matrisi A-E dokümanda: A donanım sahipliği (OAK cihazı+depthai v2/v3 pip çakışması;
Livox UDP portu) · B komut otoritesi (çift dümen; **B2: onların control_node RC ch5
sonrası auto-GUIDED çağrısı bizim start_on_mode=GUIDED tetiğini İSTEMEDEN ateşleyebilir**;
auto_arm; kill≠disarm; iki poz referansı) · C Dosya-1/2/3 çifte üretim (onlarınki /tmp +
siyah-frame/sıfır-setpoint kusurlu) · D ortam (numpy2/depthai2, domain 42, çift MAVROS,
**sistem_baslat.sh:31 pkill -9 mavros_node BİZİM MAVROS'u da öldürür**) · E fırsatlar.
**KARAR: birincil yığın = girdap.** YAŞAYAN: E1 livox UDP köprüsü (geçici, /lidar/points→
/livox/lidar remap) · E3 RC ch8 kill deseni (bizim köprüye) · E4 URDF · E5 donanım
gerçekleri. EMEKLİ (silme yok, dosya başına not): oakd_driver, perception, decision,
control, telemetri, kamera_kayit, local_map, sensor_node, gps_imu_driver.

## Faz 3 ✅ (V11-V17, commit `d52c22a`)
V11 package.xml eksik beyanlar (mavros_msgs/diagnostic_msgs/opencv/numpy — F2.1 deseni) ·
V12 requirements.txt yok + pyserial BAYAT (kod serial kullanmıyor) + pip kurulumları
container yazılabilir katmanında uçucu · V13 emekli katman Jetson'a KURULMAYACAK; yaşayan
livox_driver ayak izi rclpy+sensor_msgs+numpy = Jetson pinleriyle uyumlu ✓, domain 42
kullanılmayacak · V14 setup.py entry_points 4/10, livox_driver YOK (Faz 15'te eklenecek) ·
V15 config_ekf.yaml ölü · V16 ros2.repos 104 repo Jetson'da gereksiz · V17 dev Python 3.14
izi vs hedef 3.10 (py_compile 3.10 ✓).

## Blok B (Faz 4-12) ✅ (2026-07-13, commit `d9c837b`) — V18-V38
- 🔴 **V25 EN KRİTİK — livox_driver (YAŞAYAN köprü) paket ayrıştırması protokole aykırı,
  BİRİNCİ KAYNAK TEYİTLİ** (livox-wiki-en Mid-360 protokolü): başlık 36 bayt (kod 28
  atlıyor — timestamp ortasından başlıyor, +8 bayt kayma) VE data_type=1 noktaları
  **int32 mm** (kod `'<fff'` float32 okuyor) → köprü bu haliyle ÇÖP bulut üretir.
  Upstream'in "tcpdump doğrulandı"sı yalnız IP/port içindi. Düzeltme Faz 15 TDD:
  sentetik protokol paketi → kırmızı → offset 36 + `'<iii'` + dot_num/data_type doğrulama.
- 🔴 V22: HSV bantları (turuncu [10,34], sarı [35,55]) RAL 1026 sarıyı (H≈30) TURUNCU
  sınıflar (F4.5 analiziyle) · V18 kill -9 kapanış (ARM'da durdurma yok, GUID_TIMEOUT'a
  kalıyor) · V32 decision'da MIN_GECIT/geçit sayacı/parkur geçişi YOK (Plan-C sınırı).
- 🟠 V19 loglar her koşuda siliniyor+/tmp · V20 sağlık kontrolü yok · V34 RC kill
  fail-open + **E3 portlama reçetesi dokümanda** (mavros_bridge'e /mavros/rc/in, ch8 →
  kill zinciri+disarm) · V38 local_map Python ray-trace bütçesi.
- 🟡 V23 best.pt repoda yok/conf=0.15/hypothesis boş · V26 stamp=now() köprü notu ·
  **V27 PointCloud2 şeması bizim F-L.1 sonrası read_points ile UYUMLU ✓ (tek iş remap
  /lidar/points→/livox/lidar)** · V29 timeout ilk mesaja kadar kör + ölü IMU kontrolü ·
  V33 çok-wp atlama · V35 OverrideRCIn ölü kod · V36 telemetri alanları md 4.2 ✓ ama
  /tmp+1Hz · V37 siyah kare+bayat bbox MP4'e giriyor.

## Faz 13-15 ✅ (2026-07-13, IDA_GIT son commit `ed9a96d`)
- **Faz 13 URDF:** thruster eksenleri arası **0.594 m KESİN** (MPPI/karışım girdisi);
  diğer ofsetler DÜŞÜK GÜVEN (SolidWorks origin keyfî, COM çelişkili, kütle 8.76 kg
  gerçekdışı). En tutarlı yorum: LiDAR tekne ortası, güverteden ~0.16 m (⚠️ su hattından
  h DEĞİL — fribord eklenmeli). girdap-ida-algi `olcum_formu.md` §7'ye CAD-referans bloğu
  eklendi (eksen teyidi + fribord sorusu; commit push'lu).
- **Faz 14 transfer:** girdap-decision `docs/donanim_gercekleri_gomulu_ekip.md` (YENİ,
  push'lu fork+yedek): Livox ağ (IP 192.168.117.100, port 56301, host 192.168.117.50/24 —
  livox_ros_driver2 config'ine), F9P tek-konnektör→GPS/IMU tek yol MAVROS (fusion ✓),
  **Pixhawk ttyACM0 ÇALIŞIYOR gözlemi bizim USB-C-arızalı şüphesiyle ÇELİŞİYOR (çapraz
  test)**, RC ch8 kill/ch5 mod (OLAY'daki şüpheli kanal!). CLAUDE.md bayatları kayıtlı
  (pyserial, depthai2, domain 42 gerekçesi).
- **Faz 15 KÖPRÜ DÜZELTMESİ (V25, TDD):** `ida_topics/livox_protokol.py` saf modül +
  `test/test_livox_protokol.py` 5 test — eski mantıkla 4 KIRMIZI kanıtlandı → protokol
  düzeltmesi (başlık 36, `'<iii'` int32 mm, dot_num/data_type doğrulama, taşma koruması)
  → 5 YEŞİL; node delegasyonu çift import yollu; py_compile tümü ✓. V14 entry_point +
  V11 package.xml 4 beyan eklendi. **Köprü komutu:** `ros2 run ida_topics
  livox_driver_node --ros-args -r /lidar/points:=/livox/lidar` (bizim domain'de).
  Gerçek cihaz teyidi Faz 19'da.

## Doğrulama güçlendirme (2026-07-13, Eyüp "hızlı mı gidiyoruz" — commit `b7a310c`)
Eyüp'ün sorgusu haklıydı; kapatılan boşluklar: **e2e node testi**
(test_livox_driver_node.py: gerçek node+UDP soketi+PointCloud2 değer doğrulaması, 6/6) +
**entegrasyon koşusu** (köprü remap'li + GERÇEK girdap perception_lidar_node birlikte:
sentetik duba (5,2) → obstacle_map (4.99, 1.98) BAŞARILI; QoS RELIABLE→BEST_EFFORT ✓) +
V22 sayısal teyit (RAL 1026 H=31.0 → onların turuncu bandı [10,34] içinde).
kod_denetimi.md'ye "DOĞRULAMA DURUMU" bölümü: testli / statik / donanım-bekleyen /
bilinçli-testsiz ayrımı + ⚠️ sentetik üreteç maskeleme riski notu (F5.1/F6.1 dersi —
üreteci protokol dokümanından BİZ yazdık, test ve kod aynı yorumu paylaşıyor) →
**gerçek cihaz teyidi (Faz 19) PAZARLIKSIZ.**

## 📋 DURUM + KALAN İŞLER (yeni oturum buradan devam eder)

**Durum (2026-07-13 akşam):** Faz 1-18 + Faz 20'nin PC kısmı BİTTİ. IDA_GIT son commit
**`11f43f4` push'lu**, working tree temiz. Test tabanı: **8 passed** (ROS'lu; ROS'suz
7+1 skip; komut: `source /opt/ros/humble/setup.bash && python3 -m pytest
src/ida_topics_yeni/test/ -q`). **CI Actions'ta YEŞİL (7 passed / 1 skipped).**

### Faz 16 ✅ (`375b2f1`)
9 emekli node docstring EMEKLİ blokları · sistem_baslat.sh + config_ekf uyarıları ·
CLAUDE.md ROL KARARI bölümü. Doğrulama: py_compile + bash -n + yaml load + suite.

### Faz 17 ✅ (`ff2c18e`)
Yaşayan köprü testleri: çöp-bayt sağlamlık (recv sözleşmesi, deterministik seed) +
20k nokta parse hız smoke'u (**x86 5.5 ms** — 10 Hz bütçe içinde; Jetson kıyası Faz 19).
V28 buffer testi BİLİNÇLİ atlandı (node refaktoru > getiri, dokümanda gerekçeli).

### Faz 18 ✅ (`c0b6793`)
`docs/jetson_kopru_kurulum.md`: klon ~/IDA_GIT (ros2_ws DIŞINA, V16) · derlemesiz Yol A +
opsiyonel colcon Yol B · ros2.repos + pip YASAK (numpy altın kuralı) · domain export yok ·
statik IP 192.168.117.50/24 · köprü komutu + Faz 19 doğrulama listesi · köprünün geçiciliği.

### Faz 20 PC kısmı ✅ (`11f43f4`, 2026-07-13)
- **CI kuruldu + Actions YEŞİL:** `.github/workflows/ci.yml` (girdap-decision F16.4
  deseni; py3.10, tek bağımlılık pytest; e2e rclpy-skip; exit-5 yanlış-yeşil koruması).
- **Upstream raporu taslağı:** `docs/upstream_raporu.md` — arkadaşa iletilecek mesaj
  metni hazır (V25 diff + patch teklifi, V22, V1 ortam, kısa liste, teşekkür; PR
  AÇILMAZ) + iletim kontrol listesi. **İletim Eyüp'ün elinde; Faz 19 sonucu gelirse
  rapora bir cümle eklenecek.**
- CLAUDE.md test tabanı 6→8 güncellendi.

### Faz 19 — JETSON'DA gerçek cihaz teyidi (donanım başında!)
1. Üç repo git pull + IDA_GIT'i Jetson'a klonla (gh auth hazır).
2. **Livox'u İLK KEZ tak** (ethernet, statik IP) — FC'siz yapılabilir (yalnız
   Jetson+LiDAR; güç güvenlik bloğu FC'ye güç verilecekse geçerli: OLAY aksiyonları,
   girdap-decision masa runbook M0-öncesi bölümü — [[girdap-decision-entegrasyon]]).
3. Köprüyü koş → `ros2 topic echo /livox/lidar --once`: değerler MAKUL MÜ (menzil
   0.1-25 m, çöp yok)? Gerçek paketin data_type=1 ve dot_num'u logla. **Bu, V25'in ve
   sentetik-üreteç-maskeleme riskinin TEK gerçek kapanışıdır.**
4. Bizim perception_lidar_node ile birlikte → obstacle_map'te oda duvarları/nesneler
   makul mü. 5. pytest suite Jetson'da (beklenen 6/6). 6. Fırsat varsa Pixhawk ttyACM0
   çapraz testi (gömülü ekibin kablosuyla — çelişki dokümanda).

### Faz 20 — KALAN kapanış (Faz 19 SONRASI)
CI ✅ + rapor taslağı ✅ (yukarıda). Kalan: (1) Faz 19 sonucunu `docs/upstream_raporu.md`'ye
işle + Eyüp raporu arkadaşa iletsin; (2) kod_denetimi.md "DOĞRULAMA DURUMU" bölümünün
donanım-bekleyen maddelerini kapat; (3) bu memory'yi güncelle + memory repo push.

### Bağlam hatırlatmaları
- **ÖNCELİK HÂLÂ T0 VİDEO (21.07!):** bu IDA_GIT işi T1 — video işlerinin (suda prova,
  çekim, FC OLAY aksiyonları, M5-M8) önüne GEÇMEZ. Bkz [[girdap-decision-entegrasyon]].
- olcum_formu (§7 CAD bloğu eklendi) hâlâ mekanik/FC ekibine GÖNDERİLMEDİ ([[bekleyen-girdiler-isaret]]).
- Her memory güncellemesinden sonra: memory dizininde `git add -A && git commit && git push`
  ([[memory-repo-yedek]]).
