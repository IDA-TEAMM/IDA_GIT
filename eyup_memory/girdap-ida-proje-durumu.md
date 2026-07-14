---
name: girdap-ida-proje-durumu
description: "GİRDAP İDA algı projesi — teknik gerçekler + 2026-07-09 MPPI revizyonunda yapılan her şey (kod, rehber, doğrulamalar, push 4bbca23)"
metadata: 
  node_type: memory
  type: project
  originSessionId: 56c60e89-258c-4a9b-b1b6-606d940bac0a
---

TEKNOFEST 2026 İDA (insansız deniz aracı) projesi, repo: [[girdap-ida-algi-github-upload]] (`/home/eyup/girdap-ida-algi`, private, main branch).

## Kesinleşen teknik gerçekler (Eyüp teyit etti, 2026-07-09)

- ~~Takım Nav2 `nav2_mppi_controller` kullanıyor, arayüz `/goal_pose`~~ **BAYAT (2026-07-10'da çürütüldü):** arkadaşın gerçek mimarisi Nav2 değil, kendi RRT*+MPPI stack'i; arayüz `/perception/buoys` + `/perception/gate_passed` — ayrıntı ve faz durumu: [[girdap-decision-entegrasyon]]. Plan A artık `MOD="algi_yayin"`; Plan B (`dogrudan_surus`) saha yedeği olarak duruyor. Tek dümen kuralı geçerli.
- **MPPI/karar tarafını Eyüp DEĞİL, takım arkadaşı yazıyor** (repo: vistastris/girdap-decision).
- **YOLOv11n @ 416×416** — giriş boyutu NN Archive içinde tanımlı, kodda ayarlanmaz. ⚠️ **DİKKAT:** `yolo11n_duba_rvc2.tar.xz` bu makinede YOK; sınıf sırası muhtemelen TERS. Bkz. [[yolo-model-durumu]] — sahaya çıkmadan okunmalı.
- Saha ölçümü: **10–14 FPS bandı, tipik ~11.6** (YOLO + stereo birlikte = OAK-D Lite VPU sınırı). Kodda `FPS=12` (sensorFps) bilinçli; 14'e çıkarma — VPU doymuş, kuyrukta bayat kare birikir.
- **DepthAI v3 API** (v2 desenleri çalışmaz). Lokalde 3.6.1 kurulu, PyPI güncel 3.7.1 (aynı v3 API).

## 2026-07-09 oturumunda yapılanlar (commit 4bbca23, push edildi)

**1. API doğrulaması (koddan önce):** Koddaki TÜM DepthAI çağrıları kurulu 3.6.1 paketine karşı üye/imza düzeyinde doğrulandı — `Camera.build(boardSocket, sensorFps=)`, `SpatialDetectionNetwork.build(cam, stereo, nnArchive, resizeMode=LETTERBOX)`, `requestOutput((640,400))`, `createOutputQueue`, `getClasses`, `initialControl.setManualFocus`, `Pipeline.start/stop/isRunning` — hepsi mevcut. Nav2 MPPI parametreleri Humble branch resmi README'sinden teyit edildi (nottaki YAML doğrulanmış değerler).

**2. Kod revizyonu (`duba_gecis_navigator.py`):**
- Önceki oturumdan working tree'de commit'lenmemiş duran revizyon dahil edildi: Plan A geçiş sayacı artık **odometriyle** doğrulanıyor (geçit çizgisi odom'a sabitlenir, araç çizgiyi gerçekten aşınca sayılır; zaman sadece son-çare aşımı) — MPPI'nın hızı bizden bağımsız olduğu için zaman tahmini yanlış sayıyordu. Ayrıca: arama hedefi son görülen tarafa 0.30 rad açılı; OAK-D Lite AF varyantı için opsiyonel `RGB_SABIT_FOKUS`.
- 416×416 ve FPS gerekçeleri koda yorum olarak işlendi.
- **Canlı NN FPS ölçümü eklendi:** durum logunda sürekli `NN x.x FPS`, `FPS_UYARI_ESIK=8` altında uyarı. `py_compile` geçti.

**3. `docs/mppi_entegrasyon_notu.md` tamamen yeniden yazıldı** (59 satır → 10 bölümlük rehber, arkadaş bunu okuyarak sıfırdan kuracak): uçtan uca mimari diyagram + sorumluluk tablosu; `/goal_pose` sözleşmesi (0.5 m / 2 sn yayın filtresi, arama davranışı, 25 sn güvenlik kesmesi); algının MPPI'yı ilgilendiren gerçekleri (FPS'in kontrol frekansından bağımsızlığı, gecikme bütçesi, YOLO tespitlerinin costmap'e BİLEREK girmediği — kaçınma MID-360 LiDAR'dan); kopyala-yapıştır costmap + nav2_mppi YAML; **inflation hesabı: 1.35 m net açıklık vs 0.75 m tekne → inflation_radius 0.40–0.45, en kritik ayar**; cmd_vel→MAVROS remap köprüsü (BODY_NED şart); Orin/ARM benchmark uyarısı; 4 basamaklı test merdiveni; sorun giderme tablosu; §10'da arkadaştan beklenen 4 cevap (base_link orijini, odom/map, controller Hz, LiDAR topic adı).

**4. README:** "Algı gerçekleri" bloğu (416×416, 10–14 FPS, DepthAI v3) + rehbere yönlendirme + `depthai>=3.6` kurulum şartı. `models/README.md`'ye benchmark satırı.

## Jetson kurulum rehberi + kur.sh düzeltmeleri (2026-07-11, commit `2fcef4d` push'lu)

- **`docs/jetson_kurulum_rehberi.md`** yazıldı: sıfır Jetson → çalışır yığın (gh auth device flow, ROS Humble, jetson_kur.sh, karar bağımlılıkları, jetson_kontrol.sh, kamera kodu çalıştırma, sorun giderme). Yerleşim: **tek workspace `~/ros2_ws`, iki repo `~/ros2_ws/src/` altında** — girdap-decision'daki masa_testi_runbook da bu yerleşime hizalandı (`52fd876`).
- **jetson_kur.sh düzeltmeleri:** (1) REPO_KARAR vistastris→**EyupEker1 fork'u** (eski URL bayat kod kurardı!); (2) WiFi/BT kapatma klon/derleme SONRASINA taşındı (4b→5b — eski yerinde WiFi'li kurulumda kendi internetini kesiyordu; Eyüp "neden sildin" diye sordu, taşındığı gösterildi); (3) private klon hatasına gh auth yönlendirmesi; (4) bashrc'ye karar PYTHONPATH satırı.
- Model gerçeği değişmedi: `MODEL_NNARCHIVE=/home/girdap/models/yolo11n_duba_rvc2.tar.xz` (kullanıcı adı `girdap` değilse satır 97 güncellenmeli); tar.xz üretimi video sonrası ([[yolo-model-durumu]]).

## HubAI rehberi + geçici kamera testi + kod-dışı liste (2026-07-11, commit `fb621f6` push'lu)

- **`docs/hubai_model_rehberi.md`:** Gazebonew.pt → RVC2 NN Archive adım adım. Resmî Luxonis dokümanından teyitli (2026-07-11): **.pt DOĞRUDAN yüklenir (ONNX gerekmez)**, `hubai-sdk` `client.convert.RVC2(path=..., yolo_version="yolov11", yolo_input_shape=[416,416], yolo_class_names=["Engel Dubasi","Kenar Dubasi"], number_of_shaves=6)`; hub.luxonis.com hesabı+API key gerek; çeviri PC'de yapılır. Doğrulama pazarlıksız: `tar -xJf … -O config.json` → classes sırası 0=Engel/1=Kenar, n_classes=2. Eski tools.luxonis.com legacy.
- **`scripts/kamera_goruntu_test.py` — ⚠️ GEÇİCİ, SİLİNECEK:** modelsiz dönemde kamera/USB/udev testi (yalnız RGB+FPS, YOLO yok). **Eyüp'ün açık isteği: asıl kodla karışmasın → silme görevi `bekleyen_girdiler.md` §B/5'e işlendi, dosya başlığında da GEÇİCİ yazıyor. Model Jetson'a konduğu gün SİL.** py_compile ✓, gerçek koşu Jetson'da olacak.
- **`docs/kod_disi_ihtiyaclar.md`:** kod hariç eksikler tek sayfa — NN Archive + .pt yedeği + HubAI/YouTube/KYS hesapları; video donanımı (RFD çifti, QGC+OBS laptop, RC bandı 2.4 GHz OLMAMALI, güvenlik anahtarı, dış kamera); yarışma donanımı (OAK USB3 kablo, Livox, muhafaza, USB bellek); ölçümler (Livox `h`, base_link, ArduPilot failsafe paramları, Gazebonew.pt eğitim verisi sorusu).
- Eyüp'ün kafa karışıklığı netleştirildi: "py dosyamız yok" değil — py'lar repoda (duba_kamera_test.py, navigator); eksik olan MODEL tar.xz (YOLO VPU'da koştuğu için ayrı py'ı yok).

## 🚀 JETSON KURULUMU FİİLEN BAŞLADI (2026-07-11 akşamı)

Eyüp Jetson'un başında (`girdap@ubuntu`, JetPack r36.5/jammy). Durum:
- ✅ gh kurulu + **token ile giriş yapıldı** (browser akışı "context deadline exceeded" verdi; PC'deki `gh auth token` çıktısı `--with-token` ile aktarıldı — kullanıcı önce `gho_` önekini eksik kopyalamıştı, tam token'la çözüldü).
- ⚠️ §2.2'de "Conflicting values set for option Signed-By" — imajda ROS deposu ZATEN ekliymiş, rehberin ros2.list'i çakıştı. Çözüm verildi: `sudo rm /etc/apt/sources.list.d/ros2.list && sudo apt update && sudo apt install ros-humble-ros-base ros-dev-tools -y`. Her iki tuzak rehbere işlendi (`ee872af`).
- ✅ ROS Humble kuruldu (`ros2 --help` çalıştı), algı reposu klonlandı.
- ⚠️ `jetson_kur.sh` adım 1/6'da öldü: `set -euo pipefail` × ROS setup.bash `AMENT_TRACE_SETUP_FILES: unbound variable`. **Düzeltildi (`52b851a`, push'lu):** iki scriptte de source `set +u`/`set -u` sarmalı; bu PC'de kırmızı→yeşil kanıtlandı (Humble kurulu olduğundan gerçek test). Jetson'da devam: `cd ~/ros2_ws/src/girdap-ida-algi && git pull` → `bash scripts/jetson_kur.sh`.
- Ara pürüz: `git pull` "Repository not found" — `gh auth setup-git` atlanmıştı, çözüldü.
- ✅ jetson_kur.sh BİTTİ, §2.4 kuruldu, jetson_kontrol koştu. Çıkan 3 [HATA]: numpy 2.2.6 (§2.4 pip'leri çekti → force-reinstall numpy<2 komutu verildi; rehber pip satırlarına sabit eklendi `7f58d9a`) · WiFi açık (bilinçli — internet işleri bitince `rfkill block`) · NN Archive yok (BEKLENEN, video sonrası işi).
- 🔴 **JETSON SIFIR DEĞİLMİŞ — eski workspace gölgelemesi BULUNDU:** suite 13F/7E + 234/247 test verdi çünkü `import girdap_decision` **`/home/girdap/girdap_ws`** (arkadaşın DENETİM-ÖNCESİ eski kurulumu, bashrc'siyle) içinden çözülüyordu — düzelttiğimiz tüm buglar o kopyada yaşıyor, teknede ASLA o koşmamalı. Reçete verildi: `sed -i.yedek '/girdap_ws/d' ~/.bashrc` + `mv ~/girdap_ws ~/girdap_ws.eski` + yeni terminal + `m.__file__` teyidi (~/ros2_ws görülmeli). Rehber §5 hata 8 (`5a15cee` push'lu). Jetson klonu `52fd876` = fork güncel ✓; bizim install egg-link (symlink) ✓. SONUÇ BEKLENİYOR.
- ⚠️ **opencv×numpy DÖNGÜSÜ (iki tur):** makinede opencv-python-headless 5.0.0.93 vardı (depthai DEĞİL — o yalnız numpy<3 ister, PyPI teyitli). `<5` pini YETMEDİ: pip 4.13.0.92 seçti, o da numpy≥2 dayatıp numpy'ı 2.2.6'ya GERİ yükseltti (suite bu hâlde F/E verdi — numpy2 ABI). **PyPI metadata gerçeği: py≥3.9'da numpy<2'ye izin veren SON opencv 4.11.0.86** (4.12+ hepsi numpy≥2). Reçete (rehber §5/8, `810878b`): `pip install --user --force-reinstall opencv-python-headless==4.11.0.86 numpy==1.26.4` TEK komutta. Suite'in numpy 1.26.4 ile yeniden koşulması BEKLENİYOR.
- ✅ gtsam 4.3a0 **aarch64 cp310 manylinux wheel PyPI'da VAR** (2026-07-11 teyit) → §2.4 hızlı binary kurulum; F2.4'ün "ARM64 wheel yok, kaynaktan saatler" endişesi BAYAT.
- Sıradaki: jetson_kur.sh bitince (WiFi kapanır — internet WiFi'dense §2.4 öncesi `sudo rfkill unblock wifi`) → §2.4 (mavros+gtsam+scipy+dialout, sonra oturum kapat-aç) → §2.5 kontroller (jetson_kontrol.sh + suite 246/1) → §2.6 OAK testi → masa runbook M0-M8 (Pixhawk gerekli, M1+).
- ✅ **SONRAKİ OTURUMLARDA HEPSİ ÇÖZÜLDÜ (Jetson'a Claude kuruldu, 2026-07-11 gece + 07-12):** suite teyit (246/1, sonra F-A/F-L düzeltmeleriyle taban 250/2) · girdap_ws temiz · dialout ✓ · OAK §2.6 PASS (USB-C ucunu 180° çevir = SUPER; rehber §5 hata 10) · CUDA Faz A+B bitti (MPPI 302→9 ms) · masa M1/M3/M4 geçti. Ayrıntı + yeni açık bulgular (F-M.1 OOM, F-M.2, FC sahte-görev OLAYI): [[girdap-decision-entegrasyon]] "JETSON DONANIM GÜNLERİ" bölümü. Repo senkronları tamam (yedek dahil, PC pull'landı `203d8cc`/`eae9d9d`, 2026-07-12). WiFi hâlâ açık — en son `rfkill block`.

## docs/BURADAN_BASLA.md — Jetson günü ANA GİRİŞ rehberi (2026-07-11, son commit `ee872af` push'lu)

Eyüp'ün "tek dosyayı okuyarak hepsini yapabileyim" isteğiyle yazıldı/genişletildi (commit zinciri: `dac2a7d` ilk sürüm → `32dc91c` §2.7b → `c742284` §2.9-2.10). Desktop'ta `JETSON_REHBERI.md` symlink'i var. İçerik haritası:

- **§1** Fiziksel bağlantı şeması (ASCII) + tablo: Jetson güç/monitör/ethernet, OAK-D Lite→USB3 MAVİ port, Pixhawk→USB (ttyACM0), RFD868x #1→TELEM1, #2→YKİ laptopu; Livox BUGÜN TAKILMAZ (T1).
- **§2.1-2.4** Kurulum: gh auth (device flow) → ROS Humble → `jetson_kur.sh` (iki repoyu klonlar+derler, numpy<2, udev, en son WiFi kapatır) → mavros+mavros-msgs+gtsam+scipy+dialout.
- **§2.5** Kontrol: `jetson_kontrol.sh` + karar suite'i (beklenen `246 passed, 1 skipped`).
- **§2.6** Modelsiz kamera testi (GEÇİCİ `kamera_goruntu_test.py`, ~12 FPS).
- **§2.7/2.7b** Ana launch komutları (karar: `hardware.launch.py mission_source:=fc`; kamera: `algi.launch.py`) + iki yığının çakışmama gerekçeleri (VPU/CPU, tek OAK sahibi, respawn).
- **§2.8** Mini ROS2 sözlüğü (topic echo/hz, kill servisi).
- **§2.9 GÖREV KOŞUSU uçtan uca (YENİ `c742284`):** fc modda başlat → `/girdap/mission/state` izle → QGC Plan'dan 4 köşe Upload (başlatmadan ÖNCE) → QGC Arm (BOOT→ARM→BEKLEMEDE) → **mod→GUIDED = başlat** (kenar tetikli: boot'ta GUIDED ise HOLD'a al-geri dön; önce ARM sonra GUIDED) → TAMAMLANDI'da sıfır thrust → durdurma tablosu (`/girdap/mission/kill` KALICI latch, `/girdap/bridge/disarm`, Ctrl+C). Masa runbook M4-M6'nın kısa sürümü.
- **§2.10 Kayıt dosyaları (YENİ):** `~/girdap_logs/telemetry` (Dosya-2) · `grafik` (Ekran-2 ham 10 Hz) · `local_map` (Dosya-3) · `viz`; `run_ekran2.py` (PNG) / `--mp4 --t0 --t1` (montaj). Yollar koddan teyitli.
- **§3** Sonraki adım: masa_testi_runbook M0-M8 → suda prova → video. **§4** Rehber haritası (hangi doküman ne zaman). **§5** En sık 5 hata.

## How to apply

- Bu projede kod revize edilirken 416×416 / 10–14 FPS / DepthAI v3 varsayımları esastır; API'den emin olunmayan çağrı kurulu pakete karşı doğrulanır (bu oturumdaki yöntem).
- MPPI tarafına dair her konuda `docs/mppi_entegrasyon_notu.md` tek referanstır; kod değişirse notun sözleşme bölümüyle tutarlılık birlikte güncellenir.
- Push için `/home/eyup/girdap-ida-algi` içinden `git push` yeterli (gh auth hazır, bkz. [[girdap-ida-algi-github-upload]]).
