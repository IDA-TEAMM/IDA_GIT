---
name: girdap-decision-entegrasyon
description: Arkadaşın karar reposu (vistastris/girdap-decision) gerçek mimarisi + perception sözleşmesi + fazlı entegrasyon planının durumu
metadata: 
  node_type: memory
  type: project
  originSessionId: 2afb4def-d306-47e8-8bf0-f118690f51af
---

Arkadaşın (takım karar yazılımcısı) reposu: **github.com/vistastris/girdap-decision** (public). 2026-07-10'da incelendi. [[girdap-ida-proje-durumu]] projesinin karşı tarafı.

## Arkadaşın GERÇEK mimarisi (eski Nav2 varsayımı YANLIŞTI)

- **Nav2 YOK.** Kendi yazdığı RRT* + MPPI (NumPy, Williams 2017) + iSAM2/GTSAM + FSM. `/goal_pose` hiçbir yerde geçmiyor — eski `mppi_entegrasyon_notu.md` bu yüzden bayat.
- **TF yayınlamıyor** — poz kaynağı `/girdap/fusion/odom` (nav_msgs/Odometry) topic'i.
- ~11.5k satır Python, Layer 0 (saf çekirdek) / Layer 2 (ROS sarmalayıcı) ayrımı temiz; 92 saf-NumPy testi geçiyor. MPPI: K=1000, T=50, dt=0.05, CPU ~100ms/iter (CUDA portu Jetson'da yapılacak, henüz yok).
- Kendi kamera kodu = **HSV segmentasyon + MOCK YOLO** (sabit bbox döner, gerçek .pt yok) — bilinçli geçici; Eyüp'ün OAK node'u bunun gerçek karşılığı.

## Perception sözleşmesi (Eyüp'ün uyacağı arayüz)

- `/perception/buoys` — `vision_msgs/Detection2DArray`, bbox **piksel uzayı 640×480** (fusion `camera_image_width_px=640`, `hfov=1.2 rad` varsayıyor), `hypothesis.class_id` STRING: `"0"`=parkur_kenarı (turuncu), `"1"`=engel (sarı), `"2"`=hedef (Parkur-3). Alanlar: `bbox.center.position.x/y`, `size_x/y`, `hypothesis.score`.
- `/perception/gate_passed` — `std_msgs/Bool` → `fsm_node` parkur geçiş kanalı. **Eyüp'ün odom-doğrulamalı geçiş sayacı bu kanalı besliyor** (emekli olmadı!).
- `/perception/obstacle_map` — `PoseArray` hack (`position.{x,y}`=merkez, `orientation.z`=YARIÇAP, quaternion değil) — bu LiDAR node'unun işi, Eyüp buraya YAZMAZ.

## Arkadaşa iletilecekler (henüz iletilmedi)

1. **numpy pini bug'ı:** `requirements.txt`'te `numpy>=1.26` üst sınırsız → Jetson'da numpy 2.x çekip ROS/scipy'yi kırar (`_ARRAY_API not found`). Düzeltme: `numpy>=1.26,<2`.
2. MPPI `obstacle_margin=0.5 m` tekneye dar olabilir (0.75 m en) — inflation endişesinin karşılığı.
3. Pazarlık: Eyüp'te stereo 3D hazır → `/perception/buoys_3d` (PoseArray, obstacle_map şemasıyla aynı) bonus topic; bearing füzyonu yerine doğrudan kullanılabilir.

## Fazlı ilerleme (kredi tasarrufu — her faz sonunda burası güncellenir)

- **FAZ 1 ✅ (2026-07-10):** `duba_gecis_navigator.py`'ye `MOD="algi_yayin"` eklendi (YENİ VARSAYILAN): `/perception/buoys` (letterbox dikey düzeltmeli, `_LB_PAY=0.125` — masa testinde doğrulanacak) + `/perception/gate_passed` + `/perception/buoys_3d` yayınlar; poz kaynağı TF yerine `/girdap/fusion/odom` (`arac_poz_yaw()` soyutlaması); bu modda hiçbir hedef/hız komutu basılmaz. `mppi_hedef` (arşiv) ve `dogrudan_surus` (Plan B) aynen duruyor. `py_compile` geçti. **COMMIT EDİLMEDİ** — working tree'de.
- **FAZ 2 ✅ (2026-07-10):** `docs/mppi_entegrasyon_notu.md` baştan yazıldı (313 satır Nav2 rehberi → sözleşme + sorumluluk tablosu + kurulum + 5 basamaklı test merdiveni + arkadaştan 5 cevap + sorun giderme). README: mimari bölümü `algi_yayin`'a çekildi, 3-mod tablosu, Jetson hedef yığın sürüm tablosu (Humble, numpy `>=1.26,<2`, depthai>=3.6, vision-msgs), kontrol listesi güncellendi. **COMMIT EDİLMEDİ.**
- **FAZ 3 ✅ (2026-07-10):** commit `3bce86b` push edildi (3 dosya, +346/−361). Arkadaşa mesaj metni oturumda verildi. TÜM FAZLAR TAMAM.
- **FAZ 4 (Jetson araçları) ✅ (2026-07-10):** commit `57ebafc` push edildi — `scripts/jetson_kur.sh` (sürüm kilitli kurulum + OAK udev + `--servis`), `scripts/jetson_kontrol.sh` (PASS/FAIL ortam denetimi), `launch/algi.launch.py` (respawn'lı, setup.py data_files'a eklendi), `scripts/girdap-algi.service` (systemd). YOLO'nun .py'ı yok çünkü VPU'da NN Archive koşuyor — Eyüp'e anlatıldı; model tar.xz git dışı, Jetson'a elle taşınacak.

## Bulgu dokümanı — `docs/girdap_decision_bulgular.md` (commit 63c16b6, push edildi)

2026-07-10'da arkadaşın reposu (klon: eski oturum scratchpad'i, `d4ce88b` = origin/main, güncel) salt okunur incelendi; 5 bulgu KANIT+dosya:satır ile bizim repoya yazıldı. **Onun reposuna push YOK** (Eyüp'ün kararı: "arkadaşın reposunu ellemeyelim").

Öncelik (P1+P2 odağına göre):
1. 🔴 **P2-KRİTİK — `/perception/buoys` çift yayıncı:** onun `perception_camera_node` (HSV+MOCK YOLO) `hardware.launch.py:314`'te KOŞULSUZ başlıyor ve bizim OAK node'umuzla AYNI topic'e yayınlıyor → mock bbox'lar gerçek tespitlerle karışır. Düzeltme: launch'ta koşula bağla. **Asıl risk bu.**
2. 🟠 `requirements.txt:1` `numpy>=1.26` üst sınırsız → `,<2`. (Kendi `docs/jetson_deployment.md`'sinde riski yazmış, pinlememiş.)
3. 🟡 **Bearing işaret hatası — GERÇEK ama BUGÜN ÖLÜ KOL.** `fusion.py:83` `(cx-0.5)*hfov` (sağ+) vs `:73` `atan2(y,x)` (sol+). AMA `/perception/classified_obstacles` ABONESİZ; `planning_node.py:115` ham `/perception/obstacle_map` okuyor. → P1/P2'ye bugün etkisi YOK, acil değil. (Önceki memory "P2'yi doğrudan vurur" diyordu — YANLIŞTI, düzeltildi.) Testler yakalamıyor çünkü `synthetic_fusion.py:27` `_cx_for_bearing` tam ters fonksiyon, hata sadeleşiyor.
4. 🟡 `CLAUDE.md:321-322` RAL 2008/1003 → şartname 2003/1026.
5. ⚪ `mppi.py:88` `obstacle_margin=0.5` m, tekne 0.75 m en — saha testinde ölç.

**REDDEDİLEN yaklaşım (§6, gerekçesi dokümanda):** bearing hatasını bizim tarafta `bbox_cx` aynalayarak telafi etmek — Dosya-1 zorunlu overlay videosunda kutular dubaların üstüne oturmaz (5 ceza puanı), `buoys_3d`'yi bozar, o işaretini düzeltince çifte çevirmeyle sessizce tekrar bozulur. Onun ~11.5k satırlık yığınını bizim repoya kopyalamak da reddedildi (okumadığımız `rrt_star.py`/`isam2_smoother.py` bakımı + o aktif geliştiriyor + teslime 11 gün).
2. **Letterbox teorik teyidi:** depthai 3.6.1'de `ImgDetections.getTransformation()` → `ImgTransformation.remapPointTo/remapRectFrom` API'si var — tespit koordinatları NN giriş çerçevesinde normalize, dönüşüm metadata'yla taşınıyor (bu yüzden remap API'si var). `_LB_PAY=0.125` varsayımı sağlam; istenirse elle formül yerine `getTransformation().remapPointTo` ile kesin remap yapılabilir (gelecek iyileştirme). Masa testi yine de şart.
3. **Dosya-1a boşluğu:** Şartname 4.2 — kamera bbox+sınıf overlay mp4 (≥1 Hz, zaman etiketli) yarışma çıktısı ZORUNLU, gecikme 5 ceza puanı. Kamera bizde → sorumluluk bizim; ne bizim repoda ne onunkinde yazılmış. Arkadaşın Dosya-2 (telemetri csv) + Dosya-3 (harita png) hazır. `algi_yayin`'a passthrough frame + cv2.VideoWriter kaydedici eklenmeli (FPS etkisi masa testinde ölçülmeli — ekstra stream USB bandı yer).
4. Takvim: Otonomi videosu son teslim **21.07.2026** (11 gün) — video senaryosu arkadaşın stack'inde hazır (GPS dikdörtgen, use_isam2=false); bizim tarafın videoya katkısı opsiyonel ama Dosya-1a kaydedici videoda da kullanılabilir.

## Geçit tanımı düzeltmesi + İHA planı (2026-07-10, commit e9c495c)

- **Geçit = karşılıklı KENAR×KENAR (turuncu) çifti** — puanlama G/KD tanımından; eski kod kenar×engel çiftini geçit sanıyordu, düzeltildi (`e9c495c`). Sarı engel yalnız kaçınılır. Eski davranış `ENGEL_YEDEK=False` saha yedeğinde.
- Eyüp'ün planı: hedef dubalarını İHA halledecek, sonraki mesele. ⚠️ **İHA kısıtı (md 5.5.3.1): İHA yalnız KIYI tarafında uçabilir, deniz üstü uçuş = Parkur-3 başarısız.** İHA sadece kıyıdaki renk PLAKASINI okur (hangi renge angaje olunacağını söyler); denizdeki 3 hedef dubasından doğru renklisini BULMAK yine İDA kamerasının işi (yanlış hedefe temas 100→50→5 düşürür). Yani hedef sınıfı/renk ayrımı İDA algısından çıkarılamaz — Eyüp'e söylendi, karar ertelendi.

## Şartname uyum değerlendirmesi (2026-07-10)

Kod şartnameye karşı değerlendirildi: çekirdek uyumlu (RAL kodları doğru, Dosya-1 `2282241`, MIN_GECIT=2, otomatik parkur algılama, LETTERBOX). WiFi/BT kapatma `c51008f` ile kur+kontrol scriptlerine eklendi. KALAN: (1) 🔴 hedef duba tespiti (3 renk, modelde sınıf yok — strateji kararı Eyüp'te), (2) eğitim verisine beyaz sosis duba negatifi + bayraklı armut dubalar, (3) kamera yağmur muhafazası (mekanik ekip) + muhafaza arkası odak masa testi, (4) hedef RENK bilgisinin sözleşmeye eklenmesi (arkadaşın "2"=hedef tek sınıfı yetmez).

## ODAK KARARI (Eyüp, 2026-07-10): Parkur 1 + Parkur 2

Ana odak şimdilik P1 (nokta takibi, 55p) ve P2 (dubalı parkur, 100p). **Parkur-3 işleri ERTELENDİ:** hedef duba tespiti (3 renk, model/HSV kararı) ve İHA plaka konusu şimdilik askıda — P1+P2 oturana kadar açılmayacak. Şartname gerçeği değişmedi (3 hedef dubası var, [[sartname-ida-2026]]), sadece öncelik değişti.

P1+P2 için algı tarafında kalan işler (öncelik sırasıyla):
1. **Masa testi** (donanım, Eyüp): letterbox `_LB_PAY=0.125` doğrulaması + Dosya-1 kaydedicinin FPS etkisi ölçümü (`duba_kamera_test.py`).
2. **Arkadaşa mesaj** (metin hazır, oturumda verildi): bearing işaret hatası, numpy<2 pini, obstacle_margin, §6'daki 5 soru.
3. **Jetson kurulum/kontrol**: scriptler hazır (`57ebafc`), Jetson'da koşulacak.
4. **Eğitim verisi iyileştirme** (P2 tespit kalitesi): beyaz sosis duba negatifi + bayraklı armut duba örnekleri.
5. Kamera yağmur muhafazası (mekanik ekip) + muhafaza arkası odak testi.

## ⚠️ "Arkadaşın her şeyini biz yapalım" — PREMİS YANLIŞ (2026-07-10 doğrulaması)

Eyüp "arkadaş şu an çalışamaz, onun her şeyini biz yapalım, 50 faza böl" dedi. Koddan doğrulandı: **onun yığını YARIM DEĞİL, esasen BİTMİŞ.**
- `11.493` satır Python (prototype + ros2_ws)
- `161` test fonksiyonu (`prototype/tests/`, 20 dosya)
- **SIFIR** `TODO` / `FIXME` / `NotImplementedError` / `XXX` (tüm ağaç tarandı)
- Layer 0 (saf NumPy çekirdek) / Layer 2 (ROS sarmalayıcı) ayrımı temiz

Yani "her şeyini yapmak" = 11.5k satır bitmiş+test edilmiş kodu, yarışmaya 11 gün kala, test edilmemiş kodla değiştirmek. Yapılacak gerçek iş listesi ÇOK KÜÇÜK — `docs/girdap_decision_bulgular.md`'deki 5 madde + MPPI CUDA portu (CPU ~100ms/iter). 50 faz gerektirmiyor.

**MEKANİZMA KARARI ✅ (Eyüp onayladı):** FORK yapıldı, kopyalama YOK.
- `gh repo fork vistastris/girdap-decision` → **`EyupEker1/girdap-decision` (PRIVATE)**
- Klon: **`/home/eyup/girdap-decision`**, `origin`=bizim fork, `upstream`=vistastris. Git geçmişi tam, geri merge edilebilir.
- ⚠️ **Düzeltme:** arkadaşın reposu **PRIVATE** (bu memory eskiden "public" diyordu — YANLIŞ). Eyüp'ün erişimi var.
- `girdap-ida-algi`'ye onun kodu KOPYALANMADI (ayrı ROS2 workspace; birleştirmek colcon/setup.py'ı bozardı).

**KAPSAM KARARI (Eyüp seçti):** "Yığını baştan gözden geçir" — 11.5k satırın TAMAMI denetlenecek. (Not: gözden geçirmek ≠ yeniden yazmak; salt okuma + hedefli düzeltme, bitmiş kod çöpe atılmıyor.)

Not: onun testlerini çalıştırma izni auto-mode classifier tarafından bir kez reddedildi (dış repo kodu). Faz 18'de gerekecek — Eyüp'ün açık onayı istenecek.

## Denetim faz planı (18 blok, "50 faz" keyfi bir sayıydı; iş bu kadar bloğa bölünüyor)

Görev listesi Task #1–#17 olarak kurulu. Bulgu dokümanı: **`girdap-decision/docs/kod_denetimi.md`** (forkta, her faz sonunda commit+push).

Git kimliği forkta LOKAL kuruldu: `Team GIRDAP (Algi) <girdap@example.com>` (--global değil).

- **Faz 1 ✅** Fork + upstream + klon + kod haritası. prototype 5137 / ros2_ws 3170 / testler 3186 = **11493 satır, SIFIR TODO/FIXME**. `CLAUDE.md`'nin şemasındaki `cpp/` dizini YOK (özlem).
- **Faz 2 ✅ Bağımlılık&paketleme** (commit `90d5e5c`):
  - 🔴 **F2.1** `package.xml:14-21`'de **`vision_msgs`, `message_filters`, `python3-pillow` BEYAN EDİLMEMİŞ** ama kod kullanıyor (`perception_camera_node.py:34`, `perception_fusion_node.py:33`, `local_map.py:29`). Temiz Jetson'da `rosdep` kurmaz → 3 node ImportError. `local_map_node` ölürse **Dosya-3 üretilmez = 5 ceza puanı**. EN DEĞERLİ BULGU (mock kamera çakışmasından sonra).
  - 🟠 **F2.2** `requirements.txt:1` `numpy>=1.26` → `,<2` olmalı.
  - 🟠 **F2.3** `prototype/` kurulmuyor; 11 node `from prototype.*` import ediyor, `setup.py:18` `find_packages` onu bulmuyor. `jetson_deployment.md:141` `export PYTHONPATH=$HOME/girdap-decision` diyor → systemd servisinde `Environment=PYTHONPATH=` şart, yoksa boot'ta ImportError.
  - 🟡 **F2.4** `requirements.txt:4` `gtsam>=4.3a0` vs `CLAUDE.md` "GTSAM 4.2" çelişkisi; ARM64 wheel yok (kaynaktan derleme saatler). İyi haber: `fusion_node.py:102` GTSAM'ı **lazy import** ediyor → `use_isam2:false` video modunda hiç yüklenmiyor.
  - ⚪ **F2.5** `pyproject.toml:5-7` ruff `select` eski şema.
- **Faz 3 ✅ Launch kompozisyonu** (commit `9dda92a`):
  - 🔴 **F3.1 OAK-D CİHAZ SAHİPLİĞİ ÇAKIŞMASI** (önceki "çift yayıncı" teşhisi FAZLA ALARMCIYDI, düzeltildi): `perception_camera_node.py:95` `/oak/rgb/image_raw` dinliyor; `hardware.launch.py:353-358` yorumu donanım ekibinden `depthai_ros` sürücüsü eklemesini istiyor. Bizim node OAK'ı DOĞRUDAN DepthAI ile açıyor (VPU'da YOLO). → sürücü eklenirse USB exclusive, kamera TAMAMEN ölür. Şu an sürücü yoksa onun node'u zombi (girdi yok → sessiz, /perception/buoys'a yayın YAPMAZ). Düzeltme: `:314-317` node'u koşula bağla + `:353-358` OAK talimatını sil.
  - 🟠 **F3.2 `use_mppi` ÖLÜ launch argümanı:** `girdap_decision/*.py`'de sıfır kullanım; sadece `:49,:158,:187,:349`. `LogInfo` değeri BASIYOR → operatöre yalan söylüyor. `algorithm.use_mppi:false` sessizce yok sayılır.
  - 🟠 **F3.3 `except Exception: pass` (`:128-129`)** tüm config+cast hatalarını yutuyor, uyarı yok. Geri düşülen varsayılanlar TUTARSIZ: `_ALGO_DEFAULTS` yarışma (isam2+rrt True) ama `_MISSION_DEFAULT` = `video_mission.yaml`. Video günü YAML yazım hatası → bypass edilmesi gereken iSAM2+RRT* sessizce açılır.
  - 🟡 **F3.4 static TF'ler 0,0,0 VE hiçbir node tf2 okumuyor** (`tf2_ros`/`lookup_transform` sıfır sonuç) → sensör extrinsic'i sisteme hiç girmiyor. Sensörler ~0.5 m ayrıysa 5 m'deki dubada ~6° bearing hatası; `bearing_tolerance_rad=0.15` (≈8.6°) eşiğinin yarısı. Bearing işaret hatasıyla birlikte değerlendirilmeli.
  - ⚪ F3.5 node sıralaması yorumu yanlış varsayım.
- **Faz 4 ✅ Config YAML** (commit `1d3711c`):
  - 🔴 **F4.1 Dosya-2 (zorunlu telemetri CSV) GÖRELİ YOLA yazılıyor:** `params.yaml:92` `csv_output_dir: "data/telemetry"`, `csv_logger.py:139` `Path(x).expanduser()` göreli yolu çözmez → `ros2 launch`'ta cwd'ye, systemd'de **cwd=`/` → `mkdir("/data/telemetry")` → PermissionError, node boot'ta ölür**. Dosya-3 (`local_map.py:66` `Path.home()/girdap_logs/...`) aynı işi DOĞRU yapıyor. **5 ceza puanı.** Düzeltme: `""` varsayılan + `Path.home()` deseni.
  - 🟡 **F4.2** `control_rate_hz: 20.0` (50 ms) kendi yorumuyla (`params.yaml:26-28`, "CPU K=1000 ~100 ms") çelişiyor; CPU'da döngü taşar. Tutarlı değer ~10 Hz. Faz 11'de kesinleşecek.
  - 🟡 **F4.3** `hardware.yaml:81-86` `topics:` bloğu `_load_hardware_config()` tarafından **hiç okunmuyor** → ölü config, remap üretilmiyor, node'lar topic adını kodda sabitliyor.
  - 🟡 **F4.4 AÇIK SORU** `params.yaml:29-30` RRT* `bounds [0,200]×[0,200]`; çerçeve araç-merkezli/odom ise araç köşede → yalnız +x/+y çeyreğinde örnekler, "başlangıca dönüş" planlanamaz. Faz 11'de netleşecek.
  - ⚪ **F4.5** HSV yorumlarındaki RAL kodları yanlış (2008/1003 → 2003/1026) AMA değerler gerçek RAL için tesadüfen doğru: RAL 2003 H≈13 (aralık 5-20 ✓), RAL 1026 H≈30 (aralık 21-35 ✓). Yanlış RAL 1003 H≈20 sınırda kalırdı. Sadece yorum düzeltilecek.
  - ✅ **F4.6 DOĞRULANDI:** bbox 640×480 sözleşmesi iki repo arasında TUTARLI (`params.yaml:85-86` ↔ `duba_gecis_navigator.py:154-155`). `requestOutput((640,400))` yalnız mono/stereo akışı, bbox uzayı değil — karıştırma.
  - 📋 **F4.7** `hardware.yaml` şu an video modunda (`use_isam2/use_rrt: false`, `video_mission.yaml`) — doğru. Yarışma günü 3 flag değişecek.
- **Faz 5 ✅ Algı çekirdeği** (commit `e222bce`) — **DENETİMİN EN CİDDİ BULGUSU**:
  - 🔴 **F5.1 LiDAR Z-FİLTRESİ YANLIŞ ÇERÇEVEDE.** `perception_lidar_node.py:89-96` `/livox/lidar` noktalarını **hiç dönüştürmeden** `detect_obstacles`'a veriyor. `lidar_obstacles.py:41` `z_min=0.1` "su hattından" varsayıyor; veri ise **sensör çerçevesinde**. LiDAR su hattından `h` m yukarıdaysa 50 cm duba `z∈[−h,−h+0.5]`'te görünür → **h>0.4 m ise dubaların TAMAMI elenir** → `obstacle_map` boş → MPPI dubaların içinden geçer → **Parkur-2 biter.** Katamaranda h>0.4 kesin. 161 test yakalamıyor: `synthetic_lidar.py:39` `z=uniform(0,0.5)` yani origin'i SU YÜZEYİNE koyuyor — gerçek sürücünün asla üretmediği çerçeve. **MASKELEME DESENİ.** Düzeltme: (a) node TF ile dönüştürsün, veya (b) `lidar_height_m` param + `z∈[z_min−h, z_max−h]`. **Mekanik ekipten `h` istenecek — bilinmeden Parkur-2 sahada koşturulmamalı.**
  - 🟠 **F5.2** `perception_lidar_node.py:107` `frame_id="base_link"` **sahte etiket**; dönüşüm yok, yalnız yeniden etiketleme. Mekanik gerçek extrinsic verse bile hiçbir şey değişmez (kimse tf2 okumuyor).
  - 🟠 **F5.3** clustering: `lidar_obstacles.py:112-114` saf Python `union` döngüsü + `:95` `labels()` Python döngüsü, **voxel downsample YOK**. Livox 20k nokta/mesaj, `tol=0.5` → 10⁵–10⁶ çift → yüzlerce ms, 10 Hz tutmaz. Düzeltme: `scipy.sparse.csgraph.connected_components` + voxel downsample.
  - 🟡 **F5.4** `max_cluster_size=500` (`:118`) → nokta sayısı mesafeyle ters orantılı olduğundan **en yakın/en büyük engeli SESSİZCE siler**. Bölmek gerek, atmak değil.
  - 🟡 **F5.5** HSV etkin menzil **≈15 m** (`min_area_px=150`, 640px/1.2rad ⇒ d≈15 m) vs LiDAR 25 m. 15 m ötesi renk/sınıf bilgisi yok → geçit çifti 15 m'den önce belirlenemez. Sözleşmeye yazılmalı.
  - 🟡 **F5.6** `camera_buoys.py:108` `score = area/(w*h)` = **doluluk oranı**, güven değil. Yuvarlak duba 0.785, dikdörtgen yanlış-pozitif 1.0 → **skor TERS**. Bizim YOLO buraya gerçek güven koyuyor → iki repo aynı alana farklı anlam yüklüyor.
  - 🟡 **F5.7** `_infer_real` (`:170-172`) `int(box.cls)`'i doğrudan `class_id` yapıyor. Bizim modelde **hedef sınıfı yok** (yalnız 0/1) → `use_yolo=true` açılırsa HSV'nin 0/1'i üstüne **çift tespit**, `class_id=2` hiç üretilmez. Sınıf eşleme tablosu şart.
  - ✅ **F5.8 DÜZELTME:** mock YOLO **hiç çalışmıyor** — `use_yolo=false` (hardware.yaml:64 + params.yaml:72). "Mock bbox `/perception/buoys`'u kirletiyor" endişesi **iki kez** çürütüldü (F3.1 girdi yok + F5.8 flag kapalı). F3.1'in cihaz-sahipliği düzeltmesi yine de geçerli.
  - 🟡 **F5.9** bearing işaret hatası, F5.1 ile **aynı maskeleme desenine** sahip (`synthetic_fusion.py:27` ters fonksiyon). Füzyon iki bağımsız geometrik hata taşıyor (işaret + extrinsic).
- **Faz 6–18 YAPILMADI** (kredi bitti, 2026-07-10). Sıradaki: **Faz 6 Sentetik üreteçler (test maskeleme avı)** — F5.1 ve F5.9 aynı deseni gösterdi, `synthetic_camera.py`/`synthetic_lidar.py`/`synthetic_fusion.py` sistematik taranacak: kaç test, üretecin hatalı varsayımını aynen kullanıp hatayı maskeliyor? Sonra Faz 7 (algı ROS node'ları) → 8 (füzyon/iSAM2) → 9 MPPI(569) → 10 RRT*(477, hiç okunmadı) → 11 planlama+node (F4.2 ve F4.4 açık soruları burada kapanacak) → 12 FSM → 13 görev yönetimi → 14 MAVROS → 15 harita+telemetri → 16 testler(161 fn) → 17 doküman → 18 düzeltmeleri uygula+test+push.

## 🔴 FAZ SIRASI YENİDEN DİZİLDİ (2026-07-10) — video eleme kapısı

Şartname md 3 + 3.3 birinci kaynaktan okundu: **Otonomi videosu ELEME KAPISI** (21.07.2026 17:00 → 27.07 finalistler). Geçilmezse P1+P2+P3 = 0. Ayrıntı: [[sartname-ida-2026]].

**Videoda algı YOK.** F5.1 / sınıf sırası / bearing — hiçbiri videoyu bloke etmiyor, hepsi 30 Eylül işi.

### Video şartlarının koda eşlenmesi (kanıtlı)

| md 3.3.1 / 3.3.1.1 | Kod | Durum |
|---|---|---|
| (1) İDA↔YKİ kablosuz, İDA bilgisi YKİ'de + Ekran-1 YKİ ekranı | **YKİ yazılımı iki repoda da YOK**; `gcs_url:""` köprüyü kapatıyor | ❌ **BOŞLUK** |
| (2) YKİ'den 4 nokta tanımla + İDA'ya gönder | `mission_manager_node` yalnız `mission_file` dosyasından okur, yükleme arayüzü yok | ❌ **BOŞLUK** |
| (3) YKİ/RC komutuyla başlat, son noktada tamamlan | `/girdap/mission/start`, arrival_radius+dwell | ✅ var |
| (4) Güvenlik anahtarıyla güç kes, RC'ye rağmen motor dönmesin | `/girdap/bridge/disarm`, `/girdap/mission/kill` var ama şart **fiziksel** | ⚠ donanım |
| (5) Kapak açıp su almazlık | — | ⚠ mekanik |
| Ekran-2a hız + hız_setpoint | `csv_logger.CSV_HEADER` | ✅ var |
| Ekran-2b heading + yon_setpoint | `csv_logger.CSV_HEADER` | ✅ var |
| Ekran-2c **thrusterlardan kuvvet isteği** | `planning_node:133` `/girdap/control/thrust` **yayınlanıyor** ama CSV'ye **yazılmıyor** | ⚠ kaydedilmiyor |
| "istemsiz dönüş/sürüş → BAŞARISIZ" | MPPI kararlılığı | ⚠ Faz 9 doğrudan eleme kriteri |
| Ekran-3 dış kamera | Eyüp dış kameradan çekecek | ✅ (md 4.1 zaten İDA'dan görüntü aktarımını yasaklıyor) |

⇒ **Fazlar tek başına YETMİYOR.** Üç kod boşluğu (YKİ, görev yükleme, thrust kaydı) + donanım gösterimleri + suda prova.

### Yeni görev sırası
- **T0 (video, 11 gün):** #13 MAVROS köprüsü · #12 görev yönetimi · #10 planning_node · #8 MPPI · #14 telemetri(Ekran-2 + F4.1) · **#18 YKİ boşluğu** · **#19 görev yükleme boşluğu** · **#20 thrust kaydı** · **#21 suda 4 nokta provası**
- **T1 (yarışma, 30 Eylül):** #5 sentetik/maskeleme · #6 algı node'ları · #7 füzyon · #9 RRT* · #11 FSM · #14'ün harita kısmı (Dosya-3) · #15 testler · #16 doküman
- **T2:** Parkur-3 (145 puan — en büyük ödül, ama P2 çalışmadan erişilemez)

## T0-a ✅ MAVROS köprüsü + KILL zinciri (commit `5afe665`)

KILL zinciri doğrulandı: `mavros_bridge_node._trigger_kill()` → `/girdap/mission/kill` → `fsm_node` → `/girdap/mission/state=KILL` → `planning_node.compute_control()` None → sıfır thrust (+ `planning_node` kendi `control_gate.zero_thrust`'ı). **Çalışır AMA `planning_node` canlı + companion↔FCU hattı açıkken.**

- 🔴 **F14.1** KILL, FCU'yu **disarm etmiyor**. `_cli_arm` var (`node:103`) ama `_trigger_kill` (`:281-290`) kullanmıyor. (a) Heartbeat-kaybı KILL'inde (KILL'in #1 tetikleyicisi) `/mavros/state` gelmiyor = hat kopuk → sıfır-thrust da FCU'ya ulaşmaz. (b) KILL sonrası araç ARMED kalır. Düzeltme: `_trigger_kill` önce `/mavros/cmd/arming False` çağırsın; FCU'nun kendi failsafe'i (ArduPilot GCS/throttle) teyit edilsin (FC ekibi).
- 🔴 **F14.2 (VİDEO)** Görev sonu **kasıtlı disarm** (md 3.3.1/4 güç kesme gösterimi) `_on_monitor:160-166`'da "FAILSAFE beklenmedik disarm → KILL" sanılıyor. `_request_disarm` beklenen-disarm bayrağı set etmiyor. Kameraya çekilen anda hata basar + latch. Düzeltme: `_expected_disarm` bayrağı.
- 🟠 **F14.3 (VİDEO)** `auto_guided` (`_on_state:135`) manuel dönüşte (md 3.3.1/3) operatörle kavga eder — sürekli `set_mode GUIDED`. Görev-aktife bağla ya da manuel modda kes.
- 🟠 **F14.4** Heartbeat KILL kalıcı latch (`_killed=True`, `_on_monitor:146` erken dön). Geçici USB/mavros hıçkırığı (5 sn) → kurtarılamaz görev kaybı. Histerezis/kurtarma yok.
- 🟡 **F14.5** Üç ayrı `MavrosBridge` örneği (node:82, planning:96, +çekirdek), bağımsız `/mavros/state` aboneliği → parçalı otorite.
- 🟡 **F14.6** `planning_node` ölürse motoru durduran bağımsız watchdog yok (F14.1 FCU-disarm gerekçesi).
- ✅ **F14.7** Doğru yanlar (dokunma): pre-arm retry + KILL tetiklememe, kendiliğinden arm etmeme, `control_gate` öncelik sırası, mock'ta çökmeme.
- ⚠ Hiçbiri simülasyonda koşmadı (mock `armed=True/GUIDED`) — ilk kez gerçek FCU'da çalışacak.

## T0-b ✅ Görev yönetimi (commit `37335e9`)

- 🟠 **F13.1 (VİDEO)** Görev sonu **temiz duruş YOK**. `MissionManager` COMPLETE olunca (`mission_manager.py:114`) `current_target` yayınını keser ama `planning_node._on_target:195` son hedefi `set_reference_direct` ile tutar → MPPI istasyon-tutma titremesi. `MissionFSM` de 4. wp'de `dist_to_last_wp_p1<1.5` görüp PARKUR2'ye geçebilir (video'da P2 wp yok). md 3.3.1.1 "istemsiz hareket → BAŞARISIZ" riski. **Çözüm Faz 12 (FSM) ile: COMPLETE→TAMAMLANDI→planning None→sıfır thrust.**
- 🟠 **F13.2 (VİDEO)** Başlatma çok adımlı: `/girdap/bridge/arm` → FSM BOOT→ARM→BEKLEMEDE (`fsm_node:286` armed+killswitch off) → `/girdap/mission/start` (`_on_start_srv:263` yalnız BEKLEMEDE) → PARKUR1 → `mission_manager.start()`. "Tek komut" değil, prova+script gerekli.
- 🟡 **F13.3 DÜZELTME** `current_target` frame_id="base_link" YANLIŞ etiket (data dünya-ENU ofseti) AMA canlı bug DEĞİL: `planning_node:195-197` ofseti dünya odom `_last_xy`'ye ekliyor, tutarlı. İlk "araç yanlış yöne sürer" şüphem çürütüldü. Risk: RViz/başka tüketici base_link'i harfi harfine okursa yanılır.
- 🟡 **F13.4** Hedef GPS'ten (`_on_gps`), taban fusion-odom'dan — iki poz kaynağı; odom sürüklenirse hedef kayar.
- 🟡 **F13.5** `_started` latch (`node:146`) → görev yeniden başlatılamaz (şartname 5.5.1 restart hakkı).
- 🟡 **F13.6** GPS bayatlık kontrolü yok — düşerse bayat pozisyondan ofset.
- ✅ **F13.7** `MissionManager` arrival/dwell/index çekirdeği (`:92-123`) temiz, dokunma.

## Faz 12 ✅ FSM (commit `8ab955f`)

- 🔴 **F12.1** `dist_to_last_wp_p1` (`fsm_node._on_odom:207`) **odom orijinine (0,0)** ölçülüyor. Kaynak `last_waypoint_xy` param `params.yaml:42` `[0,0]`, yorumu "planning override eder" ama grep: **hiçbir yer yazmıyor**, kalıcı [0,0]. PARKUR1→PARKUR2 geçişi (eşik 1.5 m) gerçek son wp'yle alakasız. Video'da araç orijinde başlar → görev başlar başlamaz spurious PARKUR2 geçişi.
- 🔴 **F12.2 (VİDEO, F13.1 kök nedeni)** Video senaryosunun **terminal durumu YOK**. TAMAMLANDI yalnız `PARKUR3 + shock_detected_p3` ile (`mission_fsm.py:226`). Video tek parkur + çarpma yok → hiç TAMAMLANDI'ya varamaz → PARKUR2'de takılı → `compute_control` aktif → MPPI istasyon-tutma titremesi → md 3.3.1.1 "istemsiz hareket → BAŞARISIZ". **Düzeltme:** `mission_manager` COMPLETE → FSM TAMAMLANDI (parkur/kamikaze yolundan bağımsız yeni geçiş kuralı). TAMAMLANDI'da compute_control None → sıfır thrust zinciri ZATEN var.
- 🟠 **F12.3** İki paralel FSM: mesafe-tabanlı buggy **MissionFSM otoriter** (`/girdap/mission/state` onu yayınlıyor, `fsm_node:289,300`); doğru waypoint-index **ParkurTransitionLogic yalnız log** basıyor (dekoratif). CLAUDE.md anlatısının tersi.
- 🟡 **F12.4 (P3)** Kamikaze şok bayrağı `_on_imu:213` **her durumda** set; Deniz Durumu-2 dalgası PARKUR1/2'de 5g geçerse latch → P3'e girer girmez anında TAMAMLANDI. Şok yalnız P3'te aktif olmalı.
- ✅ **F12.5** Çekirdek yapı (`mission_fsm.py`) temiz; bug'lar onu besleyen `fsm_node` gözlemlerinde, çekirdekte değil.

**Video düzeltme paketi netleşiyor (Faz 18):** F12.2 (mission COMPLETE→TAMAMLANDI) + F12.1 (last_waypoint_xy gerçek son wp'den) + F13.1 (temiz duruş) aynı kök: video-sonu davranışı. F14.2 (kasıtlı disarm≠failsafe) + F14.3 (auto_guided manuel dönüşte kes) + T0-f (FC'den görev) + T0-g (thrust telemetri). Bunlar birlikte videoyu koşturulabilir yapar.

## T0-c ✅ planning_node + pipeline (commit `cfbf32d`)
- 🔴 **F11.1** MPPI **her hedef güncellemesinde (5 Hz) sıfırdan yaratılıyor**: `_on_target:187` → `set_reference_direct` (`pipeline.py:137`) → `_rebuild_mppi:224` = yeni `MPPIController`. Warm-start 200 ms'de bir kaybolur → zikzak (md 3.3.1.1 video BAŞARISIZ). F12.2/F13.1 ile 3. istemsiz-hareket kaynağı. Düzeltme: yeniden yaratma yerine `set_reference`/`set_obstacles` güncelle.
- 🟠 **F11.2** `set_obstacles` (`pipeline.py:154`) da `_rebuild_mppi` çağırıyor (~10 Hz) → F11.1'i katlıyor.
- ✅ **F4.4 ÇÖZÜLDÜ (video):** `bounds [0,200]` yalnız RRT*'ta; video `use_rrt=false` bypass bounds'a dokunmaz. T1'de açık soru (odom negatife geçerse RRT* örnekleme dışı).
- 🟡 **F4.2 ilerledi:** `control_rate=20 Hz` timer MPPI step'i senkron çağırıyor.
- 🟡 **F11.3** `_on_obstacles` F5.1/F5.4 zehirli girdisinden besleniyor (planning doğru, girdi bozuk). Video'da LiDAR şart değil.
- ✅ **F11.4** MAVROS geçidi + parkur profili ayrımı sağlam.

## T0-d ✅ MPPI (commit `4b910f9`)
- 🔴 **F9.1 = F11.1 KANITLANDI:** warm-start makinesi doğru (`_apply_warmstart:356` kaydırma) ama `__init__:143` `U_nominal=zeros`. Pipeline kontrolcüyü yeniden yaratınca warm-start sıfırlanıyor. Kanıt artık kesin. Düzeltme pipeline'da.
- 🟡 **F4.2 ÇÖZÜLDÜ:** `step` senkron ~100 ms/iter (yazar ölçümü); 20 Hz CPU'da imkânsız → ~10 Hz'e çek, Jetson'da ölç (bekleyen D3). Baskın maliyet: track argmin `(K,T+1,n_ref)`.
- 🟡 **F9.2** MPPI aracı NOKTA sayıyor; 0.75 m tekne yalnız `obstacle_margin=0.5`'te. Emniyet sınırında 0.275 m açıklık, Deniz Durumu-2'de dar (T1).
- ✅ **F9.3** Çekirdek SAĞLAM: softmax kararlı (S_min + guard), dt/T/K tutarlı, heading sarılı, quadratic barrier doğru. Williams 2017 sadık. **MPPI'ye dokunma.**

## T0-i ✅ Telemetri (commit `6153c2a`)
- 🔴 **F15.1 (F4.1 kesin):** Dosya-2 göreli yola. `ros2 launch`→rastgele yer; **systemd cwd=/ → `mkdir(/data/telemetry)` PermissionError → node `__init__`'te çöker → Dosya-2 üretilmez → 5 ceza.** `local_map.py:66` `Path.home()` ile doğru yapıyor. Düzeltme: `""` varsayılan + home fallback.
- 🟠 **F15.2 (Ekran-2c = T0-g):** thrust CSV'de yok. md 3.3.1.1 3. sinyal "thrusterlardan kuvvet isteği". `/girdap/control/thrust` var ama `telemetry_node` abone değil. **Dosya-2 (md 4.2) thrust İSTEMİYOR — CSV_HEADER değişmez;** video için AYRI grafik-CSV/rosbag.
- 🟡 **F15.3** heading latch (`_heading_from_imu`), IMU düşerse donar. **F15.4 (VİDEO)** hız'ın odom yedeği yok → `velocity_body` yoksa Ekran-2 hız grafiği boş. **F15.5** GPS bayatlık kontrolü yok.
- ✅ **F15.6** Dikkatli kod: fsync güç-güvenli, 2Hz, BEST_EFFORT QoS, CSV_HEADER Dosya-2 birebir. Dokunma.

## ✅ T0 (VİDEO) DENETİMİ TAMAM — 12 faz bitti (1-5, T0-a/b/c/d/i, Faz12)

**Video eleme kapısını (21.07) bloke eden BULGULAR (Faz 18'de düzeltilecek):**
- **İstemsiz hareket (md 3.3.1.1 → video BAŞARISIZ) — 3 kök, hepsi düzeltilebilir:**
  - F11.1/F9.1 MPPI 5-10 Hz yeniden yaratılıyor → warm-start kaybı → zikzak. Düzeltme: pipeline `_rebuild_mppi` yerine `set_reference` güncelle.
  - F12.2 video terminal durumu yok → 4. wp'de PARKUR2'de takılı, MPPI istasyon-tutma. Düzeltme: mission COMPLETE→FSM TAMAMLANDI.
  - F12.1 `last_waypoint_xy` hep [0,0] → spurious geçiş. Düzeltme: gerçek son wp'den besle.
  - F4.2 control_rate 20Hz CPU'da imkânsız → ~10Hz.
- **MAVROS (T0-a):** F14.1 KILL disarm etmiyor, F14.2 kasıtlı disarm=failsafe sanılıyor (video güç-kesme anı), F14.3 auto_guided manuel dönüşte kavga.
- **Zorunlu çıktı:** F15.1 Dosya-2 yolu (5 ceza), F15.4 hız grafiği boş riski.
- **Kod boşlukları (yazılacak):** T0-f görevi FC'den oku (#19), T0-g thrust telemetri (#20).

**Kalan T1 (yarışma, 30 Eylül) denetimi:** #6 algı node, #7 füzyon, #5 sentetik, #9 RRT*, #15 testler, #16 doküman.

## ✅ TEST EDİLEBİLİRLİK ÇÖZÜLDÜ (2026-07-10) — Eyüp "test edemez miyiz" dedi

Önceki "testleri koşturamıyorum" engeli KALKTI. Auto-classifier reddi *kullanıcı istemediği* içindi; Eyüp açıkça isteyince açıldı.
- **ROS 2 Humble KURULU** (`/opt/ros/humble`). `python3` sistem: pytest 6.2.5, numpy **2.2.6**, scipy 1.8.0.
- **Düzeltme hedefi modüllerin testleri GEÇİYOR:** `PYTHONPATH=/home/eyup/girdap-decision python3 -m pytest prototype/tests/test_{mppi,planning_bypass,mission_fsm,mission_manager,parkur_fsm,mavros_bridge,telemetry_logger,fusion,dynamics,local_map}.py` → **95 passed, <1 sn.** Bunlar saf-Python, scipy/gtsam/rclpy gerektirmez.
- **Çalışmayanlar (video-kritik DEĞİL):** `test_fusion_pipeline` gtsam yok (`ModuleNotFoundError`); lidar testleri scipy 1.8.0 vs numpy 2.2.6 ABI (F2.2 CANLI); 5 `*_node` testi rclpy gerektirir (ROS source'lanırsa çalışabilir). Tüm paket birden `pytest prototype/tests/` = gtsam collection error.
- **DOĞRULAMA DÖNGÜSÜ:** her düzeltmede önce ilgili testi koştur (yeşil), düzelt, tekrar koştur. Davranış değiştiren düzeltmede testi doğru davranışa göre güncelle (buggy davranışı dondurmak için değil).
- Teknenin gerçek hareketi yalnız sahada (T0-h) doğrulanır; kod mantığı burada.

## 🔧 FAZ 18 — VİDEO DÜZELTMELERİ UYGULANMAYA BAŞLANDI (test-doğrulamalı)

Komut: `source /opt/ros/humble/setup.bash; export PYTHONPATH=/home/eyup/girdap-decision[:.../ros2_ws/src/girdap_decision]; python3 -m pytest ...`

- **✅ F11.1/F9.1 MPPI warm-start** (commit `aaf3f73`): `MPPIController.set_obstacles()` eklendi; `pipeline._rebuild_mppi` aynı config'de kontrolcüyü koruyor (yalnız engel+referans günceller), config değişince warm-start'ı taşıyor. 2 yeni test (TDD kırmızı→yeşil), 97 test geçiyor. **MPPI'ye dokunulmadı** (math sağlamdı).
- **✅ F12.2 video terminal durumu** (commit `ec7e1f5`): `Observation.mission_complete` + PARKUR*→TAMAMLANDI geçişi (dist-P1→P2'den önce). `mission_manager_node` `/girdap/mission/complete` (Bool latching) yayınlıyor, `fsm_node` tüketiyor. 3 yeni FSM testi, `mission_manager_node` ROS testi 2 geçiyor. fsm_node tam smoke edilemedi (mavros_msgs makinede yok — ortam eksiği).

**Test ortamı gerçekleri:** saf-Python çekirdek testleri numpy 2.2.6 ile çalışıyor. ROS node testleri `source /opt/ros/humble` + PYTHONPATH'e paket dizini ekleyince çalışıyor AMA: `mavros_msgs` kurulu değil (fsm/mavros/planning/telemetry node'ları import edemez), gtsam yok, scipy 1.8.0×numpy2 ABI kırık (lidar). Yani mavros'a bağlı node'lar burada kurulamaz; çekirdek mantıkları test edilir.

### ✅ 6 VİDEO DÜZELTMESİ UYGULANDI + PUSH'LANDI (2026-07-10) — 104 test geçiyor

| Bulgu | Commit | Doğrulama |
|---|---|---|
| F11.1/F9.1 MPPI warm-start | `aaf3f73` | 2 yeni test, TDD |
| F12.2 video terminal (mission_complete→TAMAMLANDI) | `ec7e1f5` | 3 FSM testi + mission_manager_node ROS testi |
| F15.1 Dosya-2 mutlak yol | `c2308a2` | TelemetryNode /tmp'den kuruldu, CSV home'da |
| F4.2 control_rate 20→10 Hz | `c2308a2` sonrası | config |
| F14.1 KILL disarm etsin | `0c7e1b6` | node py_compile (mavros yok) |
| F14.2 kasıtlı disarm≠failsafe | `0c7e1b6` | 4 çekirdek testi |
| Özet tablo dokümanda | son commit | — |

**Çekirdek matematiğe (MPPI/FSM/telemetri) DOKUNULMADI** — hepsi sağlamdı; düzeltmeler yaşam döngüsü/config/yol seviyesinde.

**Doğrulama komutu (aynen kullan):**
`cd /home/eyup/girdap-decision && source /opt/ros/humble/setup.bash && export PYTHONPATH=/home/eyup/girdap-decision:/home/eyup/girdap-decision/ros2_ws/src/girdap_decision:$PYTHONPATH && python3 -m pytest prototype/tests/test_{mppi,planning_bypass,mission_fsm,mission_manager,parkur_fsm,mavros_bridge,telemetry_logger,fusion,dynamics,local_map}.py -q` → **111 passed** olmalı (T0-f'ten sonra; eskiden 104).

### ✅ T0-f UYGULANDI + PUSH'LANDI (2026-07-10, commit `3856fb0`) — görevi FC'den oku

**En önemli kalan video işi (tek gerçek yeni-kod) bitti.** Şartname md 3.3.1(2)+5.5.2.2: görev YKİ→Pixhawk→MAVROS ile yüklenir; önceki kod yalnız `video_mission.yaml`'dan okuyordu → QGC görevi ≠ icra edilen görev (video eleme kapısı 2. madde ihlali).
- **Çekirdek (Layer 0, `prototype/mission/mission_manager.py`):** `FcMissionItem` + `fc_items_to_waypoints(items, skip_home_seq0=True)`. mavros_msgs BAĞIMSIZ → pytest. Filtreler: home(index0) atla, yalnız NAV_WAYPOINT(16)/NAV_SPLINE(82), (0,0) atla. Parkur etiketi=1.
- **Node (Layer 2, `mission_manager_node.py`):** `mission_source` param `{file, fc}`. fc modu `/mavros/mission/waypoints` (`mavros_msgs/WaypointList`, **latched QoS**) dinler; görevi YALNIZ başlamadan kurar (md 5.5.2.2), başladıktan sonra reddeder; görev yoksa FSM aktifte başlatmaz (`_started` kilit önlendi). **mavros_msgs LAZY import** (file modu + pytest mavros'suz çalışır — bu makinede mavros_msgs YOK).
- **Wiring:** `hardware.yaml mission.{mission_source,skip_home_seq0}` + `hardware.launch` (`mission_source:=fc` CLI) + `params.yaml`. **Varsayılan `file`; sahada video/yarışma günü `fc`.** `qos_profiles.latched_qos()` eklendi.
- **Doğrulama:** 7 çekirdek testi (111 toplam) + 2 ROS node testi (mavros'suz duck-typed msg ile fc callback rebuild + red + başlatma kilidi) → **4/4 node testi geçti** (sourced ROS). Denetim: `docs/kod_denetimi.md` T0-f bölümü.
- **SAHA/HIL kalan:** Jetson'a `ros-humble-mavros-msgs` kur (fc modu onsuz boş görevde kalır, çökmez); gerçek QGC→MAVROS zincirinde WaypointList'in latched geldiği + ArduRover home=index0 + komut=16 teyit; `skip_home_seq0` ters çıkarsa tek anahtar.

## 📋 YARIN YAPILACAKLAR (öncelik sırasıyla)

**A. Kalan video (T0) düzeltmeleri:**
1. ✅ **T0-f: Görevi FC'den oku (#19) — BİTTİ (commit `3856fb0`).**
2. ✅ **T0-g: Ekran-2 grafik telemetri (#20) — BİTTİ (commit `08f3fb5`, push'lu, 2026-07-10).** `csv_logger`'a `GRAPH_CSV_HEADER`+`GraphSample` (+opsiyonel `header` param, Dosya-2 header testle donduruldu); `telemetry_node` `/girdap/control/thrust` dinleyip AYRI `~/girdap_logs/grafik/grafik_<UTC>.csv` yazıyor (10 Hz = MPPI control_rate; 2 Hz alias yapardı). **F15.4 de kapandı** (hız velocity_body yoksa fusion-odom twist'ten). Doğrulama: 5 yeni çekirdek testi (116 toplam) + yeni `test_telemetry_node.py` ROS smoke (thrust 12.30/-7.00 + hız 1.000 CSV'de; toplam suite 121). Test dersi: 20 Hz timer tek `spin_once`'ı doyurur → node testi birkaç kez spin etmeli. Video günü kalan: grafik CSV'den 3 panel çizen offline matplotlib script'i (istenince).
3. ✅ **F14.3 auto_guided — BİTTİ (commit `88a06c8`, push'lu, 2026-07-10).** auto-GUIDED yalnız görev aktifken (FSM PARKUR1/2/3): `MavrosBridge.set_mission_state()`+`MISSION_ACTIVE_STATES`, `needs_mode_change` görev-aktif değilse False (varsayılan False=güvenli); node `/girdap/mission/state` dinler, PARKUR1 girişinde hemen dener. 3 yeni çekirdek testi + mevcut 3 test doğru davranışa güncellendi. **Suite artık 124** (çekirdek 119 + telemetry_node 1 + mission_manager_node 4).

**⇒ T0 (VİDEO) KOD TARAFI TAMAMEN BİTTİ (2026-07-10).** Kalan video işleri kod değil: suda 4 nokta provası (T0-h), gerçek FCU'da KILL/arm zinciri, QGC→MAVROS görev yükleme teyidi, video montajı (grafik CSV'den 3 panel çizen offline matplotlib script'i istenince yazılır).

4. ✅ **F8.1 (Faz 8'de bulunan 4. istemsiz-hareket kökü) — BİTTİ (commit `dd135b1`, push'lu, 2026-07-10).** `fusion_node` Odometry twist'ini HİÇ doldurmuyordu; koddaki "planning yalnız pozu kullanıyor" yorumu BAYATTI — `planning_node._on_odom` twist'ten `u,v,r` okuyup MPPI durumuna basıyor → MPPI her adımda araç DURUYORMUŞ sanıyordu. Düzeltme: `velocity_body` iki modda da cache + `od.twist` dolduruldu (body-frame=child_frame_id doğru); bypass'a velocity_body aboneliği eklendi. Yeni `test_fusion_node.py` ROS smoke (bypass, gtsam'sız). **Suite 125.** Bu düzeltme F15.4 hız yedeğini de anlamlı kıldı (önceden hep 0 okurdu).

**B. Kalan T1 (yarışma) denetimi (salt okuma, güvenli):**
- ✅ **Faz 6 sentetik üreteçler — BİTTİ (commit `64c51aa`, push'lu, 2026-07-10).** F6.1: bearing işaret maskesinin TAM mekanizması — `_cx_for_bearing` ters fonksiyon sahneyi fiziksel imkânsız kuruyor (soldaki duba y=+3 → bbox cx=0.95 görüntünün SAĞINA); `test_bearing_from_camera_edges_are_half_hfov` yanlış kuralı DONDURUYOR (işaret düzeltilirse kırmızı olur). Düzeltme reçetesi: işaret+üreteç+kenar testi TEK commit + üreteçten bağımsız fiziksel tutarlılık testi. F6.2: F5.1 düzeltmesi üreteç+testlerle aynı commit'te olmalı. F6.3: F5.4 (max_cluster=500 en yakın engeli siler) hiçbir sahnede ateşlenemiyor (40 nokta/duba sabit). F6.4: kamera testleri döngüsel (renkler dedektör aralığından seçilmiş — docstring itiraf ediyor). F6.5: çeldirici sahne yok (beyaz sosis, FOV kenarı, 15 m sınırı).
- ✅ **Faz 7 algı ROS node'ları — BİTTİ (commit `aa10d9b`).** F7.1 🟠: füzyon sync'i iki FARKLI stamp tabanını eşliyor (bizim buoys=yayın anı `now()`, obstacle_map=Livox tarama damgası) → sapma>slop(0.1s) ise sync SESSİZCE hiç ateşlemez, log yok. F7.2: `int(class_id)` sayısal olmayanda node'u öldürür. F7.3: LiDAR aboneliği depth=10 → ağır kümeleme (F5.3) yetişemeyince ~1 s bayat bulut işlenir; depth=1 ucuz sigorta. F7.4: F3.1/F3.2 launch sorunları hâlâ açık. F7.5: girdi kesilince algı node'ları sonsuz sessiz.
- ✅ **Faz 8 füzyon/iSAM2 — BİTTİ (commit `5296a1c` denetim + `dd135b1` F8.1 düzeltmesi).** F8.1 🔴 düzeltildi (yukarıda). F8.2 🟠: bypass pozu bayatlamaya korumasız — EKF akışı ölürse 50 Hz DONMUŞ poz (T1 watchdog). F8.3: iSAM2 grafı sınırsız (20dk≈12k anahtar, Jetson'da ölç). F8.4: ilk GPS fix hareket halinde gelirse origin kayar (pre-arm fix şartı pratikte önlüyor; "arm'dan önce fix bekle" operasyon notu). iSAM2 çekirdeği SAĞLAM (GPS heading-serbest prior hilesi doğru, dokunma).
- ✅ **Faz 10 RRT* — BİTTİ (commit `18171f4`).** F10.1 🔴: `rrt.plan()` start/goal engel-payı (r+0.3m) içinde YA DA bounds dışındaysa **ValueError fırlatıyor, kimse yakalamıyor** → engel callback'inden (10 Hz) tetiklenen replan planning_node'u görev ortasında ÖLDÜRÜR (geçitte dubaya 0.45-0.7 m yaklaşınca gerçekçi). Düzeltme: `_global_replan` try/except + eski ref_path'i koru. F10.2: F4.4 KESİNLEŞTİ — bounds [0,200]² odom-merkezli çalışmaz (negatif çeyrek → ValueError → aynı ölüm); dinamik start+goal zarfı ya da [-100,100]². F10.3: `_nearest_idx` O(n²) Python fill ≈1-2 s/plan, kontrol thread'ini bloklar (D3 ölçümü). ÇEKİRDEK MATEMATİK SAĞLAM: rewire cycle riski çürütüldü (maliyet tutarlılığı + üçgen eşitsizliği), informed elips doğru — dokunma.
- Kalan denetim: #15 test paketi (Faz 16) · #16 doküman (Faz 17).
- **Faz 18-T1 düzeltme listesi (öncelik sırası):** F10.1 (try/except — en ucuz/en ölümcül) · F10.2 (bounds) · F5.1 (lidar_height_m, üreteç+testlerle AYNI commit, F6.2) · bearing işareti (işaret+üreteç+kenar testi TEK commit + fiziksel tutarlılık testi, F6.1) · F7.1 (stamp sözleşmesi + sync watchdog) · F7.3 (lidar depth=1) · F7.2 (int(class_id) try/except) · F8.2 (bypass poz bayatlık watchdog) · F5.3 (scipy connected_components + voxel) · F6.3 sahneleri.

**C. Eyüp'ün/donanım ekibinin işleri (bkz. [[bekleyen-girdiler-isaret]]):**
- Livox montaj yüksekliği (F5.1 blocker), NN Archive sınıf sırası (Jetson'da getClasses oku), base_link tanımı.
- `Gazebonew.pt` harici+bulut yedeği ([[yolo-model-durumu]]).
- Suda 4 nokta provası (T0-h).

**D. Arkadaş dönünce:** fork'taki düzeltmeleri PR ile upstream'e ([[girdap-decision-entegrasyon]] fork mekanizması). `docs/kod_denetimi.md` tüm bulguları + düzeltmeleri kanıtıyla içeriyor.

**Kural hatırlatması:** [[sartname-once-kural]] — her düzeltmeyi şartname maddesine bağla; çekirdek math'e dokunma; TDD (önce kırmızı test); erken teşhisi kanıt çürütürse geri al.

## ✅ FAZ 16 BİTTİ (2026-07-11) — test paketi denetimi + F16 düzeltmeleri

Commit `b9ccae5` (denetim) + `e2c853f` (düzeltme), İKİSİ DE PUSH'LU (fork+yedek senkron `e2c853f`).

- **F16.1 🔴 (düzeltildi):** ROS source'lu ortamda `pytest prototype/tests/` KOŞAMIYORDU — launch_testing plugin'i toplama sırasında her modülü import ediyor, tek kırık import (gtsam/scipy/vision_msgs) TÜM suite'i öldürüyor; ignore'la da "1 skipped exit 0" = yanlış-yeşil SIFIR koşu. Düzeltme: pyproject `addopts="-p no:launch_testing -p no:launch_ros"` (repo'da launch testi yok; plugin adları egg-info'dan, `no:launch_testing_ros` YANLIŞ ad).
- **F16.2 🟠 (düzeltildi):** kapılama tutarsızdı — vision_msgs çıplak import (2 dosya), scipy ABI ValueError + matplotlib ABI AttributeError importorskip'i deliyor (3 dosya, elle try/except→module-level skip), parkur_fsm_node skip nedeni yanıltıcıydı (gerçek neden fsm_node:66 mavros_msgs).
- **F16.3 🟠 (düzeltildi):** test_planning_pipeline'ın modül-düzeyi FusionPipeline (→gtsam) import'u 7 kapalı-döngü testini rehin tutuyordu; import tek testin içine + importorskip.
- **YENİ TABAN ÇİZGİSİ: ignore'suz `pytest prototype/tests/` (ROS source'lu) = 156 passed, 8 gerekçeli skip, 0 error, ~17 sn.** Kapalı-döngü MPPI testleri (reaches_goal/avoids_obstacle) bu makinede İLK KEZ koştu ve GEÇTİ. Eski 131'lik dosya-listesi komutu artık GEREKSİZ — düz dizin koşusu yeter. (156≠195 fonksiyon: parametrize + modül skip'leri.)
- **F16.4 🟠 AÇIK:** CI YOK (.github/workflows yok), CLAUDE.md "GitHub Actions var" diyor → Faz 17.
- **F16.5 🟡 AÇIK — ÖNEMLİ DÜZELTME: F12.1 UYGULANMAMIŞTI!** `fsm_node.py:89` `last_waypoint_xy=[0,0]`'ı hiçbir şey yazmıyor → başlangıçta sahte P1→P2. Önceki memory "video paketi içinde" sayıp kalan listeden DÜŞÜRMÜŞTÜ — yanlış. Video'yu F12.2 kurtarıyor (mission_complete her PARKUR*'dan TAMAMLANDI'ya gider, yalnız MPPI ağırlık profili sahte P2'ye geçer) ama YARIŞMA P1→P2 kapısı bozuk. Faz 18-T1 listesine GERİ ALINDI.
- **F16.6 🟡 AÇIK:** rrt_star (477 satır) doğrudan birim testsiz — T1'de ~5 deterministik test.
- **F16.7 ✅:** paket nitelikli, Faz 6 üreteç bulguları dışında YENİ maskeleme YOK.

## ✅ FAZ 17 BİTTİ (2026-07-11) — doküman denetimi + düzeltmeleri (commit `48e184f`, push'lu)

**DENETİM FAZLARI 1-17 TAMAMEN BİTTİ.** F17.1-F17.8 bulundu ve AYNI commit'te düzeltildi: CLAUDE.md RAL 2008/1003→2003/1026 (🔴, AI asistanlar yanlış rengi bellemesin) · CI iddiası→"kurulmadı" · MPPI "50Hz GPU"→gerçek 10Hz CPU+CUDA hedefi · cpp/ "planlandı-yazılmadı" · configs/→config/ yolu · "Dosya 1a/1b" iç-adlandırma dipnotu · GTSAM 4.2 vs >=4.3a0 notu · FSM şemasına F12.2 yolu · README'ye ROS source notu. docs/KTR teslim edilmiş rapor — DOKUNULMADI.

## 🔓 ORTAM TAMAMLANDI (2026-07-11) — eksik bağımlılıklar sudo'suz kuruldu

- **pip --user:** scipy 1.15.3 + matplotlib 3.10.9 (numpy2 ABI düzeldi) + **gtsam 4.3a0** (`--pre`, requirements ile birebir; Pose2 doğrulandı).
- **Kaynaktan colcon:** `~/girdap_deps_ws` → geographic_msgs + vision_msgs + **mavros_msgs** (ros2 branch'ler, shallow clone, 79 sn build). sudo YOK diye apt yerine bu yol.
- **YENİ TABAN: 202 passed / 0 skip / ~29 sn.** İlk kez koşanlar: gtsam füzyon (3), lidar (15), viz (11), algı node (11), parkur_fsm_node (5), e2e zinciri. fsm_node artık import edilebilir → F12.1 node-düzeyi TDD mümkün.
- Tek kozmetik uyarı: Axes3D (sistem+pip matplotlib karışımı) — testleri etkilemiyor.

## ✅ 2026-07-11 (öğleden sonra oturumu): Ekran-2 aracı + CUDA planı

- **Commit `bdbbc9d`:** `docs/mppi_cuda_plani.md` (önceki oturumda yazılıp açıkta kalmıştı — CuPy seçimi, Faz 0/A/B, float32 notu, D3 protokolü).
- **Commit `e7e87e6` — T0 VIDEO MONTAJ ARACI BİTTİ:** `prototype/viz/ekran2.py` + `scripts/run_ekran2.py` + 10 test (TDD). Grafik CSV → statik PNG + zaman imleçli MP4 (FFMpegWriter, ffmpeg bu makinede var). Kullanım: `python scripts/run_ekran2.py [--csv ...] [--mp4] [--t0/--t1 kırpma]`; varsayılan girdi `~/girdap_logs/grafik` en yenisi, çıktı `~/girdap_logs/viz/`. 60 s sentetik CSV ile uçtan uca doğrulandı (PNG görsel olarak incelendi: NaN boşlukları sahte sıfır çizmiyor, heading derece, sarım kırma çalışıyor). **SUITE: 226 passed / 0 skip.**
- Eyüp kararı: **NN Archive (tar.xz) üretimi video SONRASINA ertelendi** (videoda algı yok) — [[yolo-model-durumu]] güncellendi.
- **Commit `798ff4d` — F5.4+F6.3 KAPANDI:** `cluster_points` üst sınırı aşan kümeyi artık `_split_oversized` ile `split_cell_m=1.0` XY ızgarasına bölüyor (kayıpsız, daire ≤ ~0.71 m, min alt-kümelere uygulanmaz); param 4 yerde kablolu (node/hardware.yaml/params.yaml/launch). Yeni sahneler: `scene_yakin_duvar` (>500 voxel ön koşul testli) + `scene_uzak_seyrek_duba` (min_cluster_size kör noktası belgelendi). TDD 4 test kırmızı→yeşil; x86 20k duvar sahnesi ort. 34 ms. ⚠ "tekne kendisi" gerekçesi için **min_range değerlendirmesi F5.1 paketine bağlandı** (kod yorumunda + denetim dokümanında). **SUITE: 230 passed / 0 skip, son commit `798ff4d` push'lu (fork+yedek).**

## ✅ MPPI CUDA Faz 0 (xp backend) ÇEKİRDEĞİ BİTTİ — commit `9257fb4` push'lu (2026-07-11)

`mppi.py` tamamen xp-soyutlandı (`MPPIConfig.backend={numpy,cupy,auto}`, varsayılan auto; numpy yolu float64 BİT-BİREBİR, cupy yolu float32; `_resolve_backend` + `_load_obstacles` + `_as_numpy` + `_sample_noise` (parite testi monkeypatch noktası); step giriş/çıkışı host sözleşmeli; property'ler host'a çevirir). 6 yeni test: backend çözümü, determinizm, host-çıktı, numpy≡auto bit-eşitlik, geçersiz→ValueError, numpy↔cupy parite (**GPU'suz makinede gerekçeli skip — Jetson'da koşacak; 0-skip taban BİLEREK 240/1 oldu**).

✅ **TAM SUITE TEYİTLİ: 239 passed / 1 gerekçeli skip (cupy parite, Jetson'da koşacak)** — yeni resmi taban.

**✅ KALAN 2-3 KAPANDI (2026-07-11, commit `f93a3be` push'lu fork+yedek):** `scripts/bench_mppi.py` yazıldı (--backend numpy|cupy|auto, --K/--T/--steps/--warmup; ort/min/maks ms + ilk→son yarı sürüklenme + §5 20/50 Hz eşik raporu; referans bilerek düz çizgi start→goal, n_ref=114; 3 TDD testi `test_bench_mppi.py`) + plan/denetim dokümanlarına Faz 0 ✅ notları. Uçtan uca koşuldu: **x86 numpy ort 99.1 ms/iter (K=1000, T=50)** = denetimin "~100 ms CPU" tespitinin bağımsız teyidi; `auto` GPU'suz makinede numpy'a düştü. **SUITE: 242 passed / 1 gerekçeli skip.**
**KALAN — yalnız Jetson günü (Faz A):** `pip install cupy-cuda12x` → parite testi gerçek koşar → `nvpmodel -m 0 && jetson_clocks` → `bench_mppi.py --backend numpy` vs `--backend cupy` (+ `--steps 600` sürüklenme) → control_rate kararı.

## 🔥 JETSON DONANIM GÜNLERİ (2026-07-11 gece + 2026-07-12) — Jetson'daki Claude çalıştı, PC'den incelendi 2026-07-12

Jetson'a Claude kuruldu; 14 commit push'landı (fork). PC klonu `git pull` ile `eae9d9d`'ye çekildi (2026-07-12). **Yedek repo ZATEN senkron (`eae9d9d` teyitli); "c742284 push'lanmamış" notu da bayattı (origin'de var). İki senkron maddesi de KAPALI.** Ham günlükler: `docs/donanim_gunlugu_2026-07-12.md` + `docs/dogrulama_matrisi.md` (YENİ — bileşen bazında canlı/suite/bekleyen tablosu) + girdap-ida-algi `docs/jetson_gunlugu_2026-07-11.md`.

**✅ Kapananlar:**
- **Suite Jetson'da teyit: 246/1 → (F-A.1/A.2 + Faz B + F-L.1 sonrası) YENİ TABAN 250 passed / 2 gerekçeli skip.**
- **MPPI CUDA Faz A+B BİTTİ (`c612fb0`+`1558ead`): step 302→9.0 ms (~33×, tavan ~112 Hz), 20 VE 50 Hz kriterleri geçti, 600 adım sürüklenme −1.0%.** F-A.1: cupy Generator'da `.normal()` yok → `standard_normal*sigma` ortak yolu (parite testinin monkeypatch'i maskelemişti — maskeleme deseninin yeni örneği). Faz B: rollout tek RawKernel (RK4, fizik batch ile işlem-sırası aynı, fused≡numpy 1e-3 testli; derlenemezse WARN+jenerik yola düşüş). **CUDA planında kalan iş yok; control_rate 10 Hz rahat, 20 Hz mümkün (sahada karar).**
- **F-L.1 (🔴, düzeltildi `d9778fe`):** gerçek Livox bulutu karışık dtype (float32+uint8+float64) → `read_points_numpy` assert'le İLK gerçek mesajda node'u öldürüyordu (sentetikler hep `create_cloud_xyz32` — maskeleme). Düzeltme `read_points`+`structured_to_unstructured`, gerçek şemayı taklit eden testle. Canlı: node 9.98-10.07 Hz obstacle_map, kümeleme 38.6/52.3 ms @20k nokta (F5.3 Jetson teyidi).
- **F8.3 KAPANDI:** iSAM2 20 dk sim (11.4k anahtar): RAM +30 MB, flush p95 21 ms < 100 ms bütçe → marginalization GEREKSİZ (>60 dk'da zorlar, yarışma dışı, bilinçli önlemsiz).
- **F-L.2 revize:** Livox stamp +0.2 s ofsetine rağmen sync ATEŞLİYOR (90/20 sn — 10 Hz yoğun akış slop içinde aday buluyor; "hiç ateşlemez" erken teşhisi deneyle geri alındı). Gerçek etki ~0.2 s zaman kayması (~0.06 rad < tol 0.15) → T1, düşük öncelik (restamp en temiz). **Bearing işaret düzeltmesi (`e66cb40`) gerçek LiDAR verisiyle İLK KEZ kanıtlandı: nişanlı bbox 99/99 eşleşti, class-99 koruması ✓.**
- **Masa testleri:** M1 GEÇTİ (TELEM2 çapraz kablo sonrası, `ttyUSB0:57600`, IMU 10.4 Hz) · M3 FSM zinciri PASS · M4 fc görev PASS (mission/push 5 item→4 wp, skip_home ✓ latched ✓) · OAK SUPER+12 FPS PASS (USB-C ucunu 180° çevirme hilesi) · ortak yük PASS (CPU %30, 53°C, 7.4 W) · FCU'suz tam stack boot PASS (10 node ayakta; heartbeat-KILL doğru tetiklendi) · MPPI gerçek planning_node'da sahte beslemeyle 9.90 Hz PASS (M8 ön) · M7 ön: 3 kayıt da boot'tan üretiliyor.
- M5 KISMİ: GPS'siz GUIDED'ı ArduPilot REDDEDİYOR (bilinen sınır) → GUIDED tetiği açık alanda (su gerekmez). M2 ertelendi (QGC laptop yok; **QGC'nin ARM64 Linux sürümü YOK — Jetson'a kurulamaz, laptop şart**). M6/M8 koşulamadı.

**🔴 YENİ AÇIK BULGULAR (sonraki oturumun 1. işi):**
1. **F-M.1 planning OOM:** GPS fix yok → `home_ref` (0,0) → FC'nin 40°K/29°D wp'leri ~4400 km ENU → maliyet tensörü 92 GB → cupy OOM, node ölür. Sahada fix'le olmaz AMA korumasız (yanlış koordinat sahada da öldürür). TDD planı: fix/home_ref yokken başlatma reddi + n_ref/hedef-mesafe tavanı (>10 km = red).
2. **F-M.2 kasıtlı disarm yine FAILSAFE sanıldı** gerçek FCU'da (F14.2 `_expected_disarm` bayrağı mock'ta geçiyordu; yarış koşulu şüphesi). Video güç-kesme provasını İLGİLENDİRİR (md 3.3.1/4).
3. **🔴 OLAY (kapanış sonrası): FC hafızasındaki SAHTE görev (M4'ün 4400 km wp'leri) RC/AUTO ile KENDİ KOŞTU, motorlar tam güç** — yığın tamamen kapalıyken; pervanesiz kural hasarı önledi. Tetik muhtemel RC mod kanalı (CH5 atlamalı) + `BRD_SAFETY_DEFLT=0` (masada yazıldı). **Bir sonraki güç verişte (PERVANESİZ) ZORUNLU: (1) `/mavros/mission/clear` — sahte görev FC'de durduğu sürece her AUTO = tam-yol kaçış; (2) `BRD_SAFETY_DEFLT=1` GERİ; (3) FC ekibi RC mod kanalı/FLTMODE düzeni; (4) batarya takılı bırakılmaz.**
4. FC ekibine gidenler: Pixhawk **USB-C soketi arızalı şüphesi** (descriptor -32, çapraz test reçetesi günlükte) · RC kalibrasyonu QGC'yle baştan (masada trim'ler elle yazıldı; CH2 üst uçta dinleniyor — RCMAP kontrol) · SERIAL2/failsafe param dökümü olcum_formu'na.

**✅ F-M.1 + F-M.2 DÜZELTİLDİ (2026-07-12 PC oturumu, TDD, commit `dff52af`+`3931220` push'lu fork+yedek):**
- **F-M.1 (3 katman):** `_on_gps` (0,0) null-island'ı yok sayar · başlatmada fix yoksa WARN+bekle, en uzak hedef > `max_target_distance_m` (yeni param, 10 km, params.yaml kablolu) ise ERROR+red (`farthest_waypoint_m` yeni Layer-0 fn; `_started` latch'lenmez) · `MPPIConfig.max_ref_points=2048` set_reference tavanı (kabalaştır+WARN, ValueError DEĞİL — F10.1 dersi). 6 test önce kırmızı.
- **F-M.2 kök neden:** yarış koşulu DEĞİL — `_on_monitor`'daki `_was_armed = _was_armed or armed` latch'i disarm kenarını her tick yeniden görüyordu, tek atımlık `_expected_disarm` ilk tick'te tükenince 2. tick sahte KILL. Düzeltme: `_was_armed = armed` (kenar takibi). Yeni `test_mavros_bridge_node.py` 3 test (kırmızıda masa logu birebir üredi; komutsuz disarm hâlâ KILL — regresyon bekçisi). M6'da gerçek FCU teyidi kaldı.
- **YENİ SUITE TABANI: PC 257 passed / 4 skip (GPU'suz); Jetson beklenen 259/2.**

**Kalan saha sırası:** açık alanda M5 tam→M6 KILL→M7 dolu→M8 D3; QGC laptop gelirse M2+gerçek Plan Upload. (Güç verme öncesi OLAY aksiyonları yukarıda — hâlâ geçerli!)
**Commit `093de2b` (SON):** `docs/fc_parametre_onerileri.md` — FC ekibine doldurmalık öneri tablosu (resmî ArduPilot Rover dokümanından araştırıldı 2026-07-12: MODE1-6'da AUTO olmasın, INITIAL_MODE=HOLD, BRD_SAFETY_DEFLT=1 geri, FS_ACTION=2 Hold, FS_GCS_ENABLE karar+SYSID_MYGCS testi, FS_CRASH_CHECK⚠P3 kamikaze çelişkisi, batarya failsafe önerisi, 5 adımlı pervanesiz doğrulama) + video runbook yamaları (F-M.1 GPS-fix adımı §4/2, FC görev hafızası temizliği §2, F-M.2 notu + "QGC'den disarm sahte FAILSAFE logu basar — servis kullan" uyarısı §5/2). Upstream kontrolü: arkadaş hâlâ `d4ce88b`'de, yeni push YOK.
**Doküman commit'leri `3e0f7ae` + `909d9db` (SON, push'lu fork+yedek):** runbook'a "M0-ÖNCESİ güç verme güvenlik bloğu" (OLAY aksiyonları adım adım) + M0 taban 259/2 (git pull şart!) + M6/2'ye F-M.2 teyit notu + doğrulama matrisine F-M.1/F-M.2 satırları + **`docs/pc_gunlugu_2026-07-12.md`: oturum özeti + öncelikli yapılacaklar listesi (Eyüp'ün istediği "orda açarım" notu — yeni oturum/Jetson önce bunu açabilir)**. CI her iki fix commit'inde YEŞİL (Actions teyitli).

## 🎬 VİDEO 20-FAZLI DENETİM TURU (2026-07-13 gece) — ✅ 20/20 FAZ TAMAM

Eyüp isteği: "video kodlarını tane tane, şartnameye tam uyum + QGC 4 nokta görevi, 20 faz."
**Yeni doküman: `docs/video_denetimi.md`** (şartname md 3.3 ↔ kod eşleme matrisi + F-V bulguları).
Şartname s.9-13 birinci kaynaktan YENİDEN okundu (Ekran-2b "heading/yaw AÇISI isteği" kesinleşti).

**Biten fazlar (son commit `766277d` push'lu fork+yedek, working tree temiz):**
- Faz 1-2: taban 257/4 teyit + matris dokümanı (`2b5798c`).
- **Faz 3 ✅ F-V.1 DÜZELTİLDİ (`190c54e`, TDD):** telemetry_node `/girdap/mission/current_target` aboneliği; `yon_setpoint = atan2(y,x)` AÇI; `_on_setpoint` yalnız hız. ekran2.py değişmedi.
- Faz 4-5 (telemetri+csv_logger): temiz; yeni bulgular **F-V.2 🟡** (TAMAMLANDI'da setpoint cache donuk kalır — düzeltme: aktif-değil state'te None→CSV boş→NaN boşluğu) + **F-V.3 🟡 T1** (speed_from_body kalıcı latch).
- Faz 6-7 (mission fc): node+çekirdek temiz; `test_fc_dikdortgen_video_senaryosu` eklendi (md 3.3.1(2)+(3) birebir).
- Faz 8-12 (`a4000a4`): fsm/bridge/planning/F-M.1/config hepsi yerinde. **YENİ AÇIK: F-V.4 🟠 video_mission.yaml'da 5. nokta "P1_return" — şartname "4 nokta + dönüş MANUEL" ile ÇELİŞİR (şablondan çıkar + CLAUDE.md/runbook yorumları düzelt + QGC talimatına "dönüş noktası EKLEME"); F-V.5 🟠 = F3.3 hâlâ açık: hardware.launch:143 `except Exception: pass` — YAML bozuksa SESSİZCE yarışma modu (isam2/rrt=True) açılır; düzeltme: gürültülü stderr uyarı. F-V.6 🟡 T1: fc modunda parkur etiketleri dosyadan (FC görevi etiket taşımaz) — yarışma günü senkron notu.**
- **Faz 13 ✅ (`766277d`): test_video_e2e.py — GERÇEK node grafiğiyle uçtan uca:** sahte QGC WaypointList(latched)+FCU+GPS → mission(fc) → FSM(GUIDED kenarı) → planning(bypass, küçük MPPI) → 4 köşe varış [0,1,2,3] → TAMAMLANDI → **thrust [0,0]** → grafik CSV açıları (0/π‑2/π) + Dosya-2 TAMAMLANDI. planning_node'a `**node_kwargs` eklendi. **SUITE YENİ TABAN: 260 passed / 4 skipped.**

**FAZLARIN DÖKÜMÜ:**
- ~~Faz 14~~ ✅ (`2901239`): ekran2 uçtan uca doğrulandı — 150 s gerçekçi profil PNG görsel incelendi (3 sinyal + açı + sarım + boşluklar ✓), MP4 10 s pencere = 10.03 s imleç gerçek zamanlı ✓.
- ~~Faz 15~~ ✅ (`da+push`): runbook §6/3 Şekil-1 ASCII yerleşimi + Ekran-3 aşama-içerik eşlemesi + senkron köşe-dönüşü testi + §7 kontrol maddeleri. ~~Faz 16~~ ✅ (girdap-video `3945428`): kontrol-listesi.md — Şekil-1 ASCII + dikdörtgen şartı + "4 nokta, dönüş noktası EKLEME (F-V.4)" + Ekran-3 aşama içeriği + YKİ senkron testi + "yalnız YouTube" netleştirmesi. ~~Faz 17~~ ✅ (`fc606fc`): runbook tam geçişi — §3/4 ve §4/6 "kapanış noktası" KALDIRILDI (F-V.4 doküman ayağı), ARM→GUIDED 1-2 sn bekleme, güç-kesmede beklenen KILL logu notu; §2 OLAY bloğu + §5/2 disarm-servis notu zaten tamdı.
- ~~Faz 18~~ ✅ (`d7f9be6` push'lu): F-V.2 (setpoint sütunları yalnız görev aktifken yazılır — yazma-anı kapılaması `_mission_active`, TDD; F-V.1 testi PARKUR1 yayınlayacak şekilde güncellendi) + F-V.4 (video_mission.yaml'dan P1_return KALDIRILDI, 4 nokta; CLAUDE.md video bölümü + mission_manager_node docstring hizalı; 'yalnız PROVA, çekim fc' notu) + F-V.5/F3.3 KAPANDI (hardware.launch fallback artık stderr'e gürültülü uyarı basar; yeni test_hardware_launch_config.py, share-dizini testi kurulumsuz ortamda gerekçeli skip). **SUITE YENİ TABAN: 262 passed / 5 skipped.**
- ~~Faz 19~~ ✅ ÇİFT-BAKIM SENKRON: girdap-ida karar subtree `d7f9be6`e çekildi (`da3f84a`; önce izlenen logger __pycache__ hijyeni `96b9ba9` — pull'u kirli ağaçla blokluyordu) + girdap-video karar BİLİNÇLİ güncellendi c77dca3→d7f9be6 (`0623820`) + README dondurma notu `e90d184`. **CI her iki repoda YEŞİL** (girdap-decision d7f9be6 + girdap-ida çift-bakım koşusu 3m50s).
- ~~Faz 20~~ ✅ (`15dc238`): video_denetimi.md §6 kapanış bölümü — **KOD İŞİ KALMADI**; kapanış commit'i çift-bakımla girdap-ida + girdap-video'ya da taşındı (video README dondurma noktası `15dc238`). **BİLEŞEN TEST KOŞUSU (Eyüp isteği, girdap-video `72cc03f`): `girdap-video/testler/video_testleri.sh` — 12 bileşeni ayrı ayrı koşturur (csv_logger→…→e2e), PASS/FAIL özet, `--tam` tam suite, `GIRDAP_KOD=~/ros2_ws/src/girdap-decision` ile Jetson'daki ayrı klona karşı; PC'de 12/12 ✅ teyitli; kontrol-listesi.md §0 = 'çekim öncesi Jetson'da 12/12 yeşil' şartı. Boyut: betik 106 + README 52 = 158 satır; turun tüm kod/doküman diff'i (c77dca3→15dc238, girdap-decision): 13 dosya, +743/−34.** Kalan her şey SAHA: (1) 🔴 FC güvenlik (pervanesiz mission clear + BRD_SAFETY_DEFLT=1) → (2) Jetson sıfırla+kur → (3) açık alan M5/M6/M2 → (4) suda prova → ÇEKİM (dış kamera — İDA bakış açısı md 4.1 gereği zaten yasak) → montaj Şekil-1 → 20.07 yükleme. T1 notları: F-V.3, F-V.6, F15.3/F15.5.

## 🎬 VİDEO DERİN DENETİMİ (2026-07-13 akşam — önceki oturum; F-V.1 reçetesi YUKARIDA UYGULANDI)

Eyüp isteği: "video dosyalarını ince ince incele, şartnameye detaylı bak — 8 gün kaldı."
Şartname s.11-12 birinci kaynaktan YENİDEN okundu + video kod zinciri satır satır doğrulandı.

### Şartnameden yeni yakalanan ayrıntılar (runbook/kontrol-listesine İŞLENECEK)
1. **Şekil 1 yerleşimi KESİN:** Ekran-1 ÜST-SOL, Ekran-2 ALT-SOL, Ekran-3 SAĞ (büyük dikey).
   Montaj bu yerleşime uymalı — girdap-video/kontrol-listesi.md'de yerleşim YOK, eklenecek.
2. **md 3.3.1(2): 4 nokta DİKDÖRTGEN oluşturacak** (runbook §3.4'te var ✓, kontrol listesine ekle).
3. **Ekran-3 içeriği aşamaya göre değişir** (RC kumanda / suda hareket / iç görüntü) ve
   "İDA'nın görev yaptığı aşamada YKİ ekranı ile İDA'nın hareketleri SENKRON görülmelidir".
4. Teslim: ≥720p, 2-5 dk, YouTube LİSTE DIŞI (başka platform kabul edilmez), link KYS'ye,
   "linkte sorun = eleme". Tablo-3 adı: "Sistem Kabiliyeti Videoları" 21.07 17:00.

### ✅ Doğrulanan zincir (kod satırlarıyla)
- Runbook (video_gunu_runbook.md) sağlam ve şartnameyle uyumlu; §0 eşleme tablosu doğru.
- start_on_mode GUIDED default (fsm_node:102) ✓ · mission_source hardware.yaml'da "file",
  video günü CLI `mission_source:=fc` (runbook §3/2 ✓) · hardware.yaml video modunda
  (use_isam2/rrt false) ✓ · GRAPH_CSV_HEADER 7 sütun: zaman,hiz,hiz_setpoint,heading,
  yon_setpoint,thrust_sol,thrust_sag ✓ · telemetry_node 10 Hz grafik timer ✓ ·
  thrust /girdap/control/thrust'tan ✓ · hız body-vel + odom yedeği (F15.4) ✓ ·
  heading mavros IMU ENU yaw ✓ · ekran2.py 3 panel + sarım kırma + zaman imleçli MP4 ✓.
- current_target = PoseStamped, position.x=DOĞU ofset, y=KUZEY ofset (araç-göreli ENU;
  mission_manager_node:343-349) — IDLE/COMPLETE'te yayın durur.

### 🔴 F-V.1 — YENİ BULGU (AÇIK, sonraki oturumun 1. işi; TDD reçetesi hazır)
**yon_setpoint bir AÇI DEĞİL:** telemetry_node._on_setpoint (:179-181) cmd_vel'den
`angular.z` alıyor = planning_node:261 `(u_r−u_l)/I_z` = **yaw HIZI (rad/s)**. Şartname
Ekran-2b "heading/yaw AÇISI isteği" istiyor. ekran2.py:90 bu değeri derece sanıp heading
paneline çiziyor → panel anlamsız/yanıltıcı (heading ° ile rate karışık) = video kriter riski.
**Düzeltme (KARAR VERİLDİ, minimal):** telemetry_node'a `/girdap/mission/current_target`
aboneliği ekle; `self._yaw_sp = atan2(pose.position.y, pose.position.x)` (ENU rota açısı —
heading ile aynı konvansiyon, bypass'ta kontrolcünün gerçekten gitmeye çalıştığı yön);
`_on_setpoint` yalnız `self._speed_sp = msg.linear.x` günceller (angular.z artık yazılmaz).
Dosya-2'nin yon_setpoint'i de böylece md 4.2'ye uygun AÇI olur. ekran2.py DEĞİŞMEZ
(zaten açı bekliyor). **TDD sırası:** (1) test_telemetry_node.py'ye kırmızı test — helper
`/girdap/mission/current_target` PoseStamped(x=3,y=4) yayınlar → grafik CSV yon_setpoint
== "0.927" (atan2(4,3), 3 ondalık); mevcut testlerle çakışma YOK (dosya okundu: yalnız
thrust/hız assert'leri var). (2) düzeltme → yeşil. (3) TAM suite (taban 257/4).
(4) Commit+push girdap-decision (kaynak repo). (5) **ÇİFT-BAKIM:** girdap-video ve
girdap-ida'da `git subtree pull --prefix=karar /home/eyup/girdap-decision main`.

### Sonraki oturumun sırası
1. F-V.1 düzeltmesi (yukarıdaki reçete) + kod_denetimi.md'ye F-V.1 bölümü.
2. Runbook §6/3'e Şekil-1 yerleşimi + Ekran-3 aşama/senkron notu; girdap-video
   kontrol-listesi.md'ye yerleşim + dikdörtgen + Ekran-3 maddeleri.
3. Memory güncelle + memory repo push (bu oturumda push denenecek; Bash
   sınıflandırıcısı geçici kapalıydı, başarısızsa yeni oturumda push et).
4. 8 GÜN PLANI (taslak): 14.07 Jetson sıfırla+kur (JETSON_REHBERI.md) + FC güvenlik
   (pervanesiz mission clear + BRD_SAFETY_DEFLT=1) → 15-16.07 açık alan M5/M6 + M2
   (QGC laptop) + RC kalibrasyon + RFD kanal → 17-18.07 suda tam prova (çekimsiz) →
   19.07 ÇEKİM → 20.07 montaj+yükleme+link testi (hedef: 20'sinde KYS'de) → 21.07 tampon.

## Devam etme talimatı (yeni oturum için) — GÜNCEL: 2026-07-11

**Son commit `ae7af84` (push'lu, working tree temiz, fork+yedek senkron). Suite: 242 passed / 1 gerekçeli skip (cupy paritesi, Jetson'da koşacak) — komut: `source /opt/ros/humble/setup.bash && source ~/girdap_deps_ws/install/setup.bash && export PYTHONPATH=/home/eyup/girdap-decision:/home/eyup/girdap-decision/ros2_ws/src/girdap_decision:$PYTHONPATH && python3 -m pytest prototype/tests/ -q` (deps workspace'i source'lamayı UNUTMA; unutulursa 8 test gerekçeli skip'e düşer, hata değil).**

**📋 VİDEO GÜNÜ RUNBOOK'U (2026-07-11, `f34c142`):** `docs/video_gunu_runbook.md` — md 3.3.1 eşlemeli saha prosedürü (hazırlık→QGC fc görev→arm/başlat→gösterimler→Ekran-1/2/3 montaj→yayın kontrolü).

**✅ T0-j KAPANDI (2026-07-11, commit `c696989` = SON COMMIT, push'lu fork+yedek):** md 3.3.1(3) başlatma boşluğu (runbook yazılırken bulundu: start yalnız ROS servisi, RC dinleyen kod yok, WiFi yasak→SSH yok) Eyüp'ün onayıyla Seçenek A ile kapatıldı: **fsm_node `start_on_mode` parametresi (varsayılan GUIDED)** — BEKLEMEDE'de operatör QGC'den modu GUIDED'a çevirince `request_start()`. KENAR tetikli: boot'ta zaten GUIDED ise arm etmek BAŞLATMAZ (önce ARM sonra mod→GUIDED; gerekirse HOLD'a alıp geri dön); `""`=kapalı. F14.3 sayesinde görev-öncesi GUIDED = kesin operatör komutu. ARM için kod gerekmez (FSM armed'ı /mavros/state'ten okur, fsm_node:311). TDD: 4 yeni test (çekirdek kırmızı→yeşil + 3 guard), params.yaml kablolu, runbook §1 güncel. **SUITE: 246 passed / 1 gerekçeli skip; CI Actions yeşil (210/9).** Kalan saha teyidi: masa testinde QGC mod değişimi → "YKİ mod komutu … görev başlatıldı" logu; suda prova.

**✅ F16.4 CI KURULDU (2026-07-11, commit'ler `2a8139e`+`c78f95a`+`ae7af84`):** `.github/workflows/ci.yml` — ROS'suz çekirdek job (ubuntu-latest, Python 3.10 Jetson paritesi, requirements.txt + ffmpeg, `pytest prototype/tests/ -q`). **Gerçek Actions koşusu YEŞİL: 210 passed / 9 gerekçeli skip (8 rclpy-kapılı node modülü + 1 cupy), lokal ROS'suz koşuyla birebir.** İlk koşu 209/10'du — runner'da ffmpeg yoktu, Ekran-2 MP4 testi (T0 aracı) skip'e düşüyordu → ffmpeg adımı eklendi. Yanlış-yeşil koruması: 0 test toplanırsa pytest exit 5 → job kırmızı. Push erişimli ortakların (vistastris/yahyaseha) değişikliklerine karşı bekçi.

**✅ F12.1 DÜZELTİLDİ (2026-07-11, commit `788c46e`):** TDD — yeni `test_fsm_node.py` 3 testle bug'ı kırmızı yakaladı (sahte geçiş video senaryosunda bile). Düzeltme fsm_node'da (çekirdek FSM'e dokunulmadı): (a) `last_waypoint_xy` [0,0]="ayarlanmamış" → mesafe hesaplanmaz; (b) gerçek tetik `/girdap/mission/waypoint_reached` index == parkur-1 son index → dist=0 (waypoint-index+etiket tabanlı, yeni topic yok); param elle verilirse odom yolu yedek (testli). params.yaml yalan yorumu + F4.5 RAL yorum etiketleri (10 yer) da düzeltildi.
**✅ F16.6 DÜZELTİLDİ (commit `78aa19e` = SON COMMIT):** `test_rrt_star.py` 6 deterministik tel (F10.1 ValueError sözleşmesi, best_cost=yol uzunluğu rewire teli, seed determinizmi, ulaşılamaz goal→None...). **SUITE: 212 passed / 0 skip.**

**✅ F5.3 DÜZELTİLDİ (commit `a6aae64` + doküman `7fb9da2` = SON COMMIT):** clustering `scipy.sparse.csgraph.connected_components` + vektörize gruplama + yeni `voxel_downsample` (config `voxel_size`: çekirdek 0=kapalı, üretim 0.1 — node/params/hardware.yaml/launch 4 yerde kablolu). **Benchmark x86: 20k nokta 524.5→53.6 ms (~10×), voxel'li tam detect 49.5 ms.** TDD: O(n²) referans-eşdeğerlik teli + 3 voxel testi + üretim-config sahne testi. **SUITE: 216 passed / 0 skip.**

**SIRADAKİ — Faz 18-T1 kalanlar (öncelik sırası):**
1. ~~F5.4 (böl-atma)~~ ✅ KAPANDI (`798ff4d`, LiDAR uç sahneleriyle birlikte — yukarıda).
2. ~~F6.5 kamera çeldirici sahneleri~~ ✅ KAPANDI (`b7cee13`): 4 sahne+4 test — beyaz sosis ateşlemiyor, F5.6 skor tersliği testte görünür (şerit 0.961 > duba 0.757), FOV kenarı kör noktası, F5.5 ≈15 m menzil kanıtı. F5.5 (sözleşmeye yazım) + F5.6 (skor semantiği, iki-repo kararı) bulgu olarak hâlâ açık. **SUITE: 234 passed / 0 skip, son commit `b7cee13`.**
3. F5.1 lidar_height_m — mekanik `h` BEKLİYOR (gelmeden Parkur-2 sahaya çıkmaz; üreteç+testlerle AYNI commit, F6.2 reçetesi). ⚠ F5.4 kapanışının notu: `h` gelince **min_range filtresi de birlikte değerlendirilecek** (gövde görünür olursa).
4. F8.3 iSAM2 graf ölçümü + D3 gerçek ms (Jetson) · F16.4 CI kurulumu (ekip kararı).
5. Sonra: PR upstream'e (arkadaş dönünce), Eyüp'ün donanım işleri ([[bekleyen-girdiler-isaret]]).

**Bu oturumda yapılanlar (7 commit):** T0-g grafik telemetri (`08f3fb5`) · F14.3 auto_guided görev-aktife bağlandı (`88a06c8`) · Faz 6 sentetik maskeleme avı (`64c51aa`) · Faz 7 algı node'ları (`aa10d9b`) · Faz 8 füzyon denetimi (`5296a1c`) + **F8.1 düzeltmesi: odom.twist dolduruldu — MPPI artık gerçek hızla başlıyor, istemsiz hareketin 4. kökü kapandı** (`dd135b1`) · Faz 10 RRT* (`18171f4`).

**SIRADAKİ (öncelik):**
1. **Faz 18-T1 düzeltmeleri:** ✅ F10.1+F10.2 (`5ee87b8`: replan try/except + bounds∪start/goal±30m zarfı, `test_planning_replan.py`) · ✅ F7.2+F7.3 (`9b9e23d`: class_id try/except + lidar depth=1, py_compile — vision_msgs makinede yok) · ✅ F8.2 (`c64d94e`: `pose_timeout_s`=1.0 bayatlık bekçisi, testli) · ✅ F7.1 bekçi kısmı (`5b37dd6`: `sync_watchdog_s`=10s — iki girdi akarken eşleşme sıfırsa WARN + STAMP SÖZLEŞMESİ docstring'e; kalıcı stamp hizası = SAHA işi: Livox saat kaynağı denetimi / slop ölçümü). **Suite 129, son commit `5b37dd6` push'lu.**
   **2. tur (2026-07-10 gece, +2 commit):**
   - ✅ **BEARING İŞARETİ (F6.1/F5.9/bulgular#3) KÖKten DÜZELTİLDİ (`e66cb40`):** `bearing_from_camera = (0.5−cx)·hfov` (sol pozitif = atan2 ile AYNI) + `_cx_for_bearing` tersi + `viz/scenario.py:145` (3. kopya oradaydı!) + kenar testi — reçete gereği TEK commit. 2 yeni üreteçten-bağımsız test (fiziksel işaret tutarlılığı + ham geometriyle soldaki duba eşleşmesi). Kamera ters monte çıkarsa tek değişim noktası yine `bearing_from_camera`. **Arkadaşa mesajdaki "bearing işaret hatası" maddesi artık "forkta düzelttik + testledik" olarak verilebilir.**
   - ✅ F3.1+F3.2 launch (`eb9ff58`): `perception_camera_node` artık `use_onboard_camera` koşullu (VARSAYILAN false) — `/perception/buoys`'un TEK üreticisi bizim OAK node'umuz, HSV yalnız bilinçli yedek (`use_onboard_camera:=true`); depthai_ros talimatı "EKLEME!" uyarısına çevrildi; `use_mppi` LogInfo'dan çıkarıldı (REZERVE işaretli, davranış değiştirmiyor).
   **Suite 131, son commit `eb9ff58` push'lu, working tree temiz.**
   Kalan düzeltmeler: F5.1 (lidar_height_m — mekanik `h` bekliyor; üreteç+testle AYNI commit, F6.2; lidar testleri scipy×numpy2 ABI yüzünden ancak Jetson'da koşar) · F5.3 (scipy clustering+voxel — aynı ABI engeli) · F6.3 sahneleri · F8.3 iSAM2 graf ölçümü.

**🔒 REPO GÜVENLİĞİ (2026-07-10, Eyüp sordu "arkadaşım silemiyor değil mi"):**
- Fork'ta ORTAKLAR VAR: **vistastris ve yahyaseha PUSH (yazma) erişimli**, admin DEĞİL. Repoyu SİLEMEZLER (silme yalnız admin=EyupEker1) ama yanlışlıkla push/force-push yapabilirler; branch protection private repoda ücretsiz planla yok.
- ⚠️ **GERÇEK RİSK:** GitHub'da private upstream (`vistastris/girdap-decision`) silinirse private FORK'LAR DA SİLİNİR. Arkadaş kendi reposunu silerse fork gider. Lokal klon (`/home/eyup/girdap-decision`) tam geçmişle yaşar.
- ✅ **YEDEK KURULDU (2026-07-10, Eyüp `~/yedek_kur.sh` ile kendi eliyle çalıştırdı; script duruyor):** `github.com/EyupEker1/girdap-decision-yedek` — **private ✓, fork DEĞİL ✓** (upstream silinse de yaşar), main=`eb9ff58` (lokal ile birebir, doğrulandı). **origin'in push URL'i ÇİFT:** her `git push` fork + yedek'e birden gider, yedek bayatlamaz. (Not: benim yedek push denemem auto-classifier'ca 2 kez engellendi — bu tür toplu-yeni-remote push'ları Eyüp'ün elinden yapılmalı.) Memory dosyaları lokal, arkadaş erişemez.

**🤝 KAMERA REPO UYUMU (Eyüp sordu, cevap: EVET):** `eb9ff58` ile iki repo tak-çalıştır kıvamında — bizim OAK node'u `/perception/buoys` (640×480 bbox, class_id "0"/"1", F4.6 iki yönde teyitli) + `/perception/gate_passed` + `buoys_3d` üretir; onun launch'ı artık HSV node'unu açmaz (çakışma yok); poz kaynağı `/girdap/fusion/odom` artık twist'li (F8.1). Kalan entegrasyon işleri SAHA/masa: letterbox `_LB_PAY=0.125` doğrulaması, Jetson'a vision-msgs kurulumu, füzyon stamp hizası (F7.1 bekçisi yakalar), Dosya-1 kaydedici FPS ölçümü.
2. Faz 16 (test paketi denetimi) + Faz 17 (doküman: CLAUDE.md RAL/cpp düzeltmeleri).
3. Eyüp'ün donanım işleri değişmedi: Livox `h`, NN Archive sınıf sırası, base_link, model yedeği, suda prova ([[bekleyen-girdiler-isaret]]).

**Adımlar:**
1. `cd /home/eyup/girdap-decision` (fork, `origin`=EyupEker1, `upstream`=vistastris). Git kimliği lokal kurulu.
2. `docs/kod_denetimi.md`'ye aynı formatta yaz (🔴🟠🟡⚪ + `dosya:satır`).
3. Her faz: doküman → commit → `git push origin main` → bu bölümü güncelle.
4. **Kural:** bitmiş+test edilmiş kod yeniden yazılmaz; yalnız kanıtlı hata. Erken teşhisi kanıt çürütürse açıkça geri al (F3.1, F5.8, F13.3'te 3 kez yapıldı — bu titizlik korunmalı).
5. Düzeltmeler Faz 18'de topluca; T0 (video) düzeltmeleri önce. Test koşturmak Eyüp'ün açık izni gerektirir (dış repo). · Faz 6 Sentetik üreteçler (maskeleme avı) · Faz 7 Algı ROS node'ları · Faz 8 Füzyon/iSAM2 · Faz 9 MPPI(569) · Faz 10 RRT*(477, hiç okunmadı) · Faz 11 Planlama+node · Faz 12 FSM · Faz 13 Görev yönetimi · Faz 14 MAVROS/kontrol · Faz 15 Harita+telemetri(Dosya-2/3) · Faz 16 Testler(161 fn) · Faz 17 Doküman · Faz 18 Düzelt+test+push.

**HER FAZ BİTİNCE:** `docs/kod_denetimi.md`'ye yaz + commit + push (forka) + bu bölümü güncelle.

## Bizim reponun sağlık kontrolü (2026-07-10) ✅

`git ls-files` = 17 dosya, `build/`+`__pycache__`+`egg-info` sızmamış (gitignore çalışıyor), tüm `.py` `py_compile` geçiyor, working tree temiz, her şey push'lu (`63c16b6`). Repo doğru.

## Kalan açık işler (kod değil)

- Arkadaşın cevaplaması gerekenler: nottaki §6 (odom frekans/çerçeve, hfov, buoys_3d, mock kamera kapatma, numpy pini).
- Masa testi: letterbox `_LB_PAY=0.125` doğrulaması (`duba_kamera_test.py`) — ters çıkarsa kodda `_LB_PAY=0.0`.
- Jetson ortam kontrolü: Humble + numpy<2 + depthai>=3.6 + vision-msgs (README'de tablo ve komutlar var).

Gereksinim notu: `algi_yayin` için `sudo apt install ros-humble-vision-msgs`.

## Ek doğrulamalar (2026-07-10, "bakmadığın yer var mı" turu)

- **Arkadaşın `docs/jetson_deployment.md`'si var ve iyi:** kendi stack'inin deploy'unu kapsıyor (venv=test / sistem python=ROS runtime ayrımı, gtsam sisteme, vision-msgs, dialout, GeographicLib). **numpy 2.x ABI sorununu dokümanda kendisi de yazmış** (ama requirements.txt'i pinlememiş — mesajdaki madde hâlâ geçerli, sadece "bilmiyor" değil "pinlememiş"). Operasyon servisleri: `/girdap/bridge/arm`, `/girdap/mission/start`, `/girdap/mission/kill` (Trigger).
- **`hardware.yaml` + `params.yaml` teyit:** füzyon `camera_image_width_px=640`, `height=480`, `hfov=1.2` — bilerek OAK-D Lite'a göre yazılmış; bizim buoys yayını birebir uyumlu.
- **`duba_kamera_test.py` letterbox doğrulaması yapabilir:** bbox'ları `sdn.passthrough` karesine ham normalize koordinatla çizer → ekranda üst/alt siyah şerit + kutular tam oturuyorsa koordinatlar NN çerçevesinde = `_LB_PAY=0.125` doğru; kutular dikey kaymışsa `_LB_PAY=0.0`.
- **Hâlâ okunmayanlar (sözleşmeyi etkilemez):** rrt_star.py, isam2_smoother.py, fusion pipeline içi, KTR dokümanları, bench_mavlink_runbook, ardurover parm, mission yaml'ları, viz, testlerin çoğu, 4 remote feature branch.
