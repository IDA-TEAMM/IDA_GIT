# HATA DEFTERİ — canlı bug kaydı (tek dosya)

> **KURAL:** Yeni bir hata/bug bulunduğunda debug verisiyle birlikte DOĞRUDAN buraya
> yazılır (Claude oturumları dahil). Hata kapanınca satır silinmez, "KAPANANLAR"
> bölümüne taşınır. Derin denetim arşivi: `docs/kod_denetimi.md` (dokunma, arşiv).
> Ham loglar `~/girdap_logs/` altında kalır; buradan yalnız yol verilir.

## Kayıt şablonu (yeni hata gelince kopyala)

```
### [TARİH] KOD — kısa başlık (🔴 kritik / 🟠 önemli / 🟡 düşük)
- **Belirti:** ne görüldü (log satırı / davranış)
- **Debug verisi:** ham log yolu, komut çıktısı, ölçüm (ne bulunduysa buraya)
- **Kök neden:** biliniyorsa; bilinmiyorsa "araştırılıyor"
- **Etki:** hangi şartname maddesi / parkur / test bloke oluyor
- **Durum:** AÇIK / bloke (neyi bekliyor) / düzeltme commit'i
```

---

## 🔴 AÇIK HATALAR

### [2026-07-14] F-M.9 — USB düşünce mavros ÖLÜYOR + geri gelmiyor (respawn yok) → yığın kalıcı KILL (🔴 video-günü süreklilik riski)
- **Belirti (boot provası SONRASI, 18:55):** Pixhawk USB'si düştü → mavros_node
  `mavconn serial1: receive: End of file` + std::system_error → çöktü (exit -6) → GERİ
  GELMEDİ → köprü 5.5 sn'de doğru KILL bastı → yığın kalıcı KILL'de takıldı (F14.4 latch).
  Kurtarma yalnız `sudo systemctl restart girdap-karar`.
- **Debug verisi:** `apm.launch:11` `respawn_mavros default=false` + hardware.launch include'u
  bu argı SET ETMİYOR → mavros respawn'suz. Kernel: `18:55:01 usb 1-2.1 USB disconnect
  device 4` → 4 dk boşluk → `18:59:06 device 9` (yalnız ttyACM0!) → 5 sn sonra `18:59:11
  device 9 disconnect → device 10` (bu kez ttyACM0+ttyACM1 ikisi). **Pixhawk bir USB
  HUB'ında** (1-2:1.0, 4 port; kardeş cihazlar: Fantech fare 1-2.4, +1 cihaz 1-2.2).
- **Kök neden (2 katman):** (1) mavros respawn kapalı → USB hıçkırığında süreç ölür kalır;
  (2) F14.4 — heartbeat KILL kalıcı latch, histerezis yok → mavros geri gelse bile KILL
  temizlenmez. İkisi birleşince: kısa USB blibi = ölü yığın = elle restart.
- **Etki:** md 3.3.1/6 "tek kesintisiz çekim" — çekim ortasında 2 sn USB hıçkırığı bile
  video'yu yakar (operatör sudo restart atmak zorunda). KILL'in KENDİSİ doğru davranış
  (gerçek hat kaybında güvenlik); sıkıntı KURTARILAMAMASI.
- **✅ DONANIM ONAYLANDI (Eyüp 2026-07-14): "ben çekmedim, Type-C portunda bağlıydı."**
  Kanıt zinciri: (a) kopma anında `tegra-xusb WARN Event TRB slot 4 ep 4 with no TDs queued`
  = veri akarken ANİ fiziksel kesinti (yazılım reset/FC reboot bu uyarıyı vermez); (b) ~4 dk
  açık kaldı, kimse dokunmadan geri geldi; (c) replug'da yarım→tam çift-enumerasyon = kötü
  kontak/soğuk lehim imzası; (d) Jetson'da undervoltage/güç uyarısı YOK → sebep Jetson değil,
  FC ucundaki **USB-C soketi/kablosu.** Kardeş cihaz (fare 1-2.4) düşmedi → hub değil, cihaz ucu.
  → **"USB-C tamirli+çalışıyor" (FC OLAY kapanışı) BAYAT; tamir TUTMADI, aralıklı açılıyor.**
- **Etki (güncel):** Jetson↔Pixhawk MAVLink'in TAMAMI bu soketten. Suda/çekimde aralıklı
  kopma = mavros ölür (F-M.9) = KILL latch = ölü yığın = video yanar; yarışmada görev kaybı.
- **Durum:** 🔴 AÇIK — **video ÖNCESİ DONANIM BLOKERİ.** Aksiyonlar: (1) FC/mekanik ekip
  soketi KESİN çöz / DEĞİŞTİR + gerginlik boşaltma (strain relief); (2) yedek hat hazırla+test:
  TELEM2 FTDI serial `fcu_url=serial:///dev/ttyUSB0:57600` (57600'de IMU ~10 Hz = F-M.6 için
  bize yeten hız; FTDI kablo mevcut mu + port çakışması QGC MicoAir ile SORULACAK). Yazılım
  respawn/histerezis (F14.4) bu arızayı ÖRTME aracı DEĞİL — kırılgan güvenlik hattını yazılımla
  otomatik kurtarmak suda tanımsız duruma dönebilir; kök çözüm donanım.

### [2026-07-14] F-M.8 — taze boot'ta bile ttyACM0 "busy" → mavros ~30 sn geç bağlanıyor (🟡 T1, kozmetik gecikme)
- **Belirti (BOOT PROVASI, 2026-07-14 18:42 reboot):** eski-instance teorisi tek başına
  yetmiyor — TAZE boot'ta da (tutucu eski süreç YOK) aynı desen: `18:42:53` mavros
  `link[1000] open failed: Device or resource busy` → mavros_router hardcoded 30 sn
  retry → `18:43:23` reconnected + `Got HEARTBEAT`. Boot→FC bağlantısı toplam ~44 sn.
- **Debug verisi (journalctl, 2026-07-14 boot):** `uptime -s = 18:42:39` · servis Started
  `18:42:51` · busy `18:42:53` · **ModemManager active+enabled ve ACM portlarını yokluyor:**
  `18:43:02 ModemManager: could not grab port ttyACM0/ttyACM1 (unhandled port type)` —
  MM probe'u portu kısa süre açık tutar, busy'nin baş şüphelisi. Ayrıca `18:44:43`
  `TM: Time jump detected` (boot sonrası NTP saat sıçraması, zararsız).
- **Kök neden:** araştırılıyor — baş şüpheli ModemManager'ın seri-port probe'u;
  mavros_router'ın 30 sn sabit retry'ı (parametresiz, mavros_router.hpp:167) gecikmeyi büyütüyor.
- **Etki:** F-M.7 fix'i sayesinde ZARARSIZ (KILL yok, yığın sadece ~45 sn geç hazır olur).
  Video/yarışma sabahı operatör boot sonrası ~1 dk beklemeli (runbook notu).
- **Durum:** AÇIK, T1 iyileştirme (sudo, karar Eyüp'te). Seçenekler: (a) udev kuralı
  `ID_MM_DEVICE_IGNORE=1` (Pixhawk idVendor 3162) — en temiz; (b) `systemctl disable
  ModemManager` (teknede modem yok); (c) servise `ExecStartPre` port-boş-bekleme.

### [2026-07-14] F-V.6 — "önce AUTO, sonra ARM" akışında görev FSM'de HİÇ başlamıyor (🔴 VİDEO-KATİL)
- **Belirti (kod denetimi, AUTO dönüşü sonrası):** `fsm_node._on_mav_state` başlatma tetiği
  KENAR şartlı (`_last_mode != start_mode` → `== start_mode`) VE FSM `BEKLEMEDE`'de olmalı;
  BEKLEMEDE'ye de ancak ARM olunca giriliyor. Operatör QGC'de **önce modu AUTO yapıp
  SONRA arm ederse** (QGC "Start Mission" akışı; ArduRover AUTO'da arm olunca görevi
  başlatır) kenar hiç oluşmaz → FSM BEKLEMEDE'de KALIR.
- **Etki:** FC görevi koşar, tekne 4 noktayı gezer — ama `mission_state="BEKLEMEDE"` →
  telemetry F-V.2 gereği `hiz_setpoint`/`yon_setpoint` sütunlarını BOŞ bırakır →
  **Ekran-2'nin ZORUNLU setpoint eğrileri boş çıkar (md 3.3.1.1)**. Video TEK ÇEKİM +
  kesintisiz (md 3.3.1/6) → çekim sırasında fark edilmez, montajda anlaşılır = video yanar.
- **Kök neden:** kenar şartı GUIDED+MPPI için tasarlanmıştı (boot'ta zaten GUIDED ise
  MPPI kendiliğinden motor sürmesin). AUTO'da FSM motor sürmüyor (planning geçidi GUIDED
  bekler) → orada kenar şartı korumuyor, sadece engelliyor.
- **Durum:** ✅ TDD DÜZELTİLDİ — `fsm_node.start_on_arm_in_mode` (YARIŞMA varsayılanı
  **false**, güvenlik korunur; `hardware.yaml` VİDEO için **true**): BEKLEMEDE'ye armed
  girildiğinde mod zaten `start_on_mode` ise görev başlar (FC zaten koşuyor, FSM İZLİYOR).
  3 test (kenarsız başlatma + yarışma güvenliği + kenar regresyonu).

### [2026-07-14] F-P.1 — planning bayat pozla MPPI koşmaya devam ediyordu (🟠 yarışma; videoda etkisiz)
- **Belirti (kod denetimi):** `fusion_node` F8.2 bekçisi poz kaynağı susunca odom yayınını
  KESİYOR ("bayat pozla plan yapılmasın") — ama `planning_node._on_odom` son durumu saklıyor,
  `_on_control_step` odom'un YAŞINA BAKMADAN 10 Hz MPPI koşmaya devam ediyordu.
  Yani F8.2'nin niyeti planning tarafında KARŞILIKSIZDI.
- **Etki:** GPS/EKF kesilirse (fix kaybı, mavros kopması) araç son bilinen pozla KÖR sürer →
  yarışmada dubaya/parkur dışına gider; md 3.3.1.1 anlamında istemsiz hareket.
  **AUTO videosunda ETKİSİZ** (planning cmd_vel basmıyor, mod geçidi GUIDED bekliyor).
- **Durum:** ✅ TDD DÜZELTİLDİ — `planning_node.odom_timeout_s` (varsayılan 1.0 s, fusion'ın
  `pose_timeout_s`'iyle aynı mantık; 0 → kapalı): bayatsa thrust SIFIR + saniyede bir ERROR.
  odom hiç gelmediyse yanlış alarm basmaz (boot). 3 test.

### [2026-07-14] F-V.7 — AUTO'da yon_setpoint waypoint üstünde savruluyor + sahte bekleme (🟠)
- **Belirti (kod denetimi):** iki ayrı kusur, ikisi de Ekran-2b'nin ZORUNLU eğrisini bozar:
  1. `mission_manager` DWELL evresinde hedef index'ini 2 sn ilerletmiyor (`dwell_time_s: 2.0`).
     GUIDED'da tekneyi BİZ sürdüğümüz için gerçek bekleme; **AUTO'da FC durmaz** → tekne
     waypoint'i geçip giderken bizim hedefimiz 2 sn boyunca ARKADA kalan noktayı gösterir.
  2. `telemetry._on_target` = `atan2(y, x)`; waypoint'in üstünden geçerken ofset ~0 →
     açı savrulur, sonra ~180° döner. Her waypoint'te çöp sıçrama.
- **Etki:** md 3.3.1.1 "görüntü/hareket net değilse BAŞARISIZ" + yön setpoint eğrisi yalan söyler.
- **Durum:** ✅ DÜZELTİLDİ — (1) `hardware.yaml mission.dwell_time_s: 0.0` (yarışmada 2.0
  kalır, launch'tan geçirilir); (2) `telemetry.yaw_setpoint_min_dist_m: 0.5` — bu mesafeden
  yakında açı GÜNCELLENMEZ, son geçerli istek korunur (TDD).
- **⚠ AÇIK KALAN (hafifledi, bkz. F-V.8):** `arrival_radius_m` ↔ FC `WP_RADIUS` teyidi hâlâ
  iyi pratik ama artık KRİTİK değil — F-V.8 FC'nin kendi varış sinyalini dinliyor. Ayrıca
  `/mavros/nav_controller_output/output` (FC'nin nav_bearing'i) T1 fikri olarak duruyor —
  konvansiyon dönüşümü (pusula→ENU) suda doğrulanmadan KONMAZ.

### [2026-07-14] F-V.8 — AUTO'da görev ilerlemesi FC'den senkronlanmıyordu (🔴 → düzeltildi)
- **Belirti (2. denetim turu):** görev BİTİŞİ ve waypoint ilerlemesi YALNIZ bizim
  `arrival_radius_m` tespitimize bağlıydı; FC'nin `MISSION_ITEM_REACHED`'i hiç dinlenmiyordu.
  AUTO'da rover köşeyi bizim 2 m yarıçapımıza GİRMEDEN dönerse (WP_RADIUS farkı + dönüş
  yarıçapı) index TAKILIR: yon_setpoint görev sonuna kadar GERİYİ gösterir, COMPLETE hiç
  gelmez, FSM PARKUR1'de kalır → manuel dönüşte setpoint yazılmaya devam eder (Ekran-2 yalan).
- **Düzeltme (TDD):** `MissionManager.notify_external_reached(idx)` (yalnız İLERİ senkron;
  IDLE/geride/aralık-dışı yok sayılır) + `fc_items_to_waypoints_with_seqs` (wp_seq→index
  eşlemesi, filtre tek yerde) + node `/mavros/mission/reached` aboneliği (yalnız fc modu) +
  `_publish_reached_once` (kendi varışımızla FC sinyali aynı index'i iki kez basmasın —
  parkur katmanı Int32 tüketiyor). 7 test (5 çekirdek + 2 node).
- **⚠ QoS tuzağı (canlıdan ölçüldü):** `/mavros/mission/reached` yayıncısı TRANSIENT_LOCAL —
  latched abone olsaydık BİR ÖNCEKİ koşunun son mesajı boot'ta tekrar gelirdi. BİLEREK
  volatile abonelik + IDLE koruması (çift emniyet).
- **Yarışma etkisi YOK:** GUIDED'da FC görev koşmaz → topic hiç akmaz → davranış değişmez.

### [2026-07-14] F-M.6 — FC taze bağlantıda ~1 Hz yayınlıyor; yazılım hiç akış hızı istemiyor (🔴)
- **Belirti:** Oturum 2 masa testi: USB bağlantısı kurulduğunda `/mavros/imu/data` ~1 Hz.
  Elle `ros2 service call /mavros/set_stream_rate ... {stream_id: 0, message_rate: 50,
  on_off: true}` → ~39 Hz. İstek OTURUMLUK: servis/boot yolunda kimse istemiyordu, yani
  her taze bağlantıda (boot, USB tak-çıkar, mavros restart) FC yine 1 Hz'e düşüyor.
- **Debug verisi:** `~/girdap_logs/masa_testi/` Oturum 2 notları; 2026-07-14 ölçümü:
  elle istek sonrası `/mavros/rc/out` 35 Hz (bağlantı ayakta kaldığı sürece korunuyor).
  ArduPilot tarafı: SR0_* parametreleri (USB=SERIAL0, bant kısıtı YOK).
- **Kök neden:** ArduPilot taze bağlantıda SR0_* parametrelerine göre yayınlar; bizim
  yığın REQUEST_DATA_STREAM göndermiyordu. (FC EEPROM'undaki SR0_* değerleri düşük.)
- **Etki — üç katman, hepsi VİDEO-kritik:**
  1. **Ekran-2 (md 3.3.1.1):** hız/heading/thrust eğrileri 1 Hz basamaklı → "görüntü
     net değilse BAŞARISIZ". AUTO+FC videosunda üç eğri de FC akışından geldiği için
     (ATTITUDE/VFR_HUD/SERVO_OUTPUT_RAW) etki DAHA büyük.
  2. **fusion_node F8.2 bekçisi:** `pose_timeout_s=1.0` — 1 Hz'lik `local_position/pose`
     akışında yaş eşiği jitter'la sürekli aşılır → `/girdap/fusion/odom` yayını KESİLİR
     ("poz kaynağı bayat" WARN spam'i). Bekçi 1 Hz akışı arıza sanıyor.
  3. **planning_node:** `_on_odom` son pozu saklar, `_on_control_step` 10 Hz'te pozun
     YAŞINA BAKMADAN MPPI koşar → 1 m/s seyirde 1 metreye kadar bayat pozla düzeltme
     → salınım/zikzak = md 3.3.1.1 "istemsiz hareket → BAŞARISIZ".
- **Durum:** TDD DÜZELTİLDİ (bu oturum, F-M.6): `MavrosBridge.should_request_stream_rate()`
  (bağlantı yükselen kenarı, çekirdek) + `mavros_bridge_node._maybe_request_stream_rate()`
  → `/mavros/set_stream_rate` (STREAM_ALL, `stream_rate_hz`=10, oturumluk — FC EEPROM'una
  YAZMAZ, Eyüp'ün "FC paramlarına dokunmayın" kuralı korunur). Yeniden bağlanışta tekrar
  istenir; `stream_rate_hz: 0` → devre dışı. 8 test (5 çekirdek + 3 node).
  ✅ **CANLI DOĞRULANDI (2026-07-14 akşam, servise DOKUNMADAN):** Pixhawk 6C ikinci USB
  kanalı `/dev/ttyACM1` boştaydı → izole `ROS_DOMAIN_ID=77`'de taze mavros oturumu açıldı
  (servisin ACM0 kanalı ve node'ları hiç etkilenmedi).
  - **Fix'siz taze bağlantı (= boot davranışı):** `/mavros/imu/data` **1.000 Hz**,
    `/mavros/rc/out` **1.000 Hz**, `/mavros/global_position/global` **0.999 Hz**.
    → "boot'ta 1 Hz" iddiası KANITLANDI (varsayım değil, ölçüm).
  - **Yeni köprü çalışınca:** log `FC akış hızı isteniyor: 10 Hz (STREAM_ALL)` →
    aynı üç akış **9.99 Hz**. İlk state'te servis hazır değildi ("istek ertelendi"),
    ~1 s sonraki state'te gitti — tasarlanan retry yolu gerçek FCU'da çalıştı.
  - **Yük:** mavros süreci ~%50 (tek çekirdek, 6 çekirdekli Orin'de ~%8 toplam),
    CPU 50°C / tj 51°C, loadavg 3.4. 10 Hz'te yük/termal sorunu YOK
    (arkadaşın "35 Hz'de yük biner" itirazının ölçülmüş cevabı: 10 Hz ≠ 35 Hz).
  - `/mavros/local_position/pose` kapalı mekânda hiç yayınlanmıyor (GPS fix yok →
    EKF pozu yok) — BEKLENEN; F-M.6'dan bağımsız, açık alanda ölçülecek.
  - ⏳ **KALAN (ops):** servis hâlâ ESKİ kodu bellekte çalıştırıyor →
    `sudo systemctl restart girdap-karar` ile deploy edilmeli (sudo = Eyüp).
  Kalıcı çözümü FC ekibi SR0_* yazarak seçerse bu istek zararsız kalır (aynı hızı ister).

### [2026-07-14] F-S.1 — RC donanım kill-switch girdap_decision'da hiç yoktu (🟠, düzeltildi)
- **Belirti (kod karşılaştırması, ida_topics vs girdap_decision):** `mavros_bridge_node`
  yalnız yazılım/servis KILL yollarını biliyordu (heartbeat kaybı, beklenmedik disarm,
  fsm_node servisi) — `ida_topics/control_node.py`'deki RC kanal 8 donanım kill-switch'inin
  (companion computer'dan bağımsız, tek RC alıcısı üzerinden) karşılığı yoktu.
- **Debug verisi:** `grep -rn "rc/in\|RCIn\|RC_KILL"` girdap_decision ağacında sıfır sonuç
  verdi (2026-07-14, sude_memory tarafında dış karşılaştırma — bkz IDA_GIT/son_kodv2).
- **Etki:** companion computer donarsa/mavros_bridge çökerse yazılım KILL yollarının
  hiçbiri tetiklenemez; tek koruma fiziksel güç-kesme anahtarıydı (md 3.3.1/4) — RC
  üzerinden ikinci, bağımsız bir kill yolu yoktu.
- **Düzeltme (TDD):** `MavrosBridge.is_rc_kill_active()` (çekirdek, 6 test) +
  `mavros_bridge_node._on_rc_in()` (`/mavros/rc/in`, `sensor_data_qos()` BEST_EFFORT,
  4 test) — `rc_kill_channel`/`rc_kill_threshold_pwm` parametreleri ida_topics ile aynı
  varsayılanlar (kanal 8, PWM 1500). Tek KILL otoritesi (`_trigger_kill`, latch) korunuyor,
  bu yalnız bir tetikleyici daha.
- **Durum:** KAPANDI (bu oturumda TDD ile eklendi ve test edildi).
- **⚠️ DÜZELTME (2026-07-14, ekip geri bildirimi):** ArduPilot'ın kendi
  `RCx_OPTION=31` ("Motor Emergency Stop") firmware özelliği muhtemelen zaten
  yapılandırılmış — bu, MAVLink/companion computer'a HİÇ gitmeden, gerçek
  zamanlı ve companion computer'dan GERÇEKTEN bağımsız çalışır. Bu koddaki
  F-S.1 (mavros_bridge'in `/mavros/rc/in`'i dinlemesi) companion computer'ın
  İÇİNDE çalışıyor — yani companion computer/mavros donarsa BU kod da aynı
  anda çalışamaz hale gelir; iddia edilen "bağımsızlık" gerçekte sağlanmıyor.
  **Aksiyon:** FC ekibine `RCx_OPTION` parametrelerini (31 hangi kanalda)
  sor. Yapılandırılmışsa F-S.1 kritik değil, en fazla ikincil/loglama katmanı
  — "kritik güvenlik fix'i" etiketi kaldırılmalı. Kod GERİ ALINMADI (zararsız,
  FC'nin kendi E-Stop'una ek bir katman) ama önceliği düşürüldü.

### [2026-07-14] F-S.9 — turuncu/sarı tespiti için YOLO-lokalizasyon + HSV hibrit (algı entegrasyonu)
- **Bağlam:** `girdap_decision`'ın turuncu/sarı (class 0/1) tespiti tamamen HSV
  segmentasyonu — YOLO yalnız class 2 "hedef" için ayrılmış (Parkur-3). ida_topics'in
  `best.pt` modeli ise GENEL duba lokalizasyonu için eğitilmiş (renk sınıfı ayrıca
  HSV ile belirleniyor) — girdap'ın "hedef" kavramıyla AYNI ŞEY DEĞİL, doğrudan o
  yuvaya takmak yanlış olurdu.
- **Çözüm:** `camera_buoys.py`'ye YENİ bir alternatif yol eklendi — `BuoyLocalizer`
  (eğitilmiş genel duba modeli, SINIF ÜRETMEZ) + `classify_roi_color()` (girdap'ın
  AYARLANMIŞ HSV eşikleriyle kutunun rengini/sınıfını belirler). `use_yolo_localizer`
  (varsayılan false → mevcut saf-HSV davranışı DEĞİŞMEZ) ile açılır.
  `perception_camera_node.py`'ye `yolo_localizer_model_path` parametresi eklendi.
- **Doğrulama:** 10 yeni test (sentetik sahne üzerinde saf-HSV yoluyla eşdeğer
  sonuç, hedef-dışı kutu atlanır, localizer=None güvenli fallback, ultralytics
  yalnız gerçek modelde import edilir).
- **Durum:** KAPANDI. Gerçek `best.pt` dosyası henüz container'a kopyalanmadı —
  `yolo_localizer_model_path` parametresi verilene kadar mock modda kalır.

### [2026-07-14] F-S.10 — Karar katmanı sentezi: MPPI'ye TDD PID alternatifi (mimari birleşim)
- **Bağlam:** ida_topics/decision_node.py (basit cascade PID, donanımda kanıtlı) ile
  girdap_decision'ın RRT*+MPPI'ı (daha yetenekli, ama RRT* girdisi bu oturumda ilk
  kez bağlandı — F-S.6 — hiç donanımda koşmadı) arasında hangisinin esas alınacağı
  açık soruydu. Karar: ikisinin GÜÇLÜ yanlarını birleştiren yeni kod.
- **Çözüm:** `prototype/planning/pid_controller.py` — ida_topics'in cascade heading
  PID'i (dış döngü heading→yaw_rate, iç döngü yaw_rate→açısal düzeltme) + girdap'ın
  LiDAR `CircleObstacle` verisiyle potansiyel-alan kaçınması. `MPPIController.step()`
  ile BİREBİR AYNI arayüz — `PlanningPipelineConfig.control_mode` ("mppi" varsayılan
  | "pid") ile hiçbir çağıran kod değişmeden seçilir. FSM/mission_manager/mavros_bridge
  güvenlik çatısı AYNEN korunuyor — yalnız yerel kontrolcü değişiyor.
- **🐛 Test sırasında bulunan gerçek hata:** ilk tasarımda engel TAM ÖNDEYKEN (radyal
  itme, hedef vektörünü doğrudan iptal eder) klasik potansiyel-alan "tekil nokta"
  sorunu kapalı-döngü fiziksel simülasyonda GERÇEK ÇARPIŞMAYA yol açtı
  (`test_pid_modu_kapali_dongu_engelden_kacar`, min_clearance -2.8 m). Düzeltme:
  radyal itmeye küçük sabit bir teğetsel (90°) bileşen eklendi (`obstacle_tangential_ratio`,
  vektörel toplam — açısal işaret/sign-flip yerine), + güvenlik payı fiziksel testle
  kalibre edildi (3m → 8m). Testler bu hatayı YAKALADI ve düzeltmeyi doğruladı.
- **Doğrulama:** 17 yeni test (11 birim + 6 PlanningPipeline entegrasyonu) — kapalı-döngü
  fiziksel simülasyonda hem goal'e ulaşma HEM gerçek engelden kaçınma kanıtlandı
  (yalnız birim test değil, `CatamaranDynamics.step_rk4` ile gerçek plant simülasyonu).
- **Durum:** KAPANDI — ama SAHA DOĞRULAMASI GEREKİR (MPPI gibi, PID modu da henüz
  gerçek donanımda hiç koşmadı; varsayılan hâlâ "mppi", "pid" saha testi bekliyor).

### [2026-07-14] F-S.2/F-S.3 — sensör sürücüleri (ida_topics) son_kodv2'ye entegre edildi
- **F-S.2:** `ros2_ws/src/ida_topics_yeni` (Livox UDP, OAK-D depthai, gps_imu MAVROS
  köprüsü, kamera_kayit) son_kodv2'ye kopyalandı + `hardware.launch.py`'a
  `with_drivers:=true` bayrağıyla wire edildi (son_kod'daki desenle aynı: remap
  `/lidar/points`→`/livox/lidar`, `/camera/image_raw`→`/oak/rgb/image_raw`).
- **F-S.3 (bilinen kozmetik kısıt, T1):** `kamera_kayit_node` ida_topics'in KENDİ
  perception'ının iki ayrı topic'ini (`/perception/orange_buoys`/`yellow_buoys`)
  varsayıyor; girdap_decision'ın `perception_camera_node`'u TEK topic'te
  (`/perception/buoys`) class_id (0/1/2) taşıyor. Remap yalnız `/perception/buoys`'u
  `orange_buoys`'a bağlıyor → Dosya-1 mp4 üretiliyor (≥1Hz, bbox overlay) ama
  sarı/hedef sınıflar da "TURUNCU DUBA" etiketiyle çiziliyor. Video için engelleyici
  DEĞİL (format şartı sağlanıyor), T1'de `kamera_kayit_node`'un class_id okur hale
  getirilmesi önerilir.
- **Durum:** F-S.2 KAPANDI (build+launch doğrulandı), F-S.3 AÇIK (T1, düşük öncelik).

### [2026-07-14] F-S.4 — RC manuel-override girdap_decision'da hiç yoktu (🟠, düzeltildi)
- **Belirti:** `ida_topics/control_node.py`'deki RC kanal 5 manuel-override mantığının
  (pilot RC'den MANUAL isterse yazılım GUIDED için kavga etmez) girdap_decision'da
  karşılığı yoktu — `mavros_bridge._maybe_auto_guided()` görev aktifken (PARKUR1/2/3)
  mod hedeften farklı olduğu sürece SÜREKLİ GUIDED isteği gönderiyordu, pilot RC'den
  manuel almaya çalışsa bile.
- **Etki:** operatör görev sırasında acil RC manuel müdahale etmek isterse yazılım
  mod isteğiyle "kavga edebilir" — md 3.3.1.1 istemsiz-hareket riskiyle aynı aile.
- **Düzeltme (TDD):** `MavrosBridge.is_rc_manual_active()` + `set_rc_manual_override()`
  (çekirdek, 4 test) — `needs_mode_change()` artık `rc_manual_override` aktifken
  False dönüyor. `mavros_bridge_node._on_rc_manual_check()` (`/mavros/rc/in`, aynı
  abonelik F-S.1 ile paylaşılıyor), `rc_manual_channel`/`rc_manual_threshold_pwm`
  parametreleri ida_topics ile aynı varsayılanlar (kanal 5, PWM 1700), 3 node testi.
- **Durum:** KAPANDI (bu oturumda TDD ile eklendi ve test edildi). Suite 310→317.

### [2026-07-14] F-S.5 — disk-dolu (OSError) korumasız yazma noktaları (🔴, düzeltildi)
- **Belirti (tam sistem taraması):** `TelemetryCsvLogger._sync`/`write_sample`,
  `LocalMapDumper.write_frame`, `ida_topics/telemetri_node.py`, `local_map_node.py`
  yazma noktalarının HİÇBİRİNDE try/except yoktu. Bu proje daha önce GERÇEKTEN
  disk-dolu krizi yaşamıştı (bkz proje notları) — tekrarlarsa timer callback'i
  içinde yakalanmayan `OSError` node'u tamamen öldürür, Dosya-2/Dosya-3 zorunlu
  teslimleri sonsuza dek durur (tek örnek kaybı değil).
- **Düzeltme (TDD):** `TelemetryCsvLogger.write_sample()` artık `bool` döner
  (True=başarılı, False=disk hatası — exception SIZMAZ), `girdap_decision/
  telemetry_node.py` dönüş değerini loglar. `LocalMapDumper.write_frame()`
  disk hatasında `None` döner + boyut uyuşmazlığında (bozuk OccupancyGrid) net
  bir `ValueError` yükseltir (numpy'nin belirsiz mesajı yerine), `local_map_node.py`
  ikisini de yakalayıp kareyi atlar. `ida_topics/telemetri_node.py` ve
  `local_map_node.py`'de aynı desen (`try/except OSError`) + `os.fsync` eklendi.
  4 yeni test (2 telemetry_logger + 2 local_map).
- **Durum:** KAPANDI.

### [2026-07-14] F-S.6 — RRT* girdisi (`/girdap/mission/waypoints`) HİÇ publish edilmiyordu (🔴, düzeltildi)
- **Belirti (tam sistem taraması):** `planning_node`, `/girdap/mission/waypoints`
  (`nav_msgs/Path`) dinliyordu ama repo genelinde bunu publish eden HİÇBİR node
  yoktu (`mission_manager_node` yalnızca tekil `current_target` yayınlıyordu).
  **Sonuç: `use_rrt=true` (gerçek yarışma modu) ile global plan hiç oluşmuyor,
  `compute_control()` hep `None` dönüyor, thrust sıfırda kalıyordu** — yalnızca
  video bypass modu (`use_rrt=false`) çalışıyordu, ki bu şimdiye kadar hiç
  fark edilmemişti çünkü video için zaten `use_rrt=false` kullanılıyor.
- **Düzeltme:** `mission_manager_node._publish_waypoints_path()` — tüm
  `MissionManager.waypoints`'i `current_target` ile AYNI referansta (güncel
  GPS pozisyonuna göre `latlon_to_enu`) `base_link`-göreli `Path` olarak 5 Hz
  yayınlar. `planning_node._on_waypoints()` artık bu ofseti kendi son bilinen
  odom xy'sine ekleyip mutlak "map" konumuna çevirir (`_on_target` ile birebir
  aynı dönüşüm deseni — DRY). 4 yeni test (3 planning_node + 1 mission_manager_node,
  gerçek `latlon_to_enu` değerleriyle karşılaştırmalı).
- **Durum:** KAPANDI — ama SAHA DOĞRULAMASI BEKLİYOR (T0 öncesi öncelik):
  RRT*+MPPI tam yarışma modu şimdiye kadar hiç uçtan-uca test edilmemiş
  olabilir (video modunda bu kod yolu hiç çalışmadığı için).

### [2026-07-14] F-S.7 — OAK-D okuma thread'i timeout'suz, USB donarsa sonsuza dek asılı kalır (🟡, düzeltildi)
- **Belirti:** `ida_topics/oakd_driver_node.py:_capture_loop` bloklayan
  `queue.get()` kullanıyordu — USB glitch'inde thread sessizce sonsuza kadar
  asılı kalır, hiçbir log/yeniden-deneme yok.
- **Düzeltme:** `tryGet()` (bloklamayan) + 5s'lik "kare gelmiyor" uyarısı.
- **Durum:** KAPANDI.

### [2026-07-14] F-S.8 — Parkur-3 çarpma tespiti: iki paralel sistemden yalnız biri bağlıydı (🟠, düzeltildi)
- **Belirti:** `fsm_node._on_imu` gerçek IMU darbesini üst-katman `MissionFSM`'e
  (`_obs.shock_detected_p3`) besliyordu — AMA waypoint-index parkur katmanı
  (`ParkurTransitionLogic.confirm_impact()`) yalnızca hiçbir node'un publish
  ETMEDİĞİ `/girdap/parkur/impact` (Sprint 5 placeholder) topic'ine bağlıydı.
  **Sonuç: gerçek yarışmada `/girdap/parkur/state` sonsuza dek PARKUR_3'te
  takılı kalırdı**, `MissionFSM` doğru TAMAMLANDI'ya geçse bile.
- **Düzeltme:** `_on_imu` artık darbe eşiği aşılınca HEM `_obs.shock_detected_p3`
  HEM `self._parkur.confirm_impact()` çağırıyor (idempotent, güvenli). 1 yeni
  test — placeholder topic'e hiç dokunmadan gerçek IMU yolunu doğrudan test eder.
- **Durum:** KAPANDI.

### [2026-07-14] F-M.3 — servis yoluyla KILL FCU'yu DISARM etmiyor (🟠, KAPANDI'ya bkz)
- Bu madde koda göre zaten düzeltilmişti (`_on_mission_state` KILL gözleyince
  `_trigger_kill()` çağırıyor, `test_fm3_*` testleri PASSED) — doc-senkron gecikmesiyle
  "AÇIK" bırakılmıştı. KAPANANLAR tablosuna taşındı.

### [2026-07-14] F-M.5 — seri hat kopunca mavros_node SIGABRT ile ölüyor, respawn yok (🟡 not)
- **Belirti:** M6d USB-çekme testinde `mavconn: serial0: receive: End of file` →
  `terminate called after throwing 'std::system_error'` → mavros_node exit -6.
- **Debug verisi:** `~/girdap_logs/masa_testi/masa_stack_2026-07-14_m6d.log`.
- **Kök neden:** mavros upstream davranışı (cihaz yok olunca abort); hardware.launch'ta
  hiçbir node'a respawn tanımlı değil.
- **Etki:** heartbeat-KILL latch'i zaten KALICI (F14.4) — görev bitti sayılır, bilinçli
  stack restart gerekir; video senaryosunu BLOKE ETMEZ. T1'de değerlendirilecek:
  mavros'a respawn:=true + KILL-latch etkileşimi (latch varken respawn işe yaramaz,
  bilinçli karar gerekir).
- **Durum:** AÇIK (bilgi notu, T1).

### [2026-07-14] F-M.4 — fix'siz PARKUR1'de bridge 10 Hz "GUIDED mod isteği" spam'i (🟡)
- **Belirti:** F-M.1 senaryosunda (görev FC'den alındı, fix yok, FSM PARKUR1 ama
  mission_manager başlatmıyor) bridge ~100 ms'de bir "GUIDED mod isteği gönderildi"
  bastı; ArduPilot GPS'siz GUIDED'ı reddettiğinden istek sonsuza dek yinelenir.
- **Debug verisi:** aynı log, 1784024398-399 aralığı (saniyede ~10 satır).
- **Kök neden:** F14.3 geçidi görev-aktifliği FSM state'inden okuyor; FSM fix beklerken
  de PARKUR1'de → `needs_mode_change` sürekli True, hız sınırı yok.
- **Etki:** sahada fix ARM'dan önce geleceği için (F8.4 kuralı) gerçek koşuda tek
  istekte biter — video blokeri DEĞİL. Masa/fix'siz senaryoda log gürültüsü + FCU'ya
  gereksiz istek yağmuru. T1'de değerlendirilecek (istek hız sınırı ya da fix'e kapıla).
- **Durum:** AÇIK (düşük öncelik, T1).

### [2026-07-12] FC-OLAY — FC hafızadaki sahte görevi RC/AUTO ile kendi koştu (🔴)
- **Belirti:** yığın kapalıyken motorlar tam güç döndü; güç kesilip verilince görev devam etti.
- **Debug verisi:** `~/girdap_logs/masa_testi/masa_stack_2026-07-12_aksam.log`; olay kaydı commit `eae9d9d`.
- **Kök neden:** M4 test görevi (40°K/29°D sahte) FC hafızasında kaldı + `BRD_SAFETY_DEFLT=0` çıkışları açık bıraktı; muhtemel tetik CH5 mod kanalı.
- **YENİ VERİ (Eyüp, 2026-07-14): test boyunca RC kumandaya HİÇ dokunulmadı** → tetik insan hatası değil, FC güç verilince KENDİ o duruma geldi. Üç aday mekanizma: (a) mod kanalının dinlenme PWM'i AUTO bandında (o akşamki tuhaf kalibrasyon: CH2 üst uçta dinleniyordu — mod kanalı da benzer olabilir); (b) verici kapalıysa alıcı failsafe çıkışı kayıtlı PWM basıyor, FC gerçek RC sanıyor; (c) `INITIAL_MODE` / `ARMING_REQUIRE` gibi boot parametreleri (ARMING_REQUIRE=0 → açılışta arm'lı!). Ortak nokta: görev hafızada + çıkışlar açıkken GÜÇ VERMEK YETERLİ.
- **YENİ VERİ 2 (Eyüp, 2026-07-14): olay anında verici AÇIKTI** → (b) alıcı-failsafe yolu ELENDİ. Baş şüpheli artık (a): mod kanalının dinlenme PWM'i AUTO bandına düşüyor (kalibrasyon tuhaflığıyla tutarlı); (c) boot parametreleri ikinci sırada.
- **YENİ VERİ 3 + (d) adayı (Eyüp, 2026-07-14): ortam KAPALI ALANDI, GPS şüpheli.** GPS kötülüğü tek başına görev BAŞLATMAZ ama: bozuk/sıçrayan fix + uzak sahte wp = tam gaz davranışını açıklar. Öte yandan M5'te fix'siz GUIDED reddedilmişti → fix hiç yoktuysa AUTO'nun kabulü şüpheli; bu durumda **(d) adayı:** koşan şey görev değil, MANUAL modda tuhaf kalibrasyonlu kanalların DİNLENME değeri düz gaz bastı (RC2 üst uçta dinleniyordu: 943/2137/2146, trim≈max!). Güç döngüsünde "devam etmesi" bununla da tutarlı.
- **✅ KÖK NEDEN KESİNLEŞTİ (2026-07-14, USB üzerinden parametre dökümü — log: `~/girdap_logs/fc_teshis/teshis_20260714_124710.txt`):** ZİNCİR = (1) `ARMING_RUDDER=2` + dümen kanalı CH2'nin (`RCMAP_YAW=2`) boşta ~MAX'ta durması (TRIM 2137 / MAX 2146) → FC "dümen sağda tutuluyor" sanıp KENDİLİĞİNDEN ARM; `ARMING_CHECK=0` olduğundan hiçbir ön kontrol engellemedi. (2) Mod kanalı CH6 (`MODE_CH=6`) boşta 1577 → MODE4 dilimi (1491-1620) → `MODE4=10` = **AUTO**. (3) AUTO + hafızadaki görev + `BRD_SAFETY_DEFLT=0` = motorlar tam güç; `MIS_RESTART=0` güç döngüsünde devam ettirdi. Yani (a)+(d) BİRLİKTE + (c) kısmen (ARMING_CHECK=0).
- **✅ AKSİYON (2026-07-14):** görev hafızası MAVROS'tan silindi, geri okuma `waypoints: []` DOĞRULANDI (FC USB beslemede, motor rayı güçsüzken).
- **⚠️ FC EKİBİ KARARI (2026-07-14, Eyüp iletti):** diğer parametreler (BRD_SAFETY_DEFLT=0, ARMING_RUDDER=2, MODE4/5=AUTO, ARMING_CHECK=0) BİLİNÇLİ/doğru kabul edildi, DEĞİŞTİRİLMEDİ. **KALAN RİSK:** pil+RC açıkken tekne her güç verişte kendiliğinden ARM+AUTO'ya düşmeye devam eder; hafızada görev OLDUĞU SÜRECE sürer. → YENİ OPERASYON KURALI (aşağıda OPS-1).
- **Durum:** KAPANDI (kök neden + yakıt giderildi); kalıntı risk OPS-1 kuralıyla yönetiliyor.

### [2026-07-14] OPS-1 — Görev yüklenen HER oturumun sonunda FC hafızası SİLİNİR (🟠 kalıcı kural)
- **Neden:** FC parametreleri (ekip kararıyla) kendiliğinden ARM+AUTO'ya izin veriyor; hafızada görev kalırsa 12.07 olayı AYNEN tekrarlanır (suda daha tehlikeli).
- **Kural:** M3/QGC Upload yapılan her test/prova/çekim gününün SON işi `/mavros/mission/clear` + `waypoints: []` geri-okuma teyidi (ya da QGC → Plan → Remove All + Upload). test-plani.md genel kurallarına eklendi.
- **Durum:** AÇIK (kalıcı operasyon kuralı — kapanmaz, uygulanır).
- **Etki:** güvenlik — bir sonraki güç verişte pervanesiz zorunlu temizlik yapılmadan HİÇBİR test koşulmaz.
- **Durum:** AÇIK. Aksiyon: (1) `/mavros/mission/clear`, (2) `BRD_SAFETY_DEFLT=1` geri, (3) RC mod kanalı/FLTMODE incelemesi + şu parametreler okunacak: `MODE_CH`, `MODE1-6`, `INITIAL_MODE`, `ARMING_REQUIRE`, `FS_THR_ENABLE` + verici açık/kapalı iken `RC_CHANNELS` (mod kanalı PWM'i hangi banda düşüyor). Olay anında verici açık mıydı → Eyüp'e soruldu, bilinmiyorsa ölçümle ayırt edilecek.

### [2026-07-12] DONANIM — Pixhawk USB-C soketi arızalı (🟠)
- **Belirti:** `descriptor read error -32`, low/full-speed titremesi; cihaz numaralanamıyor.
- **Debug verisi:** `docs/donanim_gunlugu_2026-07-12.md` (çapraz-test reçetesi dahil).
- **Kök neden:** fiziksel soket şüpheli (ayar/parametre bu tabloyu üretemez).
- **Etki:** IMU 57600 tavanında ~10 Hz (hedef ~50 Hz); geçici çözüm TELEM2 `fcu_url=serial:///dev/ttyUSB0:57600`.
- **Durum:** AÇIK — FC ekibinde (tamir ya da SR2 paramlarıyla idare).

### [—] F5.1 — LiDAR z-filtresi yanlış çerçevede, `lidar_height_m` yok (🔴, bloke)
- **Belirti:** üretim config'de gerçek dubalar elenir → `obstacle_map` boş → MPPI dubaların içinden geçer.
- **Debug verisi:** `docs/kod_denetimi.md` F5.1/F6.2; atölye testi: üretim config 0 engel (beklenen).
- **Kök neden:** noktalar sensör çerçevesinde, filtre su-hattı varsayıyor; `h` bilinmiyor.
- **Etki:** Parkur-2 (T1). Videoyu bloke ETMEZ.
- **Durum:** BLOKE — mekanik `h` ölçüsü bekliyor (`olcum_formu.md`). Geldiğinde: üreteç+testlerle AYNI commit + min_range değerlendirmesi.

### [2026-07-12] F-L.2 — kamera-LiDAR sync ~0.2 s zaman kayması (🟡)
- **Belirti:** Livox stamp'i Jetson saatinden ~0.2 s geride; çiftler kaymalı eşleşiyor.
- **Debug verisi:** canlı deney 90 çıkış/20 sn; bearing sapması ~0.06 rad < tol 0.15 (`50004e9` notu).
- **Kök neden:** Livox saat kaynağı ≠ Jetson saati; slop 0.1 s.
- **Etki:** düşük — eşleştirme çalışıyor, hassasiyet payı yiyor.
- **Durum:** AÇIK, düşük öncelik. Karar T1'de: restamp / slop 0.3 / PTP.

### [—] MODEL — duba NN Archive yok + sınıf sırası ters riski (🔴, video sonrası)
- **Belirti:** `~/models/yolo11n_duba_rvc2.tar.xz` Jetson'da yok (`jetson_kontrol.sh` tek HATA satırı); `Gazebonew.pt` names: 0=Engel, 1=Kenar — kod sabitlerinin TERSİ.
- **Debug verisi:** girdap-ida-algi `docs/bekleyen_girdiler.md` §B; PC'de `Gazebonew.pt` data.pkl dökümü.
- **Kök neden:** arşiv hiç üretilmedi; sınıf sırası eğitimden geliyor.
- **Etki:** Parkur-2 algı (T1). Kod tarafı `_sinif_indeksleri_coz()` ile isimden çözüyor ama saha `getClasses` log teyidi pazarlıksız.
- **Durum:** BLOKE — NN Archive üretimi bilinçli video SONRASINA ertelendi.

### [—] F5.5 / F5.6 — sözleşme bulguları (🟡)
- F5.5: HSV etkin menzil ≈15 m, sözleşmeye yazılmadı. F5.6: `score` alanına iki repo farklı anlam yüklüyor (doluluk oranı vs YOLO güveni).
- **Durum:** AÇIK, T1 doküman/karar işi. Ayrıntı: `docs/kod_denetimi.md`.

---

## ✅ KAPANANLAR (özet — kanıt kod_denetimi.md + commit'lerde)

| Kod | Ne idi | Düzeltme |
|---|---|---|
| F-M.1 | fix'siz görevde planning_node 92 GB cupy OOM (n_ref patlaması) | upstream `dff52af` |
| F-M.2 | kasıtlı disarm yine FAILSAFE→KILL basıyordu | upstream `3931220` |
| F-L.1 | Livox karışık-dtype × `read_points_numpy` → node ilk mesajda ölüyordu | `d9778fe` |
| F-A.1 | cupy `Generator.normal` yok → GPU'da AttributeError | Faz A, `c612fb0` |
| F12.1 | `last_waypoint_xy=[0,0]` → sahte P1→P2 geçişi | `788c46e` |
| F11.1/F9.1 | MPPI her hedefte yeniden yaratılıyordu → warm-start kaybı/zikzak | `aaf3f73` |
| F12.2 | video terminal durumu yok → istasyon-tutma titremesi | `ec7e1f5` |
| F14.1/F14.2 | KILL disarm etmiyor / kasıtlı disarm=failsafe | `0c7e1b6` |
| F15.1 | Dosya-2 CSV göreli yol → systemd'de çökme (5 ceza) | `c2308a2` |
| F10.1/F10.2 | RRT* replan ValueError node öldürüyor / bounds | `5ee87b8` |
| F5.3 | kümeleme O(n²) → scipy + voxel (~10×) | `a6aae64` |
| F5.4 | 500+ nokta kümesi sessizce siliniyordu → böl | `798ff4d` |
| Bearing (F6.1/F5.9) | işaret hatası + üreteç maskesi | `e66cb40` |
| F16.1 | pytest yanlış-yeşil (launch_testing) | pyproject addopts |
| F-M.7 | restart/boot'ta FC hiç bağlanmadan heartbeat-KILL latch (`ever_connected` bekçisi) | `c2d7a10` / girdap-video `8af6c4c`; ✅ CANLI DOĞRULANDI (2026-07-14 boot provası: 30 sn busy penceresinde FAILSAFE 0, state ARM) |
| F-M.3 | servis-KILL FCU'yu disarm etmiyordu (yalnız FSM/thrust sıfırlanıyordu) | `_on_mission_state` KILL gözleyince `_trigger_kill()` çağırıyor (`test_fm3_*`); bu satır AÇIK bırakılmıştı, kod zaten düzeltilmişti — doc-senkron düzeltmesi (Sude/Claude, 2026-07-14) |
| F-S.1 | girdap_decision'da RC donanım kill-switch (companion computer'dan bağımsız) hiç yoktu | `MavrosBridge.is_rc_kill_active()` + `mavros_bridge_node._on_rc_in()` (`/mavros/rc/in`, ida_topics ile aynı kanal/eşik), Sude/Claude 2026-07-14 |

Tam liste ve kanıtlar: `docs/kod_denetimi.md`.
