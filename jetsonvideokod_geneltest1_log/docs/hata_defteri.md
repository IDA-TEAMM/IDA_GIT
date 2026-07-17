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
### 🔴 Aksiyon bekleyen (video blokeri)

### [2026-07-16] FC-OLAY-2 — SERVO passthrough + bozuk RC kalibrasyonu: RC açılınca motorlar TAM GÜÇ (🔴🔴 VİDEO BLOKERİ, kuru test olayı)
- **Belirti:** Kuru test (21:14 ARM, QGC'de FC ekibinden arkadaş). RC kumanda kapatılıp
  açıldı → motorlar TAM GÜÇ döndü (Eyüp canlı bildirdi). FC modu **HOLD'du ve
  DURDURMADI**; disarm da güvence değil (aşağıda).
- **Debug verisi (canlı okuma 21:16-21:17, mavros üzerinden):**
  - FC paramları: `SERVO1_FUNCTION=52` (RCIN2 passthrough), `SERVO3_FUNCTION=53`
    (RCIN3 passthrough), `INITIAL_MODE=0`, `RC3_TRIM=2000` (= MAKSIMUM!),
    `FS_THR_VALUE=972`, `BRD_SAFETY_DEFLT=0` → **16.07 17:15 pixhawktest.md bulgusu
    AYNEN duruyor; Yahya'nın "yazdım" dediği değerler FC'de YOK.**
  - `rc/in`: ch1=1503 ch2=**1000** ch3=1488 ch7=**2000** (bir anahtar yukarıda, işlevi?)
  - `rc/out`: ch1=**1000** ch3=**1488** → **out1 = in-ch2, out3 = in-ch3 BİREBİR** =
    passthrough canlı kanıtlandı (out ch1=Sol, ch3=Sağ motor — 12.07 masa eşleşmesi).
  - Öncesinde RC kapalıyken (21:15:05) **F-T.5b uyarısı doğru teşhis koydu:**
    "rc/out akıyor ama kanal 1/3 ARM'dan beri PWM=0 — SERVOx_FUNCTION eşleşmesini kontrol et".
  - journal 21:14:54: "Warning: Arming Checks Disabled" + "Throttle armed" → F-T.4
    ARM rotasyonu kayit/7'yi açtı (kayıt düzeni 2. kez canlı doğru).
- **Kök neden:** SERVO1/3=52/53 passthrough → FC'nin mod/arming/failsafe mantığı motor
  çıkışları üzerinde TAMAMEN devre dışı; çubuk motora kablo gibi bağlı. Üstüne RC
  kalibrasyonu bozuk (yaylı ch2 dinlenmede 1000 basıyor + RC3_TRIM=2000 = trim maksta,
  12.07 OLAY deseni) → RC açılır açılmaz çıkışlara uç değer gitti. `ARMING_CHECK=0` +
  `BRD_SAFETY_DEFLT=0` kalan emniyet katmanlarını da kaldırmış. 12.07 FC-OLAY ile aynı
  aile: param disiplinsizliği.
- **Etki:** (1) **GÜVENLİK** — masada habersiz tam güç motor; (2) **AUTO'da FC motor
  süremez** (throttle fonksiyonu hiçbir çıkışa atanmamış) → görev başlar, tekne
  KIMILDAMAZ = video-katil (md 3.3.1.1); (3) HOLD/disarm passthrough'u kesmiyor →
  md 4.2 güç-kesme provası bu paramlarla anlamsız.
- **Durum:** AÇIK — düzeltme QGC'deki FC ekibinde (biz FC'ye YAZMAYIZ):
  `SERVO1_FUNCTION=73` + `SERVO3_FUNCTION=74` + `INITIAL_MODE=4` + **RC kalibrasyonu
  baştan** (ch2 dinlenme ~1500, RC3_TRIM≈1500) → FC reboot → Jetson'dan 5'li +
  `fc_param_turu.sh` ile teyit → ANCAK ondan sonra ARM+AUTO kuru testine devam.
- **➕ İLERLEME (aynı akşam 21:4x-22:0x, canlı teyitli):** QGC'den YAZILDI —
  `SERVO1=73` ✓ `SERVO3=74` ✓ `INITIAL_MODE=4` ✓ `RC3_TRIM=1500` ✓; passthrough BİTTİ,
  HOLD'da çıkışlar 1500/1500 nötr (FC kapılaması çalışıyor). **KALAN: gaz kanalı (ch3)
  çubuk-altta 905 < FS_THR_VALUE=972 → sürekli radio failsafe (CRITICAL) → zorla HOLD →
  motor dönmüyor** + RC kalibrasyonu yarım (`RC3_MAX=1500` — gaz yukarı itilmemiş).
  Aksiyon sırası suda-prova memory'sinde (kumanda ch3 endpoint → kalibrasyon baştan →
  FC reboot → Jetson teyit turu → pervanesiz MANUAL motor testi → servis restart →
  ARM+AUTO). Yan kazanım — **mod haritası:** MODE_CH=6; ch6 ALT=MANUAL(0),
  ORTA=AUTO(10), ÜST=RTL(11) → kuru testte kendiliğinden RTL'in açıklaması.
- **➕ YKİ KARARI (2026-07-17, Eyüp):** yer istasyonu **QGC → MISSION PLANNER'a
  geçildi** — MP bu sistemde daha kararlı; QGC'de yaşanan kararlılık/param-yazım
  sıkıntısı faslı KAPANDI (16.07'deki "yazdım dedi ama FC'de yok" deseninin YKİ
  ayağı). Bundan sonraki param yazımı + RC kalibrasyonu + Plan/Upload adımları
  **Mission Planner'dan** yürür; bu defterdeki ve runbook'lardaki "QGC" tarifleri
  MP karşılığıyla okunur (qgc-video-ayarlari rehberinin MP karşılığı AÇIK İŞ).

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
- **➕ TAZE KANIT (2026-07-15 17:29, USB'den yeniden takıldı):** aynı kötü-kontak
  imzası TEKRARLADI — 17:29:14 takıldı (yalnız ttyACM0) → 17:29:18 KENDİ düştü →
  17:29:19 çift-enum (ACM0+ACM1). Soket değişmeden USB-C'ye güven YOK.
  (Aynı oturumda MM-ignore udev kuralı doğrulandı: ModemManager portu KAPMADI.)
- **Durum:** 🔴 AÇIK — **video ÖNCESİ DONANIM BLOKERİ.** Aksiyonlar: (1) FC/mekanik ekip
  soketi KESİN çöz / DEĞİŞTİR + gerginlik boşaltma (strain relief); (2) yedek hat hazırla+test:
  TELEM2 FTDI serial `fcu_url=serial:///dev/ttyUSB0:57600` (57600'de IMU ~10 Hz = F-M.6 için
  bize yeten hız; FTDI kablo mevcut mu + port çakışması QGC MicoAir ile SORULACAK). Yazılım
  respawn/histerezis (F14.4) bu arızayı ÖRTME aracı DEĞİL — kırılgan güvenlik hattını yazılımla
  otomatik kurtarmak suda tanımsız duruma dönebilir; kök çözüm donanım.

### 🟡 T1 / düşük öncelik (video'yu bloke etmez)

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
- **Durum:** ✅ **KURAL KURULDU (Eyüp, 2026-07-15 16:55-16:57):**
  `scripts/udev/99-girdap-fc.rules` → `/etc/udev/rules.d/` (md5 teyitli) +
  reload + trigger (journal 16:57: udevd worker taraması). İçerik: MM-ignore
  (a seçeneği, Pixhawk idVendor 3162 + FTDI) + F-M.9 için `/dev/pixhawk`
  FTDI symlink'i (DU0EFEA7). **KAPANIŞ KRİTERİ (bekliyor):** FTDI/Pixhawk
  takılıyken `/dev/pixhawk` oluşmalı; bir sonraki boot'ta journal'da
  "could not grab port" / "busy" satırı OLMAMALI → o gün kapanır.
  Eski seçenek listesi (tarihçe): (a) udev ID_MM_DEVICE_IGNORE — SEÇİLDİ;
  (b) MM disable; (c) ExecStartPre bekleme.

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

### [2026-07-12] F-L.2 — kamera-LiDAR sync ~0.2 s zaman kayması (🟡 → slop düzeltmesi 2026-07-17; kayma telafisi T1'de)
- **Belirti:** Livox stamp'i Jetson saatinden ~0.2 s geride; çiftler kaymalı eşleşiyor.
- **Debug verisi:** canlı deney 90 çıkış/20 sn; bearing sapması ~0.06 rad < tol 0.15 (`50004e9` notu).
- **Kök neden:** Livox saat kaynağı ≠ Jetson saati; slop 0.1 s ölçülen kaymanın ALTINDA
  (kayma 0.1 s'i aşarsa eşleşme TAMAMEN durur; node'un `_sync_watchdog`'u bile
  "sync_slop_s'i büyüt" diyordu).
- **✅ Uygulanan (2026-07-17, Eyüp "zamanlama hatalarını atlama"):** `sync_slop_s`
  0.1→**0.3** dört config kaynağında birden (node varsayılanı + hardware.yaml +
  params.yaml + launch fallback'i); TDD `test_fl2_sensor_sync_slop.py` 4 test (config
  kaynakları bir daha ayrışamaz). Yanlış-eşleşme bekçisi değişmedi
  (`bearing_tolerance_rad=0.15`); duba statik → 0.3 s'de anlamlı yer değiştirmez.
- **Durum:** eşleşme-kaybı riski KAPANDI (suda canlı teyit bekler — fusion zinciri
  video bypass'ında sürüşü etkilemez, Parkur-2 işi); kaymanın KENDİSİ (hassasiyet)
  için restamp / PTP kararı T1'de durur.


### ⛔ BLOKE — dış girdi bekliyor (ölçüm / donanım / model)

### [2026-07-12] DONANIM — Pixhawk USB-C soketi arızalı (🟠)
- **Belirti:** `descriptor read error -32`, low/full-speed titremesi; cihaz numaralanamıyor.
- **Debug verisi:** `docs/donanim_gunlugu_2026-07-12.md` (çapraz-test reçetesi dahil).
- **Kök neden:** fiziksel soket şüpheli (ayar/parametre bu tabloyu üretemez).
- **Etki:** IMU 57600 tavanında ~10 Hz (hedef ~50 Hz); geçici çözüm TELEM2 `fcu_url=serial:///dev/ttyUSB0:57600`.
- **Durum:** AÇIK — FC ekibinde. ⚠️ **Bu kayıt F-M.9 ile AYNI arızanın erken hâli**
  (12.07 ilk teşhis → tamir denendi → 14.07 tamir TUTMADI, soket kendi düştü).
  **Güncel kayıt + kanıt zinciri + aksiyonlar için F-M.9'a bak**; bu kayıt tarihsel
  başlangıç olarak duruyor. TELEM2 FTDI yedek hattı 14.07'de canlıya alındı (8.1 Hz).

### [—] F5.1 — LiDAR z-filtresi yanlış çerçevede, `lidar_height_m` yok (🔴, bloke)
- **Belirti:** üretim config'de gerçek dubalar elenir → `obstacle_map` boş → MPPI dubaların içinden geçer.
- **Debug verisi:** `docs/kod_denetimi.md` F5.1/F6.2; atölye testi: üretim config 0 engel (beklenen).
- **Kök neden:** noktalar sensör çerçevesinde, filtre su-hattı varsayıyor; `h` bilinmiyor.
- **Etki:** Parkur-2 (T1). Videoyu bloke ETMEZ.
- **Durum:** BLOKE — mekanik `h` ölçüsü bekliyor (`olcum_formu.md`). Geldiğinde: üreteç+testlerle AYNI commit + min_range değerlendirmesi.

### [—] MODEL — duba NN Archive yok + sınıf sırası ters riski (🔴, video sonrası)
- **Belirti:** `~/models/yolo11n_duba_rvc2.tar.xz` Jetson'da yok (`jetson_kontrol.sh` tek HATA satırı); `Gazebonew.pt` names: 0=Engel, 1=Kenar — kod sabitlerinin TERSİ.
- **Debug verisi:** girdap-ida-algi `docs/bekleyen_girdiler.md` §B; PC'de `Gazebonew.pt` data.pkl dökümü.
- **Kök neden:** arşiv hiç üretilmedi; sınıf sırası eğitimden geliyor.
- **Etki:** Parkur-2 algı (T1). Kod tarafı `_sinif_indeksleri_coz()` ile isimden çözüyor ama saha `getClasses` log teyidi pazarlıksız.
- **Durum:** BLOKE — NN Archive üretimi bilinçli video SONRASINA ertelendi.

### 📌 Kalıcı operasyon kuralı (kapanmaz, uygulanır)

### [2026-07-14] OPS-1 — Görev yüklenen HER oturumun sonunda FC hafızası SİLİNİR (🟠 kalıcı kural)
- **Neden:** FC parametreleri (ekip kararıyla) kendiliğinden ARM+AUTO'ya izin veriyor; hafızada görev kalırsa 12.07 olayı AYNEN tekrarlanır (suda daha tehlikeli).
- **Kural:** M3/QGC Upload yapılan her test/prova/çekim gününün SON işi `/mavros/mission/clear` + `waypoints: []` geri-okuma teyidi (ya da QGC → Plan → Remove All + Upload). test-plani.md genel kurallarına eklendi.
- **Etki:** güvenlik — bir sonraki güç verişte pervanesiz zorunlu temizlik yapılmadan HİÇBİR test koşulmaz.
- **Durum:** AÇIK (kalıcı operasyon kuralı — kapanmaz, uygulanır).
- *(Not 2026-07-15: burada FC-OLAY'ın eski aksiyon listesi mükerrer duruyordu —
  görev silme ✅ yapıldı, `BRD_SAFETY_DEFLT=1` geri alma FC ekibi kararıyla
  REDDEDİLDİ, param incelemesi kök nedeni buldu, "verici açık mıydı" cevaplandı.
  Hepsinin öyküsü KAPANANLAR'daki FC-OLAY kaydında; satır bayat olduğu için kaldırıldı.)*

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
| F-M.3 | servis yoluyla KILL FCU'yu DISARM etmiyordu | girdap-video `98b5386` (TDD) |
| F-M.6 | FC taze bağlantıda ~1 Hz yayınlıyordu; yığın akış hızı istemiyordu | `41d205f`; ✅ CANLI (10 Hz istendi → IMU 9.99 Hz; boot provasında 10.03 Hz) |
| F-V.6 | "önce AUTO sonra ARM" akışında görev FSM'de hiç başlamıyordu (VİDEO-KATİL) | `41d205f` (`start_on_arm_in_mode`) |
| F-V.7 | AUTO'da yon_setpoint waypoint üstünde savruluyor + sahte dwell | `41d205f` (`dwell=0.0` + `yaw_setpoint_min_dist_m`) |
| F-V.8 | AUTO'da görev ilerlemesi FC'den senkronlanmıyordu | `41d205f` (`/mavros/mission/reached`, volatile QoS) |
| F-P.1 | planning bayat pozla MPPI koşuyordu (kör sürüş) | `41d205f` (`odom_timeout_s`) |
| F-T.1 | Dosya-2/Ekran-2 kaynak susunca DONUK son değeri yazıyordu | `49cb8e8` / girdap-video `fd86143` (`source_timeout_s` + `_fresh()`) |
| F-T.2 | tek parkurlu görevde (video) sahte PARKUR2 satırı | `49cb8e8` / girdap-video `fd86143` |
| F-S.5b | local_map yazma hatası node'u düşürüyordu → Dosya-3 kesilir | `49cb8e8` / girdap-video `fd86143` |
| FC-OLAY | FC hafızadaki sahte görevi RC/AUTO ile kendi koştu (motorlar tam güç) | kök neden kanıtlandı (ARMING_RUDDER=2 × CH2 + CH6→MODE4=AUTO); görev silindi, `waypoints: []` teyitli; kalıntı risk → OPS-1 |
| F-M.4 | fix'siz senaryoda bridge 10 Hz GUIDED-isteği spam'i | `mode_retry_interval_s` retry hız sınırı (TDD, 2026-07-15) |
| F5.5/F5.6 | HSV menzil + `score` semantiği sözleşmede yoktu | CLAUDE.md sözleşmeye yazıldı + kod yorumu (2026-07-15) |
| F-T.3 | thrust/setpoint sütunları tazelik bekçisi dışında (öz-denetim bulgusu) | damga + `_fresh()` CSV+grafik (TDD, 2026-07-15) |
| F-M.10 | NTP saat sıçraması sahte heartbeat-KILL + MPPI durdurma (16.07 kuru testi blokladı) | tüm yaş/tazelik hesapları `time.monotonic()`'e (5 node, TDD 6 test, 2026-07-17; Eyüp onayı "zamanlama hatalarını düzelt") |
| F-L.2 | kamera-LiDAR sync: Livox stamp ~0.2 s geride, slop 0.1 dar (kayma büyürse eşleşme durur) | `sync_slop_s` 0.1→0.3 4 config kaynağında (TDD 4 test, 2026-07-17); kayma telafisi (restamp/PTP) T1'de, suda canlı teyit bekler |
| F16.5 | fusion_node F8.2 "bayat poz → yayın durur" testi izole-dışı 9/10 flaky (alıcı-sayımı DDS teslim gecikmesine yarışlı; node kusuru YOK) | ölçüm node `publish()` sayacına çevrildi (timer'la eşzamanlı → gecikmesiz); 25/25 izole yeşil (2026-07-17) |
| F16.6 | perception_fusion `test_synced` yalnız GRUP koşusunda ~3/4 flaky: kamera-node buoys (stamp 0,456) fusion pose'una (0,123) slop içinde yanlış eşleşiyordu → `['1','99','99']` (cross-test kontaminasyon; node kusuru YOK) | her `_exchange`'e süreç-benzersiz büyük stamp (`_EXCHANGE_STAMP_SEQ`, 0/555'ten uzak); grup 20/20 yeşil (2026-07-17) |

Tam liste ve kanıtlar: `docs/kod_denetimi.md`.

### Kapanan kayıtların ayrıntıları (debug verisi + ölçümler korunur)

### [2026-07-17] F16.5 + F16.6 — tam-suite debug turunda 2 flaky ROS node testi (🟠 CI güvenilirliği → düzeltildi, stres-doğrulamalı)
- **Bağlam:** "tüm kodu debug + test et" turunda 337 test tek tek + grup koşuldu.
  Çekirdek + node testlerinin tümü yeşil; İKİ node testi kararsız çıktı. İkisi de
  ÜRETİM node'unda DEĞİL, TEST kurgusunda; kök neden ROS/DDS zamanlama-izolasyonu.
- **F16.5 — fusion_node F8.2 "bayat poz → yayın durur" (izole-dışı 9/10 fail):**
  - *Belirti:* `test_bypass_stale_pose_stops_publishing` tek başına ~1/10, ardışık
    koşuda 9/10 fail (`assert len(odoms) == n_after_stale`).
  - *Debug:* publish anları monotonic ölçüldü → node bayatlık eşiğinden
    (≈t_stop+0.25 s) SONRA HİÇ yayınlamıyor (F8.2 guard doğru, `stale_warned=True`).
    Fail nedeni: eşikten ÖNCE yayınlanmış son mesajlar RELIABLE DDS teslim
    gecikmesiyle sabit 0.8 s ölçüm penceresinden SONRA alıcıya damlıyor.
  - *Kök neden:* ölçüt yanlış — ikinci bir abonenin ALDIĞI mesaj sayısı DDS teslim
    gecikmesine tabi; herhangi bir sabit/dingin pencere aşılabilir. NODE kusuru YOK.
  - *Düzeltme:* ölçüm node'un GERÇEK `publish()` çağrılarına çevrildi (timer
    callback'iyle eşzamanlı → gecikmesiz). Bayatlık sonrası publish sayacı
    ARTMAMALI; gerçek regresyonu (bayat pozla yayın sürerse 50 Hz'de sayaç büyür)
    hâlâ deterministik yakalar.
  - *Sonuç:* **25/25 izole yeşil** (öncesi 1/10).
- **F16.6 — perception_fusion `test_synced_messages_produce_expected_fusion`
  (yalnız GRUP koşusunda ~3/4 fail):**
  - *Belirti:* izole 5/5 geçer; node testleri aynı süreçte grup koşunca
    `class_ids == ['1','99','99']` (beklenen `['0','1','99']`). Tespit SAYISI (3)
    doğru, SINIFLAR yanlış.
  - *Debug:* girdi `rng(42)` deterministik + `fusion.associate` birim-testli →
    doğru mesajla HER ZAMAN `['0','1','99']`. Demek synchronizer YANLIŞ mesaj
    eşledi. `test_perception_camera_node` kamera node'unu `/perception/buoys`'a
    **stamp (0,456)** ile yayınlatıyor; fusion testi pose+det'i **(0,123)** ile.
    İki üretici de `sec=0` → grup koşusunda kamera-node'un bayat buoys mesajı
    DDS'te kalıp fusion pose'una slop (0.3 s) İÇİNDE yanlış eşleşiyor.
    `stamp_sec=555` kullanan diğer 2 test bu yüzden HİÇ fail etmiyordu (555 s
    uzaklık, slop dışı) — desen kanıtı.
  - *Kök neden:* sabit paylaşımlı stamp (sec=0) × paylaşımlı topic adı ×
    süreç-paylaşımlı DDS = cross-test kontaminasyon. Gerçek dağıtımda (ayrı
    process + sürekli ARTAN sensör stamp'i) oluşmaz → NODE kusuru YOK.
  - *Düzeltme:* her `_exchange`'e süreç-benzersiz, sabit test stamp'lerinden
    (0/555/987) UZAK bir sec (`_EXCHANGE_STAMP_SEQ`, 10000+). Bayat mesaj artık
    slop içine denk gelemez; gerçek sensörün artan-stamp davranışını da taklit eder.
  - *Sonuç:* **grup 20/20 (+ önceki 2) = 22/22 yeşil, izole 5/5** (öncesi grupta ~3/4).
- **Genel doğrulama:** tam suite fix'lerle **335 passed / 2 skipped** (2 skip = MPPI
  CUDA, VMware'da beklenen); taban 335/2 KORUNDU (test sayısı değişmedi, 2 test
  deterministik yapıldı). Değişen yalnız 2 test dosyası; üretim node kodu +
  çekirdekler dokunulmadı.
- **Durum:** düzeltme commit'i (bu tur): `prototype/tests/test_fusion_node.py`,
  `prototype/tests/test_perception_fusion_node.py`.

### [2026-07-16→17] F-M.10 — NTP saat sıçraması sahte heartbeat-kaybı KILL'i basıyordu (🟠 → düzeltildi 2026-07-17, TDD)
- **Belirti:** Boot 21:09:34 (saat ~2s10dk GERİ — artık desen). 21:11:06'da NTP saati
  **+7774 sn ileri** düzeltti; AYNI saniyede üç node birden patladı: mavros_bridge
  "FAILSAFE — heartbeat kaybı (7775.0s) → KILL" + fsm "*** KILL ***" + planning
  "poz 7774.5s'dir gelmiyor → MPPI DURDURULDU". **FC canlıydı, gerçek kayıp YOKTU.**
- **Debug verisi:** `journalctl -u girdap-karar` 21:11:06; F-T.6 uyarısı aynı anda doğru
  bastı; yaş (7775.0 s) = sıçrama (7774 sn) birebir örtüşme = kanıt.
- **Kök neden:** yaş/tazelik hesapları duvar saatine (`get_clock` = ROS system time)
  dayalıydı; ileri sıçrama son mesajı "7775 sn önce" gösterir → eşik patlar → KILL latch.
- **Düzeltme (2026-07-17, Eyüp onayı "zamanlama hatalarını düzelt"):** göreli yaş
  hesapları `time.monotonic()`'e alındı — **5 yüzey:** (1) mavros_bridge `_now()`
  (heartbeat→KILL), (2) planning `_now()` (poz bayatlığı→MPPI durdurma), (3) fusion
  F8.2 bekçisi (`_now()` yardımcısı eklendi), (4) telemetry `_now()` (F-T.1/F-T.3
  tazelik — sıçramada CSV sütunları sahte boşalıyordu), (5) **mission_manager `_now()`
  (YENİ BULGU: dwell sıçramada anında "doluyor", waypoint sahte ilerliyordu).**
  Mesaj/CSV damgaları duvar saatinde KALDI (md 4.2; F-T.6 uyarısı yerinde).
  Perception log-throttle'ları (kozmetik) bilinçli dokunulmadı.
- **TDD:** `prototype/tests/test_fm10_saat_sicramasi.py` — 5 sıçrama testi (kırmızı
  kanıtlandı: 5 failed → düzeltme → 6/6 yeşil; +7774 sn birebir senaryo) + gerçek-sessizlik
  regresyon bekçisi. Tam suite 331/2 → F-L.2 ile birlikte **yeni taban 335/2.**
- **⚠ Kalan operasyonel gerçek:** CSV/log DAMGALARI hâlâ duvar saati → saat yanlışsa
  damgalar yanlış kalır. Suda kuralı DEĞİŞMEDİ: saati kur → SONRA servisi restart et.
  Canlıya restart'la biner (16.07 akşamki servis hâlâ eski kodla koşuyor).
- **➕ SAATİN GERİ KALMASININ KÖK-NEDENİ (paralel oturum teşhisi, 2026-07-17 —
  Eyüp "saat neden geri gidiyor"):** dil/locale DEĞİL. (a) EN OLASI: **Jetson RTC
  pili yok/zayıf** — kapalıyken saat donar, açılışta kapalı kalınan süre kadar geride
  (+7774 sn ≈ 2sa10dk = "2 saat kapalıydı" deseni; TZ offset'i 3 sa OLMAZDI);
  (b) ikinci ihtimal RTC local-time/UTC karışması (tam 3 sa sabit fark verirdi).
  **Ayırt etme:** `timedatectl` → `RTC in local TZ: yes` ise
  `sudo timedatectl set-local-rtc 0`; timezone Europe/Istanbul olmalı; kapat-aç
  sonrası yine gerideyse = RTC pili (donanım, video sonrası). Saha kuralı aynı:
  `sudo date -s ...` → SONRA servis restart.

### [2026-07-15] F-T.3 — thrust + setpoint sütunları tazelik bekçisinin DIŞINDA kalmıştı (🟠 → düzeltildi; ÖZ-DENETİM bulgusu)
- **Kaynak:** Eyüp'ün "sıfırdan her bakış açısıyla incele" turu — F-T.1 diff'inin
  taze gözle yeniden okunması. Kendi düzeltmemizin eksik yüzeyi.
- **Belirti (repro kırmızı):** kaynak ölünce AYNI ekranda hız/heading dürüstçe
  boşalırken (F-T.1) thrust ve yon_setpoint DONUK akmaya devam ediyordu:
  (a) fc modunda FC/mavros ölürse `/mavros/rc/out` susar → Ekran-2c thrust
  eğrisi son PWM yüzdesinde donar; girdap modunda planning ölürse aynı şey;
  (b) görev AKTİFKEN mission_manager ölürse `current_target` susar →
  yon_setpoint donuk (F-V.2 yalnız görev-DIŞINI kapılıyordu, kaynak ölümü
  ayrı durum); (c) girdap modunda cmd_vel susunca hiz_setpoint donuk.
- **Kök neden:** F-T.1 yalnız sensör sütunlarına damga ekledi; thrust/setpoint
  cache'leri damgasız kaldı → `_fresh()` onları hiç görmüyordu.
- **Etki:** md 3.3.1.1 Ekran-2b/2c — aynı sahte-canlı-veri sınıfı. fc-cruise
  sabiti BİLEREK kapsam dışı (config sabiti, kaynağı yok).
- **Durum:** ✅ DÜZELTİLDİ (TDD 3 test: 2 kırmızı→yeşil + F-V.7 tutma regresyon
  bekçisi): `_thrust_t` (`_on_thrust` + `_on_rc_out`) + `_target_t` (F-V.7
  açıyı TUTARKEN de damgalanır — tazelik "kaynak canlı mı" sorusu, "değer yeni
  mi" değil) + `_setpoint_t`; CSV + grafik yolunda `_fresh()`. Suite 315/2.

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
- **Durum:** ✅ DÜZELTİLDİ (2026-07-15, TDD 4 test kırmızı→yeşil): çekirdeğe
  `should_request_mode(now)` + `note_mode_requested(now)` (stream_rate
  deseniyle aynı işbölümü), `mode_retry_interval_s: 2.0` (0=kapalı; node +
  launch + hardware.yaml kablolu). İlk istek hemen, retry'lar ≥2 sn aralıklı;
  mod düzelince istek kesilir (regresyon testli). Sahada davranış değişmez
  (fix ARM'dan önce → zaten tek istek).

### [—] F5.5 / F5.6 — sözleşme bulguları (🟡 → belgelendi 2026-07-15)
- F5.5: HSV etkin menzil ≈15 m sözleşmeye YAZILDI (CLAUDE.md Kamera Pipeline:
  türetim + 15-25 m bandı yalnız-LiDAR/CLASS_UNKNOWN davranışı + menzil-artırma
  takası). Test kanıtı zaten vardı (`scene_camera_menzil_siniri`).
- F5.6: `score` semantiği SÖZLEŞMEYE BAĞLANDI (CLAUDE.md + camera_buoys.py
  yorum): kaynağa bağlı — HSV=doluluk oranı (şekil), YOLO=ağ güveni; güven
  eşiği olarak tüketmek YASAK. 2026-07-15 grep teyidi: hiçbir tüketici yok
  (fusion pass-through, planning okumuyor) → davranış riski kapalı, karar belgeli.

### [2026-07-15] F-T.1 — Dosya-2/Ekran-2 kaynak susunca DONUK son değeri yazıyordu (🔴 md 4.2 veri dürüstlüğü → düzeltildi)
- **Kaynak:** Yahya'nın `sonkodv3` denetimi ("BULGU 2"; kendi gps-kayip senaryo
  koşusunda görülmüş — bizde bağımsız repro ile teyit edildi). Aynı bulguyu
  Sude de `son_kodv2`'de bağımsız kapatmış (`c4ef30f`).
- **Belirti:** GPS/IMU/velocity_body susunca telemetry cache'teki son değeri
  her tick yazmaya devam ediyordu → CSV'de veri CANLI görünüyor. Repro testi
  (`test_bulgu2_hiz_kaynagi_susunca_bos`) kırmızıda: 3 sn sessizlikten sonra
  `assert '1.400' == ''` — hız donuk.
- **Kök neden:** `telemetry_node` cache'lerinin tazelik damgası YOKTU; `_on_write`
  değeri koşulsuz yazıyordu. F-V.2 kapılaması yalnız SETPOINT sütunlarına
  uygulanmıştı, sensör sütunları açıkta kalmıştı.
- **Etki:** md 4.2 Dosya-2 (hakem telemetriyi canlı sanar) + md 3.3.1.1 Ekran-2
  (hız/heading eğrisi donuk düz çizgi). GPS kesintisi suda gerçekçi senaryo.
- **Durum:** ✅ DÜZELTİLDİ (TDD, 3 test kırmızı→yeşil). `source_timeout_s: 3.0`
  + `_fresh()` bekçisi; CSV **ve** grafik yolunda; `<=0` → kapalı (mock/masa).
  Odom yedek yolları (F15.4 hız + heading) da damga tazeliyor. Yeni taban 308/2.

### [2026-07-15] F-T.2 — tek parkurlu görevde SAHTE PARKUR2 (🟠 Dosya-2 dürüstlüğü, videoyu etkiler → düzeltildi)
- **Kaynak:** Yahya `sonkodv3` denetimi ("BULGU 1"). Bizde bağımsız repro ile teyit.
- **Belirti:** repro testi kırmızıda: `parkur son index'leri={1: 1}` (parkur-2
  YOK) olmasına rağmen son waypoint'e varışta MissionFSM PARKUR2'ye geçti
  (`assert <MissionState.PARKUR2> is <MissionState.PARKUR1>`).
- **Kök neden:** `fsm_node._on_waypoint_reached` parkur-1'in son index'inde
  `dist_to_last_wp_p1 = 0.0` KOŞULSUZ besliyordu; `ParkurTransitionLogic` kendi
  geçişini parkur varlığına göre kapılıyor ama MissionFSM gözlemi kapılanmamıştı.
- **Etki:** **video senaryosu tam da tek parkur** (4 nokta) → Dosya-2'ye (md 4.2)
  hiç koşulmayan PARKUR2 satırı düşer. Pencere `mission_complete`'e kadar; bizde
  `dwell=0.0` olduğu için kısa ama gerçek.
- **Durum:** ✅ DÜZELTİLDİ (TDD): sinyal yalnız `2 in last_index_of_parkur` iken
  beslenir. Gerçek parkur-2 yolu regresyon testiyle korundu (`labels=[1,1,2]`).

### [2026-07-15] F-S.5b — local_map yazma hatası node'u düşürüyordu → Dosya-3 sessizce kesilir (🟡 T1 → düzeltildi)
- **Kaynak:** Yahya'nın F-S.5'i (onun çekirdeğinde `write_frame` None dönüyor;
  BİZDE dönüş tipi `Path` → o guard burada ölü kod olurdu). Gerçek risk exception.
- **Belirti/repro:** `test_local_map_node.py` kırmızıda — bozuk grid
  (`data` uzunluğu != w*h) `np.reshape` ValueError'ı, disk dolu `OSError`;
  ikisi de timer callback'inden yükselip node'u düşürüyor.
- **Kök neden:** `_on_tick` yazma çağrısı korumasızdı.
- **Etki:** md 4.2 Dosya-3 görev ortasında sessizce durur, teslim eksik kalır.
- **Durum:** ✅ DÜZELTİLDİ (TDD, 2 test): `(ValueError, OSError)` yakalanır,
  throttle'lı ERROR log + kare atlanır, node yaşar. Sonraki sağlam kare yazılıyor
  (testle kanıtlı). Ek: `LocalMapNode(**node_kwargs)` passthrough (test enjeksiyonu).

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

### [2026-07-14] F-M.3 — servis yoluyla KILL FCU'yu DISARM etmiyor (🟠)
- **Belirti:** Oturum 2 masa testi (M6a): `/girdap/mission/kill` çağrısı sonrası FSM=KILL ✓,
  thrust [0,0] ✓, AMA `/mavros/state` `armed: true` KALDI (5+ sn sonra tekrar teyit).
- **Debug verisi:** `~/girdap_logs/masa_testi/masa_stack_2026-07-14_oturum2.log` —
  yalnız `[fsm_node] *** KILL — motorlar durduruluyor ***` var, bridge'ten disarm logu YOK.
- **Kök neden:** F14.1 düzeltmesi disarm'ı yalnız `mavros_bridge_node._trigger_kill()`
  içine koydu (heartbeat kaybı / beklenmedik disarm yolu). Operatör/YKİ kill servisi
  doğrudan `fsm_node`'a gider; bridge `/girdap/mission/state`'teki `KILL`'i yalnız
  F14.3 görev-aktif geçidi için okuyor (`_on_mission_state`), disarm TETİKLEMİYOR.
- **Etki:** masa runbook M6a PASS kriteri ("sıfır thrust + FCU disarm") sağlanmıyor;
  md 3.3.1(4) güç-kesme gösteriminin yazılım katmanı eksik kalır (fiziksel anahtar asıl
  mekanizma olduğundan 🟠, 🔴 değil). Araç KILL sonrası ARMED kalır → RC'den gaz riski.
- **Durum:** AÇIK → TDD düzeltme bu oturumda (bridge `_on_mission_state` KILL gözleyince
  `_trigger_kill()` çağırsın — disarm + latch; kill servisi çağrısı idempotent).

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


### [2026-07-16] F-T.4 — görev CSV'si boot kirliliğiyle açılıyordu; ARM rotasyonu eklendi (🟠 → düzeltildi)
- **Belirti (16.07 masa):** 94 dk'lık dosyanın ilk ~50 dk'sı boş/BOOT satırı;
  `run_ekran2` "en yeni" seçimi reboot sonrası yanlış dosyayı bulabiliyordu
  (`--csv` elle seçim tuzağı). Kaynak: kayıt yalnız servis boot'unda açılıyordu.
- **Düzeltme (Yahya GÖREV 1, TDD):** kayıt boot'tan sürmeye devam eder;
  `telemetry_node` FC'nin GERÇEK arm'ının (`/mavros/state.armed`, FSM değil)
  yükselen kenarında iki CSV'yi kapatıp yeni çift açar. İlk örnek armed=True
  ise kenar SAYILMAZ (görev ortası restart dosyayı bölmesin). Aynı saniyede
  ikinci kenar `unique_filename` sonekiyle çözülür (ada-göre sıralama korunur;
  çekirdek testli). Testler: rotasyon kenarları + sonek + find_latest uyumu.
- **Durum:** ✅ kod+test tamam; canlı doğrulama restart+ARM döngüsüyle.

### [2026-07-16] F-T.5 — fc modunda thrust boşluğunun sebebi loglardan teşhis edilemiyordu (🟡 → düzeltildi)
- **Belirti (16.07 masa):** thrust sütunları tüm oturum boş, logda tek uyarı
  yok. Laptop analizi "rc/out hiç mi gelmedi, PWM=0 mı" AYIRT EDEMEDİ;
  canlı ölçüm gerçeği gösterdi: rc/out 8 Hz AKIYORDU, kanal 1/3 PWM=0
  (RC kapalı + SERVO1/3=RCIN passthrough).
- **Düzeltme (Yahya GÖREV 2 + (b) genişletmesi, TDD):** `rc_warn_after_s`
  (10 sn) sonra BİR KEZlik iki ayrı uyarı: (a) bağlı ama rc/out HİÇ yok →
  stream (SRx_RC_CHAN); (b) rc/out akıyor ama seçili kanallar ARM'dan beri 0 →
  SERVOx_FUNCTION/safety. (b) yalnız arm oturumunda bakar (disarm'da 0 normal)
  ve her arm kenarında resetlenir — 16.07 vakasını (a) değil (b) yakalardı.
- **Durum:** ✅ kod+test tamam; girdap modunda uyarı üretilmez (testli).

### [2026-07-16] F-T.6 — sistem saati sıçraması CSV'ye sessizce giriyordu (🟡 → düzeltildi)
- **Belirti (16.07 boot):** saat boot'ta 3s16dk geriydi; NTP kaydın ORTASINDA
  düzeltti → tek CSV'de zaman deliği (Dosya-2 zaman etiketi md 4.2 şartı).
- **Düzeltme (Yahya GÖREV 3, TDD):** `_check_clock_jump` duvar↔monotonic
  farkını tick başına izler; >30 sn değişim = sıçrama → BİR KEZ warn
  ("kayıtları şüpheli say"). Damgalar DÜZELTİLMEZ (Dosya-2 duvar saati ister).
  KÖK NEDEN operasyonel: koşudan önce `timedatectl` senkron kontrolü
  (runbook §0-A/5'e eklendi). Ekran-2 tarafında eksen telafisi (Yahya GÖREV 4)
  P2 — MP4 artık PC'de üretildiği için ertelendi.
- **Durum:** ✅ kod+test tamam.

### [2026-07-16 akşam] F-T.4 REVİZE — Eyüp isteği: numaralı kayıt klasörleri + eski log temizliği
- **İstek:** "log1 log2 gibi gitsin, log 2 silinirse gene log 2'den devam
  etsin" + "eski logları da silsin" + "her kayıt kendi klasöründe, şu an
  çok dağınık".
- **Uygulama (TDD):** `~/girdap_logs/kayit/<N>/telemetri.csv + grafik.csv` —
  boot bir kayıt açar, FC ARM kenarı yenisini (F-T.4 rotasyonu aynen);
  `next_kayit_num` = en küçük boş numara (silinen yeniden kullanılır);
  `prune_old_kayit_dirs` = `kayit_sakla_adet`(20) üstü en eskiden silinir
  (dosya zamanına göre; aktif kayıt korunur; <=0 kapalı).
  `find_latest_graph_csv` ada-göre → DOSYA ZAMANINA göre (numara yeniden
  kullanıldığı için ad sıralaması yalan söylerdi); eski `grafik_*.csv`
  düzeni geçiş adayı olarak kaldı. `unique_filename` gereksizleşti, SİLİNDİ
  (numara benzersizliği ad çakışmasını imkânsız kılar). `csv_output_dir`/
  `graph_output_dir` paramları → `kayit_dir` + `kayit_sakla_adet`.
  run_ekran2 çıktısı `ekran2_kayit<N>` adını alır.
- **Suite:** 325/2 (322→325; +4 yeni test, unique_filename testi silindi).
- **Durum:** ✅ kod+test; canlı doğrulama restart+ARM döngüsüyle (eski
  telemetry/ + grafik/ dizinleri ~/girdap_logs/eski_duzen/ altına arşivlendi).
