# Otonomi Kabiliyeti Videosu — Saha Günü Runbook'u

> **ELEME KAPISI:** video 21.07.2026 17:00'a kadar KYS'ye yüklenmezse ya da
> kriterleri karşılamazsa takım finale KALAMAZ (şartname md 3 + 3.3) —
> P1+P2+P3 tamamı sıfırlanır. Bu doküman çekim gününün adım adım prosedürü.
> Her adım şartname maddesine ve koddaki gerçek karşılığına bağlıdır.
>
> Yazım: 2026-07-11. Kod referansları commit `ae7af84` itibarıyla doğrulandı.
> Revizyon: 2026-07-13 — 20 fazlı video denetimi (docs/video_denetimi.md)
> sonrası: F-V.1 (yon_setpoint AÇI) uygulandı, Şekil-1 yerleşimi §6'ya,
> "4 nokta — dönüş noktası EKLEME" kuralı §3'e işlendi (F-V.4).
> Revizyon: 2026-07-14 — **AUTO DÖNÜŞÜ (§0-A)**. Aşağıdaki §1/§4'teki
> "GUIDED" başlatma anlatımı ESKİ plandır; §0-A geçerli olandır.

## 0-A. ⚠️ 2026-07-14 AUTO DÖNÜŞÜ — bu bölüm eski GUIDED adımlarını EZER

Video artık **AUTO+FC** ile koşulur (B1/B2 + F-V.6/7/8, hata_defteri):
görevi FC kendi uçurur, MPPI cmd_vel basmaz. Değişen operatör adımları:

1. **Başlatma modu GUIDED değil AUTO.** QGC'den mod→**AUTO** (ya da QGC
   "Start Mission"). Sıra FARK ETMEZ: önce ARM sonra AUTO da, önce AUTO
   sonra ARM da çalışır (F-V.6 `start_on_arm_in_mode: true`). Eski §1'deki
   "boot'ta GUIDED'daysa HOLD'a al geri dön" cambazlığı GEREKSİZ.
2. **Ekran-2 thrust'ı FC servo çıkışından, YÜZDE cinsinden** gelir
   (`telemetry.setpoint_source: "fc"`). Montajda:
   `python scripts/run_ekran2.py --mp4 --thrust-birim %` — **`--thrust-birim %`
   UNUTMA**, yoksa eksen "N" yazar (yanlış birim = md 3.3.1.1 riski).
   **KARAR (16.07): MP4 render'ı PC'DE yapılır** (Jetson ~3 kare/sn = 30-50 dk;
   PC ~5-15 dk) — adım adım: repo kökünde `pc-render-yonergesi.md`.
3. **hız_setpoint sabit çizgidir** (= `fc_cruise_setpoint_mps` = FC `WP_SPEED`).
   Çekimden önce senkron teyidi TEK KOMUT: `bash scripts/fc_param_turu.sh`
   (FC bağlıyken; WP_SPEED↔fc_cruise + WP_RADIUS↔arrival_radius karşılaştırır,
   motor tavanı MOT_THR_MAX/SLEWRATE + failsafe + OLAY paramlarını okur,
   OPS-1 görev-boş kontrolü yapar — SALT OKUR, FC'ye hiçbir şey yazmaz).
4. **Çekim sabahı deploy kontrolü** (servis yeni kodla mı):
   `ros2 param get /fsm_node start_on_arm_in_mode` → **true** olmalı;
   `ros2 param get /mission_manager_node dwell_time_s` → **0.0** olmalı;
   `ros2 param get /telemetry_node source_timeout_s` → **3.0** olmalı
   (F-T.1/F-T.3 tazelik bekçisi — bayat sensör/thrust donuk yazılmaz);
   journalctl'de "FC akış hızı isteniyor: 10 Hz" satırı olmalı. Biri
   tutmuyorsa: `sudo systemctl restart girdap-karar` → tekrar sorgula.
   📌 **fcu_url notu (F-M.9 sonrası):** hat TELEM2 FTDI'da; udev kuralı
   kuruluysa (2026-07-15 ✓) FTDI takılınca `/dev/pixhawk` oluşur →
   `fcu_url: serial:///dev/pixhawk:57600` tercih edilir (ttyUSB0 numarası
   reboot'ta kayabilir).
   ⏱️ **Boot/restart sonrası ~1 dk bekle:** ttyACM0 kısa süre meşgul kalıyor
   (F-M.8, ModemManager şüphesi) → mavros FC'ye ~45 sn'de bağlanır. KILL
   basmaz (F-M.7 fix'i), sadece geç hazır olur — "10 Hz" satırı gelmeden
   ARM'a geçme.
5. **Saat kontrolü (GÖREV 3, 16.07):** servise güç vermeden/koşudan önce
   `timedatectl` → "System clock synchronized: **yes**" beklenir; değilse
   `sudo date -s 'YYYY-MM-DD HH:MM:SS'` (sahada NTP yok — 16.07 boot'unda
   saat 3s16dk geriydi, NTP kaydın ORTASINDA sıçrattı). telemetry_node
   sıçramayı yakalarsa journalctl'de "sistem saati sıçradı" uyarısı basar →
   o oturumun CSV'sini şüpheli say.
6. **Kayıt düzeni (GÖREV 1 rev., 16.07):** her koşu kendi numaralı
   klasöründe — `~/girdap_logs/kayit/<N>/telemetri.csv + grafik.csv`.
   Boot bir kayıt açar; FC gerçekten ARM olduğu anda YENİSİ açılır
   (journal: "ARM algılandı → yeni kayıt: …/kayit/N") → görev verisi temiz
   klasörde, `run_ekran2` varsayılanı (dosya zamanına göre en yeni) görev
   klasörünü bulur. Numara = en küçük boş (silinen yeniden kullanılır);
   eski kayıtlar `kayit_sakla_adet`(20)'i aşınca en eskiden otomatik
   silinir → **görev klasörünü koşudan sonra USB/PC'ye almayı unutma**.
   fc modunda rc/out akmıyorsa ya da kanallar ARM'dan beri 0 ise journal'da
   BİR KEZlik uyarı çıkar (GÖREV 2) — çekimden önce gör, sebebini çöz.
7. Görev bitişi artık FC'nin kendi varış sinyalinden de senkronlanır
   (F-V.8) — 4. noktada FSM TAMAMLANDI'ya düşer, setpoint çizgileri kesilir;
   manuel dönüşte setpoint görünmesi = bir şeyler ters, çekimi yeniden planla.
8. WiFi kapalı (rfkill) — **Bluetooth da kapat**: `rfkill block bluetooth`
   (BT klavye kullanılıyorsa USB klavye getir; md 4.1 2.4 GHz yasağı).

## 0. Şartname → runbook eşlemesi

| md 3.3.1 zorunlu gösterim | Runbook adımı | Kod karşılığı |
|---|---|---|
| (1) İDA↔YKİ kablosuz; İDA bilgisi YKİ'de | §3.2 QGC bağlantısı | QGC ↔ MicoAir LR868 @868 MHz ↔ Pixhawk (MAVROS'tan bağımsız kanal; "RFD868x" eski not) |
| (2) YKİ'den 4 nokta tanımla + İDA'ya gönder | §3.4 görev yükleme | `mission_source:=fc` → `/mavros/mission/waypoints` (T0-f, `3856fb0`) |
| (3) YKİ/RC komutuyla başlat; 4. noktada tamamlan | §4 görev koşusu | **GUIDED-mod tetiği** (`start_on_mode`, §1) + F12.2 `mission_complete`→TAMAMLANDI |
| (4) Güvenlik anahtarıyla güç kes; RC'ye rağmen motor dönmesin | §5.2-5.3 | `/girdap/bridge/disarm` (F14.2 beklenen-disarm) + fiziksel anahtar |
| (5) Kapak açıp su almazlık | §5.4 | donanım gösterimi |
| (6) Tek gösterim, kesintisiz | §5 tamamı | dış kamera TEK kesintisiz çekim |
| Ekran-1: YKİ ekranı | §3.3 + §6 | QGC ekran kaydı (OBS) |
| Ekran-2: hız+sp, heading+sp, thrust — senkron | §6 | `telemetry_node` grafik CSV → `scripts/run_ekran2.py` (`e7e87e6`) |
| Ekran-3: dış kamera | §3.5 + §6 | ≥720p kamera + tripod |

## 1. ✅ KARAR VERİLDİ + UYGULANDI: başlatma = QGC'den GUIDED-mod komutu

Arka plan: başlatma yalnız `/girdap/mission/start` ROS servisiydi; RC
dinleyen kod yok, md 4.1 WiFi yasağı yüzünden kıyıdan SSH da yok
(MicoAir LR868 MAVLink taşır, IP taşımaz) → md 3.3.1(3) "YKİ/RC'den komutla
başlat" karşılıksızdı. Eyüp Seçenek A'yı onayladı (2026-07-11), uygulandı.

**Mekanizma (`fsm_node`, `start_on_mode` parametresi, varsayılan GUIDED):**
araç BEKLEMEDE'deyken operatör QGC'den modu **GUIDED**'a çevirince
(`/mavros/state.mode` üzerinden görülür) görev başlar. F14.3 gereği
auto_guided görev-öncesi GUIDED basmadığından bu geçiş kesin operatör
komutudur. 4 testle doğrulandı (`test_fsm_node.py`).

**Operatör kuralları (kenar tetikli tasarımın sonuçları):**
- Sıra: **önce ARM** (MANUAL/HOLD modunda) → BEKLEMEDE → **sonra mod→GUIDED**
  = görev başlar. İki ayrı komut = güvenlik (arm etmek başlatmak değildir).
- Boot'ta araç zaten GUIDED'daysa tetik ÇALIŞMAZ (bilerek): modu bir kez
  HOLD/MANUAL'a alıp GUIDED'a geri dön.
- Tetiği kapatmak için `start_on_mode: ""` (başlatma yalnız servisle kalır).

**Not:** ARM için ek kod yok — FSM `armed`'ı `/mavros/state`'ten okur
(`fsm_node:311`), QGC'nin arm butonu MicoAir LR868 üzerinden yeter.
`/girdap/mission/start` servisi araç-üstü alternatif olarak duruyor.

## 2. Gün ÖNCESİ hazırlık (atölye)

- [ ] `hardware.yaml` bayrakları: `use_isam2: false`, `use_rrt: false`
      (video bypass — CLAUDE.md Video Modu), `control_rate_hz: 10` (F4.2).
- [ ] Jetson'da `ros-humble-mavros-msgs` kurulu (fc modu onsuz boş görevde
      bekler, çökmez — T0-f notu). `mavros` + GeographicLib dataseti kurulu.
- [ ] `pytest prototype/tests/` Jetson'da yeşil (README test bölümü).
- [ ] QGC ↔ MicoAir LR868 ↔ Pixhawk zinciri masada teyit: telemetri görünüyor,
      arm/disarm çalışıyor, görev yükleme gidiyor (`/mavros/mission/waypoints`
      latched geldiği ve home=index0 olduğu LOGDAN teyit — T0-f SAHA maddesi).
- [ ] ArduRover failsafe parametreleri FC ekibiyle teyit (F14.1: KILL artık
      FCU'yu disarm ediyor ama FCU'nun kendi GCS/throttle failsafe'i de düzgün
      kurulmalı). **Somut öneri listesi: `docs/fc_parametre_onerileri.md`**
      (OLAY 2026-07-12 sonrası yazıldı) — özellikle RC mod konumlarında AUTO
      olmaması + BRD_SAFETY_DEFLT=1 + RC kalibrasyonu çekimden ÖNCE bitmiş.
- [ ] **FC görev hafızası TEMİZ** (QGC Plan → Remove All) — eski/test görevi
      FC'de durursa yanlış mod geçişinde kendi koşar (OLAY 2026-07-12;
      masa runbook "M0-ÖNCESİ" bloğu).
- [ ] Jetson + YKİ laptopta dahili WiFi/BT KAPALI (md 4.1).
- [ ] YKİ laptopta ekran kayıt aracı (OBS vb.) kurulu, kayıt testi yapılmış.
- [ ] Dış kamera ≥720p: şarj, boş kart, tripod. Çekim yeri: tüm dikdörtgeni
      ve aracı NET gören açı (md 3.3.1.1 "hareket net değilse BAŞARISIZ").
- [ ] GUIDED-mod tetiği (§1) masa testinde denenmiş (QGC'den mod değişimi →
      Jetson logunda "YKİ mod komutu ... görev başlatıldı" satırı) ve suda
      provada teyit edilmiş.

## 3. Sahada kurulum sırası

1. **Fiziksel güvenlik anahtarı KAPALI** konumda başla; araç kızakta/iskelede.
2. **Güç ver** → Jetson boot. Yığını başlat:
   `ros2 launch girdap_decision hardware.launch.py mission_source:=fc`
   (systemd servisi varsa o; `mission_source` fc olduğunu logdan teyit et).
3. **QGC bağlantısı:** RFD868 linki yeşil, telemetri akıyor (md 3.3.1(1)).
   → **Ekran-1 kaydını (OBS) ŞİMDİ BAŞLAT** — görev tanımından güç kesmeye
   kadar her şey kayıtta olsun; fazlası montajda kırpılır.
4. **Görev tanımla + gönder (md 3.3.1(2)):** QGC Plan ekranında **TAM 4
   köşe — DİKDÖRTGEN** oluşturacak şekilde (koordinatları önceden §2'deki
   keşifte ölçtüysen gir, yoksa haritadan seç) → "Upload".
   ⚠ **Başlangıca-dönüş noktası EKLEME (F-V.4):** md 3.3.1(3) görevin
   4. noktada tamamlanmasını, dönüşün MANUEL yapılmasını ister; 5. nokta
   hem "4 noktalı görev" tanımını bozar hem dönüşü otonomlaştırır.
   Jetson logunda `mission_manager` "FC görevi alındı: 5 item → 4 waypoint"
   satırını teyit et (home dahil 5 item normaldir, home atlanır). Görev
   START'tan SONRA yüklenirse REDDEDİLİR (T0-f, md 5.5.2.2 provası) — sıra
   önemli.
5. **Dış kamera kaydını başlat** (md 3.3.1(6) tek kesintisiz çekim BURADAN
   güç kesme sonuna kadar). Aracı suya indir.

## 4. Görev koşusu

1. Fiziksel güvenlik anahtarını AÇ (motorlara güç hazır).
2. **GPS fix teyidi:** QGC'de "3D Fix" (ya da daha iyisi) görünmeden devam
   etme — F-M.1 (`dff52af`): geçerli fix yokken görev BAŞLAMAZ (Jetson logu:
   "geçerli GPS fix yok — görev başlatılmıyor"); fix gelince kendiliğinden
   başlar ama çekim akışında bunu beklemek yerine önce fix'i gör.
3. **QGC'den ARM** (YKİ komutu) → FSM: BOOT→ARM→BEKLEMEDE. QGC HUD'da
   armed göründüğünü Ekran-1 kaydı yakalasın.
   ⚠ ARM ile GUIDED arasında **1-2 sn bekle** (HUD'da "Armed" görünsün):
   tetik yalnız FSM BEKLEMEDE'deyken çalışır; çok hızlı ardışık komutta
   kenar kaçarsa çare basit — modu HOLD'a alıp GUIDED'a geri dön.
4. **Başlat: QGC'den modu GUIDED'a al** (ARM'dan SONRA — §1 kuralları) →
   FSM PARKUR1, araç P1'e seyir (~1 m/s, `cruise_velocity_mps`). Bu mod
   değişimi videodaki "YKİ'den komutla başlatma" anıdır — QGC mod menüsü
   Ekran-1 kaydında görünür olsun.
5. **Gözle ve gerekirse İPTAL ET:** zikzak / yerinde dönme / rota dışı
   sürüklenme = md 3.3.1.1 "istemsiz hareket → BAŞARISIZ". Görürsen çekimi
   durdur, tekrar çek (kendi videomuz — deneme sayısı sınırsız, süre 21.07'ye
   kadar). Acil durumda `/girdap/mission/kill` zinciri + RC failsafe.
6. 4. (SON) noktada `mission_manager` COMPLETE → FSM **TAMAMLANDI** →
   sıfır thrust (F12.2 + F13.1 zinciri). Araç süzülerek durur — bu an
   videoda "otonom görev tamamlandı" anıdır (md 3.3.1(3)); birkaç saniye
   bekle.

## 5. Görev sonu gösterimleri (KAMERA KESİLMEDEN)

1. **Manuel dönüş (md 3.3.1(3) sonu):** QGC/RC'den manuel moda al, kıyıya
   getir. auto_guided görev-aktif değilken mod savaşı YAPMAZ (F14.3,
   `88a06c8`).
2. **Kontrollü disarm:** araç kıyıda → `/girdap/bridge/disarm` (araç-üstü
   terminal varsa; yoksa QGC'den disarm). F14.2 (`0c7e1b6`) + F-M.2
   (`3931220`, masada yakalanan latch hatası) sonrası beklenen disarm
   failsafe sanılmaz — loglar temiz kalır. Disarm'dan sonraki birkaç saniyeyi
   de logdan izle (M6/2 teyidi buradaydı).
   ⚠ Not: QGC'den disarm edilirse köprü bunu "komutlu" BİLEMEZ (bayrağı
   yalnız `/girdap/bridge/disarm` servisi kurar) → sahte FAILSAFE logu düşer;
   video çekiminde disarm'ı MÜMKÜNSE servis üzerinden yap, değilse bu logun
   zararsız olduğunu bil (KILL zinciri motoru zaten durduruyor, görev bitmiş).
3. **Güç kesme gösterimi (md 3.3.1(4)):** fiziksel güvenlik anahtarını
   kameraya göstererek KAPAT. Ardından **RC'den gaz/yön komutu ver →
   motorların DÖNMEDİĞİNİ yakın çekimde göster.**
   ℹ Güç kesildiği anda Jetson tarafında heartbeat-KILL / FAILSAFE logları
   düşmesi **BEKLENEN davranıştır** (FCU hattı kesildi; görev zaten bitmiş,
   thrust zaten sıfır) — panik yok, çekim akışını etkilemez.
4. **Su almazlık (md 3.3.1(5)):** kapakları kameraya karşı aç, iç bölmenin
   kuru olduğunu göster, kapat.
5. Dış kamera kaydını ŞİMDİ durdur — (1)-(5) tek kesintisiz çekimde
   (md 3.3.1(6)).

## 6. Veri toplama + montaj

1. Jetson'dan al: `~/girdap_logs/kayit/<N>/` — görev koşusunun klasörü
   (16.07 düzeni: her koşu kendi numaralı klasöründe, ARM yenisini açar;
   journal "ARM algılandı → yeni kayıt: …/kayit/N" numarayı söyler).
   İçinde `grafik.csv` (Ekran-2, 10 Hz) + `telemetri.csv` (Dosya-2) →
   **klasörü USB/scp ile PC'ye kopyala** (~2-5 MB).
2. Ekran-2 panelini **PC'de** üret (KARAR 16.07 — Jetson ~3 kare/sn, yedek yol):
   önce hızlı PNG ile görev penceresini bul, sonra
   `python3 karar/scripts/run_ekran2.py --csv <dosya> --thrust-birim % --mp4 --t0 <s> --t1 <s>`
   — tek seferlik PC kurulumu + ayrıntı: repo kökünde **`pc-render-yonergesi.md`**
   → hız+setpoint, heading+yaw setpoint, thruster kuvvet istekleri
   (md 3.3.1.1'in saydığı ÜÇ sinyal; MPPI maliyet grafiği bunların yerine
   GEÇMEZ).
3. **3 bölmeli montaj — yerleşim şartnamenin Şekil 1'i ile BİREBİR
   (md 3.3.1.1, s.12; serbest düzen DEĞİL):**

   ```
   ┌─────────────┬──────────────────┐
   │  1  Ekran-1 │                  │
   │  (üst-sol)  │   3   Ekran-3    │
   ├─────────────┤  (sağ, büyük     │
   │  2  Ekran-2 │   dikey bölme)   │
   │  (alt-sol)  │                  │
   └─────────────┴──────────────────┘
   ```

   - **Ekran-1 (üst-sol):** YKİ ekran görüntüsü (QGC/OBS kaydı).
   - **Ekran-2 (alt-sol):** run_ekran2 MP4 (üç temel grafik).
   - **Ekran-3 (sağ):** dış kamera; içerik videodaki AŞAMAYA göre değişir —
     şartname örnekleri: **RC kumanda** (başlatma/manuel anlar), **İDA'nın
     suda hareketi** (görev koşusu), **İDA'nın iç görüntüsü** (su almazlık
     gösterimi, md 3.3.1(5)). Yani güç-kesme ve kapak sahnelerinde Ekran-3'e
     ilgili yakın çekimi koy; görev koşusunda tekne genel planda kalsın.
   - **Senkron şartı (md 3.3.1.1):** "İDA'nın görev yaptığı aşamada YKİ
     ekranı ile İDA'nın hareketleri senkron bir şekilde görülmelidir" —
     görev başlama anını (araç ilk hareket) ÜÇ kaynakta aynı zaman noktasına
     getir; Ekran-2 MP4'ün zaman imleci gerçek süreyle akar, gerisi otomatik
     senkron kalır. Kaba senkron kontrolü: bir köşe dönüşü anında QGC harita
     ikonu, Ekran-2 heading eğrisi ve dış kameradaki dönüş AYNI anda olmalı.
4. **Biçim:** ≥720p, toplam 2-5 dk. Fazla görüntüyü kırp; görev koşusu +
   6 zorunlu gösterim eksiksiz kalsın.
5. YouTube'a **liste dışı** yükle → linki KYS'ye gir → **linki farklı
   cihaz/oturum açmamış hesapla TEST ET** ("linkte sorun = eleme").

## 7. Yayın öncesi son kontrol (videoyu baştan izleyerek)

- [ ] md 3.3.1'in 6 maddesi de videoda AÇIKÇA görünüyor (§0 tablosu).
- [ ] **Yerleşim Şekil 1 ile birebir:** Ekran-1 üst-sol, Ekran-2 alt-sol,
      Ekran-3 sağ (büyük dikey bölme).
- [ ] Hiçbir anda istemsiz dönüş/sürüş yok; hareket ve görüntü net.
- [ ] Ekran-2'de üç sinyal de var ve İDA hareketiyle senkron (köşe dönüşü
      testi: harita + heading eğrisi + dış kamera aynı an).
- [ ] Ekran-1'de görev tanımlama + gönderme + İDA telemetrisi görünüyor.
- [ ] Ekran-3 içeriği aşamaya uygun (görev=suda hareket, güç kesme=RC+motor
      yakın çekim, su almazlık=iç görüntü).
- [ ] Süre 2-5 dk, çözünürlük ≥720p.
- [ ] Link liste dışı, çalışıyor, KYS'ye girildi. **Son teslim 21.07 17:00.**

## 8. Plan B / notlar

- `mission_source:=file` + `video_mission.yaml` yalnız PROVA içindir —
  çekimde kullanılırsa md 3.3.1(2) ("YKİ'den gönderilecek") karşılanmaz.
- Yeniden çekim maliyetsizdir; şüpheli koşuyu YÜKLEME, tekrar çek.
- Çekim günü suya girmeden 60 sn'lik kuru koşu (kızakta, thruster'lar suda
  değilken ARM YOK — yalnız yığın/log/kayıt zinciri kontrolü) zaman kazandırır.
