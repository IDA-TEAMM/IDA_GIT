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

Tam liste ve kanıtlar: `docs/kod_denetimi.md`.
