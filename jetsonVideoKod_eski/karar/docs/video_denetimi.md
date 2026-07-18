# Otonomi Kabiliyeti Videosu — Şartname ↔ Kod Denetim Matrisi

> Oluşturma: 2026-07-13 (video derin denetimi, 20 fazlı tur).
> Şartname: `~/ida_sartname_2026.pdf` md 3 / 3.3 / 3.3.1 / 3.3.1.1 (s.10-12),
> 2026-07-13'te birinci kaynaktan yeniden okundu.
> Teslim: **21.07.2026 17:00 KYS** (Tablo 3, "Sistem Kabiliyeti Videoları") — ELEME KAPISI
> (md 3: "göndermeyen takımlar yarışmaya katılmaya hak kazanamayacaklardır";
> md 3.3: "Bu aşamayı geçebilen takımlar final aşamasında yarışmaya hak kazanacaktır").

Durum işaretleri: ✅ kod+test doğrulandı · 🔶 kod var, bu turda satır satır doğrulanacak ·
🔴 açık bulgu (düzeltme gerekli) · ⚠ donanım/saha gösterimi (kod işi değil) ·
📋 doküman/prosedür işi.

## 1. md 3.3.1 — Gösterimde beklenenler (6 madde)

| # | Şart (birebir özet) | Koddaki karşılık | Durum |
|---|---|---|---|
| 1 | İDA↔YKİ **kablosuz** bağlantı; İDA bilgileri YKİ'de gösterilecek | YKİ=QGC, Pixhawk'a RFD868x ile doğrudan bağlanır (MAVROS'tan bağımsız 2. MAVLink kanalı; `gcs_url:""` doğru). QGC telemetri gösterir. | ⚠ saha (RFD çifti + QGC laptop) |
| 2 | YKİ'den **4 noktalı** görev tanımlanacak, noktalar **dikdörtgen** oluşturacak, görev İDA'ya **gönderilecek** | QGC Plan → Pixhawk görev hafızası → `mission_manager_node` `mission_source:=fc` → `/mavros/mission/waypoints` (latched) → `fc_items_to_waypoints` → ENU wp'ler (T0-f, commit `3856fb0`) | 🔶 Faz 6-7 |
| 3 | YKİ ya da RC'den **bir komutla** görev başlar; **4. noktaya ulaşılınca otonom görev tamamlanmış olacak**; sonrası manuel dönüş serbest | Başlatma: `start_on_mode=GUIDED` kenar tetiği (T0-j, `fsm_node`); tamamlanma: `MissionManager` COMPLETE → `/girdap/mission/complete` → FSM TAMAMLANDI → thrust=0 (F12.2); manuel dönüş: `auto_guided` yalnız görev-aktif (F14.3) | 🔶 Faz 8-10 |
| 4 | Dönüş sonrası motor/güç **güvenlik butonu/anahtarı** ile kesilecek; RC'den komut verildiği hâlde motorların dönmediği gösterilecek | Fiziksel güç kesme = donanım; yazılım tarafı: kasıtlı disarm ≠ FAILSAFE (F14.2/F-M.2 düzeltmesi, kenar takibi), `/girdap/bridge/disarm` | ⚠ donanım + 🔶 Faz 9 |
| 5 | Kapaklar açılıp **su almadığı** gösterilecek | — | ⚠ mekanik |
| 6 | Hepsi **tek gösterimde, kesintisiz ve aralıksız** | — | 📋 çekim planı (runbook) |

## 2. md 3.3.1.1 — Video biçimi

| Şart | Koddaki karşılık | Durum |
|---|---|---|
| **Şekil 1 yerleşimi:** Ekran-1 ÜST-SOL, Ekran-2 ALT-SOL, Ekran-3 SAĞ (büyük dikey) | ✅ Faz 15: runbook §6/3'e ASCII yerleşim + §7 kontrol maddesi işlendi | ✅ (girdap-video listesi Faz 16) |
| Ekran-1: YKİ ekran görüntüsü | QGC ekran kaydı (OBS) | 📋 saha |
| Ekran-2a: **gerçek hız + hız isteği (setpoint)**, senkron | `telemetry_node` grafik CSV `hiz` (body-vel + odom yedeği F15.4/F8.1) + `hiz_setpoint` (cmd_vel linear.x) → `ekran2.py` panel 1 | 🔶 Faz 4 |
| Ekran-2b: **gerçek heading/yaw AÇISI + heading/yaw AÇISI isteği (setpoint)** | heading: MAVROS IMU ENU yaw ✓; setpoint: F-V.1 düzeltildi — `current_target`'tan `atan2(y,x)` AÇI (TDD, taban 258/4) | ✅ Faz 3 |
| Ekran-2c: **thrusterlardan kuvvet isteği** | `/girdap/control/thrust` → grafik CSV `thrust_sol/thrust_sag` (T0-g) → panel 3 | 🔶 Faz 4 |
| Ekran-3: dış kamera; aşamaya göre RC kumanda / suda hareket / iç görüntü; **görev aşamasında YKİ ekranı ile İDA hareketleri SENKRON** | ✅ Faz 15: runbook §6/3'e aşama-içerik eşlemesi + senkron kontrol yöntemi (köşe dönüşü testi) işlendi | ✅ |
| **İstemsiz ve hareket yönünden bağımsız dönüş/sürüş → BAŞARISIZ** | MPPI kararlılık zinciri: warm-start (F11.1/F9.1), terminal durum (F12.2), sahte geçiş yok (F12.1), gerçek hız durumu (F8.1), control_rate 10 Hz (F4.2) | 🔶 Faz 10 |
| Görüntü/hareket net değilse → BAŞARISIZ | Çekim kalitesi | 📋 |
| ≥720p, 2-5 dk | Montaj | 📋 Faz 16 |
| YouTube **liste dışı**, link KYS'ye; **linkte sorun = eleme** | — | 📋 Faz 16 (yükleme provası) |

## 3. Video zinciri — düğüm haritası (fc görev akışı)

```
QGC (RFD868x) ──MAVLink──> Pixhawk görev hafızası
                              │ (USB ttyACM0)
                        MAVROS /mavros/mission/waypoints (latched)
                              │
              mission_manager_node (mission_source=fc, skip_home_seq0)
                  fc_items_to_waypoints → ENU wp listesi
                              │ /girdap/mission/waypoints + current_target
              fsm_node (start_on_mode=GUIDED kenar tetiği; TAMAMLANDI)
                              │ /girdap/mission/state
              planning_node (bypass; MPPI 10 Hz; COMPLETE→None→thrust 0)
                              │ /girdap/control/thrust + cmd_vel
              mavros_bridge_node (arm/disarm/kill; expected_disarm)
                              │
              telemetry_node (Dosya-2 CSV + grafik CSV 10 Hz)
                              │
              ekran2.py (PNG + zaman imleçli MP4) → montaj
```

## 4. Açık bulgular (bu tur)

| ID | Önem | Bulgu | Durum |
|---|---|---|---|
| F-V.1 | 🔴→✅ | `telemetry_node._on_setpoint` yon_setpoint'e `angular.z` (yaw HIZI) yazıyordu; şartname Ekran-2b AÇI istiyor | ✅ Faz 3'te düzeltildi (TDD; ayrıntı kod_denetimi.md F-V.1) |

| F-V.2 | 🟡 | Görev TAMAMLANDI/KILL sonrası `telemetry_node` setpoint cache'i temizlenmiyor: `current_target` yayını durunca `yon_setpoint` son açıda DONUK kalır; manuel dönüşte heading değişirken grafik "istek var" gibi okunur. `hiz_setpoint` sorunsuz (COMPLETE'te cmd_vel 0 basılıyor, planning_node `_on_control_step` teyitli). Düzeltme: aktif-olmayan mission_state'te `_speed_sp/_yaw_sp=None` → CSV boş → ekran2 NaN boşluğu (sahte çizgi yok). | ✅ Faz 18'de düzeltildi (TDD: yazma-anı kapılaması `_mission_active`; F-V.1 testi doğru davranışa güncellendi) |
| F-V.3 | 🟡 | `_speed_from_body` kalıcı latch: velocity_body akışı ölürse odom hız yedeği (F15.4) bir daha devreye giremez, hız son değerde donar. F15.3 (heading latch) ile aynı aile — T1 watchdog işi, video için mavros zaten tek kaynak. | T1 not |
| F-V.4 | 🟠 | `video_mission.yaml` şablonunda **5. nokta "P1_return"** var (+ dosya/CLAUDE.md yorumları "4 nokta + başlangıca dönüş" diyor). Şartname md 3.3.1(2) "**4 noktalı** görev" + (3) "son noktaya ulaşıldığında görev tamamlanmış olacak, **dönüş MANUEL**" — otonom başlangıca-dönüş noktası şartname senaryosuna aykırı; Ekran-1'de (QGC planı) 5 nokta görünmesi de "4 noktalı görev tanımlanacak" maddesini bulandırır. Düzeltme: şablondan P1_return çıkar, yorumları düzelt; runbook/QGC talimatına "4 köşe, dönüş noktası EKLEME" yaz. | ✅ Faz 17 (runbook) + Faz 18 (şablon+CLAUDE.md+node docstring) |
| F-V.5 | 🟠 | **F3.3 hâlâ açık:** `hardware.launch.py:143` `except Exception: pass` — hardware.yaml bozuk/okunamazsa SESSİZCE `_ALGO_DEFAULTS` (isam2/rrt **True** = yarışma modu) kullanılır → video günü YAML yazım hatası = bypass sessizce kapanır, iSAM2+RRT* kalibrasyonsuz açılır (istemsiz hareket riski). Düzeltme: fallback kalsın ama stderr'e GÜRÜLTÜLÜ uyarı bas. | ✅ Faz 18 (TDD: test_hardware_launch_config.py) |
| F-V.6 | 🟡 | fc modunda parkur etiketleri hâlâ `mission_file` DOSYASINDAN geliyor (fsm_node parkur katmanı); FC görevi etiket taşımaz. Video etkilenmez (tek parkur, TAMAMLANDI mission_complete'ten). Yarışma günü fc + competition_mission.yaml SENKRON tutulmalı (QGC görevi ile dosya aynı sıra/sayıda) — T1 operasyon notu + olası index-sayısı doğrulaması. | T1 not |

**Faz 15-17 (dokümanlar) sonucu:** runbook §6/3 Şekil-1 ASCII yerleşimi + Ekran-3 aşama içeriği + senkron köşe-dönüşü testi + §7 yeni kontrol maddeleri (`1d21aa6`) · girdap-video kontrol-listesi.md aynı içerik + dikdörtgen + "dönüş noktası EKLEME" + yalnız-YouTube (`3945428`) · runbook tam geçişi: §3/4 ve §4/6'daki "kapanış noktası" talimatları KALDIRILDI (F-V.4 doküman ayağı — kod şablonu Faz 18'de), ARM→GUIDED 1-2 sn bekleme uyarısı, güç kesmede heartbeat-KILL loglarının BEKLENEN olduğu notu, revizyon damgası. §2 (FC güvenlik/OLAY/WiFi/OBS) ve §5/2 (QGC-disarm sahte FAILSAFE) ZATEN tamdı — dokunulmadı.

**Faz 14 (ekran2.py) sonucu — DOĞRULANDI:** 150 s gerçekçi görev profiliyle (4 bacak rotası 0→90→180→−90°, sarım geçişi, boot boşlukları, görev sonu sıfır thrust) uçtan uca üretim yapıldı ve PNG görsel incelendi: üç md 3.3.1.1 sinyali doğru panelde ✓ · yon_setpoint artık AÇI olarak heading'le aynı eksende anlamlı (F-V.1 görsel teyidi) ✓ · ±180° sarım kırılması artefaktsız ✓ · boş hücreler sahte 0 çizmiyor ✓ · header doğrulaması yanlış CSV'yi reddediyor ✓. MP4: 10 s pencere → 10.03 s video, imleç gerçek zamanla akıyor (senkron şartı montaj kanıtı) ✓; `--t0/--t1` kırpma çalışıyor ✓. Görsel not: görev sonunda donuk yön-setpoint çizgisi F-V.2'nin panele yansıması — Faz 18 düzeltmesi bunu boşluğa çevirecek.

**Faz 8-12 sonucu:** fsm_node GUIDED kenar tetiği + F12.1/F12.2 bütünlüğü ✓ (BEKLEMEDE şartı, boot'ta-GUIDED dışlaması, mission_complete→TAMAMLANDI) · mavros_bridge F14.1/F14.2(F-M.2)/F14.3 düzeltmeleri yerinde ✓, KILL zinciri disarm+FSM çift yollu ✓ · planning F11.1 warm-start koruması (`_rebuild_mppi` aynı config'de kontrolcüyü yaşatır, config değişiminde U_nominal taşır) ✓, F10.1 try/except + F10.2 bounds zarfı ✓, COMPLETE→None→sıfır thrust ✓ · F-M.1 üç katman ✓ (node null-island+fix+mesafe; MPPI `max_ref_points=2048`; params kablolu) · config video değerleri doğru (use_isam2/rrt false, control_rate 10 Hz, start_on_mode GUIDED, mission_source launch CLI `mission_source:=fc`). Açık: F-V.4, F-V.5 (yukarıda). Operasyon notları: fiziksel güç-kesme gösteriminde heartbeat-KILL/FAILSAFE logları BEKLENEN (görev bitmiş, zarar yok) + disarm için QGC değil `/girdap/bridge/disarm` servisi (sahte FAILSAFE logu önlenir) → Faz 17 runbook.

**Faz 6-7 (mission fc yolu) sonucu:** node + çekirdek TEMİZ — latched QoS ✓, skip_home_seq0 ✓, başladıktan sonra gelen liste reddi (md 5.5.2.2) ✓, F-M.1 üç katman ✓, NAV dışı komut + (0,0) filtresi ✓, waypoint_reached tek atış ✓, COMPLETE'te hedef yayını kesiliyor ✓. Bilinen T1: F13.5 `_started` latch (görev yeniden başlatılamaz — video tek çekim, sorun değil). **Eklenen tel:** `test_fc_dikdortgen_video_senaryosu` — md 3.3.1(2)+(3) birebir: home+4 köşe dikdörtgen → 4. noktada COMPLETE (17 passed). ⚠ Operasyon notu: QGC görevine başlangıca-dönüş noktası EKLENMEZ (md 3.3.1(3) dönüş MANUEL) — runbook'a işlenecek (Faz 17).

**Faz 4-5 temiz çıkanlar:** Dosya-2 2 Hz (≥1 Hz md 4.2 ✓) + her satır fsync ✓ · grafik CSV 10 Hz = control_rate (alias yok) ✓ · mavros abonelikleri sensor_data QoS (BEST_EFFORT uyumu) ✓ · GPS fix<0 yazılmıyor ✓ · heading IMU ENU yaw + odom yedeği ✓ · CSV_HEADER md 4.2 alanları birebir, testle dondurulmuş ✓ · GRAPH_CSV_HEADER Ekran-2'nin ÜÇ sinyalini de içeriyor (hız+sp, heading+sp, thrust sol/sağ) ✓ · `_fmt` None→boş (sahte 0 yok) ✓.

## 5. Faz planı (bu tur, 20 faz)

1. ✅ Taban teyidi (257 passed / 4 skip, `c77dca3`)
2. ✅ Bu matris
3. F-V.1 düzeltmesi (TDD)
4. telemetry_node denetimi · 5. csv_logger · 6. mission_manager_node fc ·
7. fc_items_to_waypoints sınır durumları · 8. fsm_node · 9. mavros_bridge ·
10. planning/MPPI · 11. F-M.1 zinciri · 12. config · 13. uçtan uca koşu ·
14. ekran2.py · 15. runbook montaj bölümü · 16. girdap-video kontrol listesi ·
17. runbook tam geçişi · 18. kalan düzeltmeler + suite · 19. çift-bakım senkronu
(girdap-video + girdap-ida subtree pull) · 20. kapanış + memory.

## 6. TUR KAPANIŞI (2026-07-13 gece) — Faz 1-20 TAMAM

**Kod tarafı sonucu:** Video zincirinin şartname uyumu satır satır doğrulandı;
4 düzeltme uygulandı (F-V.1 açı, F-V.2 setpoint kapılaması, F-V.4 4-nokta
şablonu, F-V.5 launch uyarısı) — hepsi TDD, suite taban **262 passed /
5 skipped**, CI yeşil. QGC 4-nokta görev akışı uçtan uca GERÇEK node
grafiğiyle test edildi (`test_video_e2e.py`): WaypointList(latched) → fc →
GUIDED kenar tetiği → 4 varış → TAMAMLANDI → thrust [0,0] → CSV'ler doğru.
Çift-bakım: girdap-ida + girdap-video `d7f9be6`e senkron, CI'lar yeşil.

**Kod işi KALMADI. Kalan her şey saha/donanım/prosedür:**
1. 🔴 FC güvenlik aksiyonları (İLK güç verişte, PERVANESİZ): mission clear +
   `BRD_SAFETY_DEFLT=1` + RC mod kanalı düzeni (OLAY 2026-07-12; runbook §2 +
   masa runbook M0-öncesi).
2. Jetson sıfırla+kur (BURADAN_BASLA.md) → `git pull` ile `d7f9be6`+ gelir.
3. Açık alan: M5 tam (GPS fix'li GUIDED) → M6 KILL → M2 QGC gerçek Plan
   Upload (laptop) → M7/M8.
4. Suda 4-nokta provası (çekimsiz) → ÇEKİM (runbook §3-5) → montaj (§6,
   Şekil-1 yerleşimi) → 20.07 hedefli yükleme+link testi (§7).
5. T1'e devreden notlar: F-V.3 (hız yedeği latch watchdog'u), F-V.6 (fc
   modunda parkur etiketi senkronu), F15.3/F15.5.

**Hatırlatma (Ekran-3):** video DIŞ kameradan çekilir — İDA bakış açısı
zaten YASAK (md 4.1 görüntü aktarımı yasağı + md 3.3.1.1 "dış kamera").
