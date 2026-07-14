---
name: kod-duzeltme
description: "İDA kod düzeltme günlüğü — yapılan fix'ler, tarih ve doğrulama durumu"
metadata: 
  node_type: memory
  type: project
  originSessionId: 97fbf033-cf13-4546-afbf-c6b146529521
---

İDA kod düzeltme günlüğü (kullanıcı isteği: kod fix'leri bu dosyada birikir, [[feedback-buyuk-isleri-kaydet]]).

**2026-07-14 — PlotJuggler canlı-izleme üçlüsü + doc drift (onaylı, uygulandı):**
Dosyalar hem `~/Downloads` hem `~/Desktop/video-girdap`'ta senkron.
1. `ground_speed_publisher.py`: `WP_SPEED 5.0 → 1.0` (karar 2026-07-13, hardware.yaml `fc_cruise_setpoint_mps` ile aynı; görev öncesi QGC'den teyit notu eklendi).
2. `ground_speed_publisher.py`: görev dışında (AUTO+armed değilse) setpoint artık HİÇ yayınlanmıyor — 0.0 çizgisi yerine grafikte boşluk (telemetry_node fc modu ile aynı mantık).
3. `plotjuggler_girdap.xml`: gereksiz `return 5.0` custom math snippet'i silindi; AUTO'da hep boş kalan `/mavros/setpoint_velocity/cmd_vel` + `setpoint_raw/target_attitude` abonelik listesinden çıkarıldı.
4. Desktop repo kopyasında RFD868→MicoAir doc düzeltmesi (fsm_node docstring, CLAUDE.md, video/masa runbook'ları, doğrulama matrisi, KTR; "868 MHz" iddiaları "frekans teyidi BEKLİYOR" oldu). Tarihli günlükler (donanim/pc_gunlugu_2026-07-12) bilerek DOKUNULMADI — geçmiş kaydı.

Doğrulama: fake_mavros_publisher ile uçtan uca test — ground_speed akıyor, setpoint 1.0 geliyor; XML parse ✓; pytest çekirdek 217 ✓ (düzeltmelerden önce koşuldu, node davranışı değişmedi).

**2026-07-14 (2) — girdap-video BİRLEŞİM (3-yönlü git merge, tamamlandı):**
`~/Desktop/video-girdap/girdap-video-main` = YENİ KANONİK repo. Taban: "girdap-video-main (1).zip" (14 Tem 13:47, video reposunun yeni işleri: girdap-karar.service, test_video_e2e, F-M.3 KILL-latch, F-V.5 launch fallback loglama, hata_defteri.md) + üstüne decision(2)'deki B1/B2 fix'leri + MicoAir doc düzeltmeleri. Ortak ata = decision-main (2).zip pristine; tek çakışma (test_telemetry_node add/add) iki test de tutularak çözüldü. Ek fix: test_video_e2e'ye importorskip kapısı eklendi (yoksa ROS'suz CI'da koleksiyon kırılıyordu); girdap-karar.service yorumunda GUIDED→AUTO. Doğrulama: ROS'suz 219✓, tam ortam (PYTHONPATH prepend — EZME, yoksa rclpy kaybolur) **267✓/6 skip**, e2e dahil. Eski `girdap-decision-main` klasörü artık gereksiz (birleşimin içinde).

**⚠ ROS_DOMAIN_ID=42:** Jetson bashrc + girdap-karar.service DDS domain 42 kullanıyor — laptopta PlotJuggler/ros2 CLI Jetson'ı görecekse `export ROS_DOMAIN_ID=42` ŞART (2026-07-13 bulgusu, servis dosyası yorumu).

**2026-07-14 (5) — SÜRÜCÜ ENTEGRASYONU (son_kod artık tam bağımsız):** IDA_GIT'in 13 Tem doğrulanmış `ida_topics` paketi son_kod'a alındı (`karar/ros2_ws/src/ida_topics_yeni`). setup.py'ye 6 eksik entry point; `hardware.launch.py`'a `with_drivers:=true` bayrağı (Livox+OAK+kamera_kayit, remap'lerle; video günü varsayılan KAPALI); kamera_kayit `/tmp` → `~/girdap_logs/kamera` + output_dir param (**kamera_kayit = Dosya-1 mp4 üreticisi — şartname denetimindeki eksik KAPANDI**); 4 sürücü main'ine KeyboardInterrupt; "sürücüler başka arkadaşta" notları güncellendi. CANLI testler: Livox sahte UDP→PointCloud2 birebir (x=5.150✓, mm→m parser✓) · OAK cihazsız zarif ("No available devices" log + ayakta) · kamera_kayit SIGINT'le 7.6 sn geçerli MP4 (SIGTERM'de mp4 bozuk kalır — servis SIGINT kullanmalı) · gps_imu köprüsü fake_mavros ile ✓ · 269 test ✓. NOT: OAK USB tek-sahip — algı ekibinin ayrı YOLO node'u kullanılırsa with_drivers'taki oakd ile çakıştırma. depthai laptopta da kurulu.

**2026-07-14 (4) — Emin-olana-kadar test turu (tamamlandı):** 269 test ✓ · e2e 5/5 tekrar kararlı · systemd servis sözdizimi ✓ (systemd-analyze) · WP_SPEED 3-nokta senkron grep ✓ · **canlı smoke:** gerçek telemetry_node (fc modu, ros2 run) + fake_mavros → grafik CSV satırları doğru (hız 0.845, sp 1.000, heading rad, thrust %31.2/28.6 rc/out'tan). fake_mavros_publisher'a velocity_body + IMU eklendi (masa provası tüm sütunları doldurur; yön_setpoint yalnız mission_manager koşarken dolar — normal).

**2026-07-14 (3) — Video hazırlık denetimi (son_kod, tamamlandı):** colcon build + install source ile **268 test ✓** (launch-config testi ilk kez gerçek koştu). Ekran-2 zinciri uçtan uca doğrulandı: sahte 120 s görev CSV'si → run_ekran2.py → PNG, üç sinyal şartnameye uygun. İki fix: (a) run_ekran2 `--out` dizin verilince matplotlib sessizce `<dizin>.png`'ye yazıyordu → dosya artık dizinin İÇİNE üretiliyor; (b) ekran2 thrust ekseni "(N)" → birimsiz "kuvvet isteği" (fc modunda değerler %, N yanıltıcıydı). ffmpeg kuruldu (14.07): test_ekran2 10/10 ✓; gerçek MP4 render doğrulandı (h264 800×900, süre=CSV süresi birebir, zaman imleci senkron) — montaj zinciri HAZIR.

**GitHub (2026-07-14):** son_kod → **https://github.com/yahyaseha/girdap-kaptan-video** (PRIVATE ✓) + takım reposu **IDA-TEAMM/IDA_GIT**'e `son_kod/` üst klasörü olarak push edildi (commit 4207969; mevcut dosyalara dokunulmadı, doğrulandı). ⚠ **Masaüstü IDA_GIT-main klasörü BAYAT** (11 Tem zip): repodaki 13 Tem doğrulanmış işleri (Livox IP 192.168.117.100/port 56301, imu_timeout 1.5, gps_imu MAVROS köprüsü) İÇERMEZ — oraya asla push edilmemeli, güncel iş için repo klonlanmalı. gh CLI `yahyaseha` girişli. Collaborator davetleri: `gh api -X PUT repos/yahyaseha/girdap-kaptan-video/collaborators/<kullanıcı>`.

**Bekleyen (yapılMAdı):** Jetson'a deploy + servis kurulumu (3 komut, scripts/girdap-karar.service içinde yazılı) · SERVO1/3 saha teyidi · FC'ye WP_SPEED parametre yazımı ([[ida-video-auto]]) · collaborator davetleri (write, 14.07): EyupEker1 + snnazz — kabul bekleniyor.

**Not (2026-07-14):** WP_SPEED=1.0 SABİT DEĞİL — Yahya duruma göre ayarlayacak (FC yetkisi kendisinde, [[user-role]]). WP_SPEED değişirse `ground_speed_publisher.py` sabiti + `hardware.yaml fc_cruise_setpoint_mps` de AYNI değere güncellenmeli (3 nokta senkron kuralı).

İlgili: [[ida-decision-repo-review]] · [[ida-video-auto]]
