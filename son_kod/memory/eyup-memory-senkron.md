---
name: eyup-memory-senkron
description: "Eyüp'ün Claude hafızası (github.com/EyupEker1/memory) okundu 2026-07-14 — kritik: B1/B2'yi Eyüp de yazdı ve JETSON'DA ONUN SÜRÜMÜ KURULU; F-M.6 bizde yok; repo çatallanması"
metadata: 
  node_type: memory
  type: project
  originSessionId: 97fbf033-cf13-4546-afbf-c6b146529521
---

**Eyüp'ün Claude hafızası okundu (2026-07-14, github.com/EyupEker1/memory).** Kaynak gerçekler:

**🔴 EN KRİTİK — repo çatallanması:** Eyüp kararı (14.07): video işi YALNIZ **github.com/EyupEker1/girdap-video** (karar/ = 15dc238 + F-M.3 `98b5386` + B1/B2 `aedf6ae`). **JETSON'DA KURULU OLAN BU** (symlink install: kaynak klon = çalışan kod; .py düzeltmesi → servis restart yeter, rebuild gerekmez). Bizim son_kod paralel/ayrık bir hat — Jetson'da DEĞİL. Eyüp'ün notu: "⚠️ Yahya paralel push atarsa çakışır — kodu biz yazdık, Yahya gözden geçirsin."

**B1/B2'yi Eyüp BAĞIMSIZ yazdı** (bizim decision(2) kodumuz ona hiç ulaşmamış; KOD_DEGISIKLIKLERI.txt özetinden yola çıkmış): B1 = yalnız config (start_on_mode AUTO + auto_guided false; mode_name GUIDED BİLEREK kaldı → AUTO'da planning cmd_vel basmaz). B2 = setpoint_source fc, thrust rc/out ±%100, PWM=0→BOŞ, `run_ekran2 --thrust-birim %`. Suite tabanları: 267/2 (F-M.3) → **282/2** (B1/B2+F-M.6).

**🔴 F-M.6 — BİZDE YOK, video-kritik:** FC taze bağlantıda ~1 Hz yayınlıyor (ölçüldü: 1.000 Hz) → Ekran-2 basamaklı + fusion pose_timeout odom'u kesiyor. Fix: köprü bağlantı yükselen kenarında `/mavros/set_stream_rate` STREAM_ALL **10 Hz** (oturumluk, EEPROM'a yazmaz, alt sınır 5 Hz, yalnız USB kanalı). **Gerçek FC'de canlı doğrulandı** (ttyACM1 + ROS_DOMAIN_ID=77 izole oturum numarası; 9.99 Hz, yük ~%8/50°C). son_kod'a taşınmalı.

**✅ Bizim bekleyenlerden ÇÖZÜLENLER:** SERVO eşleşmesi TEYİTLİ (1=Sol/73, 3=Sağ/74) · FC kaçak-motor OLAYI KAPANDI (kök neden: ARMING_RUDDER=2 × CH2 boşta max = kendiliğinden ARM + CH6 1577→MODE4=AUTO + ARMING_CHECK=0; görev silindi; FC ekibi kararıyla paramlar KALDI → **OPS-1 kuralı: görev yüklenen HER oturum sonu mission clear**) · USB-C tamir edildi, fcu_url=ttyACM0 · Jetson sıfırdan kuruldu + girdap-karar.service kurulu + Livox IP NM `livox` profilinde kalıcı · Wi-Fi kapalı ✓ · MicoAir LR868 868 MHz (çift teyit).

**🟠 Yeni riskler/bilgiler:** Jetson'da **BLUETOOTH AÇIK** (2.4 GHz = md 4.1 ihlali; videodan sonra `rfkill block bluetooth`) · 🔴 SR0 boot'ta 1 Hz sorunu AÇIK (F-M.6 oturumluk çözüyor; boot provası bekliyor) · dynamics.yaml SAYILARI TAMAMEN UYDURMA (ölçüm formu bekliyor; static TF'ler 0,0,0 rastgele) · YOLO sınıf sırası TERS (0=Engel,1=Kenar; kod tersi) · **yarışma tarihi Ağustos-Eylül 2026 DSB** (30 Eylül-4 Ekim = festival, yarışma değil!) · TEST İZOLASYONU: servis çalışırken testler `ROS_DOMAIN_ID=77`'de koşulmalı (42'de canlı yayın test verisini ezer) · fake_mavros_publisher Jetson'a bilerek deploy edilmedi/edilmeyecek · Eyüp'te Dosya-1 kamera kaydedici de var (`2282241`).

**GÜNCELLEME (14.07 akşam, 2. okuma — Eyüp AUTO denetimi 41d205f, taban 298/2):** B1/B2+F-M.6 Jetson'da CANLIYA ALINDI (17:06 restart, teyitli); F-V.6/7/8+F-P.1 fix'leri 2. restart bekliyor. **Yeni bulgular (son_kod'da da YOK, taşınacak):**
- 🔴 **F-V.6 video-katil:** operatör önce AUTO sonra ARM ederse (QGC Start Mission akışı!) kenar tetik kaçar → FSM BEKLEMEDE → Ekran-2 setpoint'leri BOŞ = video sessizce başarısız. Bizim kenar-tetik tasarımımız AYNI hataya sahip; bizim runbook sırası (ARM→AUTO) tetiği doğru üretir ama tek güvence prosedür olmamalı. Fix: `start_on_arm_in_mode` (video true/yarışma false).
- F-V.7: AUTO'da dwell sahte (dwell_time_s: 0.0 video) + waypoint üstünde atan2 savrulması (yaw_setpoint_min_dist_m: 0.5).
- F-V.8: görev bitişi `/mavros/mission/reached` ile ileri senkron (QoS: bilerek volatile abone!). arrival_radius↔WP_RADIUS teyidi yine de sorulacak.
- F-P.1: planning bayat odom'la kör sürüş → odom_timeout_s 1.0.
- mission_source varsayılanı fc.
- FC ekibinden (=Yahya) İKİ SAYI isteniyor: **WP_SPEED** (↔fc_cruise 1.0) ve **WP_RADIUS** (↔arrival_radius 2.0).
- Saha notu: velocity_body yalnız GPS fix'le akıyor — kapalı alanda hız sütunu boş (bizim canlı testte de aynıydı, beklenen).

**Yapılacak (Yahya kararı bekliyor):** son_kod ↔ EyupEker1/girdap-video mutabakatı — F-M.6 + F-V.6/7/8 + F-P.1'in son_kod'a taşınması + iki B1/B2 karşılaştırmalı gözden geçirme + TEK kanonik repo kararı (Jetson gerçeği: Eyüp'ün reposu). kaptan_memory.md Eyüp'ün memory reposuna yazıldı (push Yahya'nın terminalinden).

İlgili: [[kod-duzeltme]] · [[ida-video-auto]] · [[ida-sartname-2026]]
