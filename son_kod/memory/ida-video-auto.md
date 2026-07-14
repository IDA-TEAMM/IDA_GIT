---
name: ida-video-auto
description: "Otonomi videosu AUTO modu kararı (2026-07-13) — kod değişiklikleri, QGC/FC parametre planı, çekim kuralları"
metadata: 
  node_type: memory
  type: project
  originSessionId: 533f30e2-8411-45dc-98ba-9201047e3495
---

**Otonomi videosu (son teslim 21.07.2026 17:00, ELEME KAPISI) AUTO modla çekilecek** — GUIDED/MPPI saha testleri videodan sonraya ertelendi (karar 2026-07-13). Görevi FC'nin AUTO'su sürer; **Jetson pasif kayıtçı olarak açık kalır** (Ekran-2 grafik CSV + Dosya-2/3 üretimi için — kontrole karışmaz).

**Kod değişiklikleri (Downloads/"girdap-decision-main (2)" kopyasında yapıldı, Jetson'a taşınacak — 7 dosya):**
- B1 fix: `bridge.auto_guided: false` (hardware.yaml) — true kalırsa köprü görev sırasında modu GUIDED'a zorlar → FC AUTO'dan atılır (mod savaşı).
- B2 fix: telemetry_node'a `setpoint_source: "fc"` — AUTO'da MPPI thrust'ı sahte veri olurdu; thrust artık `/mavros/rc/out` PWM'inden (% normalize, `pwm_to_thrust_pct` csv_logger'da, pwm=0→None), hız_setpoint görev-aktifken `fc_cruise_setpoint_mps` (=WP_SPEED=1.0). fc modunda MPPI kanallarına hiç abone olunmaz; mavros_msgs lazy import.
- `fsm.start_on_mode: "AUTO"` — AUTO'ya geçiş (QGC veya RC) hem FC görevini hem FSM'i başlatır.
- Varsayılanlar "girdap"/GUIDED = yarışma davranışı değişmedi. Testler: çekirdek 15/15, node 3/3 (laptopta gerçek rclpy+mavros_msgs ile koşuldu).

**FC/QGC durumu (2026-07-13):** RC kalibrasyonu YAPILDI; mod switch = **kanal 6, RTL/MANUEL/AUTO** (⚠ RC'de AUTO var — OLAY riskine karşı disiplin: FC görev hafızası temizken güç ver, boot'ta switch MANUEL, BRD_SAFETY_DEFLT=1 GERİ YAZILDI ✓). Telemetri = MicoAir modülü ([[ida-e32-lora]]); frekans teyidi bekliyor (2.4 GHz ise md 4.1 yasak). QGC laptopta kurulu (AppImage + dialout ✓, tlog'lar akıyor).

**Yazılacak FC parametreleri (kararlaştırıldı):** `INITIAL_MODE=4` (HOLD — boot'ta güvenli, AUTO kenar tetiği korunur) · `FS_THR_ENABLE=1` + `FS_THR_VALUE≈RC3min−40` + `FS_ACTION=2` (Hold) · `FS_GCS_ENABLE=0` · `WP_SPEED=1.0` (fc_cruise_setpoint_mps ile AYNI; eski CRUISE_SPEED 5.0 düşürülmeli) · `WP_RADIUS=2` m. Bekleyen: SERVO çıkışlarında 73/74 hangi kanalda teyidi (`fc_thrust_left_ch:1/right_ch:3` varsayımı).

**Çekim kuralları:** dikdörtgen ~**20×30 m** (çevre ~100 m @1 m/s ≈ 2 dk, 2-5 dk penceresine uyar). **Sıra kuralı (C6): GPS fix → görev UPLOAD + doğrula → ARM (MANUEL'de) → AUTO = başla.** Görev başladıktan sonra plan yükleme YASAK (şartname 5.5.2.2); suya değen araçta görev değişmez. Her oturum başı/sonu QGC Plan → Remove All.

**⚠ Senkron uyarısı (2026-07-14):** Downloads/girdap-video-main.zip (13 Tem 20:12) fix'lerin HİÇBİRİNİ içermiyor (eski telemetry_node, `auto_guided: true`) — fix'li tek kopya "girdap-decision-main (2)". Ayrıca yeni laptop canlı-izleme ikilisi (Downloads: ground_speed_publisher.py + plotjuggler_girdap.xml, PlotJuggler için pasif hız/setpoint yayını) setpoint'i **5.0** kullanıyor; kararlaştırılan WP_SPEED=1.0'a düşürülmeli.

İlgili: [[ida-project]] · [[ida-software-status]]
