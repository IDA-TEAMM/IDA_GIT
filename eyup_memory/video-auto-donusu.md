---
name: video-auto-donusu
description: "AUTO+FC video dönüşü (B1/B2) — Yahya önerdi, KODU BİZ YAZDIK (2026-07-14, TDD, girdap-video aedf6ae); Yahya'nın kodu hiç gelmedi. Ayrıca F-M.6 (FC 1 Hz akış) bulundu+düzeltildi"
metadata:
  node_type: memory
  type: project
  originSessionId: 186fe6aa-acca-43d9-85c1-8354c18eb5eb
---

Yahya (karar yazılımcısı) video yaklaşımını 2026-07-14'te GUIDED+MPPI → **AUTO+FC**'ye çevirdi (kaynak: `~/Downloads/KOD_DEGISIKLIKLERI.txt` özeti). **Asıl kodu HİÇ GELMEDİ** → Eyüp kararıyla **B1/B2'yi BİZ yazdık** (2026-07-14, TDD). İlgili: [[video-repo-tek-kaynak]], [[donanim-test-plani]], [[sartname-ida-2026]].

## ✅ UYGULANDI (2026-07-14) — runtime `91a5cf0`+10Hz, girdap-video **`aedf6ae`** (push'lu)

- **B1 (mod savaşı):** `hardware.yaml` → `fsm.start_on_mode: "AUTO"` + `bridge.auto_guided: false`. Görevi FC AUTO'da kendi uçurur. **`mode_name` GUIDED KALDI** (bilinçli): planning geçidi GUIDED beklediği için AUTO'da cmd_vel YAYINLANMAZ → MPPI ile FC kavga etmez. (`start_on_mode`/`auto_guided` param'ları kodda ZATEN vardı; yalnız config işiydi.)
- **B2 (Ekran-2 dürüstlüğü):** `telemetry_node` → `setpoint_source` param'ı: `"girdap"` (YARIŞMA VARSAYILANI, değişmedi: MPPI thrust N + cmd_vel setpoint) | `"fc"` (video: thrust `/mavros/rc/out` PWM'inden ±%100, `hiz_setpoint` = FC WP_SPEED = `fc_cruise_setpoint_mps`). fc modunda cmd_vel + `/girdap/control/thrust` abonelikleri HİÇ kurulmaz; RCOut lazy import (CI hijyeni). **Bizim eklediğimiz:** PWM=0 (pasif kanal) → sahte -%100 yerine BOŞ hücre; `run_ekran2.py --thrust-birim %` (fc'de eksen "N" değil "%" — yoksa grafik yalan söyler).
- **F-M.6 (BİZ BULDUK, 🔴 video-kritik):** FC taze bağlantıda ~1 Hz yayınlıyor, yığın hiç akış hızı istemiyordu. Üç katman: Ekran-2 basamaklı (md 3.3.1.1 "net değilse BAŞARISIZ") + fusion `pose_timeout_s=1.0` bekçisi 1 Hz'i bayat sayıp **odom'u KESİYOR** + planning pozun YAŞINA BAKMADAN 10 Hz MPPI koşuyor (bayat pozla salınım). Düzeltme: `MavrosBridge.should_request_stream_rate()` (bağlantı yükselen kenarı) + `mavros_bridge_node` → `/mavros/set_stream_rate` STREAM_ALL **10 Hz** (Eyüp kararı; oturumluk istek, FC SR0_* EEPROM'una YAZMAZ; yeniden bağlanışta tekrar; `stream_rate_hz: 0` → kapalı). **ALT SINIR 5 Hz** (altında fusion odom'u keser). Yalnız USB/SERIAL0 kanalını etkiler — 868 MHz telemetri ayrı port (SR1/SR2), QGC hattına yük BİNMEZ (arkadaşın "yük biner/ms farkı" itirazının cevabı).
- **Suite YENİ TABAN: 282 passed / 2 skipped** (eski 267/2, +15 test).

## ✅ F-M.6 CANLI DOĞRULANDI (2026-07-14 akşam, servise DOKUNMADAN)
Pixhawk 6C'nin **ikinci USB kanalı `/dev/ttyACM1` boştaydı** → izole `ROS_DOMAIN_ID=77`'de taze mavros oturumu (servisin ACM0 kanalı hiç etkilenmedi; sudo GEREKMEDİ — bu numara tekrar kullanılabilir):
- **Fix'siz taze bağlantı = imu 1.000 Hz, rc/out 1.000 Hz, GPS 0.999 Hz** → "boot'ta 1 Hz" ÖLÇÜMLE kanıtlandı (varsayım değildi).
- **Yeni köprüyle:** log "FC akış hızı isteniyor: 10 Hz (STREAM_ALL)" → üç akış da **9.99 Hz**. İlk state'te servis hazır değil → istek ertelendi, ~1 s sonra gitti (retry yolu gerçek FCU'da çalıştı).
- **Yük:** mavros ~%50 tek çekirdek (6 çekirdekli Orin'de ~%8 toplam), 50-51°C, loadavg 3.4 → 10 Hz'te yük/termal sorunu YOK (arkadaşın "35 Hz'de yük biner" itirazının ölçülmüş cevabı).
- `/mavros/local_position/pose` kapalı mekânda HİÇ yayınlanmıyor (GPS fix yok → EKF pozu yok) — beklenen; açık alanda ölçülecek.

## 🔬 AUTO DENETİMİ (2026-07-14 akşam) — AUTO dönüşünün AÇTIĞI 3 kusur bulundu+düzeltildi
AUTO'ya geçmek yeni yollar açtı; kod satır satır denetlendi (girdap-video `bc4577e`, suite **291/2**):
- **F-V.6 (🔴 VİDEO-KATİL, düzeltildi):** başlatma tetiği KENAR şartlıydı → operatör **önce AUTO'ya alıp SONRA ARM ederse** (QGC "Start Mission" akışı; ArduRover AUTO'da arm olunca görevi başlatır) kenar hiç oluşmuyor → FSM BEKLEMEDE'de kalıyor → telemetry F-V.2 gereği setpoint sütunlarını BOŞ bırakıyor → **Ekran-2'nin ZORUNLU hız/yön setpoint eğrileri boş çıkıyordu** (md 3.3.1.1). Video tek çekim → çekerken fark edilmez! Düzeltme: `fsm_node.start_on_arm_in_mode` (video true / **yarışma false** — GUIDED'dayken arm MPPI'yi kendiliğinden başlatmasın).
- **F-V.7 (🟠, düzeltildi):** (a) `dwell_time_s: 2.0` AUTO'da SAHTE bekleme — FC durmaz, bizim hedef index'imiz 2 sn geride kalır → yon_setpoint arkadaki waypoint'i gösterir → `hardware.yaml mission.dwell_time_s: 0.0` (yarışmada 2.0); (b) waypoint üstünden geçerken ofset ~0 → `atan2` açıyı savuruyor (~180°) → `telemetry.yaw_setpoint_min_dist_m: 0.5`.
- **F-P.1 (🟠 yarışma, videoda etkisiz, düzeltildi):** planning odom'un YAŞINA bakmadan MPPI koşuyordu (fusion F8.2 odom'u kesse bile) → GPS/EKF kesilirse KÖR sürüş. `planning_node.odom_timeout_s: 1.0` → bayatsa thrust sıfır.
- **mission_source varsayılanı `fc` yapıldı** (servis zaten `:=fc` geçiyordu; elle launch'ta sessizce araç-üstü YAML'a düşme tuzağı = md 3.3.1(2) ihlali).

## 🔴 Kalan / dikkat
- ⏳ **DEPLOY:** servis hâlâ ESKİ kodu bellekte çalıştırıyor → `sudo systemctl restart girdap-karar` (sudo = Eyüp; `!` ile de şifre soramıyor, gerçek terminal lazım). Restart sonrası AUTO config yüklenecek → pervanesiz + oturum sonu OPS-1.
- **`fc_cruise_setpoint_mps` (1.0) ↔ FC `WP_SPEED` SENKRON** tutulmalı — çekimden önce teyit, yoksa Ekran-2 setpoint çizgisi yalan.
- `fake_mavros_publisher.py` (Yahya'nın yan dosyası) Jetson'a **deploy EDİLMEDİ** ve edilmeyecek.
- **FC ekibinden İKİ SAYI:** `WP_SPEED` (↔ `fc_cruise_setpoint_mps` 1.0) ve `WP_RADIUS` (↔ `arrival_radius_m` 2.0). Farklıysa Ekran-2 yalan söyler / hedef index'imiz FC'den ayrışır.
- **Saha kontrolü (açık alan, ilk iş):** GPS fix sonrası `/mavros/local_position/velocity_body` akıyor mu — Ekran-2'nin "gerçek hız" eğrisi ona bağlı, kapalı alanda CSV'de `hiz` sütunu TAMAMEN BOŞ çıktı (EKF pozu yok, beklenen).
- 💡 T1 fikri: `yon_setpoint` için daha dürüst kaynak `/mavros/nav_controller_output/output` (FC'nin KENDİ nav_bearing'i; topic VAR, HOLD'da yayın yok). ⚠️ Konvansiyon dönüşümü ŞART (pusula derece/kuzeyden saat yönü → ENU radyan/doğudan CCW) — suda doğrulanmadan KOYMA, grafiği aynalar.
- ⚠️ **Yahya paralel push atarsa çakışır** — "kodu biz yazdık, Yahya gözden geçirsin" denmeli.
- 🧪 **TEST İZOLASYON KURALI (yeni):** `girdap-karar.service` çalışırken node testleri AYNI `ROS_DOMAIN_ID=42`'de koşarsa canlı yayın testin verisini EZER (thrust 0.00 sahte FAIL'i böyle çıktı). Testleri izole domain'de koş: `ROS_DOMAIN_ID=77 pytest`.
