---
name: yeni-arkadas-repo-plani
description: "3. arkadaşın (ebk) girdap_ws.zip'i ALINDI → EyupEker1/girdap-logger (private, /home/eyup/girdap-logger); ilk denetim L1-L8 yazıldı; SONRAKİ İŞ: tüm yığının temelli revizyonu (kapsam Eyüp'le netleşecek)"
metadata: 
  node_type: memory
  type: project
  originSessionId: 29fc60b5-19c2-4682-bbc4-8892fb0f0740
---

Eyüp'ün kararı (2026-07-13): bir arkadaşın daha kodları alınacak, sonra "temelli"
(baştan sona) HEPSİ revize edilecek.

## ✅ İÇE AKTARMA YAPILDI (2026-07-13 akşam)

- Kaynak: `~/Downloads/girdap_ws.zip` (2026-07-13 15:35 inmiş; git geçmişi YOK — zip
  anlık görüntü, fork değil). Sahibi: **"ebk"** (package.xml maintainer; hangi arkadaş
  olduğu Eyüp'ten teyit edilecek).
- Repo: **github.com/EyupEker1/girdap-logger (PRIVATE, main)**, lokal
  **/home/eyup/girdap-logger**. 1. commit = zip'in saf içeriği (build/log/install hariç),
  2. commit = `docs/kod_denetimi.md` (L1-L8) + .gitignore. Git kimliği lokal Team GIRDAP.
- ⚠️ **AD ÇAKIŞMASI notu:** zip adı `girdap_ws` ama o ad ZATEN dolu — PC'de
  `/home/eyup/girdap_ws` = Eyüp'ün Mayıs Gazebo sim ws'i (girdap_sim+ros_gz, DOKUNULMADI,
  git'siz), Jetson'da `~/girdap_ws.eski` = bayat karar kopyası. O yüzden repo adı
  **girdap-logger** (içerikten).

## İçerik + ilk bulgular (ayrıntı: girdap-logger/docs/kod_denetimi.md)

236 satır: `girdap_logger` ROS2 paketi — MAVROS'tan Dosya-2 benzeri telemetri CSV
(`~/girdap_logs/telemetry_*.csv`) + `ros2 bag record -a` launch'ı + GPS-IMU sync_test +
Gazebo deniz dünyası (`worlds/girdap_deniz.sdf`) + PlotJuggler düzeni + gerçek koşudan
TF ağacı çıktısı (frames_2026-07-12: MAVROS map→map_ned, odom→odom_ned,
base_link→imu/camera/gps_link).
- 🔴 **L1:** setpoint mantığı **AUTO-mod varsayımlı** (speed=WP_SPEED sabiti 5.0 yalnız
  AUTO+armed; heading=nav_controller_output target_bearing) → bizim GUIDED+MPPI
  yığınında setpoint'ler 0/anlamsız — **videoda kullanılamaz**; AUTO-yedek senaryosu
  için anlamlı ([[sartname-ida-2026]] 'Alternatif (zayıf)' yolu).
- 🟠 **L2:** Dosya-2 çifte üretim (girdap-decision telemetry_node ile — o md 4.2
  header'ını testle dondurmuş durumda) → rol kararı gerek.
- 🟠 **L3:** rosbag `-a` göreli yol (F4.1/F15.1 deseni) + TÜM topic'ler = Jetson diski
  şişer ([[jetson-yuk-kod-sadeligi]]).
- 🟡 L4-L6: kayıt GPS callback'e kilitli (GPS düşerse kayıt durur) · header md 4.2
  birebir değil · message_filters package.xml'de beyansız (F2.1/V11 deseni 3. tekrar).
- ✅ **L7:** MAVROS QoS DOĞRU (sensor_data profili — tuzağa düşmemiş); sync_test bizim
  F7.1 stamp sorusuna hazır araç; TF çıktısı değerli gerçek-sistem gözlemi; PlotJuggler
  düzeni Ekran-2 provasında işe yarayabilir.
- 🟡 L8: Gazebo malzemesi (girdap_deniz.sdf + Eyüp'ün Mayıs girdap_ws'i) — video
  sonrası Gazebo işine başlangıç ([[gazebo-simulasyon-erteleme]]).
- **Rol önerisi (karar Eyüp'te):** EMEKLİ/yedek + araç kutusu (sync_test, PlotJuggler);
  karara kadar Jetson'a KURULMAZ.

## SONRAKİ İŞ: temelli revizyon
Tüm yığın (girdap-decision + girdap-ida-algi + IDA_GIT + girdap-logger) bütünlüklü
gözden geçirilecek — **kapsam Eyüp'le netleştirilecek** (repolar arası sözleşme
tutarlılığı? rol/çakışma matrisinin 4 repoya genişletilmesi? tek sistem dokümanı?).
Kurallar: [[sartname-once-kural]] · [[test-dogrulama-ilkesi]] · [[jetson-yuk-kod-sadeligi]].

## Öncelik
⚠️ **T0 video (21.07.2026) hâlâ her şeyin ÖNÜNDE** — bu iş T1; video işlerini (suda
prova, çekim, FC OLAY aksiyonları, M5-M8) bloke etmez. IDA_GIT Faz 19-20 kalanları da
açık ([[ida-git-gomulu-revizyon]]).
