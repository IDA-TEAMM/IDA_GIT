---
name: grafik-formati-ekran2
description: "Video/teslim grafikleri HEP şartname md 3.3.1.1 Ekran-2 formatında üretilir (make_figure), genel dashboard değil"
metadata:
  node_type: memory
  type: feedback
---

Bu proje için üretilen tüm video/teslim grafikleri **şartname md 3.3.1.1 Ekran-2**
formatında olmalı: **3 panel, şu sırayla** — (2a) gerçek hız + hız isteği
(setpoint), (2b) heading/yaw AÇISI + yön isteği (setpoint), (2c) thrusterlardan
kuvvet isteği (sol/sağ). Kaynak `grafik.csv` (GRAPH_CSV_HEADER, 10 Hz).

**Üretim yolu (elle çizim YOK):** gerçek `prototype.telemetry.csv_logger.TelemetryCsvLogger`
→ `grafik.csv` → gerçek `prototype.viz.ekran2.make_figure` / `save_png` /
`save_mp4` (veya `scripts/run_ekran2.py --csv <grafik.csv> [--mp4]`). Video montaj
aracının tam kendisi. Örnek: IDA_GIT `jetsongeneltest1_grafLog/ekran2_sartname.png`.

**Why:** Kullanıcı 2026-07-17'de genel mühendislik teşhis grafiklerini (nokta
bulutu/histogram) reddetti — "şartnameye uymuyor". Tek kabul edilen grafik teslim
formatı Ekran-2. Genel matplotlib dashboard'lar teslim GRAFİĞİ olarak sunulmaz
(en fazla açıkça etiketli "mühendislik teşhisi" eki).

**How to apply:** "grafik ver/oluştur" denince önce Ekran-2 (`make_figure`) üret.
Dosya-2 (`telemetri.csv`, CSV_HEADER, 2 Hz md 4.2) ve Dosya-3 (local_map PNG
serisi) de şartname teslim formatları. Loglar `~/girdap_logs/kayit/<N>/` altında
ARM-başına numaralı klasörde, her satır fsync'li. İlgili: [[fm9-telem2-cozuldu]].
