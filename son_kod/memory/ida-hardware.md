---
name: ida-hardware
description: GİRDAP İDA + İHA donanım/komponent listesi ve listedeki şüpheli/hatalı girdiler
metadata: 
  node_type: memory
  type: reference
  originSessionId: b947f7ad-b0f6-4a34-b949-cbbe03fd8065
---

Komponent listesi: `~/Desktop/komponent listesi güncel.xlsx` (3 sayfa). Kullanıcı listede "bazıları hatalı" olduğunu belirtti — alımdan önce doğrulanmalı.

**Sayfa 1 — İDA ana elektronik (~262.762 TL):** Jetson Orin Nano 8GB Super, Pixhawk 6C + PM07, 4× Mucif 2838 Mitras thruster, 2× Daly Smart BMS, 4× Bidirectional ESC 40A (Motorobit), MicroAir LR868-F telemetri, Holybro H-RTK F9P (Rover Lite + Base + Tripod), XL4016 regülatör, OAK-D Lite kamera, Livox MID-360 LiDAR, Aspilsan 18650 ×28 (4S7P), XT90, RC alıcı. +10.000 TL güvenlik.

**Sayfa 2 — İHA elektronik (~69.906 TL):** Raspberry Pi 5 4GB, Pixhawk 6C Mini, RPi HQ Kamera (IMX477), H-RTK F9P Helical, MicroAir LR868-F, 4× RS2205 2300KV motor, Racestar 30A ESC, 4S 5000mAh LiPo.

**Sayfa 3 — Malzeme/üretim (~60.589 TL):** karbon profil, ARC 152 epoksi, ASA filament (Esun + Bambu Lab ASA-CF), cam elyaf, 3K karbon boru, gaz maskesi/eldiven/gözlük vb.

**Şüpheli/hatalı görünen girdiler (doğrulanmalı):**
- Sayfa1 GPS: "H-RTK F9P Rover Lite" satırlarının bazısında fiyat boş; Base 89.900 TL tek kalemde — Rover/Base/Tripod aynı sete mi ait belirsiz.
- "2838 Thuruster" yazım hatası (Thruster); link Mucif Mitras'a gidiyor.
- Sayfa1'de bazı kalemler eurolu (OAK-D 480€, Livox 699€) — TL toplamı bunları içermiyor olabilir.
- SSD satırı fiyatsız (128GB), Kargo 2500 TL.

Not: Kritik Tasarım Raporu (CDR) ile çapraz kontrol önerilir — CDR Aspilsan INR18650A28 4S7P, Mucif Mitras, OAK-D Lite, Livox MID-360, H-RTK F9P diyor; bunlar listeyle uyumlu.

İlgili: [[ida-project]] · [[ida-software-status]]
