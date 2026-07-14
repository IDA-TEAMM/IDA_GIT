---
name: tekne-cift-motor
description: "İDA tahrik = ÇİFT motor diferansiyel (katamaran, dümen yok) — Eyüp 2026-07-14 teyit etti; kod ve gerçek tekne uyumlu"
metadata: 
  node_type: memory
  type: project
  originSessionId: 9ee3d9d9-28cd-4ab8-b918-cfa44962dee9
---

**Eyüp teyidi (2026-07-14): tekne gerçekten ÇİFT motorlu — kodun varsayımı DOĞRU.**

- Kontrol vektörü `[T_left, T_right]` (N); dönüş yalnız sağ-sol fark momentiyle (`catamaran.py`, dümen modeli yok).
- ⚠️ **Eyüp (2026-07-14): arkadaşın SAYILARI DA UYDURMA — hiçbiri kontrol edilmedi.** `dynamics.yaml`'daki HER değer güvenilmez: `thruster_spacing: 0.596` ("Mitras/SolidWorks raporu" diyor ama rapor görülmedi, iddia doğrulanamaz), `max_thrust: 30 N`, RPM→N tablosu, `mass: 30`, `inertia_z: 5.0`, `Xu/Yv/Nr` — TAMAMI ölçüm/kalibrasyonla değiştirilecek. Doğru olan yalnız MODEL YAPISI (çift motor diferansiyel).
- Çıkışlar: `/girdap/control/thrust` = [T_l, T_r] (Ekran-2c kaynağı) + `/mavros/setpoint_velocity/cmd_vel_unstamped` (toplam→ileri hız, fark→yaw rate).
- ⚠️ Doğrulanmamış kalan: FC tarafında skid-steer kurulum (SERVO çıkışları ThrottleLeft/ThrottleRight, sağ/sol kanal eşleşmesi) — FC ekibinin parametresi, M6 öncesi pervanesiz kontrol edilebilir. İlgili: [[donanim-test-plani]], [[girdap-decision-entegrasyon]].
