---
name: jetson-yuk-kod-sadeligi
description: "GİRDAP: her kod eklemesinde Jetson Orin Nano Super işlem bütçesini + kod sadeliğini gözet — kullanılmayacak fazla kod ekleme"
metadata:
  node_type: memory
  type: feedback
  originSessionId: 1dcfa9b0-c5b6-44e7-9b5f-11707b5ec179
---

Eyüp (2026-07-10): kod yazarken **Jetson Orin Nano Super'ın kaldırıp kaldıramayacağını** ve **kod fazlalığını** sürekli gözet. "İşimize yaramayacak, kullanmayacağımız şeyler olmasın." [[sartname-once-kural]] (şartnameye bağla) ile birlikte uygulanır: önce şartname gerekçesi, sonra "Orin bunu kaldırır mı + gereksiz mi".

**Why:** Sahada görev bilgisayarı **Jetson Orin Nano Super 8GB**. Bitmiş+test edilmiş yığına (11.5k satır) her ekleme runtime maliyeti ve bakım yükü getirir; spekülatif kod hem CPU/RAM yer hem karışıklık yaratır.

**Yükün gerçek dağılımı (2026-07-10 doğrulandı):**
- **YOLO 416×416 → OAK-D VPU'da (Myriad X), Jetson'da DEĞİL** → Orin GPU'yu yormaz. Bkz. [[girdap-ida-proje-durumu]], [[yolo-model-durumu]].
- **MPPI (K=1000/T=50) → Orin CPU NumPy ~100 ms/iter = ASIL DARBOĞAZ.** CUDA portu bekliyor (henüz yok). F4.2'de control_rate 20→10 Hz düşürüldü.
- **LiDAR kümeleme → Orin CPU, saf Python union-find, voxel yok** → 20k nokta yüzlerce ms, 10 Hz tutmaz (F5.3, T1 işi).
- iSAM2/RRT* video modunda **kapalı** (`use_isam2/rrt=false`) → video'da yük yok.
- **Kilit içgörü:** "Super" boost esas **GPU/TOPS** (NN çıkarımı) için; ama NN zaten VPU'da, MPPI CPU'da → Super'ın fazladan TOPS'u **CUDA portu gelene kadar boşta**. Darboğaz CPU. Sahada gerçek ms **ölçülmeli** (bekleyen D3, [[bekleyen-girdiler-isaret]]).

**How to apply:**
1. Her eklemeden önce: (a) hangi şartname maddesi? (b) Orin'de runtime maliyeti ne — sürekli mi, tek seferlik mi, hangi çekirdek (CPU/GPU/VPU)? (c) gerçekten kullanılacak mı yoksa spekülatif mi?
2. Olay-güdümlü/tek seferlik kod (örn. T0-f `fc_items_to_waypoints` görev gelince 1 kez) ≈ sıfır yük — sorun değil. Sürekli döngüde koşan ağır iş (MPPI/LiDAR) = dikkat.
3. Spekülatif satır ekleme; eklediysen işaretle ve çıkarmayı öner (örn. T0-f'te `MAV_CMD_NAV_SPLINE_WAYPOINT=82` — 4-nokta dikdörtgen düz NAV_WAYPOINT kullanır, spline gelmez; Eyüp'e soruldu, katı minimalizm istenirse çıkar).
4. Bitmiş+test edilmiş çekirdeği yeniden yazma; hedefli düzeltme + minimum ekleme.
