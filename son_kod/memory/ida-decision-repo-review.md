---
name: ida-decision-repo-review
description: "girdap-decision deposu tam inceleme (14 Tem 2026) — mimari, node'lar, test sağlığı, video günü riskleri; kanonik kopya ~/Desktop/video-girdap"
metadata: 
  node_type: memory
  type: project
  originSessionId: 97fbf033-cf13-4546-afbf-c6b146529521
---

girdap-decision deposunun tam incelemesi (2026-07-14). **GÜNCEL KANONİK: `~/Desktop/son_kod`** — video reposu + B1/B2 fix'lerinin 3-yönlü birleşimi ([[kod-duzeltme]]) + canlı-izleme üçlüsü + `memory/` klasöründe hafıza kopyaları (14 Tem anlık görüntüsü, canlı hafıza değil). `~/Desktop/video-girdap` altındaki kopyalar ve `girdap-decision-main` artık eski. Yanında ground_speed_publisher.py + plotjuggler_girdap.xml + fake_mavros_publisher.py canlı-izleme/test üçlüsü. Jetson otomatik başlatma: `karar/scripts/girdap-karar.service` (kurulum komutları dosya içinde; ROS_DOMAIN_ID=42).

**Mimari (sağlam):** Tüm karar mantığı ROS-bağımsız `prototype/` paketinde; `ros2_ws/src/girdap_decision/` node'ları ince ROS sargısı. Node'lar: fsm_node (görev FSM, kenar-tetikli mod başlatma, KILL servisi), mavros_bridge_node (mod otoritesi + arm/disarm servisleri + 5s heartbeat→KILL + beklenmedik disarm→KILL; kendiliğinden arm ETMEZ), mission_manager_node (mission_source file/fc, haversine→ENU, current_target 5 Hz), planning_node (RRT*+MPPI 20 Hz → cmd_vel + thrust; KILL'de sıfır thrust otoritesi planning'de), fusion_node (use_isam2 çift mod: GTSAM ↔ EKF pass-through, GTSAM lazy import), telemetry_node (Dosya-2 2 Hz + grafik CSV 10 Hz, setpoint_source girdap/fc), perception 3'lü (lidar cluster, kamera HSV+CLAHE, bearing füzyonu), local_map_node, mock_sensors.

**Veri çıkışı:** `gcs_url: ""` — görev sırasında laptopa CANLI ROS verisi YOK (şartname 4.1 tek yönlü). Ekran-2 akışı: telemetry_node → Jetson'da `~/girdap_logs/grafik/grafik_<UTC>.csv` → görev SONRASI `scripts/run_ekran2.py --mp4` → montaj (video_gunu_runbook.md §6). Sudayken tek canlı göz QGC/telemetri radyosu. PlotJuggler ikilisi yalnız masa/SITL provası için.

**Test sağlığı (14 Tem, laptop):** `pytest prototype/tests/` → **217 geçti, 14 gerekçeli skip** (node testleri ros2_ws build+source ister; ffmpeg/cupy/gtsam opsiyonel). CI: ROS'suz çekirdek, exit-5 yanlış-yeşil koruması.

**Video günü açık uçları:**
- `video_mission.yaml` koordinatları 0.0 (yalnız prova; çekimde `mission_source:=fc` ZORUNLU — QGC'den upload).
- hardware.yaml SERVO1=73/SERVO3=74 yorumunda "FC ekibi teyit" yazıyor ama [[ida-video-auto]] teyidi bekliyor kaydetmişti — sahada doğrula.
- FC'ye WP_SPEED=1.0 vb. parametre yazımı hâlâ yapılacaklar listesinde.
- fsm_node docstring'i RFD868 diyor, gerçek telemetri MicoAir ([[ida-e32-lora]]) — doc drift, zararsız.
- Downloads/girdap-video-main.zip ESKİ (fix'siz); güncel referans Desktop kopyası.

İlgili: [[ida-video-auto]] · [[ida-software-status]] · [[ida-project]]
