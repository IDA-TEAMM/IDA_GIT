---
name: ida-software-status
description: GİRDAP İDA yazılım mimarisi katmanları ve her birinin mevcut tamamlanma durumu (boşluk analizi)
metadata: 
  node_type: memory
  type: project
  originSessionId: b947f7ad-b0f6-4a34-b949-cbbe03fd8065
---

GİRDAP İDA yazılımı 5 katmanlı: algı → konum/füzyon → planlama → kontrol → görev FSM. ROS 2 Humble + Cyclone DDS. Jetson Orin Nano 8GB ana işlemci, Pixhawk 6C (ArduRover) uçuş kontrol.

**Katman durumları (yazılım analiz raporu, 2026-06):**
- **Ortam & altyapı:** Devam ediyor — Ubuntu 24.04→22.04 migrasyonu sonrası kurulum sürüyor.
- **Simülasyon:** Devam ediyor — dalga fiziği + render crash açık. [[ida-sim-workspace]]
- **Sensör entegrasyonu:** Kısmen — 4 kritik topic eksik: `/sensor/camera/camera_info`, `/sensor/camera/depth_raw`, `/sensor/imu/mag`, `/sensor/gps/vel` (öncelik sırası: camera_info → mag → gps/vel → depth_raw).
- **Algı:** Kısmen — YOLOv11n çalışıyor; LiDAR+kamera füzyonu ve OAK-D VPU termal optimizasyonu (RGB-only inference, depth'i ROI'ye sınırla 416×416@15FPS) gerekli.
- **Planlama:** BAŞLANMADI — RRT* (global) + MPPI (lokal, K=1000 yörünge, T=50, GPU) node'ları yazılacak.
- **Kontrol:** BAŞLANMADI — Cascade PID + /cmd_vel→thruster + MAVROS/PX4 SITL + acil durdurma.
- **Görev FSM:** BAŞLANMADI — 3 parkur akışı tasarlandı, kodlanacak.

**En kritik darboğaz:** Planlama ve kontrol katmanları hiç başlamamış; üç parkur için de zorunlu. Kritik yol: simülasyon → planlama → kontrol → entegrasyon.

**Önerilen sıra:** ortam kur → sim fiziği stabilize → eksik topic'ler → algı zinciri → Parkur-1 uçtan uca → planlama+kontrol Parkur-1'de doğrula → Parkur-2/3 → failsafe → İHA iletişimi → saha testi. Toplam tahmin ~37–55 mühendis-günü.

Algoritma detayları: konum füzyonu iSAM2/GTSAM (libgtsam-dev kurulu), MPPI maliyet ağırlıkları parkura göre (engel w₂=10.0, kamikaze w=−50.0).

İlgili: [[ida-project]] · [[ida-sim-workspace]] · [[ida-hardware]]
