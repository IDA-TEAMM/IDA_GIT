---
name: ida-perception-canli
description: Canlı Livox+OAK ile perception/füzyon — read_points fix + sync queue gecikme sorunu
metadata: 
  node_type: memory
  type: project
  originSessionId: a70a49cc-c538-46e5-8247-4e8e9aa9b1d1
---

İDA topic testinde perception zinciri canlı sensörle koşuldu: `perception_lidar_node` (/livox/lidar→/perception/obstacle_map), `perception_camera_node` (/oak/rgb/image_raw→/perception/buoys), `perception_fusion_node` (ikisi→/perception/classified_obstacles). İki gerçek-donanım sorunu ve çözümü:

1. **read_points_numpy → read_points (LiDAR node):** Gerçek Livox PointCloud2 alanları KARIŞIK tipli (x/y/z float32, tag/line uint8, timestamp float64). `sensor_msgs_py.point_cloud2.read_points_numpy` bulutun TÜM alanlarının aynı tipte olmasını `assert` eder → Livox'ta `AssertionError: All fields need to have the same datatype`. Node sentetik uniform-float32 bulutla test edildiği için ilk kez sahada patladı. Fix: `read_points` (yapısal) + `np.column_stack((s["x"],s["y"],s["z"]))`. perception_lidar_node.py `_on_cloud`. 15/15 test + flake8 yeşil.

2. **Füzyon sync gecikmesi (Option 2 uygulandı):** `ApproximateTimeSynchronizer` stamp'e göre eşler. Kapalı alanda ~20k yoğun nokta → clustering YAVAŞ (~1-3.3s/kare), `obstacle_map` stamp'i doğru ama GEÇ varıyor; `buoys` anlık → aynı-stamp'li kamera karesi varsayılan queue_size=10 (~0.6s) penceresinden düşünce sync hiç tutmuyor, classified_obstacles ÜRETİLMİYOR. Ham sensör stamp'leri aynı tabanda (fark ~27ms), yani saat sorunu YOK — tek sorun gecikme. Çözüm: `sync_queue_size` parametresi eklendi (perception_fusion_node + params.yaml + hardware.yaml + hardware.launch _FUSION_DEFAULTS), slop 0.1 kalır. Bench'te ölçülen gecikme 3.3s → queue=50 (~2.9s) KIL PAYI yetmedi, **queue=100 (~6s) ile füzyon üretmeye başladı** (7-12 engel, class_id=99 çünkü içeride duba yok → eşleşmeyen LiDAR güvenlik gereği atılmaz).

⚠ Kalıcı çözüm değil: gecikme buffer'ı aşarsa yine kırılır. Bu gecikme KAPALI-ALAN artefaktı — açık suda seyrek nokta → clustering ms'ler → default queue yeter. Gerçek kök-neden fix'i clustering hızı (query_pairs yerine grid). Bkz [[ida-livox-mid360]], [[ida-oak-d-lite]], [[ida-software-status]].
