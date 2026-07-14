---
name: ida-git-decision-repos
description: "Masaüstündeki iki İDA kod deposu (IDA_GIT-main, girdap-decision-main) ve derleme tuzakları"
metadata: 
  node_type: memory
  type: project
  originSessionId: bff3c4d2-b619-4132-8685-31e94f2c6fc7
---

Masaüstünde iki ayrı İDA kod deposu var (bkz [[ida-sim-workspace]] ayrı bir üçüncü workspace):

**~/Desktop/girdap-decision-main/ros2_ws** — temiz, aktif geliştirilen karar/planlama workspace'i. Tek paket `girdap_decision` (ament_python, 11 node: fsm, mission_manager, planning, fusion, perception_camera/lidar/fusion, mavros_bridge, telemetry, local_map, mock_sensors). Depsi: rclpy, std/geometry/nav/sensor_msgs, mavros_msgs, visualization_msgs — hepsi kurulu. `colcon build` ile sorunsuz derlenir. CLAUDE.md + pytest içerir.

**~/Desktop/IDA_GIT-main** — TUZAK: `src/` içine yanlışlıkla TÜM ROS2+Gazebo kaynak ağacı (ament, ros2, eProsima, eclipse-*, gazebo-release, ros-planning...) `ros2.repos` ile indirilmiş. Bunları DERLEME — Humble zaten apt ile kurulu. `build_log.txt` başka makineden (sudenaz, Python 3.14) ve bu yüzden patlamış, alakasız. Sadece 2 takım paketi önemli:
- `girdap_yenimodel` (paket adı `Girdap`, ament_cmake, model: urdf/meshes/config/launch)
- `ida_topics_yeni` (paket adı `ida_topics`, ament_python, 4 node entry-point: sensor/perception/control/decision)

Bu 2 paketi derleme komutu (bu makinede 2026-07-11'de başarıyla derlendi):
`cd ~/Desktop/IDA_GIT-main && colcon build --paths src/girdap_yenimodel src/ida_topics_yeni`

Derleme tuzakları/düzeltmeler:
- `ida_topics` KÖK dizinde de (`~/Desktop/IDA_GIT-main/ida_topics_yeni`) var → colcon "duplicate package" hatası. `--paths` ile src'dekini seç.
- `girdap_yenimodel/CMakeLists.txt` var olmayan `textures/` klasörünü install ediyor → boş `textures/` oluştur yoksa cmake install patlar.
- `gazebo_ros` kurulu DEĞİL (Gazebo Harmonic var, bu Classic'ti). Sadece runtime dep, derlemeyi engellemez ama `gazebo.launch` (classic) çalışmaz.
- `ida_topics/setup.py`'de 3 node entry-point eksik: telemetri_node, local_map_node, kamera_kayit_node (dosyalar var, ros2 run ile çağrılamaz).

## Boşluk analizi (2026-07-11 tarama)

**girdap_decision — kod SAĞLAM ama dağıtım KIRIK.** Node'lar asıl mantığı repo kökündeki `prototype/` paketinden import ediyor (RRT*, MPPI, iSAM2 füzyon, FSM). Temiz (ROS'suz) ortamda `prototype/tests`: 130 passed, 5 skipped — algoritmalar iyi test edilmiş.
- 🔴 `prototype/` ROS paketinin DIŞINDA (repo kökünde); setup.py find_packages göremiyor → colcon install'a girmiyor. `ros2 run girdap_decision planning_node` → `ModuleNotFoundError: No module named 'prototype'` (11 node'un hepsi etkileniyor). Fix: PYTHONPATH'e repo kökü ekle VEYA prototype'ı pakete dahil et. Repo kökü pyproject.toml'da [project]/[build-system] yok → `pip install -e .` çalışmaz.
- 🔴 `gtsam` kurulu değil; `prototype/fusion/pipeline.py:34` koşulsuz `import gtsam` → fusion_node çöker, füzyon testleri toplanamaz. Fix: pip install gtsam.
- 🟡 `vision_msgs` kod import ediyor ama package.xml'de tanımsız.
- decision Gazebo köprüsü `ros_gz_bridge`/Harmonic uyumlu (DOĞRU).

**ida_topics:** bağımlılık tam (ultralytics/cv2/numpy OK).
- 🔴 `sistem_baslat.sh` hardcoded `/root/ros2_ws/...` yolu + `ros2 run` yerine düz `python3 dosya.py`.
- 🟡 `config_ekf.yaml` var ama robot_localization KURULU DEĞİL + ekf_filter_node hiç başlatılmıyor → EKF füzyon devre dışı.
- launch dosyası yok, sadece shell script.

**Girdap:** 🔴 `gazebo.launch.py` Gazebo CLASSIC (gazebo_ros/spawn_entity.py) kullanıyor → Harmonic'te çalışmaz, ros_gz_sim'e port gerekli. URDF texture kullanmıyor (7 mesh referansı, hepsi mevcut).

**Stratejik:** girdap_decision (gelişmiş, Harmonic, test'li) ve ida_topics (basit, Classic, testsiz) AYNI işi yapan paralel implementasyonlar (perception/decision/local_map/telemetry). Hangisi ana hat olacak KARAR VERİLMEMİŞ. Bkz [[ida-software-status]] planlama/kontrol darboğazı.
