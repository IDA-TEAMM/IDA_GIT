---
name: ida-livox-mid360
description: "Livox Mid-360'ı ROS 2'de /livox/lidar yayınlatma — ağ IP + build sırası tuzakları"
metadata: 
  node_type: memory
  type: project
  originSessionId: a70a49cc-c538-46e5-8247-4e8e9aa9b1d1
---

İDA topic testinde Livox Mid-360 `livox_ros_driver2` (kaynak, `~/ws_livox`) ile yayınlatıldı. Ağ: Mid-360 **Ethernet** cihazı, `eno1` üzerinden. Host `192.168.117.50`, sensör `192.168.117.100` (ping ~1.7ms). Üç tuzak:

1. **Config fabrika IP'lerinde kalmış:** `config/MID360_config.json` varsayılan `host 192.168.1.5 / lidar 192.168.1.12` idi → gerçek ağ `192.168.117.x` ile uyuşmuyordu, sürücü cihazı bulamıyordu. `host_net_info` (cmd/push/point/imu_data_ip) → `192.168.117.50`, `lidar_configs[0].ip` → `192.168.117.100` yapıldı. Hem src hem install kopyası güncellenmeli.

2. **Build sırası:** Livox-SDK2 önce `/usr/local`'e kurulmalı (`sudo make install`), YOKSA driver CMake'te `Could not find LIVOX_LIDAR_SDK_LIBRARY: liblivox_lidar_sdk_shared.so` ile patlar (Jul 4'te bu yüzden yarım kalmıştı: install'da sadece kabuk dosyaları vardı, node/launch/marker yoktu). SDK kurulduktan sonra `cd ~/ws_livox/src/livox_ros_driver2 && ./build.sh humble` → 11s'de derledi.

3. **PointCloud2 vs CustomMsg:** stok `msg_MID360_launch.py` `xfer_format=1` (CustomMsg); karar yığını (`perception_lidar_node`) PointCloud2 bekliyor. `xfer_format=0`'lı kopya `pc2_MID360_launch.py` oluşturuldu. Çalıştırma: `source ~/ws_livox/install/setup.bash && ros2 launch livox_ros_driver2 pc2_MID360_launch.py`.

Sonuç: `/livox/lidar` (PointCloud2) 10 Hz, ~19968 nokta/mesaj, frame `livox_frame`; ayrıca `/livox/imu`. Bkz [[ida-hardware]], [[ida-oak-d-lite]], [[ida-software-status]].
