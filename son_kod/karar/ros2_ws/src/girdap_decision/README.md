# girdap_decision (ROS 2 Humble)

Layer 2 ROS 2 paketi — Layer 0 Python prototiplerini (`prototype/`)
canlı topic akışına bağlar.

## Bağımlılıklar

ROS 2 Humble + paketleri:
```bash
sudo apt install ros-humble-mavros ros-humble-mavros-extras \
                 ros-humble-mavros-msgs python3-colcon-common-extensions
```

mavros GeographicLib veri seti (kurulum sonrası bir kez):
```bash
sudo /opt/ros/humble/lib/mavros/install_geographiclib_datasets.sh
```

Python: `numpy`, `scipy`, `gtsam>=4.3a0`, `pyyaml`, `matplotlib`
(repo kökündeki `requirements.txt`'ten kurulu olmalı).

## Build

`prototype/` modülünü ROS 2 paketi içinden import edebilmek için repo
kökü `PYTHONPATH`'e eklenir. `~/.bashrc` veya colcon shell'e:

```bash
export GIRDAP_ROOT=/path/to/girdap-decision
export PYTHONPATH=$GIRDAP_ROOT:$PYTHONPATH
```

Sonra:
```bash
cd ros2_ws
colcon build --packages-select girdap_decision --symlink-install
source install/setup.bash
```

## Çalıştırma

mavros'u ayrı terminalde başlat:
```bash
ros2 launch mavros apm.launch fcu_url:=/dev/ttyUSB0:921600
```

Karar yığınını başlat:
```bash
ros2 launch girdap_decision decision.launch.py
```

Görev başlatma (BEKLEMEDE → PARKUR1):
```bash
ros2 service call /girdap/mission/start std_srvs/srv/Trigger
```

Acil durdurma:
```bash
ros2 service call /girdap/mission/kill std_srvs/srv/Trigger
```

## Topic Haritası

| Yön | Topic | Mesaj | Yayınlayan / Dinleyen |
|---|---|---|---|
| ↓ | `/mavros/state` | mavros_msgs/State | fsm_node |
| ↓ | `/mavros/imu/data` | sensor_msgs/Imu | fusion_node, fsm_node, telemetry_node |
| ↓ | `/mavros/global_position/global` | sensor_msgs/NavSatFix | fusion_node, telemetry_node |
| ↓ | `/mavros/global_position/local` | nav_msgs/Odometry | fusion_node |
| ↓ | `/mavros/local_position/velocity_body` | geometry_msgs/TwistStamped | fusion_node, telemetry_node |
| ↑ | `/mavros/setpoint_velocity/cmd_vel_unstamped` | geometry_msgs/Twist | planning_node |
| → | `/girdap/fusion/odom` | nav_msgs/Odometry | fusion → planning, fsm, telemetry |
| → | `/girdap/fusion/pose` | geometry_msgs/PoseStamped | fusion → (RViz) |
| → | `/girdap/perception/obstacles` | geometry_msgs/PoseArray | (perception) → planning |
| → | `/girdap/mission/waypoints` | nav_msgs/Path | (mission) → planning |
| → | `/girdap/mission/state` | std_msgs/String | fsm → planning, telemetry |
| → | `/girdap/mission/last_gate_passed` | std_msgs/Bool | (mission) → fsm |
| → | `/girdap/control/thrust` | std_msgs/Float32MultiArray | planning → (ESC) |
| → | `/girdap/planning/global_path` | nav_msgs/Path | planning → (RViz) |

## Hata Ayıklama

Topic'leri izle:
```bash
ros2 topic list
ros2 topic echo /girdap/mission/state
ros2 topic hz /girdap/fusion/odom
```

FSM tek başına test (ROS 2 olmadan):
```bash
python -m prototype.fsm.mission_fsm
```
