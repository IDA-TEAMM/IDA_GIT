# Girdap İDA — Karar Algoritması

TEKNOFEST 2026 İnsansız Deniz Aracı Yarışması  
Takım ID: 989124

## Modüller
- **iSAM2 / GTSAM** — GPS + IMU + LiDAR sensör füzyonu
- **RRT*** — Global yol planlama
- **MPPI** — Lokal planlama ve engel kaçınma
- **FSM** — Görev yöneticisi

## Kurulum
```bash
python3.11 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
```

## Test
```bash
pytest prototype/tests/ -v
```
ROS node testleri için önce ortamı source'la (yoksa gerekçeli SKIP edilirler):
```bash
source /opt/ros/humble/setup.bash
# node testlerinin bağımlılıkları: ros-humble-vision-msgs, ros-humble-mavros-msgs
```
