set -e
LOG_DIR="/tmp/ida_logs"
mkdir -p "$LOG_DIR"
rm -f "$LOG_DIR"/*.log
PIDS=()
# Pixhawk portu (MAVROS icin)
if [ -e /dev/ttyUSB0 ]; then
    FCU_URL="serial:///dev/ttyUSB0:57600"
    echo "Pixhawk bulundu: /dev/ttyUSB0"
else
    FCU_URL="udp://127.0.0.1:14550@"
    echo "UYARI: Pixhawk bulunamadi, UDP test modunda baslatiliyor."
fi

# F9P GPS portu
if [ -e /dev/ttyACM0 ]; then
    GPS_PORT="/dev/ttyACM0"
    echo "F9P GPS bulundu: /dev/ttyACM0"
elif [ -e /dev/ttyUSB1 ]; then
    GPS_PORT="/dev/ttyUSB1"
    echo "F9P GPS bulundu: /dev/ttyUSB1"
else
    GPS_PORT=""
    echo "UYARI: F9P GPS bulunamadi"
fi
cleanup() {
    echo "=== Sistem kapatiliyor ==="
    for pid in "${PIDS[@]}"; do kill -9 "$pid" 2>/dev/null || true; done
    exit 0
}
trap cleanup SIGINT SIGTERM
pkill -9 -f 'sensor_node.py|perception_node.py|decision_node.py|control_node.py|mavros_node' 2>/dev/null || true
sleep 2
export ROS_DOMAIN_ID=42
source /opt/ros/humble/setup.bash
cd /root/ros2_ws/src/ida_topics_yeni
echo "=== 1/11 GPS/IMU Driver ==="
if [ -n "$GPS_PORT" ]; then
    python3 -u ida_topics/gps_imu_driver_node.py --ros-args -p port:="$GPS_PORT" > "$LOG_DIR/gps_imu_driver.log" 2>&1 &
else
    python3 -u ida_topics/gps_imu_driver_node.py > "$LOG_DIR/gps_imu_driver.log" 2>&1 &
fi
PIDS+=($!)
sleep 1
echo "=== 2/11 OAK-D Lite Driver ==="
python3 -u ida_topics/oakd_driver_node.py > "$LOG_DIR/oakd_driver.log" 2>&1 &
PIDS+=($!)
sleep 1
echo "=== 3/11 Livox LiDAR Driver ==="
python3 -u ida_topics/livox_driver_node.py > "$LOG_DIR/livox_driver.log" 2>&1 &
PIDS+=($!)
sleep 1
echo "=== 4/11 Sensor Node ==="
python3 -u ida_topics/sensor_node.py > "$LOG_DIR/sensor_node.log" 2>&1 &
PIDS+=($!)
sleep 1
echo "=== 5/11 Perception Node ==="
python3 -u ida_topics/perception_node.py > "$LOG_DIR/perception_node.log" 2>&1 &
PIDS+=($!)
sleep 2
echo "=== 6/11 MAVROS (FCU: $FCU_URL) ==="
ros2 run mavros mavros_node --ros-args -p fcu_url:="$FCU_URL" -p tgt_system:=1 -p tgt_component:=1 > "$LOG_DIR/mavros.log" 2>&1 &
PIDS+=($!)
sleep 3
echo "=== 7/11 Decision Node ==="
python3 -u ida_topics/decision_node.py > "$LOG_DIR/decision_node.log" 2>&1 &
PIDS+=($!)
sleep 1
echo "=== 8/11 Control Node ==="
python3 -u ida_topics/control_node.py > "$LOG_DIR/control_node.log" 2>&1 &
PIDS+=($!)
sleep 1
echo "=== 9/11 Telemetri Node ==="
python3 -u ida_topics/telemetri_node.py > "$LOG_DIR/telemetri_node.log" 2>&1 &
PIDS+=($!)
sleep 1
echo "=== 10/11 Kamera Kayit Node ==="
python3 -u ida_topics/kamera_kayit_node.py > "$LOG_DIR/kamera_kayit_node.log" 2>&1 &
PIDS+=($!)
sleep 1
echo "=== 11/11 Local Map Node ==="
python3 -u ida_topics/local_map_node.py > "$LOG_DIR/local_map_node.log" 2>&1 &
PIDS+=($!)
sleep 1
echo "=== TUM NODE'LAR BASLATILDI ==="
echo "PID'ler: ${PIDS[@]}"
wait
