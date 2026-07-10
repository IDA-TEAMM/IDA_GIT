set -e
LOG_DIR="/tmp/ida_logs"
mkdir -p "$LOG_DIR"
rm -f "$LOG_DIR"/*.log
PIDS=()
if [ -e /dev/ttyUSB0 ]; then
    FCU_URL="serial:///dev/ttyUSB0:57600"
    echo "Pixhawk bulundu: /dev/ttyUSB0"
elif [ -e /dev/ttyACM0 ]; then
    FCU_URL="serial:///dev/ttyACM0:57600"
    echo "Pixhawk bulundu: /dev/ttyACM0"
else
    FCU_URL="udp://127.0.0.1:14550@"
    echo "UYARI: Pixhawk bulunamadi, UDP test modunda baslatiliyor."
fi
cleanup() {
    echo "=== Sistem kapatiliyor ==="
    for pid in "${PIDS[@]}"; do kill -9 "$pid" 2>/dev/null || true; done
    exit 0
}
trap cleanup SIGINT SIGTERM
pkill -9 -f 'sensor_node.py|perception_node.py|decision_node.py|control_node.py|mavros_node' 2>/dev/null || true
sleep 2
source /opt/ros/humble/setup.bash
cd /root/ros2_ws/src/ida_topics_yeni
echo "=== 1/5 Sensor Node ==="
python3 -u ida_topics/sensor_node.py > "$LOG_DIR/sensor_node.log" 2>&1 &
PIDS+=($!)
sleep 1
echo "=== 2/5 Perception Node ==="
python3 -u ida_topics/perception_node.py > "$LOG_DIR/perception_node.log" 2>&1 &
PIDS+=($!)
sleep 2
echo "=== 3/5 MAVROS (FCU: $FCU_URL) ==="
ros2 run mavros mavros_node --ros-args -p fcu_url:="$FCU_URL" -p tgt_system:=1 -p tgt_component:=1 > "$LOG_DIR/mavros.log" 2>&1 &
PIDS+=($!)
sleep 3
echo "=== 4/5 Decision Node ==="
python3 -u ida_topics/decision_node.py > "$LOG_DIR/decision_node.log" 2>&1 &
PIDS+=($!)
sleep 1
echo "=== 5/5 Control Node ==="
python3 -u ida_topics/control_node.py > "$LOG_DIR/control_node.log" 2>&1 &
PIDS+=($!)
sleep 1
echo "=== 6/6 Telemetri Node ==="
python3 -u ida_topics/telemetri_node.py > "$LOG_DIR/telemetri_node.log" 2>&1 &
PIDS+=($!)
sleep 1
echo "=== TUM NODE'LAR BASLATILDI ==="
echo "PID'ler: ${PIDS[@]}"
wait
