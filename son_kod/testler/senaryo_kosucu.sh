#!/bin/bash
# Koşul matrisi koşucusu — her senaryo için taze yığın + sürücü, log toplar.
# set -u YOK: ROS setup.bash'leri unbound-variable temiz değil
KOK=~/Desktop/son_kod
CIKTI=~/girdap_logs/senaryolar
mkdir -p "$CIKTI"
source /opt/ros/humble/setup.bash
source "$KOK/karar/ros2_ws/install/setup.bash"
export PYTHONPATH="$KOK/karar:$PYTHONPATH"
export ROS_DOMAIN_ID=77

kos() {
  local ad="$1" sure="$2"
  echo "================ SENARYO: $ad (${sure}s) ================"
  ls -t ~/girdap_logs/grafik/ 2>/dev/null | head -1 > "$CIKTI/$ad.grafik_onceki"
  timeout $((sure+20)) ros2 launch girdap_decision hardware.launch.py \
      with_mavros:=false mission_source:=file \
      mission_file:=masa_test_mission.yaml \
      > "$CIKTI/$ad.stack.log" 2>&1 &
  local stack_pid=$!
  sleep 8
  timeout $((sure+5)) python3 "$KOK/testler/senaryo_surucu.py" --senaryo "$ad" \
      > "$CIKTI/$ad.surucu.log" 2>&1
  wait $stack_pid 2>/dev/null
  ls -t ~/girdap_logs/grafik/ 2>/dev/null | head -1 > "$CIKTI/$ad.grafik_yeni"
  ls -t ~/girdap_logs/telemetry/ 2>/dev/null | head -1 > "$CIKTI/$ad.telemetri_yeni"
  echo "--- $ad: gözlemler:"
  grep "GOZLEM\|SENARYO" "$CIKTI/$ad.surucu.log" | head -20
  echo "--- $ad: yığın olayları (öl/KILL/FAILSAFE):"
  grep -c "process has died" "$CIKTI/$ad.stack.log"
  grep -E "FAILSAFE|KILL|görev başla|waypoint .* varıldı|TAMAMLANDI" \
      "$CIKTI/$ad.stack.log" | sed 's/^\[[^]]*\] //' | sort | uniq -c | head -12
  sleep 3
}

kos auto-once-arm 150
kos disarm-ortada 80
kos fc-kopma 80
kos gps-kayip 100
kos tamamlandi-mod 165
kos boot-gec 120
echo "=== TUM SENARYOLAR BITTI ==="
