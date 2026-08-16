#!/bin/bash
# SANAL GÖL — kapalı döngü uçtan uca koşum. Canlı yığından İZOLE (domain 77).
#
# Kullanım:  gol_kos.sh [kapi_sayisi] [kapi_acikligi_m] [kapi_araligi_m] [engel_sayisi]
# Varsayılan: gerçek parkur geometrisi (§0.17b) — 8 kapı, 12 m açıklık, 4 m aralık.
#
# 🔴 13.08 dersi: `ros2 run X` bir SARMALAYICI süreç açar, asıl düğüm ayrı
# süreçtir ve komut satırı canlı yığınla AYNI görünür. Sarmalayıcıyı öldürmek
# çocuğu öldürmez → domain'de hayalet düğüm kalır (ölçüldü: 4 kopya fsm_node,
# görev durumu KILL'e titredi). Çözüm: her düğüm `setsid` ile KENDİ SÜREÇ
# GRUBUNDA başlar; `gol_dur.sh` grubu topluca öldürür.
source /opt/ros/humble/setup.bash
source "$HOME/ros2_ws/install/setup.bash" 2>/dev/null
export ROS_DOMAIN_ID=77
export PYTHONPATH="$HOME/IDA_GIT/son_kodv2/karar:$PYTHONPATH"
P="$HOME/ros2_ws/install/girdap_decision/share/girdap_decision/config/params.yaml"
S="$HOME/IDA_GIT/son_kodv2/karar/scripts"
L_KOK="${GIRDAP_GOL_LOG:-$HOME/girdap_logs/gol}"
L="$L_KOK"
mkdir -p "$L"; rm -f "$L"/*.log; : > "$L/gol.pgids"

KAPI="${1:-8}"; ACIK="${2:-12.0}"; ARALIK="${3:-4.0}"; ENGEL="${4:-4}"

basla() {
    local ad="$1"; shift
    setsid "$@" > "$L/$ad.log" 2>&1 &
    echo $! >> "$L/gol.pgids"
}

basla sanal_gol python3 "$S/sanal_gol.py" --ros-args \
    -p kapi_sayisi:="$KAPI" -p kapi_acikligi_m:="$ACIK" \
    -p kapi_araligi_m:="$ARALIK" -p engel_sayisi:="$ENGEL"
sleep 2
basla fusion   ros2 run girdap_decision fusion_node --ros-args --params-file "$P" -p use_isam2:=false
basla mission  ros2 run girdap_decision mission_manager_node --ros-args --params-file "$P" -p mission_source:=fc
basla fsm      ros2 run girdap_decision fsm_node --ros-args --params-file "$P"
basla bridge   ros2 run girdap_decision mavros_bridge_node --ros-args --params-file "$P"
basla planning ros2 run girdap_decision planning_node --ros-args --params-file "$P" -p use_rrt:=true
echo "sanal göl: $KAPI kapı · açıklık $ACIK m · aralık $ARALIK m · $ENGEL engel"
