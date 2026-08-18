#!/bin/bash
# BANT KOŞUMU — gerçek göl bandını GÜNCEL KODLA yeniden koştur (§0.31e: kalıcı araç).
#
# 🔴 NEDEN VAR: 17.08 göl bandı (`session_20260817_193312`) analiz edildi ama
# o veriyi BUGÜNKÜ kodla koşturmanın yolu yoktu — yani "düzelttik mi?"
# sorusuna ancak yeni bir su testiyle cevap verilebiliyordu. Bu araç aynı
# sensör akışını yığına verir; kapalı döngü DEĞİLDİR (bandın pozu bizim
# komutumuza tepki vermez) ama ALGI→PLANLAMA zinciri birebir gerçek veriyle
# sınanır: hangi kilitler, kaç RRT-RED, komut dağılımı ne.
#
# Kullanım: bant_kosum.sh <bant_dizini> [sure_s] [planning_ek_argumanlari]
#
# ⚠ İZOLE DOMAIN 88 — canlı yığın (42) ve sanal göl (77) etkilenmez.
set -o pipefail
source /opt/ros/humble/setup.bash
source "$HOME/ros2_ws/install/setup.bash" 2>/dev/null
export ROS_DOMAIN_ID=88
export ROS_LOCALHOST_ONLY=1
export PYTHONPATH="$HOME/IDA_GIT/son_kodv2/karar:${PYTHONPATH:-}"

BANT="${1:?bant dizini gerekli}"; SURE="${2:-120}"; EK="${3:-}"
P="$HOME/ros2_ws/install/girdap_decision/share/girdap_decision/config/params.yaml"
L="${GIRDAP_BANT_LOG:-$HOME/girdap_logs/bant_kosum}"
mkdir -p "$L"; rm -f "$L"/*.log; : > "$L/pgids"

basla() { local ad="$1"; shift; setsid "$@" > "$L/$ad.log" 2>&1 & echo $! >> "$L/pgids"; }

# Yalnız planlama koşar: poz ve algı BANTTAN gelir (füzyon/algı yeniden
# koşturulmaz — amaç PLANLAMA katmanını gerçek girdiyle sınamak).
basla planning ros2 run girdap_decision planning_node --ros-args \
    --params-file "$P" -p use_rrt:=true $EK
sleep 6

# 🔑 cmd_vel BANTTAN OYNATILMAZ — yoksa eski komutlar yenilerine karışır.
basla bag ros2 bag play "$BANT" --clock -r 1.0 \
    --topics /girdap/fusion/odom /perception/classified_obstacles \
             /perception/obstacle_map /girdap/mission/current_target \
             /girdap/mission/waypoints /mavros/state /girdap/parkur/state

echo "bant koşumu: $BANT · ${SURE}s · domain 88 · ek: ${EK:-yok}"
sleep "$SURE"
for pg in $(cat "$L/pgids"); do kill -TERM -"$pg" 2>/dev/null; done
sleep 2
echo "--- SONUÇ ---"
echo "RRT-RED          : $(grep -c 'RRT-RED #' "$L/planning.log" 2>/dev/null)"
echo "  goal-engel     : $(grep -c 'goal engel' "$L/planning.log" 2>/dev/null)"
echo "  çözüm bulamadı : $(grep -c 'çözüm bulamadı' "$L/planning.log" 2>/dev/null)"
echo "düz çizgi düşüşü : $(grep -c 'düz çizgi hedefine' "$L/planning.log" 2>/dev/null)"
echo "HEDEF KURTARILDI : $(grep -c 'HEDEF KURTARILDI' "$L/planning.log" 2>/dev/null)"
echo "KISMİ PLAN       : $(grep -c 'KISMİ PLAN' "$L/planning.log" 2>/dev/null)"
echo "PIVOT-OLCEMEDI   : $(grep -c 'PIVOT-OLCEMEDI' "$L/planning.log" 2>/dev/null)"
echo "kilit dağılımı   :"
grep -o 'kontrol kilidi degisti: .*' "$L/planning.log" 2>/dev/null | sed 's/.*degisti: //' | sort | uniq -c | sort -rn | head -8
