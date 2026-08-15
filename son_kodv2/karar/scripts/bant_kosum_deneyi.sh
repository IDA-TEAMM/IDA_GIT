#!/bin/bash
# GİRDAP İDA — YALITILMIŞ BANT KOŞUM DENEYİ (FAZ 5 doğrulaması, §1.18).
#
# planning_node'u TEK BAŞINA, canlı yığına dokunmadan (ROS_DOMAIN_ID=77)
# başlatır; göl bandının YALNIZ GİRDİ konularını oynatır (bandın kendi
# thrust'ı bilerek dışarıda — çıktı yalnız bu düğümden gelir) ve düğümün her
# kontrol adımında bastığı /girdap/control/thrust varış aralıklarını ölçer =
# GERÇEK kontrol kadansı (GUIDED gerekmez).
#
# İKİ DERS (15.08'de ikisi de yaşandı, ölçümleri geçersiz kılmıştı):
#  · Humble'ın `ros2 bag play`'inde --playback-duration YOK → süre `timeout`la.
#  · `ros2 run` sarmalayıcısını öldürmek python çocuğunu YETİM bırakır →
#    çalıştırılabilir DOĞRUDAN başlatılır; koşum sonrası artık denetlenir.
#
# Kullanım: bash scripts/bant_kosum_deneyi.sh <etiket> <bant_dizini> [sure_s] [offset_s]
# Özet: python3 - ile kadans_<etiket>.txt (varış damgaları) + dugum_<etiket>.log
set +u
ETIKET="$1"; BANT="$2"; SURE="${3:-360}"; OFSET="${4:-0}"
KOK="${GIRDAP_DENEY_DIZINI:-/tmp/girdap_bant_deneyi}"; mkdir -p "$KOK"

source /opt/ros/humble/setup.bash
source $HOME/ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=77
export PYTHONPATH="$HOME/IDA_GIT/son_kodv2/karar:$PYTHONPATH"

"$HOME/ros2_ws/install/girdap_decision/lib/girdap_decision/planning_node" \
  >"$KOK/dugum_$ETIKET.log" 2>&1 &
DUGUM=$!
sleep 8
python3 "$(dirname "$0")/kadans_olcer.py" "$KOK/kadans_$ETIKET.txt" $((SURE+20)) \
  >"$KOK/olcer_$ETIKET.log" 2>&1 &
OLCER=$!
sleep 2
timeout "$SURE" ros2 bag play "$BANT" --start-offset "$OFSET" \
  --topics /perception/classified_obstacles /girdap/fusion/odom \
           /girdap/fusion/pose /girdap/mission/state /girdap/mission/waypoints \
           /girdap/mission/current_target /mavros/state \
  >"$KOK/play_$ETIKET.log" 2>&1
sleep 3
kill $DUGUM 2>/dev/null; wait $OLCER 2>/dev/null
# artık denetimi — yetim düğüm sonraki ölçümü ikiye katlar (ölçüldü: 10→20 Hz)
ARTIK=$(pgrep -f "girdap_decision/planning_node$" | wc -l)
echo "== $ETIKET bitti (artık düğüm: $ARTIK — 0 olmalı) =="
