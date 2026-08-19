#!/bin/bash
# TAM BANT KOŞUMU — bant + karar yığını + DOĞRULAMA KURAL MOTORU (§1.67).
#
# 🔴 NEDEN AYRI BİR BETİK: `bant_kosum.sh` yalnız planlamayı koşturur ve
# sonucu SAYAÇLARLA raporlar ("kaç RRT-RED, kaç damga ıskası"). Bu betik
# üstüne `dogrulama_node`'u da bağlar; böylece gerçek bant, kural motorunun
# 12 kuralına (F1·F2·F2R·F4·F5 · S1·S2·S5 · C1·C2·C3) CANLI sokulur.
# Fark: sayaç "kaç kez oldu" der, kural motoru "ne kadar payımız kaldı" der
# (marj) — ve ihlali ADIYLA gösterir.
#
# ⚠ Bandın taşımadığı şey yeniden koşturulamaz: ham LiDAR/kamera kayıtta yok
# (KAR-09), o yüzden algı ve füzyon düğümleri BANTTAN beslenir, yeniden
# çalıştırılmaz. Yani bu koşum ALGI→PLANLAMA→KONTROL zincirini sınar,
# sensör sürücülerini değil.
#
# Kullanım: bant_kosum_tam.sh <bant_dizini> [sure_s] [baslangic_offset_s]
# ⚠ İZOLE DOMAIN 89 — canlı yığın (42), sanal göl (77) ve bant_kosum (88)
#   etkilenmez.
set -o pipefail
source /opt/ros/humble/setup.bash
source "$HOME/ros2_ws/install/setup.bash" 2>/dev/null
export ROS_DOMAIN_ID=89
export ROS_LOCALHOST_ONLY=1
export PYTHONPATH="$HOME/IDA_GIT/son_kodv2/karar:${PYTHONPATH:-}"

BANT="${1:?bant dizini gerekli}"; SURE="${2:-300}"; OFS="${3:-0}"
P="$HOME/ros2_ws/install/girdap_decision/share/girdap_decision/config/params.yaml"
L="${GIRDAP_BANT_LOG:-$HOME/girdap_logs/bant_kosum_tam}"
mkdir -p "$L"; rm -f "$L"/*.log; : > "$L/pgids"
basla() { local ad="$1"; shift; setsid "$@" > "$L/$ad.log" 2>&1 & echo $! >> "$L/pgids"; }

# Şalterler `bant_kosum.sh` ile BİREBİR aynı tutulur (kusur 3 dersi).
basla planning ros2 run girdap_decision planning_node --ros-args \
    --params-file "$P" \
    -p use_sim_time:=true -p use_rrt:=true \
    -p rrt_hedef_kurtarma_m:=3.0 -p rrt_kismi_plan_min_m:=0.0 \
    -p pivot_yakin_esik_m:=0.50 -p setpoint_bekci_esik_s:=0.5
sleep 6

# Kural motoru: sınanan sisteme kod enjekte etmez, yalnız abone olur (§1.58e).
basla dogrulama ros2 run girdap_decision dogrulama_node --ros-args \
    -p use_sim_time:=true -p degerlendirme_hz:=5.0
sleep 3

basla bag ros2 bag play "$BANT" --clock -r 1.0 --start-offset "$OFS" \
    --topics /girdap/fusion/odom /perception/classified_obstacles \
             /perception/obstacle_map /perception/buoys /perception/buoys_3d \
             /girdap/mission/current_target /girdap/mission/waypoints \
             /mavros/state /girdap/parkur/state /girdap/mission/state

echo "TAM bant koşumu: $BANT · ${SURE}s · offset ${OFS}s · domain 89"
sleep "$SURE"
for pg in $(cat "$L/pgids"); do kill -TERM -"$pg" 2>/dev/null; done
sleep 2

echo "═══════════ PLANLAMA ═══════════"
echo "RRT-RED          : $(grep -c 'RRT-RED #' "$L/planning.log" 2>/dev/null)"
echo "  goal-engel     : $(grep -c 'goal engel' "$L/planning.log" 2>/dev/null)"
echo "  çözüm bulamadı : $(grep -c 'çözüm bulamadı' "$L/planning.log" 2>/dev/null)"
echo "düz çizgi düşüşü : $(grep -c 'düz çizgi hedefine' "$L/planning.log" 2>/dev/null)"
echo "HEDEF KURTARILDI : $(grep -c 'HEDEF KURTARILDI' "$L/planning.log" 2>/dev/null)"
echo "POZ-BAYAT        : $(grep -c 'POZ-BAYAT' "$L/planning.log" 2>/dev/null)"
echo "damga tampon dışı: $(grep -oE 'damgada poz bulunamadı \(tampon dışı/boş, toplam [0-9]+' "$L/planning.log" 2>/dev/null | grep -oE '[0-9]+$' | tail -1)"
echo "kadans bekçisi   : $(grep -oE 'AÇIK SIFIR basıldı \(toplam [0-9]+' "$L/planning.log" 2>/dev/null | grep -oE '[0-9]+$' | tail -1)"
echo "kalıcı harita    : $(grep -oh 'kalıcı harita: [0-9]* kayıt' "$L/planning.log" 2>/dev/null | grep -oE '[0-9]+' | tail -1) kayıt (unutulan $(grep -oh 'unutulan [0-9]*' "$L/planning.log" 2>/dev/null | grep -oE '[0-9]+' | tail -1))"
echo "kilit dağılımı   :"
grep -o 'kontrol kilidi degisti: .*' "$L/planning.log" 2>/dev/null | sed 's/.*degisti: //' | sort | uniq -c | sort -rn | head -6

echo "═══════════ KURAL MOTORU ═══════════"
if [ -s "$L/dogrulama.log" ]; then
  echo "İHLAL satırı     : $(grep -cE 'İHLAL|IHLAL' "$L/dogrulama.log" 2>/dev/null)"
  echo "--- ihlal eden kurallar ---"
  grep -oE '\[(İHLAL|IHLAL)\] [A-Z0-9]+' "$L/dogrulama.log" 2>/dev/null | sort | uniq -c | sort -rn | head -12
  echo "--- STALE (hiç ölçülmedi) ---"
  grep -oE 'STALE[^,]*' "$L/dogrulama.log" 2>/dev/null | sort | uniq -c | sort -rn | head -8
  echo "--- düğüm açılış ---"
  head -5 "$L/dogrulama.log"
else
  echo "⚠ dogrulama.log BOŞ — düğüm açılmamış olabilir"
fi
