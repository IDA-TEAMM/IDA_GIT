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
# 5./6. argüman: dalga bozucusu (yanal sürüklenme m/s · yaw rad/s) — 0 = kapalı
DALGA="${5:-0.0}"; DALGA_YAW="${6:-0.0}"
# 7. argüman: başlangıç yönü (derece). 90 = kuzey = ESKİ DAVRANIŞ BİREBİR.
# -90 → burun güneye, bütün görev noktaları ARKADA: F-F.22'nin (geri sürüş)
# ölçülebildiği tek sahne. 17.08 göl bandında komutların %23,1'i geriydi ve
# %83,4'ünde hedef arkadaydı; bu sahne olmadan sim o sınıfı hiç göremiyor.
YON0="${7:-90.0}"
# ⚠ rclpy parametre tipi KATIDIR: `-90` INTEGER gelir ve düğüm
# `InvalidParameterTypeException` ile ÖLÜR — üstelik sanal_gol ölünce bütün
# zincir "poz HİÇ gelmedi" der, sebep gizlenir. Kabuk tarafında ondalığa
# zorlanır (ölçüldü: 17.08, ilk A/B koşumu tam bu yüzden boş döndü).
YON0="$(printf '%.1f' "$YON0")"
# 8./9. argüman: F-S.18 GERÇEKÇİ ALGI. 0 = kusursuz (eski davranış).
# 1.0 = 17.08 göl bandında ÖLÇÜLEN şiddet (kaçırma + konum hatası).
# HAYALET = kare başına eklenen kimliksiz (UNKNOWN) tespit sayısı;
# gerçek bantta ~95'ti (kıyı), simde parkur küçük olduğu için dozlanır.
GERCEKCILIK="$(printf '%.2f' "${8:-0.0}")"; HAYALET="${9:-0}"
# 10. argüman: algı kusur TOHUMU. Aynı tohum = aynı kusur dizisi. A/B'de
# kollar AYNI tohumla koşulur (eşleştirilmiş kıyas); gürültü tabanı ölçülürken
# tohum DEĞİŞTİRİLİR. §19.4: "A/B tek koşumla YAPILAMAZ".
TOHUM="${10:-0}"
# 11. argüman: F-P.30 kolu — kimliksiz kümenin azami yarıçapı (0 = ham temsil)
HMAKS="$(printf '%.2f' "${11:-0.0}")"
# Planlama şalterleri ortamdan geçer (A/B için) — hepsi varsayılan KAPALI.
GOL_PLANNING_EK="${GIRDAP_GOL_PLANNING_EK:-}"

basla() {
    local ad="$1"; shift
    setsid "$@" > "$L/$ad.log" 2>&1 &
    echo $! >> "$L/gol.pgids"
}

basla sanal_gol python3 "$S/sanal_gol.py" --ros-args \
    -p kapi_sayisi:="$KAPI" -p kapi_acikligi_m:="$ACIK" \
    -p kapi_araligi_m:="$ARALIK" -p engel_sayisi:="$ENGEL" \
    -p dalga_genlik_mps:="$DALGA" -p dalga_yaw_rps:="$DALGA_YAW" \
    -p baslangic_yon_derece:="$YON0" \
    -p algi_gercekcilik:="$GERCEKCILIK" -p hayalet_sayisi:="$HAYALET" \
    -p algi_tohum:="$TOHUM" -p hayalet_maks_yaricap:="$HMAKS"
sleep 2
basla fusion   ros2 run girdap_decision fusion_node --ros-args --params-file "$P" -p use_isam2:=false
basla mission  ros2 run girdap_decision mission_manager_node --ros-args --params-file "$P" -p mission_source:=fc
basla fsm      ros2 run girdap_decision fsm_node --ros-args --params-file "$P"
basla bridge   ros2 run girdap_decision mavros_bridge_node --ros-args --params-file "$P"
basla planning ros2 run girdap_decision planning_node --ros-args --params-file "$P" -p use_rrt:=true $GOL_PLANNING_EK
echo "sanal göl: $KAPI kapı · açıklık $ACIK m · aralık $ARALIK m · $ENGEL engel · dalga ${DALGA} m/s yanal + ${DALGA_YAW} rad/s yaw · başlangıç yönü ${YON0}° · algı gerçekçilik ${GERCEKCILIK} · hayalet ${HAYALET}"
[ -n "$GOL_PLANNING_EK" ] && echo "planning ek şalter: $GOL_PLANNING_EK"
