#!/bin/bash
# GÖL HAREKET ANALİZİ — İDA sanal gölde nasıl hareket etti? (18.08.2026)
#
#   ./gol_hareket.sh [saniye]        # varsayılan 170 s
#
# 🔑 NEDEN BANT KAYDEDİP `bant_kapi_olcum.py` KOŞUYORUZ (ikinci bir analizci
# YAZMIYORUZ): o araç GERÇEK göl bantlarını ölçmek için yazıldı ve metrik
# tanımları (huni payı, kilit genişliği, temas bandı, kerteriz hatası…)
# saha bulgularına çapalı. Sanal koşum için paralel bir analizci yazmak
# metriklerin sessizce AYRIŞMASI demek olurdu — bu oturumda tam o sınıftan
# çok hata çıktı. Aynı kodu koşturmak "sanalda böyle ↔ gölde şöyle"
# karşılaştırmasını GEÇERLİ kılar.
#
# ⚠ Bant burada TESLİM dosyası değil, ÖLÇÜM GİRDİSİDİR. `girdap-rosbag`
# servisi ayrı bir iştir ve gölde koşmuyor.
#
# Analizcinin istediği 13 topic'in 13'ü gölde yayınlanıyor (doğrulandı).
# 🪤 `set -u` YOK, BİLEREK: `/opt/ros/humble/setup.bash` tanımsız değişken
# okuyor (`AMENT_TRACE_SETUP_FILES: unbound variable`) ve betik daha ilk
# satırda ölüyor. Aynı sınıf `gol_kos.sh`'ta `set -e` ile yaşandı.
SURE="${1:-170}"
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=77
K="$HOME/IDA_GIT/son_kodv2/karar"
export PYTHONPATH="$K:$PYTHONPATH"
B="${GIRDAP_GOL_BANT:-$HOME/girdap_logs/gol/hareket_$(date +%Y%m%d_%H%M%S)}"

TOPICLER=(
  /girdap/control/inhibit_reason /girdap/fusion/pose /girdap/mission/current_target
  /girdap/planning/edge_buoys /girdap/planning/gate /girdap/planning/gate_count
  /mavros/imu/data /mavros/local_position/velocity_body
  /mavros/setpoint_velocity/cmd_vel_unstamped /mavros/state
  /perception/buoys /perception/classified_obstacles /perception/gate_count
)

# Göl ayakta mı — yoksa bant BOŞ çıkar ve analiz "veri yok" der (sessiz kusur).
if ! timeout 10 ros2 topic info /girdap/fusion/pose 2>/dev/null | grep -q "Publisher count: 1"; then
    echo "🔴 göl ayakta değil (/girdap/fusion/pose yayıncı ≠ 1)."
    echo "   Önce: GIRDAP_GOL_TAM=1 ./gol_kos.sh 8 6.0 4.0 4"
    exit 1
fi

echo "▶ $SURE s bant kaydı → $B"
timeout "$SURE" ros2 bag record -o "$B" "${TOPICLER[@]}" >/dev/null 2>&1
[ -d "$B" ] || { echo "🔴 bant oluşmadı"; exit 1; }
echo "▶ analiz (gerçek göl bantlarıyla AYNI araç)"
python3 "$K/scripts/bant_kapi_olcum.py" "$B"
