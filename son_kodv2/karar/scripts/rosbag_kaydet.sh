#!/usr/bin/env bash
# GİRDAP İDA — rosbag2 kayıt betiği (Jetson).
#
# §0.25d'nin "Repoda rosbag yardımcısı/scripti YOK" borcunu kapatır.
# MCAP + zstd (dosya düzeyinde) — Jetson'da SQLite3'ten daha hızlı/az CPU
# (benchmark: mcap.dev/guides/benchmarks/rosbag2-storage-plugins, 2026-08-10
# araştırması). ros-humble-rosbag2-storage-mcap paketi GEREKİR.
#
# Kullanım:
#   bash scripts/rosbag_kaydet.sh                  # varsayılan liste
#   bash scripts/rosbag_kaydet.sh --tam             # + tüm /perception/* ve /girdap/*
#   bash scripts/rosbag_kaydet.sh topic1 topic2 …   # elle liste

set -euo pipefail

CIKTI_KOK="$HOME/girdap_logs/rosbag"
DAMGA="$(date +%Y%m%d_%H%M%S)"
CIKTI="$CIKTI_KOK/session_${DAMGA}"
mkdir -p "$CIKTI_KOK"

# §0.25d "Kaydedilecek doğru liste" — karar yığınının çekirdek çıktıları.
# 11.08.2026 (§0.33): setpoint_velocity + rc/in + state + diagnostics
# EKLENDİ — eski liste komutu (cmd_vel) ve güvenlik durumunu (RC kill,
# ARM/mod, mavros_router link sağlığı) hiç kaydetmiyordu. Yarın servo/motor
# doğrulaması (§0.30b) ve RC failsafe testi (§0.30c) yapılacaksa bunlar
# olmadan kayıt sonradan "hangi komut gitti, tekne neden öyle davrandı"
# sorusuna cevap veremezdi.
VARSAYILAN=(
    /girdap/fusion/odom
    /girdap/fusion/pose
    /livox/lidar
    /mavros/imu/data
    /mavros/global_position/global
    /mavros/local_position/pose
    /mavros/local_position/velocity_body
    /mavros/setpoint_velocity/cmd_vel_unstamped
    /mavros/rc/in
    /mavros/state
    /diagnostics
    /perception/obstacle_map
    /tf
    /tf_static
)

# --tam: + algı/karar arayüz topic'leri (bu oturumda doğrulanan zincir).
# /mavros/state artık VARSAYILAN'da (11.08) — burada tekrarlanmıyor, aksi
# halde ros2 bag record'a aynı topic iki kez verilir.
EK_TAM=(
    /perception/buoys
    /perception/buoys_3d
    /perception/classified_obstacles
    /perception/gate_target
    /perception/gate_passed
    /perception/gate_count
    /girdap/planning/gate
    /girdap/mission/state
)

if [ "${1:-}" = "--tam" ]; then
    TOPICLER=("${VARSAYILAN[@]}" "${EK_TAM[@]}")
    shift
elif [ $# -ge 1 ]; then
    TOPICLER=("$@")
else
    TOPICLER=("${VARSAYILAN[@]}")
fi

echo "== GİRDAP rosbag kaydı =="
echo "çıktı   : $CIKTI"
echo "format  : mcap + zstd (dosya düzeyinde sıkıştırma)"
echo "topic'ler:"
printf '  %s\n' "${TOPICLER[@]}"
echo
echo "Disk (kayıttan önce): $(df -h "$HOME" | awk 'NR==2{print $4" boş / "$2}')"
echo "Ctrl+C ile durdur (metadata düzgün yazılır)."
echo

exec ros2 bag record \
    -o "$CIKTI" \
    -s mcap \
    --compression-mode file \
    --compression-format zstd \
    "${TOPICLER[@]}"
