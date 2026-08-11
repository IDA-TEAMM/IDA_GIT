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
#   bash scripts/rosbag_kaydet.sh --lidar           # + ham /livox/lidar (AĞIR!)
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
#
# 🔴 12.08.2026 (KAR-09) — `/livox/lidar` VARSAYILANDAN ÇIKARILDI.
# Kaptanın bag analizi: `session_19700101_020120` **14 GB / 4,9 milyon mesaj**
# ve bunun **3,6 milyonu** bu tek topic'ten. Aynı oturumda tüm hat (MAVROS
# dahil) **8-12 saniye** donuyordu; disk G/Ç doygunluğu en güçlü aday.
# 2 m/s'de 12 saniye = **24 metre kör seyir**.
#
# Ham nokta bulutu şartname çıktılarının HİÇBİRİNDE kullanılmıyor: Dosya-3
# (yerel harita) `local_map_node`'un OccupancyGrid'inden, Dosya-1b ise
# işlenmiş cluster'lardan üretiliyor. Yani bu topic kayıtta bir hata ayıklama
# lüksüydü ve bedeli görev güvenliğiydi.
# Gerçekten gerekiyorsa `--lidar` ile açıkça iste (tercihen ayrı diske).
VARSAYILAN=(
    /girdap/fusion/odom
    /girdap/fusion/pose
    /mavros/imu/data
    /mavros/global_position/global
    /mavros/local_position/pose
    /mavros/local_position/velocity_body
    /mavros/setpoint_velocity/cmd_vel_unstamped
    /mavros/rc/in
    /mavros/state
    # 12.08 (PAR-03): FC'nin pre-arm RET SEBEBİ buradan gelir. Araç 14
    # oturumda hiç ARM edilemedi ve sebebi hiçbir kayıtta yoktu — yalnız
    # operatörün ekranında bir an görünüp kayboldu.
    /mavros/statustext/recv
    /mavros/statustext/send
    /diagnostics
    # 12.08 (KAR-04): thrust NEDEN sıfır. Bu topic kaydedilmezse "komut sıfır"
    # ile "komut yok" ayrımı yine bag'den elle çıkarılmak zorunda kalır —
    # kaptanın 30.874 mesaj için yapmak zorunda kaldığı şey tam da buydu.
    /girdap/control/inhibit_reason
    /perception/obstacle_map
    /tf
    /tf_static
)

# --tam: + algı/karar arayüz topic'leri (bu oturumda doğrulanan zincir).
# /mavros/state artık VARSAYILAN'da (11.08) — burada tekrarlanmıyor, aksi
# halde ros2 bag record'a aynı topic iki kez verilir.
# 11.08.2026 (öğleden sonra) — GUIDED görev denemesi için EKLENENLER:
# kayıt "tekne gitmeye çalıştı mı" sorusuna cevap veremiyordu. Komut zinciri
# FSM durumu → hedef → thrust şeklinde ilerliyor; ortadaki iki halka
# (current_target, control/thrust) hiç kaydedilmiyordu, yani tekne kıpırdamazsa
# zincirin NEREDE koptuğu sonradan anlaşılamıyordu. MP'den yüklenen ham görev
# (/mavros/mission/waypoints) ve karar katmanının onu nasıl okuduğu
# (/girdap/mission/waypoints) de kayda girdi — ikisi arasındaki fark
# mission_manager'ın görevi doğru sindirip sindirmediğini gösterir.
EK_TAM=(
    /perception/buoys
    /perception/buoys_3d
    /perception/classified_obstacles
    /perception/gate_target
    /perception/gate_passed
    /perception/gate_count
    /girdap/planning/gate
    /girdap/planning/gate_count
    /girdap/planning/edge_buoys
    /girdap/mission/state
    /girdap/mission/current_target
    /girdap/mission/waypoints
    /girdap/mission/waypoint_reached
    /girdap/mission/complete
    /girdap/control/thrust
    /girdap/parkur/state
    /mavros/mission/waypoints
    /mavros/mission/reached
)

EK_LIDAR=(/livox/lidar)

LIDAR_ISTENDI=0
if [ "${1:-}" = "--lidar" ]; then
    LIDAR_ISTENDI=1
    shift
fi

if [ "${1:-}" = "--tam" ]; then
    TOPICLER=("${VARSAYILAN[@]}" "${EK_TAM[@]}")
    shift
elif [ $# -ge 1 ]; then
    TOPICLER=("$@")
else
    TOPICLER=("${VARSAYILAN[@]}")
fi

if [ "$LIDAR_ISTENDI" = "1" ]; then
    TOPICLER+=("${EK_LIDAR[@]}")
    echo "⚠ HAM LiDAR KAYDI ACIK — kaptanin olcumunde bu topic tek basina"
    echo "  3,6 milyon mesaj / ~14 GB uretti ve tum hattin 8-12 s donmasiyla"
    echo "  ayni oturumda gorundu (KAR-09). Uzun kosuda kullanma."
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

# 🔴 KAR-09 öneri #2: termal/CPU korelasyonu ANCAK eşzamanlı kayıtla kurulur.
# Donmaların sebebi (disk G/Ç mi, termal kısıtlama mı) bag verisinden tek
# başına ayırt EDİLEMİYOR — kaptanın raporunda ikisi de "en olası aday"
# olarak açık kaldı. tegrastats bunu ayırır ve maliyeti ~sıfırdır.
# ⚠ tegrastats log'u bag DIZININİN İÇİNE yazılamaz: `ros2 bag record -o`
# hedef dizin ZATEN VARSA hata verir, dizini önceden yaratmak kaydı komple
# engellerdi. Kardeş dosya olarak yazıyoruz (aynı zaman damgası eşleştirir).
TEGRA_LOG="$CIKTI_KOK/session_${DAMGA}_tegrastats.txt"
if command -v tegrastats >/dev/null 2>&1; then
    tegrastats --interval 1000 --logfile "$TEGRA_LOG" &
    TEGRA_PID=$!
    # Ctrl+C rosbag'e gider; tegrastats'ı biz toplamalıyız.
    trap 'kill "$TEGRA_PID" 2>/dev/null || true' EXIT INT TERM
    echo "tegrastats: $TEGRA_LOG (1 s aralik, KAR-09 korelasyonu)"
else
    echo "⚠ tegrastats yok — donma sebebi (termal mi G/C mi) ayirt edilemez"
fi
echo

exec ros2 bag record \
    -o "$CIKTI" \
    -s mcap \
    --compression-mode file \
    --compression-format zstd \
    "${TOPICLER[@]}"
