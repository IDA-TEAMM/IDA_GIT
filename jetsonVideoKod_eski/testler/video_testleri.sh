#!/usr/bin/env bash
# GİRDAP — video zinciri BİLEŞEN BİLEŞEN doğrulama koşusu (Jetson + PC).
#
# Amaç: çekimden önce her bileşenin ("ayrı ayrı") kendi testleriyle
# KANITLI çalıştığını tek komutla göstermek. Testlerin kendisi karar/
# (girdap-decision) içindedir; bu betik onları bileşen sırasıyla koşturur
# ve sonunda PASS/FAIL özet tablosu basar.
#
# Kullanım:
#   bash testler/video_testleri.sh            # bu repodaki karar/ kopyasıyla
#   bash testler/video_testleri.sh --tam      # + sonda TAM suite
#   GIRDAP_KOD=~/ros2_ws/src/girdap-decision bash testler/video_testleri.sh
#       (Jetson'daki AYRI klona karşı koşmak için — video kurulum düzeni)
#
# Çıkış kodu: 0 = tüm bileşenler yeşil; 1 = en az bir bileşen kırmızı.

set -u

KOD="${GIRDAP_KOD:-$(cd "$(dirname "$0")/../karar" && pwd)}"
TAM=0
[ "${1:-}" = "--tam" ] && TAM=1

# ROS ortamı (varsa). set +u sarmalı ŞART: Humble setup.bash
# AMENT_TRACE_SETUP_FILES unbound-variable hatası verir (bilinen tuzak).
set +u
[ -f /opt/ros/humble/setup.bash ] && source /opt/ros/humble/setup.bash
[ -f "$HOME/girdap_deps_ws/install/setup.bash" ] && \
    source "$HOME/girdap_deps_ws/install/setup.bash"
set -u

export PYTHONPATH="$KOD:$KOD/ros2_ws/src/girdap_decision:${PYTHONPATH:-}"
cd "$KOD" || { echo "HATA: karar kodu bulunamadı: $KOD"; exit 1; }

echo "=== GİRDAP video zinciri bileşen testleri ==="
echo "kod dizini : $KOD"
echo "python     : $(python3 --version 2>&1) | numpy: $(python3 -c 'import numpy; print(numpy.__version__)' 2>/dev/null || echo 'YOK')"
echo "rclpy      : $(python3 -c 'import rclpy; print("var")' 2>/dev/null || echo 'YOK (node testleri atlanır)')"
echo "mavros_msgs: $(python3 -c 'import mavros_msgs; print("var")' 2>/dev/null || echo 'YOK (fc/e2e testleri atlanır — Jetson için apt ros-humble-mavros-msgs)')"
echo

# Bileşen listesi — sıra video zincirinin veri akış sırasıdır.
BILESEN_AD=(
  "csv_logger çekirdeği (Dosya-2 + grafik CSV sözleşmeleri)"
  "telemetry_node (F-V.1 açı + F-V.2 kapılaması)"
  "görev çekirdeği (4-nokta dikdörtgen, fc dönüşümü, F-M.1)"
  "mission_manager_node (QGC fc yolu, latched, skip_home)"
  "FSM çekirdeği (terminal durum F12.2, parkur katmanı)"
  "fsm_node (GUIDED kenar tetiği T0-j, F12.1)"
  "MAVROS köprüsü (KILL/disarm, F14.x, F-M.2)"
  "MPPI (warm-start, backend, bench)"
  "planning (bypass, replan F10.x, pipeline)"
  "launch config (F-V.5 sessiz-fallback uyarısı)"
  "Ekran-2 montaj aracı (PNG/MP4)"
  "UÇTAN UCA video zinciri (QGC görevden temiz duruşa)"
)
BILESEN_DOSYA=(
  "prototype/tests/test_telemetry_logger.py"
  "prototype/tests/test_telemetry_node.py"
  "prototype/tests/test_mission_manager.py"
  "prototype/tests/test_mission_manager_node.py"
  "prototype/tests/test_mission_fsm.py prototype/tests/test_parkur_fsm.py"
  "prototype/tests/test_fsm_node.py prototype/tests/test_parkur_fsm_node.py"
  "prototype/tests/test_mavros_bridge.py prototype/tests/test_mavros_bridge_node.py"
  "prototype/tests/test_mppi.py prototype/tests/test_bench_mppi.py"
  "prototype/tests/test_planning_bypass.py prototype/tests/test_planning_replan.py prototype/tests/test_planning_pipeline.py"
  "prototype/tests/test_hardware_launch_config.py"
  "prototype/tests/test_ekran2.py"
  "prototype/tests/test_video_e2e.py"
)

SONUC=()
KIRMIZI=0
for i in "${!BILESEN_AD[@]}"; do
  ad="${BILESEN_AD[$i]}"
  dosyalar="${BILESEN_DOSYA[$i]}"
  printf '\n--- [%02d/%02d] %s\n' "$((i+1))" "${#BILESEN_AD[@]}" "$ad"
  # shellcheck disable=SC2086 — dosya listesi bilerek sözcüklere ayrılır
  cikti="$(python3 -m pytest $dosyalar -q 2>&1)"
  kod=$?
  ozet="$(echo "$cikti" | grep -E "passed|failed|error|skipped" | tail -1)"
  if [ $kod -eq 0 ]; then
    echo "✅ $ozet"
    SONUC+=("✅ $ad — $ozet")
  else
    echo "❌ KIRMIZI (exit=$kod)"
    echo "$cikti" | tail -15
    SONUC+=("❌ $ad — $ozet")
    KIRMIZI=1
  fi
done

if [ $TAM -eq 1 ]; then
  printf '\n--- [EK] TAM SUITE (prototype/tests/)\n'
  python3 -m pytest prototype/tests/ -q 2>&1 | tail -1
fi

echo
echo "=== ÖZET ==="
for s in "${SONUC[@]}"; do echo "$s"; done
echo
if [ $KIRMIZI -eq 0 ]; then
  echo "SONUÇ: TÜM BİLEŞENLER YEŞİL ✅ — kontrol-listesi.md'ye geçebilirsin."
else
  echo "SONUÇ: KIRMIZI BİLEŞEN VAR ❌ — çekime ÇIKMA, önce nedeni bul."
fi
exit $KIRMIZI
