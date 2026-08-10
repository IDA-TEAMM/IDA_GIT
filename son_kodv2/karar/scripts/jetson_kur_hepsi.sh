#!/usr/bin/env bash
# GİRDAP İDA — Jetson'da EKSİK OLAN HER ŞEYİ tek komutta kurar
#
#   sudo bash jetson_kur_hepsi.sh
#
# 2026-08-10 SSD arızasından sonra yapılan taramada bulunan boşlukların
# tamamını kapatır. Her adım idempotent — tekrar koşmak zarar vermez.
#
#   1. Ağ    → IP'yi KALICI yap (LiDAR config'i sabit IP'ye bağlı)
#   2. Saat  → udev kuralı (/dev/pixhawk) + GPS'ten saat kuran servis
#   3. LiDAR → livox_ros_driver2 servisi (BAŞLATAN HİÇBİR ŞEY YOKTU)
#   4. Karar → girdap-karar.service (yolları kendi bulur)
#
# Aynı klasörde bulunması gerekenler:
#   jetson_kur_ag.sh · jetson_kur_saat.sh · jetson_kur_karar.sh
#   girdap-livox.service · girdap-saat.service · girdap_saat_kur.py
#   99-girdap-fc.rules
set -uo pipefail
K="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
KULLANICI="${SUDO_USER:-girdap}"
EV="$(getent passwd "$KULLANICI" | cut -d: -f6)"

bas(){ printf '\n\033[1;44m  %s  \033[0m\n' "$*"; }
ok(){  printf '   \033[32m✓\033[0m %s\n' "$*"; }
uy(){  printf '   \033[33m!\033[0m %s\n' "$*"; }
ht(){  printf '   \033[31m✗\033[0m %s\n' "$*"; }
[[ $EUID -ne 0 ]] && { ht "root gerekli: sudo bash $0"; exit 1; }

bas "1/4 — AĞ (IP kalıcı)"
bash "$K/jetson_kur_ag.sh" || uy "ağ adımı sorunlu, devam ediliyor"

bas "2/4 — SAAT + UDEV"
bash "$K/jetson_kur_saat.sh" || uy "saat adımı sorunlu, devam ediliyor"

bas "3/4 — LIDAR (livox_ros_driver2)"
LWS="$EV/livox_ws"
if [[ ! -f "$LWS/install/setup.bash" ]]; then
  ht "$LWS/install/setup.bash yok — livox_ws derlenmemiş, servis kurulmadı"
elif [[ -f /etc/systemd/system/girdap-livox.service ]] && grep -q "$LWS" /etc/systemd/system/girdap-livox.service; then
  ok "girdap-livox.service zaten kurulu"
  systemctl is-enabled girdap-livox >/dev/null 2>&1 && ok "enabled" || { systemctl enable girdap-livox >/dev/null 2>&1; ok "enable edildi"; }
else
  sed -e "s|__USER__|$KULLANICI|g" -e "s|__LWS__|$LWS|g" \
      "$K/girdap-livox.service" > /etc/systemd/system/girdap-livox.service
  systemctl daemon-reload
  systemctl enable girdap-livox >/dev/null 2>&1
  ok "kuruldu ve enable edildi (livox_ws: $LWS)"
fi
# Config'in host IP'si gerçekten bizim IP mi — sessiz arızanın tek uyarısı bu.
CFG="$LWS/src/livox_ros_driver2/config/MID360_config.json"
if [[ -f "$CFG" ]]; then
  HIP=$(python3 -c "import json;print(json.load(open('$CFG'))['MID360']['host_net_info'].get('point_data_ip',''))" 2>/dev/null)
  [[ "$HIP" == "192.168.117.60" ]] && ok "config host IP doğru: $HIP" \
     || uy "config host IP = '$HIP' — 192.168.117.60 BEKLENİYOR, LiDAR sessizce veri üretmez"
fi

bas "4/4 — KARAR YIĞINI"
bash "$K/jetson_kur_karar.sh" || uy "karar adımı sorunlu"

bas "ÖZET"
for s in girdap-saat girdap-livox girdap-karar; do
  printf '   %-16s enabled=%-10s active=%s\n' "$s" \
    "$(systemctl is-enabled $s 2>&1)" "$(systemctl is-active $s 2>&1)"
done
cat <<'SON'

   AÇILIŞ SIRASI (systemd kendi çözer):
     ag(IP) → girdap-saat → girdap-livox → girdap-karar

   TEKNE BESLENİNCE DOĞRULAMA:
     ls -l /dev/pixhawk                     # symlink oluşmalı
     sudo systemctl restart girdap-livox girdap-karar
     source /opt/ros/humble/setup.bash
     export ROS_DOMAIN_ID=42
     ros2 topic hz /livox/lidar             # ~10 Hz
     ros2 topic hz /mavros/imu/data         # ~8 Hz  (FC bağlı demek)
     journalctl -u girdap-saat -n 20 --no-pager   # "SAAT KURULDU" bekleniyor

   🔴 YARIŞMA GÜNÜ EK ADIM (unutulursa yığın SESSİZCE video modunda koşar):
     girdap-karar-yarisma.conf drop-in'i kurulacak.
SON
