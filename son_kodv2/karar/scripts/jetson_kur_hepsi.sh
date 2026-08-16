#!/usr/bin/env bash
# GİRDAP İDA — Jetson'da EKSİK OLAN HER ŞEYİ tek komutta kurar
#
#   sudo bash jetson_kur_hepsi.sh
#
# 2026-08-10 SSD arızasından sonra yapılan taramada bulunan boşlukların
# tamamını kapatır. Her adım idempotent — tekrar koşmak zarar vermez.
#
#   1. Ağ     → IP'yi KALICI yap (LiDAR config'i sabit IP'ye bağlı)
#   2. Saat   → udev kuralı (/dev/pixhawk) + GPS'ten saat kuran servis
#   3. LiDAR  → livox_ros_driver2 servisi (BAŞLATAN HİÇBİR ŞEY YOKTU)
#   4. ALGI   → girdap-algi.service            🆕 11.08.2026
#   5. Karar  → girdap-karar.service (yolları kendi bulur)
#   6. Rosbag → girdap-rosbag.service + mcap paketi   🆕 11.08.2026
#
# 🔴 4. ve 6. ADIMLAR NEDEN EKLENDİ (11.08.2026 denetimi):
# Bu betik "eksik olan HER ŞEYİ kurar" diyordu ama iki servisi hiç kurmuyordu:
#   · girdap-algi.service  → yalnız `algi/scripts/jetson_kur.sh --servis` ile
#     kuruluyordu, yani ayrı bir betiği ayrıca koşmayı bilmek gerekiyordu.
#   · girdap-rosbag.service → HİÇBİR betikte yoktu, elle `cp` gerekiyordu.
# Algı servisi kurulmazsa `/perception/buoys` HİÇ akmaz — `hardware.launch.py`
# algıyı açmıyor (`use_onboard_camera` varsayılanı false, HSV yedek kolu da
# kapalı), yani başka üretici YOK. Zincir: buoys yok → fusion senkronu hiç
# tetiklenmez → classified_obstacles yok → select_gate None → KAPI SAYISI
# YAPISAL OLARAK SIFIR (§0.30e). Tekne sessizce ham görev noktasına sürer.
#
# Aynı klasörde bulunması gerekenler:
#   jetson_kur_ag.sh · jetson_kur_saat.sh · jetson_kur_karar.sh
#   girdap-livox.service · girdap-saat.service · girdap_saat_kur.py
#   girdap-rosbag.service · rosbag_kaydet.sh · 99-girdap-fc.rules
# Algı servisi algı deposundan okunur: <repo>/son_kodv2/algi/scripts/
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

bas "4/6 — ALGI (girdap-algi.service)"
# Algı servisi algı deposunda duruyor. Bu betik karar/scripts/ içinde, yani
# repo kökü iki seviye yukarısı: <repo>/son_kodv2/karar/scripts → son_kodv2.
ALGI_S=""
for _y in "$K/../../algi/scripts/girdap-algi.service" \
          "$EV/IDA_GIT/son_kodv2/algi/scripts/girdap-algi.service"; do
  [[ -f "$_y" ]] && { ALGI_S="$(readlink -f "$_y")"; break; }
done
AWS="$EV/ros2_ws"
if [[ -z "$ALGI_S" ]]; then
  ht "girdap-algi.service bulunamadı — algı deposu yerinde mi?"
  uy "Aranan: $K/../../algi/scripts/ ve $EV/IDA_GIT/son_kodv2/algi/scripts/"
  uy "🔴 KURULMAZSA /perception/buoys HİÇ akmaz → kapı sayısı sıfır (§0.30e)"
elif [[ ! -f "$AWS/install/setup.bash" ]]; then
  ht "$AWS/install/setup.bash yok — workspace derlenmemiş, algı servisi kurulmadı"
else
  sed -e "s|__USER__|$KULLANICI|g" -e "s|__WS__|$AWS|g" \
      "$ALGI_S" > /etc/systemd/system/girdap-algi.service
  systemctl daemon-reload
  systemctl enable girdap-algi >/dev/null 2>&1
  ok "kuruldu ve enable edildi (kaynak: $ALGI_S)"
  # Tek OAK var; veri seti toplayıcısı açıkken algı kamerayı alamaz. Toplayıcı
  # 2026-08-16'da repodan kaldırıldı, dolayısıyla `Conflicts=` koruması da yok
  # (olmayan bir unit'i işaret eden satır sahte güven verirdi). Kalıntı unit
  # eski kurulumlarda diskte kalmış olabilir; enabled ise boot'ta kamerayı
  # kapar ve algı HİÇ açılmaz (md 4.1).
  if [ -f /etc/systemd/system/girdap-veriseti.service ]; then
    uy "KALINTI: girdap-veriseti.service hâlâ kurulu — tek OAK, boot'ta algıyı kilitleyebilir."
    uy "Temizlik: sudo systemctl disable --now girdap-veriseti; sudo rm /etc/systemd/system/girdap-veriseti.service; sudo systemctl daemon-reload"
  fi
fi

bas "5/6 — KARAR YIĞINI"
bash "$K/jetson_kur_karar.sh" || uy "karar adımı sorunlu"

bas "6/6 — ROSBAG KAYDI (girdap-rosbag.service)"
# rosbag_kaydet.sh `-s mcap` ile çağırıyor; paket yoksa servis her açılışta
# çöker ve Restart=on-failure 10 sn'de bir yeniden dener (journal şişer, kayıt
# hiç oluşmaz). Önce paket, sonra servis.
if dpkg -s ros-humble-rosbag2-storage-mcap >/dev/null 2>&1; then
  ok "ros-humble-rosbag2-storage-mcap kurulu"
else
  uy "mcap depolama paketi yok — kuruluyor (internet gerektirir)"
  if apt-get install -y ros-humble-rosbag2-storage-mcap >/dev/null 2>&1; then
    ok "kuruldu"
  else
    ht "kurulamadı — YARIŞMA GÜNÜ İNTERNET YOK (md 4.1), şimdi hallet."
    uy "Aksi hâlde girdap-rosbag sürekli çöker, bant kaydı HİÇ oluşmaz."
  fi
fi
# __KARAR__ = repo karar kökü (bu betiğin bir üstü). Yer tutucu adı bilerek
# __WS__ DEĞİL: diğer unit'lerde __WS__ = ~/ros2_ws (colcon workspace) demek,
# aynı adı iki anlamda kullanmak toplu sed'de sessiz yanlış yola bağlardı.
KARAR_KOK="$(cd "$K/.." && pwd)"
if [[ ! -f "$K/rosbag_kaydet.sh" ]]; then
  ht "rosbag_kaydet.sh yok — rosbag servisi kurulmadı"
elif [[ ! -f "$K/girdap-rosbag.service" ]]; then
  ht "girdap-rosbag.service şablonu yok"
else
  sed -e "s|__USER__|$KULLANICI|g" -e "s|__KARAR__|$KARAR_KOK|g" \
      "$K/girdap-rosbag.service" > /etc/systemd/system/girdap-rosbag.service
  systemctl daemon-reload
  systemctl enable girdap-rosbag >/dev/null 2>&1
  ok "kuruldu ve enable edildi (karar kökü: $KARAR_KOK)"
  uy "Her boot YENİ session_<damga>/ açar — uzun beklemede 'df -h ~' ile bak."
fi

bas "ÖZET"
for s in girdap-saat girdap-livox girdap-algi girdap-karar girdap-rosbag; do
  printf '   %-16s enabled=%-10s active=%s\n' "$s" \
    "$(systemctl is-enabled $s 2>&1)" "$(systemctl is-active $s 2>&1)"
done
cat <<'SON'

   AÇILIŞ SIRASI (systemd kendi çözer):
     ag(IP) → girdap-saat → girdap-livox + girdap-algi → girdap-karar
              → girdap-rosbag
   (girdap-karar artık ikisini de Wants= ediyor; Requires DEĞİL — biri
    kalkmazsa görev YİNE başlar, çünkü başlamazsa hiç puan yok.)

   TEKNE BESLENİNCE DOĞRULAMA:
     ls -l /dev/pixhawk                     # symlink oluşmalı
     sudo systemctl restart girdap-livox girdap-algi girdap-karar
     source /opt/ros/humble/setup.bash
     export ROS_DOMAIN_ID=42
     ros2 topic hz /livox/lidar             # ~10 Hz
     ros2 topic hz /mavros/imu/data         # ~8 Hz  (FC bağlı demek)
     ros2 topic hz /perception/buoys        # algı zinciri ayakta mı
     ls ~/girdap_logs/rosbag/               # session_<damga>/ oluşmalı
     journalctl -u girdap-saat -n 20 --no-pager   # "SAAT KURULDU" bekleniyor

   TAM TEYİT (salt okur, sudo istemez):
     bash scripts/jetson_teyit.sh

   🔴 YARIŞMA GÜNÜ EK ADIM: girdap-karar-yarisma.conf drop-in'i.
   (11.08'den beri hardware.yaml zaten YARIŞMA tabanı — drop-in unutulursa
    yığın artık video moduna DÜŞMEZ. Drop-in hâlâ ROS_LOCALHOST_ONLY=1 için
    gerekli, md 4.1. Parkur-1 tek başına koşulacaksa girdap-karar-parkur1.conf;
    ikisi AYNI ANDA kurulmaz.)
SON
