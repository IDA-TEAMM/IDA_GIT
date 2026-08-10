#!/usr/bin/env bash
# GİRDAP İDA — girdap-karar.service'i Jetson'a kurar (yolları KENDİ bulur)
#
#   sudo bash jetson_kur_karar.sh
#   sudo bash jetson_kur_karar.sh --baslat   # kurulumdan sonra hemen başlat (duman testi)
#
# 🔴 NEDEN YOLLARI ELLE YAZMIYORUZ: repodaki şablon
# `PYTHONPATH=__WS__/src/girdap-decision` varsayıyor, ama 10.08 SSD
# yenilemesinde yerleşim DEĞİŞTİ:
#     ~/ros2_ws/src/girdap_decision  →  symlink  →  ~/IDA_GIT/son_kodv2/karar/ros2_ws/src/girdap_decision
#     prototype/                     →            ~/IDA_GIT/son_kodv2/karar/prototype
# Node'ların çoğu `from prototype...` import ediyor (fsm · fusion · planning ·
# mavros_bridge · telemetry · local_map · lidar_kayit …). PYTHONPATH yanlışsa
# yığın **açılışta çöker**. O yüzden `prototype/` dizini ARANIR, bulunduğu
# yerin ÜST dizini PYTHONPATH yapılır — yerleşim bir daha değişse de tutar.
set -uo pipefail

BASLAT=0
[[ "${1:-}" == "--baslat" ]] && BASLAT=1
say(){ printf '\n\033[1m== %s\033[0m\n' "$*"; }
ok(){  printf '   \033[32m✓\033[0m %s\n' "$*"; }
uy(){  printf '   \033[33m!\033[0m %s\n' "$*"; }
ht(){  printf '   \033[31m✗\033[0m %s\n' "$*"; }
[[ $EUID -ne 0 ]] && { ht "root gerekli: sudo bash $0"; exit 1; }

KULLANICI="${SUDO_USER:-girdap}"
EV="$(getent passwd "$KULLANICI" | cut -d: -f6)"
WS="$EV/ros2_ws"

say "1) Yolları bul"
[[ -f "$WS/install/setup.bash" ]] || { ht "$WS/install/setup.bash yok — workspace derlenmemiş"; exit 1; }
ok "workspace: $WS"

PROTO="$(find "$EV" -maxdepth 6 -type d -name prototype -path '*karar*' 2>/dev/null | head -1)"
[[ -n "$PROTO" ]] || { ht "prototype/ bulunamadı — node'lar import edemez, kurulum ANLAMSIZ"; exit 1; }
PP="$(dirname "$PROTO")"
ok "prototype: $PROTO"
ok "PYTHONPATH: $PP"
python3 -c "import sys; sys.path.insert(0,'$PP'); import prototype.telemetry.saat_guveni as m; print('   ✓ prototype import DENENDI ve calisti:', m.saat_guvenilir_mi()[1])" 2>&1 | tail -1

say "2) Servisi yaz"
cat > /etc/systemd/system/girdap-karar.service <<UNIT
# GIRDAP IDA — karar yigini (jetson_kur_karar.sh tarafindan URETILDI)
# Yollar kurulum aninda tespit edildi; yerlesim degisirse script yeniden kosulur.
[Unit]
Description=GIRDAP IDA karar yigini (mavros + FSM + planning + telemetri)
After=network.target
# md 4.2: saat, teslim node'lari dosya adlarini uretmeden ONCE dogru olmali.
# Wants (Requires DEGIL): saat kurulamazsa gorev YINE baslamali.
After=girdap-saat.service
Wants=girdap-saat.service

[Service]
Type=simple
User=$KULLANICI
Environment=PYTHONPATH=$PP
Environment=ROS_DOMAIN_ID=42
WorkingDirectory=$EV
ExecStart=/bin/bash -lc 'source /opt/ros/humble/setup.bash && source $WS/install/setup.bash && exec ros2 launch girdap_decision hardware.launch.py mission_source:=fc'
Restart=on-failure
RestartSec=10

[Install]
WantedBy=multi-user.target
UNIT
systemctl daemon-reload
ok "yazildi: /etc/systemd/system/girdap-karar.service"
systemd-analyze verify /etc/systemd/system/girdap-karar.service 2>&1 | grep -i girdap-karar && uy "unit uyarisi var" || ok "unit dogrulamasi temiz"

say "3) Enable"
systemctl enable girdap-karar >/dev/null 2>&1 && ok "acilista baslayacak" || uy "enable edilemedi"

if [[ $BASLAT -eq 1 ]]; then
  say "4) Duman testi — 20 saniye kosturup log'a bakiyoruz"
  uy "Tekne beslenmiyorsa MAVROS baglanamaz, bu NORMAL."
  uy "Aradigimiz sey: PYTHONPATH/import hatasi VAR MI."
  systemctl start girdap-karar
  sleep 20
  printf '   durum: %s\n' "$(systemctl is-active girdap-karar)"
  printf '   node sayisi: %s\n' "$(pgrep -fc 'girdap_decision' || echo 0)"
  echo "   --- import/launch hatasi arayisi ---"
  journalctl -u girdap-karar --since "-30s" --no-pager 2>/dev/null \
    | grep -iE "ModuleNotFoundError|ImportError|No module|package not found|executable not found|Traceback" \
    | head -8 || echo "   (import/launch hatasi YOK)"
  echo "   --- son 12 satir ---"
  journalctl -u girdap-karar -n 12 --no-pager 2>/dev/null | sed 's/^/   /'
else
  say "4) Baslatilmadi"
  uy "Duman testi icin: sudo bash $0 --baslat"
fi

say "NOTLAR"
cat <<'SON'
   • Tekne beslenince:  sudo systemctl restart girdap-karar
     ardindan:          journalctl -fu girdap-karar
   • Durdurmak icin:    sudo systemctl stop girdap-karar
   • Acilista basmasin: sudo systemctl disable girdap-karar
   • YARISMA GUNU ek adim: girdap-karar-yarisma.conf drop-in'i kurulacak
     (ROS_LOCALHOST_ONLY=1 + GIRDAP_CONFIG_OVERLAY=yarisma.yaml).
     Kurulmazsa yigin SESSIZCE video modunda kosar.
SON
