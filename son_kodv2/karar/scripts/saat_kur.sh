#!/usr/bin/env bash
# GİRDAP İDA — saat servisi + Pixhawk symlink kurulumu (Jetson'da, sudo ile)
#
#   sudo bash saat_kur.sh            # kur (idempotent, tekrar koşulabilir)
#   sudo bash saat_kur.sh --teshis   # HİÇBİR ŞEY KURMA, yalnız durumu bildir
#
# Ne yapar (sırayla, her adımda kontrol ederek):
#   1. FTDI cihazlarını listeler, Pixhawk'ı HEARTBEAT ile bulur
#   2. udev kuralını gerçek seri numarasıyla yazar → /dev/pixhawk_telem2
#   3. pymavlink'i kurar (yoksa)
#   4. girdap_saat_kur.py + girdap-saat.service kurar, enable eder
#   5. girdap-karar.service'e After=/Wants= satırlarını ekler
#   6. hardware.yaml fcu_url'ini symlink'e çevirir — YALNIZ symlink varsa
#
# 🔴 Saati KURMAZ: GPS fix gerekiyor, kapalı alanda gelmez. Kurulum bitince
#    tekne dışarıdayken:  sudo systemctl start girdap-saat
#                         journalctl -u girdap-saat -n 30 --no-pager
set -uo pipefail

TESHIS=0
[[ "${1:-}" == "--teshis" ]] && TESHIS=1

KOK="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
say() { printf '\n\033[1m== %s\033[0m\n' "$*"; }
ok()  { printf '   \033[32m✓\033[0m %s\n' "$*"; }
uyar(){ printf '   \033[33m!\033[0m %s\n' "$*"; }
hata(){ printf '   \033[31m✗\033[0m %s\n' "$*"; }

if [[ $TESHIS -eq 0 && $EUID -ne 0 ]]; then
  hata "root gerekli:  sudo bash $0"; exit 1
fi

# ---------------------------------------------------------------- 1. FTDI'ler
say "1) Seri cihazlar"
if [[ -d /dev/serial/by-id ]]; then
  ls -l /dev/serial/by-id/ | sed 's/^/   /'
else
  uyar "/dev/serial/by-id yok — hiç USB seri cihaz görünmüyor. Kablo?"
fi

PIX_SERI=""
PIX_PORT=""
for p in /dev/ttyUSB*; do
  [[ -e "$p" ]] || continue
  s=$(udevadm info -q property -n "$p" 2>/dev/null | sed -n 's/^ID_SERIAL_SHORT=//p')
  printf '   %s  seri=%s\n' "$p" "${s:-?}"
  # Hangisi Pixhawk? Tek güvenilir ölçüt HEARTBEAT (isim/seri no tahmin olur).
  if python3 - "$p" <<'PY' >/dev/null 2>&1
import sys
from pymavlink import mavutil
m = mavutil.mavlink_connection(sys.argv[1], baud=57600)
sys.exit(0 if m.wait_heartbeat(timeout=6) else 1)
PY
  then
    ok "$p → HEARTBEAT var, bu Pixhawk"
    PIX_PORT="$p"; PIX_SERI="$s"
  fi
done

if [[ -z "$PIX_SERI" ]]; then
  uyar "Pixhawk heartbeat'i bulunamadı."
  uyar "Sebepleri: FC kapalı · kablo · pymavlink yok · MAVROS portu TUTUYOR."
  uyar "MAVROS koşuyorsa önce durdur:  sudo systemctl stop girdap-karar"
  uyar "→ udev kuralı ATLANDI (seri no bilinmeden yazılamaz)."
else
  ok "Pixhawk seri no: $PIX_SERI  (port şu an $PIX_PORT)"
  BEKLENEN="DU0EFEA7"
  [[ "$PIX_SERI" == "$BEKLENEN" ]] \
    && ok "hardware.yaml yorumundaki seri no ($BEKLENEN) DOĞRULANDI" \
    || uyar "hardware.yaml '$BEKLENEN' diyordu, gerçek '$PIX_SERI' → kural gerçek olanla yazılıyor, dokümanı güncelle"
fi

# ------------------------------------------------------------- 2. udev kuralı
say "2) udev kuralı → /dev/pixhawk_telem2"
if [[ -n "$PIX_SERI" && $TESHIS -eq 0 ]]; then
  sed "s/DU0EFEA7/$PIX_SERI/" "$KOK/99-girdap-pixhawk.rules" \
    > /etc/udev/rules.d/99-girdap-pixhawk.rules
  udevadm control --reload-rules && udevadm trigger --subsystem-match=tty
  sleep 2
  if [[ -e /dev/pixhawk_telem2 ]]; then
    ok "symlink hazır: $(readlink -f /dev/pixhawk_telem2)"
  else
    hata "symlink OLUŞMADI — seri no ya da VID/PID uyuşmuyor olabilir"
    uyar "kontrol: udevadm info -a -n $PIX_PORT | grep -E 'ATTRS\{(serial|idVendor|idProduct)\}' | head"
  fi
else
  [[ -e /dev/pixhawk_telem2 ]] && ok "symlink zaten var: $(readlink -f /dev/pixhawk_telem2)" \
                               || uyar "symlink yok (teşhis modu ya da seri no bulunamadı)"
fi

# --------------------------------------------------------------- 3. pymavlink
say "3) pymavlink"
if python3 -c "import pymavlink" 2>/dev/null; then
  ok "kurulu"
elif [[ $TESHIS -eq 0 ]]; then
  uyar "kurulu değil, apt ile kuruluyor…"
  apt-get install -y python3-pymavlink >/dev/null 2>&1 \
    || pip3 install --break-system-packages pymavlink >/dev/null 2>&1
  python3 -c "import pymavlink" 2>/dev/null && ok "kuruldu" || hata "KURULAMADI — saat servisi çalışmaz"
else
  uyar "kurulu DEĞİL"
fi

# ------------------------------------------------------- 4. script + servis
say "4) saat servisi"
PORT="/dev/ttyUSB0"
[[ -e /dev/pixhawk_telem2 ]] && PORT="/dev/pixhawk_telem2"
if [[ $TESHIS -eq 0 ]]; then
  install -m 0755 "$KOK/girdap_saat_kur.py" /usr/local/bin/girdap_saat_kur.py
  sed "s#--port /dev/ttyUSB0#--port $PORT#" "$KOK/girdap-saat.service" \
    > /etc/systemd/system/girdap-saat.service
  systemctl daemon-reload
  systemctl enable girdap-saat >/dev/null 2>&1
  ok "kuruldu ve enable edildi (port: $PORT)"
else
  systemctl is-enabled girdap-saat >/dev/null 2>&1 && ok "enable" || uyar "kurulu değil"
fi

# ---------------------------------------------------- 5. girdap-karar sırası
say "5) girdap-karar.service sırası"
KARAR=/etc/systemd/system/girdap-karar.service
if [[ ! -f "$KARAR" ]]; then
  uyar "$KARAR yok — karar servisi henüz kurulmamış, bu adım atlandı"
elif grep -q "girdap-saat.service" "$KARAR"; then
  ok "After=/Wants= satırları zaten var"
elif [[ $TESHIS -eq 0 ]]; then
  cp "$KARAR" "$KARAR.yedek.$(date +%s 2>/dev/null || echo bak)"
  sed -i '/^\[Unit\]/a After=girdap-saat.service\nWants=girdap-saat.service' "$KARAR"
  systemctl daemon-reload
  ok "eklendi (yedek alındı)"
else
  uyar "After=/Wants= YOK — teslim dosya adları yanlış saatle üretilebilir"
fi

# ------------------------------------------------------------ 6. fcu_url
say "6) hardware.yaml fcu_url"
YAML=$(find /home -name hardware.yaml -path '*girdap_decision*' 2>/dev/null | head -1)
if [[ -z "$YAML" ]]; then
  uyar "hardware.yaml bulunamadı (son_kodv2 bu Jetson'da yok olabilir) — elle çevir"
elif [[ ! -e /dev/pixhawk_telem2 ]]; then
  uyar "symlink yok → fcu_url DEĞİŞTİRİLMEDİ (bilinçli: symlink'siz çevirmek yığını kapatır)"
elif grep -q "pixhawk_telem2" "$YAML"; then
  ok "zaten symlink'e bakıyor"
elif [[ $TESHIS -eq 0 ]]; then
  cp "$YAML" "$YAML.yedek"
  sed -i 's#serial:///dev/ttyUSB0:#serial:///dev/pixhawk_telem2:#' "$YAML"
  grep -q pixhawk_telem2 "$YAML" && ok "çevrildi: $YAML (yedek: $YAML.yedek)" \
                                 || uyar "beklenen desen bulunamadı, elle çevir"
  uyar "colcon build gerekebilir (config install/'e kopyalanıyor)"
else
  uyar "hâlâ ttyUSB0'a bakıyor"
fi

# ------------------------------------------------------------------- özet
say "SONRAKİ ADIM"
cat <<'SON'
   Saat HENÜZ kurulmadı — GPS fix şart, kapalı alanda gelmez.
   Tekne DIŞARIDA ve GPS fix varken:

     sudo systemctl start girdap-saat
     journalctl -u girdap-saat -n 30 --no-pager
     timedatectl          # "System clock synchronized: yes" görmeli
     date -u              # GPS UTC ile tutmalı

   Saati DEĞİŞTİRMEDEN önce denemek istersen:
     sudo /usr/local/bin/girdap_saat_kur.py --kuru
SON
