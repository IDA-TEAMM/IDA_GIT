#!/usr/bin/env bash
# GİRDAP İDA — saat servisi + Pixhawk symlink kurulumu (Jetson'da, sudo ile)
#
#   sudo bash saat_kur.sh            # kur (idempotent, tekrar koşulabilir)
#   sudo bash saat_kur.sh --teshis   # HİÇBİR ŞEY KURMA, yalnız durumu bildir
#
# Ne yapar (sırayla, her adımda kontrol ederek):
#   1. FTDI cihazlarını listeler, Pixhawk'ı HEARTBEAT ile bulur (varsa)
#   2. /dev/pixhawk sabit adının VARLIĞINI doğrular — kural YAZMAZ, sebebi
#      aşağıda (Eyüp'ün 99-girdap-fc.rules'u zaten kurulu ve daha kapsamlı)
#   3. pymavlink'i kurar (yoksa)
#   4. girdap_saat_kur.py + girdap-saat.service kurar, enable eder
#   5. girdap-karar.service'e After=/Wants= satırlarını ekler (yedek alarak)
#   6. hardware.yaml fcu_url'ini symlink'e çevirir — YALNIZ symlink varsa
#      (2026-08-09: Jetson'da zaten /dev/pixhawk'a bakıyor, bu adım no-op)
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

for p in /dev/ttyUSB*; do
  [[ -e "$p" ]] || continue
  s=$(udevadm info -q property -n "$p" 2>/dev/null | sed -n 's/^ID_SERIAL_SHORT=//p')
  printf '   %s  seri=%s\n' "$p" "${s:-?}"
  [[ "$s" == "DU0EFEA7" ]] && ok "$p → Pixhawk TELEM2 FTDI (seri no teyitli)"
done
# NOT: burada HEARTBEAT denenMİYOR. Sebep: MAVROS koşarken portu tutar, bu test
# hep başarısız olur ve yanlış alarm verir; ayrıca cihazı seri numarasından
# tanımak zaten yeterli (udev kuralı da onu kullanıyor). FC'nin gerçekten
# konuştuğu, saat servisi koşulduğunda görülecek — asıl doğrulama orası.

# ------------------------------------------------------------- 2. symlink
# 🔴 BU SCRIPT UDEV KURALI YAZMAZ — 2026-08-09'da Jetson'da Eyüp'ün
# `99-girdap-fc.rules` kuralı KURULU ve ÇALIŞIR bulundu (/dev/pixhawk →
# ttyUSB0, seri no DU0EFEA7 teyitli) ve o kural fazlasını da yapıyor:
# `ID_MM_DEVICE_IGNORE=1` ile ModemManager'ın portu problayıp MAVROS'u ~30 s
# geciktirmesini engelliyor (F-M.8). İkinci bir kural yazmak aynı cihaza iki
# farklı symlink adı üretir → kafa karışıklığı, kazanç yok.
say "2) Pixhawk sabit adı (/dev/pixhawk)"
if [[ -e /dev/pixhawk ]]; then
  ok "symlink var: /dev/pixhawk → $(readlink -f /dev/pixhawk)"
  if [[ -f /etc/udev/rules.d/99-girdap-fc.rules ]]; then
    ok "kaynağı: 99-girdap-fc.rules (Eyüp; ModemManager engeli de içinde)"
  else
    uyar "symlink var ama 99-girdap-fc.rules YOK → reboot'ta kaybolabilir"
  fi
else
  hata "/dev/pixhawk YOK — 99-girdap-fc.rules kurulmalı (girdap-video reposunda)"
  uyar "kurulmadan saat servisi FC'yi bulamaz. Geçici: --port /dev/ttyUSB0 ile koş"
fi

# ------------------------------------------------------- 3. MAVLink okuma yolu
# 🔴 pymavlink KURULMAZ ve GEREKMİYOR. 09.08'de Jetson'da ölçüldü:
# varsayılan rota YOK, DNS YOK → apt/pip çalışmaz. Bu bir arıza değil, md 4.1'in
# sonucu (WiFi yasak). Hedef makinede kurulamayan bir bağımlılık yarışma-kritik
# bir açılış servisinde olmamalı → çerçeveleme `girdap_saat_kur.py` içinde elle
# yapılıyor (CRC doğrulamalı, 8 testle çivili).
# Mesajı istemek de gerekmiyor: SR2_EXTRA3=10 ve SYSTEM_TIME EXTRA3 grubunda →
# TELEM2'de 10 Hz kendiliğinden akıyor.
say "3) MAVLink okuma yolu"
if python3 -c "import serial" 2>/dev/null; then
  ok "pyserial kurulu → bağımsız ayrıştırıcı bunu kullanacak"
elif python3 -c "import termios" 2>/dev/null; then
  ok "pyserial yok ama stdlib termios var → yedek yol çalışır"
else
  hata "ne pyserial ne termios — port açılamaz (beklenmeyen durum)"
fi
python3 -c "import pymavlink" 2>/dev/null \
  && ok "pymavlink de var (kullanılır, ama şart değil)" \
  || ok "pymavlink yok — SORUN DEĞİL, bağımsız yol birincil"

# ------------------------------------------------------- 4. script + servis
say "4) saat servisi"
PORT="/dev/ttyUSB0"
[[ -e /dev/pixhawk ]] && PORT="/dev/pixhawk"
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
elif [[ ! -e /dev/pixhawk ]]; then
  uyar "symlink yok → fcu_url DEĞİŞTİRİLMEDİ (bilinçli: symlink'siz çevirmek yığını kapatır)"
elif grep -q "/dev/pixhawk" "$YAML"; then
  ok "zaten symlink'e bakıyor"
elif [[ $TESHIS -eq 0 ]]; then
  cp "$YAML" "$YAML.yedek"
  sed -i 's#serial:///dev/ttyUSB0:#serial:///dev/pixhawk:#' "$YAML"
  grep -q "/dev/pixhawk" "$YAML" && ok "çevrildi: $YAML (yedek: $YAML.yedek)" \
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
