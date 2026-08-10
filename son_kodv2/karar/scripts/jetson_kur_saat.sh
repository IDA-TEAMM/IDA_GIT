#!/usr/bin/env bash
# GİRDAP İDA — Jetson'a saat servisi + Pixhawk udev kuralı kurar
#
#   sudo bash jetson_kur_saat.sh            # kur (idempotent)
#   sudo bash jetson_kur_saat.sh --teshis   # hiçbir şey kurma, durumu bildir
#
# NEDEN VAR: 2026-08-10'da Jetson'ın SSD'si arızalandı ve **her şey gitti** —
# SSH anahtarı, saat servisi, `99-girdap-fc.rules`. Yeni SSD'ye kurulum
# yapılırken bu adımlar atlanıyor (kaptan workspace'i kuruyor, servisleri değil).
# Bu script o boşluğu tek komuta indiriyor.
#
# Aynı klasörde şu dosyalar bulunmalı:
#   girdap_saat_kur.py · girdap-saat.service · 99-girdap-fc.rules
set -uo pipefail

TESHIS=0
[[ "${1:-}" == "--teshis" ]] && TESHIS=1
K="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

say(){ printf '\n\033[1m== %s\033[0m\n' "$*"; }
ok(){  printf '   \033[32m✓\033[0m %s\n' "$*"; }
uy(){  printf '   \033[33m!\033[0m %s\n' "$*"; }
ht(){  printf '   \033[31m✗\033[0m %s\n' "$*"; }

if [[ $TESHIS -eq 0 && $EUID -ne 0 ]]; then ht "root gerekli: sudo bash $0"; exit 1; fi

# ---------------------------------------------------------------- 1. udev
say "1) Pixhawk udev kuralı → /dev/pixhawk"
if [[ -f /etc/udev/rules.d/99-girdap-fc.rules ]]; then
  ok "kural zaten kurulu"
elif [[ $TESHIS -eq 1 ]]; then
  uy "kural YOK"
elif [[ -f "$K/99-girdap-fc.rules" ]]; then
  install -m 0644 "$K/99-girdap-fc.rules" /etc/udev/rules.d/
  udevadm control --reload-rules && udevadm trigger --subsystem-match=tty
  ok "kural kuruldu"
else
  ht "$K/99-girdap-fc.rules bulunamadı"
fi
if [[ -e /dev/pixhawk ]]; then
  ok "symlink hazır: /dev/pixhawk → $(readlink -f /dev/pixhawk)"
else
  uy "/dev/pixhawk YOK — FTDI takılı değil (tekne bağlı değilse NORMAL)."
  uy "Kural yerinde; kablo takılınca symlink kendiliğinden oluşur."
fi

# ------------------------------------------------------- 2. MAVLink okuma yolu
say "2) MAVLink okuma yolu"
if python3 -c "import serial" 2>/dev/null; then
  ok "pyserial var"
elif python3 -c "import termios" 2>/dev/null; then
  ok "pyserial yok ama stdlib termios var → saat servisi YİNE çalışır"
  uy "(bu yüzden pyserial ZORUNLU bağımlılık yapılmadı — Jetson'da internet yok)"
else
  ht "ne pyserial ne termios — beklenmeyen"
fi

# ------------------------------------------------------------ 3. saat servisi
say "3) Saat servisi (GPS'ten sistem saatini kurar)"
if [[ $TESHIS -eq 1 ]]; then
  systemctl is-enabled girdap-saat >/dev/null 2>&1 && ok "enabled" || uy "kurulu DEĞİL"
else
  [[ -f "$K/girdap_saat_kur.py" ]] || { ht "girdap_saat_kur.py yok"; exit 1; }
  [[ -f "$K/girdap-saat.service" ]] || { ht "girdap-saat.service yok"; exit 1; }
  install -m 0755 "$K/girdap_saat_kur.py" /usr/local/bin/girdap_saat_kur.py
  install -m 0644 "$K/girdap-saat.service" /etc/systemd/system/girdap-saat.service
  systemctl daemon-reload
  systemctl enable girdap-saat >/dev/null 2>&1
  ok "kuruldu ve enable edildi"
fi

# --------------------------------------------------- 4. girdap-karar sırası
say "4) girdap-karar.service sırası"
KS=/etc/systemd/system/girdap-karar.service
if [[ ! -f "$KS" ]]; then
  uy "girdap-karar.service YOK — karar yığını servisi henüz kurulmamış."
  uy "Kurulunca [Unit] altına şu iki satır EKLENMELİ:"
  uy "    After=girdap-saat.service"
  uy "    Wants=girdap-saat.service"
  uy "Yoksa teslim dosya adları YANLIŞ saatle üretilir (md 4.2)."
elif grep -q girdap-saat.service "$KS"; then
  ok "After=/Wants= zaten var"
elif [[ $TESHIS -eq 1 ]]; then
  uy "After=/Wants= YOK"
else
  cp "$KS" "$KS.yedek.$(date +%s)"
  sed -i '/^\[Unit\]/a After=girdap-saat.service\nWants=girdap-saat.service' "$KS"
  systemctl daemon-reload
  ok "eklendi (yedek alındı)"
fi

# ------------------------------------------------------------------ 5. saat
say "5) Saatin şu anki durumu"
printf '   %s\n' "$(date)"
timedatectl 2>/dev/null | grep -iE "System clock|RTC time|NTP service" | sed 's/^/   /'
uy "Senkron GÖRÜNSE BİLE sahada internet yok → ~8,9 saat sonra çekirdek"
uy "bayrağı 'unsync'e döner. RTC pilsizse güç kesilince saat de sıfırlanır."
uy "Bu yüzden GPS'ten kuran servis şart."

# ------------------------------------------------------------------- özet
say "SONRAKİ ADIM — tekne beslenince"
cat <<'SON'
   FTDI takılı, Pixhawk açık ve GPS FIX varken:

     ls -l /dev/pixhawk                 # symlink oluşmalı
     sudo systemctl start girdap-saat
     journalctl -u girdap-saat -n 30 --no-pager
     timedatectl                        # "System clock synchronized: yes"

   Beklenen log: "SAAT KURULDU: ... (duzeltme ±X.X s)"
   Saati DEĞİŞTİRMEDEN denemek için:
     sudo /usr/local/bin/girdap_saat_kur.py --kuru
SON
