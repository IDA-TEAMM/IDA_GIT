#!/usr/bin/env bash
# GİRDAP İDA — USB teslim toplayıcısını Jetson'a kur (udev + systemd).
#   sudo ./teslim_kur.sh                 # kur
#   sudo ./teslim_kur.sh --kaldir        # kaldır
# Kurulduktan sonra: USB tak → kopya otomatik → USB'deki RAPOR.txt'ye bak.
set -euo pipefail
WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
LOGS="${GIRDAP_LOGS:-$HOME/girdap_logs}"
RULE=/etc/udev/rules.d/99-girdap-teslim.rules
UNIT=/etc/systemd/system/girdap-teslim@.service

if [[ "${1:-}" == "--kaldir" ]]; then
  rm -f "$RULE" "$UNIT"
  udevadm control --reload-rules && systemctl daemon-reload
  echo "kaldirildi"; exit 0
fi
[[ $EUID -eq 0 ]] || { echo "sudo ile calistir"; exit 1; }

install -m 644 "$WS/scripts/99-girdap-teslim.rules" "$RULE"
sed -e "s|__WS__|$WS|g" -e "s|__LOGS__|$LOGS|g" \
    "$WS/scripts/girdap-teslim@.service" > "$UNIT"
chmod 644 "$UNIT"
udevadm control --reload-rules && udevadm trigger --subsystem-match=block
systemctl daemon-reload
echo "kuruldu:"
echo "  kural : $RULE"
echo "  birim : $UNIT   (WS=$WS, LOGS=$LOGS)"
echo
echo "TEST (USB takmadan):"
echo "  python3 $WS/scripts/teslim_topla.py --hedef /tmp/teslim_deneme --kuru"
echo "SAHADA: USB tak -> LED sonene kadar bekle -> USB'de RAPOR.txt'yi oku"
