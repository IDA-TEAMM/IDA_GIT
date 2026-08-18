#!/bin/bash
# Jetson taklidinin rakip yükünü durdurur (göl ayrı: `gol_dur.sh`).
# Ayrı betik, çünkü yük gölün ömrü boyunca yaşamalı — bkz. jetson_taklit.sh
# içindeki "TRAP TASARIM KUSURU" notu.
D="${GIRDAP_GOL_LOG:-$HOME/girdap_logs/gol}/jetson_yuk.pids"
N=0
[ -f "$D" ] && while read -r p; do
    [ -n "$p" ] && kill "$p" 2>/dev/null && N=$((N+1))
done < "$D"
rm -f "$D"
pkill -f "while True: pass" 2>/dev/null
echo "jetson yükü durduruldu (pid dosyasından $N)"
exit 0
