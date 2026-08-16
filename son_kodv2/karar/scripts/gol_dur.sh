#!/bin/bash
# SANAL GÖL durdurucu — YALNIZ kendi başlattığı süreç GRUPLARINI öldürür.
# Canlı yığına (launch ile açılmış, saatlerdir koşan) hiçbir koşulda dokunmaz.
#
# 🔴 14.08.2026 — §0.31e KURTARMASI: bu betik ve `gol_kos.sh`/`sanal_gol.py`
# 13.08'de yazılıp DEPOYA ALINMAMIŞTI; bir Claude oturumunun `/tmp`
# scratchpad'inde duruyorlardı. `/tmp` son açılışta (13.08 14:41) silinmişti —
# yani araçlar bir yeniden başlatma uzaklıktaydı. Yollar depoya taşındı;
# geçici klasöre BAĞLI DEĞİL artık.
S="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"          # betiklerin yeri (depo)
L="${GIRDAP_GOL_LOG:-$HOME/girdap_logs/gol}"               # log + pgid yeri

if [ -f "$L/gol.pgids" ]; then
    while read -r pg; do
        [ -n "$pg" ] && kill -TERM -"$pg" 2>/dev/null
    done < "$L/gol.pgids"
fi
sleep 3
# Kalan varsa yaş ölçütüyle süpür (canlı yığın çok daha yaşlı).
python3 "$S/gol_temizle.py" 1800 | tail -1
