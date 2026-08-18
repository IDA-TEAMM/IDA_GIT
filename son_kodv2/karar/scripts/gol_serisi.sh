#!/bin/bash
# SANAL GÖL SERİSİ — dalga dozu artırılarak PDÇ tepkisi (§1.48 A/B tabanı).
# Kalıcı: GIRDAP_DURUM §1.50 (dört noktalı dalga serisi). Çıktı: ~/girdap_logs/gol_seri/
# ⚠ Canlı servisler ÇAĞIRAN tarafından durdurulmuş olmalı (domain 77 izole olsa da CPU paylaşılır).
# Servisler ÇAĞIRAN tarafından durdurulmuş olmalı.
S="${GIRDAP_SERI_LOG:-$HOME/girdap_logs/gol_seri}"
K=$HOME/IDA_GIT/son_kodv2/karar
SONUC="$S/seri_sonuc.csv"
echo "etiket;ornek;sure_s;son_x;son_y;iceride;disarida;kapsam_disi;pdc;pdc_etkin;dis_sure_s;en_uzun_s;en_derin_m;p1;p2" > "$SONUC"

# etiket:yanal_mps:yaw_rps
KOSUMLAR=("taban:0.0:0.0" "hafif:0.09:0.025" "orta:0.18:0.05" "agir:0.27:0.075")

for k in "${KOSUMLAR[@]}"; do
    IFS=: read -r ad yanal yaw <<< "$k"
    echo "── KOŞUM: $ad (yanal ${yanal} m/s · yaw ${yaw} rad/s)"
    export GIRDAP_GOL_LOG="$S/gol_$ad"
    bash "$K/scripts/gol_kos.sh" 8 12.0 4.0 4 "$yanal" "$yaw" > /dev/null 2>&1
    sleep 8
    (
        source /opt/ros/humble/setup.bash
        source "$HOME/ros2_ws/install/setup.bash" 2>/dev/null
        export ROS_DOMAIN_ID=77
        export PYTHONPATH="$K:$PYTHONPATH"
        setsid python3 "$K/scripts/gol_iz_kaydet.py" "$S/iz_$ad.csv" > /dev/null 2>&1 &
        echo $! > "$S/kayit_$ad.pid"
    )
    # Bitişi bekle: "PARKUR TAMAMLANDI" ya da 240 s tavan
    for i in $(seq 1 240); do
        sleep 1
        if grep -aq "PARKUR TAMAMLANDI" "$S/gol_$ad/sanal_gol.log" 2>/dev/null; then
            echo "   🏁 tamamlandı (${i} s)"; break
        fi
        [ "$i" = 240 ] && echo "   ⏱ 240 s tavanına vuruldu (görev bitmedi)"
    done
    sleep 2
    kill "$(cat "$S/kayit_$ad.pid")" 2>/dev/null
    sleep 1
    bash "$K/scripts/gol_dur.sh" > /dev/null 2>&1
    sleep 3
    python3 "$K/scripts/gol_pdc_olc.py" "$S/iz_$ad.csv" "$ad" 8 12.0 4.0 >> "$SONUC"
    tail -1 "$SONUC"
done
echo "── SERİ BİTTİ → $SONUC"
