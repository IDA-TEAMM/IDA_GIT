#!/usr/bin/env bash
# GİRDAP — FC PARAM OKUMA TURU (SALT OKUR — FC'ye HİÇBİR ŞEY YAZMAZ)
#
# Pixhawk bağlıyken tek komut: senkron çiftleri karşılaştırır, motor güç
# tavanını + failsafe + 12.07 OLAY paramlarını okur, OPS-1 görev-boş
# kontrolü yapar. "Önce OKU, sonra karar" ilkesinin aracı (saha listesi P0.4).
#
# Kullanım (girdap-karar servisi çalışırken, domain 42):
#   bash scripts/fc_param_turu.sh
#
# ⚠️ İLK GERÇEK FC BAĞLANTISINDA DOĞRULANACAK — betik FC yokken yazıldı
#    (2026-07-15, kuru koşu yapılamadı); mavros param arayüzü sürüme göre
#    değişebilirse ilk koşuda düzeltilir.

set -u
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-42}"

set +u
[ -f /opt/ros/humble/setup.bash ] && source /opt/ros/humble/setup.bash
[ -f "$HOME/ros2_ws/install/setup.bash" ] && source "$HOME/ros2_ws/install/setup.bash"
set -u

C_OK=$'\e[32m'; C_WARN=$'\e[33m'; C_ERR=$'\e[31m'; C_0=$'\e[0m'

echo "=== GİRDAP FC PARAM TURU (salt-okur) — domain $ROS_DOMAIN_ID ==="

# --- 0) FC bağlı mı ---
STATE="$(timeout 12 ros2 topic echo /mavros/state --once 2>/dev/null)"
if ! grep -q "connected: true" <<<"$STATE"; then
    echo "${C_ERR}HATA: mavros connected değil (FC bağlı mı? servis ayakta mı?)${C_0}"
    echo "  kontrol: ls -l /dev/pixhawk /dev/ttyUSB*  +  systemctl status girdap-karar"
    exit 1
fi
echo "${C_OK}FC bağlı ✓${C_0}  (mod: $(grep '^mode:' <<<"$STATE" | head -1 | cut -d' ' -f2))"

# --- 1) FCU parametrelerini mavros'a çek (salt okuma öncesi cache doldurma) ---
timeout 30 ros2 service call /mavros/param/pull mavros_msgs/srv/ParamPull "{force_pull: false}" >/dev/null 2>&1 \
    || echo "${C_WARN}uyarı: param/pull başarısız — mevcut cache ile denenecek${C_0}"

fcu_param() {  # $1 = FCU param adı → değer (okunamazsa "?")
    local out
    out="$(timeout 10 ros2 param get /mavros/param "$1" 2>/dev/null | sed 's/.*value is: //')"
    [ -n "$out" ] && echo "$out" || echo "?"
}
node_param() {  # $1=node $2=param → değer ("?" = ulaşılamadı)
    local out
    out="$(timeout 10 ros2 param get "$1" "$2" 2>/dev/null | sed 's/.*value is: //')"
    [ -n "$out" ] && echo "$out" || echo "?"
}

# --- 2) SENKRON ÇİFTLERİ (yanlışsa Ekran-2 yalan söyler, md 3.3.1.1) ---
echo
echo "--- SENKRON ÇİFTLERİ ---"
WP_SPEED="$(fcu_param WP_SPEED)";   FC_CRUISE="$(node_param /telemetry_node fc_cruise_setpoint_mps)"
WP_RADIUS="$(fcu_param WP_RADIUS)"; ARR_R="$(node_param /mission_manager_node arrival_radius_m)"
cmp_pair() {  # $1=etiket $2=FC değeri $3=bizim değer $4=fark mesajı
    printf "%-14s FC=%-8s bizim=%-8s " "$1" "$2" "$3"
    if [ "$3" = "?" ]; then
        echo "${C_WARN}bizim değer okunamadı — servis domain'inde (42) koşmuyorsan normal; elle kıyasla${C_0}"
    elif [ "$2" = "$3" ]; then
        echo "${C_OK}EŞ ✓${C_0}"
    else
        echo "$4"
    fi
}
cmp_pair "WP_SPEED"  "$WP_SPEED"  "$FC_CRUISE" "${C_ERR}FARK — hardware.yaml fc_cruise_setpoint_mps'i FC'ye eşitle${C_0}"
cmp_pair "WP_RADIUS" "$WP_RADIUS" "$ARR_R"     "${C_WARN}fark (F-V.8 senkronu sayesinde kritik değil, yine de eşitle)${C_0}"

# --- 3) MOTOR GÜÇ TAVANI (Eyüp sorusu: "tam güç çalışır mı?") ---
echo
echo "--- MOTOR GÜÇ TAVANI (AUTO'da hızı/gücü FC yönetir) ---"
for prm in MOT_THR_MAX MOT_SLEWRATE CRUISE_SPEED CRUISE_THROTTLE MOT_THR_MIN; do
    printf "  %-16s = %s\n" "$prm" "$(fcu_param $prm)"
done
echo "  (MOT_THR_MAX = sert %% tavan; WP_SPEED hedef HIZ'dır, tavan değildir)"

# --- 4) FAILSAFE (saha listesi P0.5: telemetri koparsa motorlar durmalı) ---
echo
echo "--- FAILSAFE PARAMLARI ---"
for prm in FS_ACTION FS_TIMEOUT FS_THR_ENABLE FS_CRASH_CHECK FS_GCS_ENABLE; do
    printf "  %-16s = %s\n" "$prm" "$(fcu_param $prm)"
done

# --- 5) 12.07 OLAY PARAMLARI (FC ekibi kararıyla bilinçli böyle — İZLE) ---
echo
echo "--- OLAY PARAMLARI (beklenen: ARMING_RUDDER=2, ARMING_CHECK=0, MODE4=10/AUTO, BRD_SAFETY_DEFLT=0) ---"
for prm in ARMING_RUDDER ARMING_CHECK MODE_CH MODE4 MODE5 BRD_SAFETY_DEFLT MIS_RESTART INITIAL_MODE; do
    printf "  %-18s = %s\n" "$prm" "$(fcu_param $prm)"
done
echo "  (bu değerler = güç verilince kendiliğinden ARM+AUTO riski → OPS-1 pazarlıksız)"

# --- 6) OPS-1: FC görev hafızası BOŞ mu ---
echo
echo "--- OPS-1: FC GÖREV HAFIZASI ---"
WPS="$(timeout 12 ros2 topic echo /mavros/mission/waypoints --once 2>/dev/null)"
if grep -q "waypoints: \[\]" <<<"$WPS"; then
    echo "${C_OK}görev hafızası BOŞ ✓ (OPS-1 sağlanmış)${C_0}"
elif [ -z "$WPS" ]; then
    echo "${C_WARN}okunamadı (latched topic henüz gelmemiş olabilir — 30 sn sonra tekrar dene)${C_0}"
else
    N="$(grep -c "command:" <<<"$WPS" || true)"
    echo "${C_ERR}DİKKAT: FC hafızasında ~$N görev item'ı VAR — OPS-1: oturum sonunda sil,${C_0}"
    echo "${C_ERR}batarya+RC açıkken bu görev kendiliğinden koşabilir (12.07 OLAYI)${C_0}"
fi

echo
echo "=== TUR BİTTİ — hiçbir parametre YAZILMADI ==="
