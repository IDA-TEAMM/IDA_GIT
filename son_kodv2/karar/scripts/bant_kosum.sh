#!/bin/bash
# BANT KOŞUMU — gerçek göl bandını GÜNCEL KODLA yeniden koştur (§0.31e: kalıcı araç).
#
# 🔴 NEDEN VAR: 17.08 göl bandı (`session_20260817_193312`) analiz edildi ama
# o veriyi BUGÜNKÜ kodla koşturmanın yolu yoktu — yani "düzelttik mi?"
# sorusuna ancak yeni bir su testiyle cevap verilebiliyordu. Bu araç aynı
# sensör akışını yığına verir; kapalı döngü DEĞİLDİR (bandın pozu bizim
# komutumuza tepki vermez) ama ALGI→PLANLAMA zinciri birebir gerçek veriyle
# sınanır: hangi kilitler, kaç RRT-RED, komut dağılımı ne.
#
# 🔴 18.08 — ÜÇ SESSİZ YANLIŞ-SONUÇ KUSURU DÜZELTİLDİ (ölçüldü, aşağıda).
# Araç önce "her şey yolunda" diyordu; üçü de sayıyı YANLIŞ yönde bozuyordu:
#
#  1) `use_sim_time` VERİLMİYORDU. Poz tamponu `_now()` (duvar saati) ile
#     dolar, `_poz_damgada()` ise BANT DAMGASIYLA (dün) sorgular → her sorgu
#     ıskalar. Ölçüm: "damgada poz bulunamadı" 899 → **4**. O 899 gerçek bir
#     kusur sanılıyordu; koşumun kendi yapaydı.
#  2) `/girdap/mission/state` OYNATILMIYORDU → FSM `BOOT`ta çakılı kalıyor,
#     `SETPOINT-KAPALI` hiç açılmıyor, yani MPPI ağır işi HİÇ yapmıyordu.
#     Ölçüm: kilitlerin %100'ü `FSM-DISI(BOOT)` → gerçek dağılım çıktı.
#     ⚠ Bu en sinsisi: "POZ-BAYAT 0" diyordu ama MPPI zaten koşmadığı için.
#  3) `ros2 run` LAUNCH'I ATLIYOR. Şalterlerin launch varsayılanı
#     (`_RRT_DEFAULTS`: 3.0) ile düğüm varsayılanı (0.0) FARKLI, ve
#     `params.yaml`da bu anahtarlar YOK → koşum düzeltmeyi KAPALI ölçüyordu.
#     Ölçüm: `HEDEF KURTARILDI` 0 → **91**. Şalterler artık açıkça veriliyor.
#
# Kullanım: bant_kosum.sh <bant_dizini> [sure_s] [planning_ek_argumanlari]
#
# ⚠ İZOLE DOMAIN 88 — canlı yığın (42) ve sanal göl (77) etkilenmez.
set -o pipefail
source /opt/ros/humble/setup.bash
source "$HOME/ros2_ws/install/setup.bash" 2>/dev/null
export ROS_DOMAIN_ID=88
export ROS_LOCALHOST_ONLY=1
export PYTHONPATH="$HOME/IDA_GIT/son_kodv2/karar:${PYTHONPATH:-}"

BANT="${1:?bant dizini gerekli}"; SURE="${2:-120}"; EK="${3:-}"
P="$HOME/ros2_ws/install/girdap_decision/share/girdap_decision/config/params.yaml"
L="${GIRDAP_BANT_LOG:-$HOME/girdap_logs/bant_kosum}"
mkdir -p "$L"; rm -f "$L"/*.log; : > "$L/pgids"

basla() { local ad="$1"; shift; setsid "$@" > "$L/$ad.log" 2>&1 & echo $! >> "$L/pgids"; }

# Yalnız planlama koşar: poz ve algı BANTTAN gelir (füzyon/algı yeniden
# koşturulmaz — amaç PLANLAMA katmanını gerçek girdiyle sınamak).
# 🔑 Şalterler AÇIKÇA veriliyor (kusur 3): launch atlandığı için düğüm
# varsayılanları geçerli olurdu ve düzeltme KAPALI ölçülürdü. Değerler
# `hardware.launch.py` içindeki `_RRT_DEFAULTS` ile aynı tutulmalıdır —
# `test_bant_kosum_salterleri.py` bunu dondurur.
basla planning ros2 run girdap_decision planning_node --ros-args \
    --params-file "$P" \
    -p use_sim_time:=true \
    -p use_rrt:=true \
    -p rrt_hedef_kurtarma_m:=3.0 \
    -p rrt_kismi_plan_min_m:=0.0 \
    -p pivot_yakin_esik_m:=0.50 \
    -p setpoint_bekci_esik_s:=0.5 \
    $EK
sleep 6

# 🔑 cmd_vel BANTTAN OYNATILMAZ — yoksa eski komutlar yenilerine karışır.
# 🔑 `/girdap/mission/state` OYNATILIR (kusur 2) — yoksa FSM BOOT'ta kalır.
basla bag ros2 bag play "$BANT" --clock -r 1.0 \
    --topics /girdap/fusion/odom /perception/classified_obstacles \
             /perception/obstacle_map /girdap/mission/current_target \
             /girdap/mission/waypoints /mavros/state /girdap/parkur/state \
             /girdap/mission/state

echo "bant koşumu: $BANT · ${SURE}s · domain 88 · ek: ${EK:-yok}"
sleep "$SURE"
for pg in $(cat "$L/pgids"); do kill -TERM -"$pg" 2>/dev/null; done
sleep 2
echo "--- SONUÇ ---"
echo "RRT-RED          : $(grep -c 'RRT-RED #' "$L/planning.log" 2>/dev/null)"
echo "  goal-engel     : $(grep -c 'goal engel' "$L/planning.log" 2>/dev/null)"
echo "  çözüm bulamadı : $(grep -c 'çözüm bulamadı' "$L/planning.log" 2>/dev/null)"
echo "düz çizgi düşüşü : $(grep -c 'düz çizgi hedefine' "$L/planning.log" 2>/dev/null)"
echo "HEDEF KURTARILDI : $(grep -c 'HEDEF KURTARILDI' "$L/planning.log" 2>/dev/null)"
echo "KISMİ PLAN       : $(grep -c 'KISMİ PLAN' "$L/planning.log" 2>/dev/null)"
echo "PIVOT-OLCEMEDI   : $(grep -c 'PIVOT-OLCEMEDI' "$L/planning.log" 2>/dev/null)"
echo "POZ-BAYAT        : $(grep -c 'POZ-BAYAT' "$L/planning.log" 2>/dev/null)"
# 🔴 18.08 — DÖRDÜNCÜ SESSİZ YANLIŞ-SONUÇ KUSURU. Eski hâl `toplam [0-9]+`
# desenini DOSYA GENELİNDE arayıp SONUNCUYU alıyordu; oysa "toplam N" ifadesi
# KADANS BEKÇİSİ ve SETPOINT BOŞLUK mesajlarında da geçiyor. Ölçüm: gerçek
# değer **300** iken özet **6** yazıyordu (kadans bekçisinin sayacı) — 50×
# İYİMSER yönde. Sayaç artık KENDİ mesajına çıpalı.
echo "damga tampon dışı: $(grep -oE 'damgada poz bulunamadı \(tampon dışı/boş, toplam [0-9]+' "$L/planning.log" 2>/dev/null | grep -oE '[0-9]+$' | tail -1)"
# ⚠ AYNI SINIF: uyarı satırları KISILMIŞ (throttle) basılıyor, satır saymak
# iç sayacı OLDUĞUNDAN AZ gösterir (ölçüldü: 3 satır ↔ gerçek 6).
echo "kadans bekçisi   : $(grep -oE 'AÇIK SIFIR basıldı \(toplam [0-9]+' "$L/planning.log" 2>/dev/null | grep -oE '[0-9]+$' | tail -1)"
echo "kilit dağılımı   :"
grep -o 'kontrol kilidi degisti: .*' "$L/planning.log" 2>/dev/null | sed 's/.*degisti: //' | sort | uniq -c | sort -rn | head -8
