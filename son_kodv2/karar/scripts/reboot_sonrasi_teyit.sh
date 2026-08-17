#!/bin/bash
# REBOOT SONRASI TEYİT — 17.08.2026 değişikliklerinin boot sınavı.
#
# NEDEN: bugün değişen her şey BOOT'ta yükleniyor ve son reboot hepsinden
# önceydi. Özellikle `girdap-ff-ayar`: kusuru tam olarak "boot'tan 29 sn
# sonra ölmek"ti; `systemctl restart` ile doğrulandı ama restart ≠ boot
# (MAVROS'un parametre indirme zamanlaması boot'ta farklı — yarış durumu
# zaten oradan çıkmıştı).
#
# KOŞUM:  bash ~/reboot_sonrasi_teyit.sh 2>&1 | tee ~/reboot_sonrasi.txt
set -uo pipefail
KIRMIZI=0
b()  { printf "\n\e[1;44m %s \e[0m\n" "$*"; }
ok() { printf "  \e[32m✓\e[0m %s\n" "$*"; }
no() { printf "  \e[31m✗\e[0m %s\n" "$*"; KIRMIZI=$((KIRMIZI+1)); }
bilgi(){ printf "     %s\n" "$*"; }

echo "REBOOT SONRASI TEYİT — $(date '+%Y-%m-%d %H:%M:%S')"
bilgi "boot: $(who -b | awk '{print $3, $4}')  ·  uptime: $(uptime -p)"

b "1/7 — SERVİSLER KENDİ BAŞINA KALKTI MI"
for s in girdap-algi girdap-karar girdap-livox girdap-saat girdap-ayar-defteri; do
  e=$(systemctl is-enabled $s 2>/dev/null); a=$(systemctl is-active $s 2>/dev/null)
  [ "$e" = enabled ] && [ "$a" = active ] && ok "$s  ($e/$a)" || no "$s  ($e/$a)"
done
bilgi "girdap-livox LiDAR yokken 'activating' döngüsünde olabilir — NORMAL"

b "2/7 — 🔴 girdap-ff-ayar BOOT'TA ÖLÜYOR MU (asıl sınav)"
a=$(systemctl is-active girdap-ff-ayar 2>/dev/null)
if [ "$a" = active ]; then
  ok "ff-ayar AYAKTA — düzeltme tuttu"
else
  no "ff-ayar $a — boot'ta öldü, düzeltme TUTMADI"
fi
if journalctl -b -u girdap-ff-ayar --no-pager 2>/dev/null | grep -q "beklenmedik tip"; then
  no "'beklenmedik tip (0)' HATASI GERİ GELDİ"
else
  ok "'beklenmedik tip' hatası YOK"
fi
journalctl -b -u girdap-ff-ayar --no-pager 2>/dev/null | tail -3 | sed 's/^/     /'
bilgi "beklenen: '⏳ bekleniyor — eksik: ARM, GUIDED'"

b "3/7 — 🔴 LIVOX SPIN YAMASI BOOT'TA YÜRÜRLÜKTE Mİ"
PID=$(pgrep -f livox_ros_driver2_node | head -1)
if [ -z "$PID" ]; then
  no "livox düğümü koşmuyor (LiDAR bağlı değilse normal olabilir)"
else
  declare -A T0
  for t in /proc/$PID/task/*; do T0[$t]=$(awk '{print $14+$15}' $t/stat 2>/dev/null); done
  sleep 3
  EN=0
  for t in /proc/$PID/task/*; do
    B=$(awk '{print $14+$15}' $t/stat 2>/dev/null); [ -z "$B" ] && continue
    P=$(( (B - ${T0[$t]:-0}) * 100 / 300 )); [ $P -gt $EN ] && EN=$P
  done
  if [ $EN -lt 30 ]; then ok "en yüklü thread %$EN çekirdek (yamadan önce %97)"
  else no "en yüklü thread %$EN — SPIN GERİ GELDİ, yama boot'ta yok"; fi
fi

b "4/7 — DAĞITIM TAZE Mİ"
CIHAZ=$(sha256sum ~/models/yolo11n_duba_rvc2.blob 2>/dev/null | cut -c1-16)
[ "$CIHAZ" = "c4d69ec75132854a" ] && ok "blob c4d69ec7… (17.08 düzeltilmiş)" \
                                  || no "blob $CIHAZ — BEKLENEN c4d69ec75132854a"
GERI=$(git -C ~/IDA_GIT rev-list --count HEAD..origin/main 2>/dev/null || echo "?")
[ "$GERI" = "0" ] && ok "IDA_GIT güncel" || no "IDA_GIT $GERI commit GERİDE (§1.41a tekrarı)"

b "5/7 — ALGI ↔ KARAR AYNI KEŞİF DÜNYASINDA MI"
source /opt/ros/humble/setup.bash >/dev/null 2>&1
source /home/girdap/ros2_ws/install/setup.bash >/dev/null 2>&1
export ROS_DOMAIN_ID=42 ROS_LOCALHOST_ONLY=1
S=$(timeout 25 ros2 topic info /perception/buoys 2>/dev/null || true)
Y=$(echo "$S" | grep -c "Publisher count: 1"); A2=$(echo "$S" | sed -n 's/.*Subscription count: //p' | head -1)
[ "$Y" = "1" ] && ok "/perception/buoys yayıncı var" || no "/perception/buoys yayıncı YOK"
[ "${A2:-0}" -ge 1 ] 2>/dev/null && ok "abone sayısı ${A2} — karar tarafı bağlı" \
                                  || no "abone YOK — §1.25 izolasyon hatası"
ls -l /dev/pixhawk >/dev/null 2>&1 && ok "/dev/pixhawk (udev boot'ta çalıştı)" \
                                    || no "/dev/pixhawk YOK — udev kuralı yüklenmedi"

b "6/7 — FC PARAMETRELERİ REBOOT'TAN SAĞ ÇIKTI MI"
for p in ATC_STR_RAT_FF:0.52 ATC_STR_RAT_P:0.104 ATC_ACCEL_MAX:0.30 COMPASS_LEARN:0; do
  ad=${p%%:*}; bek=${p##*:}
  v=$(timeout 20 ros2 param get /mavros/param $ad 2>/dev/null | sed 's/.*is: //')
  if [ -z "$v" ]; then bilgi "$ad okunamadı (yüklü makinede timeout olabilir, tekrar dene)"
  elif python3 -c "import sys;sys.exit(0 if abs(float('$v')-float('$bek'))<0.01 else 1)" 2>/dev/null
  then ok "$ad = $v"; else no "$ad = $v (beklenen ~$bek)"; fi
done

b "7/7 — POZ-BAYAT (bugünün asıl kazanımı)"
sleep 45
N=$(journalctl -u girdap-karar --no-pager --since '-45 s' 2>/dev/null | grep -c 'poz .*gelmiyor')
if [ "$N" -le 3 ]; then ok "POZ-BAYAT $N/45 sn (düzeltmelerden önce: 34/60 sn)"
else no "POZ-BAYAT $N/45 sn — yük geri gelmiş, CPU'ya bak"; fi
uptime | sed 's/^/     /'

echo
if [ $KIRMIZI -eq 0 ]; then
  printf "\e[1;42m TEYİT GEÇTİ — 17.08 değişiklikleri boot'tan sağ çıktı \e[0m\n"
else
  printf "\e[1;41m %d MADDE KIRMIZI — yukarı bak \e[0m\n" $KIRMIZI
fi
echo
echo "Ayrıca koş:  bash ~/IDA_GIT/son_kodv2/karar/scripts/reboot_teyit.sh"
echo "             bash ~/IDA_GIT/son_kodv2/karar/scripts/gol_hazir_mi.sh"
exit $KIRMIZI
