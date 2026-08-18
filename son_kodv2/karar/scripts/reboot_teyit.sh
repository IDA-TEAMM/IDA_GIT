#!/usr/bin/env bash
# GİRDAP İDA — REBOOT (AUTOSTART) TEYİDİ — 16.08.2026
#
# 🔴 NEDEN VAR: yarışma günü yığını EL ile başlatmak İMKÂNSIZ.
#   md 4.1  — WiFi/BT kapalı ⇒ SSH YOK.
#   md 5.5.3.1 — YKİ'den "başlat" komutu verilmez.
# ⇒ **autostart tek yol** ve onun çalıştığının TEK geçerli kanıtı gerçek bir
#   reboot. "systemctl start ile kalktı" bunu KANITLAMAZ (memory: *"elle
#   çalışıyor ≠ servis olarak çalışıyor"*); `WorkingDirectory` eksikliği tam da
#   bu farkta yakalanmıştı — elle koşarken görünmüyordu, boot'ta node hiç
#   açılmıyordu.
#
# KULLANIM:
#   sudo reboot            # ya da: sudo systemctl reboot
#   # açıldıktan sonra, en az 90 sn bekleyip:
#   bash ~/IDA_GIT/son_kodv2/karar/scripts/reboot_teyit.sh | tee ~/reboot_teyit.txt
#
# HİÇBİR ŞEY DEĞİŞTİRMEZ — yalnız okur. sudo GEREKMEZ.

ok(){ printf '  \033[32m✓\033[0m %s\n' "$*"; }
ht(){ printf '  \033[31m✗\033[0m %s\n' "$*"; }
uy(){ printf '  \033[33m?\033[0m %s\n' "$*"; }
bas(){ printf '\n\033[1;44m %s \033[0m\n' "$*"; }

BOOT=$(date -d "$(uptime -s)" +%s)
SIMDI=$(date +%s)
AYAKTA=$(( SIMDI - BOOT ))

bas "0/5 — BOOT"
echo "     boot: $(uptime -s)   ($AYAKTA sn önce)"
(( AYAKTA < 90 )) && uy "boot'tan bu yana < 90 sn — DDS keşfi ~60-75 sn'de oturuyor, erken bakma"

bas "1/5 — SERVİSLER KENDİ BAŞINA KALKTI MI (elle start YOK)"
# Kanıt: servisin ExecMainStartTimestamp'i boot'a YAKIN olmalı. Elle
# başlatılmışsa aradaki fark dakikalar mertebesinde olur.
for s in girdap-saat girdap-algi girdap-karar girdap-rosbag; do
  akt=$(systemctl is-active  $s 2>&1)
  enb=$(systemctl is-enabled $s 2>&1)
  ts=$(systemctl show $s -p ExecMainStartTimestamp --value 2>/dev/null)
  if [[ -n $ts && $ts != "n/a" ]]; then
    bas_s=$(date -d "$ts" +%s 2>/dev/null || echo 0)
    fark=$(( bas_s - BOOT ))
  else
    fark="?"
  fi
  printf '  %-16s enabled=%-9s active=%-10s boot+%ss\n' "$s" "$enb" "$akt" "$fark"
  [[ $enb != enabled ]] && ht "$s ENABLED DEĞİL → bir daha ki boot'ta HİÇ kalkmaz"
  [[ $akt != active ]]  && ht "$s AYAKTA DEĞİL"
done
echo "     ⚠ 'boot+N' değeri dakikaları buluyorsa servis muhtemelen ELLE başlatılmış"
echo "       (o zaman bu test GEÇERSİZ — tekrar reboot et, hiçbir şeye dokunma)"

bas "2/5 — YARIŞMA PROFİLİ (md 4.1) reboot'tan SONRA da yerinde mi"
D=/etc/systemd/system/girdap-karar.service.d
[[ -f $D/girdap-karar-yarisma.conf ]] && ok "girdap-karar-yarisma.conf" \
  || ht "yarışma drop-in'i YOK — yığın video/masa ayarında koşuyor olabilir"
[[ -f $D/girdap-karar-parkur1.conf ]] && ht "parkur1.conf DA kurulu — ikisi aynı anda kurulmaz"
journalctl -u girdap-karar -b --no-pager 2>/dev/null | grep -m1 'config overlay UYGULANDI' \
  | sed 's/^.*\*\*\*/     ***/' || uy "overlay satırı bu boot'ta görünmüyor"
eksik=""
for s in girdap-algi girdap-livox girdap-rosbag girdap-saat-gec; do
  systemctl show $s -p Environment 2>/dev/null | grep -q 'ROS_LOCALHOST_ONLY=1' || eksik="$eksik $s"
done
[[ -z $eksik ]] && ok "ROS_LOCALHOST_ONLY dört serviste de" \
  || ht "ROS_LOCALHOST_ONLY EKSİK:$eksik → algı↔karar AYRI keşif dünyasında (P1+P2=0, belirtisiz)"

bas "3/5 — ALGI ↔ KARAR BAĞLI MI (asıl kanıt: topic sözleşmesi)"
export ROS_DOMAIN_ID=42 ROS_LOCALHOST_ONLY=1
source /opt/ros/humble/setup.bash 2>/dev/null
source "$HOME/ros2_ws/install/setup.bash" 2>/dev/null
# ⚠ `ros2 node list` bilerek KULLANILMIYOR: keşif penceresi kısa olduğu için
# koşudan koşuya farklı sonuç veriyor (aynı yığında 9 / 21 / 32 node görüldü).
# Kırılgan bekçi, yanlış alarmdan daha kötü — "✗" görünce operatör gerçekte
# sağlam olan sistemi kurcalar. Asıl sorulacak soru zaten node sayısı değil:
# **algının ürettiği topic'e karar tarafı abone olabiliyor mu?**
# `/perception/buoys` bizim TEK tüketilen çıktımız (10.08 ölçümü) — abonesi
# 0 ise izolasyon kopuktur ve P1+P2 yapısal olarak 0'dır.
ros2 daemon stop >/dev/null 2>&1; sleep 5
BILGI=$(timeout 45 ros2 topic info /perception/buoys 2>/dev/null)
YAY=$(echo "$BILGI" | grep -oP 'Publisher count: \K[0-9]+')
ABO=$(echo "$BILGI" | grep -oP 'Subscription count: \K[0-9]+')
echo "     /perception/buoys → yayıncı=${YAY:-?} abone=${ABO:-?}"
if [[ ${YAY:-0} -ge 1 ]]; then
  ok "algı yayıncısı görünüyor"
else
  # 🔴 İKİ AYRI SEBEP, İKİSİ DE SESSİZ — operatör yanlışını kovalamasın.
  # 16.08 mutasyonunda ÖLÇÜLDÜ: girdap-algi'nin izolasyon drop-in'i alınıp
  # servis restart edilince yayıncı 1 -> 0 oldu; node ise TAMAMEN SAĞLIKLIYDI,
  # yalnız başka bir keşif dünyasındaydı. "Node öldü" diye aramak o sabah
  # yarım saat yedirir.
  ht "YAYINCI GÖRÜNMÜYOR — iki olası sebep:"
  echo "       (a) izolasyon uyuşmazlığı: girdap-algi'de ROS_LOCALHOST_ONLY=1 YOK"
  echo "           iken karar'da VAR → node sağlıklı ama AYRI dünyada (bölüm 2'ye bak)"
  echo "       (b) algı node'u gerçekten ölü → systemctl status girdap-algi"
fi
if [[ ${ABO:-0} -ge 1 ]]; then
  ok "karar tarafı abone — iki yığın AYNI keşif dünyasında"
else
  ht "ABONE YOK → algı↔karar KOPUK (kapı takibi ve sınıflı engel HİÇ gelmez ⇒ P1+P2=0, belirtisiz)"
fi
echo "     ℹ bilgi amaçlı node sayısı: $(timeout 45 ros2 node list --no-daemon 2>/dev/null | grep -c .)"

bas "4/5 — PARKUR-3 KAPALI MI (P1/P2 ölçüm koşusu)"
systemctl show girdap-algi -p Environment 2>/dev/null | grep -q 'GIRDAP_P3_HEDEF=1' \
  && ht "GIRDAP_P3_HEDEF=1 — P3 hedef yayını AÇIK (OpenCV yolu da açılır)" \
  || ok "GIRDAP_P3_HEDEF yok/0 ⇒ /perception/targets yayını ve mono OpenCV yolu KAPALI"
# `ros2 param get` daemon üzerinden gider; yukarıda durdurulduğu için
# ilk çağrı boş dönebiliyordu → daemon'ın yeniden doğmasını bekle.
ros2 daemon start >/dev/null 2>&1; sleep 8
renk=$(timeout 30 ros2 param get /kamikaze_param_node kamikaze_target_color 2>/dev/null | tail -1)
echo "     kamikaze_target_color: ${renk:-<okunamadı>}"
echo "     ℹ boş olmalı ⇒ p3_bekleniyor=False ⇒ FSM PARKUR3'e HİÇ geçmez"

bas "5/5 — ÇÖKME DÖNGÜSÜ VAR MI (bu boot'ta)"
for s in girdap-algi girdap-karar; do
  n=$(systemctl show $s -p NRestarts --value 2>/dev/null)
  printf '  %-16s NRestarts=%s\n' "$s" "$n"
  [[ ${n:-0} -gt 3 ]] && ht "$s bu boot'ta $n kez yeniden başladı — çökme döngüsü"
done
journalctl -b -u girdap-algi -u girdap-karar --no-pager 2>/dev/null \
  | grep -icE 'Traceback|AttributeError|process has died' \
  | xargs -I{} echo "     bu boot'ta ölümcül log satırı sayısı: {}"

printf '\n\033[1m ÖZET: yukarıda ✗ YOKSA autostart KANITLANDI.\033[0m\n'
printf ' Kanıtı sakla: bash %s | tee ~/reboot_teyit.txt\n\n' "$0"
