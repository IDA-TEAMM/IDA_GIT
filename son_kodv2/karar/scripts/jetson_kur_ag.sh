#!/usr/bin/env bash
# GİRDAP İDA — Jetson'ın ethernet IP'sini KALICI yapar (NetworkManager)
#
#   sudo bash jetson_kur_ag.sh
#
# ═══════════════════════════════════════════════════════════════════════════
# 🔴 NEDEN — 2026-08-10'da bulundu, iki şeyi birden kırıyordu:
#
#  1) **LiDAR sessizce ölür.** `livox_ws/.../MID360_config.json` host IP'sini
#     **192.168.117.60** olarak SABİT yazıyor. Jetson açılışta IP'siz gelirse
#     sürücü porta bağlanamaz → `/livox/lidar` HİÇ akmaz → `obstacle_map` boş
#     → **Parkur-2 engel kaçınması imkânsız.** Belirtisi yok.
#  2) **Laptop'tan erişim gider.** Her oturumda elle `ip addr add` yazmak
#     gerekiyordu; yarışma sabahı unutulacak türden bir adım.
#
# ⚠️ DDS TUZAĞI (2026-08-07'de ölçüldü): ROS 2 katılımcıları **arayüzleri
# başlangıçta bağlar**. IP sonradan eklenirse çalışan yığın onu GÖRMEZ.
# Yani IP, yığın başlamadan ÖNCE var olmalı — kalıcı yapmanın asıl gerekçesi.
set -uo pipefail

IP=192.168.117.60/24
IF=enP8p1s0
say(){ printf '\n\033[1m== %s\033[0m\n' "$*"; }
ok(){  printf '   \033[32m✓\033[0m %s\n' "$*"; }
uy(){  printf '   \033[33m!\033[0m %s\n' "$*"; }
ht(){  printf '   \033[31m✗\033[0m %s\n' "$*"; }
[[ $EUID -ne 0 ]] && { ht "root gerekli: sudo bash $0"; exit 1; }

say "1) Arayüz"
ip link show "$IF" >/dev/null 2>&1 || { ht "$IF yok. 'ip -br a' ile adı kontrol et"; exit 1; }
ok "$IF mevcut"

say "2) NetworkManager bağlantısı"
if ! systemctl is-active --quiet NetworkManager; then
  ht "NetworkManager aktif değil — bu script onu kullanıyor"; exit 1
fi
# Arayüz "externally managed" olabilir; kendi profilimizi yaratıp ona bağlıyoruz.
PROF=girdap-lidar
if nmcli -t -f NAME connection show 2>/dev/null | grep -qx "$PROF"; then
  ok "profil zaten var: $PROF (güncelleniyor)"
else
  nmcli connection add type ethernet ifname "$IF" con-name "$PROF" >/dev/null 2>&1 \
    && ok "profil oluşturuldu: $PROF" || { ht "profil oluşturulamadı"; exit 1; }
fi

say "3) Statik IP + otomatik bağlan"
# ⚠️ ipv4.method manual + gateway YOK: bu ağ yalnız LiDAR/laptop bağlantısı,
# internet oradan gitmiyor. Gateway verilirse varsayılan rota bozulur.
nmcli connection modify "$PROF" \
  ipv4.method manual ipv4.addresses "$IP" ipv4.gateway "" ipv4.dns "" \
  ipv6.method ignore connection.autoconnect yes connection.autoconnect-priority 100 \
  >/dev/null 2>&1 && ok "statik $IP · autoconnect açık · gateway YOK (bilinçli)" \
                  || { ht "ayarlanamadı"; exit 1; }

say "4) Devreye al"
nmcli connection up "$PROF" >/dev/null 2>&1 && ok "profil devrede" || uy "up başarısız (kablo takılı mı?)"
sleep 2
ip -4 addr show "$IF" | grep -q "192.168.117.60" \
  && ok "IP aktif: $(ip -4 -br addr show "$IF")" \
  || uy "IP görünmüyor — 'nmcli device status' ile bak"

say "5) Reboot dayanıklılığı"
nmcli -t -f connection.autoconnect connection show "$PROF" | grep -q yes \
  && ok "autoconnect=yes → açılışta kendiliğinden gelecek" \
  || ht "autoconnect kapalı — reboot'ta IP yine gitmiş olur"

say "NOT"
cat <<'SON'
   • Artık her oturumda 'sudo ip addr add ...' yazmak GEREKMEZ.
   • Doğrulama (reboot sonrası):  ip -br a   → enP8p1s0 üzerinde 192.168.117.60
   • Bu ağda GATEWAY yok, bilinçli: internet buradan gitmiyor, varsayılan
     rotayı bozmasın diye. (Jetson'da internet zaten yok — md 4.1.)
SON
