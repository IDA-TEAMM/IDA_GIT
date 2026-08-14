# GİRDAP — ROS ortamı (source ile kullan, çalıştırma)
#
#   source scripts/girdap_ortam.sh
#
# 🔴 NEDEN VAR (14.08.2026'da ölçüldü). Etkileşimsiz kabuklar
# (`ssh makine 'komut'`, systemd olmayan betikler, cron) `~/.bashrc`'yi
# baştan sona OKUMAZ: Ubuntu'nun varsayılan `.bashrc`'si 6-8. satırda
#     case $- in *i*) ;; *) return;; esac
# ile etkileşimsiz kabukta ERKEN DÖNER. `export ROS_DOMAIN_ID=42` ise
# 123. satırda, yani o `return`'den SONRA — hiç çalışmıyor.
#
# Sonucu bir hafta boyunca yanlış teşhis ettim: `ros2 node list` boş dönüyor,
# `ros2 topic hz` hiçbir şey göstermiyordu ve bunu "DDS ağ arayüzünü katılımcı
# oluşturulurken bağlıyor, ethernet değişince yeni süreçler kör kalıyor" diye
# yorumlamıştım. YANLIŞTI: CLI domain **0**'da koşuyordu, yığın ise 42'de.
# Aynı makinede oldukları için keşif zaten PAYLAŞIMLI BELLEK üzerinden
# yapılıyor (`/dev/shm/fastrtps_*` — ölçüldü, 5 segment) ve ağdan tamamen
# bağımsız. Discovery Server'a GEREK YOK.
#
# ⚠ Bu kusurun türü tanıdık: sessizce YANLIŞ-NEGATİF üretiyor. "Hiçbir şey
# görünmüyor" ile "hiçbir şey çalışmıyor" ayırt edilemez hale geliyor ve
# sahada "sistem çöktü" paniğine dönüşür — oysa yığın 10 Hz yayında olabilir.
# O yüzden bu betik domain'i SESSİZCE ayarlamakla yetinmiyor, EKRANA YAZIYOR.

export ROS_DOMAIN_ID=42                      # canlı sistem (girdap-*.service)

# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
[ -f "$HOME/ros2_ws/install/setup.bash" ] && source "$HOME/ros2_ws/install/setup.bash"

# prototype/* importları için (F2.3) — yalnız ayarlı değilse
if [ -z "${PYTHONPATH:-}" ]; then
    _girdap_kok="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
    export PYTHONPATH="$_girdap_kok"
    unset _girdap_kok
fi

echo "girdap ortam: ROS_DOMAIN_ID=$ROS_DOMAIN_ID · $(ros2 --version 2>/dev/null || echo ros2)"
echo "  ⚠ domain 42 DEGILSE hicbir topic gormezsin — 'yigin olmus' sanma."
