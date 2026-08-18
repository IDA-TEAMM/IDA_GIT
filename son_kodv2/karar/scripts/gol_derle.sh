#!/bin/bash
# GOL DERLEME — girdap_decision + girdap_ida_algi paketlerini ~/ros2_ws'e kurar.
#
# 🔵 18.08: ALGI paketi de eklendi. Sebep olculdu: golde `/perception/buoys`
# YAYINCI 0 idi -> dogrulama kurallari S1/S2/S5 ve C3'un bir kolu STALE
# ("veri yok"), yani IHLAL YOK degil HIC OLCULMEMIS. O topic'in tek uretecisi
# `duba_gecis_navigator` (girdap_ida_algi) ve paket kurulu olmadigi icin
# `ros2 run` onu bulamiyordu.
#
# 🔴 NEDEN AYRI BETIK (18.08.2026): gol_kos.sh `$HOME/ros2_ws/install/...`
# bekliyor ama orasi bos olabilir (paket hic derlenmemis). O halde tum zincir
# "poz HIC gelmedi" der ve SEBEP GIZLENIR — 18.08'de tam bu oldu.
#
# 🪤 `--symlink-install` KULLANMA. setuptools 60+ (bu makinede 81.0.0)
# `setup.py develop --editable` ve `--uninstall` bayraklarini KALDIRDI:
#     error: option --editable not recognized
#     error: option --uninstall not recognized
# Bir kez denenirse build/ dizininde artik kalir ve SONRAKI DUZ build'ler de
# ayni hatayi verir. Bu betik onu otomatik temizler.
#
# ⚠ Symlink olmadigi icin KAYNAK DEGISINCE YENIDEN DERLEMEK SART
# (kural 7'nin install/ ↔ src/ ayrismasi burada da gecerli).
set -e
source /opt/ros/humble/setup.bash
WS="$HOME/ros2_ws"
SRC="$HOME/IDA_GIT/son_kodv2/karar/ros2_ws/src/girdap_decision"
# ⚠ KURAL 7: algi paketinin KAYNAGI `girdap-ida-algi`, `son_kodv2/algi` onun
# AYNASI. Golde aynadan derliyoruz cunku olculen sey ekibin ortak alanindaki
# kod olmali (ayna bayatsa gol bunu gostersin, gizlemesin).
SRC_ALGI="$HOME/IDA_GIT/son_kodv2/algi/girdap_ida_algi"

mkdir -p "$WS/src"
[ -e "$WS/src/girdap_decision" ] || ln -s "$SRC" "$WS/src/girdap_decision"
[ -e "$WS/src/girdap_ida_algi" ] || ln -s "$SRC_ALGI" "$WS/src/girdap_ida_algi"

# Bozuk onceki denemelerin artigini temizle (--editable/--uninstall tuzagi)
rm -rf "$WS/build/girdap_decision" "$WS/install/girdap_decision"
rm -rf "$WS/build/girdap_ida_algi" "$WS/install/girdap_ida_algi"

cd "$WS"
colcon build --packages-select girdap_decision girdap_ida_algi 2>&1 \
    | grep -vE "UserWarning|warnings.warn"

P="$WS/install/girdap_decision/share/girdap_decision/config/params.yaml"
[ -f "$P" ] && echo "✅ karar kurulumu tamam: $P" || { echo "🔴 params.yaml olusmadi"; exit 1; }

# Algi tarafinin kanit dosyasi: `ros2 run`in bulacagi calistirilabilir.
A="$WS/install/girdap_ida_algi/lib/girdap_ida_algi/duba_gecis_navigator"
[ -x "$A" ] && echo "✅ algi kurulumu tamam: $A" \
             || { echo "🔴 duba_gecis_navigator kurulmadi"; exit 1; }
