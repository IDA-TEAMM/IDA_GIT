#!/bin/bash
# GOL DERLEME — girdap_decision paketini ~/ros2_ws'e kurar.
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

mkdir -p "$WS/src"
[ -e "$WS/src/girdap_decision" ] || ln -s "$SRC" "$WS/src/girdap_decision"

# Bozuk onceki denemelerin artigini temizle (--editable/--uninstall tuzagi)
rm -rf "$WS/build/girdap_decision" "$WS/install/girdap_decision"

cd "$WS"
colcon build --packages-select girdap_decision 2>&1 | grep -vE "UserWarning|warnings.warn"

P="$WS/install/girdap_decision/share/girdap_decision/config/params.yaml"
[ -f "$P" ] && echo "✅ kurulum tamam: $P" || { echo "🔴 params.yaml olusmadi"; exit 1; }
