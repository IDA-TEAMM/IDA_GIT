#!/usr/bin/env bash
# GİRDAP İDA — Jetson kurulum scripti (Ubuntu 22.04 / JetPack 6 / ROS Humble)
# Sıfır Jetson'a algı tarafının tüm bağımlılıklarını DOĞRU sürümlerle kurar,
# workspace'i oluşturur ve derler. Güvenle tekrar koşulabilir (idempotent).
#
# Kullanım:
#   bash scripts/jetson_kur.sh            # kurulum + derleme
#   bash scripts/jetson_kur.sh --servis   # + açılışta otomatik başlatma (systemd)
#
# NOT: ROS 2 Humble'ın kendisini KURMAZ (büyük ve tek seferlik iş) — yoksa
# adımları yazdırıp çıkar. Karar stack'i (girdap-decision) da klonlanır;
# onun GTSAM/numpy bağımlılıkları kendi README'sinin işi.

set -euo pipefail

WS=~/ros2_ws
REPO_ALGI="https://github.com/EyupEker1/girdap-ida-algi.git"
# Karar yığını BİZİM FORK'tan: tüm denetim düzeltmeleri (F12.1, F14.x, T0-j
# GUIDED tetiği...) orada; vistastris/girdap-decision BAYAT kalır.
REPO_KARAR="https://github.com/EyupEker1/girdap-decision.git"

echo "== 1/6 ROS 2 Humble kontrolü =="
if [ ! -d /opt/ros/humble ]; then
    cat <<'EOF'
ROS 2 Humble kurulu değil. Önce şunları koş (resmi adımlar), sonra bu scripti tekrar çalıştır:
  sudo apt install software-properties-common curl -y && sudo add-apt-repository universe -y
  sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
  echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list
  sudo apt update && sudo apt install ros-humble-ros-base ros-dev-tools -y
EOF
    exit 1
fi
echo "   Humble var."
# ROS setup.bash tanımsız değişken okur (AMENT_TRACE_SETUP_FILES) — set -u
# ile çakışır; source süresince gevşet (Jetson'da 2026-07-11'de yaşandı).
set +u
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
set -u

echo "== 2/6 apt paketleri =="
sudo apt update
sudo apt install -y \
    ros-humble-tf2-ros ros-humble-tf2-geometry-msgs \
    ros-humble-vision-msgs \
    python3-pip python3-colcon-common-extensions git

echo "== 3/6 pip paketleri (sürüm kilitli) =="
# numpy<2 ŞART: 2.x, apt'ın scipy/matplotlib/ROS derlemelerinin ABI'sini kırar.
pip install --break-system-packages "numpy>=1.26,<2" "depthai>=3.6"

echo "== 4/6 OAK udev kuralı =="
if [ ! -f /etc/udev/rules.d/80-movidius.rules ]; then
    echo 'SUBSYSTEM=="usb", ATTRS{idVendor}=="03e7", MODE="0666"' \
        | sudo tee /etc/udev/rules.d/80-movidius.rules >/dev/null
    sudo udevadm control --reload-rules && sudo udevadm trigger
    echo "   Kural yazıldı — OAK takılıysa ÇIKAR-TAK."
else
    echo "   Kural zaten var."
fi

echo "== 5/6 workspace + derleme =="
mkdir -p "$WS/src"
cd "$WS/src"
# Repolar PRIVATE — klon başarısızsa önce GitHub kimliği kur:
# gh auth login (bkz. docs/jetson_kurulum_rehberi.md §1)
[ -d girdap-ida-algi ]   || git clone "$REPO_ALGI" || {
    echo "!! klon başarısız — repolar private, önce 'gh auth login' (rehber §1)"; exit 1; }
[ -d girdap-decision ]   || git clone "$REPO_KARAR" || {
    echo "!! klon başarısız — repolar private, önce 'gh auth login' (rehber §1)"; exit 1; }
cd "$WS"
colcon build --symlink-install --packages-select girdap_ida_algi || {
    echo "!! girdap_ida_algi derlenemedi"; exit 1; }
# Karar paketi de derlensin (varsa) — hata bizi durdurmasın, onun sorumluluğu
colcon build --symlink-install --packages-select girdap_decision || \
    echo "   (girdap_decision derlenemedi — arkadaşın README'sine bak, bizim taraf tamam)"
grep -q "ros2_ws/install/setup.bash" ~/.bashrc || \
    echo "source $WS/install/setup.bash" >> ~/.bashrc
# Karar yığını prototype/* importları için (girdap-decision F2.3):
grep -q "src/girdap-decision" ~/.bashrc || \
    echo 'export PYTHONPATH=$HOME/ros2_ws/src/girdap-decision:$PYTHONPATH' >> ~/.bashrc

echo "== 5b/6 WiFi/Bluetooth kapatma (şartname 4.1 — teknik kontrolde bakılır) =="
# 2.4-2.8 GHz tamamen yasak: dahili WiFi VE Bluetooth kapalı olacak.
# BİLEREK klon/derlemeden SONRA: kurulum WiFi üzerinden yapılıyorsa erken
# kapatmak scriptin kendi internetini keserdi. rfkill yeniden başlatmada
# systemd-rfkill ile kalıcıdır; yarışma sabahı jetson_kontrol.sh ile DOĞRULA.
sudo rfkill block wifi bluetooth 2>/dev/null || echo "   (rfkill yok/başarısız — elle kapat)"

echo "== 6/6 model dosyası hatırlatması =="
MODEL=$(grep -oP 'MODEL_NNARCHIVE\s*=\s*"\K[^"]+' \
    "$WS/src/girdap-ida-algi/girdap_ida_algi/girdap_ida_algi/duba_gecis_navigator.py")
if [ -f "$MODEL" ]; then
    echo "   NN Archive yerinde: $MODEL"
else
    echo "   !! NN Archive eksik: $MODEL — HubAI'den indirilen tar.xz'yi bu yola koy (git'te YOK, elle taşınır)."
fi

# --- opsiyonel: açılışta otomatik başlatma ---
if [ "${1:-}" = "--servis" ]; then
    echo "== systemd servisi kuruluyor =="
    sudo cp "$WS/src/girdap-ida-algi/scripts/girdap-algi.service" /etc/systemd/system/
    sudo sed -i "s|__USER__|$USER|g; s|__WS__|$WS|g" /etc/systemd/system/girdap-algi.service
    sudo systemctl daemon-reload
    sudo systemctl enable girdap-algi.service
    echo "   Etkin. Şimdi başlat: sudo systemctl start girdap-algi"
    echo "   Log izle:            journalctl -fu girdap-algi"
fi

echo
echo "KURULUM TAMAM. Doğrula: bash $WS/src/girdap-ida-algi/scripts/jetson_kontrol.sh"
