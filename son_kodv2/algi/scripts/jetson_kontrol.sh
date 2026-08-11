#!/usr/bin/env bash
# GİRDAP İDA — Jetson ortam kontrolü
# Tek komutla hedef yığını denetler (README "Hedef yığın" tablosu).
# Kullanım:  bash scripts/jetson_kontrol.sh
# Çıkış kodu: 0 = hepsi geçti, 1 = en az bir FAIL var.
# Çıktıyı olduğu gibi kopyalayıp paylaş — uyuşmazlık tek bakışta görülür.

set -u
HATA=0

yesil() { printf "\033[32m[GEÇTİ]\033[0m %s\n" "$1"; }
kirmizi() { printf "\033[31m[HATA ]\033[0m %s\n" "$1"; HATA=1; }
sari() { printf "\033[33m[UYARI]\033[0m %s\n" "$1"; }

echo "=== GİRDAP İDA Jetson kontrolü ($(date '+%F %T')) ==="

# --- 1) Donanım / L4T ---
if [ -f /etc/nv_tegra_release ]; then
    yesil "Jetson L4T: $(head -c 60 /etc/nv_tegra_release)"
else
    sari "Bu makine Jetson değil (nv_tegra_release yok) — PC'de test modunda olabilir"
fi

# --- 2) Ubuntu 22.04 ---
UBUNTU=$(. /etc/os-release && echo "$VERSION_ID")
if [ "$UBUNTU" = "22.04" ]; then
    yesil "Ubuntu $UBUNTU"
else
    kirmizi "Ubuntu $UBUNTU — hedef 22.04 (JetPack 6 / ROS Humble hattı)"
fi

# --- 3) Python 3.10 ---
PYV=$(python3 -c 'import sys; print(".".join(map(str, sys.version_info[:2])))')
if [ "$PYV" = "3.10" ]; then
    yesil "Python $PYV"
else
    kirmizi "Python $PYV — hedef 3.10 (ROS Humble buna derli)"
fi

# --- 4) ROS 2 Humble ---
if [ -d /opt/ros/humble ]; then
    yesil "ROS 2 Humble kurulu (/opt/ros/humble)"
    # set -u ile çakışır (AMENT_TRACE_SETUP_FILES) — source süresince gevşet
    set +u
    # shellcheck disable=SC1091
    source /opt/ros/humble/setup.bash
    set -u
else
    kirmizi "ROS 2 Humble YOK — bkz. README kurulum; Jazzy KURMA"
fi

# --- 5) numpy < 2 ---
NPV=$(python3 -c 'import numpy; print(numpy.__version__)' 2>/dev/null)
if [ -z "$NPV" ]; then
    kirmizi "numpy import edilemiyor"
elif [ "${NPV%%.*}" -ge 2 ]; then
    kirmizi "numpy $NPV — 2.x ROS/scipy ABI'sini kırar. Düzelt: pip install 'numpy>=1.26,<2' --break-system-packages"
else
    yesil "numpy $NPV (<2)"
fi

# --- 6) depthai == 2.30.0.0 (v2 API) ---
# 🔴 2026-08-07 DÜZELTMESİ: bu blok TERSİNİ kontrol ediyordu ("hedef >=3.6,
# v2 kodu bu repoda çalışmaz") — yani DOĞRU kurulumu KIRMIZI/hata raporluyor,
# kamerayı kıran v3'ü ise yeşil veriyordu. Gerçek: kod 05.08'de v2'ye taşındı
# ve v3 firmware'i bu OAK-D Lite'ta stereo üretmiyor (v3 %0 / v2 29,7 FPS).
# Her iki makinede de kurulu sürüm 2.30.0.0.
DAI=$(python3 -c 'import depthai; print(depthai.__version__)' 2>/dev/null)
if [ -z "$DAI" ]; then
    kirmizi "depthai YOK — pip install --user 'depthai==2.30.0.0'"
elif [ "$DAI" = "2.30.0.0" ]; then
    yesil "depthai $DAI (v2 API — doğru sürüm)"
elif python3 -c "import depthai as d; exit(0 if int(d.__version__.split('.')[0]) >= 3 else 1)" 2>/dev/null; then
    kirmizi "depthai $DAI — v3! STEREO ÜRETMEZ (P3 için stereo zorunlu) ve kod v2 API'sinde."
    kirmizi "  Düzelt: pip install --user 'depthai==2.30.0.0'"
else
    sari "depthai $DAI — beklenen 2.30.0.0; v2 ailesinde ama ölçümler bu sürümle yapılmadı"
fi

# --- 6b) algı servisi: kurulu/enabled mı, unit'te WorkingDirectory var mı ---
# WorkingDirectory eksikse boot'ta cwd=/ olur, depthai .cache'i yazamaz ve node
# HİÇ açılmaz — elle çalıştırınca görünmeyen, yalnız boot'ta çıkan arıza.
UNIT=/etc/systemd/system/girdap-algi.service
if [ -f "$UNIT" ]; then
    yesil "girdap-algi.service kurulu ($(systemctl is-enabled girdap-algi 2>/dev/null || echo '?') / $(systemctl is-active girdap-algi 2>/dev/null || echo '?'))"
    grep -q "^WorkingDirectory=" "$UNIT" \
        && yesil "  unit'te WorkingDirectory var" \
        || kirmizi "  unit'te WorkingDirectory YOK → boot'ta depthai '/.cache' yazamaz, node açılmaz. Yeniden kur: jetson_kur.sh --servis"
    if [ "$(systemctl is-enabled girdap-veriseti 2>/dev/null)" = "enabled" ] \
       && [ "$(systemctl is-enabled girdap-algi 2>/dev/null)" = "enabled" ]; then
        kirmizi "  İKİSİ DE enabled — tek OAK var, boot'ta yarışırlar. Yarışma günü: sudo systemctl disable --now girdap-veriseti (md 4.1)"
    fi
else
    sari "girdap-algi.service kurulu değil — açılışta algı başlamaz (kur: jetson_kur.sh --servis)"
fi

# --- 7) ROS mesaj paketleri ---
for PKG in vision_msgs tf2_ros nav_msgs; do
    if python3 -c "import $PKG" 2>/dev/null; then
        yesil "ros paketi: $PKG"
    else
        kirmizi "ros paketi eksik: $PKG — sudo apt install ros-humble-${PKG//_/-}"
    fi
done

# --- 8) OAK-D Lite bağlı mı ---
if lsusb 2>/dev/null | grep -qiE "03e7|Movidius|Luxonis"; then
    yesil "OAK cihazı USB'de görünüyor"
    if python3 -c "import depthai as d; devs=d.Device.getAllAvailableDevices(); exit(0 if devs else 1)" 2>/dev/null; then
        yesil "depthai cihazı görüyor (udev tamam)"
    else
        sari "USB'de var ama depthai göremiyor — udev kuralı için jetson_kur.sh koş, kabloyu çıkar-tak"
    fi
else
    sari "OAK USB'de görünmüyor (takılı değilse normal)"
fi

# --- 8b) WiFi/BT kapalı mı (şartname 4.1 — 2.4-2.8 GHz yasak) ---
if command -v rfkill >/dev/null; then
    if rfkill list wifi 2>/dev/null | grep -q "Soft blocked: yes"; then
        yesil "WiFi rfkill ile kapalı"
    else
        kirmizi "WiFi AÇIK görünüyor — şartname 4.1: sudo rfkill block wifi bluetooth"
    fi
else
    sari "rfkill yok — WiFi durumunu elle doğrula (teknik kontrol maddesi)"
fi

# --- 9) Model dosyası (yol koddaki MODEL_BLOB'dan okunur) ---
# 🔴 2026-08-11 DÜZELTMESİ: burası `MODEL_NNARCHIVE` arıyordu — o değişken
#    depthai v3 döneminden kalma ve canlı kodda ARTIK YOK (yalnız eski
#    build/lib/ kopyasında duruyor). Sonuç: model DOĞRU yerdeyken bile bu
#    madde her seferinde KIRMIZI basıyordu. Yanlış alarm, kontrol betiğinin
#    kırmızısını görmezden gelmeyi öğretir — asıl tehlike bu.
KOD="$(dirname "$0")/../girdap_ida_algi/girdap_ida_algi/duba_gecis_navigator.py"
MODEL=$(grep -oP 'MODEL_BLOB\s*=\s*"\K[^"]+' "$KOD" 2>/dev/null || true)
if [ -z "$MODEL" ]; then
    kirmizi "MODEL_BLOB kodda okunamadı — $KOD değişti mi?"
elif [ ! -f "$MODEL" ]; then
    kirmizi "Blob YOK: '$MODEL' — kopyala: cp models/yolo11n_duba_rvc2.blob models/config.json $(dirname "$MODEL")/"
else
    yesil "Blob yerinde: $MODEL"
    # config.json blob'un YANINDA olmak ZORUNDA: sınıflar oradan İSİMLE
    # çözülüyor. Yoksa yedek sabitlere düşer ve turuncu↔sarı SESSİZCE takas
    # olabilir (eğitilmiş modelin sırası yedek sabitlerin TERSİ) ⇒ P2 çöker.
    if [ -f "$(dirname "$MODEL")/config.json" ]; then
        yesil "config.json blob'un yanında"
    else
        kirmizi "config.json YOK ($(dirname "$MODEL")/) — sınıflar yedek sabitlere düşer, turuncu↔sarı takas riski (⇒ P2)"
    fi
    # 🔴 Yol KODDA SABİT (/home/girdap/...). Yeni kart farklı kullanıcı adıyla
    #    kurulursa node modeli bulamaz ve HİÇ açılmaz.
    case "$MODEL" in
        "$HOME"/*) : ;;
        *) sari "Model yolu bu kullanıcının ev dizininde değil (MODEL_BLOB=$MODEL, HOME=$HOME) — kod bu yolu SABİT tutuyor, kullanıcı adı 'girdap' değilse düzelt" ;;
    esac
fi

echo "=================================================="
if [ "$HATA" -eq 0 ]; then
    echo "SONUÇ: ortam HAZIR ✓"
else
    echo "SONUÇ: eksikler var — yukarıdaki [HATA] satırlarını düzelt (jetson_kur.sh çoğunu halleder)"
fi
exit "$HATA"
