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
# adımları yazdırıp çıkar.
# 🔴 2026-08-10: karar stack'i ARTIK KLONLANMIYOR (bkz. 5/6). Klonlanan repo
# BAYATTI ve içinde gate_follower.py YOKTU ⇒ kurulsaydı P1/P2 sessizce giderdi.
# Karar yığınının kurulumu sahibine ait; biz yalnız var mı/sağlam mı diye bakarız.

set -euo pipefail

WS=~/ros2_ws
REPO_ALGI="https://github.com/EyupEker1/girdap-ida-algi.git"
# Karar yığını BİZİM FORK'tan: tüm denetim düzeltmeleri (F12.1, F14.x, T0-j
# GUIDED tetiği...) orada; vistastris/girdap-decision BAYAT kalır.
# 🔴 KULLANILMIYOR (2026-08-10) — bu repo 17.07'de donmuş; canlı karar kodu
# IDA-TEAMM/IDA_GIT → son_kodv2/karar. Silinmedi ki "neden yok" sorusu
# cevapsız kalmasın. Klonlama kaldırıldı, yerine sağlamlık kontrolü kondu.
REPO_KARAR_BAYAT="https://github.com/EyupEker1/girdap-decision.git"

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
pip install --break-system-packages "numpy>=1.26,<2"

# 🔴 depthai SÜRÜMÜ PAZARLIKSIZ 2.30.0.0 — bu satır 2026-08-07'ye kadar
# "depthai>=3.6" diyordu ve betiği koşan herkesin kamerasını sessizce kırardı:
# v3 firmware'i bu OAK-D Lite'ta STEREO'yu üretmiyor (05.08 ölçümü: v3'te
# stereo %0 / mono ~%25-33, v2.30'da stereo 29,7 FPS ve 5/5 tekrarlanabilir).
# Kodun tamamı 05.08'de v2 API'sine taşındı (0efa49c/3d18ac8/17d08e1) →
# v3'e çıkmak hem stereo'yu hem kodu kırar. YOLO26 için v3.6+ gerekiyor ama
# o yol bilerek kapatıldı (menzil stereo'dan geliyor, P3 için stereo zorunlu).
DAI_HEDEF="2.30.0.0"
DAI_VAR=$(python3 -c 'import depthai; print(depthai.__version__)' 2>/dev/null || echo "yok")
if [ "$DAI_VAR" = "$DAI_HEDEF" ]; then
    echo "   depthai $DAI_VAR — doğru sürüm, DOKUNULMUYOR."
else
    echo "   depthai '$DAI_VAR' → $DAI_HEDEF kuruluyor…"
    # --user: Jetson'a 05.08'de böyle kuruldu (~/.local, sudo gerekmedi).
    # --user kurulumu sistem kopyasını GÖLGELER; ikisi karışmasın diye aynı
    # yol korunuyor. Kurulum sonrası doğrulama aşağıda (sessiz başarısızlık yok).
    pip install --user "depthai==$DAI_HEDEF"
    DAI_SON=$(python3 -c 'import depthai; print(depthai.__version__)' 2>/dev/null || echo "yok")
    [ "$DAI_SON" = "$DAI_HEDEF" ] || {
        echo "!! depthai hâlâ '$DAI_SON' — beklenen $DAI_HEDEF."
        echo "   Muhtemel sebep: sistem geneli ikinci bir kopya gölgeliyor."
        echo "   Bak: python3 -c 'import depthai; print(depthai.__file__)'"
        exit 1; }
    echo "   depthai $DAI_SON ✓"
fi

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
# 🔴🔴 2026-08-10 DÜZELTMESİ — KARAR REPOSU ARTIK KLONLANMIYOR.
# Eskiden burada `git clone $REPO_KARAR` vardı ve BAYAT kodu kuruyordu:
#     EyupEker1/girdap-decision : son push 2026-07-17 · 136 dosya
#     IDA_GIT/son_kodv2/karar   : CANLI            · 220 dosya
#   fsm_node 18.572 B ↔ 29.380 B · fusion 9.709 B ↔ 19.522 B
#   sync_queue_size YOK · 🔴 gate_follower.py HİÇ YOK (404)
# gate_follower olmadan kapı orta noktası ÜRETİLMEZ ⇒ bizim tespitlerimiz akar
# ama kimse kullanamaz ⇒ P1 (G1/KD1≥0,5) ve P2 (≥2 ikili) SIFIR — hata basılmadan.
# Karar yığınının kurulumu SAHİBİNE aittir (kapsam kuralı); biz yalnız kontrol
# edip uyarıyoruz. Beklenen konum karar servisinin kendi PYTHONPATH'i:
#   Environment=PYTHONPATH=<ws>/src/girdap-decision
if [ -d girdap-decision ]; then
    _gf="girdap-decision/prototype/mission/gate_follower.py"
    if [ -f "$_gf" ]; then
        echo "   girdap-decision var ve gate_follower.py YERİNDE ✓"
    else
        echo "   !! girdap-decision var ama gate_follower.py YOK —"
        echo "      BAYAT kopya olabilir (EyupEker1/girdap-decision, 17.07)."
        echo "      Doğrusu: IDA-TEAMM/IDA_GIT → son_kodv2/karar içeriği."
        echo "      Bu hâliyle kapı takibi ÇALIŞMAZ ⇒ P1/P2 gider. Karar ekibine sor."
    fi
else
    echo "   (girdap-decision yok — karar yığınını sahibi kuracak, biz klonlamıyoruz)"
fi
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
# 🔴 2026-08-07: bu blok `MODEL_NNARCHIVE` arıyordu ama kod 05.08'de v2'ye
# taşınınca değişken `MODEL_BLOB` oldu → grep hiç eşleşmiyor → `set -euo
# pipefail` altında komut ikamesi exit 1 veriyor → BETİK TAM BURADA ÖLÜYORDU
# ve aşağıdaki `--servis` bloğuna HİÇ ULAŞILMIYORDU. Yani "servisi kur" komutu
# sessizce servisi kurmuyordu. `|| true` bunu bir daha yaşatmaz.
MODEL=$(grep -oP 'MODEL_BLOB\s*=\s*"\K[^"]+' \
    "$WS/src/girdap-ida-algi/girdap_ida_algi/girdap_ida_algi/duba_gecis_navigator.py" || true)
if [ -z "$MODEL" ]; then
    echo "   !! MODEL_BLOB satırı bulunamadı — kod değişmiş olabilir, elle bak."
elif [ -f "$MODEL" ]; then
    echo "   Model yerinde: $MODEL"
    # Blob'un shave sayısı YÜKLEMEDEN doğrulanabilir (v2.30'da header'dan okur,
    # cihaz gerekmez). Fazla shave = model HİÇ yüklenmez, sahada telafisi yok.
    python3 - "$MODEL" <<'PY' || true
import sys
try:
    import depthai as dai
    b = dai.OpenVINO.Blob(sys.argv[1])
    ok = "OK" if b.numShaves <= 4 else "!! FAZLA — cihaz REDDEDER"
    print(f"   blob numShaves={b.numShaves} (bütçe 4) {ok}")
except Exception as e:
    print(f"   (blob okunamadı: {e})")
PY
    [ -f "$(dirname "$MODEL")/config.json" ] \
        && echo "   config.json yanında ✓ (sınıf isimleri buradan okunuyor)" \
        || echo "   !! config.json YOK — sınıf çözümü yedek sabite düşer (kenar=0/engel=1)"
else
    echo "   Model hedefte yok: $MODEL"
    # 🔴 2026-08-10 DÜZELTMESİ: burada "git'te YOK, elle taşınır" yazıyordu —
    # ARTIK BAYAT. `cb1773d` ile blob + config.json REPOYA girdi (.gitignore
    # istisnası; gerekçe: yarışma alanında internet YOK (md 4.1) ve blobconverter
    # bulut ⇒ blob sahada üretilemez). Kod yolu sabit olduğu için tek eksik
    # KOPYALAMA adımıydı ve o adım insan hafızasına bırakılmıştı.
    REPO_MODELS="$WS/src/girdap-ida-algi/models"
    if [ -f "$REPO_MODELS/yolo11n_duba_rvc2.blob" ]; then
        echo "      Blob REPODA var → $(dirname "$MODEL")/ altına kopyalanıyor"
        mkdir -p "$(dirname "$MODEL")"
        cp "$REPO_MODELS/yolo11n_duba_rvc2.blob" "$REPO_MODELS/config.json" \
           "$(dirname "$MODEL")/" && echo "      ✓ kopyalandı"
        echo "      sha256: $(sha256sum "$MODEL" 2>/dev/null | cut -c1-16)…  (beklenen 5726819a101eb4f6…)"
    else
        echo "      !! Repoda da yok. Gereken: DÜZ .blob (≤4 shave) + yanında config.json."
        echo "      NNArchive (.tar.xz) v2.30'da YÜKLENMEZ: 'tar -xJf' ile içinden blob çıkar."
    fi
fi

# --- opsiyonel: VERİ TOPLAMA aşaması (sıfırlanmış Jetson, deniz/göl oturumu) ---
# 🔴 Tek OAK var: algı ile toplayıcı AYNI ANDA açılamaz. Bu bayrak bilerek
#    algıyı DEVRE DIŞI bırakır — "ikisi de enabled" hâli boot'ta yarış demektir
#    ve hangisinin kazandığı belirsizdir (unit'lerdeki Conflicts= yalnız İKİSİ
#    de kuruluysa işler). Yarışma günü geri alınır: --servis
if [ "${1:-}" = "--veriseti-servis" ]; then
    echo "== VERİ TOPLAMA servisi kuruluyor =="
    sudo cp "$WS/src/girdap-ida-algi/scripts/girdap-veriseti.service" /etc/systemd/system/
    sudo sed -i "s|__USER__|$USER|g; s|__WS__|$WS|g" /etc/systemd/system/girdap-veriseti.service
    sudo systemctl daemon-reload
    # Algıyı AÇIKÇA kapat (kuruluysa). `|| true`: hiç kurulu değilse hata verme.
    sudo systemctl disable --now girdap-algi.service 2>/dev/null || true
    sudo systemctl enable girdap-veriseti.service
    echo "   Etkin. Başlat: sudo systemctl start girdap-veriseti"
    echo "   Log izle:     journalctl -fu girdap-veriseti"
    echo "   🔴 DENİZE GİRMEDEN reboot testi ŞART:"
    echo "      sudo reboot  →  systemctl is-active girdap-veriseti  →  ls ~/girdap_veriseti/images | wc -l"
    echo "   🔴 Yarışma günü geri al: bash $0 --servis"
fi

# --- opsiyonel: açılışta otomatik başlatma (YARIŞMA/ALGI) ---
if [ "${1:-}" = "--servis" ]; then
    echo "== systemd servisi kuruluyor =="
    sudo cp "$WS/src/girdap-ida-algi/scripts/girdap-algi.service" /etc/systemd/system/
    sudo sed -i "s|__USER__|$USER|g; s|__WS__|$WS|g" /etc/systemd/system/girdap-algi.service
    sudo systemctl daemon-reload
    # 🔴 md 4.1 + tek OAK: toplayıcı yarışma günü ENABLED KALMAMALI.
    sudo systemctl disable --now girdap-veriseti.service 2>/dev/null || true
    sudo systemctl enable girdap-algi.service
    echo "   Etkin. Şimdi başlat: sudo systemctl start girdap-algi"
    echo "   (girdap-veriseti devre dışı bırakıldı — tek OAK, ikisi birden açılamaz)"
    echo "   Log izle:            journalctl -fu girdap-algi"
fi

echo
echo "KURULUM TAMAM. Doğrula: bash $WS/src/girdap-ida-algi/scripts/jetson_kontrol.sh"
