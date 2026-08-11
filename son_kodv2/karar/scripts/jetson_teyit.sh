#!/usr/bin/env bash
# GİRDAP İDA — JETSON FİİLİ TEYİT (11.08.2026)
#
# Kaptan: "onlar Jetson'da halledildi ama teyit edin."
# Bu betik HİÇBİR ŞEY DEĞİŞTİRMEZ — yalnız okur ve rapor basar. sudo GEREKMEZ.
#
#   bash jetson_teyit.sh
#
# Her satır ya ✓ (beklenen) ya ✗ (eksik/yanlış) ya ? (bakılamadı) döner.

ok(){ printf '  \033[32m✓\033[0m %s\n' "$*"; }
ht(){ printf '  \033[31m✗\033[0m %s\n' "$*"; }
uy(){ printf '  \033[33m?\033[0m %s\n' "$*"; }
bas(){ printf '\n\033[1;44m %s \033[0m\n' "$*"; }

bas "1/8 — SERVİSLER: kurulu + enabled mi"
for s in girdap-saat girdap-livox girdap-karar girdap-algi girdap-veriseti girdap-rosbag; do
  u=/etc/systemd/system/$s.service
  if [[ -f $u ]]; then
    e=$(systemctl is-enabled $s 2>&1); a=$(systemctl is-active $s 2>&1)
    printf '  %-18s kurulu  enabled=%-10s active=%s\n' "$s" "$e" "$a"
  else
    ht "$s.service KURULU DEĞİL"
  fi
done
echo "  ⚠ girdap-algi ile girdap-veriseti AYNI ANDA enabled OLMAMALI (boot yarışı, §0.33d)"

bas "2/8 — VERİ SETİ AYARLARI (kaptan: 1 sn/kare, ~/veri_seti, algı otomatik dönüş)"
V=/etc/systemd/system/girdap-veriseti.service
if [[ -f $V ]]; then
  grep -qE '\-\-interval[= ]+1(\.0)?\b' $V && ok "--interval 1.0" || ht "--interval 1.0 DEĞİL: $(grep -oE '\-\-interval[= ]+[0-9.]+' $V)"
  grep -qE '\-\-min-fark[= ]+0\b'      $V && ok "--min-fark 0 (filtre kapalı)" || ht "--min-fark 0 DEĞİL: $(grep -oE '\-\-min-fark[= ]+[0-9.]+' $V) → 1 sn = 1 kare GARANTİ DEĞİL"
  grep -qE '\-\-out[= ]+.*veri_seti'   $V && ok "--out ~/veri_seti" || ht "--out yanlış: $(grep -oE '\-\-out[= ]+[^ ]+' $V)"
  grep -qE '^OnSuccess=girdap-algi'    $V && ok "OnSuccess=girdap-algi.service" || ht "OnSuccess SATIRI YOK → toplayıcı kapanınca algı GERİ GELMEZ"
  grep -qE '^Conflicts=girdap-algi'    $V && ok "Conflicts=girdap-algi.service" || ht "Conflicts YOK → iki süreç tek OAK'ı kapar"
else
  uy "girdap-veriseti.service kurulu değil — veri seti toplama YOK"
fi

bas "3/8 — KARAR SERVİSİ: LiDAR bağımlılığı + overlay"
K=/etc/systemd/system/girdap-karar.service
if [[ -f $K ]]; then
  grep -qE '^(Wants|Requires)=.*girdap-livox' $K && ok "Wants/Requires=girdap-livox" \
    || ht "Wants=girdap-livox YOK → livox enable değilse karar SESSİZCE LiDAR'sız kalkar (§0.30e: kapı sayısı yapısal olarak 0)"
  grep -qE '^ExecStart' $K && printf '     ExecStart: %s\n' "$(grep -m1 '^ExecStart' $K | cut -c1-120)"
  grep -oE 'PYTHONPATH=[^ "]*' $K | head -2 | sed 's/^/     /'
else
  ht "girdap-karar.service KURULU DEĞİL"
fi
D=/etc/systemd/system/girdap-karar.service.d
if [[ -d $D ]]; then
  n=$(ls -1 $D/*.conf 2>/dev/null | wc -l)
  ls -1 $D/*.conf 2>/dev/null | sed 's/^/     drop-in: /'
  (( n > 1 )) && ht "BİRDEN FAZLA drop-in — yarisma.conf ile parkur1.conf AYNI ANDA KURULMAZ"
  grep -h 'GIRDAP_CONFIG_OVERLAY' $D/*.conf 2>/dev/null | sed 's/^/     /'
else
  uy "drop-in dizini yok → overlay YOK"
  echo "     ℹ 11.08 sonrası bu ARTIK KRİTİK DEĞİL: hardware.yaml yarışma tabanı oldu"
  echo "       (commit 08a9811). Overlay'siz kalkan yığın yarışma modunda koşar."
  echo "       parkur1.yaml hâlâ P1 görev etiketleri için anlamlı."
fi

bas "4/8 — KOD SÜRÜMÜ: yığın HANGİ kopyayı koşuyor"
S=~/ros2_ws/src/girdap_decision
if [[ -L $S ]]; then
  T=$(readlink -f $S); ok "symlink → $T"
  [[ $T == *son_kodv2* ]] && ok "son_kodv2 (DOĞRU)" || ht "son_kodv2 DEĞİL → ESKİ KODLA KOŞUYOR (e349826'nın yakaladığı arıza)"
elif [[ -d $S ]]; then uy "gerçek dizin (symlink değil): $S"
else ht "$S YOK"; fi
[[ -d ~/IDA_GIT ]] && { printf '     IDA_GIT: '; git -C ~/IDA_GIT log --oneline -1 2>/dev/null; \
  git -C ~/IDA_GIT status --short 2>/dev/null | head -3 | sed 's/^/     kirli: /'; } || uy "~/IDA_GIT yok"

bas "5/8 — PIXHAWK: udev symlink + fcu_url uyumu"
if [[ -e /dev/pixhawk ]]; then ok "/dev/pixhawk → $(readlink -f /dev/pixhawk)"
else ht "/dev/pixhawk YOK (99-girdap-fc.rules kurulu mu?) → ttyUSBn takas riski"; fi
ls -1 /dev/ttyUSB* 2>/dev/null | sed 's/^/     /' || uy "ttyUSB yok (Pixhawk bağlı değil)"
H=$(readlink -f $S 2>/dev/null)/../config/hardware.yaml
[[ -f $H ]] && printf '     fcu_url: %s\n' "$(grep -m1 '^fcu_url' $H)"
echo "     ⚠ fcu_url ttyUSB0 diyorsa ve iki FTDI varsa reboot'ta takas olur —"
echo "       MAVROS telemetri radyosuna bağlanır, Pixhawk'ı HİÇ bulamaz (belirti YOK)"

bas "6/8 — PYTHON ORTAMI (iSAM2/GTSAM için kritik)"
python3 - <<'PY' 2>/dev/null || uy "python3 sorgusu başarısız"
for m in ("numpy","gtsam","cupy","depthai","scipy","cv2"):
    try:
        mod=__import__(m); print(f"     {m:10s} {getattr(mod,'__version__','?')}")
    except Exception as e:
        print(f"     {m:10s} YOK ({type(e).__name__})")
PY
echo "     ⚠ numpy 2.x + gtsam = SEGFAULT (§0.8e). Beklenen: numpy 1.26.4 + cupy-cuda12x 13.6.0"
dpkg -l 2>/dev/null | grep -q rosbag2-storage-mcap && ok "rosbag2-storage-mcap kurulu" || ht "rosbag2-storage-mcap YOK → rosbag kaydı MCAP yazamaz"

bas "7/8 — SAAT (Dosya-1/2/3 zaman etiketleri buna bağlı)"
date '+     şu an: %F %T %Z'
timedatectl 2>/dev/null | grep -E "synchronized|RTC" | sed 's/^/     /'
echo "     ⚠ RTC pili yok → güç kesilince sıfırlanır; girdap-saat.service Pixhawk GPS'inden kuruyor"
journalctl -u girdap-saat -n 3 --no-pager 2>/dev/null | sed 's/^/     /'

bas "8/8 — ORTAMA DUYARLI İKİ TEST (yalnız INSTALL'lu ortamda koşar)"
# 🔑 NEDEN BURADA: bu iki test laptopta (paket install EDİLMEMİŞ) `pytest.skip`
# ile atlanıyor, Jetson'da (install'lu) KOŞUYOR — ve laptopta sahte ament share
# dizini kurularak yapılan benzetimde İKİSİ DE KIRMIZI verdi. Kırmızılık
# tabanda da var (kod kusuru değil): launch_ros `Node(parameters=[{...}])`
# anahtarlarını TextSubstitution'a çeviriyor, testler düz metin arıyor.
# ⚠ §0.28a'nın "727 geçti / 1 bilinen-bayat kırmızı" kaydıyla ÇELİŞEBİLİR —
# gerçek Jetson sonucu buradan okunacak.
KARAR="$(readlink -f "$EV/ros2_ws/src/girdap_decision" 2>/dev/null)"
KARAR="${KARAR%/ros2_ws/src/girdap_decision}"
if [[ -z "$KARAR" || ! -d "$KARAR/prototype/tests" ]]; then
  uy "karar kökü bulunamadı — testler koşulamadı"
elif ! command -v pytest >/dev/null && ! python3 -c "import pytest" 2>/dev/null; then
  uy "pytest yok — testler koşulamadı"
else
  ok "karar kökü: $KARAR"
  # ⚠ ${VAR:-...} İÇİNDE APOSTROF KULLANMA — bash onu tırnak açılışı sayar ve
  #   dosyanın geri kalanını yutar ("unexpected EOF"). Bu satır bir kez öyle yazıldı.
  echo "     AMENT_PREFIX_PATH: ${AMENT_PREFIX_PATH:-<bos - install ortami DEGIL>}"
  (
    cd "$KARAR" || exit 1
    # shellcheck disable=SC1091
    source /opt/ros/humble/setup.bash 2>/dev/null
    export PYTHONPATH="$KARAR:$KARAR/ros2_ws/src/girdap_decision:${PYTHONPATH:-}"
    python3 -c "import rclpy" 2>/dev/null || echo "     ⚠ rclpy YOK → süit yalancı olur"
    for T in test_isam2_launch_argumanlari_fusion_nodea_gecer \
             test_B0_montaj_parametreleri_lidar_nodeun_EN_SONUNDA; do
      SONUC=$(python3 -m pytest "prototype/tests/test_hardware_launch_config.py::$T" \
                -q --no-header 2>&1 | tail -3 | tr '\n' ' ')
      printf '     %-52s %s\n' "$T" "$SONUC"
    done
  )
  echo "     ℹ Tam süit için: §0.31g biçimi (izole numpy 1.26.4 + tam PYTHONPATH)"
fi

bas "TOPİK AKIŞI (yığın koşuyorken ayrıca koş)"
cat <<'SON'
     source /opt/ros/humble/setup.bash && export ROS_DOMAIN_ID=42
     ros2 topic hz /livox/lidar                      # ~10 Hz  — yoksa livox servisi
     ros2 topic hz /perception/buoys                 # algı düğümü (duba_gecis_navigator)
     ros2 topic hz /perception/classified_obstacles  # 🔑 İKİSİ birden akmazsa KAPI YOK
     ros2 topic hz /mavros/imu/data                  # ~8 Hz   — Pixhawk bağlı mı
     journalctl -u girdap-karar | grep -E "fusion_node aktif|config overlay"
       # "fusion_node aktif [iSAM2]"  <- iSAM2 açık
       # "[MAVROS EKF geçişi (video)]" <- iSAM2 KAPALI koşuyor
SON
