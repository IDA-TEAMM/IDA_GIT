#!/usr/bin/env bash
# ═══════════════════════════════════════════════════════════════════════════
# GİRDAP İDA — GÖL KOŞUMU ÖN DENETİMİ ("tekneyi suya indirmeden önce")
#
# 🔴 NEDEN VAR: göl oturumları pahalı ve defterde SESSİZ arızalara kaybedilmiş
#   oturumlar var. Bu betiğin her maddesi GERÇEK bir olaydan geliyor:
#
#     §0.99  `/dev/pixhawk` ilk saniyeden yoktu → 78 dakikalık bant TAMAMEN KÖR
#     §1.29  kamera 16:03'te kesildi, node çökme döngüsünde → 3,7 saat "algı akmadı"
#     §1.41a cihaz 2 commit gerideydi → iş yapılmıştı ama TEKNEYE İNMEMİŞTİ
#     §1.25  algı ↔ karar ayrı keşif dünyalarında → topic'ler var, kimse duymuyor
#     §1.20g EKF `waiting for GPS config data` → GUIDED istekleri 25 kez reddedildi
#     §1.05  RC10 e-stop HIGH sabit → 10 dk komut, tekne 0,54 m kımıldadı
#     §1.42  girdap-ff-ayar boot'ta öldü, `Restart=on-failure` diriltmedi
#     §1.43  MAVROS param tablosu inmeden araçlar "FC'de yok" sanıp iptal etti
#     §1.43i ARMING_CHECK=0 YEDİ ayrı arm engelleyicisini birden susturuyor
#     saat   Jetson'da RTC yok; saat yanlışsa bütün log korelasyonu çöker
#
# 🔑 ORTAK DERS: bunların hiçbiri hata vermedi. Hepsi "çalışıyor gibi" görünüp
#   sessizce ölmüştü. Bu betik onları SUYA GİRMEDEN sorar.
#
# KULLANIM (göle varınca, tekneyi indirmeden önce):
#   bash ~/IDA_GIT/son_kodv2/karar/scripts/gol_hazir_mi.sh | tee ~/gol_hazir.txt
#
# HİÇBİR ŞEY DEĞİŞTİRMEZ — yalnız okur. sudo GEREKMEZ.
# Çıkış kodu: 0 = tek ✗ yok · 1 = en az bir ✗ var.
# ═══════════════════════════════════════════════════════════════════════════
# ⚠️ `set -u` KULLANMA: ROS'un setup.bash'i tanımsız değişkenlere dokunuyor
# ve betiği ilk source'ta sessizce öldürüyor (çıkış 1, TEK SATIR çıktı yok).
# Bu betiğin işi sessiz arızaları yakalamak; kendisi sessizce ölemez.
HATA=0
ok(){ printf '  \033[32m✓\033[0m %s\n' "$*"; }
ht(){ printf '  \033[31m✗\033[0m %s\n' "$*"; HATA=1; }
uy(){ printf '  \033[33m?\033[0m %s\n' "$*"; }
nt(){ printf '      \033[2m%s\033[0m\n' "$*"; }
bas(){ printf '\n\033[1;44m %s \033[0m\n' "$*"; }

export ROS_DOMAIN_ID=42 ROS_LOCALHOST_ONLY=1
source /opt/ros/humble/setup.bash 2>/dev/null || true
source "$HOME/ros2_ws/install/setup.bash" 2>/dev/null || true

# ── 0 ──────────────────────────────────────────────────────────────────────
bas "0/9 — SAAT (yanlışsa bütün log korelasyonu çöker)"
if timedatectl 2>/dev/null | grep -q "System clock synchronized: yes"; then
    ok "saat senkron — $(date '+%Y-%m-%d %H:%M:%S %z')"
else
    ht "saat SENKRON DEĞİL — $(date '+%Y-%m-%d %H:%M:%S')"
    nt "Jetson'da RTC pili yok. girdap-saat'e bak; yanlış saatle çekilen bant"
    nt "ile FC logu ÜST ÜSTE BİNMEZ, teşhis imkânsızlaşır."
fi
YIL=$(date +%Y)
[ "$YIL" -lt 2026 ] && ht "saat $YIL yılında — boot'ta saat kurulmamış (session_SAATSIZ_* üretir)"

# ── 1 ──────────────────────────────────────────────────────────────────────
bas "1/9 — DAĞITIM TAZE Mİ (§1.41a: iş yapıldı ama tekneye inmedi)"
for D in "$HOME/IDA_GIT" "$HOME/ros2_ws/src/girdap_ida_algi"; do
    [ -d "$D/.git" ] || continue
    AD=$(basename "$D")
    timeout 15 git -C "$D" fetch --quiet 2>/dev/null \
        || uy "$AD: origin'e ulaşılamadı — GERİDE Mİ BİLİNMİYOR (sahada ağ yok, normal)"
    GERI=$(git -C "$D" rev-list --count HEAD..@{u} 2>/dev/null || echo "?")
    KIRLI=$(git -C "$D" status --porcelain 2>/dev/null | wc -l)
    if [ "$GERI" = "0" ]; then ok "$AD güncel ($(git -C "$D" rev-parse --short HEAD))"
    elif [ "$GERI" = "?" ]; then uy "$AD: uzak dal karşılaştırılamadı ($(git -C "$D" rev-parse --short HEAD))"
    else ht "$AD $GERI commit GERİDE — cihaza inmemiş iş var"; fi
    [ "$KIRLI" -gt 0 ] && uy "$AD: $KIRLI dosya commit'lenmemiş (kasıtlı mı?)"
done

# ── 2 ──────────────────────────────────────────────────────────────────────
bas "2/9 — SERVİSLER (§1.42: sessizce ölen servis 'enabled' görünür)"
for S in girdap-saat girdap-karar girdap-algi girdap-rosbag girdap-livox; do
    A=$(systemctl is-active "$S" 2>/dev/null); A=${A:-yok}
    R=$(systemctl show "$S" -p NRestarts --value 2>/dev/null || echo 0)
    if [ "$A" = "active" ]; then
        if [ "${R:-0}" -gt 2 ]; then ht "$S ayakta ama $R KEZ yeniden başlamış — çökme döngüsü"
        else ok "$S ayakta (restart $R)"; fi
    else
        ht "$S $A"
    fi
done

# ── 3 ──────────────────────────────────────────────────────────────────────
bas "3/9 — DONANIM (§0.99: /dev/pixhawk yoktu, 78 dk kör bant)"
if [ -e /dev/pixhawk ]; then ok "/dev/pixhawk → $(readlink -f /dev/pixhawk)"
else ht "/dev/pixhawk YOK — uçuş kontrolcüsü bağlı değil, bant KÖR olur"; fi
if lsusb 2>/dev/null | grep -qiE "movidius|luxonis|03e7"; then ok "OAK kamera USB'de görünüyor"
else ht "OAK kamera USB'de YOK (§1.29: kamera kesilince 3,7 saat kaybedildi)"; fi
if ping -c1 -W2 192.168.117.100 >/dev/null 2>&1; then ok "Livox LiDAR ağda (192.168.117.100)"
else ht "Livox LiDAR ağda YOK — P2 LiDAR'sız imkânsız"; fi

# ── 4 + 5 + 6 ──────────────────────────────────────────────────────────────
python3 - <<'PY'
import os, sys, time, subprocess
os.environ.setdefault("ROS_DOMAIN_ID", "42")
os.environ.setdefault("ROS_LOCALHOST_ONLY", "1")
G="\033[32m✓\033[0m"; K="\033[31m✗\033[0m"; S="\033[33m?\033[0m"
def bas(t): print(f"\n\033[1;44m {t} \033[0m")
def nt(t): print(f"      \033[2m{t}\033[0m")
hata = 0
try:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
    from mavros_msgs.msg import State, StatusText
    from sensor_msgs.msg import NavSatFix
    from std_msgs.msg import Float64
except Exception as e:
    print(f"  {K} ROS ortamı yüklenemedi: {e}")
    sys.exit(3)

IZLE = {   # topic -> (asgari Hz, neden önemli)
    "/mavros/state":                 (0.5, "FC bağlantısı"),
    "/mavros/global_position/raw/fix":(0.5, "GPS"),
    "/mavros/local_position/pose":   (2.0, "poz — bu ölürse itki sıfırlanır (F-P.1)"),
    "/perception/buoys":             (1.0, "algı — §1.29'da 3,7 saat akmadı"),
    "/livox/lidar":                  (5.0, "LiDAR ham bulut"),
    "/girdap/fusion/pose":           (2.0, "füzyon pozu — planlamanın girdisi"),
}
rclpy.init()
d = Node("gol_hazir_mi")
qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                 history=HistoryPolicy.KEEP_LAST, depth=10)
sayac = {t: 0 for t in IZLE}
durum = {"state": None, "fix": None, "hdg": None, "metin": []}

# 🔴 KEŞİF YARIŞI: `get_topic_names_and_types()` düğüm açılır açılmaz ÇAĞRILIRSA
# eksik liste döner ve var olan topic'e "HİÇ YOK" denir (ilk koşumda tam bunu
# yaptı: /mavros/local_position/pose 1 yayıncı 1 aboneyle ayaktaydı). Bu betiğin
# işi yanlış alarm üretmemek — keşfin oturmasını BEKLE.
_kesif = time.monotonic() + 3.0
while time.monotonic() < _kesif:
    rclpy.spin_once(d, timeout_sec=0.05)

def say(t):
    def f(_m): sayac[t] += 1
    return f
tipler = dict(d.get_topic_names_and_types())
kendi_abonem = set()
for t in IZLE:
    tl = tipler.get(t)
    if not tl:
        continue
    try:
        from rosidl_runtime_py.utilities import get_message as gm
        d.create_subscription(gm(tl[0]), t, say(t), qos)
        kendi_abonem.add(t)
    except Exception:
        pass
d.create_subscription(State, "/mavros/state",
                      lambda m: durum.__setitem__("state", m), qos)
d.create_subscription(NavSatFix, "/mavros/global_position/raw/fix",
                      lambda m: durum.__setitem__("fix", m), qos)
d.create_subscription(StatusText, "/mavros/statustext/recv",
                      lambda m: durum["metin"].append(m.text), qos)

SURE = 8.0
son = time.monotonic() + SURE
while time.monotonic() < son:
    rclpy.spin_once(d, timeout_sec=0.05)

bas("4/9 — VERİ AKIYOR MU (%.0f sn ölçüm)" % SURE)
for t, (asg, neden) in IZLE.items():
    if t not in tipler:
        print(f"  {K} {t} — topic HİÇ YOK  ({neden})"); hata = 1; continue
    hz = sayac[t] / SURE
    if hz >= asg: print(f"  {G} {t:34s} {hz:6.1f} Hz")
    else:
        print(f"  {K} {t:34s} {hz:6.1f} Hz  (gereken ≥{asg})  ← {neden}")
        hata = 1

bas("5/9 — KEŞİF: ALGI ↔ KARAR AYNI DÜNYADA MI (§1.25)")
for t in ("/perception/buoys", "/girdap/fusion/pose"):
    try:
        y = d.count_publishers(t); a = d.count_subscribers(t)
    except Exception:
        continue
    # kendi aboneliğimizi düş — YALNIZCA gerçekten abone olduysak
    if t in kendi_abonem:
        a = max(0, a - 1)
    if y >= 1 and a >= 1: print(f"  {G} {t}  yayıncı={y} abone={a}")
    elif y >= 1: print(f"  {K} {t}  yayıncı={y} ama ABONE YOK — "
                       "topic akıyor, kimse dinlemiyor"); hata = 1
    else: print(f"  {K} {t}  YAYINCI YOK"); hata = 1

bas("6/9 — UÇUŞ KONTROLCÜSÜ")
st = durum["state"]
if st is None:
    print(f"  {K} /mavros/state gelmedi — MAVROS/FC bağlı değil"); hata = 1
else:
    print(f"  {G if st.connected else K} FC bağlantısı: {st.connected}")
    if not st.connected: hata = 1
    print(f"  {G} mod={st.mode}  armed={st.armed}")
fx = durum["fix"]
if fx is None:
    print(f"  {K} GPS mesajı gelmedi"); hata = 1
else:
    ad = {-1: "FIX YOK", 0: "tek nokta", 1: "SBAS", 2: "RTK"}.get(fx.status.status, "?")
    if fx.status.status < 0:
        print(f"  {K} GPS: {ad} — EKF kilitlenmez, GUIDED reddedilir (§1.20g)"); hata = 1
    elif fx.status.status < 2:
        print(f"  {S} GPS: {ad} — RTK yok, konum hassasiyeti düşük")
    else:
        print(f"  {G} GPS: {ad}")

bas("7/9 — ARM ENGELLEYİCİLERİ (§1.43i: ARMING_CHECK=0 yedisini birden susturur)")
pre = sorted({m for m in durum["metin"] if "PreArm" in m or m.startswith("Arm:")})
estop = [m for m in durum["metin"] if "MotorEStop" in m]
if pre:
    for m in pre:
        print(f"  {K} {m}"); hata = 1
    nt("Bunlar ARMING_CHECK geri açılınca arm'ı KESER.")
else:
    print(f"  {S} bu pencerede pre-arm mesajı yok")
    nt("ARMING_CHECK=0 iken denetimler KOŞMAZ — mesaj yokluğu 'sorun yok'")
    nt("DEMEK DEĞİLDİR. Gerçek sınama: ARMING_CHECK=1 yapıp bakmak.")
if estop:
    print(f"  {K} RC10 motor acil durdurma sinyali: {estop[-1]}"); hata = 1
    nt("§1.05: HIGH kaldığında 10 dk komut verildi, tekne 0,54 m kımıldadı.")

d.destroy_node(); rclpy.shutdown()
sys.exit(1 if hata else 0)
PY
[ $? -ne 0 ] && HATA=1

# ── 8 ──────────────────────────────────────────────────────────────────────
bas "8/9 — AYAR SERVİSLERİ (koşumdan önce AÇIK, yarışma günü KAPALI)"
for S in girdap-ff-ayar girdap-plant-ayar girdap-pusula-ayar; do
    A=$(systemctl is-active "$S" 2>/dev/null); A=${A:-yok}
    E=$(systemctl is-enabled "$S" 2>/dev/null || echo yok)
    if [ "$A" = "active" ]; then ok "$S: $A / $E"
    else ht "$S: $A / $E — ayar KOŞMAZ ve kimse fark etmez (§1.42)"; fi
done
nt "🔴 YARIŞMA GÜNÜ ÜÇÜ DE KAPATILACAK — üçü de koşum sırasında FC'ye YAZAR:"
nt "   sudo systemctl disable --now girdap-ff-ayar girdap-plant-ayar girdap-pusula-ayar"

# ── 9 ──────────────────────────────────────────────────────────────────────
bas "9/9 — KAYIT (şartname md 4.2: her gecikmiş dosya 5 ceza puanı)"
KALAN=$(df -BG --output=avail "$HOME/girdap_logs" 2>/dev/null | tail -1 | tr -dc '0-9')
if [ -n "${KALAN:-}" ] && [ "$KALAN" -ge 20 ]; then ok "disk boş alan: ${KALAN} GB"
elif [ -n "${KALAN:-}" ]; then ht "disk boş alan yalnız ${KALAN} GB — bant koşum ortasında dolabilir"
else uy "disk alanı okunamadı"; fi
SON_BANT=$(ls -dt "$HOME"/girdap_logs/rosbag/session_* 2>/dev/null | head -1)
if [ -n "$SON_BANT" ]; then
    YAS=$(( ( $(date +%s) - $(stat -c %Y "$SON_BANT") ) / 60 ))
    if [ "$YAS" -lt 5 ]; then ok "bant yazıyor: $(basename "$SON_BANT") (${YAS} dk önce)"
    else uy "son bant ${YAS} dk önce yazılmış — rosbag gerçekten kaydediyor mu?"; fi
else uy "hiç bant oturumu yok"; fi

# ── özet ───────────────────────────────────────────────────────────────────
printf '\n'
if [ "$HATA" -eq 0 ]; then
    printf '\033[1;42m  TEK ✗ YOK — tekne suya indirilebilir  \033[0m\n'
else
    printf '\033[1;41m  ✗ VAR — yukarıdakiler çözülmeden suya indirme  \033[0m\n'
fi
printf '\n\033[2m  Bu betik yalnız OKUR. Hiçbir ✓ "ölçüldü" demek değil, "şu an ayakta"\n'
printf '  demektir — asıl kanıt koşumun kendisidir.\033[0m\n\n'
exit "$HATA"
