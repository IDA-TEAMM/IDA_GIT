#!/usr/bin/env bash
# GİRDAP İDA — rosbag2 kayıt betiği (Jetson).
#
# §0.25d'nin "Repoda rosbag yardımcısı/scripti YOK" borcunu kapatır.
# MCAP + zstd (dosya düzeyinde) — Jetson'da SQLite3'ten daha hızlı/az CPU
# (benchmark: mcap.dev/guides/benchmarks/rosbag2-storage-plugins, 2026-08-10
# araştırması). ros-humble-rosbag2-storage-mcap paketi GEREKİR.
#
# Kullanım:
#   bash scripts/rosbag_kaydet.sh                  # varsayılan liste
#   bash scripts/rosbag_kaydet.sh --tam             # + tüm /perception/* ve /girdap/*
#   bash scripts/rosbag_kaydet.sh --lidar           # + ham /livox/lidar (AĞIR!)
#   bash scripts/rosbag_kaydet.sh topic1 topic2 …   # elle liste

set -euo pipefail

CIKTI_KOK="$HOME/girdap_logs/rosbag"
mkdir -p "$CIKTI_KOK"

# --- Saat güvenilir mi? (§0.53) ------------------------------------------
# 🔴 12.08.2026: Jetson'un gerçek zaman saati (RTC) tutmuyor. Açılışta sistem
# 1970-01-01'den başlıyor; saat ancak Pixhawk GPS'i (girdap-saat) ya da NTP
# devreye girince düzeliyor — ölçülen gecikme o açılışta **16 dakika**.
# Sonuç: `session_19700101_020151` adlı bir kayıt oluşuyor ve mesaj damgaları
# kayıt ORTASINDA 56 yıl sıçrıyor. §0.51b'nin "bantlar üst üste biniyor /
# saat geri yükleniyor" bulgusunun kök nedeni budur.
#
# Kayıt saatin düzelmesini BEKLEMEZ — veri kaybetmek saatsiz kalmaktan kötü.
# Bunun yerine: adı işaretle ve sıçramayı ölçülebilir kıl (aşağıdaki nöbetçi).
YIL="$(date +%Y)"
BOOT_SN="$(cut -d. -f1 /proc/uptime)"
if [ "$YIL" -lt 2020 ]; then
    SAAT_GUVENILIR=0
    DAMGA="SAATSIZ_boot$(printf '%06d' "$BOOT_SN")"
    echo "⚠️  SAAT GÜVENİLİR DEĞİL (yıl=$YIL) — kayıt adı 'SAATSIZ' ile"
    echo "   işaretlendi. Saat düzelince eşleme _saat.txt'ye yazılacak."
else
    SAAT_GUVENILIR=1
    DAMGA="$(date +%Y%m%d_%H%M%S)"
fi
CIKTI="$CIKTI_KOK/session_${DAMGA}"
SAAT_LOG="$CIKTI_KOK/session_${DAMGA}_saat.txt"

# Kayıt damgalarını sonradan gerçek saate çevirebilmek için gereken her şey.
{
    echo "# GIRDAP bant saat kaydi — damgalari gercek saate cevirmek icin"
    echo "kayit_adi=session_${DAMGA}"
    echo "baslangic_saat_guvenilir=${SAAT_GUVENILIR}"
    echo "baslangic_duvar_saati_unix=$(date +%s)"
    echo "baslangic_duvar_saati_iso=$(date -Is)"
    echo "baslangic_boot_sn=${BOOT_SN}"
} > "$SAAT_LOG"

# Nöbetçi: saat sıçrarsa ofseti yakala. Bir kez yazar, sonra çıkar.
# Sıçrama = duvar saatindeki artışın, tekdüze (monotonic) uptime artışından
# belirgin şekilde büyük olması. Böylece 1970'te başlayan bir kaydın
# damgaları geriye dönük gerçek saate çevrilebilir.
(
    onceki_duvar=$(date +%s); onceki_boot=$(cut -d. -f1 /proc/uptime)
    while :; do
        sleep 5
        simdi_duvar=$(date +%s); simdi_boot=$(cut -d. -f1 /proc/uptime)
        d_duvar=$((simdi_duvar - onceki_duvar))
        d_boot=$((simdi_boot - onceki_boot))
        if [ "$((d_duvar - d_boot))" -gt 10 ] || [ "$((d_boot - d_duvar))" -gt 10 ]; then
            {
                echo "saat_sicramasi_tespit_edildi=1"
                echo "sicrama_oncesi_unix=${onceki_duvar}"
                echo "sicrama_sonrasi_unix=${simdi_duvar}"
                echo "sicrama_miktari_sn=$((d_duvar - d_boot))"
                echo "sicrama_anindaki_boot_sn=${simdi_boot}"
                echo "sicrama_sonrasi_iso=$(date -Is)"
                echo "# Bu kaydin sicramadan ONCEKI damgalarina"
                echo "# 'sicrama_miktari_sn' eklenerek gercek saat bulunur."
            } >> "$SAAT_LOG"
            break
        fi
        onceki_duvar=$simdi_duvar; onceki_boot=$simdi_boot
    done
) &
SAAT_NOBETCI_PID=$!

# ─────────────────────────────────────────────────────────────────────────────
# F-F.13 (14.08.2026, §0.99g) — UÇUŞ KONTROLCÜSÜ YOKSA KAYIT **SESSİZ KALMASIN**
#
# 🔴 Ölçülen olay: 16:04'te başlayan oturum **78 dakika** koştu ve içinde
# kullanılabilir TEK satır yoktu — `/dev/pixhawk` ilk saniyeden itibaren yoktu,
# hız/poz/RC/engel topic'leri **sıfır mesaj**. Köprü yer istasyonuna 461 kez
# "MAVROS-YOK" yolladı ama kimse görmedi; arıza ancak akşam bant incelenirken
# fark edildi. Aynı hâl 18:28 oturumunda da tekrarladı.
#
# TASARIM: kayıt **ASLA reddedilmez** — bağlantı koşum ORTASINDA düşerse o ana
# kadarki veri en değerli şeydir. Bunun yerine olay, sonradan bakan kişinin
# kaçıramayacağı bir yere yazılır: oturum dizininin İÇİNE bir işaret dosyası.
# 60 s beklenir (açılışta udev/enumerasyon yarışı meşru), sonra bakılır.
(
    sleep 60
    [ -e /dev/pixhawk ] && exit 0
    ISARET="${CIKTI}/UCUS_KONTROLCUSU_YOK.txt"
    {
        echo "# ⚠️ BU KAYIT BÜYÜK OLASILIKLA BOŞTUR — F-F.13"
        echo "#"
        echo "# Kayit basladiktan 60 saniye sonra /dev/pixhawk YOKTU."
        echo "# Yani ucus kontrolcusu bagli degil: hiz, poz, RC ve engel"
        echo "# topic'leri bu kayitta MUHTEMELEN SIFIR mesaj icerir."
        echo "#"
        echo "# Sebep genelde fizikseldir: TELEM2 FTDI kablosu takili degil,"
        echo "# tekne beslemesi kesik, ya da USB-C'den baglandiysa /dev/pixhawk"
        echo "# symlink'i olusmamis (bkz. GIRDAP_DURUM 0.99m)."
        echo "#"
        echo "kontrol_zamani_iso=$(date -Is)"
        echo "kayit_adi=session_${DAMGA}"
        echo "dev_pixhawk=YOK"
        echo "ttyUSB/ttyACM=$(ls /dev/ttyUSB* /dev/ttyACM* 2>/dev/null | tr '\n' ' ')"
    } > "$ISARET"
    echo "🔴 UCUS KONTROLCUSU YOK — '$ISARET' yazildi; bu kayit muhtemelen bos." >&2
    # Sonradan gelirse onu da yaz: bandin hangi anindan itibaren veri var?
    while :; do
        sleep 30
        if [ -e /dev/pixhawk ]; then
            {
                echo "ucus_kontrolcusu_geri_geldi_iso=$(date -Is)"
                echo "# Bu andan SONRAKI parcalarda veri olmasi beklenir."
            } >> "$ISARET"
            exit 0
        fi
    done
) &
FC_NOBETCI_PID=$!

# §0.25d "Kaydedilecek doğru liste" — karar yığınının çekirdek çıktıları.
# 11.08.2026 (§0.33): setpoint_velocity + rc/in + state + diagnostics
# EKLENDİ — eski liste komutu (cmd_vel) ve güvenlik durumunu (RC kill,
# ARM/mod, mavros_router link sağlığı) hiç kaydetmiyordu. Yarın servo/motor
# doğrulaması (§0.30b) ve RC failsafe testi (§0.30c) yapılacaksa bunlar
# olmadan kayıt sonradan "hangi komut gitti, tekne neden öyle davrandı"
# sorusuna cevap veremezdi.
#
# 🔴 12.08.2026 (KAR-09) — `/livox/lidar` VARSAYILANDAN ÇIKARILDI.
# Kaptanın bag analizi: `session_19700101_020120` **14 GB / 4,9 milyon mesaj**
# ve bunun **3,6 milyonu** bu tek topic'ten. Aynı oturumda tüm hat (MAVROS
# dahil) **8-12 saniye** donuyordu; disk G/Ç doygunluğu en güçlü aday.
# 2 m/s'de 12 saniye = **24 metre kör seyir**.
#
# Ham nokta bulutu şartname çıktılarının HİÇBİRİNDE kullanılmıyor: Dosya-3
# (yerel harita) `local_map_node`'un OccupancyGrid'inden, Dosya-1b ise
# işlenmiş cluster'lardan üretiliyor. Yani bu topic kayıtta bir hata ayıklama
# lüksüydü ve bedeli görev güvenliğiydi.
# Gerçekten gerekiyorsa `--lidar` ile açıkça iste (tercihen ayrı diske).
VARSAYILAN=(
    /girdap/fusion/odom
    /girdap/fusion/pose
    /mavros/imu/data
    /mavros/global_position/global
    /mavros/local_position/pose
    /mavros/local_position/velocity_body
    /mavros/setpoint_velocity/cmd_vel_unstamped
    /mavros/rc/in
    # F-F.2 (14.08, §0.99k): rc/in KOMUTU gösterir, rc/out UÇUŞ KONTROLCÜSÜNÜN
    # MOTORA NE VERDİĞİNİ. İkisi olmadan "GUIDED'da kumanda motorları sürebiliyor
    # mu" sorusu bantla CEVAPLANAMIYOR — 14.08'de iki ayrı koşumda denendi,
    # ikisinde de elde yalnız komut vardı. Ucuz: ~10 Hz, 16 kanal.
    /mavros/rc/out
    # F-F.17 (14.08, §0.99p): PUSULA. 14.08 akşamı canlı ölçümde manyetik alanın
    # EĞİM AÇISI 1,4° çıktı (Bolu'da ~58° olmalı) — ciddi bir yönelim şüphesi.
    # Ama günün BANTLARINDAN geriye dönük doğrulanamadı, çünkü bu topic kayıtta
    # yoktu. Pusula açık kalem olduğu sürece (§0.91) kaydı ZORUNLU.
    /mavros/imu/mag
    /mavros/state
    # 12.08 (PAR-03): FC'nin pre-arm RET SEBEBİ buradan gelir. Araç 14
    # oturumda hiç ARM edilemedi ve sebebi hiçbir kayıtta yoktu — yalnız
    # operatörün ekranında bir an görünüp kayboldu.
    /mavros/statustext/recv
    /mavros/statustext/send
    /diagnostics
    # 12.08 (KAR-04): thrust NEDEN sıfır. Bu topic kaydedilmezse "komut sıfır"
    # ile "komut yok" ayrımı yine bag'den elle çıkarılmak zorunda kalır —
    # kaptanın 30.874 mesaj için yapmak zorunda kaldığı şey tam da buydu.
    /girdap/control/inhibit_reason
    /perception/obstacle_map
    /tf
    /tf_static
)

# --tam: + algı/karar arayüz topic'leri (bu oturumda doğrulanan zincir).
# /mavros/state artık VARSAYILAN'da (11.08) — burada tekrarlanmıyor, aksi
# halde ros2 bag record'a aynı topic iki kez verilir.
# 11.08.2026 (öğleden sonra) — GUIDED görev denemesi için EKLENENLER:
# kayıt "tekne gitmeye çalıştı mı" sorusuna cevap veremiyordu. Komut zinciri
# FSM durumu → hedef → thrust şeklinde ilerliyor; ortadaki iki halka
# (current_target, control/thrust) hiç kaydedilmiyordu, yani tekne kıpırdamazsa
# zincirin NEREDE koptuğu sonradan anlaşılamıyordu. MP'den yüklenen ham görev
# (/mavros/mission/waypoints) ve karar katmanının onu nasıl okuduğu
# (/girdap/mission/waypoints) de kayda girdi — ikisi arasındaki fark
# mission_manager'ın görevi doğru sindirip sindirmediğini gösterir.
EK_TAM=(
    /perception/buoys
    /perception/buoys_3d
    /perception/classified_obstacles
    /perception/gate_target
    /perception/gate_passed
    /perception/gate_count
    /girdap/planning/gate
    /girdap/planning/gate_count
    /girdap/planning/edge_buoys
    # 🔴 17.08 EKLENDİ — MPPI'nin GERÇEK REFERANSI. Bu satır olmadığı için
    # 17.08 göl bandı (`session_20260817_193312`) çözümlenemedi: nöbetçi
    # **43 kez** `RRT-RED global plan uretilemedi` bastı ve pivot kapısı
    # geri komutların %91'inde kapalıydı — ama kapının okuduğu referansın
    # o anda ne olduğu **hiçbir yerde kayıtlı değildi**, yani "plan boştu"
    # hipotezi ne doğrulanabildi ne çürütülebildi. Bandın tamamı 340 MB;
    # bu konu 10 Hz'te birkaç yüz nokta, maliyeti ihmal edilebilir.
    /girdap/planning/global_path
    /girdap/mission/state
    /girdap/mission/current_target
    /girdap/mission/waypoints
    /girdap/mission/waypoint_reached
    /girdap/mission/complete
    /girdap/control/thrust
    /girdap/parkur/state
    /mavros/mission/waypoints
    /mavros/mission/reached
)

EK_LIDAR=(/livox/lidar)

LIDAR_ISTENDI=0
if [ "${1:-}" = "--lidar" ]; then
    LIDAR_ISTENDI=1
    shift
fi

if [ "${1:-}" = "--tam" ]; then
    TOPICLER=("${VARSAYILAN[@]}" "${EK_TAM[@]}")
    shift
elif [ $# -ge 1 ]; then
    TOPICLER=("$@")
else
    TOPICLER=("${VARSAYILAN[@]}")
fi

if [ "$LIDAR_ISTENDI" = "1" ]; then
    TOPICLER+=("${EK_LIDAR[@]}")
    echo "⚠ HAM LiDAR KAYDI ACIK — kaptanin olcumunde bu topic tek basina"
    echo "  3,6 milyon mesaj / ~14 GB uretti ve tum hattin 8-12 s donmasiyla"
    echo "  ayni oturumda gorundu (KAR-09). Uzun kosuda kullanma."
fi

# --- Dayanıklılık ayarları (§0.53) ---------------------------------------
# MCAP yazıcı ayarı betiğin yanında durur (kurulumda kopyalanır).
MCAP_AYAR="${GIRDAP_MCAP_AYAR:-$(dirname "$(readlink -f "$0")")/rosbag_mcap_dayanikli.yaml}"
if [ ! -f "$MCAP_AYAR" ]; then
    echo "🔴 MCAP ayar dosyası YOK: $MCAP_AYAR" >&2
    echo "   Dayanıklı kayıt bu dosyaya bağlı — kayıt BAŞLATILMIYOR." >&2
    exit 1
fi

# Bant her BOLME_SN saniyede bir kapatılıp yenisi açılır. Kapanan dosya tam
# footer + index ile mühürlenir; ani kesinti yalnız açık olan dosyayı riske
# atar. 60 sn = en kötü hâlde 1 dakikalık dilim şüpheli, öncesi kesin sağlam.
BOLME_SN="${GIRDAP_BANT_BOLME_SN:-60}"

# `write()` verinin diske indiği anlamına gelmez, yalnız çekirdeğin sayfa
# önbelleğine girdiği anlamına gelir; fişi çeken kesinti onu da götürür.
# `99-girdap-bant-dayaniklilik.conf` bu pencereyi 30 sn'den 3 sn'ye indiriyor;
# buradaki düzenli `sync` ise kalan payı da kapatır (uçuş kontrolcüsünün
# SD karta doğrudan yazmasının karşılığı).
SYNC_SN="${GIRDAP_BANT_SYNC_SN:-5}"

echo "== GİRDAP rosbag kaydı =="
echo "çıktı   : $CIKTI"
echo "format  : mcap, SIKIŞTIRMASIZ + parçalama KAPALI (doğrudan yazım)"
echo "davranış: Pixhawk dataflash kaydı gibi — mesaj geldiği anda dosyaya"
echo "          eklenir; fiş çekilirse yalnız dosyanın SONU kesilir."
echo "bölme   : her $BOLME_SN sn (kapanan dosya mühürlenir)"
echo "disk    : her $SYNC_SN sn 'sync' (sayfa önbelleği diske indirilir)"
echo "mcap    : $MCAP_AYAR"
echo "topic'ler:"
printf '  %s\n' "${TOPICLER[@]}"
echo
echo "Disk (kayıttan önce): $(df -h "$HOME" | awk 'NR==2{print $4" boş / "$2}')"
echo "Ctrl+C ile durdur (metadata düzgün yazılır)."
echo

# 🔴 KAR-09 öneri #2: termal/CPU korelasyonu ANCAK eşzamanlı kayıtla kurulur.
# Donmaların sebebi (disk G/Ç mi, termal kısıtlama mı) bag verisinden tek
# başına ayırt EDİLEMİYOR — kaptanın raporunda ikisi de "en olası aday"
# olarak açık kaldı. tegrastats bunu ayırır ve maliyeti ~sıfırdır.
# ⚠ tegrastats log'u bag DIZININİN İÇİNE yazılamaz: `ros2 bag record -o`
# hedef dizin ZATEN VARSA hata verir, dizini önceden yaratmak kaydı komple
# engellerdi. Kardeş dosya olarak yazıyoruz (aynı zaman damgası eşleştirir).
TEGRA_LOG="$CIKTI_KOK/session_${DAMGA}_tegrastats.txt"
if command -v tegrastats >/dev/null 2>&1; then
    tegrastats --interval 1000 --logfile "$TEGRA_LOG" &
    TEGRA_PID=$!
    echo "tegrastats: $TEGRA_LOG (1 s aralik, KAR-09 korelasyonu)"
else
    TEGRA_PID=""
    echo "⚠ tegrastats yok — donma sebebi (termal mi G/C mi) ayirt edilemez"
fi

# 🔴 §0.45c DÜZELTİLDİ: eskiden burada `exec ros2 bag record …` vardı.
# `exec` kabuğu rosbag süreciyle DEĞİŞTİRDİĞİ için yukarıdaki `trap` yok
# oluyordu → tegrastats (ve şimdi saat nöbetçisi) öksüz kalıyor, bant servisi
# her durdurmada 5 dakika asılıp `failed` bitiyordu; reboot da 5 dakika
# uzuyordu. Artık `exec` YOK: rosbag arka planda başlatılıp beklenir, sinyal
# geldiğinde ÜÇ çocuk da düzgün toplanır.
# Düzenli disk indirme — kayıt sürerken veriyi sayfa önbelleğinde bırakmaz.
( while :; do sleep "$SYNC_SN"; sync; done ) &
SYNC_PID=$!

temizle() {
    [ -n "${ROSBAG_PID:-}" ] && kill -INT "$ROSBAG_PID" 2>/dev/null || true
    [ -n "${ROSBAG_PID:-}" ] && wait "$ROSBAG_PID" 2>/dev/null || true
    [ -n "${TEGRA_PID:-}" ] && kill "$TEGRA_PID" 2>/dev/null || true
    [ -n "${SAAT_NOBETCI_PID:-}" ] && kill "$SAAT_NOBETCI_PID" 2>/dev/null || true
    [ -n "${FC_NOBETCI_PID:-}" ] && kill "$FC_NOBETCI_PID" 2>/dev/null || true
    [ -n "${SYNC_PID:-}" ] && kill "$SYNC_PID" 2>/dev/null || true
    sync            # son kalanları da indir
}
trap temizle EXIT INT TERM
echo

ros2 bag record \
    -o "$CIKTI" \
    -s mcap \
    --storage-config-file "$MCAP_AYAR" \
    --compression-mode none \
    --max-bag-duration "$BOLME_SN" \
    "${TOPICLER[@]}" &
ROSBAG_PID=$!
wait "$ROSBAG_PID"
