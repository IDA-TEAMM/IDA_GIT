#!/bin/bash
# SANAL GÖL — kapalı döngü uçtan uca koşum. Canlı yığından İZOLE (domain 77).
#
# Kullanım:  gol_kos.sh [kapi_sayisi] [kapi_acikligi_m] [kapi_araligi_m] [engel_sayisi]
# Varsayılan: gerçek parkur geometrisi (§0.17b) — 8 kapı, 12 m açıklık, 4 m aralık.
#
# 🔴 13.08 dersi: `ros2 run X` bir SARMALAYICI süreç açar, asıl düğüm ayrı
# süreçtir ve komut satırı canlı yığınla AYNI görünür. Sarmalayıcıyı öldürmek
# çocuğu öldürmez → domain'de hayalet düğüm kalır (ölçüldü: 4 kopya fsm_node,
# görev durumu KILL'e titredi). Çözüm: her düğüm `setsid` ile KENDİ SÜREÇ
# GRUBUNDA başlar; `gol_dur.sh` grubu topluca öldürür.
source /opt/ros/humble/setup.bash
source "$HOME/ros2_ws/install/setup.bash" 2>/dev/null
export ROS_DOMAIN_ID=77
export PYTHONPATH="$HOME/IDA_GIT/son_kodv2/karar:$PYTHONPATH"
P="$HOME/ros2_ws/install/girdap_decision/share/girdap_decision/config/params.yaml"
S="$HOME/IDA_GIT/son_kodv2/karar/scripts"
L_KOK="${GIRDAP_GOL_LOG:-$HOME/girdap_logs/gol}"
L="$L_KOK"
mkdir -p "$L"; rm -f "$L"/*.log; : > "$L/gol.pgids"

KAPI="${1:-8}"; ACIK="${2:-12.0}"; ARALIK="${3:-4.0}"; ENGEL="${4:-4}"
# 5./6. argüman: dalga bozucusu (yanal sürüklenme m/s · yaw rad/s) — 0 = kapalı
DALGA="${5:-0.0}"; DALGA_YAW="${6:-0.0}"
# 7. argüman: başlangıç yönü (derece). 90 = kuzey = ESKİ DAVRANIŞ BİREBİR.
# -90 → burun güneye, bütün görev noktaları ARKADA: F-F.22'nin (geri sürüş)
# ölçülebildiği tek sahne. 17.08 göl bandında komutların %23,1'i geriydi ve
# %83,4'ünde hedef arkadaydı; bu sahne olmadan sim o sınıfı hiç göremiyor.
YON0="${7:-90.0}"
# ⚠ rclpy parametre tipi KATIDIR: `-90` INTEGER gelir ve düğüm
# `InvalidParameterTypeException` ile ÖLÜR — üstelik sanal_gol ölünce bütün
# zincir "poz HİÇ gelmedi" der, sebep gizlenir. Kabuk tarafında ondalığa
# zorlanır (ölçüldü: 17.08, ilk A/B koşumu tam bu yüzden boş döndü).
YON0="$(printf '%.1f' "$YON0")"
# 8./9. argüman: F-S.18 GERÇEKÇİ ALGI. 0 = kusursuz (eski davranış).
# 1.0 = 17.08 göl bandında ÖLÇÜLEN şiddet (kaçırma + konum hatası).
# HAYALET = kare başına eklenen kimliksiz (UNKNOWN) tespit sayısı;
# gerçek bantta ~95'ti (kıyı), simde parkur küçük olduğu için dozlanır.
GERCEKCILIK="$(printf '%.2f' "${8:-0.0}")"; HAYALET="${9:-0}"
# 10. argüman: algı kusur TOHUMU. Aynı tohum = aynı kusur dizisi. A/B'de
# kollar AYNI tohumla koşulur (eşleştirilmiş kıyas); gürültü tabanı ölçülürken
# tohum DEĞİŞTİRİLİR. §19.4: "A/B tek koşumla YAPILAMAZ".
TOHUM="${10:-0}"
# 11. argüman: F-P.30 kolu — kimliksiz kümenin azami yarıçapı (0 = ham temsil)
HMAKS="$(printf '%.2f' "${11:-0.0}")"
# Planlama şalterleri ortamdan geçer (A/B için) — hepsi varsayılan KAPALI.
GOL_PLANNING_EK="${GIRDAP_GOL_PLANNING_EK:-}"

basla() {
    local ad="$1"; shift
    setsid "$@" > "$L/$ad.log" 2>&1 &
    echo $! >> "$L/gol.pgids"
}

# 🔴 ALGI ZINCIRI ACIKSA sanal golun IDEAL algi ciktisi /gercek/* altina
# alinir. Sebep: `sahte_ham_sensor` onu ham LiDAR/kameraya cevirecek ve
# GERCEK algi dugumleri /perception/* uretecek. Remap olmazsa IKI URETICI
# ayni topic'e basar; fuzyon hangisini aldigini bilemez ve olcum anlamsizlasir.
# (Remap `sanal_gol`e konur — `sahte_ham_sensor`e DEGIL: o zaten /gercek/*
#  dinliyor. Ters kurulum sessizce hicbir sey degistirmezdi.)
SG_REMAP=""
if [ "${GIRDAP_GOL_ALGI:-0}" = "1" ] || [ "${GIRDAP_GOL_TAM:-0}" = "1" ]; then
    SG_REMAP="-r /perception/obstacle_map:=/gercek/obstacle_map \
              -r /perception/classified_obstacles:=/gercek/classified_obstacles"
fi
# ── ARIZA ENJEKSIYONU (18.08) — hepsi VARSAYILAN KAPALI ─────────────────
# Kural motorunun DUYARLILIGINI sinar. Temiz kosumda sessiz kalmasi
# (ozgulluk) zaten olculuyor; bunlar ihlalde KIRMIZI yandigini gosterir.
# Ayni tohum = ayni ariza dizisi => A/B eslestirilmis kiyas yapilabilir.
#   GIRDAP_ARIZA_SICRAMA=6.54   GIRDAP_ARIZA_SICRAMA_ORAN=0.3   (F1)
#   GIRDAP_ARIZA_NAN=0.2                                        (F4)
#   GIRDAP_ARIZA_DAMGA=5.0                                      (S1)
#   GIRDAP_ARIZA_KADANS=12                                      (C3)
#   GIRDAP_ARIZA_KESINTI=40                                     (C3/C1)
#   GIRDAP_ARIZA_GOVDE=0.05                                     (F5)
# GERCEK TEKNE DINAMIGI (18.08) — varsayilan KAPALI.
# Basit model olculdu: ivme 4,2x fazla · zaman sabiti 5,9x cevik · donus 2,8x
# hizli. GIRDAP_GERCEK_DINAMIK=1 ile MPPI'nin kullandigi CatamaranDynamics
# tesis olarak kosar. GIRDAP_DINAMIK_BOZUCU=0.2 ile tesis plandan %20 saptirilir
# (model uyusmazligi sinifi da sinanabilsin).
# ⚠ rclpy parametre tipi KATIDIR: `1` INTEGER gelir ve dugum
# InvalidParameterTypeException ile OLUR (betigin YON0 icin verdigi dersin
# aynisi). Kabuk tarafinda bool'a normalize edilir.
case "${GIRDAP_GERCEK_DINAMIK:-0}" in 1|true|TRUE|yes) _GD=true;; *) _GD=false;; esac
DYN="-p gercek_dinamik:=${_GD}
     -p gercek_dinamik_bozucu:=${GIRDAP_DINAMIK_BOZUCU:-0.0}"

AR="-p ariza_poz_sicramasi_m:=${GIRDAP_ARIZA_SICRAMA:-0.0}
    -p ariza_poz_sicrama_orani:=${GIRDAP_ARIZA_SICRAMA_ORAN:-0.0}
    -p ariza_poz_nan_orani:=${GIRDAP_ARIZA_NAN:-0.0}
    -p ariza_damga_kaydirma_s:=${GIRDAP_ARIZA_DAMGA:-0.0}
    -p ariza_kadans_bolen:=${GIRDAP_ARIZA_KADANS:-1}
    -p ariza_kesinti_t_s:=${GIRDAP_ARIZA_KESINTI:-0.0}
    -p ariza_govde_yansimasi_m:=${GIRDAP_ARIZA_GOVDE:-0.0}"

# ── STATİK TF AĞACI (18.08.2026 — GÖLDE HİÇ YOKTU) ────────────────────────
# 🔴 Ölçüldü: gölde `/tf` ve `/tf_static` YAYINCI 0. Dağıtımda
# `hardware.launch` üç static TF basıyor (base_link → livox_frame / oak_frame
# / imu_link) ve değerler `hardware.yaml tf:` bloğunda ÖLÇÜLMÜŞ montaj
# sayıları (ör. oak yaw = +0,0415 rad, 11.08'de şeritle ölçüldü).
# TF olmadan tf2 sorgusu yapan her yol gölde sessizce farklı davranır.
# Değerler TEK KAYNAKTAN (hardware.yaml) okunur — burada elle sayı YAZILMAZ,
# yoksa ayrışır ve göl yanlış montajı yansıtır.
HW="$HOME/IDA_GIT/son_kodv2/karar/ros2_ws/src/girdap_decision/config/hardware.yaml"
tf_bas() {   # $1 = cocuk cerceve
    local c="$1"
    local v
    v="$(python3 - "$HW" "$c" <<'PYTF'
import sys, yaml
tf = (yaml.safe_load(open(sys.argv[1], encoding="utf-8")) or {}).get("tf", {})
d = tf.get(sys.argv[2]) or {}
print(" ".join(str(float(d.get(k, 0.0)))
                for k in ("x", "y", "z", "yaw", "pitch", "roll")))
PYTF
)"
    set -- $v
    basla "tf_$c" ros2 run tf2_ros static_transform_publisher \
        --x "$1" --y "$2" --z "$3" --yaw "$4" --pitch "$5" --roll "$6" \
        --frame-id base_link --child-frame-id "$c"
}
tf_bas livox_frame; tf_bas oak_frame; tf_bas imu_link
echo "  + TF: base_link → livox_frame · oak_frame · imu_link (hardware.yaml)"

basla sanal_gol python3 "$S/sanal_gol.py" --ros-args $SG_REMAP $AR $DYN \
    -p kapi_sayisi:="$KAPI" -p kapi_acikligi_m:="$ACIK" \
    -p kapi_araligi_m:="$ARALIK" -p engel_sayisi:="$ENGEL" \
    -p dalga_genlik_mps:="$DALGA" -p dalga_yaw_rps:="$DALGA_YAW" \
    -p baslangic_yon_derece:="$YON0" \
    -p algi_gercekcilik:="$GERCEKCILIK" -p hayalet_sayisi:="$HAYALET" \
    -p algi_tohum:="$TOHUM" -p hayalet_maks_yaricap:="$HMAKS"
sleep 2
basla fusion   ros2 run girdap_decision fusion_node --ros-args --params-file "$P" -p use_isam2:=false
basla mission  ros2 run girdap_decision mission_manager_node --ros-args --params-file "$P" -p mission_source:=fc
basla fsm      ros2 run girdap_decision fsm_node --ros-args --params-file "$P"
basla bridge   ros2 run girdap_decision mavros_bridge_node --ros-args --params-file "$P"
basla planning ros2 run girdap_decision planning_node --ros-args --params-file "$P" -p use_rrt:=true $GOL_PLANNING_EK

# ══════════════════════════════════════════════════════════════════════════
# TAM SİSTEM KATMANLARI (18.08.2026) — varsayılan KAPALI, davranış bit birebir
#
# 🔴 NEDEN GEREKLİ: yukarıdaki beş düğüm dağıtımda koşan **17** düğümün
# yalnız bir bölümü. Bugüne kadar gölde HİÇ koşmayan yedi gerçek düğüm var
# ve ikisi doğrudan **teslim dosyası** üretiyor (md 4.2 — eksik dosya başına
# 5 CEZA PUANI). Yani teslim zinciri uçtan uca hiç sınanmamıştı.
#
# Katmanlar ayrı şalterlerde: biri patlarsa diğerleri koşmaya devam eder ve
# hangi katmanın soruna yol açtığı belli olur (tek "hepsi açık" şalteri,
# arıza ayrıştırmayı imkânsız kılardı).
#
#   GIRDAP_GOL_ALGI=1     ham sensör → GERÇEK algı zinciri
#   GIRDAP_GOL_TESLIM=1   Dosya-1/2/3 üreticileri
#   GIRDAP_GOL_P3=1       Parkur-3 hedef rengi kapısı
#   GIRDAP_GOL_IZLEYICI=1 doğrulama izleyicisi (kural motoru)
#   GIRDAP_GOL_TAM=1      hepsi birden
# ══════════════════════════════════════════════════════════════════════════
[ "${GIRDAP_GOL_TAM:-0}" = "1" ] && {
    GIRDAP_GOL_ALGI=1; GIRDAP_GOL_TESLIM=1
    GIRDAP_GOL_P3=1; GIRDAP_GOL_IZLEYICI=1
}

# ── ALGI ZİNCİRİ ──────────────────────────────────────────────────────────
# `sanal_gol` /perception/obstacle_map + classified_obstacles'ı DOĞRUDAN
# yayınlıyor ⇒ gerçek algı düğümleri baypas ediliyor. `sahte_ham_sensor`
# o ideal çıktıyı HAM LiDAR bulutu + kamera karesine geri çevirir; böylece
# kümeleme, bearing füzyonu ve sınıflandırma GERÇEKTEN koşar.
# ⚠ Sanal gölün kendi algı yayını `/gercek/...` altına alınır — iki üretici
# aynı topic'e basarsa füzyon hangisini aldığını bilemez.
if [ "${GIRDAP_GOL_ALGI:-0}" = "1" ]; then
    # Remap YOK: bu dugum zaten /gercek/* dinliyor (remap sanal_gol'de).
    # 🔴 KAMERA: KADANS gerçek (8 Hz), ÇÖZÜNÜRLÜK gölün taşıma sınırı.
    #
    # Gerçekte görüntü DDS'ten **HİÇ GEÇMEZ** — depthai onu USB'den aynı
    # sürece verir. Gölde iki ayrı süreç olduğu için DDS'e girmek zorunda ve
    # orada ham görüntü çok pahalı:
    #     1280×720 → 2,7 MB/kare ⇒ abone ~1 Hz alıyordu
    #      512×512 → 786 KB/kare ⇒ abone 4,25 Hz (yayıncı 8,11)
    #      256×256 → 196 KB/kare ⇒ `net.core.rmem_max` (208 KB) İÇİNDE
    # Kök neden ölçüldü: tek kare soket tamponundan büyükse FastDDS
    # parçaları toparlayamıyor. Kalıcı çözüm `sysctl net.core.rmem_max`
    # büyütmek (root ister; Jetson kurulumunda YAPILMALI).
    #
    # 🔑 Neyi koruduğumuz önemli: **KADANS ve ZAMANLAMA** gerçek (8 Hz kamera
    # ↔ 10 Hz LiDAR) — kapı zincirini bozan şey odur. Çözünürlük yalnız
    # tespit hassasiyetini etkiler, onu da bu kip zaten sınamıyor (YOLO
    # yerine renk eşiği koşuyor). bbox NORMALİZE olduğu için ölçekten
    # bağımsız. `GIRDAP_GOL_KAM_PX=512` ile büyütülebilir (kare kaybı pahasına).
    basla ham_sensor python3 "$S/sahte_ham_sensor.py" --ros-args \
        -p kamera_genislik_px:="${GIRDAP_GOL_KAM_PX:-256}" \
        -p kamera_yukseklik_px:="${GIRDAP_GOL_KAM_PX:-256}" \
        -p kamera_hz:=8.0 -p lidar_hz:=10.0
    basla p_lidar  ros2 run girdap_decision perception_lidar_node --ros-args --params-file "$P"
    basla p_fusion ros2 run girdap_decision perception_fusion_node --ros-args --params-file "$P"

    # ── BİZİM KATMANIMIZ: kamera duba tespiti (`/perception/buoys`) ──────
    # 🔴 18.08 ÖLÇÜMÜ: bu satır yokken `/perception/buoys` YAYINCI 0 idi ve
    # kural motoru S1 · S2 · S5 ile C3'ün bir kolunu "veri yok" diye STALE
    # bırakıyordu. STALE, "ihlal yok" DEĞİLDİR — "hiç ölçülmedi" demektir;
    # yani gölün, P1/P2 puanını üreten kendi katmanımızı sınamadığı bir kör
    # noktası vardı. Bağlandıktan sonra: yayıncı 1 · abone 2 · 4,999 Hz,
    # S1 +0,2077 s · S2 +1 · S5 +1 · C3 (+0,1765 / +0,2704) — hepsi ÖLÇÜLÜR.
    #
    # `GIRDAP_SIM_KAYNAK=1` OAK-D'yi açmaz; `/perception/obstacle_map`'i ters
    # pinhole ile sahte NN kuyruğuna çevirir. Düğüm GÖVDESİ bit-birebir
    # dağıtım kodudur (bkz. §SİM KAYNAK KİPİ, algı deposunda).
    #
    # ✅ Güvenlik (kod okunarak doğrulandı, VARSAYILMADI):
    #   MOD="algi_yayin"          → cmd_vel YOK, /goal_pose YOK; karar
    #                               hattının kontrol yoluna HİÇ dokunmaz.
    #   GATE_PASSED_YAYINLA=False → gate_passed BASILMAZ. Basılsaydı FSM
    #                               İLK geçitte PARKUR2→PARKUR3'e atlardı
    #                               (P2 gider, ~40 puan) — o tuzak kapalı.
    #
    # ⚠ TEZGÂH SINIRI (dürüstlük): tespitler LiDAR engel haritasından
    # türetiliyor. Bu kip bbox→metre dönüşümünü, menzil çözümünü, geçit
    # kurmayı ve yayın sözleşmesini SINAR; YOLO'yu, VPU'yu ve blob'u
    # SINAMAZ. Füzyonun kamera↔LiDAR eşleştirmesi de burada tanım gereği
    # tutarlıdır (aynı kaynaktan türüyor) — onun gerçek sınavı bant
    # koşumudur (KONTROL 3 `--bant`), göl değil.
    # KİP: 2 = GÖRÜNTÜ (varsayılan, daha çok kod yolu koşar) · 1 = GEOMETRİK
    #   1 → tespit `/perception/obstacle_map`'ten türetilir. Hızlı ve konumlar
    #       kesin, ama görüntü taşıma · QoS uyumu · bgr8 çözme · CLAHE ·
    #       doygunluk germesi · kontur · MONO MENZİL YEDEĞİ hiç koşmaz.
    #   2 → kare `/oak/rgb/image_raw`'dan gelir ve yukarıdakilerin HEPSİ koşar;
    #       stereo bilerek YOK (z=0) ⇒ suyun stereo'yu öldürdüğü gerçek durum
    #       (08.08 bulgusu) ve onun için yazılan pinhole yedeği sınanır.
    #       Damga da GERÇEK kareden gelir ⇒ kamera↔LiDAR ms farkı anlamlı olur.
    basla algi_navigator env "GIRDAP_SIM_KAYNAK=${GIRDAP_GOL_ALGI_KIP:-2}" \
        ros2 run girdap_ida_algi duba_gecis_navigator
    echo "  + ALGI zinciri: sahte_ham_sensor → perception_lidar → perception_fusion"
    echo "  + BİZİM KATMAN: duba_gecis_navigator (SİM KAYNAK) → /perception/buoys"
fi

# ── TESLİM DOSYALARI (md 4.2) ─────────────────────────────────────────────
# Her eksik/oynatılamaz dosya 5 ceza puanı. PAR-10: 14 bag'in 13'ü
# sonlandırılmamıştı — aynı sınıf Dosya-1 mp4'ünün moov atomunu da vurur.
# Bu katman olmadan C5 (temiz kapanış) kuralı gölde HİÇ sınanamaz.
if [ "${GIRDAP_GOL_TESLIM:-0}" = "1" ]; then
    basla telemetri ros2 run girdap_decision telemetry_node --ros-args --params-file "$P"
    basla yerel_harita ros2 run girdap_decision local_map_node --ros-args --params-file "$P"
    basla lidar_kayit ros2 run girdap_decision lidar_kayit_node --ros-args --params-file "$P"
    echo "  + TESLIM: telemetry (Dosya-2) · local_map (Dosya-3) · lidar_kayit"
fi

# ── PARKUR-3 renk kapısı ──────────────────────────────────────────────────
# `kamikaze_param_node` olmadan `/girdap/mission/hedef_rengi` HİÇ yayınlanmaz
# ⇒ `p3_bekleniyor` hep False ⇒ FSM PARKUR3'e hiç geçmez. Yani P3 zinciri
# gölde tanım gereği sınanamıyordu.
if [ "${GIRDAP_GOL_P3:-0}" = "1" ]; then
    basla p3_renk ros2 run girdap_decision kamikaze_param_node --ros-args \
        --params-file "$P" -p kamikaze_target_color:="${GIRDAP_GOL_RENK:-kirmizi}"
    echo "  + PARKUR-3: kamikaze_param_node (renk ${GIRDAP_GOL_RENK:-kirmizi})"
fi

# ── DOĞRULAMA İZLEYİCİSİ (kural motoru) ───────────────────────────────────
# Salt-okur; hiçbir sistem topic'ine yazmaz. İhlalleri /girdap/dogrulama'ya
# ve kural başına /girdap/dogrulama/<KURAL>'a basar.
if [ "${GIRDAP_GOL_IZLEYICI:-0}" = "1" ]; then
    basla dogrulama ros2 run girdap_decision dogrulama_node --ros-args --params-file "$P"
    echo "  + IZLEYICI: dogrulama_node (kural motoru, salt-okur)"
fi

echo "sanal göl: $KAPI kapı · açıklık $ACIK m · aralık $ARALIK m · $ENGEL engel · dalga ${DALGA} m/s yanal + ${DALGA_YAW} rad/s yaw · başlangıç yönü ${YON0}° · algı gerçekçilik ${GERCEKCILIK} · hayalet ${HAYALET}"
[ -n "$GOL_PLANNING_EK" ] && echo "planning ek şalter: $GOL_PLANNING_EK"
