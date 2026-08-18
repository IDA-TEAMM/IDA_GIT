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
AR="-p ariza_poz_sicramasi_m:=${GIRDAP_ARIZA_SICRAMA:-0.0}
    -p ariza_poz_sicrama_orani:=${GIRDAP_ARIZA_SICRAMA_ORAN:-0.0}
    -p ariza_poz_nan_orani:=${GIRDAP_ARIZA_NAN:-0.0}
    -p ariza_damga_kaydirma_s:=${GIRDAP_ARIZA_DAMGA:-0.0}
    -p ariza_kadans_bolen:=${GIRDAP_ARIZA_KADANS:-1}
    -p ariza_kesinti_t_s:=${GIRDAP_ARIZA_KESINTI:-0.0}
    -p ariza_govde_yansimasi_m:=${GIRDAP_ARIZA_GOVDE:-0.0}"

basla sanal_gol python3 "$S/sanal_gol.py" --ros-args $SG_REMAP $AR \
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
    basla ham_sensor python3 "$S/sahte_ham_sensor.py"
    basla p_lidar  ros2 run girdap_decision perception_lidar_node --ros-args --params-file "$P"
    basla p_fusion ros2 run girdap_decision perception_fusion_node --ros-args --params-file "$P"
    echo "  + ALGI zinciri: sahte_ham_sensor → perception_lidar → perception_fusion"
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
