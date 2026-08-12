#!/usr/bin/env bash
# FC TEŞHİS BETİĞİ — Oturum 1, adım 1.3+1.4 (SALT OKUMA, hiçbir şey değiştirmez)
# Kullanım: mavros çalışırken (örn. ros2 launch mavros apm.launch
#   fcu_url:=serial:///dev/pixhawk:921600)  →  bash testler/fc_teshis.sh
#   ⚠ Hat hızı 12.08'de 57600'den 921600'e düzeltildi (§0.52), aygıt adı da
#   kalıcı udev symlink'i; ham `ttyUSB0` adı açılışa göre kayıyor (§0.56m).
# Çıktı: ~/girdap_logs/fc_teshis/teshis_<zaman>.txt  (ekrana da basar)
# Amaç: 12.07 FC-OLAY kök neden ayrımı (a: mod kanalı dinlenmede AUTO /
#   c: boot parametreleri / d: gaz kanalı dinlenmede yüksek).
set -u
OUT_DIR="$HOME/girdap_logs/fc_teshis"
mkdir -p "$OUT_DIR"
OUT="$OUT_DIR/teshis_$(date +%Y%m%d_%H%M%S).txt"
log() { echo "$@" | tee -a "$OUT"; }

log "=== FC TEŞHİS $(date -Is) ==="
log "--- KURAL: bu betik koşarken RC kumandaya ve tekneye DOKUNMA ---"

log ""
log "### 1) MAVROS durumu (mod + armed) — 5 sn bekle"
timeout 10 ros2 topic echo /mavros/state --once 2>&1 | tee -a "$OUT"

log ""
log "### 2) RC kanal PWM'leri (dinlenme değerleri) — 3 örnek"
for i in 1 2 3; do
  log "--- örnek $i ---"
  timeout 10 ros2 topic echo /mavros/rc/in --once 2>&1 | grep -A20 'channels' | tee -a "$OUT"
  sleep 1
done

log ""
log "### 3) GPS fix durumu"
timeout 10 ros2 topic echo /mavros/global_position/raw/fix --once 2>&1 | grep -E 'status|latitude|longitude' | tee -a "$OUT"

log ""
log "### 4) FCU parametre dökümü"
log "(önce pull — FCU'dan tam liste çekiliyor, ~30 sn sürebilir)"
timeout 90 ros2 service call /mavros/param/pull mavros_msgs/srv/ParamPull "{force_pull: true}" >/dev/null 2>&1

PARAMS=(MODE_CH MODE1 MODE2 MODE3 MODE4 MODE5 MODE6 INITIAL_MODE \
  ARMING_REQUIRE ARMING_CHECK BRD_SAFETY_DEFLT \
  FS_ACTION FS_TIMEOUT FS_GCS_ENABLE FS_THR_ENABLE FS_CRASH_CHECK \
  RCMAP_ROLL RCMAP_PITCH RCMAP_THROTTLE RCMAP_YAW \
  SERIAL2_BAUD SERIAL2_PROTOCOL MIS_RESTART \
  SERVO1_FUNCTION SERVO2_FUNCTION SERVO3_FUNCTION SERVO4_FUNCTION \
  SERVO5_FUNCTION SERVO6_FUNCTION SERVO7_FUNCTION SERVO8_FUNCTION \
  RC1_MIN RC1_TRIM RC1_MAX RC2_MIN RC2_TRIM RC2_MAX \
  RC3_MIN RC3_TRIM RC3_MAX RC5_MIN RC5_TRIM RC5_MAX \
  RC8_MIN RC8_TRIM RC8_MAX)

# ⚠ `/mavros/param/get` (mavros_msgs/srv/ParamGet) ROS 1 API'sidir — MAVROS 2
# (Humble) o servisi HİÇ açmaz; 13.08.2026'da canlı ölçüldü: `param` eklentisi
# yüklüyken bile yalnız `/mavros/param/pull` ve `/mavros/param/set` var.
# Eski çağrı bu yüzden 45 parametrenin 45'inde de BOŞ dönüyordu ve betik
# "SERIAL2_BAUD : " diye basıp geçiyordu — okuyan "parametre boş/sıfır" sanar.
# Doğru yol, MAVROS 2'nin parametreleri düğüm parametresi olarak sunması:
#   ros2 param get /mavros/param SERIAL2_BAUD  →  "Integer value is: 921"
for p in "${PARAMS[@]}"; do
  RES=$(timeout 10 ros2 param get /mavros/param "$p" 2>/dev/null | tail -1)
  case "$RES" in
    "Integer value is: "*) RES="${RES#Integer value is: }" ;;
    "Double value is: "*)  RES="${RES#Double value is: }" ;;
    "String value is: "*)  RES="${RES#String value is: }" ;;
    "Parameter not set."*) RES="YOK (uçuş kontrolcüsünde tanımsız)" ;;
    # 🔑 Boş çıktıyı ASLA değer gibi basma — sessiz sahte-yeşilin kaynağı buydu.
    "")                    RES="OKUNAMADI (mavros yanıt vermedi — bağlantı var mı?)" ;;
  esac
  log "$p : $RES"
done

log ""
log "### 5) FC'deki görev (waypoint) listesi — 0 item olmalı (temizlik SONRASI)"
timeout 10 ros2 topic echo /mavros/mission/waypoints --once 2>&1 | head -30 | tee -a "$OUT"

log ""
log "=== BİTTİ. Dosya: $OUT ==="
log "Yorumlama: MODE kanalı (MODE_CH işaret ettiği RCx) dinlenme PWM'i hangi"
log "MODE1-6 bandına düşüyor? RCMAP_THROTTLE kanalının dinlenmesi TRIM'den"
log "çok yüksekse (d) adayı. ARMING_REQUIRE=0 ya da INITIAL_MODE=10(AUTO) ise (c)."
