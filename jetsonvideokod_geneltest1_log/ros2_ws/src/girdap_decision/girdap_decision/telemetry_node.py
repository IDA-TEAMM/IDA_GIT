"""
Girdap İDA — Telemetri logger node'u (Layer 2).

Şartname 4.2 — Dosya 2:
    Telemetri CSV, ≥1 Hz, header satırlı, alanlar:
        zaman, lat, lon, hiz, roll, pitch, heading,
        hiz_setpoint, yon_setpoint, mission_state
    Görev bitiminden 20 dk içinde teslim. Geç teslim = 5 ceza puanı.

Subscribed topics:
    /mavros/global_position/global   sensor_msgs/NavSatFix       (lat/lon)
    /mavros/imu/data                 sensor_msgs/Imu             (roll/pitch/heading)
    /mavros/local_position/velocity_body  geometry_msgs/TwistStamped (hiz)
    /mavros/setpoint_velocity/cmd_vel_unstamped  geometry_msgs/Twist
                                                 (hiz_setpoint)
    /girdap/mission/current_target   geometry_msgs/PoseStamped
                                     (yon_setpoint = atan2(y, x), ENU AÇI)
    /girdap/fusion/odom              nav_msgs/Odometry           (yedek poz/hız)
    /girdap/mission/state            std_msgs/String             (durum etiketi)
    /girdap/control/thrust           std_msgs/Float32MultiArray  [T_sol, T_sag] N

Publishes:
    Yok — diske CSV yazar (16.07 kayıt düzeni: her kayıt kendi numaralı
    klasöründe, ~/girdap_logs/kayit/<N>/; boot bir kayıt açar, FC'nin gerçek
    ARM kenarı yenisini; N = en küçük boş numara; retention kayit_sakla_adet):
    - Dosya-2 (Şartname 4.2): <N>/telemetri.csv, 2 Hz, CSV_HEADER sabit.
    - Grafik CSV (T0-g, md 3.3.1.1 Ekran-2): <N>/grafik.csv, 10 Hz —
      hız+setpoint, heading+setpoint, thrust isteği. Video montajında
      Ekran-2 senkron grafikleri buradan çizilir; Dosya-2'ye thrust GİRMEZ.

Notlar:
    - CSV formatı + fsync mantığı prototype.telemetry.csv_logger'da; bu node
      yalnızca ROS 2 mesaj alanlarını cache'ler ve her yazma tick'inde
      TelemetrySample üretir. Test (test_telemetry_logger.py) aynı çekirdeği
      kullanır.
    - roll/pitch/heading /mavros/imu/data quaternion'undan (ENU yaw = heading).
      IMU henüz gelmediyse heading fusion odom'dan yedeklenir.
    - Yazma 2 Hz (≥1 Hz şartname garantisi). Her satırda os.fsync — güç
      kesintisinde en fazla son yarım saniye kaybı.
"""

from __future__ import annotations

import math
import time
from pathlib import Path
from typing import Optional

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped, Twist, TwistStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, NavSatFix
from std_msgs.msg import Float32MultiArray, String

from girdap_decision.qos_profiles import sensor_data_qos

# Setpoint'lerin anlamlı olduğu FSM durumları (F-V.2): görev aktif değilken
# (BOOT/BEKLEMEDE/TAMAMLANDI/KILL) cache'teki son istek CSV'ye YAZILMAZ —
# donuk setpoint çizgisi md 3.3.1.1 senkron grafiklerini yanıltır; boş hücre
# ekran2'de NaN boşluğu olur (sahte çizgi yok).
_SETPOINT_ACTIVE_STATES = ("PARKUR1", "PARKUR2", "PARKUR3")
from prototype.telemetry.csv_logger import (
    GRAPH_CSV_HEADER,
    GraphSample,
    TelemetryCsvLogger,
    TelemetrySample,
    next_kayit_num,
    prune_old_kayit_dirs,
    quat_to_rpy,
    utc_isoformat,
)

# GÖREV 3: duvar↔monotonic farkı tek tick'te bundan fazla değişirse sistem
# saati sıçramış demektir (NTP düzeltmesi) — CSV zaman etiketleri karışır.
_CLOCK_JUMP_WARN_S = 30.0


class TelemetryNode(Node):
    """Şartname 4.2 Dosya 2 üreticisi."""

    def __init__(self, **node_kwargs) -> None:
        # node_kwargs → parameter_overrides passthrough (test enjeksiyonu).
        super().__init__("telemetry_node", **node_kwargs)

        # --- Parametreler ---
        # GÖREV 1 (rev. Eyüp 16.07): her kayıt kendi numaralı klasöründe —
        # <kayit_dir>/<N>/telemetri.csv + grafik.csv. Boş → ~/girdap_logs/kayit
        # (MUTLAK yol; göreli yol systemd'de cwd=/ → PermissionError → Dosya-2
        # üretilmez → 5 ceza puanı). N = en küçük boş numara (silinen yeniden
        # kullanılır); boot bir kayıt açar, FC'nin gerçek ARM kenarı yenisini.
        self.declare_parameter("kayit_dir", "")
        # Eski kayıtlar bu sayıyı aşınca en eskiden silinir; <=0 → silme kapalı.
        self.declare_parameter("kayit_sakla_adet", 20)
        self.declare_parameter("log_rate_hz", 2.0)          # ≥1 Hz garanti
        # Grafik CSV (Ekran-2): MPPI control_rate ile aynı 10 Hz — 2 Hz
        # örnekleme thrust isteğini alias'lar, grafik yanıltıcı olur.
        self.declare_parameter("graph_rate_hz", 10.0)
        # B2 — setpoint/thrust kaynağı (md 3.3.1.1 Ekran-2 dürüstlüğü):
        #   "girdap" (YARIŞMA VARSAYILANI): GUIDED+MPPI. thrust = MPPI'nin
        #            /girdap/control/thrust isteği (N), hiz_setpoint = cmd_vel.
        #   "fc"     (AUTO video): görevi FC uçurur, MPPI cmd_vel BASMAZ
        #            (planning geçidi GUIDED bekler) → MPPI thrust'ını göstermek
        #            YANILTICI olur. thrust = FC servo çıkışı (/mavros/rc/out
        #            PWM → ±%100), hiz_setpoint = FC'nin WP_SPEED'i.
        self.declare_parameter("setpoint_source", "girdap")
        # ⚠ fc modunda FC'nin WP_SPEED parametresiyle SENKRON tutulmalı —
        # farklıysa Ekran-2'deki setpoint çizgisi yalan söyler.
        self.declare_parameter("fc_cruise_setpoint_mps", 1.0)
        # Skid-steer servo kanalları (masa teyidi: SERVO1=Sol, SERVO3=Sağ).
        self.declare_parameter("fc_thrust_left_ch", 1)
        self.declare_parameter("fc_thrust_right_ch", 3)
        self.declare_parameter("fc_pwm_center", 1500)     # nötr PWM (µs)
        self.declare_parameter("fc_pwm_span", 500)        # ±tam yetki (µs)
        # F-V.7: hedefe bu mesafeden yakınken yon_setpoint GÜNCELLENMEZ.
        # current_target araç-göreli ofsettir; waypoint'in üstünden geçerken
        # ofset sıfıra iner, sonra ARKADA kalır → atan2 açıyı ~180° savurur.
        # AUTO'da tekne durmadığı için bu her waypoint'te olur → Ekran-2b'nin
        # zorunlu eğrisinde çöp sıçrama (md 3.3.1.1 "görüntü net değilse
        # BAŞARISIZ"). Son geçerli açı korunur (sahte veri değil, son istek).
        self.declare_parameter("yaw_setpoint_min_dist_m", 0.5)
        # BULGU 2: kaynak susunca cache'teki son değer DONUK tekrarlanıyordu
        # (gps-kayip senaryosu: hız çizgisi sabit değerde donuyor) → hakem
        # veriyi canlı sanır. F-V.2'nin setpoint kapılama deseni sensör
        # sütunlarına da uygulanır: bilinmiyorsa BOŞ hücre, uydurma değil.
        # <=0 → bekçi kapalı (düşük hızlı mock/masa yayıncısı sütunu boşaltmasın).
        self.declare_parameter("source_timeout_s", 3.0)

        # GÖREV 2: fc modunda rc/out sağlık uyarısı bekleme süresi (sn).
        self.declare_parameter("rc_warn_after_s", 10.0)

        root = str(self.get_parameter("kayit_dir").value)
        if not root:
            root = str(Path.home() / "girdap_logs" / "kayit")
        self._kayit_root = Path(root).expanduser()
        self._kayit_keep = int(self.get_parameter("kayit_sakla_adet").value)
        # DİKKAT: rclpy Node dahili logger'ını `self._logger`'da tutar; bu isimle
        # CSV logger'ı atamak get_logger()'ı ezer → `self._csv` kullanılır.
        # Dosya-2 teslimi = aktif kaydın telemetri.csv'si (USB'ye o klasör
        # kopyalanır); grafik.csv yarışma çıktısı değil, Ekran-2 verisi.
        self._csv: Optional[TelemetryCsvLogger] = None
        self._graph_csv: Optional[TelemetryCsvLogger] = None
        self._open_new_kayit()

        self._source_timeout = float(
            self.get_parameter("source_timeout_s").value
        )

        # --- Cache (en son alınan değer her tick'te yazılır) ---
        # BULGU 2: her cache'in yanında kaynağın son duyulma zamanı (_*_t)
        # tutulur; yazarken _fresh() bayat değeri boşa çevirir.
        self._lat: Optional[float] = None
        self._lon: Optional[float] = None
        self._latlon_t: Optional[float] = None
        self._speed: Optional[float] = None
        self._speed_t: Optional[float] = None
        self._speed_from_body: bool = False       # F15.4: odom yedeği bayrağı
        self._roll: Optional[float] = None
        self._pitch: Optional[float] = None
        self._rp_t: Optional[float] = None        # roll+pitch yalnız IMU'dan
        self._heading: Optional[float] = None
        self._heading_t: Optional[float] = None   # IMU ya da odom yedeği
        self._heading_from_imu: bool = False
        self._speed_sp: Optional[float] = None
        self._setpoint_t: Optional[float] = None  # F-T.3: cmd_vel canlılığı
        self._yaw_sp: Optional[float] = None
        # F-T.3: current_target'ın SON DUYULMA anı — F-V.7 açıyı tutarken de
        # damgalanır (tazelik "kaynak canlı mı" sorusudur, "değer yeni mi" değil).
        self._target_t: Optional[float] = None
        self._mission_state: str = ""
        self._thrust_sol: Optional[float] = None  # Ekran-2c: N (girdap) | % (fc)
        self._thrust_sag: Optional[float] = None
        self._thrust_t: Optional[float] = None    # F-T.3: thrust kaynağı canlılığı

        # --- GÖREV 1/2/3 durumu (jetson_kayit_gorevi.md, 16.07) ---
        self._rc_warn_after = float(self.get_parameter("rc_warn_after_s").value)
        self._fc_armed: Optional[bool] = None     # None = FC state hiç duyulmadı
        self._armed_edge_t: Optional[float] = None
        self._connected_t: Optional[float] = None
        self._rc_msg_seen = False                 # GÖREV 2a: rc/out hiç geldi mi
        self._rc_nonzero_since_arm = False        # GÖREV 2b: seçili kanal PWM>0
        self._rc_silence_warned = False
        self._rc_zero_warned = False
        self._clock_ref: Optional[tuple] = None   # GÖREV 3: (duvar, monotonic)
        self._clock_jump_warned = False

        # --- B2: setpoint/thrust kaynağı ---
        src = str(self.get_parameter("setpoint_source").value).lower()
        if src not in ("girdap", "fc"):
            self.get_logger().warn(
                f"setpoint_source='{src}' geçersiz → 'girdap' varsayılanına "
                "düşüldü (yarışma modu: MPPI thrust'ı + cmd_vel setpoint'i)"
            )
            src = "girdap"
        self._setpoint_source = src
        self._fc_cruise = float(self.get_parameter("fc_cruise_setpoint_mps").value)
        self._fc_left_ch = int(self.get_parameter("fc_thrust_left_ch").value)
        self._fc_right_ch = int(self.get_parameter("fc_thrust_right_ch").value)
        self._fc_pwm_center = int(self.get_parameter("fc_pwm_center").value)
        self._fc_pwm_span = max(1, int(self.get_parameter("fc_pwm_span").value))
        self._rc_len_warned = False
        self._sub_rc = None
        self._yaw_sp_min_dist = float(
            self.get_parameter("yaw_setpoint_min_dist_m").value
        )

        # --- Subscribers ---
        # mavros sensör topic'leri BEST_EFFORT (sensor_data) yayınlar; RELIABLE
        # abone olursak mesaj gelmez (QoS uyumsuz). Setpoint/odom/state internal
        # ve RELIABLE olduğundan varsayılan depth=10 yeterli.
        self._sub_gps = self.create_subscription(
            NavSatFix, "/mavros/global_position/global",
            self._on_gps, sensor_data_qos(),
        )
        self._sub_imu = self.create_subscription(
            Imu, "/mavros/imu/data", self._on_imu, sensor_data_qos()
        )
        self._sub_vel = self.create_subscription(
            TwistStamped, "/mavros/local_position/velocity_body",
            self._on_vel_body, sensor_data_qos(),
        )
        # girdap modunda hız setpoint'i MPPI'nin cmd_vel isteğidir; fc modunda
        # cmd_vel HİÇ yayınlanmaz (AUTO'da planning geçidi kapalı) → abone olma.
        self._sub_setpoint = None
        if self._setpoint_source == "girdap":
            self._sub_setpoint = self.create_subscription(
                Twist, "/mavros/setpoint_velocity/cmd_vel_unstamped",
                self._on_setpoint, 10,
            )
        # F-V.1 (md 3.3.1.1 Ekran-2b): yon_setpoint bir AÇI olmalı; cmd_vel
        # angular.z yaw HIZIdır (rad/s) — o yüzden setpoint açısı hedeften
        # türetilir: current_target araç-göreli ENU ofset → atan2(y, x).
        self._sub_target = self.create_subscription(
            PoseStamped, "/girdap/mission/current_target", self._on_target, 10
        )
        self._sub_odom = self.create_subscription(
            Odometry, "/girdap/fusion/odom", self._on_odom, 10
        )
        self._sub_state = self.create_subscription(
            String, "/girdap/mission/state", self._on_mission_state, 10
        )
        # GÖREV 1: rotasyon kenarı FC'nin GERÇEK arm'ından (FSM ARM durumu
        # köprünün İSTEĞİNİ yansıtır, FC'nin kabulünü değil). Lazy import —
        # RCOut ile aynı CI hijyeni (modül importu mavros_msgs'siz çalışır).
        from mavros_msgs.msg import State as MavState
        self._sub_fc_state = self.create_subscription(
            MavState, "/mavros/state", self._on_fc_state, 10
        )
        # Ekran-2c kuvvet isteği: girdap → MPPI thrust'ı (N); fc → FC servo
        # çıkışı (/mavros/rc/out PWM → %). İkisi AYNI ANDA dinlenmez: AUTO'da
        # MPPI thrust'ı hesaplanmaya devam eder ama araca GİTMEZ → grafikte
        # göstermek md 3.3.1.1 anlamında yanıltıcı olur.
        self._sub_thrust = None
        if self._setpoint_source == "girdap":
            self._sub_thrust = self.create_subscription(
                Float32MultiArray, "/girdap/control/thrust", self._on_thrust, 10
            )
        else:
            from mavros_msgs.msg import RCOut      # lazy: yalnız fc modunda
            self._sub_rc = self.create_subscription(
                RCOut, "/mavros/rc/out", self._on_rc_out, sensor_data_qos()
            )

        # --- Yazma timer'ları ---
        rate = float(self.get_parameter("log_rate_hz").value)
        self._timer = self.create_timer(1.0 / rate, self._on_write)
        graph_rate = float(self.get_parameter("graph_rate_hz").value)
        self._graph_timer = self.create_timer(
            1.0 / graph_rate, self._on_graph_write
        )

        if self._setpoint_source == "fc":
            kaynak = (
                f"fc (AUTO video: thrust=/mavros/rc/out %, kanal "
                f"{self._fc_left_ch}/{self._fc_right_ch}; hiz_setpoint="
                f"{self._fc_cruise} m/s — FC WP_SPEED ile SENKRON OLMALI)"
            )
        else:
            kaynak = "girdap (yarışma: MPPI thrust'ı N + cmd_vel setpoint'i)"
        self.get_logger().info(
            f"telemetry_node aktif → {self._csv.path} (rate={rate} Hz), "
            f"grafik → {self._graph_csv.path} (rate={graph_rate} Hz), "
            f"setpoint_source={kaynak}"
        )

    # ----- subscriber callback'leri -----

    def _on_gps(self, msg: NavSatFix) -> None:
        if msg.status.status >= 0:               # fix yoksa (-1) yazma
            self._lat = msg.latitude
            self._lon = msg.longitude
            self._latlon_t = self._now()         # BULGU 2: tazelik damgası

    def _on_imu(self, msg: Imu) -> None:
        q = msg.orientation
        roll, pitch, yaw = quat_to_rpy(q.x, q.y, q.z, q.w)
        self._roll = roll
        self._pitch = pitch
        # mavros IMU yaw'ı ENU heading olarak verir → doğrudan kullan
        self._heading = yaw
        self._heading_from_imu = True
        now = self._now()                        # BULGU 2
        self._rp_t = now
        self._heading_t = now

    def _on_vel_body(self, msg: TwistStamped) -> None:
        v = msg.twist.linear
        self._speed = math.sqrt(v.x * v.x + v.y * v.y)
        self._speed_from_body = True
        self._speed_t = self._now()              # BULGU 2

    def _on_setpoint(self, msg: Twist) -> None:
        # angular.z yaw HIZI — yon_setpoint'e YAZILMAZ (F-V.1).
        self._speed_sp = msg.linear.x
        self._setpoint_t = self._now()           # F-T.3

    def _on_target(self, msg: PoseStamped) -> None:
        # Araç-göreli ENU ofsetten istenen rota açısı (heading ile aynı
        # konvansiyon). IDLE/COMPLETE'te yayın durur → son açı cache kalır.
        self._target_t = self._now()             # F-T.3: F-V.7 tutuşunda da damgala
        p = msg.pose.position
        # F-V.7: waypoint'in üstündeyken ofset ~0 → atan2 anlamsız açı üretir.
        if math.hypot(p.x, p.y) < self._yaw_sp_min_dist:
            return                                # son geçerli açıyı koru
        self._yaw_sp = math.atan2(p.y, p.x)

    def _on_odom(self, msg: Odometry) -> None:
        # IMU henüz gelmediyse heading'i fusion odom'dan yedekle
        if not self._heading_from_imu:
            q = msg.pose.pose.orientation
            self._heading = 2.0 * math.atan2(q.z, q.w)
            self._heading_t = self._now()        # BULGU 2: yedek kaynak da tazeler
        # F15.4: velocity_body gelmiyorsa hızı odom twist'ten yedekle —
        # yoksa Ekran-2 hız grafiği boş kalır (Odometry twist child frame'de).
        if not self._speed_from_body:
            v = msg.twist.twist.linear
            self._speed = math.sqrt(v.x * v.x + v.y * v.y)
            self._speed_t = self._now()          # BULGU 2: yedek kaynak da tazeler

    def _on_mission_state(self, msg: String) -> None:
        self._mission_state = msg.data

    # ----- GÖREV 1: FC arm kenarında CSV rotasyonu -----

    def _on_fc_state(self, msg) -> None:             # mavros_msgs/msg/State
        if bool(msg.connected) and self._connected_t is None:
            self._connected_t = self._now()          # GÖREV 2a başlangıç damgası
        armed = bool(msg.armed)
        prev = self._fc_armed
        self._fc_armed = armed
        # Kenar = GÖZLENEN False→True. İlk örnek True ise (servis görev
        # ortasında açıldı) rotasyon YOK — koşan görevin dosyası bölünmesin.
        if armed and prev is False:
            self._armed_edge_t = self._now()
            self._rc_nonzero_since_arm = False       # GÖREV 2b: oturum başı reset
            self._rc_zero_warned = False
            self._open_new_kayit()
            self.get_logger().info(
                f"ARM algılandı → yeni kayıt: {self._csv.path.parent}"
            )

    def _open_new_kayit(self) -> None:
        """Sıradaki numaralı kayıt klasörünü açar (telemetri+grafik çifti)
        ve retention uygular. Kayıt boot'tan sürer (görev başını kaçırma
        riski sıfır); ARM kenarı yenisini açar → görev verisi temiz klasörde,
        run_ekran2'nin "en yeni" seçimi kendiliğinden doğru olur. Koşu zamanı
        dosya adında değil, CSV'nin ilk `zaman` satırındadır."""
        for logger in (self._csv, self._graph_csv):
            if logger is None:
                continue
            try:
                logger.close()
            except Exception as e:                   # rotasyon yazmayı durdurmasın
                self.get_logger().error(f"CSV kapatma hatası: {e}")
        kayit = self._kayit_root / str(next_kayit_num(self._kayit_root))
        self._csv = TelemetryCsvLogger(kayit, filename="telemetri.csv")
        self._graph_csv = TelemetryCsvLogger(
            kayit, filename="grafik.csv", header=GRAPH_CSV_HEADER
        )
        silinen = prune_old_kayit_dirs(
            self._kayit_root, self._kayit_keep, keep_dirs=(kayit,)
        )
        if silinen:
            self.get_logger().info(
                f"eski kayıtlar silindi (kayit_sakla_adet={self._kayit_keep}): "
                + ", ".join(p.name for p in silinen)
            )

    def _on_thrust(self, msg: Float32MultiArray) -> None:
        if len(msg.data) >= 2:
            self._thrust_sol = float(msg.data[0])
            self._thrust_sag = float(msg.data[1])
            self._thrust_t = self._now()         # F-T.3

    # ----- B2: FC servo çıkışı (fc modu) -----

    def _pwm_to_pct(self, pwm: int) -> Optional[float]:
        """Servo PWM (µs) → ±%100 kuvvet isteği. Pasif kanal (PWM≤0) → None.

        PWM=0, FC'nin o kanala çıkış VERMEDİĞİ anlamına gelir (disarm/kanal
        tanımsız). Merkezden fark alırsak bu -%300 → kırpmayla -%100 olurdu:
        grafikte "tam geri" görünen sahte bir çizgi. Boş bırakmak dürüst olan.
        """
        if pwm <= 0:
            return None
        pct = (pwm - self._fc_pwm_center) / self._fc_pwm_span * 100.0
        return max(-100.0, min(100.0, pct))

    def _on_rc_out(self, msg) -> None:               # mavros_msgs/msg/RCOut
        self._rc_msg_seen = True                     # GÖREV 2a: akış var
        ch = msg.channels
        idx_sol = self._fc_left_ch - 1               # kanal no 1-tabanlı
        idx_sag = self._fc_right_ch - 1
        if len(ch) <= max(idx_sol, idx_sag):
            if not self._rc_len_warned:
                self._rc_len_warned = True
                self.get_logger().warn(
                    f"/mavros/rc/out {len(ch)} kanal — sol={self._fc_left_ch}/"
                    f"sağ={self._fc_right_ch} okunamıyor (Ekran-2c boş kalır)"
                )
            return
        self._thrust_sol = self._pwm_to_pct(ch[idx_sol])
        self._thrust_sag = self._pwm_to_pct(ch[idx_sag])
        if ch[idx_sol] > 0 or ch[idx_sag] > 0:
            self._rc_nonzero_since_arm = True        # GÖREV 2b: kanal canlandı
        # F-T.3: mesaj geldi = kaynak canlı (PWM=0 → değer None olsa bile;
        # o ayrım _pwm_to_pct'nin işi, tazelik ayrı soru).
        self._thrust_t = self._now()

    # ----- yazma -----

    def _now(self) -> float:
        # F-M.10: tazelik damgaları GÖRELİ süre — duvar saati sıçrayınca taze
        # kaynaklar donuk görünüp CSV sütunları sahte boşalıyordu. CSV zaman
        # etiketleri duvar saatinde KALIR (md 4.2); F-T.6 sıçramada uyarır.
        return time.monotonic()

    def _fresh(
        self, value: Optional[float], t: Optional[float]
    ) -> Optional[float]:
        """BULGU 2: kaynak `source_timeout_s`'ten uzun susmuşsa None döner —
        donuk son değer yerine boş hücre. `source_timeout_s <= 0` → kapalı."""
        if self._source_timeout <= 0.0:
            return value
        if t is None or (self._now() - t) > self._source_timeout:
            return None
        return value

    def _mission_active(self) -> bool:
        """F-V.2: setpoint sütunları yalnız görev aktifken yazılır."""
        return self._mission_state in _SETPOINT_ACTIVE_STATES

    def _rc_health_message(self) -> Optional[str]:
        """GÖREV 2: fc modunda rc/out sağlık teşhisi — BİR KEZlik uyarı ya da None.

        (a) FC bağlı ama rc/out HİÇ akmıyor → stream sorunu (SRx_RC_CHAN).
        (b) rc/out akıyor ama seçili kanallar ARM'dan beri hep PWM=0 →
            kanal eşleşmesi/safety sorunu (16.07 masa vakası: 8 Hz akış vardı,
            kanal 1/3 sıfırdı — salt-sessizlik uyarısı bunu YAKALAMAZDI).
        Disarm'dayken kanal 0 NORMALDİR → (b) yalnız arm oturumunda bakar.
        """
        if self._setpoint_source != "fc":
            return None
        now = self._now()
        if (not self._rc_silence_warned
                and not self._rc_msg_seen
                and self._connected_t is not None
                and now - self._connected_t >= self._rc_warn_after):
            self._rc_silence_warned = True
            return (
                "fc modu: /mavros/rc/out akmıyor — Ekran-2c thrust boş kalacak "
                "(FC SRx_RC_CHAN stream hızını kontrol et)"
            )
        if (not self._rc_zero_warned
                and self._rc_msg_seen
                and not self._rc_nonzero_since_arm
                and self._fc_armed
                and self._armed_edge_t is not None
                and now - self._armed_edge_t >= self._rc_warn_after):
            self._rc_zero_warned = True
            return (
                f"fc modu: rc/out akıyor ama kanal {self._fc_left_ch}/"
                f"{self._fc_right_ch} ARM'dan beri PWM=0 — SERVOx_FUNCTION "
                "eşleşmesini ve safety'yi kontrol et (Ekran-2c thrust boş kalır)"
            )
        return None

    def _check_clock_jump(
        self, wall_now: float, mono_now: float
    ) -> Optional[float]:
        """GÖREV 3: duvar↔monotonic farkı önceki tick'e göre >30 sn değiştiyse
        sıçrama miktarını (sn, işaretli) döner. Damgalar DÜZELTİLMEZ (Dosya-2
        duvar saati ister); amaç operatörün oturumu şüpheli sayması."""
        if self._clock_ref is None:
            self._clock_ref = (wall_now, mono_now)
            return None
        jump = (wall_now - self._clock_ref[0]) - (mono_now - self._clock_ref[1])
        self._clock_ref = (wall_now, mono_now)
        if abs(jump) > _CLOCK_JUMP_WARN_S:
            return jump
        return None

    def _hiz_setpoint(self, active: bool) -> Optional[float]:
        """Hız setpoint'i: girdap → cmd_vel isteği, fc → FC seyir hızı.

        fc modunda FC kendi WP_SPEED'ini hedefler; MAVLink'te anlık hız
        setpoint'i yayınlanmaz → sabit seyir hızı (fc_cruise_setpoint_mps)
        savunulabilir tek dürüst değerdir. F-V.2 kuralı korunur: görev aktif
        değilken hiçbir setpoint yazılmaz.
        """
        if not active:
            return None
        if self._setpoint_source == "fc":
            return self._fc_cruise               # config sabiti — tazelik yok
        # F-T.3: planning ölürse cmd_vel donuk kalmasın (girdap/yarışma modu).
        return self._fresh(self._speed_sp, self._setpoint_t)

    def _on_write(self) -> None:
        # GÖREV 2: rc/out sağlık teşhisi (yalnız fc modu, bir kez uyarır).
        health = self._rc_health_message()
        if health:
            self.get_logger().warn(health)
        # GÖREV 3: saat sıçraması tespiti (bir kez uyarır, damga düzeltilmez).
        jump = self._check_clock_jump(time.time(), time.monotonic())
        if jump is not None and not self._clock_jump_warned:
            self._clock_jump_warned = True
            self.get_logger().warn(
                f"sistem saati sıçradı ({jump:+.0f} sn) — CSV zaman etiketleri "
                "karışık olabilir; bu oturumun kayıtlarını şüpheli say "
                "(çekim öncesi `timedatectl` kontrolü runbook §0-A)"
            )
        active = self._mission_active()
        sample = TelemetrySample(
            lat=self._fresh(self._lat, self._latlon_t),
            lon=self._fresh(self._lon, self._latlon_t),
            hiz=self._fresh(self._speed, self._speed_t),
            roll=self._fresh(self._roll, self._rp_t),
            pitch=self._fresh(self._pitch, self._rp_t),
            heading=self._fresh(self._heading, self._heading_t),
            hiz_setpoint=self._hiz_setpoint(active),
            # F-T.3: aktif + kaynak canlı. F-V.7 tutuşu bozulmaz — mesaj
            # aktıkça _target_t tazelenir, yalnız DEĞER tutulur.
            yon_setpoint=(
                self._fresh(self._yaw_sp, self._target_t) if active else None
            ),
            mission_state=self._mission_state,
        )
        self._csv.write_sample(utc_isoformat(), sample)

    def _on_graph_write(self) -> None:
        active = self._mission_active()
        sample = GraphSample(
            hiz=self._fresh(self._speed, self._speed_t),
            hiz_setpoint=self._hiz_setpoint(active),
            heading=self._fresh(self._heading, self._heading_t),
            yon_setpoint=(
                self._fresh(self._yaw_sp, self._target_t) if active else None
            ),
            # F-T.3: FC/planning ölünce Ekran-2c thrust eğrisi donuk akmasın
            # (hız/heading F-T.1 ile boşalırken thrust'ın akması tutarsızdı).
            thrust_sol=self._fresh(self._thrust_sol, self._thrust_t),
            thrust_sag=self._fresh(self._thrust_sag, self._thrust_t),
        )
        self._graph_csv.write_sample(utc_isoformat(), sample)

    # ----- yaşam döngüsü -----

    def destroy_node(self) -> bool:
        for logger in (self._csv, self._graph_csv):
            try:
                logger.close()
            except Exception as e:               # yıkım sırasında loglama
                self.get_logger().error(f"CSV kapatma hatası: {e}")
        return super().destroy_node()


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = TelemetryNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
