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
    Yok — diske CSV yazar:
    - Dosya-2 (Şartname 4.2): telemetri_<UTC>.csv, 2 Hz, CSV_HEADER sabit.
    - Grafik CSV (T0-g, md 3.3.1.1 Ekran-2): grafik_<UTC>.csv, 10 Hz —
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
    quat_to_rpy,
    utc_isoformat,
    utc_timestamp,
)


class TelemetryNode(Node):
    """Şartname 4.2 Dosya 2 üreticisi."""

    def __init__(self, **node_kwargs) -> None:
        # node_kwargs → parameter_overrides passthrough (test enjeksiyonu).
        super().__init__("telemetry_node", **node_kwargs)

        # --- Parametreler ---
        # Boş → ~/girdap_logs/telemetry (MUTLAK yol). Göreli yol KULLANMA:
        # csv_logger expanduser göreli yolu çözmez → systemd'de cwd=/ →
        # mkdir(/data/...) PermissionError → Dosya-2 üretilmez → 5 ceza puanı.
        # local_map_node ile aynı desen (Şartname 4.2 Dosya-3).
        self.declare_parameter("csv_output_dir", "")
        self.declare_parameter("log_rate_hz", 2.0)          # ≥1 Hz garanti
        # Grafik CSV (Ekran-2): MPPI control_rate ile aynı 10 Hz — 2 Hz
        # örnekleme thrust isteğini alias'lar, grafik yanıltıcı olur.
        self.declare_parameter("graph_output_dir", "")
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
        # BULGU 2 (Yahya, son_kod video koşul matrisi 2026-07-14): kaynak
        # susunca (GPS/IMU/hız) cache'teki son değer DONUK tekrarlanıyordu —
        # veri hâlâ canlıymış izlenimi. F-V.2'nin setpoint kapılama desenini
        # sensör alanlarına da uygular. <=0 → bekçi kapalı (mock/masa testi).
        self.declare_parameter("source_timeout_s", 3.0)

        out_dir = str(self.get_parameter("csv_output_dir").value)
        if not out_dir:
            out_dir = str(Path.home() / "girdap_logs" / "telemetry")
        # DİKKAT: rclpy Node dahili logger'ını `self._logger`'da tutar; bu isimle
        # CSV logger'ı atamak get_logger()'ı ezer → `self._csv` kullanılır.
        self._csv = TelemetryCsvLogger(out_dir)

        # Grafik CSV ayrı dizinde — Dosya-2 teslim klasörüne karışmasın
        # (USB'ye yalnız telemetry/ kopyalanır, grafik yarışma çıktısı değil).
        graph_dir = str(self.get_parameter("graph_output_dir").value)
        if not graph_dir:
            graph_dir = str(Path.home() / "girdap_logs" / "grafik")
        self._graph_csv = TelemetryCsvLogger(
            graph_dir,
            filename=f"grafik_{utc_timestamp()}.csv",
            header=GRAPH_CSV_HEADER,
        )

        # --- Cache (en son alınan değer her tick'te yazılır) ---
        self._lat: Optional[float] = None
        self._lon: Optional[float] = None
        self._speed: Optional[float] = None
        self._speed_from_body: bool = False       # F15.4: odom yedeği bayrağı
        self._roll: Optional[float] = None
        self._pitch: Optional[float] = None
        self._heading: Optional[float] = None
        self._heading_from_imu: bool = False
        self._speed_sp: Optional[float] = None
        self._yaw_sp: Optional[float] = None
        self._mission_state: str = ""
        self._thrust_sol: Optional[float] = None  # Ekran-2c: N (girdap) | % (fc)
        self._thrust_sag: Optional[float] = None

        # --- BULGU 2: kaynak-tazelik zaman damgaları (None = hiç gelmedi) ---
        self._latlon_t: Optional[float] = None
        self._speed_t: Optional[float] = None
        self._rp_t: Optional[float] = None        # roll/pitch, yalnız IMU
        self._heading_t: Optional[float] = None
        # F-P.5 (robustness taraması, 2026-07-15): BULGU 2'nin bekçisi lat/
        # lon/hiz/roll/pitch/heading'i kapsıyordu ama thrust/setpoint
        # sütunlarını KAPSAMIYORDU — planning_node görev ortasında çökerse
        # (ör. F10.1'in yakalamadığı beklenmedik bir istisna) Ekran-2/Dosya-2
        # son thrust/setpoint değerini SONSUZA DEK donuk yazmaya devam eder,
        # tam BULGU 2'nin önlemeye çalıştığı yanıltıcı-veri sınıfı.
        self._speed_sp_t: Optional[float] = None
        self._yaw_sp_t: Optional[float] = None
        self._thrust_t: Optional[float] = None
        self._source_timeout = float(
            self.get_parameter("source_timeout_s").value
        )

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
            self._latlon_t = self._now()

    def _on_imu(self, msg: Imu) -> None:
        q = msg.orientation
        roll, pitch, yaw = quat_to_rpy(q.x, q.y, q.z, q.w)
        self._roll = roll
        self._pitch = pitch
        self._rp_t = self._now()
        # mavros IMU yaw'ı ENU heading olarak verir → doğrudan kullan
        self._heading = yaw
        self._heading_from_imu = True
        self._heading_t = self._now()

    def _on_vel_body(self, msg: TwistStamped) -> None:
        v = msg.twist.linear
        self._speed = math.sqrt(v.x * v.x + v.y * v.y)
        self._speed_from_body = True
        self._speed_t = self._now()

    def _on_setpoint(self, msg: Twist) -> None:
        # angular.z yaw HIZI — yon_setpoint'e YAZILMAZ (F-V.1).
        self._speed_sp = msg.linear.x
        self._speed_sp_t = self._now()

    def _on_target(self, msg: PoseStamped) -> None:
        # Araç-göreli ENU ofsetten istenen rota açısı (heading ile aynı
        # konvansiyon). IDLE/COMPLETE'te yayın durur → son açı cache kalır.
        # F-P.5: zaman damgası F-V.7 erken-dönüşünden ÖNCE — mesaj geldi
        # (kaynak yaşıyor), açı güncellenmeyebilir ama tazelik öyle değil.
        self._yaw_sp_t = self._now()
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
            self._heading_t = self._now()
        # F15.4: velocity_body gelmiyorsa hızı odom twist'ten yedekle —
        # yoksa Ekran-2 hız grafiği boş kalır (Odometry twist child frame'de).
        if not self._speed_from_body:
            v = msg.twist.twist.linear
            self._speed = math.sqrt(v.x * v.x + v.y * v.y)
            self._speed_t = self._now()

    def _on_mission_state(self, msg: String) -> None:
        self._mission_state = msg.data

    def _on_thrust(self, msg: Float32MultiArray) -> None:
        if len(msg.data) >= 2:
            self._thrust_sol = float(msg.data[0])
            self._thrust_sag = float(msg.data[1])
            self._thrust_t = self._now()

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
        self._thrust_t = self._now()

    # ----- yazma -----

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _fresh(self, value: Optional[float], t: Optional[float]) -> Optional[float]:
        """BULGU 2: kaynak `source_timeout_s`'ten uzun süredir susmuşsa None
        döner — donuk son değer yazmak yerine boş hücre (F-V.2 ile aynı
        dürüstlük ilkesi, setpoint yerine sensör alanlarına uygulanır).
        `source_timeout_s<=0` → bekçi kapalı (mock/masa testi)."""
        if self._source_timeout <= 0.0:
            return value
        if t is None or (self._now() - t) > self._source_timeout:
            return None
        return value

    def _mission_active(self) -> bool:
        """F-V.2: setpoint sütunları yalnız görev aktifken yazılır."""
        return self._mission_state in _SETPOINT_ACTIVE_STATES

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
            return self._fc_cruise    # sabit param, kaynak-tazelik uygulanmaz
        # F-P.5: girdap modunda cmd_vel kaynağı (planning_node) susarsa
        # donuk son isteği yazma.
        return self._fresh(self._speed_sp, self._speed_sp_t)

    def _on_write(self) -> None:
        active = self._mission_active()
        sample = TelemetrySample(
            lat=self._fresh(self._lat, self._latlon_t),
            lon=self._fresh(self._lon, self._latlon_t),
            hiz=self._fresh(self._speed, self._speed_t),
            roll=self._fresh(self._roll, self._rp_t),
            pitch=self._fresh(self._pitch, self._rp_t),
            heading=self._fresh(self._heading, self._heading_t),
            hiz_setpoint=self._hiz_setpoint(active),
            yon_setpoint=self._fresh(self._yaw_sp, self._yaw_sp_t) if active else None,
            mission_state=self._mission_state,
        )
        if not self._csv.write_sample(utc_isoformat(), sample):
            self.get_logger().error(
                "Dosya-2 CSV yazma hatası (disk dolu olabilir) — bu satır "
                "atlandı", throttle_duration_sec=5.0)

    def _on_graph_write(self) -> None:
        active = self._mission_active()
        sample = GraphSample(
            hiz=self._fresh(self._speed, self._speed_t),
            hiz_setpoint=self._hiz_setpoint(active),
            heading=self._fresh(self._heading, self._heading_t),
            yon_setpoint=self._fresh(self._yaw_sp, self._yaw_sp_t) if active else None,
            thrust_sol=self._fresh(self._thrust_sol, self._thrust_t),
            thrust_sag=self._fresh(self._thrust_sag, self._thrust_t),
        )
        if not self._graph_csv.write_sample(utc_isoformat(), sample):
            self.get_logger().error(
                "Ekran-2 grafik CSV yazma hatası (disk dolu olabilir) — bu "
                "satır atlandı", throttle_duration_sec=5.0)

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
