"""
Girdap İDA — SANAL GÖL: kapalı döngü uçtan uca sınama.

Kaptanın sorusu: *"MP'de waypoint verince Jetson'daki kodlarımız aracılığıyla
gitmiyor — bunu test edebiliyor muyuz? Gölde sanal ortam ayarla, parkuru
yapmaya çalışsın, veriyi de MP'den veriyormuş gibi ver."*

Bu düğüm SAHTE DONANIM + SAHTE MISSION PLANNER + SAHTE ALGI'dır. Karar
yığınının GERÇEK düğümleri (fusion / mission_manager / fsm / planning /
mavros_bridge) hiç değiştirilmeden koşar. Döngü KAPALIDIR: yığının bastığı
`cmd_vel` tekneyi hareket ettirir, hareket eden tekne yeni GPS/IMU üretir.

    /mavros/setpoint_velocity/cmd_vel_unstamped ──► [tekne modeli] ──┐
                                                                      │
    ┌─────────────────────────────────────────────────────────────────┘
    ├─► /mavros/global_position/global   (NavSatFix, 5 Hz)
    ├─► /mavros/imu/data                 (Imu, 50 Hz)
    ├─► /mavros/local_position/velocity_body (TwistStamped, 50 Hz)
    ├─► /mavros/state                    (armed+GUIDED, 2 Hz)
    ├─► /mavros/mission/waypoints        ("Mission Planner görevi", 1 Hz)
    ├─► /mavros/mission/reached          (varışta — F-V.8 senkronu)
    ├─► /perception/obstacle_map         (PoseArray, base_link, 10 Hz)
    └─► /perception/classified_obstacles (Detection3DArray, 10 Hz)

⚠ İZOLASYON: `ROS_DOMAIN_ID` canlı yığından (42) FARKLI verilmeli — gerçek
Pixhawk'a bağlı yığınla aynı alanda koşarsa sahte state/cmd_vel gerçek araca
karışır.
"""

from __future__ import annotations

import math
import sys

import rclpy
from geometry_msgs.msg import Pose, PoseArray, PoseStamped, Twist, TwistStamped
from mavros_msgs.msg import State, Waypoint, WaypointList, WaypointReached
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from sensor_msgs.msg import Imu, NavSatFix, NavSatStatus
from vision_msgs.msg import (
    Detection3D,
    Detection3DArray,
    ObjectHypothesisWithPose,
)

R_DUNYA = 6378137.0
KENAR_SINIF = "0"          # turuncu parkur kenarı (camera_buoys.CLASS_PARKUR_KENARI)
BILINMEYEN = "99"          # füzyon sözleşmesi: eşleşmeyen küme


class SanalGol(Node):
    def __init__(self) -> None:
        super().__init__("sanal_gol")
        # --- Göl ve parkur ------------------------------------------------
        self.declare_parameter("lat0", 40.7162000)
        self.declare_parameter("lon0", 31.5247500)
        self.lat0 = float(self.get_parameter("lat0").value)
        self.lon0 = float(self.get_parameter("lon0").value)

        # ═══ PARKUR — GERÇEK GEOMETRİ (§0.17b, `parkur_nihai.world`) ═══
        # Varsayılanlar ölçülmüş dosyadan: 8 P1 kapısı, ortalar x=6..34'te 4 m
        # aralıkla zigzag, KAPI AÇIKLIĞI 12 m (kaptan teyidi). Güzergah
        # noktaları GN1-GN4 kapı ortalarından KAÇIK — §0.17c'nin ölçtüğü asıl
        # zorluk bu (kaçıklık 2,0-6,4 m; ham noktaya sürülürse en az 3 kapıdan
        # bandın DIŞINDAN geçilir ve puan gider).
        # Hepsi ROS parametresi: kapı sayısı/açıklığı/aralığı denemek için.
        self.declare_parameter("kapi_sayisi", 8)
        self.declare_parameter("kapi_acikligi_m", 12.0)
        self.declare_parameter("kapi_araligi_m", 4.0)
        self.declare_parameter("zigzag_m", 5.0)
        self.declare_parameter("gercek_gn", True)      # §0.17b'nin kaçık GN'leri
        n = int(self.get_parameter("kapi_sayisi").value)
        acik = float(self.get_parameter("kapi_acikligi_m").value)
        aralik = float(self.get_parameter("kapi_araligi_m").value)
        zig = float(self.get_parameter("zigzag_m").value)

        # Kapı ortaları: gerçek dosyadaki zigzag deseni (0, +z, 0, −z, …)
        desen = [0.0, zig, 0.0, -zig]
        self.kapilar = []
        for i in range(n):
            gx = desen[i % 4]
            gy = 6.0 + i * aralik
            self.kapilar.append((gx, gy, acik / 2.0))

        if bool(self.get_parameter("gercek_gn").value) and n == 8:
            # §0.17b'nin GN'leri (x↔y çevrilmiş: bizim eksende y=ileri)
            self.gorev_xy = [(0.0, 2.0), (5.0, 12.0), (-5.0, 20.0), (5.0, 32.0)]
        else:
            # Kapı ortalarından türet ama KAÇIKLIK KORU (2 m) — ham noktaya
            # sürmenin bandın dışına çıkardığı özellik kaybolmasın.
            self.gorev_xy = [
                (gx + (2.0 if i % 2 else -2.0), gy)
                for i, (gx, gy, _) in enumerate(self.kapilar)
            ]

        # Sarı engeller — §0.17b: parkur_nihai.world'de y=±2 bandında
        self.declare_parameter("engel_sayisi", 4)
        m = int(self.get_parameter("engel_sayisi").value)
        son_y = self.kapilar[-1][1] if self.kapilar else 30.0
        self.engeller = [
            (2.0 if i % 2 else -2.0, son_y + 6.0 + i * 4.0) for i in range(m)
        ]

        # --- Tekne durumu (ENU, göl orijinine göre) -----------------------
        self.x, self.y, self.psi = 0.0, 0.0, math.radians(90.0)   # burun kuzeye
        self.u, self.r = 0.0, 0.0
        self.cmd_u, self.cmd_r = 0.0, 0.0
        self.son_cmd_t = None
        self.varilan = -1
        self.t = 0.0
        self.cmd_sayaci = 0
        self.bosluk_sayaci = 0
        self.en_uzun_bosluk = 0.0
        # Şartname çarpmayı CEZALANDIRIYOR (P2: −30×Ç2 / (KD2+ED2)) → açıklık
        # ölçülür. Gövde yarı genişliği 0,3925 m + duba yarıçapı 0,15 m.
        self.en_yakin_m = 99.0
        self.carpma = 0
        self._temas = set()

        # --- Yayıncılar ----------------------------------------------------
        self.p_gps = self.create_publisher(NavSatFix, "/mavros/global_position/global", 10)
        self.p_imu = self.create_publisher(Imu, "/mavros/imu/data", 10)
        self.p_vel = self.create_publisher(TwistStamped, "/mavros/local_position/velocity_body", 10)
        self.p_state = self.create_publisher(State, "/mavros/state", 10)
        # ⚠ use_isam2=false kolunda fusion_node pozu BURADAN alır (MAVROS'un
        # kendi EKF'i). Simülatörün ilk turunda bu topic yoktu ve füzyon
        # "henüz tahmin yok" deyip poz üretmedi → planning POZ-YOK.
        self.p_lpose = self.create_publisher(
            PoseStamped, "/mavros/local_position/pose", 10
        )
        # MAVROS gorev listesini TRANSIENT_LOCAL yayinlar; abone de oyle
        # bekliyor. VOLATILE yayinlarsak DDS 'incompatible QoS' deyip mesaji
        # HIC teslim etmez — simulatorun ilk turunda tam olarak bu yasandi.
        qos_gorev = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
        )
        self.p_wps = self.create_publisher(
            WaypointList, "/mavros/mission/waypoints", qos_gorev
        )
        self.p_reached = self.create_publisher(WaypointReached, "/mavros/mission/reached", 10)
        self.p_obs = self.create_publisher(PoseArray, "/perception/obstacle_map", 10)
        self.p_cls = self.create_publisher(Detection3DArray, "/perception/classified_obstacles", 10)

        self.create_subscription(
            Twist, "/mavros/setpoint_velocity/cmd_vel_unstamped", self._on_cmd, 10
        )

        self.create_timer(0.02, self._fizik)        # 50 Hz
        self.create_timer(0.10, self._algi)         # 10 Hz
        self.create_timer(0.20, self._gps)          # 5 Hz
        self.create_timer(0.50, self._durum)        # 2 Hz
        self.create_timer(1.00, self._gorev)        # 1 Hz — "Mission Planner"
        self.create_timer(2.00, self._rapor)
        self.get_logger().info(
            f"SANAL GÖL açıldı — {len(self.kapilar)} kapı "
            f"(açıklık {2*self.kapilar[0][2]:.0f} m, aralık "
            f"{self.kapilar[1][1]-self.kapilar[0][1] if len(self.kapilar)>1 else 0:.0f} m), "
            f"{len(self.gorev_xy)} görev noktası, {len(self.engeller)} engel"
        )

    # ---------------- fizik ----------------
    def _on_cmd(self, msg: Twist) -> None:
        simdi = self.get_clock().now().nanoseconds / 1e9
        if self.son_cmd_t is not None:
            bosluk = simdi - self.son_cmd_t
            if bosluk > 0.5:                       # planning_node'un kendi eşiği
                self.bosluk_sayaci += 1
                self.en_uzun_bosluk = max(self.en_uzun_bosluk, bosluk)
        self.son_cmd_t = simdi
        self.cmd_sayaci += 1
        self.cmd_u = float(msg.linear.x)
        self.cmd_r = float(msg.angular.z)

    def _fizik(self) -> None:
        dt = 0.02
        self.t += dt
        # Birinci mertebe gecikme (tekne ataleti) — ölçülen seyir 1,05 m/s
        self.u += (max(-0.5, min(1.05, self.cmd_u)) - self.u) * dt / 0.8
        self.r += (max(-0.8, min(0.8, self.cmd_r)) - self.r) * dt / 0.5
        self.psi += self.r * dt
        self.x += self.u * math.cos(self.psi) * dt
        self.y += self.u * math.sin(self.psi) * dt

        imu = Imu()
        imu.header.stamp = self.get_clock().now().to_msg()
        imu.header.frame_id = "base_link"
        imu.orientation.z = math.sin(self.psi / 2.0)
        imu.orientation.w = math.cos(self.psi / 2.0)
        imu.angular_velocity.z = self.r
        imu.linear_acceleration.z = 9.81
        self.p_imu.publish(imu)

        tw = TwistStamped()
        tw.header = imu.header
        tw.twist.linear.x = self.u
        tw.twist.angular.z = self.r
        self.p_vel.publish(tw)

        lp = PoseStamped()
        lp.header.stamp = imu.header.stamp
        lp.header.frame_id = "map"
        lp.pose.position.x, lp.pose.position.y = self.x, self.y
        lp.pose.orientation = imu.orientation
        self.p_lpose.publish(lp)

        # Açıklık: gövde YÜZEYİNDEN duba YÜZEYİNE
        for i, (wx, wy, yari) in enumerate(
            [(kx - yr, ky, 0.15) for (kx, ky, yr) in self.kapilar]
            + [(kx + yr, ky, 0.15) for (kx, ky, yr) in self.kapilar]
            + [(ex, ey, 0.25) for (ex, ey) in self.engeller]
        ):
            d = math.hypot(wx - self.x, wy - self.y) - 0.3925 - yari
            self.en_yakin_m = min(self.en_yakin_m, d)
            if d <= 0.0 and i not in self._temas:
                self._temas.add(i)
                self.carpma += 1
                self.get_logger().warn(f"💥 ÇARPMA #{self.carpma}")

    # ---------------- sahte sensör/durum ----------------
    def _gps(self) -> None:
        m = NavSatFix()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = "base_link"
        m.status.status = NavSatStatus.STATUS_GBAS_FIX          # RTK fixed
        m.status.service = NavSatStatus.SERVICE_GPS
        m.latitude = self.lat0 + math.degrees(self.y / R_DUNYA)
        m.longitude = self.lon0 + math.degrees(
            self.x / (R_DUNYA * math.cos(math.radians(self.lat0)))
        )
        m.altitude = 871.0
        self.p_gps.publish(m)

    def _durum(self) -> None:
        s = State()
        s.header.stamp = self.get_clock().now().to_msg()
        s.connected = True
        s.armed = True                    # kaptan MP'den arm etmiş gibi
        # 🔑 GERÇEK OPERATÖR DAVRANIŞI: görev, moda GEÇİŞ ANINDA başlar
        # (F-V.6). Kaptan MP'de önce MANUAL'de arm eder, sonra GUIDED'a
        # çeker. Baştan GUIDED yayınlarsak kenar oluşmaz ve FSM BEKLEMEDE'de
        # takılır — simülatörün ikinci turunda tam olarak bu yaşandı.
        guided = self.t > 10.0
        s.guided = guided
        s.mode = "GUIDED" if guided else "MANUAL"
        s.system_status = 4
        self.p_state.publish(s)

    def _gorev(self) -> None:
        """Mission Planner'ın yüklediği görev — mission_source=fc bunu okur."""
        wl = WaypointList()
        wl.current_seq = 0
        ev = Waypoint()                    # seq 0 = home (skip_home_seq0=True)
        ev.frame, ev.command, ev.is_current = 0, 16, False
        ev.x_lat, ev.y_long, ev.z_alt = self.lat0, self.lon0, 0.0
        wl.waypoints.append(ev)
        for (gx, gy) in self.gorev_xy:
            w = Waypoint()
            w.frame, w.command, w.autocontinue = 3, 16, True
            w.x_lat = self.lat0 + math.degrees(gy / R_DUNYA)
            w.y_long = self.lon0 + math.degrees(
                gx / (R_DUNYA * math.cos(math.radians(self.lat0)))
            )
            w.z_alt = 0.0
            wl.waypoints.append(w)
        self.p_wps.publish(wl)

        # F-V.8: uçuş kontrolcüsünün varış senkronu
        for i, (gx, gy) in enumerate(self.gorev_xy):
            if i > self.varilan and math.hypot(gx - self.x, gy - self.y) < 2.0:
                self.varilan = i
                wr = WaypointReached()
                wr.wp_seq = i + 1          # home seq 0 olduğu için +1
                self.p_reached.publish(wr)
                self.get_logger().info(f"✅ GÖREV NOKTASI {i+1} — VARILDI")

    # ---------------- sahte algı ----------------
    def _dunya_to_govde(self, wx: float, wy: float):
        dx, dy = wx - self.x, wy - self.y
        c, s = math.cos(-self.psi), math.sin(-self.psi)
        return c * dx - s * dy, s * dx + c * dy

    def _algi(self) -> None:
        """Kapı direkleri (turuncu) + engeller (sarı) — LiDAR menzili 25 m."""
        pa = PoseArray()
        pa.header.stamp = self.get_clock().now().to_msg()
        pa.header.frame_id = "base_link"
        da = Detection3DArray()
        da.header = pa.header

        cisimler = []
        for (kx, ky, yari) in self.kapilar:
            cisimler.append((kx - yari, ky, 0.15, KENAR_SINIF))
            cisimler.append((kx + yari, ky, 0.15, KENAR_SINIF))
        for (ex, ey) in self.engeller:
            cisimler.append((ex, ey, 0.25, BILINMEYEN))

        for (wx, wy, yaricap, sinif) in cisimler:
            bx, by = self._dunya_to_govde(wx, wy)
            if math.hypot(bx, by) > 25.0 or bx < -2.0:      # LiDAR menzili
                continue
            p = Pose()
            p.position.x, p.position.y = bx, by
            p.orientation.z, p.orientation.w = yaricap, 1.0   # yarıçap hack'i
            pa.poses.append(p)

            d = Detection3D()
            d.bbox.center.position.x, d.bbox.center.position.y = bx, by
            d.bbox.size.x = 2.0 * yaricap
            h = ObjectHypothesisWithPose()
            # Kamera 69° kadraj + 15 m menzil: dışında kalan turuncu bile
            # UNKNOWN gelir (gerçek davranış — §0.17e'nin çözdüğü hâl).
            aci = abs(math.atan2(by, bx))
            gorunur = aci < math.radians(34.5) and math.hypot(bx, by) < 15.0
            h.hypothesis.class_id = sinif if gorunur else BILINMEYEN
            h.hypothesis.score = 0.9
            d.results.append(h)
            da.detections.append(d)

        self.p_obs.publish(pa)
        self.p_cls.publish(da)

    def _rapor(self) -> None:
        hedef = self.gorev_xy[min(self.varilan + 1, len(self.gorev_xy) - 1)]
        self.get_logger().info(
            f"[{self.t:6.1f} s] konum=({self.x:6.2f}, {self.y:6.2f}) "
            f"ψ={math.degrees(self.psi) % 360:5.1f}° u={self.u:4.2f} m/s | "
            f"hedef {self.varilan+1}/{len(self.gorev_xy)} "
            f"({math.hypot(hedef[0]-self.x, hedef[1]-self.y):5.1f} m) | "
            f"cmd_vel {self.cmd_sayaci} mesaj, {self.bosluk_sayaci} boşluk "
            f"(en uzun {self.en_uzun_bosluk:.2f} s) | açıklık min "
            f"{self.en_yakin_m:.2f} m, çarpma {self.carpma}"
        )
        if self.varilan == len(self.gorev_xy) - 1:
            self.get_logger().info("🏁 PARKUR TAMAMLANDI")


def main() -> None:
    rclpy.init(args=sys.argv[1:])
    n = SanalGol()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    finally:
        n.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
