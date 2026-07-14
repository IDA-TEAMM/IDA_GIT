"""
Girdap İDA — Video görev yöneticisi node'u (Layer 2).

Otonomi Kabiliyeti videosu: DİKDÖRTGEN oluşturan 4 GPS waypoint; son noktada
görev tamamlanır, başlangıca dönüş MANUEL (md 3.3.1(3) — F-V.4).
Görev durum makinesi + haversine/ENU dönüşümü ROS-bağımsız
prototype.mission.mission_manager.MissionManager'da; bu node onu sarar:
video_mission.yaml okur, GPS besler, hedefi yayınlar.

Görev kaynağı (`mission_source` param) iki modludur:
    "file" — araç üstü `mission_file` YAML'ı (offline/geliştirme; varsayılan).
    "fc"   — YKİ→Pixhawk→MAVROS `/mavros/mission/waypoints` (mavros_msgs/
             WaypointList). Şartname md 3.3.1(2) + md 5.5.2.2: görev YKİ'de
             tanımlanıp İDA'ya YÜKLENİR — video/yarışma günü ZORUNLU mod.
             lat/lon → Waypoint dönüşümü ROS-bağımsız çekirdekte
             (prototype.mission.mission_manager.fc_items_to_waypoints).

Boot: `mission_file` param'ından görev dosyası (video veya competition) okunur.
Subscribed:
    /mavros/global_position/global   sensor_msgs/NavSatFix   (SensorDataQoS)
    /girdap/mission/state            std_msgs/String         (FSM start tetiği)
    /mavros/mission/waypoints        mavros_msgs/WaypointList (yalnız fc modu,
                                     latched QoS; görev yalnız başlamadan yüklenir)
Published:
    /girdap/mission/current_target   geometry_msgs/PoseStamped (base_link, 5 Hz)
        position.{x,y} = güncel hedefe ENU ofseti (east, north).
    /girdap/mission/current_parkur   std_msgs/Int32   (Sprint 4)
        Güncel hedef waypoint'in parkur numarası (5 Hz). Video → hep 1.
    /girdap/mission/waypoint_reached std_msgs/Int32   (Sprint 4)
        Bir waypoint'e VARILINCA tek atış index yayını (ACTIVE→DWELL geçişi).
        fsm_node parkur katmanı bu sinyalle ilerler (waypoint-index tabanlı).

Not: Görev FSM'den başlar — /girdap/mission/state aktif parkura (PARKUR1)
geçince MissionManager.start() çağrılır (ayrı servis yerine durum takibi;
planning ile aynı FSM otoritesi).
"""

from __future__ import annotations

from typing import List, Optional

import yaml

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Bool, Int32, String

from girdap_decision.qos_profiles import latched_qos, sensor_data_qos
from prototype.mission.mission_manager import (
    FcMissionItem,
    MissionManager,
    MissionManagerConfig,
    MissionPhase,
    Waypoint,
    farthest_waypoint_m,
    fc_items_to_waypoints,
)

# FSM'de aracın hareket ettiği (görev aktif) durumlar.
_ACTIVE_STATES = ("PARKUR1", "PARKUR2", "PARKUR3")


class MissionManagerNode(Node):
    """Video waypoint görev yöneticisi — MissionManager çekirdeğini sarar."""

    def __init__(self, **node_kwargs) -> None:
        # node_kwargs → parameter_overrides passthrough (test enjeksiyonu).
        super().__init__("mission_manager_node", **node_kwargs)

        self.declare_parameter("mission_source", "file")   # "file" | "fc"
        self.declare_parameter("mission_file", "")
        self.declare_parameter("publish_rate_hz", 5.0)
        self.declare_parameter("skip_home_seq0", True)      # fc: ArduPilot home item'ı
        # fc modu görev config'i (file modunda YAML kazanır).
        _dflt = MissionManagerConfig()
        self.declare_parameter("arrival_radius_m", _dflt.arrival_radius_m)
        self.declare_parameter("dwell_time_s", _dflt.dwell_time_s)
        self.declare_parameter("cruise_velocity_mps", _dflt.cruise_velocity_mps)

        # F-M.1: hedef-mesafe makullük tavanı (m) — bir waypoint mevcut
        # konumdan bundan uzaksa görev başlatılmaz (masa OOM olayı).
        self.declare_parameter("max_target_distance_m", 10_000.0)

        self._source = str(self.get_parameter("mission_source").value).lower()
        self._skip_home = bool(self.get_parameter("skip_home_seq0").value)
        self._max_target_m = float(
            self.get_parameter("max_target_distance_m").value
        )

        if self._source == "fc":
            # Görev FC'den (WaypointList) gelecek — boş başla, callback kurar.
            cfg = self._cfg_from_params()
            self._mgr = MissionManager([], cfg)
        else:
            waypoints, cfg = self._load_mission(
                str(self.get_parameter("mission_file").value)
            )
            self._mgr = MissionManager(waypoints, cfg)
        self._cfg = cfg

        self._lat: Optional[float] = None
        self._lon: Optional[float] = None
        self._started = False
        self._prev_phase = self._mgr.phase        # waypoint-varış tespiti için

        # --- Subscribers ---
        self._sub_gps = self.create_subscription(
            NavSatFix, "/mavros/global_position/global",
            self._on_gps, sensor_data_qos(),
        )
        self._sub_state = self.create_subscription(
            String, "/girdap/mission/state", self._on_state, 10
        )

        # --- Publishers ---
        self._pub_target = self.create_publisher(
            PoseStamped, "/girdap/mission/current_target", 10
        )
        # Sprint 4 parkur katmanı: güncel parkur (periyodik) + varış (tek atış).
        self._pub_parkur = self.create_publisher(
            Int32, "/girdap/mission/current_parkur", 10
        )
        self._pub_reached = self.create_publisher(
            Int32, "/girdap/mission/waypoint_reached", 10
        )
        # Görev tamamlandı (tüm waypoint'ler) — fsm_node bunu TAMAMLANDI terminal
        # geçişi için kullanır (video senaryosu; kamikaze çarpması olmadan da
        # temiz duruş). Latching: bir kez True olunca öyle kalır.
        self._pub_complete = self.create_publisher(
            Bool, "/girdap/mission/complete", 10
        )

        # --- fc kaynağı: /mavros/mission/waypoints aboneliği ---
        if self._source == "fc":
            self._setup_fc_source()

        # --- Yayım döngüsü (5 Hz — MPPI referansı taze kalsın) ---
        rate = float(self.get_parameter("publish_rate_hz").value)
        self._timer = self.create_timer(1.0 / rate, self._on_tick)

        if self._source == "fc":
            self.get_logger().info(
                "mission_manager_node aktif (kaynak=fc): görev FC'den beklenecek, "
                f"arrival={cfg.arrival_radius_m} m, dwell={cfg.dwell_time_s} s, "
                f"yayım={rate} Hz"
            )
        else:
            self.get_logger().info(
                f"mission_manager_node aktif (kaynak=file): "
                f"{self._mgr.waypoint_count} waypoint, "
                f"arrival={cfg.arrival_radius_m} m, dwell={cfg.dwell_time_s} s, "
                f"yayım={rate} Hz"
            )

    # ----- görev config / kaynağı -----

    def _cfg_from_params(self) -> MissionManagerConfig:
        """fc modu görev config'ini node parametrelerinden üretir."""
        return MissionManagerConfig(
            arrival_radius_m=float(self.get_parameter("arrival_radius_m").value),
            dwell_time_s=float(self.get_parameter("dwell_time_s").value),
            cruise_velocity_mps=float(
                self.get_parameter("cruise_velocity_mps").value
            ),
        )

    def _setup_fc_source(self) -> None:
        """FC (MAVROS) görev kaynağını kur — /mavros/mission/waypoints aboneliği.

        mavros_msgs lazy import: yalnız fc modunda gerekir (file modu ve pytest
        çekirdeği mavros_msgs olmadan çalışır). Kurulu değilse hata loglanır,
        node çökmemek için boş görevde kalır (operatör uyarılır).
        """
        try:
            from mavros_msgs.msg import WaypointList
        except ImportError as exc:
            self.get_logger().error(
                f"mission_source=fc ama mavros_msgs yok ({exc}); görev boş kalır. "
                "'sudo apt install ros-humble-mavros-msgs' kurun."
            )
            return
        self._sub_wps = self.create_subscription(
            WaypointList, "/mavros/mission/waypoints",
            self._on_fc_waypoints, latched_qos(),
        )
        self.get_logger().info(
            "mission_source=fc — /mavros/mission/waypoints bekleniyor "
            f"(skip_home_seq0={self._skip_home})"
        )

    def _on_fc_waypoints(self, msg) -> None:                      # noqa: ANN001
        """FC görev listesi geldi → görevi (yeniden) kur. Yalnız başlamadan.

        Şartname md 5.5.2.2: görev yükleme güç sonrası, görev başlamadan yapılır.
        Görev başladıysa (araç hareket halinde) sonradan gelen liste yok sayılır.
        """
        if self._started:
            self.get_logger().warn(
                "FC görev güncellemesi görev başladıktan sonra geldi — yok sayıldı"
            )
            return
        items = [
            FcMissionItem(
                seq=i, command=int(w.command),
                lat=float(w.x_lat), lon=float(w.y_long),
            )
            for i, w in enumerate(msg.waypoints)
        ]
        wps = fc_items_to_waypoints(items, skip_home_seq0=self._skip_home)
        if not wps:
            self.get_logger().warn(
                f"FC görevi {len(msg.waypoints)} item ama gezinme waypoint'i yok "
                "— görev güncellenmedi (henüz yüklenmemiş olabilir)"
            )
            return
        self._mgr = MissionManager(wps, self._cfg)
        self._prev_phase = self._mgr.phase
        self.get_logger().info(
            f"FC görevi alındı: {len(msg.waypoints)} item → {len(wps)} waypoint "
            f"(arrival={self._cfg.arrival_radius_m} m, dwell={self._cfg.dwell_time_s} s)"
        )

    # ----- görev dosyası -----

    def _load_mission(self, path: str):
        """video_mission.yaml → (waypoints, config). Okunamazsa boş görev."""
        wps: List[Waypoint] = []
        cfg = MissionManagerConfig()
        if not path:
            self.get_logger().error("mission_file parametresi boş — görev yok")
            return wps, cfg
        try:
            with open(path, "r", encoding="utf-8") as fh:
                data = yaml.safe_load(fh) or {}
            for w in data.get("waypoints", []):
                wps.append(
                    Waypoint(
                        lat=float(w["lat"]),
                        lon=float(w["lon"]),
                        name=str(w.get("name", "")),
                        parkur=int(w.get("parkur", 1)),   # video → 1 (etiketsiz)
                    )
                )
            cfg = MissionManagerConfig(
                arrival_radius_m=float(
                    data.get("arrival_radius_m", cfg.arrival_radius_m)
                ),
                dwell_time_s=float(data.get("dwell_time_s", cfg.dwell_time_s)),
                cruise_velocity_mps=float(
                    data.get("cruise_velocity_mps", cfg.cruise_velocity_mps)
                ),
            )
        except Exception as exc:
            self.get_logger().error(f"görev dosyası okunamadı ({path}): {exc}")
        return wps, cfg

    # ----- callback'ler -----

    def _on_gps(self, msg: NavSatFix) -> None:
        if msg.status.status < 0:
            return
        # F-M.1: (0,0) = ArduPilot'un fix'siz "null island" çıktısı — status
        # FIX görünse bile konum GEÇERSİZ say (masa OOM olayının girdisi:
        # (0,0)'dan 40°K/29°D hedefe ~4400 km referans üretilmişti).
        if msg.latitude == 0.0 and msg.longitude == 0.0:
            return
        self._lat = msg.latitude
        self._lon = msg.longitude

    def _on_state(self, msg: String) -> None:
        # FSM aktif parkura geçince görevi başlat (tek seferlik).
        if not self._started and msg.data in _ACTIVE_STATES:
            if self._mgr.waypoint_count == 0:
                # fc modu: görev henüz FC'den gelmedi — başlatma, WaypointList
                # bekle (yoksa _started latch'lenip görevi kilitlerdi).
                self.get_logger().warn(
                    "FSM aktif ama görev yüklü değil — FC WaypointList bekleniyor"
                )
                return
            # F-M.1 guard 1: geçerli fix olmadan görev başlamaz (F8.4'ün kod
            # karşılığı — "arm'dan önce fix bekle" artık yalnız operasyon
            # notu değil). _started latch'lenmez → fix gelince başlar.
            if self._lat is None or self._lon is None:
                self.get_logger().warn(
                    "FSM aktif ama geçerli GPS fix yok — görev başlatılmıyor "
                    "(F-M.1)",
                    throttle_duration_sec=5.0,
                )
                return
            # F-M.1 guard 2: hedef-mesafe makullüğü — yanlış/sahte koordinat
            # devasa MPPI referansı üretmeden BURADA reddedilir.
            far = farthest_waypoint_m(self._lat, self._lon, self._mgr.waypoints)
            if far > self._max_target_m:
                self.get_logger().error(
                    f"görev REDDEDİLDİ: en uzak hedef {far / 1000.0:.1f} km > "
                    f"tavan {self._max_target_m / 1000.0:.1f} km — görev "
                    "koordinatlarını ve GPS konumunu kontrol et (F-M.1)",
                    throttle_duration_sec=5.0,
                )
                return
            self._mgr.start()
            self._started = True
            self.get_logger().info("görev başlatıldı (FSM aktif parkur)")

    # ----- yayım -----

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _on_tick(self) -> None:
        if self._lat is None or self._lon is None:
            return
        offset = self._mgr.update(self._lat, self._lon, self._now())

        # Görev tamamlandı bayrağını her tick yayınla (latching; fsm_node
        # TAMAMLANDI terminal geçişi için okur — F12.2).
        self._pub_complete.publish(Bool(data=self._mgr.is_complete))

        # Waypoint varış tespiti: DWELL'e GİRİŞ (ACTIVE→DWELL) → tek atış index.
        # "girişte bir kez" mantığı: DWELL sürerken tekrar yayınlamaz.
        phase = self._mgr.phase
        if phase is MissionPhase.DWELL and self._prev_phase is not MissionPhase.DWELL:
            idx = int(self._mgr.current_index)
            self._pub_reached.publish(Int32(data=idx))
            wp = self._mgr.current_waypoint
            self.get_logger().info(
                f"waypoint {idx} varıldı"
                + (f" (parkur {wp.parkur})" if wp is not None else "")
            )
        self._prev_phase = phase

        if offset is None:                       # IDLE / COMPLETE
            return
        east, north = offset

        # Güncel hedefin parkur numarası (periyodik) — video → hep 1.
        wp = self._mgr.current_waypoint
        if wp is not None:
            self._pub_parkur.publish(Int32(data=int(wp.parkur)))

        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"        # araç göreli ENU ofseti
        msg.pose.position.x = float(east)
        msg.pose.position.y = float(north)
        msg.pose.orientation.w = 1.0
        self._pub_target.publish(msg)


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node = MissionManagerNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
