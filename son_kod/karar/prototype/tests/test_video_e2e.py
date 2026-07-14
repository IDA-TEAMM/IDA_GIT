"""
Girdap İDA — Otonomi videosu UÇTAN UCA zincir testi (Faz 13, video denetimi).

Şartname md 3.3.1(2)+(3) senaryosunun tamamını GERÇEK node grafiğiyle koşar:

    sahte QGC (WaypointList, latched)          [md 3.3.1(2): YKİ'den 4 nokta]
        → mission_manager_node (mission_source=fc, skip_home)
    sahte FCU (/mavros/state: ARM → GUIDED kenarı)  [md 3.3.1(3): tek komut]
        → fsm_node (BOOT→ARM→BEKLEMEDE→PARKUR1)
    sahte GPS 4 köşeyi gezer
        → waypoint_reached ×4 → /girdap/mission/complete → TAMAMLANDI
    planning_node (use_rrt=false bypass, MPPI)
        → TAMAMLANDI'da /girdap/control/thrust == [0, 0]   [md 3.3.1.1 istemsiz
          hareket yok: görev sonunda motor komutu kesilir]
    telemetry_node
        → grafik CSV'de yon_setpoint AÇI (F-V.1) + Dosya-2 mission_state.

rclpy + mavros_msgs gerektirir (deps ws source'lu ROS ortamı); yoksa SKIP.
"""

from __future__ import annotations

import csv
import math
import time

import pytest

rclpy = pytest.importorskip("rclpy", reason="rclpy yok (.venv) — ROS ortamında koş")
mavros_msgs = pytest.importorskip(
    "mavros_msgs", reason="mavros_msgs yok — deps ws source'la"
)

from rclpy.executors import SingleThreadedExecutor          # noqa: E402
from rclpy.parameter import Parameter                       # noqa: E402

from mavros_msgs.msg import State as MavState, Waypoint, WaypointList  # noqa: E402
from nav_msgs.msg import Odometry                           # noqa: E402
from sensor_msgs.msg import NavSatFix                       # noqa: E402
from std_msgs.msg import Bool, Float32MultiArray, Int32, String  # noqa: E402

# F16.2 kapılaması: kardeş node testleriyle aynı desen — girdap_decision
# source'lanmamışsa (ROS'suz CI dahil) koleksiyonu KIRMADAN gerekçeli SKIP.
pytest.importorskip(
    "girdap_decision.fsm_node",
    reason="girdap_decision source'lanmamış (ros2_ws/install/setup.bash)",
)

from girdap_decision.fsm_node import FSMNode                # noqa: E402
from girdap_decision.mission_manager_node import MissionManagerNode  # noqa: E402
from girdap_decision.planning_node import PlanningNode      # noqa: E402
from girdap_decision.telemetry_node import TelemetryNode    # noqa: E402
from girdap_decision.qos_profiles import latched_qos, sensor_data_qos  # noqa: E402

# Video senaryosu: ~22 m × ~25 m dikdörtgen (gerçekçi göl koordinatı).
_LAT0, _LON0 = 40.8000000, 29.3000000
_DLAT, _DLON = 0.0002, 0.0003
_CORNERS = [
    (_LAT0 + _DLAT, _LON0),
    (_LAT0 + _DLAT, _LON0 + _DLON),
    (_LAT0, _LON0 + _DLON),
    (_LAT0, _LON0),
]


class _Sahte:
    """Sahte QGC + FCU + GPS: yayınlar ve gözlemler tek yardımcı node'da."""

    def __init__(self) -> None:
        self.node = rclpy.create_node("video_e2e_helper")
        self.pub_wps = self.node.create_publisher(
            WaypointList, "/mavros/mission/waypoints", latched_qos()
        )
        self.pub_state = self.node.create_publisher(MavState, "/mavros/state", 10)
        self.pub_gps = self.node.create_publisher(
            NavSatFix, "/mavros/global_position/global", sensor_data_qos()
        )
        self.pub_odom = self.node.create_publisher(Odometry, "/girdap/fusion/odom", 10)

        self.fsm_state = ""
        self.reached: list[int] = []
        self.complete = False
        self.last_thrust: list[float] | None = None
        self.node.create_subscription(
            String, "/girdap/mission/state",
            lambda m: setattr(self, "fsm_state", m.data), 10,
        )
        self.node.create_subscription(
            Int32, "/girdap/mission/waypoint_reached",
            lambda m: self.reached.append(int(m.data)), 10,
        )
        self.node.create_subscription(
            Bool, "/girdap/mission/complete",
            lambda m: setattr(self, "complete", self.complete or m.data), 10,
        )
        self.node.create_subscription(
            Float32MultiArray, "/girdap/control/thrust",
            lambda m: setattr(self, "last_thrust", list(m.data)), 10,
        )

    # --- yayın yardımcıları ---

    def mav_state(self, armed: bool, mode: str) -> None:
        msg = MavState()
        msg.connected = True
        msg.armed = armed
        msg.guided = mode == "GUIDED"
        msg.mode = mode
        self.pub_state.publish(msg)

    def gps(self, lat: float, lon: float) -> None:
        msg = NavSatFix()
        msg.status.status = 0                    # FIX var
        msg.latitude = lat
        msg.longitude = lon
        self.pub_gps.publish(msg)

    def odom_origin(self) -> None:
        od = Odometry()
        od.pose.pose.orientation.w = 1.0
        self.pub_odom.publish(od)

    def qgc_mission(self) -> None:
        """QGC görev yüklemesi: home (seq 0) + 4 köşe NAV_WAYPOINT."""
        wl = WaypointList()
        home = Waypoint()
        home.command = 16
        home.x_lat, home.y_long = _LAT0, _LON0
        wl.waypoints.append(home)
        for la, lo in _CORNERS:
            w = Waypoint()
            w.command = 16
            w.x_lat, w.y_long = la, lo
            wl.waypoints.append(w)
        self.pub_wps.publish(wl)


def _spin(ex: SingleThreadedExecutor, seconds: float) -> None:
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        ex.spin_once(timeout_sec=0.02)


def _spin_until(ex, cond, seconds: float, msg: str) -> None:
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        if cond():
            return
        ex.spin_once(timeout_sec=0.02)
    pytest.fail(f"zaman aşımı: {msg}")


@pytest.fixture()
def ros():                                       # noqa: ANN201
    rclpy.init()
    yield
    rclpy.shutdown()


def test_video_zinciri_qgc_gorevden_temiz_durusa(ros, tmp_path) -> None:
    """4 nokta QGC görevi → GUIDED tetiği → 4 varış → TAMAMLANDI → thrust 0."""
    mission = MissionManagerNode(
        parameter_overrides=[
            Parameter("mission_source", Parameter.Type.STRING, "fc"),
            Parameter("dwell_time_s", Parameter.Type.DOUBLE, 0.2),
            Parameter("publish_rate_hz", Parameter.Type.DOUBLE, 20.0),
        ]
    )
    fsm = FSMNode(
        parameter_overrides=[
            Parameter("tick_rate_hz", Parameter.Type.DOUBLE, 20.0),
        ]
    )
    planning = PlanningNode(
        parameter_overrides=[
            Parameter("use_rrt", Parameter.Type.BOOL, False),
            Parameter("control_rate_hz", Parameter.Type.DOUBLE, 5.0),
            Parameter("mppi_K", Parameter.Type.INTEGER, 50),
            Parameter("mppi_T", Parameter.Type.INTEGER, 10),
        ]
    )
    telemetry = TelemetryNode(
        parameter_overrides=[
            Parameter("csv_output_dir", Parameter.Type.STRING,
                      str(tmp_path / "telemetry")),
            Parameter("graph_output_dir", Parameter.Type.STRING,
                      str(tmp_path / "grafik")),
            Parameter("graph_rate_hz", Parameter.Type.DOUBLE, 20.0),
            Parameter("log_rate_hz", Parameter.Type.DOUBLE, 10.0),
        ]
    )
    sahte = _Sahte()
    ex = SingleThreadedExecutor()
    for n in (mission.node if hasattr(mission, "node") else mission,
              fsm, planning, telemetry, sahte.node):
        ex.add_node(n)
    try:
        # 1) QGC görev yüklemesi (md 3.3.1/2) — latched, boot sırası önemsiz.
        sahte.qgc_mission()
        sahte.odom_origin()
        sahte.gps(_LAT0, _LON0)

        # 2) FCU: bağlı + disarm + MANUAL → FSM BOOT→ARM.
        for _ in range(5):
            sahte.mav_state(armed=False, mode="MANUAL")
            ex.spin_once(timeout_sec=0.02)
        # ARM (operatör QGC'den) → FSM ARM→BEKLEMEDE.
        sahte.mav_state(armed=True, mode="MANUAL")
        _spin_until(ex, lambda: sahte.fsm_state == "BEKLEMEDE", 5.0,
                    "FSM BEKLEMEDE'ye gelmedi")

        # 3) md 3.3.1(3): TEK komut = mod → GUIDED (kenar tetikli başlatma).
        sahte.mav_state(armed=True, mode="GUIDED")
        _spin_until(ex, lambda: sahte.fsm_state == "PARKUR1", 5.0,
                    "GUIDED kenarı görevi başlatmadı")

        # 4) GPS 4 köşeyi gezer; her köşede varış + dwell.
        for i, (la, lo) in enumerate(_CORNERS):
            def _at_corner(idx=i):
                return len(sahte.reached) >= idx + 1
            end = time.monotonic() + 10.0
            while time.monotonic() < end and not _at_corner():
                sahte.gps(la, lo)
                sahte.mav_state(armed=True, mode="GUIDED")
                sahte.odom_origin()
                ex.spin_once(timeout_sec=0.02)
            assert _at_corner(), f"köşe {i} varışı gelmedi (reached={sahte.reached})"

        assert sahte.reached[:4] == [0, 1, 2, 3]

        # 5) Son noktada görev TAMAM (md 3.3.1/3) → FSM TAMAMLANDI.
        def _pump():
            sahte.gps(*_CORNERS[-1][::1])
            sahte.mav_state(armed=True, mode="GUIDED")
            sahte.odom_origin()
            return False
        _spin_until(ex, lambda: sahte.complete or _pump(), 10.0,
                    "mission complete gelmedi")
        _spin_until(ex, lambda: sahte.fsm_state == "TAMAMLANDI" or _pump(), 10.0,
                    "FSM TAMAMLANDI'ya geçmedi")

        # 6) İstemsiz hareket yok: TAMAMLANDI sonrası thrust [0, 0].
        sahte.last_thrust = None
        _spin_until(ex, lambda: sahte.last_thrust is not None or _pump(), 10.0,
                    "TAMAMLANDI sonrası thrust yayını gelmedi")
        assert sahte.last_thrust == [0.0, 0.0]

        # 7) Telemetri: grafik CSV'de yon_setpoint AÇI üretti (F-V.1) ve
        #    Dosya-2 mission_state sütunu TAMAMLANDI'yı gördü.
        _spin(ex, 0.3)                            # son satırlar diske insin
        with open(telemetry._graph_csv.path, newline="", encoding="utf-8") as f:
            rows = list(csv.reader(f))
        yon = [r[4] for r in rows[1:] if r[4] != ""]
        assert yon, "grafik CSV'de hiç yon_setpoint yok"
        # Köşeler arası rota açıları: hepsi geçerli [-π, π] açı değeri
        # (CSV 3 ondalık yuvarlar: π → "3.142" — tolerans 5e-3).
        assert all(abs(float(v)) <= math.pi + 5e-3 for v in yon)
        with open(telemetry._csv.path, newline="", encoding="utf-8") as f:
            states = [r[9] for r in csv.reader(f)][1:]
        assert "TAMAMLANDI" in states
    finally:
        for n in (mission, fsm, planning, telemetry, sahte.node):
            ex.remove_node(n)
            n.destroy_node()
