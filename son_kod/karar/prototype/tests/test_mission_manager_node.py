"""
Girdap İDA — mission_manager_node parkur sinyalleri testi (Sprint 4).

FAZ 5 entegrasyonu: waypoint'e VARILINCA /girdap/mission/waypoint_reached
(tek atış index) + /girdap/mission/current_parkur (periyodik) yayınlanır.
Bu, fsm_node parkur katmanını besleyen kritik halkadır.

rclpy gerektirir → sistem python3.10 + ROS Humble; .venv'de SKIP.
"""

from __future__ import annotations

import textwrap
import time

import pytest

rclpy = pytest.importorskip("rclpy", reason="rclpy yok (.venv) — ROS ortamında koş")

from rclpy.node import Node                              # noqa: E402
from rclpy.parameter import Parameter                    # noqa: E402
from sensor_msgs.msg import NavSatFix                    # noqa: E402
from std_msgs.msg import Int32, String                   # noqa: E402

girdap = pytest.importorskip(
    "girdap_decision.mission_manager_node",
    reason="girdap_decision source'lanmamış (ros2_ws/install/setup.bash)",
)


@pytest.fixture(scope="module")
def ros_context():                                       # noqa: ANN201
    rclpy.init()
    yield
    rclpy.shutdown()


@pytest.fixture
def mission_file(tmp_path_factory) -> str:               # noqa: ANN001
    """2 waypoint (41°K/29°D), parkur [1,2], uzun dwell → wp0 DWELL'de kalır.

    F-M.1 sonrası (0,0) fix/waypoint GEÇERSİZ sayılır (null island) —
    testler gerçekçi koordinat kullanır; fix aynı noktada → anında varış.
    """
    path = tmp_path_factory.mktemp("mm") / "m.yaml"
    path.write_text(
        textwrap.dedent(
            """
            waypoints:
              - {lat: 41.0, lon: 29.0, parkur: 1}
              - {lat: 41.0, lon: 29.0, parkur: 2}
            arrival_radius_m: 5.0
            dwell_time_s: 30.0
            """
        ),
        encoding="utf-8",
    )
    return str(path)


@pytest.fixture
def node(ros_context, mission_file):                     # noqa: ANN001
    n = girdap.MissionManagerNode(
        parameter_overrides=[
            Parameter("mission_file", Parameter.Type.STRING, mission_file),
        ]
    )
    yield n
    n.destroy_node()


def test_arrival_publishes_waypoint_reached_and_parkur(node) -> None:  # noqa: ANN001
    helper = rclpy.create_node("test_mm_helper")
    reached: list[int] = []
    parkur: list[int] = []
    helper.create_subscription(
        Int32, "/girdap/mission/waypoint_reached", lambda m: reached.append(m.data), 10
    )
    helper.create_subscription(
        Int32, "/girdap/mission/current_parkur", lambda m: parkur.append(m.data), 10
    )
    gps_pub = helper.create_publisher(
        NavSatFix, "/mavros/global_position/global", 10
    )
    state_pub = helper.create_publisher(String, "/girdap/mission/state", 10)
    try:
        deadline = time.monotonic() + 5.0
        # GPS fix (wp ile aynı nokta) + FSM aktif sinyali → görev başlar, wp0'a "varır".
        fix = NavSatFix()
        fix.status.status = 0                    # STATUS_FIX
        fix.latitude = 41.0
        fix.longitude = 29.0
        while not reached:
            gps_pub.publish(fix)
            state_pub.publish(String(data="PARKUR1"))
            rclpy.spin_once(node, timeout_sec=0.05)
            rclpy.spin_once(helper, timeout_sec=0.05)
            assert time.monotonic() < deadline, "waypoint_reached yayınlanmadı"
        assert reached[0] == 0                    # ilk waypoint index'i
        # current_parkur wp0'ın parkuru (1) olmalı
        deadline = time.monotonic() + 3.0
        while not parkur:
            gps_pub.publish(fix)
            rclpy.spin_once(node, timeout_sec=0.05)
            rclpy.spin_once(helper, timeout_sec=0.05)
            assert time.monotonic() < deadline, "current_parkur yayınlanmadı"
        assert parkur[-1] == 1
    finally:
        helper.destroy_node()


def test_waypoint_reached_fires_once_per_arrival(node) -> None:  # noqa: ANN001
    """DWELL sürerken waypoint_reached TEKRAR yayınlanmamalı (tek atış)."""
    helper = rclpy.create_node("test_mm_once")
    reached: list[int] = []
    helper.create_subscription(
        Int32, "/girdap/mission/waypoint_reached", lambda m: reached.append(m.data), 10
    )
    gps_pub = helper.create_publisher(
        NavSatFix, "/mavros/global_position/global", 10
    )
    state_pub = helper.create_publisher(String, "/girdap/mission/state", 10)
    try:
        fix = NavSatFix()
        fix.status.status = 0
        fix.latitude = 41.0
        fix.longitude = 29.0
        deadline = time.monotonic() + 5.0
        while not reached:
            gps_pub.publish(fix)
            state_pub.publish(String(data="PARKUR1"))
            rclpy.spin_once(node, timeout_sec=0.05)
            rclpy.spin_once(helper, timeout_sec=0.05)
            assert time.monotonic() < deadline
        # DWELL'de bir süre daha spin — tekrar yayın olmamalı (dwell=30 s uzun)
        for _ in range(20):
            gps_pub.publish(fix)
            rclpy.spin_once(node, timeout_sec=0.02)
            rclpy.spin_once(helper, timeout_sec=0.02)
        assert reached == [0]                     # yalnız bir kez, idx 0
    finally:
        helper.destroy_node()


def test_fc_source_rebuilds_mission_from_waypoints(ros_context) -> None:  # noqa: ANN001
    """fc modu: /mavros/mission/waypoints callback görevi yeniden kurar.

    mavros_msgs kurulu olmasa da callback duck-typed mesajla test edilir
    (abonelik kurulmaz; dönüşüm + rebuild mantığı yine de doğrulanır).
    """
    from types import SimpleNamespace

    n = girdap.MissionManagerNode(
        parameter_overrides=[
            Parameter("mission_source", Parameter.Type.STRING, "fc"),
            Parameter("arrival_radius_m", Parameter.Type.DOUBLE, 3.0),
        ]
    )
    try:
        assert n._mgr.waypoint_count == 0             # fc: başlangıçta boş

        def _wp(cmd, lat, lon):                        # noqa: ANN001, ANN202
            return SimpleNamespace(command=cmd, x_lat=lat, y_long=lon)

        msg = SimpleNamespace(waypoints=[
            _wp(16, 40.0, 29.0),        # index 0 = home → atlanır
            _wp(16, 40.001, 29.0),      # NAV_WAYPOINT → tut
            _wp(16, 40.001, 29.001),    # NAV_WAYPOINT → tut
            _wp(177, 40.002, 29.002),   # DO_JUMP → atlanır (gezinme değil)
        ])
        n._on_fc_waypoints(msg)
        assert n._mgr.waypoint_count == 2             # home + DO_JUMP atlandı
        assert abs(n._cfg.arrival_radius_m - 3.0) < 1e-9   # cfg param'dan

        # Görev başladıktan sonra gelen liste yok sayılmalı (md 5.5.2.2).
        n._started = True
        n._on_fc_waypoints(SimpleNamespace(waypoints=[_wp(16, 41.0, 30.0)]))
        assert n._mgr.waypoint_count == 2             # değişmedi
    finally:
        n.destroy_node()


def test_fc_source_starts_only_after_mission_loaded(ros_context) -> None:  # noqa: ANN001
    """fc modu: görev yüklenmeden FSM aktif olsa da başlamaz (latch kilidi yok)."""
    from std_msgs.msg import String as _String

    n = girdap.MissionManagerNode(
        parameter_overrides=[
            Parameter("mission_source", Parameter.Type.STRING, "fc"),
        ]
    )
    try:
        # Görev yokken FSM PARKUR1 → başlatma reddedilmeli, _started False kalmalı.
        n._on_state(_String(data="PARKUR1"))
        assert n._started is False
    finally:
        n.destroy_node()


# --------------------------------------------------------------------------- #
# F-M.1 — masa OOM olayının guard'ları (2026-07-12)
# --------------------------------------------------------------------------- #


def test_fm1_null_island_fix_yoksayilir_gorev_baslamaz(ros_context, mission_file) -> None:  # noqa: ANN001
    """status=FIX ama (0,0) konum (ArduPilot fix'siz çıktısı) → konum yok
    sayılır, FSM aktif olsa da görev BAŞLAMAZ (masa senaryosu birebir)."""
    n = girdap.MissionManagerNode(
        parameter_overrides=[
            Parameter("mission_file", Parameter.Type.STRING, mission_file),
        ]
    )
    helper = rclpy.create_node("test_mm_fm1_null")
    gps_pub = helper.create_publisher(NavSatFix, "/mavros/global_position/global", 10)
    state_pub = helper.create_publisher(String, "/girdap/mission/state", 10)
    try:
        fix = NavSatFix()
        fix.status.status = 0                    # STATUS_FIX görünüyor ama...
        fix.latitude = 0.0                       # ...null island
        fix.longitude = 0.0
        end = time.monotonic() + 1.0
        while time.monotonic() < end:
            gps_pub.publish(fix)
            state_pub.publish(String(data="PARKUR1"))
            rclpy.spin_once(n, timeout_sec=0.02)
            rclpy.spin_once(helper, timeout_sec=0.02)
        assert n._lat is None                    # (0,0) konum olarak cache'lenmedi
        assert n._started is False               # görev başlamadı
    finally:
        helper.destroy_node()
        n.destroy_node()


def test_fm1_uzak_hedef_gorevi_reddeder(ros_context, tmp_path) -> None:  # noqa: ANN001
    """Geçerli fix VAR ama hedef ~111 km uzakta (> max_target_distance_m)
    → görev reddedilir; _started latch'lenmez (düzeltilmiş görevle tekrar
    denenebilir)."""
    path = tmp_path / "uzak.yaml"
    path.write_text(
        "waypoints:\n  - {lat: 40.0, lon: 29.0, parkur: 1}\n"
        "arrival_radius_m: 5.0\ndwell_time_s: 1.0\n",
        encoding="utf-8",
    )
    n = girdap.MissionManagerNode(
        parameter_overrides=[
            Parameter("mission_file", Parameter.Type.STRING, str(path)),
        ]
    )
    helper = rclpy.create_node("test_mm_fm1_uzak")
    gps_pub = helper.create_publisher(NavSatFix, "/mavros/global_position/global", 10)
    state_pub = helper.create_publisher(String, "/girdap/mission/state", 10)
    try:
        fix = NavSatFix()
        fix.status.status = 0
        fix.latitude = 41.0                      # hedefe ~111 km
        fix.longitude = 29.0
        end = time.monotonic() + 1.0
        while time.monotonic() < end:
            gps_pub.publish(fix)
            state_pub.publish(String(data="PARKUR1"))
            rclpy.spin_once(n, timeout_sec=0.02)
            rclpy.spin_once(helper, timeout_sec=0.02)
        assert n._lat is not None                # fix geçerli, cache'lendi
        assert n._started is False               # ama görev reddedildi
    finally:
        helper.destroy_node()
        n.destroy_node()
