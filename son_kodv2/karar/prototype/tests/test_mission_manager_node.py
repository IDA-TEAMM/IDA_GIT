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
# F-P.4 (robustness taraması, 2026-07-15) — GPS bayatlık bekçisi. Diğer tüm
# kritik kaynaklarda (fusion pose_timeout_s, planning odom/obstacle_
# timeout_s, telemetry source_timeout_s) vardı, mission_manager'da YOKTU —
# GPS kesilirse _lat/_lon SONSUZA DEK donuk kalır, current_target/waypoints
# gerçeği yansıtmayan bir konumdan hesaplanmaya devam ederdi.
# --------------------------------------------------------------------------- #


def test_fp4_bayat_gps_yayin_durdurur(ros_context, mission_file) -> None:  # noqa: ANN001
    """gps_timeout_s aşılınca _on_tick hedef/waypoint yayınını durdurmalı."""
    n = girdap.MissionManagerNode(
        parameter_overrides=[
            Parameter("mission_file", Parameter.Type.STRING, mission_file),
            Parameter("gps_timeout_s", Parameter.Type.DOUBLE, 1.0),
        ]
    )
    try:
        t = [100.0]
        n._now = lambda: t[0]
        fix = NavSatFix()
        fix.status.status = 0
        fix.latitude, fix.longitude = 41.001, 29.000
        n._on_gps(fix)
        assert n._gps_stale() is False              # taze

        t[0] = 100.5
        assert n._gps_stale() is False               # eşik içinde

        t[0] = 101.5                                 # 1.5 s sessizlik
        assert n._gps_stale() is True, (
            "bayat GPS'le hesaplamaya devam ediyor (F-P.4)"
        )
    finally:
        n.destroy_node()


def test_fp4_gps_hic_gelmediyse_bayat_degil(ros_context, mission_file) -> None:  # noqa: ANN001
    """GPS hiç gelmediyse 'bayat' alarmı basılmaz — zaten `_lat is None`
    guard'ı ayrı ele alır (boot gürültüsü, F-P.1/F-P.2 ile aynı prensip)."""
    n = girdap.MissionManagerNode(
        parameter_overrides=[
            Parameter("mission_file", Parameter.Type.STRING, mission_file),
        ]
    )
    try:
        assert n._gps_stale() is False
    finally:
        n.destroy_node()


def test_fp4_kapatilabilir(ros_context, mission_file) -> None:  # noqa: ANN001
    """gps_timeout_s=0 → bekçi devre dışı (mock/masa testi geriye uyum)."""
    n = girdap.MissionManagerNode(
        parameter_overrides=[
            Parameter("mission_file", Parameter.Type.STRING, mission_file),
            Parameter("gps_timeout_s", Parameter.Type.DOUBLE, 0.0),
        ]
    )
    try:
        t = [100.0]
        n._now = lambda: t[0]
        fix = NavSatFix()
        fix.status.status = 0
        fix.latitude, fix.longitude = 41.001, 29.000
        n._on_gps(fix)
        t[0] = 999.0
        assert n._gps_stale() is False
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


# ----- F-V.8: FC varış senkronu (/mavros/mission/reached) -----


def _fc_list(n: int = 4):                                # noqa: ANN201
    """home + n gezinme waypoint'li sahte FC WaypointList (QGC yüklemesi)."""
    from mavros_msgs.msg import Waypoint as MavWp, WaypointList

    msg = WaypointList()
    pts = [(41.0, 29.0)] + [(41.001 + i * 0.001, 29.001) for i in range(n)]
    for lat, lon in pts:
        w = MavWp()
        w.command = 16
        w.x_lat = lat
        w.y_long = lon
        msg.waypoints.append(w)
    return msg


def _reached(seq: int):                                  # noqa: ANN201
    from mavros_msgs.msg import WaypointReached

    m = WaypointReached()
    m.wp_seq = seq
    return m


def test_fv8_fc_reached_indexi_senkronlar(ros_context) -> None:  # noqa: ANN001
    """FC 'vardım' dedikçe index ilerler; son seq görevi COMPLETE yapar."""
    pytest.importorskip("mavros_msgs", reason="mavros_msgs yok")
    node = girdap.MissionManagerNode(
        parameter_overrides=[
            Parameter("mission_source", Parameter.Type.STRING, "fc"),
            Parameter("dwell_time_s", Parameter.Type.DOUBLE, 0.0),
        ]
    )
    try:
        node._on_fc_waypoints(_fc_list(4))               # QGC → FC → mavros
        assert node._fc_seqs == [1, 2, 3, 4]             # home atlandı

        gps = NavSatFix()
        gps.latitude, gps.longitude = 41.0, 29.0
        node._on_gps(gps)
        node._on_state(String(data="PARKUR1"))           # FSM görevi başlattı
        assert node._started is True

        node._on_fc_reached(_reached(1))                 # FC: wp1'e vardım
        assert node._mgr.current_index == 1
        assert node._last_reached_pub == 0               # tek-atış yayınlandı

        node._on_fc_reached(_reached(1))                 # tekrar → etkisiz
        assert node._mgr.current_index == 1

        node._on_fc_reached(_reached(4))                 # son waypoint
        assert node._mgr.is_complete is True
    finally:
        node.destroy_node()


def test_fv8_baslamadan_gelen_reached_yok_sayilir(ros_context) -> None:  # noqa: ANN001
    """Bayat/erken reached (görev başlamadan) index'i İLERLETMEMELİ."""
    pytest.importorskip("mavros_msgs", reason="mavros_msgs yok")
    node = girdap.MissionManagerNode(
        parameter_overrides=[
            Parameter("mission_source", Parameter.Type.STRING, "fc"),
        ]
    )
    try:
        node._on_fc_waypoints(_fc_list(4))
        node._on_fc_reached(_reached(2))                 # görev başlamadı
        assert node._mgr.current_index == 0
        assert node._mgr.is_complete is False
        assert node._last_reached_pub == -1              # yayın da yok
    finally:
        node.destroy_node()


# --------------------------------------------------------------------------- #
# F-S.6/F-S.11: /girdap/mission/waypoints hiç publish edilmiyordu — planning_
# node'un RRT* girdisi (use_rrt=true) hiçbir zaman gelmiyordu, global plan hiç
# oluşmuyordu (thrust sıfırda kalırdı). F-S.6 TÜM waypoint listesini birden
# yayınlayarak bunu "düzeltti" ama PlanningPipeline._global_replan() RRT*
# hedefini HER ZAMAN listenin SON elemanı alır (test_kamikaze_target_is_
# last_waypoint bunu doğrular) — yani ara waypoint'ler (slalom kapıları)
# tamamen atlanıyordu, sıralı waypoint_reached hiç tetiklenmiyordu (gerçek
# parkur dosyasıyla SITL'de canlı bulundu). F-S.11: yalnız O ANKİ AKTİF
# waypoint yayınlanır — current_target ile aynı desen, sıralı ilerleme geri
# kazanıldı.
# --------------------------------------------------------------------------- #


@pytest.fixture
def mission_file_distinct(tmp_path_factory) -> str:      # noqa: ANN001
    """2 FARKLI koordinatlı waypoint — ENU dönüşümünü anlamlı doğrulamak için."""
    path = tmp_path_factory.mktemp("mm_wp") / "m2.yaml"
    path.write_text(
        textwrap.dedent(
            """
            waypoints:
              - {lat: 41.001, lon: 29.000, parkur: 1}
              - {lat: 41.002, lon: 29.001, parkur: 1}
            arrival_radius_m: 5.0
            dwell_time_s: 30.0
            """
        ),
        encoding="utf-8",
    )
    return str(path)


def test_fs6_waypoints_path_yayinlanir(ros_context, mission_file_distinct) -> None:  # noqa: ANN001
    """F-S.11: yalnız O ANKİ AKTİF waypoint (tek eleman) yayınlanır —
    tüm liste DEĞİL (bkz. yukarıdaki F-S.6/F-S.11 notu: RRT* hedefi her
    zaman listenin SON elemanını alır, çoklu waypoint verilirse aradakiler
    hiç ziyaret edilmez)."""
    from nav_msgs.msg import Path
    from std_msgs.msg import String

    from prototype.mission.mission_manager import latlon_to_enu

    node = girdap.MissionManagerNode(
        parameter_overrides=[
            Parameter("mission_file", Parameter.Type.STRING, mission_file_distinct),
        ]
    )
    helper = rclpy.create_node("test_mm_wp_helper")
    paths: list = []
    helper.create_subscription(
        Path, "/girdap/mission/waypoints", lambda m: paths.append(m), 10
    )
    gps_pub = helper.create_publisher(
        NavSatFix, "/mavros/global_position/global", 10
    )
    state_pub = helper.create_publisher(String, "/girdap/mission/state", 10)
    try:
        fix = NavSatFix()
        fix.status.status = 0
        fix.latitude, fix.longitude = 40.999, 28.999   # araç, waypoint'lerden ayrı bir konumda

        # current_waypoint yalnız görev AKTİFKEN (IDLE/COMPLETE değil) None
        # dönmez — F-S.11 sonrası yayın için görevin başlamış olması gerekir.
        deadline = time.monotonic() + 5.0
        while not paths:
            gps_pub.publish(fix)
            state_pub.publish(String(data="PARKUR1"))
            rclpy.spin_once(node, timeout_sec=0.05)
            rclpy.spin_once(helper, timeout_sec=0.05)
            assert time.monotonic() < deadline, "/girdap/mission/waypoints yayınlanmadı (F-S.6)"

        path_msg = paths[-1]
        assert path_msg.header.frame_id == "base_link"
        assert len(path_msg.poses) == 1, (
            "F-S.11: birden fazla waypoint yayınlanıyor — RRT* ara "
            "noktaları atlayıp doğrudan sona gider (regresyon)"
        )

        exp0 = latlon_to_enu(fix.latitude, fix.longitude, 41.001, 29.000)
        got0 = (path_msg.poses[0].pose.position.x, path_msg.poses[0].pose.position.y)
        assert got0 == pytest.approx(exp0, abs=1e-6)
    finally:
        helper.destroy_node()
        node.destroy_node()


@pytest.fixture
def mission_file_short_dwell(tmp_path_factory) -> str:      # noqa: ANN001
    """mission_file_distinct ile aynı waypoint'ler, ama dwell_time_s KISA —
    ACTIVE→DWELL→(idx++)→ACTIVE geçişini testte 30 s beklemeden gözlemlemek
    için (mission_file_distinct'in 30 s'lik dwell'i kasıtlı, başka testler
    ona bağımlı olabilir — dokunulmadı, ayrı fixture)."""
    path = tmp_path_factory.mktemp("mm_wp_sd") / "m3.yaml"
    path.write_text(
        textwrap.dedent(
            """
            waypoints:
              - {lat: 41.001, lon: 29.000, parkur: 1}
              - {lat: 41.002, lon: 29.001, parkur: 1}
            arrival_radius_m: 5.0
            dwell_time_s: 0.05
            """
        ),
        encoding="utf-8",
    )
    return str(path)


def test_fs11_waypoints_path_ilerler_sirayla(ros_context, mission_file_short_dwell) -> None:  # noqa: ANN001
    """F-S.11: ilk waypoint'e varılınca yayınlanan tek-elemanlı liste bir
    SONRAKİ waypoint'e geçmeli — RRT* modunda sıralı ilerlemenin gerçekten
    geri kazanıldığını doğrular (SITL'de bulunan gerçek regresyonun testi)."""
    from nav_msgs.msg import Path
    from std_msgs.msg import String

    from prototype.mission.mission_manager import latlon_to_enu

    node = girdap.MissionManagerNode(
        parameter_overrides=[
            Parameter("mission_file", Parameter.Type.STRING, mission_file_short_dwell),
        ]
    )
    helper = rclpy.create_node("test_mm_wp_advance_helper")
    paths: list = []
    helper.create_subscription(
        Path, "/girdap/mission/waypoints", lambda m: paths.append(m), 10
    )
    gps_pub = helper.create_publisher(
        NavSatFix, "/mavros/global_position/global", 10
    )
    state_pub = helper.create_publisher(String, "/girdap/mission/state", 10)
    try:
        # Aracı doğrudan İLK waypoint'in (41.001, 29.000) ÜSTÜNE koy —
        # arrival_radius_m=5.0 içinde, hemen "varıldı" tetiklenir.
        fix = NavSatFix()
        fix.status.status = 0
        fix.latitude, fix.longitude = 41.001, 29.000

        deadline = time.monotonic() + 5.0
        while node._mgr.current_index == 0:
            gps_pub.publish(fix)
            state_pub.publish(String(data="PARKUR1"))
            rclpy.spin_once(node, timeout_sec=0.05)
            rclpy.spin_once(helper, timeout_sec=0.05)
            assert time.monotonic() < deadline, (
                "waypoint 0'a varılmasına rağmen index ilerlemedi (dwell "
                "sonrası) — dwell_time_s=30 çok uzun, node parametresini "
                "kısa tut ya da dwell'i bekleme mantığını kontrol et"
            )
        # index 1'e geçti — yayınlanan tek waypoint artık İKİNCİ olmalı.
        deadline = time.monotonic() + 5.0
        while not paths or paths[-1].poses[0].pose.position.x == pytest.approx(
            latlon_to_enu(fix.latitude, fix.longitude, 41.001, 29.000)[0]
        ):
            gps_pub.publish(fix)
            rclpy.spin_once(node, timeout_sec=0.05)
            rclpy.spin_once(helper, timeout_sec=0.05)
            if time.monotonic() > deadline:
                break
        assert len(paths[-1].poses) == 1
        exp1 = latlon_to_enu(fix.latitude, fix.longitude, 41.002, 29.001)
        got = (paths[-1].poses[0].pose.position.x, paths[-1].poses[0].pose.position.y)
        assert got == pytest.approx(exp1, abs=1e-6), (
            "waypoint 0 varıldıktan sonra yayınlanan hedef HÂLÂ waypoint 0 "
            "— RRT* sıralı ilerlemiyor (asıl SITL regresyonu)"
        )
    finally:
        helper.destroy_node()
        node.destroy_node()
