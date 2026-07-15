"""
Girdap İDA — telemetry_node grafik CSV testi (T0-g, md 3.3.1.1 Ekran-2).

Doğrular:
    - grafik_<UTC>.csv GRAPH_CSV_HEADER ile açılır, Dosya-2 CSV'si değişmez
    - /girdap/control/thrust [T_sol, T_sag] grafik satırına yazılır (Ekran-2c)
    - F15.4: velocity_body yokken hız odom twist'ten yedeklenir

rclpy gerektirir → sistem python3.10 + ROS Humble; .venv'de SKIP.
"""

from __future__ import annotations

import csv
import time

import pytest

rclpy = pytest.importorskip("rclpy", reason="rclpy yok (.venv) — ROS ortamında koş")

from rclpy.parameter import Parameter                    # noqa: E402
from geometry_msgs.msg import PoseStamped, Twist, Vector3  # noqa: E402
from nav_msgs.msg import Odometry                        # noqa: E402
from sensor_msgs.msg import NavSatFix                     # noqa: E402
from std_msgs.msg import Float32MultiArray, String       # noqa: E402

girdap = pytest.importorskip(
    "girdap_decision.telemetry_node",
    reason="girdap_decision source'lanmamış (ros2_ws/install/setup.bash)",
)

from prototype.telemetry.csv_logger import CSV_HEADER, GRAPH_CSV_HEADER  # noqa: E402


@pytest.fixture(scope="module")
def ros_context():                                       # noqa: ANN201
    rclpy.init()
    yield
    rclpy.shutdown()


def test_graph_csv_receives_thrust_and_odom_speed(ros_context, tmp_path) -> None:
    """thrust + odom yayınla → grafik CSV'de thrust sütunları + hız yedeği."""
    node = girdap.TelemetryNode(
        parameter_overrides=[
            Parameter("csv_output_dir", Parameter.Type.STRING,
                      str(tmp_path / "telemetry")),
            Parameter("graph_output_dir", Parameter.Type.STRING,
                      str(tmp_path / "grafik")),
            Parameter("graph_rate_hz", Parameter.Type.DOUBLE, 20.0),
        ]
    )
    helper = rclpy.create_node("test_telemetry_helper")
    thrust_pub = helper.create_publisher(
        Float32MultiArray, "/girdap/control/thrust", 10
    )
    odom_pub = helper.create_publisher(Odometry, "/girdap/fusion/odom", 10)
    try:
        # Dosya adları ve header'lar
        assert node._csv.path.name.startswith("telemetri_")
        assert node._graph_csv.path.name.startswith("grafik_")

        thrust = Float32MultiArray()
        thrust.data = [12.3, -7.0]
        odom = Odometry()
        odom.twist.twist.linear.x = 0.6          # F15.4: body vel yok → odom
        odom.twist.twist.linear.y = 0.8          # hız = 1.0

        deadline = time.monotonic() + 5.0
        rows: list[list[str]] = []
        while time.monotonic() < deadline:
            thrust_pub.publish(thrust)
            odom_pub.publish(odom)
            # Timer 20 Hz'te sürekli hazır → tek spin_once aboneliği aç
            # bırakabilir (wait-set'te timer önce); birkaç kez spin et.
            rclpy.spin_once(helper, timeout_sec=0.01)
            for _ in range(6):
                rclpy.spin_once(node, timeout_sec=0.01)
            with open(node._graph_csv.path, newline="", encoding="utf-8") as f:
                rows = list(csv.reader(f))
            # header + en az 1 dolu satır (thrust sütunu boş değil)
            if len(rows) >= 2 and rows[-1][5] != "":
                break
        assert rows[0] == GRAPH_CSV_HEADER
        data = rows[-1]
        assert data[5] == "12.30"                # thrust_sol, 2 ondalık
        assert data[6] == "-7.00"                # thrust_sag
        assert data[1] == "1.000"                # hız odom yedeğinden (F15.4)

        # Dosya-2 sözleşmesi bozulmadı
        with open(node._csv.path, newline="", encoding="utf-8") as f:
            header = next(csv.reader(f))
        assert header == CSV_HEADER
    finally:
        helper.destroy_node()
        node.destroy_node()


def test_yon_setpoint_is_angle_from_current_target(ros_context, tmp_path) -> None:
    """
    F-V.1 (Şartname md 3.3.1.1 Ekran-2b): yon_setpoint bir AÇI olmalı —
    'Gerçek heading/yaw açısı, heading/yaw açısı isteği (setpoint)'.

    /girdap/mission/current_target araç-göreli ENU ofsettir (x=Doğu, y=Kuzey);
    istenen rota açısı atan2(y, x) — heading ile aynı ENU konvansiyonu.
    cmd_vel'in angular.z'si yaw HIZIDIR (rad/s), yon_setpoint'e YAZILMAZ.
    """
    node = girdap.TelemetryNode(
        parameter_overrides=[
            Parameter("csv_output_dir", Parameter.Type.STRING,
                      str(tmp_path / "telemetry")),
            Parameter("graph_output_dir", Parameter.Type.STRING,
                      str(tmp_path / "grafik")),
            Parameter("graph_rate_hz", Parameter.Type.DOUBLE, 20.0),
        ]
    )
    helper = rclpy.create_node("test_telemetry_target_helper")
    target_pub = helper.create_publisher(
        PoseStamped, "/girdap/mission/current_target", 10
    )
    cmd_pub = helper.create_publisher(
        Twist, "/mavros/setpoint_velocity/cmd_vel_unstamped", 10
    )
    state_pub = helper.create_publisher(String, "/girdap/mission/state", 10)
    try:
        target = PoseStamped()
        target.pose.position.x = 3.0             # Doğu ofset (m)
        target.pose.position.y = 4.0             # Kuzey ofset (m)
        cmd = Twist()
        cmd.linear.x = 0.8                       # hız isteği → hiz_setpoint
        cmd.angular.z = 0.5                      # yaw HIZI — yon_setpoint'e girmemeli

        deadline = time.monotonic() + 5.0
        rows: list[list[str]] = []
        while time.monotonic() < deadline:
            # F-V.2: setpoint sütunları yalnız görev aktifken yazılır.
            state_pub.publish(String(data="PARKUR1"))
            target_pub.publish(target)
            cmd_pub.publish(cmd)
            rclpy.spin_once(helper, timeout_sec=0.01)
            for _ in range(6):
                rclpy.spin_once(node, timeout_sec=0.01)
            with open(node._graph_csv.path, newline="", encoding="utf-8") as f:
                rows = list(csv.reader(f))
            if len(rows) >= 2 and rows[-1][4] != "":
                break
        data = rows[-1]
        assert data[4] == "0.927"                # atan2(4,3) rad — AÇI, 0.500 değil
        assert data[2] == "0.800"                # hiz_setpoint cmd_vel'den sürer
    finally:
        helper.destroy_node()
        node.destroy_node()


def test_setpointler_gorev_aktif_degilken_bos(ros_context, tmp_path) -> None:
    """
    F-V.2: setpoint sütunları yalnız görev AKTİFKEN (PARKUR1/2/3) yazılır.

    Görev TAMAMLANDI/KILL olunca current_target yayını durur; cache'teki son
    açı yazılmaya devam ederse manuel dönüşte heading değişirken grafik
    "istek var" gibi okunur (md 3.3.1.1 senkron/anlaşılırlık riski). Boş
    hücre → ekran2 NaN boşluğu (sahte çizgi yok).
    """
    node = girdap.TelemetryNode(
        parameter_overrides=[
            Parameter("csv_output_dir", Parameter.Type.STRING,
                      str(tmp_path / "telemetry")),
            Parameter("graph_output_dir", Parameter.Type.STRING,
                      str(tmp_path / "grafik")),
            Parameter("graph_rate_hz", Parameter.Type.DOUBLE, 20.0),
        ]
    )
    helper = rclpy.create_node("test_telemetry_state_helper")
    target_pub = helper.create_publisher(
        PoseStamped, "/girdap/mission/current_target", 10
    )
    cmd_pub = helper.create_publisher(
        Twist, "/mavros/setpoint_velocity/cmd_vel_unstamped", 10
    )
    state_pub = helper.create_publisher(String, "/girdap/mission/state", 10)

    def _pump(state: str, rounds: int = 8) -> list[list[str]]:
        target = PoseStamped()
        target.pose.position.x = 3.0
        target.pose.position.y = 4.0
        cmd = Twist()
        cmd.linear.x = 0.8
        for _ in range(rounds):
            state_pub.publish(String(data=state))
            target_pub.publish(target)
            cmd_pub.publish(cmd)
            rclpy.spin_once(helper, timeout_sec=0.01)
            for _ in range(6):
                rclpy.spin_once(node, timeout_sec=0.01)
        with open(node._graph_csv.path, newline="", encoding="utf-8") as f:
            return list(csv.reader(f))

    try:
        # Görev aktif → setpoint'ler yazılır.
        deadline = time.monotonic() + 5.0
        rows: list[list[str]] = []
        while time.monotonic() < deadline:
            rows = _pump("PARKUR1", rounds=2)
            if len(rows) >= 2 and rows[-1][4] != "":
                break
        assert rows[-1][4] == "0.927" and rows[-1][2] == "0.800"

        # Görev bitti → yeni satırlarda setpoint sütunları BOŞ (cache dolu olsa da).
        rows = _pump("TAMAMLANDI")
        data = rows[-1]
        assert data[4] == "", f"yon_setpoint donuk yazılıyor: {data[4]}"
        assert data[2] == "", f"hiz_setpoint donuk yazılıyor: {data[2]}"
    finally:
        helper.destroy_node()
        node.destroy_node()


def test_bulgu2_kaynak_susunca_donuk_deger_yazilmiyor(ros_context, tmp_path) -> None:
    """BULGU 2 repro (Yahya, son_kod video koşul matrisi 2026-07-14): GPS
    kaynağı susunca Dosya-2 CSV'si son değeri DONUK tekrarlıyordu — sanki
    veri hâlâ canlıymış gibi (ör. hız sabit yazılıyor). F-V.2'nin setpoint
    kapılama desenini sensör alanlarına da uygular: source_timeout_s
    aşılınca lat/lon BOŞ yazılır (donuk değer değil)."""
    node = girdap.TelemetryNode(
        parameter_overrides=[
            Parameter("csv_output_dir", Parameter.Type.STRING,
                      str(tmp_path / "telemetry")),
            Parameter("graph_output_dir", Parameter.Type.STRING,
                      str(tmp_path / "grafik")),
            Parameter("source_timeout_s", Parameter.Type.DOUBLE, 1.0),
        ]
    )
    try:
        t = [100.0]
        node._now = lambda: t[0]                      # sahte saat
        gps = NavSatFix()
        gps.status.status = 0
        gps.latitude = 40.71
        gps.longitude = 31.52
        node._on_gps(gps)
        node._on_write()
        with open(node._csv.path, newline="", encoding="utf-8") as f:
            rows = list(csv.reader(f))
        assert rows[-1][1] != "", "taze GPS boş yazıldı"

        t[0] = 102.0                                  # 2 s sessizlik > 1.0 s eşik
        node._on_write()
        with open(node._csv.path, newline="", encoding="utf-8") as f:
            rows = list(csv.reader(f))
        assert rows[-1][1] == "", f"bayat GPS hâlâ donuk yazılıyor: {rows[-1][1]}"
        assert rows[-1][2] == "", "bayat lon hâlâ donuk yazılıyor"
    finally:
        node.destroy_node()


def test_bulgu2_source_timeout_kapali_eski_davranis(ros_context, tmp_path) -> None:
    """source_timeout_s<=0 → bekçi devre dışı (mock/masa testi geriye uyum)."""
    node = girdap.TelemetryNode(
        parameter_overrides=[
            Parameter("csv_output_dir", Parameter.Type.STRING,
                      str(tmp_path / "telemetry")),
            Parameter("graph_output_dir", Parameter.Type.STRING,
                      str(tmp_path / "grafik")),
            Parameter("source_timeout_s", Parameter.Type.DOUBLE, 0.0),
        ]
    )
    try:
        t = [100.0]
        node._now = lambda: t[0]
        gps = NavSatFix()
        gps.status.status = 0
        gps.latitude = 40.71
        gps.longitude = 31.52
        node._on_gps(gps)
        t[0] = 999999.0                               # çok uzun sessizlik
        node._on_write()
        with open(node._csv.path, newline="", encoding="utf-8") as f:
            rows = list(csv.reader(f))
        assert rows[-1][1] != "", "timeout kapalıyken GPS yine de boşaltıldı"
    finally:
        node.destroy_node()


# ----- B2: setpoint_source="fc" (AUTO video, md 3.3.1.1 Ekran-2 dürüstlüğü) -----
#
# AUTO+FC videosunda görevi FC kendi uçurur; MPPI cmd_vel BASMAZ (planning
# mod geçidi GUIDED bekler). O yüzden Ekran-2'de MPPI thrust'ını göstermek
# YANILTICI olur — gerçek kuvvet isteği FC'nin servo çıkışıdır. fc modunda
# thrust /mavros/rc/out PWM'inden ±%100'e normalize edilir, hız setpoint'i ise
# FC'nin WP_SPEED'i (fc_cruise_setpoint_mps) ile SENKRON sabit değerdir.
#
# YARIŞMA varsayılanı DEĞİŞMEZ: setpoint_source="girdap" (MPPI thrust'ı, N).

_GRAPH_HIZ_SP = GRAPH_CSV_HEADER.index("hiz_setpoint")
_GRAPH_T_SOL = GRAPH_CSV_HEADER.index("thrust_sol")
_GRAPH_T_SAG = GRAPH_CSV_HEADER.index("thrust_sag")


def _fc_node(tmp_path, **extra):                         # noqa: ANN001, ANN201
    from rclpy.parameter import Parameter

    params = [
        Parameter("csv_output_dir", Parameter.Type.STRING,
                  str(tmp_path / "telemetry")),
        Parameter("graph_output_dir", Parameter.Type.STRING,
                  str(tmp_path / "grafik")),
        Parameter("setpoint_source", Parameter.Type.STRING, "fc"),
        Parameter("fc_cruise_setpoint_mps", Parameter.Type.DOUBLE, 1.2),
    ]
    params.extend(extra.get("extra_params", []))
    return girdap.TelemetryNode(parameter_overrides=params)


def _rc_out(channels: list[int]):                        # noqa: ANN201
    from mavros_msgs.msg import RCOut

    msg = RCOut()
    msg.channels = channels
    return msg


def _last_graph_row(node) -> list[str]:                  # noqa: ANN001
    with open(node._graph_csv.path, newline="", encoding="utf-8") as fh:
        return list(csv.reader(fh))[-1]


def test_fc_modu_thrust_rc_out_pwm_yuzdesi(ros_context, tmp_path) -> None:
    """rc/out PWM → ±%100 thrust; hiz_setpoint = fc_cruise (görev aktifken)."""
    pytest.importorskip("mavros_msgs", reason="mavros_msgs yok")
    node = _fc_node(tmp_path)
    try:
        # kanal 1 (sol) = 2000 → +%100 ; kanal 3 (sağ) = 1000 → -%100
        node._on_rc_out(_rc_out([2000, 1500, 1000, 1500]))
        node._on_mission_state(String(data="PARKUR1"))
        node._on_graph_write()

        row = _last_graph_row(node)
        assert float(row[_GRAPH_T_SOL]) == pytest.approx(100.0)
        assert float(row[_GRAPH_T_SAG]) == pytest.approx(-100.0)
        assert float(row[_GRAPH_HIZ_SP]) == pytest.approx(1.2)
    finally:
        node.destroy_node()


def test_fc_modu_pwm_kirpilir_ve_bos_kanal_yazilmaz(ros_context, tmp_path) -> None:
    """Aralık dışı PWM ±%100'e kırpılır; PWM=0 (kanal pasif) → sütun BOŞ."""
    pytest.importorskip("mavros_msgs", reason="mavros_msgs yok")
    node = _fc_node(tmp_path)
    try:
        node._on_rc_out(_rc_out([2400, 1500, 800, 1500]))   # aralık dışı
        node._on_mission_state(String(data="PARKUR1"))
        node._on_graph_write()
        row = _last_graph_row(node)
        assert float(row[_GRAPH_T_SOL]) == pytest.approx(100.0)
        assert float(row[_GRAPH_T_SAG]) == pytest.approx(-100.0)

        # FC servo çıkışı vermiyor (PWM=0) → sahte -%100 YAZMA, boş bırak.
        node._on_rc_out(_rc_out([0, 0, 0, 0]))
        node._on_graph_write()
        row = _last_graph_row(node)
        assert row[_GRAPH_T_SOL] == "", "pasif kanal -%100 olarak yazılıyor"
        assert row[_GRAPH_T_SAG] == ""
    finally:
        node.destroy_node()


def test_fc_modu_gorev_aktif_degilken_hiz_setpoint_bos(ros_context, tmp_path) -> None:
    """F-V.2 korunur: BEKLEMEDE'de sabit cruise değeri YAZILMAZ (donuk çizgi yok)."""
    pytest.importorskip("mavros_msgs", reason="mavros_msgs yok")
    node = _fc_node(tmp_path)
    try:
        node._on_rc_out(_rc_out([1500, 1500, 1500, 1500]))
        node._on_mission_state(String(data="BEKLEMEDE"))
        node._on_graph_write()
        assert _last_graph_row(node)[_GRAPH_HIZ_SP] == ""
    finally:
        node.destroy_node()


def test_girdap_modu_varsayilan_mppi_thrustu_yazar(ros_context, tmp_path) -> None:
    """Yarışma varsayılanı: setpoint_source=girdap → rc/out aboneliği YOK."""
    from rclpy.parameter import Parameter

    node = girdap.TelemetryNode(
        parameter_overrides=[
            Parameter("csv_output_dir", Parameter.Type.STRING,
                      str(tmp_path / "telemetry")),
            Parameter("graph_output_dir", Parameter.Type.STRING,
                      str(tmp_path / "grafik")),
        ]
    )
    try:
        assert node._sub_rc is None                  # fc yolu kapalı
        node._on_thrust(Float32MultiArray(data=[12.0, -3.0]))
        node._on_setpoint(Twist(linear=Vector3(x=0.8)))
        node._on_mission_state(String(data="PARKUR1"))
        node._on_graph_write()

        row = _last_graph_row(node)
        assert float(row[_GRAPH_T_SOL]) == pytest.approx(12.0)   # N (MPPI)
        assert float(row[_GRAPH_HIZ_SP]) == pytest.approx(0.8)   # cmd_vel
    finally:
        node.destroy_node()


def test_fv7_waypoint_ustundeyken_yon_setpoint_savrulmaz(ros_context, tmp_path) -> None:
    """F-V.7: hedefe ~0 m kalınca atan2 çöp açı üretir → son geçerli açı korunur.

    AUTO'da tekne waypoint'te DURMAZ; üstünden geçerken current_target ofseti
    sıfıra yaklaşır, sonra ARKADA kalır → yon_setpoint ~180° savrulur. Ekran-2b
    zorunlu eğrisinde her waypoint'te çöp sıçrama demek (md 3.3.1.1 "net değil").
    """
    from rclpy.parameter import Parameter

    node = girdap.TelemetryNode(
        parameter_overrides=[
            Parameter("csv_output_dir", Parameter.Type.STRING, str(tmp_path / "t")),
            Parameter("graph_output_dir", Parameter.Type.STRING, str(tmp_path / "g")),
        ]
    )
    try:
        uzak = PoseStamped()
        uzak.pose.position.x = 10.0          # 10 m ileride → açı 0 rad
        uzak.pose.position.y = 0.0
        node._on_target(uzak)
        assert node._yaw_sp == pytest.approx(0.0)

        yakin = PoseStamped()                # waypoint'in üstündeyiz (5 cm)
        yakin.pose.position.x = -0.03
        yakin.pose.position.y = 0.04         # atan2 → ~127° (çöp)
        node._on_target(yakin)
        assert node._yaw_sp == pytest.approx(0.0), (
            "sıfıra yakın ofsetten açı üretildi — Ekran-2b'de çöp sıçrama"
        )
    finally:
        node.destroy_node()
