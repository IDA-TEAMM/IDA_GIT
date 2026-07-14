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
from geometry_msgs.msg import PoseStamped, Twist         # noqa: E402
from nav_msgs.msg import Odometry                        # noqa: E402
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


def test_fc_source_thrust_from_rc_out_not_mppi(ros_context, tmp_path) -> None:
    """
    AUTO video kararı (2026-07-13, setpoint_source="fc"):

    AUTO'da aracı FC sürer; MPPI'nin /girdap/control/thrust'ı aracı SÜRMEZ →
    Ekran-2c oradan beslenirse araç hareketiyle senkronsuz SAHTE grafik çıkar
    (md 3.3.1.1 "istemsiz/senkronsuz = BAŞARISIZ"). fc modunda:
      - thrust /mavros/rc/out PWM'inden % olarak türetilir (1750→50, 1250→-50)
      - /girdap/control/thrust YOK SAYILIR (abonelik hiç kurulmaz)
      - hiz_setpoint görev aktifken (PARKUR1) fc_cruise_setpoint_mps sabitidir
    """
    mavros_msgs = pytest.importorskip(
        "mavros_msgs.msg", reason="mavros_msgs yok — ROS ortamında koş"
    )
    from std_msgs.msg import String                       # noqa: E402

    node = girdap.TelemetryNode(
        parameter_overrides=[
            Parameter("csv_output_dir", Parameter.Type.STRING,
                      str(tmp_path / "telemetry")),
            Parameter("graph_output_dir", Parameter.Type.STRING,
                      str(tmp_path / "grafik")),
            Parameter("graph_rate_hz", Parameter.Type.DOUBLE, 20.0),
            Parameter("setpoint_source", Parameter.Type.STRING, "fc"),
            Parameter("fc_cruise_setpoint_mps", Parameter.Type.DOUBLE, 1.2),
            Parameter("fc_thrust_left_ch", Parameter.Type.INTEGER, 1),
            Parameter("fc_thrust_right_ch", Parameter.Type.INTEGER, 3),
        ]
    )
    helper = rclpy.create_node("test_telemetry_fc_helper")
    # rc/out BEST_EFFORT yayınlanır (mavros sensör kategorisi) — abone
    # sensor_data_qos olduğundan BEST_EFFORT publisher ile eşleşir.
    from girdap_decision.qos_profiles import sensor_data_qos  # noqa: E402

    rcout_pub = helper.create_publisher(
        mavros_msgs.RCOut, "/mavros/rc/out", sensor_data_qos()
    )
    mppi_pub = helper.create_publisher(
        Float32MultiArray, "/girdap/control/thrust", 10
    )
    state_pub = helper.create_publisher(String, "/girdap/mission/state", 10)
    try:
        rcout = mavros_msgs.RCOut()
        # SERVO1=1750 (sol +50%), SERVO2 boş, SERVO3=1250 (sağ -50%)
        rcout.channels = [1750, 0, 1250, 0, 0, 0, 0, 0]
        fake_mppi = Float32MultiArray()
        fake_mppi.data = [99.0, 99.0]              # görünmemeli (sahte veri)
        state = String()
        state.data = "PARKUR1"                     # görev aktif → cruise sp

        deadline = time.monotonic() + 5.0
        rows: list[list[str]] = []
        while time.monotonic() < deadline:
            rcout_pub.publish(rcout)
            mppi_pub.publish(fake_mppi)
            state_pub.publish(state)
            rclpy.spin_once(helper, timeout_sec=0.01)
            for _ in range(6):
                rclpy.spin_once(node, timeout_sec=0.01)
            with open(node._graph_csv.path, newline="", encoding="utf-8") as f:
                rows = list(csv.reader(f))
            if len(rows) >= 2 and rows[-1][5] != "":
                break
        data = rows[-1]
        assert data[5] == "50.00", "thrust_sol rc/out PWM'inden gelmeli (%)"
        assert data[6] == "-50.00", "thrust_sag rc/out PWM'inden gelmeli (%)"
        assert data[5] != "99.00", "MPPI thrust'ı fc modunda YOK SAYILMALI"
        assert data[2] == "1.200", "hiz_setpoint görev aktifken cruise sabiti"
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
