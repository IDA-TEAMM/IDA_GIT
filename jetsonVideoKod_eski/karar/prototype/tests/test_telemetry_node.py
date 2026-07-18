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
from geometry_msgs.msg import (                          # noqa: E402
    PoseStamped, Twist, TwistStamped, Vector3,
)
from nav_msgs.msg import Odometry                        # noqa: E402
from sensor_msgs.msg import NavSatFix                    # noqa: E402
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
            Parameter("kayit_dir", Parameter.Type.STRING,
                      str(tmp_path / "kayit")),
            Parameter("graph_rate_hz", Parameter.Type.DOUBLE, 20.0),
        ]
    )
    helper = rclpy.create_node("test_telemetry_helper")
    thrust_pub = helper.create_publisher(
        Float32MultiArray, "/girdap/control/thrust", 10
    )
    odom_pub = helper.create_publisher(Odometry, "/girdap/fusion/odom", 10)
    try:
        # Kayıt düzeni: kayit/1/ altında sabit adlı çift (Eyüp 16.07)
        assert node._csv.path.name == "telemetri.csv"
        assert node._graph_csv.path.name == "grafik.csv"
        assert node._csv.path.parent == node._graph_csv.path.parent
        assert node._csv.path.parent.name == "1"

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
            Parameter("kayit_dir", Parameter.Type.STRING,
                      str(tmp_path / "kayit")),
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
            Parameter("kayit_dir", Parameter.Type.STRING,
                      str(tmp_path / "kayit")),
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
        Parameter("kayit_dir", Parameter.Type.STRING,
                  str(tmp_path / "kayit")),
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
            Parameter("kayit_dir", Parameter.Type.STRING,
                      str(tmp_path / "kayit")),
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
            Parameter("kayit_dir", Parameter.Type.STRING, str(tmp_path / "kayit")),
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


# --------------------- BULGU 2: kaynak-tazelik bekçisi ---------------------
# Yahya sonkodv3 denetimi (2026-07-15), gps-kayip senaryosunda görüldü: kaynak
# susunca Dosya-2 (md 4.2) son değeri DONUK tekrarlıyordu — hakeme veri hâlâ
# canlıymış gibi görünür. F-V.2'nin setpoint kapılama deseni sensör sütunlarına
# da uygulanır: bilinmiyorsa BOŞ hücre yazılır, uydurma değil.

def _telemetry_node(tmp_path, timeout: float):           # noqa: ANN001, ANN201
    return girdap.TelemetryNode(
        parameter_overrides=[
            Parameter("kayit_dir", Parameter.Type.STRING,
                      str(tmp_path / "kayit")),
            Parameter("source_timeout_s", Parameter.Type.DOUBLE, timeout),
        ]
    )


def _last_row(node) -> list:                             # noqa: ANN001
    with open(node._csv.path, newline="", encoding="utf-8") as f:
        return list(csv.reader(f))[-1]


def test_bulgu2_gps_susunca_lat_lon_donuk_yazilmiyor(ros_context, tmp_path) -> None:
    """GPS `source_timeout_s`'ten uzun susunca lat/lon BOŞ olmalı."""
    node = _telemetry_node(tmp_path, timeout=1.0)
    try:
        t = [100.0]
        node._now = lambda: t[0]                         # sahte saat enjeksiyonu
        gps = NavSatFix()
        gps.status.status = 0
        gps.latitude = 40.71
        gps.longitude = 31.52
        node._on_gps(gps)
        node._on_write()
        assert _last_row(node)[1] != "", "taze GPS boş yazıldı"

        t[0] = 102.0                                     # 2 s > 1.0 s eşik
        node._on_write()
        row = _last_row(node)
        assert row[1] == "", f"bayat lat donuk yazılıyor: {row[1]} (BULGU 2)"
        assert row[2] == "", f"bayat lon donuk yazılıyor: {row[2]} (BULGU 2)"
    finally:
        node.destroy_node()


def test_bulgu2_hiz_kaynagi_susunca_bos(ros_context, tmp_path) -> None:
    """velocity_body susunca hız donuk kalmamalı (gps-kayip senaryosunun
    asıl bulgusu: Ekran-2'de hız çizgisi sabit değerde donuyordu)."""
    node = _telemetry_node(tmp_path, timeout=1.0)
    try:
        t = [50.0]
        node._now = lambda: t[0]
        vel = TwistStamped()
        vel.twist.linear.x = 1.4
        node._on_vel_body(vel)
        node._on_write()
        assert _last_row(node)[3] != "", "taze hız boş yazıldı"

        t[0] = 53.0                                      # 3 s sessizlik
        node._on_write()
        assert _last_row(node)[3] == "", "bayat hız donuk yazılıyor (BULGU 2)"
    finally:
        node.destroy_node()


def test_bulgu2_bekci_kapali_eski_davranis(ros_context, tmp_path) -> None:
    """`source_timeout_s <= 0` → bekçi kapalı (masa testi/mock yayıncı düşük
    hızlıysa sütunlar boşalmasın). Geri uyumluluk garantisi."""
    node = _telemetry_node(tmp_path, timeout=0.0)
    try:
        t = [10.0]
        node._now = lambda: t[0]
        gps = NavSatFix()
        gps.status.status = 0
        gps.latitude = 40.71
        gps.longitude = 31.52
        node._on_gps(gps)
        t[0] = 999.0                                     # bekçi kapalı → önemsiz
        node._on_write()
        assert _last_row(node)[1] != "", "bekçi kapalıyken sütun boşaldı"
    finally:
        node.destroy_node()


# ------- F-T.3 (öz-denetim turu 2026-07-15): F-T.1'in kalan yüzeyleri -------
# F-T.1 sensör sütunlarını (lat/lon/hız/rpy) tazelik bekçisine bağladı ama AYNI
# ekrandaki thrust ve setpoint sütunları açıkta kalmıştı: kaynak ölünce Ekran-2'de
# hız/heading dürüstçe boşalırken thrust/yon_setpoint DONUK akmaya devam ediyordu
# (aynı sahte-canlı-veri sınıfı, md 3.3.1.1 / md 4.2).

def test_ft3_thrust_kaynagi_susunca_grafik_bos(ros_context, tmp_path) -> None:
    """girdap modunda planning ölürse thrust sütunları donuk kalmamalı."""
    node = _telemetry_node(tmp_path, timeout=1.0)
    try:
        t = [200.0]
        node._now = lambda: t[0]
        node._on_thrust(Float32MultiArray(data=[12.5, -3.0]))
        node._on_graph_write()
        with open(node._graph_csv.path, newline="", encoding="utf-8") as f:
            row = list(csv.reader(f))[-1]
        assert row[5] != "" and row[6] != "", "taze thrust boş yazıldı"

        t[0] = 203.0                                 # 3 s > 1.0 s eşik
        node._on_graph_write()
        with open(node._graph_csv.path, newline="", encoding="utf-8") as f:
            row = list(csv.reader(f))[-1]
        assert row[5] == "" and row[6] == "", (
            f"bayat thrust donuk yazılıyor: {row[5]}/{row[6]} (F-T.3)"
        )
    finally:
        node.destroy_node()


def test_ft3_target_susunca_yon_setpoint_bos(ros_context, tmp_path) -> None:
    """Görev AKTİFKEN mission_manager ölürse yon_setpoint donuk kalmamalı
    (F-V.2 yalnız görev-dışını kapılıyordu; kaynak ölümü ayrı durum)."""
    node = _telemetry_node(tmp_path, timeout=1.0)
    try:
        t = [300.0]
        node._now = lambda: t[0]
        node._on_mission_state(String(data="PARKUR1"))
        target = PoseStamped()
        target.pose.position.x = 3.0
        target.pose.position.y = 4.0
        node._on_target(target)
        node._on_write()
        assert _last_row(node)[8] != "", "taze yon_setpoint boş yazıldı"

        t[0] = 303.0                                 # görev hâlâ aktif, kaynak öldü
        node._on_write()
        assert _last_row(node)[8] == "", (
            f"kaynak ölü ama yon_setpoint donuk: {_last_row(node)[8]} (F-T.3)"
        )
    finally:
        node.destroy_node()


def test_ft3_fv7_tutma_davranisi_bozulmadi(ros_context, tmp_path) -> None:
    """F-V.7 regresyonu: mesajlar AKMAYA DEVAM ederken (waypoint üstü, ofset
    küçük) son geçerli açı TUTULUR ve yazılır — tazelik 'kaynak canlı mı'ya
    bakar, 'değer güncellendi mi'ye değil."""
    node = _telemetry_node(tmp_path, timeout=1.0)
    try:
        t = [400.0]
        node._now = lambda: t[0]
        node._on_mission_state(String(data="PARKUR1"))
        far = PoseStamped()
        far.pose.position.x = 3.0
        far.pose.position.y = 4.0
        node._on_target(far)                         # geçerli açı üretir

        near = PoseStamped()
        near.pose.position.x = 0.1                   # < 0.5 m → açı güncellenmez
        near.pose.position.y = 0.0
        for _ in range(30):                          # 3 s boyunca akış sürüyor
            t[0] += 0.1
            node._on_target(near)
        node._on_write()
        assert _last_row(node)[8] != "", (
            "F-V.7 tutması bozuldu: kaynak canlıyken açı boşa düştü"
        )
    finally:
        node.destroy_node()


# ---------------------------------------------------------------------------
# 16.07 kayıt görevleri (jetson_kayit_gorevi.md): GÖREV 1 ARM'da CSV rotasyonu,
# GÖREV 2(a/b) fc modunda rc/out sağlık uyarısı, GÖREV 3 saat sıçraması.
# Kenar kaynağı FC'nin GERÇEK arm'ı (/mavros/state.armed) — FSM durumu değil.

def _mav_state(armed: bool, connected: bool = True):     # noqa: ANN201
    State = pytest.importorskip(
        "mavros_msgs.msg", reason="mavros_msgs yok — mavros kurulumuyla koş"
    ).State
    m = State()
    m.armed = armed
    m.connected = connected
    return m


def _fc_telemetry_node(tmp_path, warn_after: float = 0.0):  # noqa: ANN001, ANN201
    return girdap.TelemetryNode(
        parameter_overrides=[
            Parameter("kayit_dir", Parameter.Type.STRING,
                      str(tmp_path / "kayit")),
            Parameter("setpoint_source", Parameter.Type.STRING, "fc"),
            Parameter("rc_warn_after_s", Parameter.Type.DOUBLE, warn_after),
        ]
    )


def test_gorev1_arm_yukselen_kenari_kayit_rotasyonu(ros_context, tmp_path) -> None:
    """GÖREV 1 (rev. Eyüp 16.07): her kayıt kendi numaralı klasöründe —
    kayit/<N>/telemetri.csv + grafik.csv. Boot kayit/1'i açar; FC arm'ı
    False→True olunca kayit/2 açılır; disarm ve tekrarlanan armed=True
    DEĞİŞTİRMEZ; ikinci kenar kayit/3."""
    node = _telemetry_node(tmp_path, timeout=0.0)
    try:
        d0 = node._csv.path.parent
        assert d0.name == "1"                            # boot kaydı
        assert node._csv.path.name == "telemetri.csv"
        assert node._graph_csv.path == d0 / "grafik.csv"

        node._on_fc_state(_mav_state(armed=False))
        assert node._csv.path.parent == d0               # kenar yok

        node._on_fc_state(_mav_state(armed=True))        # yükselen kenar
        d1 = node._csv.path.parent
        assert d1.name == "2" and node._graph_csv.path.parent == d1
        assert (d0 / "telemetri.csv").exists()           # eski kayıt durur

        node._on_fc_state(_mav_state(armed=True))        # True→True: değişmez
        assert node._csv.path.parent == d1
        node._on_fc_state(_mav_state(armed=False))       # disarm: değişmez
        assert node._csv.path.parent == d1

        node._on_fc_state(_mav_state(armed=True))        # 2. kenar
        assert node._csv.path.parent.name == "3"

        # rotasyon sonrası yazma YENİ dosyaya gider, header sözleşmesi korunur
        node._on_write()
        with open(node._csv.path, newline="", encoding="utf-8") as f:
            rows = list(csv.reader(f))
        assert rows[0] == CSV_HEADER and len(rows) == 2

        # run_ekran2'nin "en yeni" seçimi kayıt kökünden rotasyonluyu bulmalı
        from prototype.viz.ekran2 import find_latest_graph_csv
        assert find_latest_graph_csv(tmp_path / "kayit") == node._graph_csv.path
    finally:
        node.destroy_node()


def test_gorev1_silinen_numara_yeniden_kullanilir(ros_context, tmp_path) -> None:
    """Eyüp kuralı (16.07): 'log 2 silinirse gene log 2'den devam etsin' —
    numara sayaç değil, dizindeki EN KÜÇÜK boş numara."""
    import shutil

    node = _telemetry_node(tmp_path, timeout=0.0)
    try:
        node._on_fc_state(_mav_state(armed=False))
        node._on_fc_state(_mav_state(armed=True))        # kayit/2 aktif
        assert node._csv.path.parent.name == "2"
        shutil.rmtree(tmp_path / "kayit" / "1")          # eski kayıt silindi
        node._on_fc_state(_mav_state(armed=False))
        node._on_fc_state(_mav_state(armed=True))        # en küçük boş = 1
        assert node._csv.path.parent.name == "1"
    finally:
        node.destroy_node()


def test_gorev1_retention_en_eski_kayitlari_siler(ros_context, tmp_path) -> None:
    """'Eski logları da silsin' (Eyüp 16.07): yeni kayıt açılırken
    kayit_sakla_adet'ten fazlası EN ESKİDEN silinir; aktif kayıt korunur."""
    node = girdap.TelemetryNode(
        parameter_overrides=[
            Parameter("kayit_dir", Parameter.Type.STRING,
                      str(tmp_path / "kayit")),
            Parameter("kayit_sakla_adet", Parameter.Type.INTEGER, 2),
        ]
    )
    try:
        node._on_fc_state(_mav_state(armed=False))
        node._on_fc_state(_mav_state(armed=True))        # kayit/2
        node._on_fc_state(_mav_state(armed=False))
        node._on_fc_state(_mav_state(armed=True))        # kayit/3 → 1 silinir
        root = tmp_path / "kayit"
        assert not (root / "1").exists()
        assert (root / "2").exists() and (root / "3").exists()
        assert node._csv.path.parent.name == "3"
    finally:
        node.destroy_node()


def test_gorev1_ilk_ornek_armed_true_kenar_sayilmaz(ros_context, tmp_path) -> None:
    """Servis FC zaten arm'lıyken açılırsa (ilk örnek True) rotasyon YAPILMAZ —
    kenar tanımı 'GÖZLENEN False→True'dur (görev ortası dosya bölünmesin)."""
    node = _telemetry_node(tmp_path, timeout=0.0)
    try:
        p0 = node._csv.path
        node._on_fc_state(_mav_state(armed=True))
        assert node._csv.path == p0
    finally:
        node.destroy_node()


def test_gorev2a_rc_out_sessizlik_uyarisi(ros_context, tmp_path) -> None:
    """GÖREV 2(a): fc modunda FC bağlandıktan warn_after sonra HİÇ rc/out
    yoksa BİR KEZ uyarı üretilir; girdap modunda hiç üretilmez."""
    node = _fc_telemetry_node(tmp_path)
    try:
        t = [100.0]
        node._now = lambda: t[0]
        assert node._rc_health_message() is None         # bağlantı damgası yok
        node._on_fc_state(_mav_state(armed=False))       # connected=True
        t[0] += 11.0
        msg = node._rc_health_message()
        assert msg is not None and "rc/out" in msg
        assert node._rc_health_message() is None         # bir kez
    finally:
        node.destroy_node()


def test_gorev2a_girdap_modunda_uyari_yok(ros_context, tmp_path) -> None:
    node = _telemetry_node(tmp_path, timeout=0.0)
    try:
        node._on_fc_state(_mav_state(armed=False))
        assert node._rc_health_message() is None
    finally:
        node.destroy_node()


def test_gorev2b_kanallar_arm_boyunca_sifir_uyarisi(ros_context, tmp_path) -> None:
    """GÖREV 2(b) — 16.07 masa vakası: rc/out AKIYOR (8 Hz) ama seçili kanal
    1/3 ARM'dan beri PWM=0 → arm oturumu başına BİR uyarı. Kanal dolarsa yok."""
    RCOut = pytest.importorskip(
        "mavros_msgs.msg", reason="mavros_msgs yok"
    ).RCOut
    node = _fc_telemetry_node(tmp_path)
    try:
        t = [100.0]
        node._now = lambda: t[0]
        node._on_fc_state(_mav_state(armed=False))
        rc = RCOut()
        rc.channels = [0, 1500, 0, 0]                    # bugünkü gerçek tablo
        node._on_rc_out(rc)
        node._on_fc_state(_mav_state(armed=True))        # arm kenarı
        node._on_rc_out(rc)
        t[0] += 11.0
        msg = node._rc_health_message()
        assert msg is not None and "kanal" in msg
        assert node._rc_health_message() is None         # arm oturumunda bir kez

        # yeni arm oturumunda kanal DOLUYSA uyarı üretilmez
        node._on_fc_state(_mav_state(armed=False))
        node._on_fc_state(_mav_state(armed=True))
        rc2 = RCOut()
        rc2.channels = [1600, 1500, 1400, 0]
        node._on_rc_out(rc2)
        t[0] += 11.0
        assert node._rc_health_message() is None
    finally:
        node.destroy_node()


def test_gorev3_saat_sicramasi_tespiti(ros_context, tmp_path) -> None:
    """GÖREV 3: duvar↔monotonic farkı bir tick'te >30 sn değişirse sıçrama
    döner (BULGU: 16.07 boot'unda NTP +3s16dk sıçrattı, CSV ortasında)."""
    node = _telemetry_node(tmp_path, timeout=0.0)
    try:
        assert node._check_clock_jump(1000.0, 500.0) is None   # ilk referans
        assert node._check_clock_jump(1001.0, 501.0) is None   # normal akış
        jump = node._check_clock_jump(1201.0, 502.0)           # duvar+200/mono+1
        assert jump is not None and jump > 30.0
        assert node._check_clock_jump(1202.0, 503.0) is None   # sonrası normal
    finally:
        node.destroy_node()
