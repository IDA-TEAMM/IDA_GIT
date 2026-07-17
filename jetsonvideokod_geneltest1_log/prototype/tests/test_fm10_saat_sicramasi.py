"""
Girdap İDA — F-M.10 duvar saati sıçraması testleri (hata_defteri 2026-07-16).

Kuru test kanıtı (journal 21:11:06): boot'ta saat ~2s10dk geriydi; NTP saati
+7774 sn ileri düzeltince köprü sahte "heartbeat kaybı (7775.0s)" KILL'i bastı,
planning MPPI'yi durdurdu, FSM motorları kesti — FC CANLIYDI, gerçek kayıp
YOKTU. Kök neden: tazelik/yaş hesapları duvar saatine (get_clock = ROS system
time) dayalıydı; NTP sıçraması son mesajın "yaşı" gibi görünür.

Düzeltme: göreli yaş/tazelik hesapları `time.monotonic()`'e alınır. Mesaj ve
CSV damgaları duvar saatinde KALIR (Dosya-2 md 4.2 duvar saati ister; F-T.6
sıçramada yalnız uyarır, damga düzeltmez).

Testler duvar saatini node'un `get_clock()`'unu sararak sıçratır (NTP taklidi);
`time.monotonic()` gerçek kalır. Gerçek-sessizlik regresyon bekçileri sahte
`_now` enjeksiyonuyla korunur (mevcut F-P.1/BULGU-2 test deseni).

rclpy gerektirir → .venv'de SKIP. İzole domain'de koş (ROS_DOMAIN_ID=77).
"""

from __future__ import annotations

import textwrap

import pytest

rclpy = pytest.importorskip("rclpy", reason="rclpy yok (.venv) — ROS ortamında koş")
mavros_msgs = pytest.importorskip(
    "mavros_msgs", reason="mavros_msgs yok — girdap_deps_ws source'la"
)

from geometry_msgs.msg import PoseStamped                # noqa: E402
from mavros_msgs.msg import State as MavState            # noqa: E402
from nav_msgs.msg import Odometry                        # noqa: E402
from rclpy.parameter import Parameter                    # noqa: E402
from rclpy.time import Time                              # noqa: E402
from sensor_msgs.msg import NavSatFix                    # noqa: E402
from std_msgs.msg import String                          # noqa: E402

bridge_mod = pytest.importorskip(
    "girdap_decision.mavros_bridge_node",
    reason="girdap_decision source'lanmamış (ros2_ws/install/setup.bash)",
)
planning_mod = pytest.importorskip("girdap_decision.planning_node")
fusion_mod = pytest.importorskip("girdap_decision.fusion_node")
telemetry_mod = pytest.importorskip("girdap_decision.telemetry_node")
mission_mod = pytest.importorskip("girdap_decision.mission_manager_node")

from prototype.mission.mission_manager import MissionPhase  # noqa: E402

_SICRAMA_S = 7774.0                                      # 16.07 vakasının birebir değeri


@pytest.fixture(scope="module")
def ros_context():                                       # noqa: ANN201
    rclpy.init()
    yield
    rclpy.shutdown()


def _duvar_saatini_sicrat(node, offset_s: float) -> None:  # noqa: ANN001
    """Node'un ROS duvar saatini offset kadar ileri göster (NTP düzeltme taklidi).

    Gerçek Time nesnesi döner → `.nanoseconds` ve `.to_msg()` çalışmaya devam
    eder (damga üreten yollar kırılmaz). `time.monotonic()` etkilenmez —
    gerçek NTP sıçramasında da monotonic saat sıçramaz.
    """
    gercek_saat = node.get_clock()

    class _SicramisSaat:
        def now(self):                               # noqa: ANN202
            t = gercek_saat.now()
            return Time(
                nanoseconds=t.nanoseconds + int(offset_s * 1e9),
                clock_type=t.clock_type,
            )

    node.get_clock = lambda: _SicramisSaat()         # type: ignore[method-assign]


def _mav_state(*, armed: bool = True) -> MavState:
    msg = MavState()
    msg.connected = True
    msg.armed = armed
    msg.guided = False
    msg.mode = "MANUAL"
    return msg


# ----- köprü: sahte heartbeat-kaybı KILL'i -----


def test_fm10_saat_sicramasi_sahte_heartbeat_kill_uretmez(ros_context) -> None:  # noqa: ANN001
    """FC canlı + duvar saati +7774 sn sıçradı → KILL BASILMAMALI (16.07 vakası)."""
    n = bridge_mod.MavrosBridgeNode()
    try:
        n._on_state(_mav_state())
        n._on_monitor()
        assert n._killed is False

        _duvar_saatini_sicrat(n, _SICRAMA_S)
        n._on_monitor()
        assert n._killed is False, (
            "NTP sıçraması sahte heartbeat-kaybı KILL'i bastı (F-M.10)"
        )
    finally:
        n.destroy_node()


def test_fm10_gercek_heartbeat_kaybi_hala_kill(ros_context) -> None:  # noqa: ANN001
    """Regresyon bekçisi: GERÇEK sessizlik (monotonic akışında) hâlâ KILL üretir."""
    n = bridge_mod.MavrosBridgeNode()
    try:
        t = [100.0]
        n._now = lambda: t[0]                        # sahte saat enjeksiyonu
        n._on_state(_mav_state())
        n._on_monitor()
        assert n._killed is False

        t[0] = 106.0                                 # 6 s sessizlik > 5 s eşik
        n._on_monitor()
        assert n._killed is True, "gerçek heartbeat kaybı KILL üretmedi"
    finally:
        n.destroy_node()


# ----- planning: sahte bayat-poz (MPPI durdurma) -----


def test_fm10_saat_sicramasi_pozu_bayat_gostermez(ros_context) -> None:  # noqa: ANN001
    """Poz akıyor + duvar saati sıçradı → odom bayat SAYILMAMALI (MPPI durmaz)."""
    node = planning_mod.PlanningNode(
        parameter_overrides=[
            Parameter("odom_timeout_s", Parameter.Type.DOUBLE, 1.0)
        ]
    )
    try:
        odom = Odometry()
        odom.pose.pose.orientation.w = 1.0
        node._on_odom(odom)
        assert node._odom_stale() is False

        _duvar_saatini_sicrat(node, _SICRAMA_S)
        assert node._odom_stale() is False, (
            "NTP sıçraması pozu bayat gösterdi → MPPI sahte durdu (F-M.10)"
        )
    finally:
        node.destroy_node()


# ----- fusion: F8.2 bekçisinin sahte kesintisi -----


def test_fm10_saat_sicramasi_fusion_odom_yayinini_kesmez(ros_context) -> None:  # noqa: ANN001
    """Girdi canlı + duvar saati sıçradı → F8.2 bekçisi yayını KESMEMELİ."""
    node = fusion_mod.FusionNode(
        parameter_overrides=[
            Parameter("use_isam2", Parameter.Type.BOOL, False),
            Parameter("pose_timeout_s", Parameter.Type.DOUBLE, 0.3),
        ]
    )
    yayinlar: list = []

    class _Stub:
        def publish(self, msg) -> None:              # noqa: ANN001
            yayinlar.append(msg)

    try:
        node._pub_pose = _Stub()                     # spin'siz yakalama
        node._pub_odom = _Stub()
        pose = PoseStamped()
        pose.pose.orientation.w = 1.0
        node._on_ekf_pose(pose)

        _duvar_saatini_sicrat(node, _SICRAMA_S)
        node._on_publish_timer()
        assert yayinlar, (
            "NTP sıçraması poz kaynağını bayat gösterdi → odom yayını "
            "sahte kesildi (F-M.10)"
        )
    finally:
        node.destroy_node()


# ----- telemetry: tazelik bekçisinin sahte boş hücreleri -----


def test_fm10_saat_sicramasi_taze_veriyi_bos_gostermez(ros_context, tmp_path) -> None:  # noqa: ANN001
    """Kaynak canlı + duvar saati sıçradı → _fresh None DÖNMEMELİ (CSV boşalmaz)."""
    node = telemetry_mod.TelemetryNode(
        parameter_overrides=[
            Parameter("kayit_dir", Parameter.Type.STRING, str(tmp_path / "kayit")),
        ]
    )
    try:
        t0 = node._now()                             # kaynak az önce damgalandı
        _duvar_saatini_sicrat(node, _SICRAMA_S)
        assert node._fresh(5.0, t0) == 5.0, (
            "NTP sıçraması taze kaynağı donuk gösterdi → CSV sütunları "
            "sahte boşaldı (F-M.10; F-T.1 dürüstlük bekçisinin ters yüzü)"
        )
    finally:
        node.destroy_node()


# ----- mission: dwell'in sahte tamamlanması -----


def test_fm10_saat_sicramasi_dwelli_sahte_tamamlamaz(ros_context, tmp_path) -> None:  # noqa: ANN001
    """DWELL'de duvar saati sıçradı → 30 s dwell anında 'doldu' SAYILMAMALI."""
    mission_file = tmp_path / "m.yaml"
    mission_file.write_text(
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
    node = mission_mod.MissionManagerNode(
        parameter_overrides=[
            Parameter("mission_file", Parameter.Type.STRING, str(mission_file)),
        ]
    )
    try:
        fix = NavSatFix()
        fix.status.status = 0
        fix.latitude = 41.0
        fix.longitude = 29.0
        node._on_gps(fix)
        node._on_state(String(data="PARKUR1"))       # görev başlar
        node._on_tick()                              # aynı nokta → anında varış
        assert node._mgr.phase is MissionPhase.DWELL
        assert node._mgr.current_index == 0

        _duvar_saatini_sicrat(node, _SICRAMA_S)
        node._on_tick()
        assert node._mgr.current_index == 0, (
            "NTP sıçraması 30 s dwell'i anında tamamladı → waypoint sahte "
            "ilerledi (F-M.10)"
        )
        assert node._mgr.phase is MissionPhase.DWELL
    finally:
        node.destroy_node()
