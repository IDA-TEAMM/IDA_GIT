"""
Girdap İDA — fusion_node bypass modu testi (F8.1) + GPS fix kalitesi kapısı.

Doğrular (video zinciri, use_isam2=false → gtsam GEREKMEZ):
    - /mavros/local_position/pose → /girdap/fusion/odom pose geçişi
    - F8.1: /mavros/local_position/velocity_body → odom.twist doldurulur
      (planning_node MPPI durum vektörüne u,v,r'yi buradan okur)
    - GPS fix kalitesi: NavSatFix.status.status → ölçüm sigma'sı;
      STATUS_NO_FIX'te add_gps HİÇ çağrılmaz

rclpy gerektirir → sistem python3.10 + ROS Humble; .venv'de SKIP.
"""

from __future__ import annotations

import time

import pytest

rclpy = pytest.importorskip("rclpy", reason="rclpy yok (.venv) — ROS ortamında koş")

from rclpy.parameter import Parameter                    # noqa: E402
from geometry_msgs.msg import PoseStamped, TwistStamped  # noqa: E402
from nav_msgs.msg import Odometry                        # noqa: E402
from sensor_msgs.msg import NavSatFix, NavSatStatus      # noqa: E402

girdap = pytest.importorskip(
    "girdap_decision.fusion_node",
    reason="girdap_decision source'lanmamış (ros2_ws/install/setup.bash)",
)


@pytest.fixture(scope="module")
def ros_context():                                       # noqa: ANN201
    rclpy.init()
    yield
    rclpy.shutdown()


def test_bypass_odom_carries_pose_and_twist(ros_context) -> None:
    """EKF poz + body hız yayınla → odom'da pose VE twist dolu olmalı (F8.1)."""
    node = girdap.FusionNode(
        parameter_overrides=[
            Parameter("use_isam2", Parameter.Type.BOOL, False),
        ]
    )
    helper = rclpy.create_node("test_fusion_helper")
    pose_pub = helper.create_publisher(
        PoseStamped, "/mavros/local_position/pose", 10
    )
    vel_pub = helper.create_publisher(
        TwistStamped, "/mavros/local_position/velocity_body", 10
    )
    odoms: list[Odometry] = []
    helper.create_subscription(
        Odometry, "/girdap/fusion/odom", odoms.append, 10
    )
    try:
        pose = PoseStamped()
        pose.pose.position.x = 3.0
        pose.pose.position.y = -1.5
        pose.pose.orientation.w = 1.0
        vel = TwistStamped()
        vel.twist.linear.x = 1.2                 # ileri sürat (body u)
        vel.twist.linear.y = -0.1                # yanal (body v)
        vel.twist.angular.z = 0.25               # yaw rate (r)

        deadline = time.monotonic() + 5.0
        good: Odometry | None = None
        while time.monotonic() < deadline and good is None:
            pose_pub.publish(pose)
            vel_pub.publish(vel)
            rclpy.spin_once(helper, timeout_sec=0.01)
            # 50 Hz timer spin_once'ı doyurabilir — birkaç kez spin et
            for _ in range(6):
                rclpy.spin_once(node, timeout_sec=0.01)
            good = next(
                (o for o in odoms if o.twist.twist.linear.x != 0.0), None
            )
        assert good is not None, "twist'i dolu odom mesajı gelmedi (F8.1)"
        assert good.pose.pose.position.x == pytest.approx(3.0)
        assert good.pose.pose.position.y == pytest.approx(-1.5)
        assert good.twist.twist.linear.x == pytest.approx(1.2)
        assert good.twist.twist.linear.y == pytest.approx(-0.1)
        assert good.twist.twist.angular.z == pytest.approx(0.25)
        assert good.child_frame_id == "base_link"  # twist body-frame sözleşmesi
    finally:
        helper.destroy_node()
        node.destroy_node()


def test_bypass_stale_pose_stops_publishing(ros_context) -> None:
    """F8.2: EKF poz akışı kesilince fusion odom yayını DURMALI (bayat pozla
    50 Hz yayına devam etmek downstream'i donmuş pozla plan yapmaya iter)."""
    node = girdap.FusionNode(
        parameter_overrides=[
            Parameter("use_isam2", Parameter.Type.BOOL, False),
            Parameter("pose_timeout_s", Parameter.Type.DOUBLE, 0.3),
        ]
    )
    helper = rclpy.create_node("test_fusion_stale_helper")
    pose_pub = helper.create_publisher(
        PoseStamped, "/mavros/local_position/pose", 10
    )
    odoms: list[Odometry] = []
    helper.create_subscription(
        Odometry, "/girdap/fusion/odom", odoms.append, 10
    )
    try:
        pose = PoseStamped()
        pose.pose.orientation.w = 1.0
        # Akış canlıyken odom gelmeli
        deadline = time.monotonic() + 5.0
        while time.monotonic() < deadline and not odoms:
            pose_pub.publish(pose)
            rclpy.spin_once(helper, timeout_sec=0.01)
            for _ in range(6):
                rclpy.spin_once(node, timeout_sec=0.01)
        assert odoms, "canlı akışta odom gelmedi"

        # Akışı KES; timeout'u (0.3 s) aşacak kadar spin et — yayın durmalı
        t_stop = time.monotonic()
        while time.monotonic() - t_stop < 0.8:
            rclpy.spin_once(node, timeout_sec=0.02)
            rclpy.spin_once(helper, timeout_sec=0.005)
        n_after_stale = len(odoms)
        while time.monotonic() - t_stop < 1.4:
            rclpy.spin_once(node, timeout_sec=0.02)
            rclpy.spin_once(helper, timeout_sec=0.005)
        assert len(odoms) == n_after_stale, (
            "bayat pozla odom yayını sürüyor (F8.2)"
        )
    finally:
        helper.destroy_node()
        node.destroy_node()


def test_fp7_bayat_velocity_body_twist_sifirlanir(ros_context) -> None:
    """F-P.7 (robustness taraması, 2026-07-15): velocity_body TEK BAŞINA
    kesilirse (pose akışı sürerken) pose_timeout_s bekçisi tetiklenmez —
    ayrı bir bekçi olmadan odom.twist SONSUZA DEK donuk son hızı yayınlar.
    Pose akışını KESMEDEN yalnız vel akışını durdurup twist'in sıfırlandığını
    (pose'un ise güncellenmeye devam ettiğini) doğrular."""
    node = girdap.FusionNode(
        parameter_overrides=[
            Parameter("use_isam2", Parameter.Type.BOOL, False),
            Parameter("vel_timeout_s", Parameter.Type.DOUBLE, 0.3),
        ]
    )
    helper = rclpy.create_node("test_fusion_vel_stale_helper")
    pose_pub = helper.create_publisher(
        PoseStamped, "/mavros/local_position/pose", 10
    )
    vel_pub = helper.create_publisher(
        TwistStamped, "/mavros/local_position/velocity_body", 10
    )
    odoms: list[Odometry] = []
    helper.create_subscription(
        Odometry, "/girdap/fusion/odom", odoms.append, 10
    )
    try:
        pose = PoseStamped()
        pose.pose.position.x = 7.0
        pose.pose.orientation.w = 1.0
        vel = TwistStamped()
        vel.twist.linear.x = 2.5

        deadline = time.monotonic() + 5.0
        good: Odometry | None = None
        while time.monotonic() < deadline and good is None:
            pose_pub.publish(pose)
            vel_pub.publish(vel)
            rclpy.spin_once(helper, timeout_sec=0.01)
            for _ in range(6):
                rclpy.spin_once(node, timeout_sec=0.01)
            good = next(
                (o for o in odoms if o.twist.twist.linear.x != 0.0), None
            )
        assert good is not None, "canlı akışta dolu twist gelmedi"

        # Vel akışını KES, pose akışı SÜRSÜN — 0.3 s eşiği aşılana dek spin.
        t_stop = time.monotonic()
        last_odom: Odometry | None = None
        while time.monotonic() - t_stop < 0.8:
            pose_pub.publish(pose)                # pose canlı kalıyor
            rclpy.spin_once(helper, timeout_sec=0.005)
            rclpy.spin_once(node, timeout_sec=0.02)
            if odoms:
                last_odom = odoms[-1]
        assert last_odom is not None, "pose canlıyken odom yayını durdu (beklenmedik)"
        assert last_odom.twist.twist.linear.x == pytest.approx(0.0), (
            "bayat velocity_body hâlâ donuk yazılıyor (F-P.7)"
        )
        assert last_odom.pose.pose.position.x == pytest.approx(7.0), (
            "pose akışı canlıyken yayın durmuş olmamalıydı"
        )
    finally:
        helper.destroy_node()
        node.destroy_node()


# --------------------------------------------------------------------------- #
# GPS fix kalitesi kapısı (_on_gps)
# --------------------------------------------------------------------------- #


class _KaydedenKaynak:
    """FusionPipeline yerine geçen kayıt tutucu (gtsam yüklenmesin)."""

    def __init__(self) -> None:
        self.cagrilar: list[tuple[float, float, float | None]] = []

    def on_gps(self, lat: float, lon: float, sigma_xy: float | None = None) -> None:
        self.cagrilar.append((lat, lon, sigma_xy))

    def current_pose(self) -> tuple[float, float, float]:
        raise RuntimeError("tahmin yok")            # yayım timer'ı sessiz kalsın


def _gps_node(ros_context):                          # noqa: ANN001, ANN202
    """Bypass modunda node + sahte kaynak: _on_gps'i doğrudan sınayabiliriz.

    use_isam2=false seçilir ki test gtsam GEREKTİRMESİN; _on_gps'in kalite
    kapısı moddan bağımsız aynı koddur.
    """
    node = girdap.FusionNode(
        parameter_overrides=[
            Parameter("use_isam2", Parameter.Type.BOOL, False),
        ]
    )
    kaynak = _KaydedenKaynak()
    node._source = kaynak
    return node, kaynak


def _navsatfix(status: int, lat: float = 36.85, lon: float = 28.27) -> NavSatFix:
    msg = NavSatFix()
    msg.status.status = status
    msg.status.service = NavSatStatus.SERVICE_GPS
    msg.latitude = lat
    msg.longitude = lon
    return msg


def test_status_no_fix_add_gps_cagrilmaz(ros_context) -> None:
    """STATUS_NO_FIX → ölçüm smoother'a HİÇ verilmemeli.

    Fix yokken NavSatFix'in lat/lon alanı tanımsızdır (sürücüye göre 0/0 ya
    da son geçerli değer). Prior olarak eklenirse grafiği kalıcı bozar —
    üstelik yarışma alanında (0,0) binlerce km uzakta bir noktadır.
    """
    node, kaynak = _gps_node(ros_context)
    try:
        node._on_gps(_navsatfix(NavSatStatus.STATUS_NO_FIX))
        assert kaynak.cagrilar == [], "NO_FIX ölçümü smoother'a gitti"
        assert node._n_gps == 0
        assert node._n_gps_rejected == 1
    finally:
        node.destroy_node()


def test_fix_kalitesi_sigmayi_secer(ros_context) -> None:
    """RTK / SBAS / tek nokta → farklı ölçüm sigma'ları (hardware.yaml)."""
    node, kaynak = _gps_node(ros_context)
    try:
        for status, beklenen in (
            (NavSatStatus.STATUS_GBAS_FIX, 0.05),     # RTK fixed
            (NavSatStatus.STATUS_SBAS_FIX, 0.50),
            (NavSatStatus.STATUS_FIX, 2.50),          # tek nokta
        ):
            node._on_gps(_navsatfix(status))
            assert kaynak.cagrilar[-1][2] == pytest.approx(beklenen), (
                f"status={status} için yanlış sigma"
            )
        assert node._n_gps == 3 and node._n_gps_rejected == 0
    finally:
        node.destroy_node()


def test_sigma_tablosu_parametreden_gelir(ros_context) -> None:
    """hardware.yaml fusion.gps_sigma_by_status → ROS parametreleri → tablo."""
    node = girdap.FusionNode(
        parameter_overrides=[
            Parameter("use_isam2", Parameter.Type.BOOL, False),
            Parameter("gps_sigma_gbas_fix", Parameter.Type.DOUBLE, 0.02),
            Parameter("gps_sigma_fix", Parameter.Type.DOUBLE, 7.5),
        ]
    )
    kaynak = _KaydedenKaynak()
    node._source = kaynak
    try:
        node._on_gps(_navsatfix(NavSatStatus.STATUS_GBAS_FIX))
        node._on_gps(_navsatfix(NavSatStatus.STATUS_FIX))
        assert kaynak.cagrilar[0][2] == pytest.approx(0.02)
        assert kaynak.cagrilar[1][2] == pytest.approx(7.5)
    finally:
        node.destroy_node()


def test_fix_geri_gelince_olcum_yeniden_kabul_edilir(ros_context) -> None:
    """NO_FIX kalıcı bir kapı DEĞİL — fix dönünce akış sürmeli."""
    node, kaynak = _gps_node(ros_context)
    try:
        node._on_gps(_navsatfix(NavSatStatus.STATUS_NO_FIX))
        node._on_gps(_navsatfix(NavSatStatus.STATUS_NO_FIX))
        assert kaynak.cagrilar == []
        node._on_gps(_navsatfix(NavSatStatus.STATUS_GBAS_FIX))
        assert len(kaynak.cagrilar) == 1
        assert node._n_gps_rejected == 2 and node._n_gps == 1
    finally:
        node.destroy_node()


# ------------------------------------------ KAR-05: girdi yokken YAYIN YOK


def test_KAR05_girdi_HIC_gelmeden_odom_YAYINLANMAZ(ros_context) -> None:  # noqa: ANN001
    """🔴 KAR-05 nöbetçisi — projenin en tehlikeli "sahte yeşil"i.

    Kaptanın bag analizi: `session_20260811_171943`'te `/girdap/fusion/odom`
    **16.974 mesajın %100'ü (0,0,0)**, kusursuz 10,001 Hz, NaN yok, kovaryans
    işaretlenmemiş, stamp geçerli. Aşağı akıştaki hiçbir düğüm bunun geçersiz
    olduğunu anlayamıyordu; operatör `ros2 topic hz` ile "sağlıklı" görüyordu.

    Kök neden: F8.2 bayatlık bekçisi `_last_input_t is not None` şartına bağlı
    olduğu için, girdi **hiç gelmediyse** bekçi hiç çalışmıyordu.

    Ayrıca bu, KAR-11'in besleyicisi: bozuk poz → dünya konumları oynar →
    kenar hafızası mükerrer kayıtla patlar → kontrol döngüsü 10→2,5 Hz.
    """
    node = girdap.FusionNode(
        parameter_overrides=[Parameter("use_isam2", Parameter.Type.BOOL, False)]
    )
    try:
        yayinlanan = []
        class _Spy:
            def publish(self, msg):                      # noqa: ANN001, ANN202
                yayinlanan.append(msg)
        node._pub_odom = _Spy()
        assert node._last_input_t is None, "test kurulumu: girdi olmamali"
        for _ in range(20):
            node._on_publish_timer()
        assert yayinlanan == [], (
            f"girdi HIC gelmeden {len(yayinlanan)} odom yayinlandi — "
            f"asagi akisa 'orijindeyim' diye yalan soyleniyor (KAR-05)"
        )
    finally:
        node.destroy_node()


# --------------------------------- KAR-06: sıfır kovaryans + yenilik kapısı


def _fix(lat: float, lon: float, status: int = 0, kov0: float = 4.0,
         kov_tipi: int = 2) -> NavSatFix:
    m = NavSatFix()
    m.status.status = status
    m.latitude, m.longitude = lat, lon
    m.position_covariance = [kov0, 0.0, 0.0, 0.0, kov0, 0.0, 0.0, 0.0, kov0]
    m.position_covariance_type = kov_tipi
    return m


def test_KAR06_SIFIR_kovaryansli_GPS_reddediliyor(ros_context) -> None:  # noqa: ANN001
    """🔴 Geçerli bir GPS alıcısı ASLA sıfır kovaryans bildirmez.

    Kaptanın analizinde canlı domaine sızan 24.430 sahte GPS mesajının hepsi
    sıfır kovaryanslıydı; füzyon bunu "sonsuz güven" okuyup pozu ışınlatıyordu
    (25 ms'de 6,54 m = 257 m/s, 60+ kez).
    """
    node, _ = _gps_node(ros_context)
    try:
        onceki = node._n_gps
        node._on_gps(_fix(36.85, 28.27, kov0=0.0))
        assert node._n_gps == onceki, "sifir kovaryansli GPS kabul edildi"
    finally:
        node.destroy_node()


def test_KAR06_bilinmeyen_kovaryans_tipi_REDDEDILMEZ(ros_context) -> None:  # noqa: ANN001
    """`COVARIANCE_TYPE_UNKNOWN` "bilmiyorum" demek, "sıfır" demek değil —
    onu elemek gerçek alıcıları da elerdi."""
    node, _ = _gps_node(ros_context)
    try:
        onceki = node._n_gps
        node._on_gps(_fix(36.85, 28.27, kov0=0.0, kov_tipi=0))
        assert node._n_gps == onceki + 1, "bilinmeyen kovaryans tipi elenmemeli"
    finally:
        node.destroy_node()


def test_KAR06_imkansiz_SICRAMA_reddediliyor(ros_context) -> None:  # noqa: ANN001
    """Su üstü aracı 25 ms'de 6,5 m gidemez. Ardışık ölçüm arası hız kapısı."""
    node, _ = _gps_node(ros_context)
    try:
        # Referansi ELLE 100 ms geriye koyuyoruz: gercek sistemde GPS 10 Hz'dir.
        # Iki cagriyi arka arkaya yapmak dt~0 uretir ve kapi (dogru olarak)
        # uygulanmaz — olculemeyecek araliktan hiz cikarilmaz.
        # ⚠ §0.61: referans, dugumun KENDI bayatlik saatinden tohumlanir.
        # `get_clock()` (duvar saati) ile tohumlamak dt'yi ~1,8 milyar saniye
        # yapar → kapi "uzun sessizlik" sanip hic uygulanmaz, test sahte yesil
        # verirdi (13.08 suit kosumunda birebir yakalandi).
        simdi = node._saat()
        node._son_gps = (36.85, 28.27, simdi - 0.1)
        n0 = node._n_gps
        # ~0.01 derece enlem ≈ 1,1 km / 100 ms = 11 km/s — imkansiz
        node._on_gps(_fix(36.86, 28.27))
        assert node._n_gps == n0, "imkansiz sicrama kabul edildi"
    finally:
        node.destroy_node()


def test_KAR06_makul_hareket_GECIYOR(ros_context) -> None:  # noqa: ANN001
    """Kapı gerçek hareketi engellememeli — yoksa tekne ilerleyemez."""
    node, _ = _gps_node(ros_context)
    try:
        simdi = node._saat()          # §0.61: dugumun kendi bayatlik saati
        node._son_gps = (36.85, 28.27, simdi - 0.1)
        n0 = node._n_gps
        # ~1e-6 derece ≈ 11 cm / 100 ms = 1,1 m/s — teknenin seyir hizi
        node._on_gps(_fix(36.850001, 28.27))
        assert node._n_gps == n0 + 1, "makul hareket reddedildi"
    finally:
        node.destroy_node()


# --------------------------------------------------------------------------
# F-F.1 (§0.98a) — POZUN MAKULLÜK KAPISI
#
# 14.08.2026 su koşumunda `/girdap/fusion/pose` iSAM2 diverjansıyla 10¹⁴⁹
# mertebesine çıktı ve düğüm bunu 10 Hz düzenlilikte yayınlamaya devam etti.
# Mevcut üç kapı (KAR-05 · F8.2 · F-P.7) tazelik ölçtüğü için hiçbiri görmedi.
# Aşağıdaki iki nöbetçi kapıyı ve KARŞITINI (yanlış pozitif) donduruyor.
# --------------------------------------------------------------------------


def _bypass_node_ve_yardimci():                          # noqa: ANN202
    """Bypass modunda düğüm + yayıncı/dinleyici (gtsam GEREKMEZ)."""
    node = girdap.FusionNode(
        parameter_overrides=[
            Parameter("use_isam2", Parameter.Type.BOOL, False),
        ]
    )
    helper = rclpy.create_node("test_fusion_ff1_helper")
    pose_pub = helper.create_publisher(
        PoseStamped, "/mavros/local_position/pose", 10
    )
    odoms: list[Odometry] = []
    helper.create_subscription(
        Odometry, "/girdap/fusion/odom", odoms.append, 10
    )
    return node, helper, pose_pub, odoms


def _pompala(node, helper, pose_pub, pose, sure_s: float) -> None:  # noqa: ANN001
    deadline = time.monotonic() + sure_s
    while time.monotonic() < deadline:
        pose_pub.publish(pose)
        rclpy.spin_once(helper, timeout_sec=0.01)
        for _ in range(6):
            rclpy.spin_once(node, timeout_sec=0.01)


def test_ff1_sacma_poz_YAYINLANMAZ(ros_context) -> None:  # noqa: ANN001
    """10¹⁴⁹'luk poz odom'a ÇIKMAMALI — KAR-05'in kuralı: yalan söyleme, sus.

    Ölçülen arıza birebir bu: değer taze ve düzenliydi, yalnız anlamsızdı;
    aşağı akış onu 'geçerli poz' sanıp geri komut üretti (§0.98a).
    """
    node, helper, pose_pub, odoms = _bypass_node_ve_yardimci()
    try:
        pose = PoseStamped()
        pose.pose.position.x = 1.63e149          # 14.08'de ÖLÇÜLEN büyüklük
        pose.pose.position.y = 7.05e148
        pose.pose.orientation.w = 1.0
        _pompala(node, helper, pose_pub, pose, 2.0)
        assert not odoms, (
            f"saçma poz yayınlandı ({len(odoms)} mesaj) — F-F.1 kapısı yok "
            "ya da menzil eşiği devre dışı"
        )
    finally:
        helper.destroy_node()
        node.destroy_node()


def test_ff1_nan_poz_YAYINLANMAZ(ros_context) -> None:   # noqa: ANN001
    """`nan` poz da elenmeli — menzil testi TEK BAŞINA yakalayamaz.

    `nan <= menzil` her zaman False döner, yani menzil testi nan'ı 'makul'
    saymaz; ama işaret şu ki `isfinite` olmadan nan bir ψ sessizce nan
    quaternion üretip aşağı akışa taşınırdı.
    """
    node, helper, pose_pub, odoms = _bypass_node_ve_yardimci()
    try:
        pose = PoseStamped()
        pose.pose.position.x = float("nan")
        pose.pose.position.y = float("inf")
        pose.pose.orientation.w = 1.0
        _pompala(node, helper, pose_pub, pose, 2.0)
        assert not odoms, "nan/inf poz yayınlandı — isfinite kapısı yok"
    finally:
        helper.destroy_node()
        node.destroy_node()


def test_ff1_makul_poz_ENGELLENMEZ(ros_context) -> None:  # noqa: ANN001
    """KARŞIT NÖBETÇİ: normal poz geçmeye devam etmeli.

    Kapının yanlış pozitif üretmediğini dondurur — yoksa 'arıza yok' diye
    bütün yığını susturan bir düzeltme yapmış olurduk.
    """
    node, helper, pose_pub, odoms = _bypass_node_ve_yardimci()
    try:
        pose = PoseStamped()
        pose.pose.position.x = 123.4             # makul saha ölçeği
        pose.pose.position.y = -56.7
        pose.pose.orientation.w = 1.0
        _pompala(node, helper, pose_pub, pose, 3.0)
        assert odoms, "makul poz yayınlanmadı — kapı fazla dar"
        assert abs(odoms[-1].pose.pose.position.x - 123.4) < 1e-6
    finally:
        helper.destroy_node()
        node.destroy_node()
