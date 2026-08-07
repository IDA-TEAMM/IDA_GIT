"""LiDAR veri seti kaydedici (md 4.2 "Diğer Otonomi Sensörleri") testleri.

Ayrıca Dosya-3 zincirinin 07.08 denetiminde bulunan iki hatasını dondurur:
frame etiketi (`odom`) ve kenar dubası katmanı.
"""

from __future__ import annotations

import math

import pytest

rclpy = pytest.importorskip("rclpy", reason="rclpy yok — ROS ortamında koş")

from rclpy.parameter import Parameter                  # noqa: E402
from geometry_msgs.msg import Pose, PoseArray          # noqa: E402
from nav_msgs.msg import Odometry                      # noqa: E402
from vision_msgs.msg import (                          # noqa: E402
    BoundingBox3D, Detection3D, Detection3DArray, ObjectHypothesisWithPose,
)

from girdap_decision import lidar_kayit_node as lk     # noqa: E402
from girdap_decision import planning_node as pn        # noqa: E402


@pytest.fixture()
def ros_context():
    rclpy.init()
    yield
    rclpy.shutdown()


def _odom(x: float, y: float, psi: float) -> Odometry:
    m = Odometry()
    m.pose.pose.position.x = x
    m.pose.pose.position.y = y
    m.pose.pose.orientation.z = math.sin(psi / 2.0)
    m.pose.pose.orientation.w = math.cos(psi / 2.0)
    return m


# ------------------------------------------------------------- dönüşüm

@pytest.mark.parametrize(
    "bx,by,ax,ay,psi",
    [
        (5.0, 0.0, 0.0, 0.0, 0.0),
        (5.0, 0.0, 0.0, 0.0, math.pi / 2),
        (3.0, -2.0, 10.0, -4.0, 0.7),
        (0.0, 7.5, -6.0, 2.0, -2.1),
    ],
)
def test_govde_to_dunya_planning_node_ILE_AYNI(ros_context, bx, by, ax, ay, psi):
    """🔴 İki kopya AYRIŞMAMALI — bu projenin iki kez yediği hata bu.

    `lidar_kayit_node.govde_to_dunya` ile `planning_node._body_to_world`
    aynı sonucu vermezse engeller kayıtta bir yerde, planlamada başka bir
    yerde görünür ve teslim edilen video gerçeği yansıtmaz.
    """
    node = pn.PlanningNode()
    try:
        node._last_xy = (ax, ay)
        node._last_psi = psi
        beklenen = node._body_to_world(bx, by)
    finally:
        node.destroy_node()
    bizim = lk.govde_to_dunya(bx, by, (ax, ay), psi)
    assert bizim == pytest.approx(beklenen, abs=1e-12)


# ------------------------------------------------------- kayıt davranışı

def test_1Hz_ALTI_acilista_REDDEDILIR(ros_context, tmp_path):
    """md 4.2 'En az 1 Hz' — sessizce ihlale düşmek yerine açılışta patla."""
    with pytest.raises(ValueError):
        lk.LidarKayitNode(parameter_overrides=[
            Parameter("output_dir", Parameter.Type.STRING, str(tmp_path)),
            Parameter("dump_rate_hz", Parameter.Type.DOUBLE, 0.5),
        ])


def test_siniflandirilmis_engeller_DUNYAYA_tasinir(ros_context, tmp_path):
    """Küme merkezleri gövdeden dünyaya taşınmalı; sınıf korunmalı."""
    node = lk.LidarKayitNode(parameter_overrides=[
        Parameter("output_dir", Parameter.Type.STRING, str(tmp_path)),
        Parameter("ham_bulut_enabled", Parameter.Type.BOOL, False),
    ])
    try:
        node._on_odom(_odom(10.0, -5.0, math.pi / 2))     # burun kuzeye
        msg = Detection3DArray()
        det = Detection3D()
        det.bbox = BoundingBox3D()
        det.bbox.center.position.x = 4.0                  # 4 m İLERİ
        det.bbox.center.position.y = 0.0
        det.bbox.size.x = 0.3
        h = ObjectHypothesisWithPose()
        h.hypothesis.class_id = "0"                       # turuncu KENAR
        det.results.append(h)
        msg.detections.append(det)
        node._on_classified(msg)

        assert len(node._kumeler) == 1
        k = node._kumeler[0]
        # ψ=90° iken 4 m ileri = 4 m KUZEY → (10, -5+4) = (10, -1)
        assert k.merkez[0] == pytest.approx(10.0, abs=1e-9)
        assert k.merkez[1] == pytest.approx(-1.0, abs=1e-9)
        assert k.sinif == 0
        assert k.kume_id == 0
        assert k.yaricap == pytest.approx(0.15)
    finally:
        node.destroy_node()


def test_sinifsiz_yedek_kaynak_MANDALLANMAZ(ros_context, tmp_path):
    """Kayıt node'unda tek yönlü mandal OLMAMALI.

    `planning_node._classified_seen` bilerek tek yönlüdür (güvenlik), ama
    KAYIT tarafında aynı mandal, sınıflı akış düşünce teslimi de öldürürdü.
    Sınıfsız küme yazmak, hiç kare yazmamaktan iyidir.
    """
    node = lk.LidarKayitNode(parameter_overrides=[
        Parameter("output_dir", Parameter.Type.STRING, str(tmp_path)),
        Parameter("ham_bulut_enabled", Parameter.Type.BOOL, False),
        Parameter("veri_timeout_s", Parameter.Type.DOUBLE, 1.0),
    ])
    try:
        t = [100.0]
        node._now = lambda: t[0]
        node._on_odom(_odom(0.0, 0.0, 0.0))
        node._siniflilar_geldi = True
        node._son_veri_t = t[0]

        pa = PoseArray()
        q = Pose()
        q.position.x = 6.0
        q.orientation.z = 0.2                             # yarıçap sözleşmesi
        pa.poses.append(q)

        node._on_obstacles(pa)                            # taze sınıflı varken
        assert node._kumeler == [], "taze sınıflı akış varken yedek ezmemeli"

        t[0] = 200.0                                      # sınıflı akış BAYAT
        node._on_obstacles(pa)
        assert len(node._kumeler) == 1, "sınıflı akış düşünce yedek devralmalı"
        assert node._kumeler[0].sinif is None
    finally:
        node.destroy_node()


def test_zaman_damgasi_VERININ_kendi_stampinden(ros_context, tmp_path):
    msg = Detection3DArray()
    msg.header.stamp.sec = 1786000000
    msg.header.stamp.nanosec = 500_000_000
    d = lk.LidarKayitNode._damga(msg.header)
    assert d.endswith("Z") and ".500" in d
    msg.header.stamp.sec = 0
    msg.header.stamp.nanosec = 0
    assert lk.LidarKayitNode._damga(msg.header) == "STAMP-YOK"


# --------------------------------------------- Dosya-3 zinciri (H7 / H1)

def test_yerel_harita_frame_ODOM_base_link_DEGIL(ros_context):
    """🔴 H7 nöbetçisi — harita DÜNYA eksenli, etiketi de öyle olmalı.

    `local_cost_grid()` hücreleri dünya ENU'da kurar ve ψ'yi HİÇ kullanmaz;
    `base_link` etiketi TF/RViz tüketicilerini ψ kadar yanıltıyordu.
    """
    node = pn.PlanningNode()
    try:
        yakalanan = {}
        node._pub_map.publish = lambda m: yakalanan.setdefault("m", m)
        node._publish_local_map()
        assert yakalanan["m"].header.frame_id == "odom", (
            "yerel harita yine 'base_link' diye etiketlenmiş — veri dünya "
            "eksenli olduğu için bu etiket engelleri ψ kadar kaydırır"
        )
    finally:
        node.destroy_node()


def test_kenar_dubalari_DUNYA_cercevesinde_yayinlanir(ros_context):
    """H1 — kapı dubaları Dosya-3 haritasına ayrı katman olarak girsin diye."""
    node = pn.PlanningNode()
    try:
        yakalanan = {}
        node._pub_edge_buoys.publish = lambda m: yakalanan.setdefault("m", m)
        node._publish_edge_buoys([(3.0, 4.0), (-1.0, 2.0)])
        m = yakalanan["m"]
        assert m.header.frame_id == "odom"
        assert len(m.poses) == 2
        assert m.poses[0].position.x == pytest.approx(3.0)
        assert m.poses[1].position.y == pytest.approx(2.0)
    finally:
        node.destroy_node()
