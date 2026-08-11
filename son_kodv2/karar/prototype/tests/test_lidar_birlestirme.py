"""
Girdap İDA — F-L.3 LiDAR paket birleştirme testleri (11.08.2026).

GERÇEK ARIZA (Jetson'da canlı ölçüldü, bu testlerin dondurduğu şey):
livox_ros_driver2 bu sürümde `publish_freq`'i mesaj birleştirmede KULLANMIYOR
(`Lddc::publish_period_ns_` hesaplanıp hiçbir yerde okunmuyor — ölü kod;
`PublishPointcloud2` kuyruktaki HER paketi tek tek yayınlıyor). Ölçüm:
/livox/lidar ~475 Hz, her mesaj width=96 — publish_freq=10.0 olmasına rağmen.

Sonuç: `perception_lidar_node` kümelemeyi mesaj başına koşturduğu için 30 cm'lik
bir duba tek pakette `min_cluster_size` eşiğini toplayamaz; obstacle_map "akar"
ama içi boştur. Aşağıdaki `test_birlestirme_KAPALI_parcalanmis_duba_KAYBOLUYOR`
bu arızayı doğrudan üretir — birleştirme geri alınırsa o test yeşile döner ve
`..._ACIK_...` kırmızıya düşer.

rclpy gerektirir; .venv'de otomatik SKIP.
"""

from __future__ import annotations

import time

import numpy as np
import pytest

rclpy = pytest.importorskip("rclpy", reason="rclpy yok (.venv) — ROS ortamında koş")
try:
    import scipy.spatial  # noqa: F401
except Exception as exc:  # ImportError VEYA numpy/scipy ABI ValueError
    pytest.skip(f"scipy kullanılamıyor: {exc}", allow_module_level=True)

from rclpy.parameter import Parameter                    # noqa: E402
from sensor_msgs_py import point_cloud2                  # noqa: E402
from std_msgs.msg import Header                          # noqa: E402

girdap = pytest.importorskip(
    "girdap_decision.perception_lidar_node",
    reason="girdap_decision.perception_lidar_node import edilemedi",
)


@pytest.fixture(scope="module")
def ros_context():                                       # noqa: ANN201
    rclpy.init()
    yield
    rclpy.shutdown()


class _SahtePub:
    """`node._pub` yerine geçer — yayınlananları toplar (ROS trafiği yok)."""

    def __init__(self) -> None:
        self.mesajlar: list = []

    def publish(self, msg) -> None:                      # noqa: ANN001
        self.mesajlar.append(msg)


def _dugum(birlestirme_s: float):                        # noqa: ANN202
    """Kümeleme değişkenlerini SABİTLEYEN düğüm.

    voxel_size=0 (altörnekleme testin sayısını bulandırmasın),
    mount_z=0.41 (B0 ölçülü değeri — sıfır bırakılırsa düğüm haklı olarak
    ERROR basar), min_cluster_size=5.
    """
    return girdap.PerceptionLidarNode(
        parameter_overrides=[
            Parameter("birlestirme_s", value=float(birlestirme_s)),
            Parameter("voxel_size", value=0.0),
            Parameter("min_cluster_size", value=5),
            Parameter("mount_z", value=0.41),
            Parameter("z_min", value=0.1),
        ]
    )


def _duba_noktalari(n: int = 30) -> np.ndarray:
    """5 m ileride, ~0,15 m çapında tek duba — base_link'te z≈0,5 m."""
    rng = np.random.default_rng(1234)
    merkez = np.array([5.0, 0.0, 0.1])
    return merkez + rng.uniform(-0.075, 0.075, size=(n, 3))


def _bulut(points: np.ndarray):                          # noqa: ANN202
    header = Header()
    header.frame_id = "livox_frame"
    header.stamp.sec = 0
    header.stamp.nanosec = 123
    return point_cloud2.create_cloud_xyz32(
        header, points.astype(np.float32).tolist()
    )


def _parcala(points: np.ndarray, boyut: int = 3) -> list[np.ndarray]:
    """Sürücünün yaptığını taklit et: bulutu küçük paketlere böl."""
    return [points[i:i + boyut] for i in range(0, len(points), boyut)]


# --------------------------------------------------------------- testler

def test_birlestirme_KAPALI_parcalanmis_duba_KAYBOLUYOR(ros_context) -> None:
    """ARIZANIN KENDİSİ: 3'erlik paketler min_cluster_size=5'i geçemez."""
    n = _dugum(birlestirme_s=0.0)
    n._pub = _SahtePub()
    try:
        for parca in _parcala(_duba_noktalari(), boyut=3):
            n._on_cloud(_bulut(parca))
        # Her paket ayrı kümelendi → her seferinde yayın var ama HEPSİ BOŞ.
        assert len(n._pub.mesajlar) == 10, "kapalıyken her mesaj ayrı yayınlanmalı"
        toplam_engel = sum(len(m.poses) for m in n._pub.mesajlar)
        assert toplam_engel == 0, (
            "3 noktalık paketler tek başına duba üretmemeli — bu, gerçek "
            "sürücüde yaşanan sessiz arızanın ta kendisi"
        )
    finally:
        n.destroy_node()


def test_birlestirme_ACIK_parcalanmis_duba_BULUNUYOR(ros_context) -> None:
    """DÜZELTME: aynı paketler pencerede birleşince duba tek küme olur."""
    n = _dugum(birlestirme_s=0.05)
    n._pub = _SahtePub()
    try:
        parcalar = _parcala(_duba_noktalari(), boyut=3)
        for parca in parcalar[:-1]:
            n._on_cloud(_bulut(parca))
        # Pencere dolmadan HİÇ yayın olmamalı.
        assert n._pub.mesajlar == [], "pencere dolmadan kümeleme yapılmamalı"
        time.sleep(0.06)                      # pencereyi geçir
        n._on_cloud(_bulut(parcalar[-1]))
        assert len(n._pub.mesajlar) == 1, "pencere dolunca TEK yayın olmalı"
        assert len(n._pub.mesajlar[0].poses) == 1, (
            "birleştirilmiş 30 nokta tek duba kümesi vermeli"
        )
        poz = n._pub.mesajlar[0].poses[0]
        assert poz.position.x == pytest.approx(5.0, abs=0.3)
        assert poz.position.y == pytest.approx(0.0, abs=0.3)
    finally:
        n.destroy_node()


def test_birlestirme_tamponu_yayindan_sonra_bosalir(ros_context) -> None:
    """Pencere kapanınca tampon sıfırlanır — noktalar iki kez kümelenmez."""
    n = _dugum(birlestirme_s=0.05)
    n._pub = _SahtePub()
    try:
        n._on_cloud(_bulut(_duba_noktalari()))
        time.sleep(0.06)
        n._on_cloud(_bulut(_duba_noktalari()))
        assert n._biriken == [], "yayından sonra tampon boşalmalı"
        assert n._biriktirme_t0 is None, "pencere saati sıfırlanmalı"
    finally:
        n.destroy_node()


def test_abonelik_derinligi_pencereye_gore_buyur(ros_context) -> None:
    """depth=1 birleştirmeyi ANLAMSIZ kılar: paketler kuyruktan düşer."""
    kapali = _dugum(birlestirme_s=0.0)
    acik = _dugum(birlestirme_s=0.1)
    try:
        d_kapali = kapali._sub.qos_profile.depth
        d_acik = acik._sub.qos_profile.depth
        assert d_kapali == 1, "kapalıyken F7.3'ün depth=1 davranışı korunmalı"
        assert d_acik >= 50, (
            f"birleştirme açıkken kuyruk pencereyi taşımalı (depth={d_acik})"
        )
    finally:
        kapali.destroy_node()
        acik.destroy_node()


def test_varsayilan_birlestirme_KAPALI(ros_context) -> None:
    """Nöbetçi: varsayılan AÇILIRSA kümeleme bütçesi aşılır.

    Bu test 11.08.2026 öğleden sonra TERSİNE çevrildi. Önceki hâli
    "varsayılan AÇIK olmalı" diyordu; dayanağı /livox/lidar'ın 475 Hz'te
    96 noktalık paketler yayınladığı ölçümüydü. O ölçüm sürücünün ARIZALI
    bir örneğinden alınmış: `girdap-livox` açılışta "bind failed" ile ölmüştü.
    Servis düzgün başlatılınca gerçek donanımda ölçülen 10 Hz × ~20 000 nokta.
    Birleştirme bu doğru kareler üstünde 2-3 kareyi bindirip kümelemeyi
    172-300 ms'e (bütçe 100 ms) çıkardı, obstacle_map 9,3 → 2,2 Hz düştü.
    Mekanizmanın kendisi silinmedi; 96-nokta hali geri gelirse >0 vermek yeter.
    """
    n = girdap.PerceptionLidarNode()
    try:
        assert n._birlestirme_s == pytest.approx(0.0), (
            "varsayılan KAPALI olmalı — sürücü zaten 10 Hz × 20 000 noktalık "
            "tam kare veriyor; birleştirme açıkken kümeleme bütçesi aşılıyor "
            "ve F7.3'ün depth=1 bayat-tarama koruması da kalkıyor (F-L.3)"
        )
    finally:
        n.destroy_node()
