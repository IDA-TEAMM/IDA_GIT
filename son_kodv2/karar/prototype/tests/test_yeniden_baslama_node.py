"""Soft-restart ROS davranışı — madde #11, md 5.5.3.1.

Çekirdek anlamlar `test_yeniden_baslama.py`'de. Burada ROS'a özgü olanlar:
  1. tek servis → fan-out yayını (operatör BEŞ çağrı yapmak zorunda değil;
     md 5.5.3.1 "süre durmaz" dediği için bu bir hız gereksinimi),
  2. sayaç mantığı: aynı değer iki kez gelirse iş İKİ KEZ koşmaz,
  3. QoS TRANSIENT_LOCAL: geç doğan/geç bağlanan node sıfırlamayı KAÇIRMAZ,
  4. bir abonenin patlaması diğerlerini etkilemez.

rclpy gerektirir → yoksa dürüst SKIP.
"""

from __future__ import annotations

import pytest

rclpy = pytest.importorskip("rclpy", reason="rclpy yok — ROS ortamında koş")

from rclpy.node import Node                              # noqa: E402
from std_msgs.msg import Int32                           # noqa: E402

yb = pytest.importorskip(
    "girdap_decision.yeniden_baslama",
    reason="girdap_decision import edilemedi (ros2_ws source'lanmamış)",
)


@pytest.fixture(scope="module")
def ros_context():                                       # noqa: ANN201
    rclpy.init()
    yield
    rclpy.shutdown()


def _spin(node, saniye: float = 0.6) -> None:            # noqa: ANN001
    """TRANSIENT_LOCAL teslimini beklemek için kısa spin."""
    son = node.get_clock().now().nanoseconds + int(saniye * 1e9)
    while node.get_clock().now().nanoseconds < son:
        rclpy.spin_once(node, timeout_sec=0.05)


def test_qos_TRANSIENT_LOCAL_ve_RELIABLE() -> None:
    """Tek atışlık olay kaybolmamalı — yayıncı ve abone AYNI profili kullanır."""
    from rclpy.qos import DurabilityPolicy, ReliabilityPolicy

    q = yb.reset_qos()
    assert q.durability == DurabilityPolicy.TRANSIENT_LOCAL
    assert q.reliability == ReliabilityPolicy.RELIABLE
    assert q.depth == 1


def test_sayac_artiyor(ros_context) -> None:              # noqa: ANN001
    n = Node("yb_yayinci_test")
    try:
        y = yb.ResetYayinci(n)
        assert y.sayac == 0
        assert y.yayinla() == 1
        assert y.yayinla() == 2
        assert y.sayac == 2
    finally:
        n.destroy_node()


def test_abone_isi_kosturuyor(ros_context) -> None:       # noqa: ANN001
    yayinci = Node("yb_y2")
    abone = Node("yb_a2")
    try:
        y = yb.ResetYayinci(yayinci)
        cagrildi = []
        yb.ResetAbonesi(abone, lambda: cagrildi.append(1))
        y.yayinla()
        _spin(abone)
        assert cagrildi == [1], "sifirlama abonede kosmadi"
    finally:
        yayinci.destroy_node()
        abone.destroy_node()


def test_GEC_BAGLANAN_abone_sifirlamayi_KACIRMIYOR(ros_context) -> None:  # noqa: ANN001
    """🔴 TRANSIENT_LOCAL'in asıl sebebi.

    Bir node meşgulse ya da `Restart=on-failure` ile yeniden doğduysa,
    sıfırlama olayını yayında bulmalı. VOLATILE QoS'ta olay sessizce
    kaybolurdu ve bu ancak ikinci turun ortasında yanlış davranışla
    fark edilirdi.
    """
    yayinci = Node("yb_y3")
    try:
        y = yb.ResetYayinci(yayinci)
        y.yayinla()                     # ÖNCE yayınla
        abone = Node("yb_a3")           # SONRA aboneyi kur
        try:
            cagrildi = []
            yb.ResetAbonesi(abone, lambda: cagrildi.append(1))
            _spin(abone)
            assert cagrildi == [1], (
                "gec baglanan abone gecmis sifirlamayi almadi — QoS yanlis"
            )
        finally:
            abone.destroy_node()
    finally:
        yayinci.destroy_node()


def test_ayni_sayac_IKI_KEZ_kosmuyor(ros_context) -> None:  # noqa: ANN001
    """Tekrar yayın / geç teslim yüzünden iş iki kez koşmamalı.

    Bayrak yerine SAYAÇ kullanmanın sebebi bu: "bunu işledim mi?" sorusu
    bayrakla belirsiz kalır.
    """
    n = Node("yb_a4")
    try:
        cagrildi = []
        a = yb.ResetAbonesi(n, lambda: cagrildi.append(1))
        m = Int32()
        m.data = 5
        a._on_reset(m)
        a._on_reset(m)                  # aynı değer tekrar
        m2 = Int32()
        m2.data = 4                     # ESKİ değer (sıra dışı teslim)
        a._on_reset(m2)
        assert cagrildi == [1], f"is {len(cagrildi)} kez kostu"
        m3 = Int32()
        m3.data = 6                     # yeni gerçek sıfırlama
        a._on_reset(m3)
        assert cagrildi == [1, 1]
    finally:
        n.destroy_node()


def test_patlayan_abone_DIGERLERINI_etkilemiyor(ros_context) -> None:  # noqa: ANN001
    """Yarım sıfırlanmış yığın kötü, ama çöken node daha kötü.

    İstisna yakalanıp loglanır; sayaç yine ilerler (aynı bozuk sıfırlama
    sonsuz tekrarlanmasın).
    """
    n = Node("yb_a5")
    try:
        def patla() -> None:
            raise RuntimeError("bilerek")

        a = yb.ResetAbonesi(n, patla)
        m = Int32()
        m.data = 1
        a._on_reset(m)                  # çökmemeli
        assert a._son == 1
    finally:
        n.destroy_node()


def test_servis_ve_topic_adlari_sabit() -> None:
    """Operatör bu adları ezberliyor / runbook'a yazıyor — sessizce değişmesin."""
    assert yb.RESET_SERVICE == "/girdap/mission/reset"
    assert yb.RESET_TOPIC == "/girdap/mission/reset_seq"
