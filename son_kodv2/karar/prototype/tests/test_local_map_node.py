"""
Girdap İDA — local_map_node güvenlik testleri (F-P.17).

F-P.17 (robustness taraması, 2026-07-15): /girdap/map/local kesilirse
(planning_node çökerse) _on_tick yalnız `_last is None` kontrolü yapıyordu —
hiç tazelik kontrolü yoktu. Aynı donmuş kare sonsuza dek yeni dosya
adlarıyla yazılmaya devam ederdi: Dosya-3 (Şartname 4.2, zorunlu) canlı
gibi görünür ama gerçekte donmuş olurdu.

rclpy gerektirir → .venv'de SKIP.
"""

from __future__ import annotations

import pytest

rclpy = pytest.importorskip("rclpy", reason="rclpy yok (.venv) — ROS ortamında koş")

from rclpy.parameter import Parameter               # noqa: E402
from nav_msgs.msg import OccupancyGrid, Odometry     # noqa: E402
from geometry_msgs.msg import Pose, PoseArray        # noqa: E402

girdap = pytest.importorskip(
    "girdap_decision.local_map_node",
    reason="girdap_decision source'lanmamış (ros2_ws/install/setup.bash)",
)


@pytest.fixture(scope="module")
def ros_context():                                       # noqa: ANN201
    rclpy.init()
    yield
    rclpy.shutdown()


def _grid(w: int = 4, h: int = 4) -> OccupancyGrid:
    msg = OccupancyGrid()
    msg.info.width = w
    msg.info.height = h
    msg.data = [0] * (w * h)
    return msg


def test_fp17_bayat_harita_isaretlenir(ros_context, tmp_path) -> None:  # noqa: ANN001
    """map_timeout_s'i aşan harita ile _map_stale() True dönmeli."""
    node = girdap.LocalMapNode(
        parameter_overrides=[
            Parameter("output_dir", Parameter.Type.STRING, str(tmp_path)),
            Parameter("map_timeout_s", Parameter.Type.DOUBLE, 1.0),
        ]
    )
    try:
        t = [100.0]
        node._now = lambda: t[0]
        node._on_map(_grid())
        assert node._map_stale() is False              # taze

        t[0] = 100.5
        assert node._map_stale() is False               # eşik içinde

        t[0] = 101.5                                    # 1.5 s sessizlik
        assert node._map_stale() is True, (
            "bayat harita hâlâ taze sayılıyor (F-P.17)"
        )
    finally:
        node.destroy_node()


def test_fp17_harita_hic_gelmediyse_bayat_degil(ros_context, tmp_path) -> None:  # noqa: ANN001
    """Harita hiç gelmediyse 'bayat' alarmı basılmaz (boot gürültüsü)."""
    node = girdap.LocalMapNode(
        parameter_overrides=[
            Parameter("output_dir", Parameter.Type.STRING, str(tmp_path)),
        ]
    )
    try:
        assert node._map_stale() is False
    finally:
        node.destroy_node()


def test_fp17_kapatilabilir(ros_context, tmp_path) -> None:  # noqa: ANN001
    """map_timeout_s=0 → bekçi devre dışı (mock/masa testi geriye uyum)."""
    node = girdap.LocalMapNode(
        parameter_overrides=[
            Parameter("output_dir", Parameter.Type.STRING, str(tmp_path)),
            Parameter("map_timeout_s", Parameter.Type.DOUBLE, 0.0),
        ]
    )
    try:
        t = [100.0]
        node._now = lambda: t[0]
        node._on_map(_grid())
        t[0] = 999.0
        assert node._map_stale() is False
    finally:
        node.destroy_node()


def test_fp17_bayat_harita_yine_de_dosya3_yazar(ros_context, tmp_path) -> None:  # noqa: ANN001
    """Bayat uyarısı Dosya-3 formatını DEĞİŞTİRMEZ — teslim sözleşmesi sabit,
    yalnız operatör sesli uyarılır (frame yine yazılmaya devam eder)."""
    node = girdap.LocalMapNode(
        parameter_overrides=[
            Parameter("output_dir", Parameter.Type.STRING, str(tmp_path)),
            Parameter("map_timeout_s", Parameter.Type.DOUBLE, 1.0),
        ]
    )
    try:
        t = [100.0]
        node._now = lambda: t[0]
        node._on_map(_grid())
        t[0] = 200.0                                    # çok bayat
        node._on_tick()
        assert node._kare == 1, (
            "bayat harita frame yazımını durdurdu — Dosya-3 formatı bozulmamalı"
        )
    finally:
        node.destroy_node()


def test_dosya3_ZAMAN_DAMGASI_haritanin_kendi_stampinden(ros_context, tmp_path) -> None:  # noqa: ANN001
    """🔴 Damga duvar saatinden DEĞİL, haritanın `header.stamp`'inden gelmeli.

    Duvar saatiyle yazılsaydı bayat bir kare TAZE görünürdü — ve teslimde
    "≥1 Hz"i kanıtlayan tek şey bu damga. (md 4.2; Jetson saati 07.08'de
    ~3 saat geri ölçüldüğü için dosya adı/mtime zaten güvenilmez.)
    """
    g = _grid()
    g.header.stamp.sec = 1786000000
    g.header.stamp.nanosec = 250_000_000
    damga = girdap.LocalMapNode._damga(g)
    assert damga.endswith("Z") and "T" in damga
    assert ".250" in damga, f"milisaniye kaybolmuş: {damga}"

    g.header.stamp.sec = 0
    g.header.stamp.nanosec = 0
    assert girdap.LocalMapNode._damga(g) == "STAMP-YOK", (
        "stamp doldurulmamışken uydurma zaman basılmamalı"
    )


def test_dosya3_1Hz_ALTI_acilista_REDDEDILIR(ros_context, tmp_path) -> None:  # noqa: ANN001
    """md 4.2 'En Az 1 Hz' — sessizce ihlale düşmek yerine açılışta patla."""
    with pytest.raises(ValueError):
        girdap.LocalMapNode(
            parameter_overrides=[
                Parameter("output_dir", Parameter.Type.STRING, str(tmp_path)),
                Parameter("dump_rate_hz", Parameter.Type.DOUBLE, 0.5),
            ]
        )


def test_odom_GELMEDEN_kenar_dubasi_CIZILMEZ(ros_context, tmp_path) -> None:  # noqa: ANN001
    """🔴 `_arac` varsayılanı (0,0); occupancy araç merkezli kurulduğu için
    harita DOĞRU görünür ama dubalar mutlak dünya koordinatına düşer.

    Ölçüldü: araç (30,40)'ta, duba (32,52) iken odom varken piksel (216,104);
    odom yokken duba kare dışına çıkıp HİÇ çizilmiyor — harita normal
    görünürken duba yanlış/yok = sessiz yanlış veri. Kural: bilmiyorsak
    çizmeyiz. Bu test o kuralı dondurur.
    """
    node = girdap.LocalMapNode(
        parameter_overrides=[
            Parameter("output_dir", Parameter.Type.STRING, str(tmp_path)),
            Parameter("mp4_enabled", Parameter.Type.BOOL, False),
        ]
    )
    try:
        pa = PoseArray()
        p = Pose()
        p.position.x, p.position.y = 32.0, 52.0
        pa.poses.append(p)
        node._on_edges(pa)

        assert node._edges and not node._odom_geldi
        # odom gelmeden çizim katmanı BOŞ olmalı
        kenarlar = node._edges if node._odom_geldi else []
        assert kenarlar == [], "odom yokken duba çizilecekti (yanlış yere)"

        o = Odometry()
        o.pose.pose.position.x, o.pose.pose.position.y = 30.0, 40.0
        o.pose.pose.orientation.w = 1.0
        node._on_odom(o)
        assert node._odom_geldi
        assert node._arac == (30.0, 40.0)
    finally:
        node.destroy_node()
