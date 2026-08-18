"""Poz tamponu SAAT TABANI — hayalet dubanın sessiz kalan yarısı (18.08.2026).

🔴 BUNU DOĞURAN ARIZA: `_poz_tampon`, `self._now()` ile dolduruluyordu. O
**bayatlık saati** ve donanımda `time.monotonic` — `saat_kaynagi.bayatlik_saati`
kendi docstring'inde *"mesaj damgası yapılmaz"* diye uyarıyor. Aranan anahtar
ise `msg.header.stamp` = ROS **duvar saati**. Gerçek `_poz_damgada` ile ölçüldü:
tampon 4.630 sn'de, damga 1.787.051.062 sn'de — **~57 yıl** arayla ⇒
`ilk_t <= t <= son_t` ASLA tutmuyor ⇒ her çağrı `None` ⇒ `_damga_pozu_ya_da_son`
EN SON POZA düşüyor = düzeltmenin kapatmak istediği hayalet duba yolu AÇIK.

🪤 Neden kimse görmedi: ölçüm **Gazebo**'da yapıldı, orada `use_sim_time=true`
⇒ `bayatlik_saati` ROS saatine döner, tabanlar ÇAKIŞIR, düzeltme çalışır.
`hardware.launch.py` varsayılanı `use_sim_time=false` ⇒ **teknede** sessizce
devre dışı. Ve bu özelliğin **hiç testi yoktu** — bu dosya o boşluğu kapatıyor.
"""

from __future__ import annotations

import math

import pytest

rclpy = pytest.importorskip("rclpy", reason="rclpy yok (.venv) — ROS ortamında koş")

from nav_msgs.msg import Odometry                       # noqa: E402
from rclpy.parameter import Parameter                   # noqa: E402

pn = pytest.importorskip(
    "girdap_decision.planning_node",
    reason="girdap_decision source'lanmamış (ros2_ws/install/setup.bash)",
)

#: Gerçekçi bir ROS duvar saati anı (epoch). Monotonic'ten mertebe farkı VAR —
#: testin bütün mesele
DUVAR = 1_787_051_062.0
#: Bayatlık saatinin donanımdaki tipik değeri (makine açılışından beri).
MONOTONIC = 4_630.5


@pytest.fixture(scope="module")
def ros_context():                                      # noqa: ANN201
    rclpy.init()
    yield
    rclpy.shutdown()


def _odom(t: float | None, x: float = 0.0, y: float = 0.0,
          psi: float = 0.0) -> Odometry:
    """`t` = header.stamp (sn). None → damgasız mesaj (stamp 0)."""
    m = Odometry()
    m.pose.pose.position.x = x
    m.pose.pose.position.y = y
    m.pose.pose.orientation.z = math.sin(psi / 2.0)
    m.pose.pose.orientation.w = math.cos(psi / 2.0)
    if t is not None:
        m.header.stamp.sec = int(t)
        m.header.stamp.nanosec = int((t % 1) * 1e9)
    return m


class _Stamp:
    def __init__(self, t: float) -> None:
        self.sec = int(t)
        self.nanosec = int((t % 1) * 1e9)


def _node():
    n = pn.PlanningNode(
        parameter_overrides=[Parameter("odom_timeout_s", Parameter.Type.DOUBLE, 1.0)]
    )
    n._now = lambda: MONOTONIC          # donanımdaki bayatlık saati
    return n


# ══════════════════════ ÇEKİRDEK: tabanlar aynı mı ═══════════════════════
def test_tampon_MESAJIN_DAMGASINDA_tutulur(ros_context) -> None:  # noqa: ANN001
    """Tamponun anahtarı `header.stamp` olmalı, bayatlık saati DEĞİL.

    Bu testin kırılması = 57 yıllık taban farkının ve hayalet duba yolunun
    geri gelmesi.
    """
    n = _node()
    try:
        n._on_odom(_odom(DUVAR))
        assert n._poz_tampon, "poz tampona yazılmadı"
        assert n._poz_tampon[-1][0] == pytest.approx(DUVAR, abs=1e-3)
        assert n._poz_tampon[-1][0] != pytest.approx(MONOTONIC), \
            "bayatlık saati (monotonic) tampona yazılmış — asıl arıza bu"
    finally:
        n.destroy_node()


def test_DUVAR_SAATLI_damga_poz_BULUR(ros_context) -> None:  # noqa: ANN001
    """Uçtan uca regresyon: kamera/LiDAR duvar saatiyle damgalıyor.

    Arıza hâlinde bu çağrı `None` dönüyordu ve çağıran EN SON POZA düşüyordu.
    """
    n = _node()
    try:
        for i in range(5):
            n._on_odom(_odom(DUVAR + i * 0.1, x=i * 1.0))
        poz = n._poz_damgada(_Stamp(DUVAR + 0.25))
        assert poz is not None, "duvar saatli damga tamponda BULUNAMADI"
        assert poz[0] == pytest.approx(2.5, abs=1e-6), "interpolasyon yanlış"
        assert n._damga_disi_sayaci == 0
    finally:
        n.destroy_node()


def test_BAYATLIK_hala_monotonic(ros_context) -> None:  # noqa: ANN001
    """`_last_odom_t` DEĞİŞMEDİ — bayatlık saat adımına bağışık kalmalı (F-P.1).

    Damga duvar saatidir ve `girdap-saat-gec` onu adımlayabilir; bayatlığı
    ona bağlamak, saat ileri sıçradığında pozu ANINDA "bayat" yapardı.
    """
    n = _node()
    try:
        n._on_odom(_odom(DUVAR))
        assert n._last_odom_t == pytest.approx(MONOTONIC), \
            "bayatlık saati mesaj damgasına bağlanmış — F-P.1 kırılır"
    finally:
        n.destroy_node()


# ══════════════════════ KORUMALAR ════════════════════════════════════════
def test_DAMGASIZ_odom_tampona_YAZILMAZ(ros_context) -> None:  # noqa: ANN001
    """İki tabanı karıştırmaktansa bilinen davranışa düşmek.

    Damgasızı `_now()` ile yazsaydık tampon karışır ve interpolasyon 57 yıllık
    boşluğu geçip **uydurma poz** üretirdi — sessiz ve teşhis edilemez.
    """
    n = _node()
    try:
        n._on_odom(_odom(None))
        assert not n._poz_tampon, "damgasız mesaj tampona yazıldı"
        assert n._damgasiz_odom == 1, "sessiz kalmamalı — sahada tek kanal bu"
        # Poz yine de kullanılabilir olmalı: yalnız TAMPON kapalı.
        assert n._last_xy is not None
    finally:
        n.destroy_node()


def test_SAAT_GERI_giderse_tampon_TEMIZLENIR(ros_context) -> None:  # noqa: ANN001
    """`girdap-saat-gec` adımı / yayıncı yeniden başlaması.

    Karışık tamponda interpolasyon iki farklı epoch arasında yapılırdı.
    """
    n = _node()
    try:
        for i in range(3):
            n._on_odom(_odom(DUVAR + i * 0.1, x=i * 1.0))
        assert len(n._poz_tampon) == 3
        n._on_odom(_odom(DUVAR - 100.0, x=9.0))          # saat GERİ gitti
        assert len(n._poz_tampon) == 1, "eski epoch kayıtları düşmeliydi"
        assert n._saat_geri_gitti == 1
    finally:
        n.destroy_node()


def test_MONOTONIC_damga_pencere_DISI_kalir(ros_context) -> None:  # noqa: ANN001
    """Kontrol grubu: arızanın kendisi. Monotonic tabanlı bir damga aranırsa
    tamponda BULUNMAMALI — yani iki taban gerçekten ayrık."""
    n = _node()
    try:
        for i in range(3):
            n._on_odom(_odom(DUVAR + i * 0.1))
        assert n._poz_damgada(_Stamp(MONOTONIC)) is None
        assert n._damga_disi_sayaci == 1
    finally:
        n.destroy_node()
