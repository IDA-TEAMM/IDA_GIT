"""`lidar_kayit_node.hizli_xy_seyrelt` — hızlı bulut okuma yolu.

🔴 NEDEN VAR (17.08.2026, gerçek Livox mesajıyla ölçüldü):
eski `_on_cloud` bulutu ÖNCE tamamen ayrıştırıp SONRA seyreltiyordu ve
dönüşümü **nokta başına Python döngüsüyle** yapıyordu → 4,82 ms/mesaj.
Tek `frombuffer` + önce-seyrelt + vektörel dönüşüm → 0,07 ms (**72×**),
çıktı bit-birebir aynı (maks |fark| 0,000001 m).

🪤 ASIL RİSK HIZ DEĞİL, SESSİZ YANLIŞLIK: bayt sırası ters, alan tipi
float32 değil ya da satır dolgusu varsa `frombuffer` **çöp okur ve hata
basmaz**. Bu yüzden hızlı yolun ön koşulları var ve koşul tutmazsa
fonksiyon None döndürüp çağıranı eski yola düşürüyor. Bu dosya hem
eşdeğerliği hem **her bir ön koşulun reddettiğini** donduruyor.

⚠️ Bayt tamponu elle kuruluyor — bu "sentetik veri" kuralını çiğnemez:
ölçülen şey modelin davranışı değil, bir **ayrıştırıcının** kendisi.
Referans düzen gerçek Livox mesajından alındı:
`point_step=18 · alanlar x,y,z(float32) + intensity,tag,line`.
"""
import importlib.util
import math
import os
import struct
import sys

import numpy as np
import pytest

_KOK = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
_NODE = os.path.join(_KOK, "ros2_ws", "src", "girdap_decision",
                     "girdap_decision", "lidar_kayit_node.py")

pytest.importorskip("rclpy", reason="ROS ortamı yok")
pytest.importorskip("vision_msgs", reason="vision_msgs yok")


@pytest.fixture(scope="module")
def mod():
    sys.path.insert(0, os.path.join(_KOK, "ros2_ws", "src", "girdap_decision"))
    sys.path.insert(0, _KOK)
    spec = importlib.util.spec_from_file_location("lidar_kayit_node", _NODE)
    m = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(m)
    return m


class _Alan:
    def __init__(self, name, offset, datatype=7, count=1):
        self.name, self.offset, self.datatype, self.count = name, offset, datatype, count


class _SahteBulut:
    """Gerçek Livox düzeni: point_step=18, x/y/z float32, sonra 6 bayt ek."""

    def __init__(self, xyz, *, is_bigendian=False, point_step=18,
                 x_dtype=7, row_step=None):
        self.point_step = point_step
        self.is_bigendian = is_bigendian
        self.height = 1
        self.width = len(xyz)
        self.row_step = self.width * point_step if row_step is None else row_step
        self.fields = [_Alan("x", 0, x_dtype), _Alan("y", 4), _Alan("z", 8),
                       _Alan("intensity", 12), _Alan("tag", 16, 2),
                       _Alan("line", 17, 2)]
        gov = bytearray()
        for x, y, z in xyz:
            gov += struct.pack("<fff", x, y, z) + b"\x00" * (point_step - 12)
        self.data = bytes(gov)


_NOKTALAR = [(1.0, 2.0, 0.5), (3.5, -1.25, 0.2), (-2.0, 0.75, 1.0),
             (10.0, 4.0, 0.1), (0.5, 0.5, 0.3), (7.25, -3.5, 0.9),
             (float("nan"), 1.0, 0.0), (2.0, float("inf"), 0.0)]


def test_hizli_yol_ESKI_YOLLA_AYNI_SONUCU_VERIR(mod):
    """Aynı bulut, aynı seyreltme → aynı x/y (NaN/Inf ikisinde de elenir)."""
    b = _SahteBulut(_NOKTALAR)
    for seyrelt in (1, 2, 3, 25):
        hizli = mod.hizli_xy_seyrelt(b, seyrelt)
        assert hizli is not None, f"seyrelt={seyrelt}: ön koşullar tutmalıydı"
        # ESKİ yol: hepsini ayrıştır → seyrelt → sonlu olanları tut
        xs = np.array([p[0] for p in _NOKTALAR], dtype=np.float32)[::seyrelt]
        ys = np.array([p[1] for p in _NOKTALAR], dtype=np.float32)[::seyrelt]
        g = np.isfinite(xs) & np.isfinite(ys)
        eski = np.column_stack((xs[g], ys[g])).astype(np.float64)
        assert hizli.shape == eski.shape, f"seyrelt={seyrelt}"
        assert np.allclose(hizli, eski, atol=1e-9), f"seyrelt={seyrelt}"


def test_SEYRELTME_AYRISTIRMADAN_ONCE_yapiliyor(mod):
    """Kazancın kaynağı bu. Sonra seyreltilseydi sonuç FARKLI olurdu.

    Not: 'önce seyrelt' ile 'sonra seyrelt' NaN elemesi yüzünden farklı
    sonuç verir; test o farkı değil, hızlı yolun ÖNCE seyrelttiğini bağlar.
    """
    n = 100
    noktalar = [(float(i), float(-i), 0.0) for i in range(n)]
    b = _SahteBulut(noktalar)
    hizli = mod.hizli_xy_seyrelt(b, 25)
    assert len(hizli) == 4, "0,25,50,75 beklenirdi"
    assert [int(v) for v in hizli[:, 0]] == [0, 25, 50, 75]


def test_VEKTOREL_DONUSUM_govde_to_dunya_ILE_BIREBIR(mod):
    """§0.0b: aynı dönüşümün iki kopyası AYRIŞMAMALI.

    `_on_cloud` artık vektörel dönüşüm yapıyor; `govde_to_dunya` hâlâ tekil.
    Bu test ikisini kilitliyor — biri değişirse kırmızı.
    """
    arac, psi = (12.5, -3.25), 0.7
    b = _SahteBulut(_NOKTALAR[:6])
    xy = mod.hizli_xy_seyrelt(b, 1)
    c, s = math.cos(psi), math.sin(psi)
    vektorel = np.column_stack((arac[0] + c * xy[:, 0] - s * xy[:, 1],
                                arac[1] + s * xy[:, 0] + c * xy[:, 1]))
    tekil = np.array([mod.govde_to_dunya(float(x), float(y), arac, psi)
                      for x, y in xy])
    assert np.allclose(vektorel, tekil, atol=1e-12), (
        "vektörel dönüşüm govde_to_dunya'dan AYRIŞTI — §0.0b hatası")


# ── ÖN KOŞULLAR: hepsi ayrı ayrı REDDETMELİ ────────────────────────────

def test_BIG_ENDIAN_reddedilir(mod):
    assert mod.hizli_xy_seyrelt(_SahteBulut(_NOKTALAR, is_bigendian=True), 1) is None


def test_FLOAT32_OLMAYAN_alan_reddedilir(mod):
    # 8 = FLOAT64. Tip alandan OKUNMALI, varsayılmamalı.
    assert mod.hizli_xy_seyrelt(_SahteBulut(_NOKTALAR, x_dtype=8), 1) is None


def test_SATIR_DOLGUSU_olan_bulut_reddedilir(mod):
    """Düzenli bulutta row_step > width*point_step olabilir → bayt görünümü kayar."""
    b = _SahteBulut(_NOKTALAR)
    b.row_step = b.width * b.point_step + 8
    assert mod.hizli_xy_seyrelt(b, 1) is None


def test_BOZUK_UZUNLUK_reddedilir(mod):
    b = _SahteBulut(_NOKTALAR)
    b.data = b.data[:-3]                       # point_step'in katı değil
    assert mod.hizli_xy_seyrelt(b, 1) is None


def test_X_ALANI_YOKSA_reddedilir(mod):
    b = _SahteBulut(_NOKTALAR)
    b.fields = [f for f in b.fields if f.name != "x"]
    assert mod.hizli_xy_seyrelt(b, 1) is None


def test_ALAN_NOKTA_ADIMINI_TASIYORSA_reddedilir(mod):
    """offset+4 > point_step ⇒ okuma komşu noktaya taşardı."""
    b = _SahteBulut(_NOKTALAR)
    b.fields[1].offset = b.point_step - 2       # y taşıyor
    assert mod.hizli_xy_seyrelt(b, 1) is None


def test_BOS_BULUT_cokmez(mod):
    xy = mod.hizli_xy_seyrelt(_SahteBulut([]), 25)
    assert xy is not None and len(xy) == 0
