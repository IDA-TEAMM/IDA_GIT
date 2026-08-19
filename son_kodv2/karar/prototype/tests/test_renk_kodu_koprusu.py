"""PARKUR-3 renk köprüsü — FC parametresi → `kamikaze_param_node` (19.08.2026).

🔴 **Neden var:** rengin sahaya girmesinin tek yolu terminalden
`ros2 param set` idi. Şartname s.21: *"Görev yükleme aşamasında … YKİ'de
**sadece YKİ arayüzü** açık olacak"*; s.12: *"bütün bilgisayarların dahili
wi-fi özellikleri kapatılmış olacaktır"*. Mission Planner'ın parametre ekranı
bir YKİ arayüzüdür ve telemetri üzerinden çalışır.

Bu dosya köprünün ROS yolunu **gerçek servislerle** sınar: sahte
`/mavros/param/get` + GERÇEK `KamikazeParamNode`. 13.08'de bu yol
"PC'de mavros_msgs yok" diye hiç koşturulamamıştı; bugün paket kurulu.
"""
from __future__ import annotations

import os
import sys
import time

import pytest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

rclpy = pytest.importorskip("rclpy", reason="rclpy kurulu değil")
pytest.importorskip("mavros_msgs", reason="mavros_msgs kurulu değil")

from mavros_msgs.srv import ParamGet                     # noqa: E402
from rclpy.node import Node                              # noqa: E402
from std_msgs.msg import String                          # noqa: E402

girdap = pytest.importorskip(
    "girdap_decision.renk_kodu_koprusu", reason="girdap_decision kurulu değil")
from girdap_decision.kamikaze_param_node import KamikazeParamNode  # noqa: E402


class _SahteFC(Node):
    """`/mavros/param/get` sağlar — istenen kodu döndürür."""

    def __init__(self, kod: float) -> None:
        super().__init__("sahte_fc")
        self.kod = float(kod)
        self.istenen: list[str] = []
        self.create_service(ParamGet, "/mavros/param/get", self._ver)

    def _ver(self, req, res):                            # noqa: ANN001
        self.istenen.append(req.param_id)
        res.success = True
        res.value.real = self.kod
        res.value.integer = 0
        return res


@pytest.fixture(scope="module")
def ros_context():                                       # noqa: ANN201
    rclpy.init()
    yield
    rclpy.shutdown()


def _dondur(nodelar, saniye=3.0, kosul=None):            # noqa: ANN001
    """Düğümleri birlikte döndür; `kosul()` True olunca erken çık."""
    bitis = time.monotonic() + saniye
    while time.monotonic() < bitis:
        for n in nodelar:
            rclpy.spin_once(n, timeout_sec=0.02)
        if kosul is not None and kosul():
            return True
    return kosul() if kosul is not None else False


def test_FC_kodu_hedef_node_parametresine_UYGULANIR(ros_context) -> None:  # noqa: ANN001
    """🔑 Uçtan uca: SCR_USER1 = 3.0 → `kamikaze_target_color = "siyah"`.

    Şartname s.18 hedef renkleri RAL 3026/6037/**9005 siyah**; kod tablosu
    1=kirmizi 2=yesil 3=siyah (`prototype/mission/renk_kodu.py`).
    """
    fc = _SahteFC(3.0)
    hedef = KamikazeParamNode()
    kopru = girdap.RenkKoduKoprusu()
    try:
        # Köprü periyodu 2 sn; en fazla ~6 sn bekle.
        ok = _dondur(
            [fc, hedef, kopru], 8.0,
            kosul=lambda: str(
                hedef.get_parameter("kamikaze_target_color").value) == "siyah",
        )
        assert ok, (
            "renk uygulanmadı — köprü hedef node'un parametresini set etmiyor "
            f"(okunan param: {fc.istenen[:3]})"
        )
        assert "SCR_USER1" in fc.istenen, "beklenen FC parametresi okunmadı"
    finally:
        for n in (kopru, hedef, fc):
            n.destroy_node()


def test_KOD_0_iken_HICBIR_SEY_yapilmaz(ros_context) -> None:  # noqa: ANN001
    """0 = "karar yok". Elle girilen rengi EZMEMELİ.

    Operatör rengi terminalden girdiyse ve FC parametresi 0'da kaldıysa köprü
    onu temizlemeye kalkmamalı — iki yol yan yana yaşayabilmeli.
    """
    fc = _SahteFC(0.0)
    hedef = KamikazeParamNode()
    hedef.set_parameters([
        rclpy.parameter.Parameter("kamikaze_target_color",
                                  rclpy.Parameter.Type.STRING, "kirmizi")])
    kopru = girdap.RenkKoduKoprusu()
    try:
        _dondur([fc, hedef, kopru], 5.0)
        assert str(hedef.get_parameter("kamikaze_target_color").value) == \
            "kirmizi", "köprü elle girilen rengi EZDİ"
    finally:
        for n in (kopru, hedef, fc):
            n.destroy_node()


def test_HAREKET_BASLAYINCA_yoklama_DURUR(ros_context) -> None:  # noqa: ANN001
    """md 5.5.3.1: hedef bilgisi hareket başladıktan sonra aktarılamaz.

    Hedef node zaten reddediyor; köprü AYRICA okumayı kesiyor — "reddediliyor"
    savunması "hiç denemiyor"dan zayıftır (ve journal'da koşu boyunca param
    trafiği görünmesin).
    """
    fc = _SahteFC(2.0)
    kopru = girdap.RenkKoduKoprusu()
    durum_pub = kopru.create_publisher(String, "/girdap/mission/state", 10)
    try:
        _dondur([fc, kopru], 1.0)
        durum_pub.publish(String(data="PARKUR1"))         # hareket BAŞLADI
        _dondur([fc, kopru], 1.0)
        onceki = len(fc.istenen)
        _dondur([fc, kopru], 5.0)
        assert len(fc.istenen) == onceki, (
            f"hareket başladıktan sonra {len(fc.istenen) - onceki} kez daha "
            "FC parametresi okundu — md 5.5.3.1 yoklaması durmadı"
        )
    finally:
        for n in (kopru, fc):
            n.destroy_node()


def test_FC_kodu_P3_KAPISINI_acar(ros_context) -> None:  # noqa: ANN001
    """🔑🔑 ASIL HALKA: FC'deki kod → latched `/girdap/mission/hedef_rengi`.

    `fsm_node` bu topic'ten `p3_bekleniyor`u öğreniyor ve PARKUR3'e geçiş
    ONA bağlı (`mission_fsm.py`: mission_complete + p3_bekleniyor). Yani bu
    test "renk girildi" ile "kamikaze açılabilir" arasındaki zinciri
    donduruyor. Kopuk olsaydı belirti YOK: tekne son waypoint'te temiz durur.
    """
    from girdap_decision.qos_profiles import latched_qos

    fc = _SahteFC(1.0)                                    # 1 = kirmizi
    hedef = KamikazeParamNode()
    kopru = girdap.RenkKoduKoprusu()
    dinleyici = rclpy.create_node("renk_dinleyici")
    gelen: list[str] = []
    dinleyici.create_subscription(
        String, "/girdap/mission/hedef_rengi",
        lambda m: gelen.append(m.data), latched_qos())
    try:
        ok = _dondur([fc, hedef, kopru, dinleyici], 8.0,
                     kosul=lambda: any(g.strip() for g in gelen))
        assert ok, "hedef rengi HİÇ ilan edilmedi — P3 kapısı açılamaz"
        assert gelen[-1].strip() == "kirmizi", f"yanlış renk ilan edildi: {gelen}"
    finally:
        for n in (dinleyici, kopru, hedef, fc):
            n.destroy_node()
