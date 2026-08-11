"""`KamikazeHedefKapisi` ROS davranışı — madde #4, md 5.5.3.1.

Çekirdek mantık `test_kamikaze_hedef.py`'de test ediliyor. Burada ROS'a özgü
üç şey doğrulanıyor:
  1. `ros2 param set` yolu GERÇEKTEN çalışıyor (parametre callback'i olmadan
     bu node'larda hiçbir etkisi olmazdı — belgede yazan aktarım yolu sessizce
     ölü kalırdı),
  2. md 5.5.3.1 zamanlama kapısı ROS parametre değerini de REDDEDİYOR,
  3. görev durumu topic'inden beslenme.

rclpy gerektirir → yoksa dürüst SKIP (laptopta ROS yok, Jetson'da koşar).
Çalıştır: pytest prototype/tests/test_kamikaze_param_node.py -v
"""

from __future__ import annotations

from dataclasses import dataclass

import pytest

rclpy = pytest.importorskip("rclpy", reason="rclpy yok — ROS ortamında koş")

from rclpy.node import Node                              # noqa: E402
from rclpy.parameter import Parameter                    # noqa: E402
from std_msgs.msg import String                          # noqa: E402

kp = pytest.importorskip(
    "girdap_decision.kamikaze_param",
    reason="girdap_decision import edilemedi (ros2_ws source'lanmamış)",
)

from prototype.mission.kamikaze_hedef import (           # noqa: E402
    CLASS_HEDEF,
    CLASS_KIRMIZI,
    CLASS_PARKUR_KENARI,
    CLASS_YESIL,
)


@dataclass
class _T:
    class_id: int


@pytest.fixture(scope="module")
def ros_context():                                       # noqa: ANN201
    rclpy.init()
    yield
    rclpy.shutdown()


def _kapili_node(ros_context, renk: str = ""):           # noqa: ANN001, ANN202
    """Yalın bir Node + kapı — kamera/füzyon node'unun ağırlığı olmadan."""
    n = Node(
        "kamikaze_test_node",
        parameter_overrides=[Parameter("kamikaze_target_color", value=renk)],
        allow_undeclared_parameters=False,
    )
    return n, kp.KamikazeHedefKapisi(n)


def _durum_ver(node, kapi, durum: str) -> None:          # noqa: ANN001
    """Görev durumunu callback'i doğrudan çağırarak besle (DDS'e bağlı olmadan)."""
    m = String()
    m.data = durum
    kapi._on_mission_state(m)


def test_varsayilan_hedef_ATANMAMIS(ros_context) -> None:  # noqa: ANN001
    n, k = _kapili_node(ros_context)
    try:
        assert k.sinif is None
        t = [_T(CLASS_KIRMIZI)]
        assert k.uygula(t) == 0
        assert t[0].class_id == CLASS_KIRMIZI      # dokunulmadı
    finally:
        n.destroy_node()


def test_config_ten_gelen_renk_okunuyor(ros_context) -> None:  # noqa: ANN001
    n, k = _kapili_node(ros_context, "kirmizi")
    try:
        assert k.sinif == CLASS_KIRMIZI
        t = [_T(CLASS_KIRMIZI), _T(CLASS_YESIL)]
        assert k.uygula(t) == 1
        assert t[0].class_id == CLASS_HEDEF
        assert t[1].class_id == CLASS_YESIL
    finally:
        n.destroy_node()


def test_ros2_param_set_GERCEKTEN_etkili(ros_context) -> None:  # noqa: ANN001
    """🔴 Callback olmadan `ros2 param set` bu node'da HİÇBİR ŞEY yapmazdı."""
    n, k = _kapili_node(ros_context)
    try:
        _durum_ver(n, k, "BEKLEMEDE")
        sonuc = n.set_parameters(
            [Parameter("kamikaze_target_color", value="yesil")]
        )
        assert sonuc[0].successful, sonuc[0].reason
        assert k.sinif == CLASS_YESIL
        t = [_T(CLASS_YESIL)]
        assert k.uygula(t) == 1 and t[0].class_id == CLASS_HEDEF
    finally:
        n.destroy_node()


def test_HAREKET_BASLADIKTAN_SONRA_param_set_REDDEDILIYOR(ros_context) -> None:  # noqa: ANN001
    """🔴 md 5.5.3.1: harekete başladıktan sonra aktarım YASAK.

    Reddin ROS parametre DEĞERİNE de yansıması şart — yalnız log basıp değeri
    kabul etmek, "kod izin verdi" yanılgısı yaratırdı.
    """
    n, k = _kapili_node(ros_context, "kirmizi")
    try:
        _durum_ver(n, k, "PARKUR2")
        sonuc = n.set_parameters(
            [Parameter("kamikaze_target_color", value="yesil")]
        )
        assert not sonuc[0].successful
        assert "5.5.3.1" in sonuc[0].reason
        assert k.sinif == CLASS_KIRMIZI            # eski değer KORUNDU
        assert (
            n.get_parameter("kamikaze_target_color").value == "kirmizi"
        ), "ROS parametre degeri de degismemis olmali"
    finally:
        n.destroy_node()


def test_KILL_sonrasi_yeniden_verilebiliyor(ros_context) -> None:  # noqa: ANN001
    """md 5.5.3.1 yeniden başlama hakkı: hareket bittikten sonra renk yeniden
    verilebilmeli (bkz. /girdap/mission/reset).
    """
    n, k = _kapili_node(ros_context, "kirmizi")
    try:
        _durum_ver(n, k, "PARKUR1")
        assert not n.set_parameters(
            [Parameter("kamikaze_target_color", value="yesil")]
        )[0].successful
        _durum_ver(n, k, "KILL")
        assert n.set_parameters(
            [Parameter("kamikaze_target_color", value="yesil")]
        )[0].successful
        assert k.sinif == CLASS_YESIL
    finally:
        n.destroy_node()


def test_yasak_renk_param_set_ile_de_giremiyor(ros_context) -> None:  # noqa: ANN001
    """Turuncu = kenar dubası; kapılar kaybolur + tekne kapıya sürer."""
    n, k = _kapili_node(ros_context)
    try:
        _durum_ver(n, k, "BEKLEMEDE")
        sonuc = n.set_parameters(
            [Parameter("kamikaze_target_color", value="turuncu")]
        )
        assert not sonuc[0].successful
        assert "kenar" in sonuc[0].reason.lower()
        assert k.sinif is None
    finally:
        n.destroy_node()


def test_gecersiz_renk_ile_acilista_node_OLMUYOR(ros_context) -> None:  # noqa: ANN001
    """Parkur-1/2 hedef rengine bağlı değil — onları da düşürmek zararı büyütür."""
    n, k = _kapili_node(ros_context, "mor")     # geçersiz
    try:
        assert k.sinif is None                  # hedefsiz, ama ayakta
    finally:
        n.destroy_node()


def test_bos_deger_ile_hedef_kaldirilabiliyor(ros_context) -> None:  # noqa: ANN001
    n, k = _kapili_node(ros_context, "kirmizi")
    try:
        _durum_ver(n, k, "BEKLEMEDE")
        assert n.set_parameters(
            [Parameter("kamikaze_target_color", value="")]
        )[0].successful
        assert k.sinif is None
    finally:
        n.destroy_node()


def test_idempotent_zaten_etiketli_sahnede_yanlis_alarm_YOK(ros_context) -> None:  # noqa: ANN001
    """İki node birlikte koşarsa yukarı akış zaten 2'ye taşımış olur; füzyon
    "hedef hiç görülmedi" diye yanlış alarm VERMEMELİ.
    """
    n, k = _kapili_node(ros_context, "kirmizi")
    try:
        t = [_T(CLASS_HEDEF), _T(CLASS_PARKUR_KENARI)]   # kamera zaten taşımış
        assert k.uygula(t) == 0                          # taşınacak kırmızı yok
        assert k._gorulmedi_uyarildi is False, (
            "zaten-hedef olan sahnede 'gorulmedi' uyarisi tetiklenmis"
        )
    finally:
        n.destroy_node()
