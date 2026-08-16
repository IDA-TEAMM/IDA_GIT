"""Hedef rengi parametresinin SAHİBİ var mı — ROS'suz koşar.

🔴 16.08.2026: `KamikazeHedefKapisi` eskiden `perception_camera_node`
içindeydi. O node (HSV yedek kamera hattı) kaldırılınca
`kamikaze_target_color` **sahipsiz** kaldı ⇒ operatör rengi yükleyemez ⇒
`p3_bekleniyor` hiç true olmaz ⇒ FSM **PARKUR3'e geçmez** ⇒ Parkur-3 = 0
puan (145 puan). Ve bu **sessiz**: hata basılmaz, tekne son waypoint'te
temiz durur.

Bu testler rclpy GEREKTİRMEZ — kayıt düşerse geliştirme makinesinde de
yakalanır (ROS testleri burada atlanıyor).
"""
import os

_KOK = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
_SRC = os.path.join(_KOK, "ros2_ws/src/girdap_decision")


def test_node_dosyasi_var():
    assert os.path.exists(os.path.join(_SRC, "girdap_decision/kamikaze_param_node.py"))


def test_setup_entry_point_var():
    """Entry point düşerse `ros2 run` node'u bulamaz."""
    s = open(os.path.join(_SRC, "setup.py")).read()
    assert "kamikaze_param_node = girdap_decision.kamikaze_param_node:main" in s


def test_launch_kaydi_var():
    """Launch kaydı düşerse node hiç başlamaz — parametre yine sahipsiz."""
    s = open(os.path.join(_SRC, "launch/hardware.launch.py")).read()
    assert 'executable="kamikaze_param_node"' in s


def test_parametre_adi_degismedi():
    """Operatörün yazacağı ad; değişirse kart ve prosedür yanlış olur."""
    s = open(os.path.join(_SRC, "girdap_decision/kamikaze_param.py")).read()
    assert '_PARAM = "kamikaze_target_color"' in s
