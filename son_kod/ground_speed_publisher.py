#!/usr/bin/env python3
"""
GIRDAP - Yer Hizi Yayinlayici   (Alt Alan C / PlotJuggler icin)
================================================================

NE YAPAR:
  /mavros/local_position/velocity_local  topic'ini DINLER
  Iki yeni topic YAYINLAR:
      /girdap/ground_speed     -> gercek yer hizi  = sqrt(vx^2 + vy^2)   [m/s]
      /girdap/speed_setpoint   -> hedef hiz (YALNIZ AUTO + armed iken yayinlanir;
                                  degilse yayin yok = grafikte bosluk, sahte cizgi yok)

NEDEN GEREKLI:
  MAVROS'un velocity_local topic'i hizi BILESENLERE ayirir:
      linear.x = dogu bileseni   (batiya giderken NEGATIF)
      linear.y = kuzey bileseni  (guneye giderken NEGATIF)
  PlotJuggler'da linear.x cizdirilirse, tekne batiya donunce grafik NEGATIF gosterir.
  Sartname "gercek hiz" ister; bu, bilesenlerin buyuklugudur.
  Bu node dogru degeri hesaplayip tek bir temiz topic olarak yayinlar.

GUVENLIK:
  Bu node TAMAMEN PASIFTIR.
    - Pixhawk'a hicbir komut GONDERMEZ
    - Hicbir parametre DEGISTIRMEZ
    - Mevcut topic'leri BOZMAZ
    - MAVROS'a, telemetry_logger'a, donanima DOKUNMAZ
  Yalnizca var olan bir topic'i okur ve yeni topic yayinlar.
  Kapatilirsa hicbir sey bozulmaz; sadece yeni topic'ler kaybolur.

KULLANIM:
  1) ROS2 ortamini yukle:
       source /opt/ros/humble/setup.bash
  2) Calistir (MAVROS ve SITL zaten calisiyorken):
       python3 ground_speed_publisher.py
  3) PlotJuggler'da yenile -> agacta /girdap/ground_speed ve /girdap/speed_setpoint gorunur
  4) Ust panele ikisini de surukle
  5) PlotJuggler: File > Layout > Save   (kaydetmeyi UNUTMA)

WP_SPEED:
  Asagidaki sabit, ArduPilot'un WP_SPEED parametresidir (AUTO modda hedef hiz).
  >>> GERCEK GOREVDEN ONCE QGC > Parameters > WP_SPEED degerini OKU ve
      buradaki degeri ona esitle. telemetry_logger.py icindeki deger ile de
      AYNI olmalidir. <<<
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from geometry_msgs.msg import TwistStamped
from std_msgs.msg import Float64
from mavros_msgs.msg import State


# === AYAR: ArduPilot WP_SPEED parametresi (m/s) ===
# Karar 2026-07-13: WP_SPEED = 1.0 (hardware.yaml fc_cruise_setpoint_mps ile
# AYNI). Gorevden once QGC > Parameters > WP_SPEED ile teyit et.
WP_SPEED = 1.0


class GroundSpeedPublisher(Node):

    def __init__(self):
        super().__init__('girdap_ground_speed')

        self.mode = 'UNKNOWN'
        self.armed = False
        self.yayin_sayisi = 0

        qos = qos_profile_sensor_data

        # --- YAYINLAR ---
        self.pub_hiz = self.create_publisher(
            Float64, '/girdap/ground_speed', 10)
        self.pub_setpoint = self.create_publisher(
            Float64, '/girdap/speed_setpoint', 10)

        # --- DINLER (sadece okur) ---
        self.create_subscription(
            TwistStamped,
            '/mavros/local_position/velocity_local',
            self.hiz_callback, qos)

        self.create_subscription(
            State,
            '/mavros/state',
            self.state_callback, 10)

        self.get_logger().info('GIRDAP yer hizi yayinlayici basladi.')
        self.get_logger().info(f'WP_SPEED = {WP_SPEED} m/s')
        self.get_logger().info('Yayinlanan topicler:')
        self.get_logger().info('  /girdap/ground_speed')
        self.get_logger().info('  /girdap/speed_setpoint')

    def hiz_callback(self, msg):
        """velocity_local geldiginde: buyuklugu hesapla ve yayinla."""
        vx = msg.twist.linear.x   # dogu bileseni
        vy = msg.twist.linear.y   # kuzey bileseni

        yer_hizi = math.sqrt(vx * vx + vy * vy)   # her zaman >= 0

        m = Float64()
        m.data = float(yer_hizi)
        self.pub_hiz.publish(m)

        # Hiz setpointi:
        # ArduPilot Rover AUTO modda gercek bir hiz setpointi YAYINLAMAZ.
        # (/mavros/setpoint_velocity/cmd_vel bostur; QGC'nin airSpeedSetpoint
        #  sutunu da olculen hizin kopyasidir.)
        # Bu nedenle WP_SPEED parametresi sabit deger olarak kullanilir.
        # telemetry_node fc modu ile ayni mantik: gorev disinda YAYIN YOK
        # (0 cizgisi yerine grafikte bosluk — sahte setpoint gorunmez).
        if self.mode == 'AUTO' and self.armed:
            sp = Float64()
            sp.data = float(WP_SPEED)
            self.pub_setpoint.publish(sp)

        self.yayin_sayisi += 1

    def state_callback(self, msg):
        """Mod ve arm durumunu takip et (setpoint mantigi icin)."""
        if msg.mode != self.mode:
            self.get_logger().info(f'Mod: {self.mode} -> {msg.mode}')
        if msg.armed != self.armed:
            self.get_logger().info(
                'Arm durumu: ' + ('ARMED' if msg.armed else 'DISARMED'))
        self.mode = msg.mode
        self.armed = msg.armed

    def destroy_node(self):
        self.get_logger().info(
            f'Kapaniyor. Toplam {self.yayin_sayisi} mesaj yayinlandi.')
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = GroundSpeedPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
