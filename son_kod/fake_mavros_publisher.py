#!/usr/bin/env python3
"""
GIRDAP - Sahte MAVROS yayinlayici (SADECE TEST)
================================================
Gercek Pixhawk/MAVROS YOKKEN ground_speed_publisher.py ve PlotJuggler
duzenini denemek icin. MAVROS'un yayinladigi topic'lerin sahtesini uretir:

    /mavros/local_position/velocity_local   TwistStamped  (hiz bilesenleri, ENU)
    /mavros/local_position/velocity_body    TwistStamped  (body hiz — telemetry_node)
    /mavros/imu/data                        Imu           (yonelim — telemetry_node)
    /mavros/state                           State         (AUTO + armed)
    /mavros/global_position/compass_hdg     Float64       (pusula, derece)
    /mavros/nav_controller_output/output    NavControllerOutput (target_bearing)
    /mavros/rc/out                          RCOut         (itici PWM'leri)

Senaryo: tekne ~20x30 m dikdortgende 1 m/s ile geziyormus gibi;
her 30 saniyede 90 derece donus, hizda hafif dalgalanma, PWM 1500+-150.

KULLANIM (gercek MAVROS calisirken ASLA acma - ayni topic'lere yazar!):
    source /opt/ros/humble/setup.bash
    python3 fake_mavros_publisher.py
"""

import math

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64
from mavros_msgs.msg import State, RCOut, NavControllerOutput


class FakeMavros(Node):

    def __init__(self):
        super().__init__('fake_mavros')

        self.pub_vel = self.create_publisher(
            TwistStamped, '/mavros/local_position/velocity_local', 10)
        self.pub_vel_body = self.create_publisher(
            TwistStamped, '/mavros/local_position/velocity_body', 10)
        self.pub_imu = self.create_publisher(Imu, '/mavros/imu/data', 10)
        self.pub_state = self.create_publisher(State, '/mavros/state', 10)
        self.pub_hdg = self.create_publisher(
            Float64, '/mavros/global_position/compass_hdg', 10)
        self.pub_nav = self.create_publisher(
            NavControllerOutput, '/mavros/nav_controller_output/output', 10)
        self.pub_rc = self.create_publisher(RCOut, '/mavros/rc/out', 10)

        self.t = 0.0
        self.create_timer(0.1, self.tick)        # 10 Hz veri
        self.create_timer(1.0, self.tick_state)  # 1 Hz state
        self.get_logger().info('Sahte MAVROS basladi (AUTO+armed, 10 Hz).')

    def tick(self):
        self.t += 0.1
        now = self.get_clock().now().to_msg()

        # Dikdortgen turu: her 30 s'de bir 90 derece don (0->90->180->270)
        kenar = int(self.t // 30) % 4
        hedef_yon = kenar * 90.0
        # Pusula hedefe yumusak yaklassin + kucuk salinim
        pusula = hedef_yon + 8.0 * math.exp(-(self.t % 30) / 5.0) \
            + 2.0 * math.sin(self.t * 1.3)

        # Hiz ~1 m/s + dalgalanma; donuslerde yavaslama
        hiz = 1.0 + 0.15 * math.sin(self.t * 0.7) \
            - 0.4 * math.exp(-(self.t % 30) / 3.0)
        yon_rad = math.radians(90.0 - pusula)     # pusula -> ENU aci
        vx = hiz * math.cos(yon_rad)              # dogu
        vy = hiz * math.sin(yon_rad)              # kuzey

        v = TwistStamped()
        v.header.stamp = now
        v.twist.linear.x = vx
        v.twist.linear.y = vy
        self.pub_vel.publish(v)

        # Body hiz (tekne hep burnuna dogru gider) — telemetry_node bunu dinler
        vb = TwistStamped()
        vb.header.stamp = now
        vb.twist.linear.x = hiz
        self.pub_vel_body.publish(vb)

        # IMU: ENU yaw quaternion'u (pusula → ENU aci) — roll/pitch ~0
        imu = Imu()
        imu.header.stamp = now
        imu.orientation.z = math.sin(yon_rad / 2.0)
        imu.orientation.w = math.cos(yon_rad / 2.0)
        self.pub_imu.publish(imu)

        h = Float64()
        h.data = pusula % 360.0
        self.pub_hdg.publish(h)

        nav = NavControllerOutput()
        nav.header.stamp = now
        nav.target_bearing = int(hedef_yon)
        nav.nav_bearing = int(pusula)
        nav.wp_dist = int(max(0.0, 30.0 - (self.t % 30)))
        self.pub_nav.publish(nav)

        # Iticiler: temel 1650 (ileri), donuste sol/sag farki
        fark = 80.0 * math.exp(-(self.t % 30) / 3.0)
        rc = RCOut()
        rc.header.stamp = now
        rc.channels = [0] * 8
        rc.channels[0] = int(1650 + fark)   # SERVO1 = sol itici (73)
        rc.channels[2] = int(1650 - fark)   # SERVO3 = sag itici (74)
        self.pub_rc.publish(rc)

    def tick_state(self):
        s = State()
        s.connected = True
        s.armed = True
        s.mode = 'AUTO'
        self.pub_state.publish(s)


def main():
    rclpy.init()
    node = FakeMavros()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
