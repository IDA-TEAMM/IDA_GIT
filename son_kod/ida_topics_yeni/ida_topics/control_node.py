#!/usr/bin/env python3
"""
IDA/Girdap USV - Control Node (MAVROS Entegrasyonlu)
=====================================================
Mimari:
  decision_node → /cmd_vel → control_node → MAVROS → ArduPilot → Pixhawk → ESC

Mod yonetimi:
  - Baslangic: RC ile arm + AUTO mod
  - Parkur: GUIDED modda /cmd_vel komutlarini MAVROS'a ilet
  - Bitis: RC ile MANUAL mod
  - Acil: RC kill-switch → DISARM

Yazar: IDA/Girdap Takim 989124 - Alt Alan B
Tarih: Haziran 2026
"""

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from geometry_msgs.msg import Twist, TwistStamped
from sensor_msgs.msg import Imu, NavSatFix
from std_msgs.msg import String, Bool
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from mavros_msgs.msg import State, RCIn, OverrideRCIn
from mavros_msgs.srv import CommandBool, SetMode

import math
import time


# ── Sabitler ──────────────────────────────────────────────────────────────────
RC_KILL_CHANNEL     = 7   # RC kanal 8 (0-indexed=7) → kill switch
RC_KILL_THRESHOLD   = 1500  # bu deger altinda kill aktif
RC_MANUAL_CHANNEL   = 4   # RC kanal 5 (0-indexed=4) → mod secimi
RC_MANUAL_THRESHOLD = 1700  # bu deger ustunde manuel moda gec

WATCHDOG_TIMEOUT_S  = 2.0   # cmd_vel gelmezse durdur
ARMING_TIMEOUT_S    = 10.0  # arm icin bekleme suresi


class ControlNode(Node):
    """
    MAVROS entegrasyonlu kontrol node'u.
    /cmd_vel komutlarini alip ArduPilot'a iletir.
    RC kill-switch ve mod yonetimini yapar.
    """

    def __init__(self):
        super().__init__('control_node')
        self.cb_group = ReentrantCallbackGroup()

        # ── Parametreler ──────────────────────────────────────────────────────
        self.declare_parameter('max_linear_vel',  1.5)
        self.declare_parameter('max_angular_vel', 1.0)
        self.declare_parameter('control_frequency', 20.0)
        self.declare_parameter('auto_arm', False)   # Baslangicta otomatik arm

        self.max_lin  = self.get_parameter('max_linear_vel').value
        self.max_ang  = self.get_parameter('max_angular_vel').value
        freq          = self.get_parameter('control_frequency').value
        self.auto_arm = self.get_parameter('auto_arm').value
        self.dt       = 1.0 / freq

        # ── Durum degiskenleri ────────────────────────────────────────────────
        self.mavros_state     = State()
        self.latest_cmd_vel   = Twist()
        self.last_cmd_time    = self.get_clock().now()
        self.kill_active      = False
        self.manual_override  = False
        self.system_ready     = False

        # ── Subscribers ───────────────────────────────────────────────────────
        # decision_node'dan gelen hiz komutu
        self.create_subscription(
            Twist, '/cmd_vel',
            self._cmd_vel_cb, 10,
            callback_group=self.cb_group)

        # ArduPilot durumu (armed, mode, connected)
        self.create_subscription(
            State, '/mavros/state',
            self._mavros_state_cb, 10,
            callback_group=self.cb_group)

        # RC girisini izle (kill-switch, manuel override)
        self.create_subscription(
            RCIn, '/mavros/rc/in',
            self._rc_in_cb, 10,
            callback_group=self.cb_group)

        # ── Publishers ────────────────────────────────────────────────────────
        # MAVROS'a gonderilecek hiz komutu (GUIDED modda)
        self.cmd_vel_pub = self.create_publisher(
            TwistStamped,
            '/mavros/setpoint_velocity/cmd_vel',
            10)

        # RC override (failsafe/kill icin)
        self.rc_override_pub = self.create_publisher(
            OverrideRCIn,
            '/mavros/rc/override',
            10)

        # Durum yayini (diger nodlar icin)
        self.status_pub = self.create_publisher(
            String, '/control/status', 10)

        self.diag_pub = self.create_publisher(
            DiagnosticArray, '/diagnostics', 10)

        # ── Servisler ─────────────────────────────────────────────────────────
        self.arming_client   = self.create_client(
            CommandBool, '/mavros/cmd/arming',
            callback_group=self.cb_group)
        self.set_mode_client = self.create_client(
            SetMode, '/mavros/set_mode',
            callback_group=self.cb_group)

        # ── Timer'lar ─────────────────────────────────────────────────────────
        # Ana kontrol dongusu
        self.create_timer(self.dt, self._control_loop,
                          callback_group=self.cb_group)

        # Watchdog: cmd_vel gelmezse dur
        self.create_timer(0.5, self._watchdog,
                          callback_group=self.cb_group)

        # Diagnostic yayini
        self.create_timer(1.0, self._publish_diagnostics,
                          callback_group=self.cb_group)

        self.get_logger().info('Control Node (MAVROS) baslatildi.')
        self.get_logger().info(f'max_linear={self.max_lin} m/s, '
                               f'max_angular={self.max_ang} rad/s')

    # ── Callback'ler ──────────────────────────────────────────────────────────

    def _cmd_vel_cb(self, msg: Twist):
        """decision_node'dan gelen hiz komutunu kaydet."""
        self.latest_cmd_vel = msg
        self.last_cmd_time  = self.get_clock().now()

    def _mavros_state_cb(self, msg: State):
        """ArduPilot baglanti/arm/mod durumunu izle."""
        prev_connected = self.mavros_state.connected
        self.mavros_state = msg

        if msg.connected and not prev_connected:
            self.get_logger().info('ArduPilot/MAVROS baglantisi kuruldu!')
            if self.auto_arm:
                self._request_guided_and_arm()

        if not msg.connected and prev_connected:
            self.get_logger().warn('ArduPilot/MAVROS baglantisi kesildi!')

    def _rc_in_cb(self, msg: RCIn):
        """RC kanallarini izle: kill-switch ve manuel override."""
        if len(msg.channels) <= max(RC_KILL_CHANNEL, RC_MANUAL_CHANNEL):
            return

        # Kill switch kontrolu
        kill_val = msg.channels[RC_KILL_CHANNEL]
        if kill_val > 0 and kill_val < RC_KILL_THRESHOLD:
            if not self.kill_active:
                self.get_logger().warn('RC KILL SWITCH AKTIF - Sistem durduruluyor!')
                self.kill_active = True
                self._emergency_stop()
        else:
            if self.kill_active:
                self.get_logger().info('RC Kill switch pasif edildi.')
                self.kill_active = False

        # Manuel override kontrolu
        manual_val = msg.channels[RC_MANUAL_CHANNEL]
        if manual_val > RC_MANUAL_THRESHOLD:
            if not self.manual_override:
                self.get_logger().info('RC MANUEL OVERRIDE aktif - MANUAL moda geciliyor.')
                self.manual_override = True
                self._set_mode('MANUAL')
        else:
            if self.manual_override:
                self.get_logger().info('Manuel override pasif - GUIDED moda donuluyor.')
                self.manual_override = False
                self._set_mode('GUIDED')

    # ── Ana kontrol dongusu ───────────────────────────────────────────────────

    def _control_loop(self):
        """
        Ana kontrol dongusu (20Hz).
        cmd_vel komutunu MAVROS'a ilet.
        """
        # Kill veya manuel override aktifse komut gonderme
        if self.kill_active or self.manual_override:
            return

        # MAVROS bagli ve armed degil ise bekle
        if not self.mavros_state.connected:
            return

        # cmd_vel'i clamp'le ve MAVROS'a gonder
        cmd = self.latest_cmd_vel
        twist_stamped = TwistStamped()
        twist_stamped.header.stamp = self.get_clock().now().to_msg()
        twist_stamped.header.frame_id = 'base_link'

        # Hizi sinirla
        twist_stamped.twist.linear.x = max(
            -self.max_lin, min(self.max_lin, cmd.linear.x))
        twist_stamped.twist.linear.y = max(
            -self.max_lin, min(self.max_lin, cmd.linear.y))
        twist_stamped.twist.angular.z = max(
            -self.max_ang, min(self.max_ang, cmd.angular.z))

        self.cmd_vel_pub.publish(twist_stamped)

    def _watchdog(self):
        """cmd_vel timeout kontrolu — 2 saniye gelmezse dur."""
        now = self.get_clock().now()
        elapsed = (now - self.last_cmd_time).nanoseconds / 1e9

        if elapsed > WATCHDOG_TIMEOUT_S:
            # Dur komutu gonder
            self.latest_cmd_vel = Twist()
            self.get_logger().warn(
                f'cmd_vel timeout ({elapsed:.1f}s) - Dur komutu gonderildi.',
                throttle_duration_sec=5.0)

    # ── Yardimci metodlar ─────────────────────────────────────────────────────

    def _emergency_stop(self):
        """Acil durus: sifir hiz gonder."""
        self.latest_cmd_vel = Twist()
        stop_cmd = TwistStamped()
        stop_cmd.header.stamp = self.get_clock().now().to_msg()
        self.cmd_vel_pub.publish(stop_cmd)
        self.get_logger().error('ACIL DURUS gonderildi!')

    def _set_mode(self, mode: str):
        """ArduPilot mod degisiklik istegi."""
        if not self.set_mode_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('set_mode servisi hazir degil.')
            return

        req = SetMode.Request()
        req.custom_mode = mode
        future = self.set_mode_client.call_async(req)
        future.add_done_callback(
            lambda f: self.get_logger().info(
                f'Mod degisiklik sonucu [{mode}]: {f.result()}'))

    def _request_guided_and_arm(self):
        """GUIDED moda gec ve arm et (auto_arm=True ise)."""
        self._set_mode('GUIDED')
        time.sleep(1.0)

        if not self.arming_client.wait_for_service(timeout_sec=3.0):
            self.get_logger().warn('Arming servisi hazir degil.')
            return

        arm_req = CommandBool.Request()
        arm_req.value = True
        future = self.arming_client.call_async(arm_req)
        future.add_done_callback(
            lambda f: self.get_logger().info(
                f'Arm sonucu: {f.result()}'))

    def _publish_diagnostics(self):
        """Sistem durumunu diagnostics topic'ine yayinla."""
        arr = DiagnosticArray()
        arr.header.stamp = self.get_clock().now().to_msg()

        status = DiagnosticStatus()
        status.name = 'control_node'
        status.hardware_id = 'IDA_USV'

        if self.kill_active:
            status.level   = DiagnosticStatus.ERROR
            status.message = 'KILL SWITCH AKTIF'
        elif self.manual_override:
            status.level   = DiagnosticStatus.WARN
            status.message = 'MANUEL OVERRIDE'
        elif not self.mavros_state.connected:
            status.level   = DiagnosticStatus.ERROR
            status.message = 'MAVROS BAGLANTI YOK'
        elif not self.mavros_state.armed:
            status.level   = DiagnosticStatus.WARN
            status.message = 'DISARMED'
        else:
            status.level   = DiagnosticStatus.OK
            status.message = f'OK | Mod: {self.mavros_state.mode}'

        status.values = [
            KeyValue(key='connected',
                     value=str(self.mavros_state.connected)),
            KeyValue(key='armed',
                     value=str(self.mavros_state.armed)),
            KeyValue(key='mode',
                     value=str(self.mavros_state.mode)),
            KeyValue(key='kill_active',
                     value=str(self.kill_active)),
            KeyValue(key='manual_override',
                     value=str(self.manual_override)),
        ]
        arr.status.append(status)
        self.diag_pub.publish(arr)

        # Ayrica basit durum mesaji
        s = String()
        s.data = status.message
        self.status_pub.publish(s)


def main(args=None):
    rclpy.init(args=args)
    node = ControlNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
