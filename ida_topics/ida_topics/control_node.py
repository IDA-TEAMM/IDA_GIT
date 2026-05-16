import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Path
from std_msgs.msg import Float32, String
from sensor_msgs.msg import Imu, NavSatFix
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
import math
 
 
class ControlNode(Node):
    def __init__(self):
        super().__init__('control_node')
 
        # --- Parametreler ---
        self.declare_parameter('max_linear_vel', 2.0)
        self.declare_parameter('max_angular_vel', 1.5)
        self.declare_parameter('pid_kp_heading', 1.2)
        self.declare_parameter('pid_ki_heading', 0.0)
        self.declare_parameter('pid_kd_heading', 0.15)
        self.declare_parameter('pid_kp_speed', 0.8)
        self.declare_parameter('waypoint_tolerance', 0.5)
        self.declare_parameter('control_frequency', 20.0)  # Hz
 
        self.max_linear = self.get_parameter('max_linear_vel').value
        self.max_angular = self.get_parameter('max_angular_vel').value
        self.kp_h = self.get_parameter('pid_kp_heading').value
        self.ki_h = self.get_parameter('pid_ki_heading').value
        self.kd_h = self.get_parameter('pid_kd_heading').value
        self.kp_s = self.get_parameter('pid_kp_speed').value
        self.waypoint_tol = self.get_parameter('waypoint_tolerance').value
        freq = self.get_parameter('control_frequency').value
        self.dt = 1.0 / freq
 
        # PID durumu
        self.pid_integral_h = 0.0
        self.pid_prev_error_h = 0.0
 
        # Sistem durumu
        self.current_path = None
        self.current_waypoint_idx = 0
        self.latest_imu = None
        self.latest_gps = None
        self.current_heading = 0.0
        self.current_position = [0.0, 0.0]
        self.mission_status = 'NORMAL'
        self.system_ok = True
        self.battery_level = 100.0
 
        # Watchdog
        self.last_imu_time = None
        self.last_gps_time = None
        self.watchdog_timeout = 1.0  # saniye
 
        # --- Subscriber'lar ---
        self.path_sub = self.create_subscription(
            Path, '/planned_path', self.path_callback, 10)
        self.cmd_vel_sub = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.imu_sub = self.create_subscription(
            Imu, '/imu/data', self.imu_callback, 10)
        self.gps_sub = self.create_subscription(
            NavSatFix, '/gps/fix', self.gps_callback, 10)
        self.mission_sub = self.create_subscription(
            String, '/mission/status', self.mission_callback, 10)
 
        # --- Publisher'lar ---
        self.cmd_vel_out_pub = self.create_publisher(Twist, '/cmd_vel_out', 10)
        self.thruster_pub = self.create_publisher(Float32, '/thruster_cmd', 10)
        self.status_pub = self.create_publisher(String, '/mission/status', 10)
        self.diagnostics_pub = self.create_publisher(
            DiagnosticArray, '/diagnostics', 10)
 
        # --- Kontrol döngüsü ---
        self.timer = self.create_timer(self.dt, self.control_loop)
 
        self.get_logger().info('Control Node başlatıldı. Kontrol frekansı: '
                               f'{freq} Hz')
 
    # ------------------------------------------------------------------ #
    #  CALLBACK'LER
    # ------------------------------------------------------------------ #
    def path_callback(self, msg):
        self.current_path = msg
        self.current_waypoint_idx = 0
        self.get_logger().info(
            f'Yol planı alındı: {len(msg.poses)} waypoint')
 
    def cmd_vel_callback(self, msg):
        # Karar katmanından gelen hız komutunu işle
        safe_cmd = self.apply_limits(msg)
        self.cmd_vel_out_pub.publish(safe_cmd)
        self.apply_thruster(safe_cmd)
 
    def imu_callback(self, msg):
        self.latest_imu = msg
        self.last_imu_time = self.get_clock().now()
        # Yaw açısını çıkar
        q = msg.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.current_heading = math.atan2(siny, cosy)
 
    def gps_callback(self, msg):
        self.latest_gps = msg
        self.last_gps_time = self.get_clock().now()
        self.current_position = [msg.latitude, msg.longitude]
 
    def mission_callback(self, msg):
        self.mission_status = msg.data
        if msg.data == 'FAILSAFE':
            self.system_ok = False
            self.get_logger().error('FAILSAFE komutu alındı!')
 
    # ------------------------------------------------------------------ #
    #  ANA KONTROL DÖNGÜSÜ (20 Hz)
    # ------------------------------------------------------------------ #
    def control_loop(self):
        # 1. Watchdog kontrolü
        if not self.watchdog_check():
            self.failsafe()
            return
 
        # 2. Sistem durumu kontrolü
        if not self.system_ok:
            self.failsafe()
            return
 
        # 3. Veri geçerlilik kontrolü
        if not self.validate_sensors():
            return
 
        # 4. Waypoint takibi
        if self.current_path is not None:
            self.follow_path()
 
        # 5. Durum yayınla
        self.publish_status()
        self.publish_diagnostics()
 
    # ------------------------------------------------------------------ #
    #  WATCHDOG
    # ------------------------------------------------------------------ #
    def watchdog_check(self):
        now = self.get_clock().now()
 
        if self.last_imu_time is not None:
            imu_age = (now - self.last_imu_time).nanoseconds / 1e9
            if imu_age > self.watchdog_timeout:
                self.get_logger().error(
                    f'IMU timeout! {imu_age:.2f}s veri yok.')
                return False
 
        if self.last_gps_time is not None:
            gps_age = (now - self.last_gps_time).nanoseconds / 1e9
            if gps_age > 2.0:  # GPS için daha uzun tolerans
                self.get_logger().warn(
                    f'GPS timeout! {gps_age:.2f}s veri yok.')
 
        return True
 
    # ------------------------------------------------------------------ #
    #  SENSÖR DOĞRULAMA
    # ------------------------------------------------------------------ #
    def validate_sensors(self):
        if self.latest_imu is None:
            self.get_logger().warn('IMU verisi bekleniyor...')
            return False
        return True
 
    # ------------------------------------------------------------------ #
    #  YOL TAKİBİ (Pure Pursuit)
    # ------------------------------------------------------------------ #
    def follow_path(self):
        if (self.current_waypoint_idx >= len(self.current_path.poses)):
            self.get_logger().info('Tüm waypoint\'ler tamamlandı.')
            self.current_path = None
            return
 
        target = self.current_path.poses[self.current_waypoint_idx]
        tx = target.pose.position.x
        ty = target.pose.position.y
 
        # Hedefe mesafe
        dist = math.sqrt(tx**2 + ty**2)
 
        # Waypoint tamamlandı mı?
        if dist < self.waypoint_tol:
            self.current_waypoint_idx += 1
            self.get_logger().info(
                f'Waypoint {self.current_waypoint_idx} tamamlandı.')
            return
 
        # Yön hesapla
        target_heading = math.atan2(ty, tx)
        heading_error = target_heading - self.current_heading
        # Açıyı [-pi, pi] aralığına normalize et
        while heading_error > math.pi:
            heading_error -= 2 * math.pi
        while heading_error < -math.pi:
            heading_error += 2 * math.pi
 
        # PID ile açısal hız
        angular_vel = self.pid_heading(heading_error)
 
        # Hız kontrolü: hedefe yaklaştıkça yavaşla
        linear_vel = min(self.max_linear, dist * self.kp_s)
        # Dönüş yapıyorsa yavaşla
        linear_vel *= max(0.2, 1.0 - abs(heading_error) / math.pi)
 
        cmd = Twist()
        cmd.linear.x = linear_vel
        cmd.angular.z = angular_vel
        safe_cmd = self.apply_limits(cmd)
        self.cmd_vel_out_pub.publish(safe_cmd)
        self.apply_thruster(safe_cmd)
 
    # ------------------------------------------------------------------ #
    #  PID HEADING KONTROLCÜ
    # ------------------------------------------------------------------ #
    def pid_heading(self, error):
        self.pid_integral_h += error * self.dt
        # Integral windup koruması
        self.pid_integral_h = max(-1.0, min(1.0, self.pid_integral_h))
        derivative = (error - self.pid_prev_error_h) / self.dt
        output = (self.kp_h * error +
                  self.ki_h * self.pid_integral_h +
                  self.kd_h * derivative)
        self.pid_prev_error_h = error
        return max(-self.max_angular, min(self.max_angular, output))
 
    # ------------------------------------------------------------------ #
    #  HIZ LIMITLEME
    # ------------------------------------------------------------------ #
    def apply_limits(self, cmd):
        safe = Twist()
        safe.linear.x = max(-self.max_linear,
                            min(self.max_linear, cmd.linear.x))
        safe.angular.z = max(-self.max_angular,
                             min(self.max_angular, cmd.angular.z))
        return safe
 
    # ------------------------------------------------------------------ #
    #  THRUSTER KOMUTU
    # ------------------------------------------------------------------ #
    def apply_thruster(self, cmd):
        thruster = Float32()
        # linear.x'i thruster PWM'e çevir (0-1 arası normalize)
        thruster.data = float(
            max(0.0, min(1.0, cmd.linear.x / self.max_linear)))
        self.thruster_pub.publish(thruster)
 
    # ------------------------------------------------------------------ #
    #  FAILSAFE
    # ------------------------------------------------------------------ #
    def failsafe(self):
        # Motor komutları kes
        stop = Twist()
        self.cmd_vel_out_pub.publish(stop)
        thruster = Float32()
        thruster.data = 0.0
        self.thruster_pub.publish(thruster)
 
        status = String()
        status.data = 'FAILSAFE'
        self.status_pub.publish(status)
        self.get_logger().error('FAILSAFE: Motorlar durduruldu!')
 
    # ------------------------------------------------------------------ #
    #  DURUM YAYINI
    # ------------------------------------------------------------------ #
    def publish_status(self):
        status = String()
        status.data = self.mission_status
        self.status_pub.publish(status)
 
    def publish_diagnostics(self):
        diag_array = DiagnosticArray()
        diag_array.header.stamp = self.get_clock().now().to_msg()
 
        # IMU durumu
        imu_status = DiagnosticStatus()
        imu_status.name = 'IMU'
        imu_status.level = (DiagnosticStatus.OK
                            if self.latest_imu is not None
                            else DiagnosticStatus.ERROR)
        imu_status.message = 'OK' if self.latest_imu else 'Veri yok'
        diag_array.status.append(imu_status)
 
        # GPS durumu
        gps_status = DiagnosticStatus()
        gps_status.name = 'GPS'
        gps_status.level = (DiagnosticStatus.OK
                            if self.latest_gps is not None
                            else DiagnosticStatus.WARN)
        gps_status.message = 'OK' if self.latest_gps else 'Veri yok'
        diag_array.status.append(gps_status)
 
        # Sistem durumu
        sys_status = DiagnosticStatus()
        sys_status.name = 'Sistem'
        sys_status.level = (DiagnosticStatus.OK
                            if self.system_ok
                            else DiagnosticStatus.ERROR)
        sys_status.message = self.mission_status
        diag_array.status.append(sys_status)
 
        self.diagnostics_pub.publish(diag_array)
 
 
def main(args=None):
    rclpy.init(args=args)
    node = ControlNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
 
 
if __name__ == '__main__':
    main()
 