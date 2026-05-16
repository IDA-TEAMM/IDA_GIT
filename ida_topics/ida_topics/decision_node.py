import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point, PointStamped
from nav_msgs.msg import Path
from std_msgs.msg import String, Float32
from vision_msgs.msg import Detection2DArray
from sensor_msgs.msg import PointCloud2
import math
 
 
class DecisionNode(Node):
    def __init__(self):
        super().__init__('decision_node')
 
        # --- Parametreler ---
        self.declare_parameter('current_mission', 'buoy')  # buoy / gate / obstacle
        self.declare_parameter('safe_distance', 2.0)
        self.declare_parameter('pid_kp', 1.0)
        self.declare_parameter('pid_ki', 0.0)
        self.declare_parameter('pid_kd', 0.1)
        self.declare_parameter('waypoint_tolerance', 0.5)
 
        self.mission = self.get_parameter('current_mission').value
        self.safe_distance = self.get_parameter('safe_distance').value
 
        # PID değişkenleri
        self.pid_kp = self.get_parameter('pid_kp').value
        self.pid_ki = self.get_parameter('pid_ki').value
        self.pid_kd = self.get_parameter('pid_kd').value
        self.pid_integral = 0.0
        self.pid_prev_error = 0.0
 
        # Durum değişkenleri
        self.current_position = Point()
        self.target = None
        self.latest_objects = None
        self.latest_obstacles = None
        self.latest_environment = None
        self.mission_state = 'NORMAL'
        self.system_ok = True
 
        # --- Subscriber'lar ---
        self.objects_sub = self.create_subscription(
            Detection2DArray, '/perception/objects',
            self.objects_callback, 10)
        self.obstacles_sub = self.create_subscription(
            PointCloud2, '/perception/obstacles',
            self.obstacles_callback, 10)
        self.mission_sub = self.create_subscription(
            String, '/mission/status',
            self.mission_callback, 10)
 
        # --- Publisher'lar ---
        self.path_pub = self.create_publisher(Path, '/planned_path', 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.mission_pub = self.create_publisher(String, '/mission/status', 10)
 
        # --- Kontrol döngüsü (10 Hz) ---
        self.timer = self.create_timer(0.1, self.process)
 
        self.get_logger().info(f'Decision Node başlatıldı. Görev: {self.mission}')
 
    # ------------------------------------------------------------------ #
    #  CALLBACK'LER
    # ------------------------------------------------------------------ #
    def objects_callback(self, msg):
        self.latest_objects = msg
 
    def obstacles_callback(self, msg):
        self.latest_obstacles = msg
 
    def mission_callback(self, msg):
        if msg.data in ['buoy', 'gate', 'obstacle']:
            self.mission = msg.data
            self.get_logger().info(f'Görev değişti: {self.mission}')
 
    # ------------------------------------------------------------------ #
    #  ANA İŞLEM DÖNGÜSÜ
    # ------------------------------------------------------------------ #
    def process(self):
        if not self.system_ok:
            self.failsafe()
            return
 
        if self.mission == 'buoy':
            self.handle_buoy_mission()
        elif self.mission == 'gate':
            self.handle_gate_mission()
        elif self.mission == 'obstacle':
            self.handle_obstacle_mission()
        else:
            self.get_logger().warn('Görev türü bilinmiyor.')
 
    # ------------------------------------------------------------------ #
    #  GÖREV FONKSİYONLARI
    # ------------------------------------------------------------------ #
    def handle_buoy_mission(self):
        """Şamandıra görevi: en yakın şamandırayı bul, hedefe git."""
        if self.latest_objects is None or len(self.latest_objects.detections) == 0:
            self.publish_command(0.0, 0.0)
            return
 
        # En yakın şamandırayı seç
        best = None
        best_dist = float('inf')
        for det in self.latest_objects.detections:
            cx = det.bbox.center.position.x
            cy = det.bbox.center.position.y
            dist = math.sqrt(cx**2 + cy**2)
            if dist < best_dist:
                best_dist = dist
                best = det
 
        if best is None:
            return
 
        # Hedefe yönel
        cx = best.bbox.center.position.x
        cy = best.bbox.center.position.y
        yaw_error = math.atan2(cy, cx)
        linear_vel = min(1.0, best_dist * 0.3)
        angular_vel = self.pid_compute(yaw_error)
 
        self.publish_command(linear_vel, angular_vel)
 
        # Yol planını yayınla
        path = Path()
        path.header.stamp = self.get_clock().now().to_msg()
        path.header.frame_id = 'map'
        self.path_pub.publish(path)
 
    def handle_gate_mission(self):
        """Kapı geçiş görevi: iki şamandıra arasından geç."""
        if self.latest_objects is None or len(self.latest_objects.detections) < 2:
            self.publish_command(0.5, 0.0)
            return
 
        detections = self.latest_objects.detections
        # Sol ve sağ şamandırayı bul
        sorted_dets = sorted(detections,
                             key=lambda d: d.bbox.center.position.x)
        left = sorted_dets[0]
        right = sorted_dets[-1]
 
        # Orta nokta hesapla
        cx = (left.bbox.center.position.x + right.bbox.center.position.x) / 2
        cy = (left.bbox.center.position.y + right.bbox.center.position.y) / 2
        gate_width = abs(right.bbox.center.position.x -
                         left.bbox.center.position.x)
 
        if gate_width < 1.0:
            self.get_logger().warn('Kapı çok dar!')
            self.publish_command(0.0, 0.0)
            return
 
        yaw_error = math.atan2(cy, cx)
        angular_vel = self.pid_compute(yaw_error)
        self.publish_command(0.8, angular_vel)
 
    def handle_obstacle_mission(self):
        """Engel parkuru görevi: engelleri geç."""
        if self.latest_obstacles is None:
            self.publish_command(0.5, 0.0)
            return
 
        # Basit engel kaçınma: sağa veya sola dön
        angular_vel = 0.5  # Varsayılan: sola dön
        self.publish_command(0.4, angular_vel)
 
    # ------------------------------------------------------------------ #
    #  PID KONTROLCÜ
    # ------------------------------------------------------------------ #
    def pid_compute(self, error):
        self.pid_integral += error * 0.1
        derivative = (error - self.pid_prev_error) / 0.1
        output = (self.pid_kp * error +
                  self.pid_ki * self.pid_integral +
                  self.pid_kd * derivative)
        self.pid_prev_error = error
        return max(-1.5, min(1.5, output))
 
    # ------------------------------------------------------------------ #
    #  FAILSAFE
    # ------------------------------------------------------------------ #
    def failsafe(self):
        self.publish_command(0.0, 0.0)
        status = String()
        status.data = 'FAILSAFE'
        self.mission_pub.publish(status)
        self.get_logger().error('FAILSAFE aktif! Motor komutları durduruldu.')
 
    # ------------------------------------------------------------------ #
    #  YARDIMCI FONKSİYONLAR
    # ------------------------------------------------------------------ #
    def publish_command(self, linear_x, angular_z):
        cmd = Twist()
        cmd.linear.x = float(linear_x)
        cmd.angular.z = float(angular_z)
        self.cmd_vel_pub.publish(cmd)
 
 
def main(args=None):
    rclpy.init(args=args)
    node = DecisionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
 
 
if __name__ == '__main__':
    main()
 