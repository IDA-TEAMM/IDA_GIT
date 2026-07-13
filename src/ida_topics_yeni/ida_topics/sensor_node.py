import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, Imu, NavSatFix, Image, LaserScan
from std_msgs.msg import String
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
import math
 
 
class SensorNode(Node):
    def __init__(self):
        super().__init__('sensor_node')
 
        # --- Parametreler ---
        self.declare_parameter('lidar_timeout', 0.5)
        self.declare_parameter('imu_timeout', 1.5)
        self.declare_parameter('gps_timeout', 2.0)
        self.declare_parameter('camera_timeout', 0.5)
 
        self.lidar_timeout = self.get_parameter('lidar_timeout').value
        self.imu_timeout = self.get_parameter('imu_timeout').value
        self.gps_timeout = self.get_parameter('gps_timeout').value
        self.camera_timeout = self.get_parameter('camera_timeout').value
 
        # Sensör durumları
        self.last_lidar_time = None
        self.last_imu_time = None
        self.last_gps_time = None
        self.last_camera_time = None
 
        self.lidar_ok = False
        self.imu_ok = False
        self.gps_ok = False
        self.camera_ok = False
 
        # Son veriler
        self.latest_lidar = None
        self.latest_imu = None
        self.latest_gps = None
        self.latest_image = None
 
        # --- Subscriber'lar (Gazebo → ROS2) ---
        # Gazebo Harmonic bridge topic'leri
        self.lidar_sub = self.create_subscription(
            LaserScan, '/lidar/scan', self.lidar_callback, 10)
        self.lidar_pc_sub = self.create_subscription(
            PointCloud2, '/lidar/points', self.lidar_pc_callback, 10)
        self.imu_sub = self.create_subscription(
            Imu, '/imu/data', self.imu_callback, 10)
        self.gps_sub = self.create_subscription(
            NavSatFix, '/gps/fix', self.gps_callback, 10)
        self.camera_sub = self.create_subscription(
            Image, '/camera/image_raw', self.camera_callback, 10)
 
        # --- Publisher'lar (Katman 1 topic'leri) ---
        self.lidar_pub = self.create_publisher(PointCloud2, '/sensor/lidar_points', 10)
        self.imu_pub = self.create_publisher(Imu, '/sensor/imu', 10)
        self.gps_pub = self.create_publisher(NavSatFix, '/sensor/gps', 10)
        self.camera_pub = self.create_publisher(Image, '/sensor/camera', 10)
        self.diagnostics_pub = self.create_publisher(DiagnosticArray, '/diagnostics', 10)
        self.sensor_status_pub = self.create_publisher(String, '/sensor/status', 10)
        
        # --- İzleme döngüsü (5 Hz) ---
        self.monitor_timer = self.create_timer(0.2, self.monitor_sensors)
 
        self.get_logger().info('Sensor Node başlatıldı.')
        self.get_logger().info('Gazebo bridge bağlantısı bekleniyor...')
 
    # ------------------------------------------------------------------ #
    #  CALLBACK'LER — Gazebo'dan gelen veriler
    # ------------------------------------------------------------------ #
    def lidar_callback(self, msg):
        """LaserScan olarak gelen LiDAR verisini işle."""
        self.last_lidar_time = self.get_clock().now()
        self.lidar_ok = True
 
    def lidar_pc_callback(self, msg):
        """PointCloud2 olarak gelen LiDAR verisini ilet."""
        self.latest_lidar = msg
        self.last_lidar_time = self.get_clock().now()
        self.lidar_ok = True
        # Doğrudan ilet
        self.lidar_pub.publish(msg)
 
    def imu_callback(self, msg):
        """IMU verisini doğrula ve ilet."""
        self.last_imu_time = self.get_clock().now()
 
        # Anormal değer kontrolü
        acc_mag = math.sqrt(
            msg.linear_acceleration.x**2 +
            msg.linear_acceleration.y**2 +
            msg.linear_acceleration.z**2)
 
        if acc_mag < 0.01 and self.imu_ok:
            self.get_logger().warn('IMU donmuş olabilir! İvme sıfıra yakın.')
 
        self.latest_imu = msg
        self.imu_ok = True
        self.imu_pub.publish(msg)
 
    def gps_callback(self, msg):
        """GPS verisini doğrula ve ilet."""
        self.last_gps_time = self.get_clock().now()
 
        # Fix kontrolü
        if msg.status.status < 0:
            self.get_logger().warn('GPS fix yok!')
            self.gps_ok = False
        else:
            self.gps_ok = True
 
        self.latest_gps = msg
        self.gps_pub.publish(msg)
 
    def camera_callback(self, msg):
        """Kamera görüntüsünü ilet."""
        self.last_camera_time = self.get_clock().now()
        self.camera_ok = True
        self.latest_image = msg
        self.camera_pub.publish(msg)
 
    # ------------------------------------------------------------------ #
    #  SENSÖR İZLEME (5 Hz)
    # ------------------------------------------------------------------ #
    def monitor_sensors(self):
        now = self.get_clock().now()
 
        # Timeout kontrolleri
        if self.last_lidar_time is not None:
            age = (now - self.last_lidar_time).nanoseconds / 1e9
            if age > self.lidar_timeout:
                self.lidar_ok = False
                self.get_logger().warn(
                    f'LiDAR timeout! {age:.2f}s veri gelmedi.')
 
        if self.last_imu_time is not None:
            age = (now - self.last_imu_time).nanoseconds / 1e9
            if age > self.imu_timeout:
                self.imu_ok = False
                self.get_logger().error(
                    f'IMU timeout! {age:.2f}s veri gelmedi. CRITICAL!')
 
        if self.last_gps_time is not None:
            age = (now - self.last_gps_time).nanoseconds / 1e9
            if age > self.gps_timeout:
                self.gps_ok = False
                self.get_logger().warn(
                    f'GPS timeout! {age:.2f}s veri gelmedi.')
 
        if self.last_camera_time is not None:
            age = (now - self.last_camera_time).nanoseconds / 1e9
            if age > self.camera_timeout:
                self.camera_ok = False
                self.get_logger().warn(
                    f'Kamera timeout! {age:.2f}s veri gelmedi.')
 
        # Diagnostics yayınla
        self.publish_diagnostics()
        self.publish_sensor_status()
 
    # ------------------------------------------------------------------ #
    #  DURUM YAYINLARI
    # ------------------------------------------------------------------ #
    def publish_sensor_status(self):
        status = String()
        status.data = (
            f'lidar:{self.lidar_ok},'
            f'imu:{self.imu_ok},'
            f'gps:{self.gps_ok},'
            f'camera:{self.camera_ok}'
        )
        self.sensor_status_pub.publish(status)
 
    def publish_diagnostics(self):
        diag_array = DiagnosticArray()
        diag_array.header.stamp = self.get_clock().now().to_msg()
 
        sensors = [
            ('LiDAR', self.lidar_ok),
            ('IMU', self.imu_ok),
            ('GPS', self.gps_ok),
            ('Kamera', self.camera_ok),
        ]
 
        for name, ok in sensors:
            s = DiagnosticStatus()
            s.name = name
            s.level = DiagnosticStatus.OK if ok else DiagnosticStatus.ERROR
            s.message = 'Aktif' if ok else 'Veri yok'
            diag_array.status.append(s)
 
        self.diagnostics_pub.publish(diag_array)
 
 
def main(args=None):
    rclpy.init(args=args)
    node = SensorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
 
 
if __name__ == '__main__':
    main()
 