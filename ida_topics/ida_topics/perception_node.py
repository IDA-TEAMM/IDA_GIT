import rclpy
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2, Image, NavSatFix, Imu
from vision_msgs.msg import Detection2DArray, Detection2D, BoundingBox2D
from geometry_msgs.msg import Point, PointStamped
from std_msgs.msg import String
import math
 
 
class PerceptionNode(Node):
    def __init__(self):
        super().__init__('perception_node')
 
        # --- Parametreler ---
        self.declare_parameter('mission_type', 'buoy')  # buoy / gate / obstacle
        self.declare_parameter('confidence_threshold', 0.5)
        self.declare_parameter('min_cluster_size', 5)
        self.declare_parameter('max_cluster_size', 500)
        self.declare_parameter('sync_tolerance', 0.05)  # 50ms
 
        self.mission_type = self.get_parameter('mission_type').value
        self.conf_threshold = self.get_parameter('confidence_threshold').value
        self.sync_tolerance = self.get_parameter('sync_tolerance').value
 
        # Son sensör verileri
        self.latest_lidar_msg = None
        self.latest_image_msg = None
        self.latest_imu_msg = None
        self.latest_gps_msg = None
        self.latest_lidar_time = None
        self.latest_image_time = None
 
        # --- Subscriber'lar (Katman 1: Sensör → Algılama) ---
        self.lidar_sub = self.create_subscription(
            PointCloud2, '/sensor/lidar_points', self.lidar_callback, 10)
        self.image_sub = self.create_subscription(
            Image, '/sensor/camera', self.image_callback, 10)
        self.imu_sub = self.create_subscription(
            Imu, '/sensor/imu', self.imu_callback, 10)
        self.gps_sub = self.create_subscription(
            NavSatFix, '/sensor/gps', self.gps_callback, 10)
        self.mission_sub = self.create_subscription(
            String, '/mission/status', self.mission_callback, 10)
 
        # --- Publisher'lar (Katman 2: Algılama → Karar) ---
        self.objects_pub = self.create_publisher(
            Detection2DArray, '/perception/objects', 10)
        self.obstacles_pub = self.create_publisher(
            PointCloud2, '/perception/obstacles', 10)
        self.buoy_target_pub = self.create_publisher(
            Detection2DArray, '/perception/buoy_target', 10)
        self.gate_target_pub = self.create_publisher(
            PointStamped, '/perception/gate_target', 10)
        self.obstacle_target_pub = self.create_publisher(
            PointStamped, '/perception/obstacle_target', 10)
        self.environment_pub = self.create_publisher(
            String, '/perception/environment', 10)
 
        # --- İşlem döngüsü (10 Hz) ---
        self.timer = self.create_timer(0.1, self.process)
 
        self.get_logger().info('Perception Node başlatıldı.')
 
    # ------------------------------------------------------------------ #
    #  CALLBACK'LER
    # ------------------------------------------------------------------ #
    def lidar_callback(self, msg):
        self.latest_lidar_msg = msg
        self.latest_lidar_time = self.get_clock().now()
 
    def image_callback(self, msg):
        self.latest_image_msg = msg
        self.latest_image_time = self.get_clock().now()
 
    def imu_callback(self, msg):
        self.latest_imu_msg = msg
 
    def gps_callback(self, msg):
        self.latest_gps_msg = msg
 
    def mission_callback(self, msg):
        if msg.data in ['buoy', 'gate', 'obstacle']:
            self.mission_type = msg.data
            self.get_logger().info(f'Görev türü güncellendi: {self.mission_type}')
 
    # ------------------------------------------------------------------ #
    #  ANA İŞLEM DÖNGÜSÜ
    # ------------------------------------------------------------------ #
    def process(self):
        # 1. Zaman senkronizasyonu kontrolü
        if not self.check_sync():
            return
 
        # 2. Aktif göreve göre algılama
        if self.mission_type == 'buoy':
            self.detect_buoy()
        elif self.mission_type == 'gate':
            self.detect_gate()
        elif self.mission_type == 'obstacle':
            self.detect_obstacle()
 
        # 3. Çevre bilgisini yayınla
        self.publish_environment()
 
    # ------------------------------------------------------------------ #
    #  ZAMAN SENKRONİZASYONU
    # ------------------------------------------------------------------ #
    def check_sync(self):
        if self.latest_lidar_time is None or self.latest_image_time is None:
            return False
        now = self.get_clock().now()
        lidar_age = (now - self.latest_lidar_time).nanoseconds / 1e9
        image_age = (now - self.latest_image_time).nanoseconds / 1e9
        if lidar_age > 0.5:
            self.get_logger().warn('LiDAR verisi timeout!')
            return False
        if image_age > 0.5:
            self.get_logger().warn('Kamera verisi timeout!')
            return False
        return True
 
    # ------------------------------------------------------------------ #
    #  ŞAMANDIRA TESPİTİ (Kamera + YOLO benzeri basit tespit)
    # ------------------------------------------------------------------ #
    def detect_buoy(self):
        detections = Detection2DArray()
        detections.header.stamp = self.get_clock().now().to_msg()
        detections.header.frame_id = 'kamera_link'
 
        # Simülasyonda YOLO yerine örnek tespit
        if self.latest_image_msg is not None:
            det = Detection2D()
            det.bbox = BoundingBox2D()
            det.bbox.center.position.x = 0.0
            det.bbox.center.position.y = 0.0
            det.bbox.size_x = 50.0
            det.bbox.size_y = 50.0
            detections.detections.append(det)
 
        self.objects_pub.publish(detections)
        self.buoy_target_pub.publish(detections)
 
    # ------------------------------------------------------------------ #
    #  KAPI TESPİTİ (LiDAR Cluster)
    # ------------------------------------------------------------------ #
    def detect_gate(self):
        detections = Detection2DArray()
        detections.header.stamp = self.get_clock().now().to_msg()
        detections.header.frame_id = 'lidar_link'
 
        if self.latest_lidar_msg is not None:
            # Sol şamandıra
            left = Detection2D()
            left.bbox.center.position.x = -2.0
            left.bbox.center.position.y = 5.0
            detections.detections.append(left)
 
            # Sağ şamandıra
            right = Detection2D()
            right.bbox.center.position.x = 2.0
            right.bbox.center.position.y = 5.0
            detections.detections.append(right)
 
            # Kapı orta noktası
            gate_center = PointStamped()
            gate_center.header.stamp = self.get_clock().now().to_msg()
            gate_center.header.frame_id = 'map'
            gate_center.point.x = 0.0
            gate_center.point.y = 5.0
            gate_center.point.z = 0.0
            self.gate_target_pub.publish(gate_center)
 
        self.objects_pub.publish(detections)
 
    # ------------------------------------------------------------------ #
    #  ENGEL TESPİTİ (LiDAR Occupancy Grid)
    # ------------------------------------------------------------------ #
    def detect_obstacle(self):
        if self.latest_lidar_msg is None:
            return
 
        # Engel konumu yayınla
        obstacle_point = PointStamped()
        obstacle_point.header.stamp = self.get_clock().now().to_msg()
        obstacle_point.header.frame_id = 'lidar_link'
        obstacle_point.point.x = 3.0
        obstacle_point.point.y = 0.0
        obstacle_point.point.z = 0.0
        self.obstacle_target_pub.publish(obstacle_point)
 
        # Engelleri yayınla
        obstacles = PointCloud2()
        obstacles.header = self.latest_lidar_msg.header
        self.obstacles_pub.publish(obstacles)
 
    # ------------------------------------------------------------------ #
    #  ÇEVRE BİLGİSİ
    # ------------------------------------------------------------------ #
    def publish_environment(self):
        env_msg = String()
        gps_ok = self.latest_gps_msg is not None
        imu_ok = self.latest_imu_msg is not None
        env_msg.data = f'mission:{self.mission_type},gps:{gps_ok},imu:{imu_ok}'
        self.environment_pub.publish(env_msg)
 
 
def main(args=None):
    rclpy.init(args=args)
    node = PerceptionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
 
 
if __name__ == '__main__':
    main()
 