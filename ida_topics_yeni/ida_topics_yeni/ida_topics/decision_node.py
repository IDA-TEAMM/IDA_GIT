import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from vision_msgs.msg import Detection2DArray
from sensor_msgs.msg import Imu
import math

WAYPOINTS = [(2,0),(12,5),(20,-5),(32,5),(43,-3),(47,4),(51,-3),(55,4),(59,-3),(63,4),(67,-3),(71,4),(75,-3),(90,0),(102,0)]
WAYPOINT_RADIUS = 2.5
LINEAR_SPEED = -0.5
MAX_ANGULAR = 1.0

class DecisionNode(Node):
    def __init__(self):
        super().__init__("decision_node")
        self.pos_x = -10.0
        self.pos_y = 0.0
        self.yaw = None
        self.yaw_rate = 0.0
        self.prev_yaw_rate_err = 0.0
        self.OUTER_KP = 1.0
        self.INNER_KP = 1.0
        self.INNER_KD = 0.0
        self.MAX_YAW_RATE = 1.0
        self.wp_index = 0
        self.orange_buoys = None
        self.yellow_buoys = None
        self.image_center = 320
        mavros_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10)
        self.pose_sub = self.create_subscription(Odometry, "/mavros/local_position/odom", self.pose_cb, mavros_qos)
        self.orange_sub = self.create_subscription(Detection2DArray, "/perception/orange_buoys", self.orange_cb, 10)
        self.yellow_sub = self.create_subscription(Detection2DArray, "/perception/yellow_buoys", self.yellow_cb, 10)
        self.imu_sub = self.create_subscription(Imu, "/imu/data", self.imu_cb, 10)
        self.cmd_vel_pub = self.create_publisher(Twist, "/model/Girdap/cmd_vel", 10)
        self.timer = self.create_timer(0.1, self.process)
        self.get_logger().info("Decision Node baslatildi.")

    def pose_cb(self, msg):
        self.pos_x = msg.pose.pose.position.x
        self.pos_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.yaw = math.atan2(siny, cosy)
        self.yaw_rate = msg.twist.twist.angular.z

    def orange_cb(self, msg):
        self.orange_buoys = msg

    def yellow_cb(self, msg):
        self.yellow_buoys = msg

    def imu_cb(self, msg):
        pass  # yaw artik pose_cb (Gazebo odometry) uzerinden geliyor, IMU kullanilmiyor

    def process(self):
        if self.yaw is None:
            return
        if self.wp_index >= len(WAYPOINTS):
            self.cmd_vel_pub.publish(Twist())
            self.get_logger().info("Parkur tamamlandi!", once=True)
            return
        while self.wp_index < len(WAYPOINTS):
            wx, wy = WAYPOINTS[self.wp_index]
            dx = wx - self.pos_x
            dy = wy - self.pos_y
            dist = math.sqrt(dx*dx + dy*dy)
            if dist < WAYPOINT_RADIUS:
                self.get_logger().info(f"WP{self.wp_index} tamamlandi!")
                self.wp_index += 1
            else:
                break
        if self.wp_index >= len(WAYPOINTS):
            self.cmd_vel_pub.publish(Twist())
            return
        wx, wy = WAYPOINTS[self.wp_index]
        dx = wx - self.pos_x
        dy = wy - self.pos_y
        dist = math.sqrt(dx*dx + dy*dy)
        target_yaw_raw = math.atan2(dy, dx)
        if not hasattr(self, "smoothed_target_yaw") or self.smoothed_target_yaw is None:
            self.smoothed_target_yaw = target_yaw_raw
        diff = target_yaw_raw - self.smoothed_target_yaw
        while diff > math.pi: diff -= 2*math.pi
        while diff < -math.pi: diff += 2*math.pi
        self.smoothed_target_yaw += diff * 0.15
        target_yaw = self.smoothed_target_yaw
        yaw_err = target_yaw - self.yaw
        while yaw_err > math.pi: yaw_err -= 2*math.pi
        while yaw_err < -math.pi: yaw_err += 2*math.pi
        desired_yaw_rate = max(-self.MAX_YAW_RATE, min(self.MAX_YAW_RATE, yaw_err * self.OUTER_KP))
        yaw_rate_err = desired_yaw_rate - self.yaw_rate
        yaw_rate_err_deriv = yaw_rate_err - self.prev_yaw_rate_err
        self.prev_yaw_rate_err = yaw_rate_err
        nav_angular = max(-MAX_ANGULAR, min(MAX_ANGULAR, self.INNER_KP * yaw_rate_err + self.INNER_KD * yaw_rate_err_deriv))
        avoid_angular = 0.0
        orange_dets = (self.orange_buoys.detections if self.orange_buoys and self.orange_buoys.detections else [])
        yellow_dets = (self.yellow_buoys.detections if self.yellow_buoys and self.yellow_buoys.detections else [])
        close_orange = [d for d in orange_dets if d.bbox.size_y > 80]
        close_yellow = [d for d in yellow_dets if d.bbox.size_y > 45]
        close_all = close_orange + close_yellow
        if close_all:
            left_dets = [d for d in close_all if d.bbox.center.position.x < self.image_center]
            right_dets = [d for d in close_all if d.bbox.center.position.x >= self.image_center]
            wp_dist_now = math.sqrt((WAYPOINTS[self.wp_index][0]-self.pos_x)**2 + (WAYPOINTS[self.wp_index][1]-self.pos_y)**2)
            pull = 0.0
            if wp_dist_now < 8.0:
                pull = max(0.0, (8.0 - wp_dist_now) / 8.0)
            if left_dets and not right_dets:
                side_angular = -0.7
                avoid_angular = side_angular * (1.0 - pull*0.8) + nav_angular * (pull*0.8)
                self.get_logger().info(f"Sola kac ang={avoid_angular:.2f}")
            elif right_dets and not left_dets:
                side_angular = 0.7
                avoid_angular = side_angular * (1.0 - pull*0.8) + nav_angular * (pull*0.8)
                self.get_logger().info(f"Saga kac ang={avoid_angular:.2f}")
            elif left_dets and right_dets:
                left_closest = max(left_dets, key=lambda d: d.bbox.size_y)
                right_closest = min(right_dets, key=lambda d: -d.bbox.size_y)
                left_x = max(d.bbox.center.position.x for d in left_dets)
                right_x = min(d.bbox.center.position.x for d in right_dets)
                mid_x = (left_x + right_x) / 2.0
                left_size = left_closest.bbox.size_y
                right_size = right_closest.bbox.size_y
                size_diff = left_size - right_size
                mid_x += size_diff * 0.3
                err = (mid_x - self.image_center) / self.image_center
                gate_angular = -err * 3.0
                wp_dist_now = math.sqrt((WAYPOINTS[self.wp_index][0]-self.pos_x)**2 + (WAYPOINTS[self.wp_index][1]-self.pos_y)**2)
                if wp_dist_now < 8.0:
                    pull = max(0.0, (8.0 - wp_dist_now) / 8.0)
                    avoid_angular = gate_angular * (1.0 - pull*0.8) + nav_angular * (pull*0.8)
                else:
                    avoid_angular = gate_angular
                self.get_logger().info(f"Orta: mid={mid_x:.0f} ang={avoid_angular:.2f}")
        if close_all:
            angular = avoid_angular
        else:
            angular = nav_angular
        cmd = Twist()
        cmd.linear.y = LINEAR_SPEED
        cmd.angular.z = -max(-MAX_ANGULAR, min(MAX_ANGULAR, angular))
        self.cmd_vel_pub.publish(cmd)
        self.get_logger().info(f"[NAV] WP{self.wp_index} dist={dist:.1f} pos=({self.pos_x:.1f},{self.pos_y:.1f}) yaw={math.degrees(self.yaw):.0f} tgt={math.degrees(target_yaw):.0f} nav={nav_angular:.2f} avoid={avoid_angular:.2f} ang={cmd.angular.z:.2f}")

def main(args=None):
    rclpy.init(args=args)
    node = DecisionNode()
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)
    executor.spin()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()