import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray, Detection2D, BoundingBox2D
from std_msgs.msg import String
import numpy as np
import cv2
from ultralytics import YOLO

class PerceptionNode(Node):
    def __init__(self):
        super().__init__('perception_node')
        self.model = YOLO('/root/best.pt')
        self.get_logger().info('YOLO modeli yuklendi!')
        self.image_sub = self.create_subscription(
            Image, '/camera/image_raw', self.image_callback, 10)
        self.objects_pub = self.create_publisher(
            Detection2DArray, '/perception/objects', 10)
        self.buoy_target_pub = self.create_publisher(
            Detection2DArray, '/perception/buoy_target', 10)
        self.orange_pub = self.create_publisher(
            Detection2DArray, '/perception/orange_buoys', 10)
        self.yellow_pub = self.create_publisher(
            Detection2DArray, '/perception/yellow_buoys', 10)

    def get_color(self, bgr_roi):
        if bgr_roi.size == 0:
            return 'unknown'
        hsv = cv2.cvtColor(bgr_roi, cv2.COLOR_BGR2HSV)
        avg_h = np.mean(hsv[:,:,0])
        avg_s = np.mean(hsv[:,:,1])
        self.get_logger().info(f'HSV avg_h={avg_h:.1f} avg_s={avg_s:.1f}')
        if avg_s < 20:
            return 'unknown'
        if 10 <= avg_h <= 34:
            return 'orange'
        elif 35 <= avg_h <= 55:
            return 'yellow'
        else:
            return 'other'

    def image_callback(self, msg):
        self.get_logger().info(f'Encoding: {msg.encoding}', once=True)
        img = np.frombuffer(msg.data, dtype=np.uint8).reshape(
            msg.height, msg.width, -1)
        if msg.encoding == 'rgb8':
            # Gazebo R8G8B8 → BGR
            bgr = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
        else:
            bgr = img

        results = self.model(bgr, conf=0.15, verbose=False)

        all_detections = Detection2DArray()
        orange_detections = Detection2DArray()
        yellow_detections = Detection2DArray()
        all_detections.header.stamp = self.get_clock().now().to_msg()
        orange_detections.header = all_detections.header
        yellow_detections.header = all_detections.header

        for result in results:
            for box in result.boxes:
                x1, y1, x2, y2 = box.xyxy[0].tolist()
                x1i, y1i, x2i, y2i = int(x1), int(y1), int(x2), int(y2)
                roi = bgr[y1i:y2i, x1i:x2i]
                color = self.get_color(roi)
                self.get_logger().info(f'Renk: {color}')

                det = Detection2D()
                det.bbox = BoundingBox2D()
                det.bbox.center.position.x = float((x1 + x2) / 2)
                det.bbox.center.position.y = float((y1 + y2) / 2)
                det.bbox.size_x = float(x2 - x1)
                det.bbox.size_y = float(y2 - y1)

                all_detections.detections.append(det)
                if color == 'orange':
                    orange_detections.detections.append(det)
                elif color == 'yellow':
                    yellow_detections.detections.append(det)

        self.objects_pub.publish(all_detections)
        self.buoy_target_pub.publish(orange_detections)
        self.orange_pub.publish(orange_detections)
        self.yellow_pub.publish(yellow_detections)

        if all_detections.detections:
            self.get_logger().info(
                f'Toplam: {len(all_detections.detections)}, '
                f'Turuncu: {len(orange_detections.detections)}, '
                f'Sari: {len(yellow_detections.detections)}')

def main(args=None):
    rclpy.init(args=args)
    node = PerceptionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
