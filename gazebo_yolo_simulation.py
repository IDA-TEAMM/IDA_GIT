import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image # Gazebo'dan gelecek olan resim paketi türü
from cv_bridge import CvBridge # ROS resmini OpenCV (YOLO) resmine çeviren köprü
import cv2
from ultralytics import YOLO # YOLO kütüphanesi
class GazeboYoloDugumu(Node):
    def __init__(self):
        super().__init__('gazebo_yolo_node')
        self.model = YOLO('yolo11n.pt')
        self.bridge = CvBridge()
        self.subscriptions = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.camera_callback,
            10
        ) 
        self.get_logger().info("YOLOV11 BASLADI")
    def camera_callback(self, msg):
        try:
            
            cv_image= self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            sonuclar = self.model(cv_image)
            for sonuc in sonuclar:
                kutular = sonuc.boxes
                for kutu in kutular:
                    sinif_id = int(kutu.cls[0])
                    guven = float(kutu.conf[0])
                    self.get_logger().info(f"TESPIT EDILDI!! SINIF {sinif_id} | Güven: {guven: .2f}")
            # --- YENİ EKLENEN GÖRSELLEŞTİRME KISMI ---
            # YOLO'nun bulduğu kutuları resmin üzerine otomatik çizdiriyoruz
            cizilmis_resim = sonuclar[0].plot()
            
            # Oluşan bu yeni resmi bir pencerede açıyoruz
            cv2.imshow("Gazebo - İDA YOLOv11 Kamerasi", cizilmis_resim)
            
            # OpenCV penceresinin donmadan videoyu akıtması için 1 milisaniyelik gecikme (ŞARTTIR)
            cv2.waitKey(1)
            # ----------------------------------------
        except Exception as e:
            self.get_logger().error(f"Goruntu islenirken hata olustu: {e}")
def main(args=None):
    rclpy.init(args=args)
    node = GazeboYoloDugumu()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()           
