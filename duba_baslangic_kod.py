
import rclpy
from rclpy.node import Node
import geometry_msgs.msg

class Duba: #nasıl sınıflandıracağımız
    def __init__(self, sinif_id, x, y):
        self.id = sinif_id
        self.x = x
        self.y = y
        self.tip = "Kenar (turuncu)" if sinif_id == 0 else "Engel (sari)"

class IdaAlgilamaDugumu(Node): #iletişim yetenekleri(mesaj gönderme, veri tutma)
    def __init__(self):
        super().__init__('ida_perception_node')
        self.subscription = self.create_subscription( #abone sistemi
            geometry_msgs.msg.Point,                  #beklenen veri tipi
            '/oakd/detections',     #dinlenecek kanal
            self.tespit_callback,   #veri gelince hangi fonksiyon çalışacak
            10)                     #sistem kalitesi(sistem yoğunlaşınca 10 mesajı beklemede tutar)
        self.tespit_edilenler = []

    def tespit_callback(self, msg):
        yeni_duba = Duba(int(msg.z), msg.x, msg.y)

        for kayitli in self.tespit_edilenler:
            fark = ((kayitli.x - yeni_duba.x)**2 + (kayitli.y - yeni_duba.y)**2)**0.5 #ne kadar yer değiştirdi
            if fark < 0.10:
                return

        self.tespit_edilenler.append(yeni_duba) #listenin sonuna ekler
        self.get_logger().info(
            f"Tespit Kaydedildi: {yeni_duba.tip} | Konum: ({yeni_duba.x:.2f}, {yeni_duba.y:.2f})"
        ) #ekrana yazdırır

    def tespitleri_listele(self):
        if not self.tespit_edilenler:
            self.get_logger().info("Hiç duba tespit edilmedi.")
            return

        self.get_logger().info(f"--- Toplam {len(self.tespit_edilenler)} Duba Tespit Edildi ---")
        for i, duba in enumerate(self.tespit_edilenler):
            self.get_logger().info(
                f"  [{i+1}] {duba.tip} | Konum: ({duba.x:.2f}, {duba.y:.2f})"
            )

def main():
    rclpy.init()
    node = IdaAlgilamaDugumu()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.tespitleri_listele()
        node.destroy_node()
        rclpy.shutdown()
if __name__ == '__main__':
    main()



        
    
