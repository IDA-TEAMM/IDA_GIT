#!/usr/bin/env python3
"""
IDA/Girdap USV - OAK-D Lite Kamera Driver Node
================================================
DepthAI SDK üzerinden OAK-D Lite kamera verisi

Publish:
  /camera/image_raw  → sensor_msgs/Image (bgr8, 1280x720, 30fps)

Gereksinim: pip3 install depthai

Yazar: IDA/Girdap Takım 989124 - Alt Alan B

Not (2026-07-17, gerçek donanım testi): 640x480 önizleme çözünürlüğü ile
sahada 2m mesafedeki bir duba bile net görülemedi/tespit edilemedi — HSV
sınıflandırma için piksel detayı yetersizdi. 1280x720'e çıkarıldı (~4x daha
fazla piksel): hem görüntü netliği hem de aynı fiziksel mesafedeki dubanın
kapladığı piksel alanı (dolayısıyla min_area_px eşiğini aşma mesafesi, yani
etkin menzil) artar. `width`/`height` parametreleri hâlâ launch-arg'dan
override edilebilir, sahada gerekirse ayarlanabilir.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Header

import numpy as np
import threading
import time


class OakdDriverNode(Node):
    # Kamera bağlı değilken yeniden bağlanma denemesi arası bekleme (s).
    _RECONNECT_PERIOD_S = 5.0

    def __init__(self):
        super().__init__('oakd_driver_node')

        # ── Parametreler ──────────────────────────────────────────────────────
        self.declare_parameter('width', 1280)
        self.declare_parameter('height', 720)
        self.declare_parameter('fps', 30)

        self.width  = self.get_parameter('width').value
        self.height = self.get_parameter('height').value
        self.fps    = self.get_parameter('fps').value

        # ── Publisher ─────────────────────────────────────────────────────────
        self.img_pub = self.create_publisher(Image, '/camera/image_raw', 10)

        # ── DepthAI başlat ────────────────────────────────────────────────────
        self.pipeline = None
        self.device   = None
        self.queue    = None
        self._init_depthai()

        # ── Okuma thread'i ────────────────────────────────────────────────────
        self.running = True
        self.thread  = threading.Thread(target=self._capture_loop, daemon=True)
        self.thread.start()

        self.get_logger().info(
            f'OAK-D Lite Driver başlatıldı ({self.width}x{self.height}@{self.fps}fps)')

    def _init_depthai(self):
        """DepthAI pipeline oluştur."""
        try:
            import depthai as dai

            pipeline = dai.Pipeline()

            # RGB kamera node
            cam_rgb = pipeline.createColorCamera()
            # 2026-07-17: sensör çözünürlüğü açıkça 1080p'ye sabitlendi —
            # setPreviewSize TEK BAŞINA hangi taban sensör modundan
            # ölçeklendiğini garanti etmez; 1280x720 önizleme bu tabandan
            # (yukarı örneklemeden, DÜŞÜREREK) üretilsin diye.
            cam_rgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
            cam_rgb.setPreviewSize(self.width, self.height)
            cam_rgb.setInterleaved(False)
            cam_rgb.setFps(self.fps)
            cam_rgb.setBoardSocket(dai.CameraBoardSocket.CAM_A)

            # Çıkış
            xout_rgb = pipeline.createXLinkOut()
            xout_rgb.setStreamName('rgb')
            cam_rgb.preview.link(xout_rgb.input)

            self.pipeline = pipeline
            self.device   = dai.Device(pipeline)
            self.queue    = self.device.getOutputQueue(
                name='rgb', maxSize=4, blocking=False)

            self.get_logger().info('OAK-D Lite bağlantısı kuruldu.')

        except ImportError:
            self.get_logger().error(
                'DepthAI kurulu değil! '
                'Jetson\'da: pip3 install depthai')
            self.queue = None
        except Exception as e:
            self.get_logger().error(f'OAK-D Lite bağlantı hatası: {e}')
            self.queue = None

    def _capture_loop(self):
        """Sürekli kare yakala ve publish et.

        tryGet() (bloklamayan) kullanılır - depthai'nin bloklayan get()'i
        USB kopmasi/donmasinda thread'i sonsuza kadar askida birakabilirdi,
        hicbir log/yeniden-baglanma olmadan (F-S.7).

        🔴 18.08.2026 — YENİDEN BAĞLANMA EKLENDİ. Eskiden `self.queue is None`
        (ilk bağlantı başarısız — soğuk açılışta USB enumeration yarışı çok
        yaygın — ya da aşağıdaki except'te cihaz koptuğu tespit edilip
        `queue=None` yapıldıktan sonra) hiçbir zaman `_init_depthai()`'a
        TEKRAR gidilmiyordu: düğüm sonsuza dek "1s uyu, tekrar kontrol et"
        döngüsünde kilitli kalıyordu — kamera bir daha asla geri gelmiyordu,
        görevin geri kalanında körlük demekti. Şimdi boş kuyrukta periyodik
        (`_RECONNECT_PERIOD_S`) yeniden deneme var; `_init_depthai()` zaten
        kendi hata yakalamasını yapıyor (bağlanamazsa sessizce queue=None
        bırakır), o yüzden burada ekstra try/except gerekmiyor.
        """
        last_frame_time = time.monotonic()
        last_reconnect_attempt = time.monotonic()
        while self.running:
            if self.queue is None:
                now = time.monotonic()
                if now - last_reconnect_attempt >= self._RECONNECT_PERIOD_S:
                    last_reconnect_attempt = now
                    self.get_logger().warn(
                        'OAK-D bağlı değil — yeniden bağlanma deneniyor...')
                    self._init_depthai()
                    if self.queue is not None:
                        last_frame_time = time.monotonic()
                        self.get_logger().info('OAK-D yeniden bağlandı.')
                time.sleep(0.2)
                continue

            try:
                in_rgb = self.queue.tryGet()
                if in_rgb is None:
                    if time.monotonic() - last_frame_time > 5.0:
                        self.get_logger().error(
                            'OAK-D 5s\'tir kare vermiyor (USB/cihaz sorunu '
                            'olabilir)', throttle_duration_sec=5.0)
                    time.sleep(0.01)
                    continue
                last_frame_time = time.monotonic()
                frame = in_rgb.getCvFrame()

                msg = Image()
                msg.header.stamp    = self.get_clock().now().to_msg()
                msg.header.frame_id = 'camera'
                msg.height   = frame.shape[0]
                msg.width    = frame.shape[1]
                msg.encoding = 'bgr8'
                msg.step     = frame.shape[1] * 3
                msg.data     = frame.tobytes()

                self.img_pub.publish(msg)

            except Exception as e:
                # Bu noktada tryGet()/getCvFrame() patladıysa cihaz bağlantısı
                # muhtemelen koptu (XLink hatası vb.) — aynı bozuk queue'yu
                # sonsuza dek yeniden denemek yerine kapat + queue=None yap ki
                # yukarıdaki dal yeniden bağlanmayı denesin.
                self.get_logger().error(
                    f'Kare yakalama hatası: {e}',
                    throttle_duration_sec=5.0)
                if self.device:
                    try:
                        self.device.close()
                    except Exception:
                        pass
                self.device = None
                self.queue = None

    def destroy_node(self):
        self.running = False
        if self.device:
            self.device.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = OakdDriverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass  # launch/systemd SIGINT'i normal kapanıştır (traceback basma)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
