#!/usr/bin/env python3
"""
IDA/Girdap USV - Kamera Kayıt Node
=====================================
Şartname zorunlu MP4 kaydı:
  - İşlenmiş kamera görüntüsü (YOLO bbox overlay ile)
  - Minimum 1Hz (gerçekte kamera FPS'inde kaydeder)
  - ~/girdap_logs/kamera/ klasörüne MP4 kaydeder (output_dir parametresi)

Kaynak topic'ler:
  /camera/image_raw              → ham kamera görüntüsü
  /perception/orange_buoys       → turuncu duba bbox'ları
  /perception/yellow_buoys       → sarı duba bbox'ları

Yazar: IDA/Girdap Takım 989124 - Alt Alan B
"""

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray

import cv2
import numpy as np
import os
from datetime import datetime

# 🔴 2026-08-04 (algı ekibi) — F-S.3 kapandı.
# Şartname md 4.2 Dosya-1: "Tespit ve takip işlemleri sonucunda obje çerçeve
# çizimleri ve **yapıldıysa tespit sınıf bilgileri görünecek** şekilde".
# Bu node iki AYRI topic'ten (orange/yellow) besleniyordu ve etiketi topic'ten
# türetiyordu. Ama hardware.launch.py tek-topic'li üreticiyi
# (/perception/buoys, class_id'yi mesajın İÇİNDE taşır) orange_buoys'a
# remap ediyor → sarı/hedef/bilinmeyen dâhil HER ŞEY "TURUNCU DUBA" yazılıyordu,
# yani teslim edilen mp4'te sınıf bilgisi YANLIŞ görünüyordu.
# Çözüm: class_id varsa ONDAN, yoksa (ida_topics'in kendi perception_node'u
# `det.results`'ı hiç doldurmuyor) topic varsayılanından etiketle.
# Sözleşme: "0"=kenar(turuncu) "1"=engel(sarı) "2"=hedef "3"/"4"/"5"=kırmızı/
# yeşil/siyah, "99"=eşleşmemiş (füzyon CLASS_UNKNOWN).
SINIF_ETIKET = {
    0: ('KENAR DUBASI', (0, 127, 255)),
    1: ('ENGEL DUBASI', (0, 255, 255)),
    2: ('HEDEF', (255, 0, 255)),
    3: ('KIRMIZI', (0, 0, 255)),
    4: ('YESIL', (0, 200, 0)),
    # 18.08.2026: sinif 5 KAHVERENGI -> SIYAH (RAL 9005, sartname s.18).
    # Etiket teslim edilen mp4'un ustune basiliyor; yanlis ad hakemde
    # yanlis sinif izlenimi birakirdi (ayni satirin daha once duzeltilmis
    # bir surumu bu dosyanin ust notunda anlatiliyor).
    5: ('SIYAH', (40, 40, 40)),
    99: ('BILINMIYOR', (200, 200, 200)),
}


def etiket_ve_renk(det, varsayilan):
    """Detection2D → (etiket, BGR renk). class_id yoksa varsayılana düşer.

    Savunmacı: results boş olabilir, class_id sayısal olmayabilir (şema serbest
    metin taşıyabilir) — hiçbir durumda kayıt döngüsünü patlatmamalı.
    """
    try:
        if det.results:
            return SINIF_ETIKET[int(det.results[0].hypothesis.class_id)]
    except (ValueError, KeyError, AttributeError, IndexError):
        pass
    return varsayilan


class KameraKayitNode(Node):
    def __init__(self):
        super().__init__('kamera_kayit_node')
        self.cb_group = ReentrantCallbackGroup()

        # ── Parametreler ──────────────────────────────────────────────────────
        self.declare_parameter('fps', 10)
        # 2026-07-17: oakd_driver_node'un çözünürlüğüyle EŞLEŞMELİ (1280x720,
        # bkz. o dosyanın docstring'i — 640x480'de sahada 2m'deki duba bile
        # net görülemiyordu). Uyuşmazsa VideoWriter gelen kareyi bozuk/hatalı
        # yazar ya da hiç yazmaz.
        self.declare_parameter('width', 1280)
        self.declare_parameter('height', 720)

        self.fps    = self.get_parameter('fps').value
        self.width  = self.get_parameter('width').value
        self.height = self.get_parameter('height').value

        # Şartname 4.2 Dosya-1 teslim dosyası — /tmp KULLANMA: tmpfs'te
        # reboot/güç kesintisinde kaybolur (dosya başı 5 ceza puanı).
        # telemetry_node ~/girdap_logs deseniyle aynı.
        self.declare_parameter('output_dir', '')

        # ── Durum değişkenleri ────────────────────────────────────────────────
        self.latest_frame      = None
        self.orange_detections = []
        self.yellow_detections = []
        self.frame_count       = 0

        self.output_dir = self.get_parameter('output_dir').value or \
            os.path.expanduser('~/girdap_logs/kamera')

        # ── Video yazıcı ──────────────────────────────────────────────────────
        os.makedirs(self.output_dir, exist_ok=True)
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        self.video_path = os.path.join(
            self.output_dir, f'kamera_{timestamp}.mp4')

        fourcc = cv2.VideoWriter_fourcc(*'mp4v')
        self.writer = cv2.VideoWriter(
            self.video_path, fourcc, self.fps,
            (self.width, self.height))

        # F-P.11 (robustness taraması, 2026-07-15): VideoWriter'ın açılıp
        # açılmadığı hiç kontrol edilmiyordu — codec (mp4v) kullanılamazsa ya
        # da output_dir yazılamazsa write() sessizce hiçbir şey yapmaz,
        # Dosya-1 (Şartname 4.2, zorunlu) görev bitene kadar fark edilmeyen
        # BOŞ bir mp4 olurdu.
        if not self.writer.isOpened():
            self.get_logger().error(
                f'VideoWriter AÇILAMADI ({self.video_path}, codec=mp4v) — '
                'Dosya-1 kaydı BAŞLAMAYACAK, kodek/dizin izinlerini kontrol et'
            )

        self.get_logger().info(f'Kamera kaydı başlatıldı: {self.video_path}')

        # ── QoS ───────────────────────────────────────────────────────────────
        qos_reliable = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10)

        qos_best = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10)

        # ── Subscribers ───────────────────────────────────────────────────────
        self.create_subscription(
            Image, '/camera/image_raw',
            self._image_cb, qos_reliable,
            callback_group=self.cb_group)

        self.create_subscription(
            Detection2DArray, '/perception/orange_buoys',
            self._orange_cb, qos_reliable,
            callback_group=self.cb_group)

        self.create_subscription(
            Detection2DArray, '/perception/yellow_buoys',
            self._yellow_cb, qos_reliable,
            callback_group=self.cb_group)

        # ── Timer ─────────────────────────────────────────────────────────────
        self.create_timer(1.0 / self.fps, self._yaz,
                          callback_group=self.cb_group)

        self.get_logger().info(
            f'Kamera Kayıt Node başlatıldı ({self.fps}fps, {self.width}x{self.height})')

    # ── Callback'ler ──────────────────────────────────────────────────────────

    def _image_cb(self, msg: Image):
        """Ham kamera görüntüsünü numpy array'e çevir."""
        try:
            # ROS Image → numpy
            if msg.encoding == 'rgb8':
                frame = np.frombuffer(msg.data, dtype=np.uint8).reshape(
                    msg.height, msg.width, 3)
                self.latest_frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
            elif msg.encoding == 'bgr8':
                self.latest_frame = np.frombuffer(msg.data, dtype=np.uint8).reshape(
                    msg.height, msg.width, 3)
            else:
                self.get_logger().warn(
                    f'Desteklenmeyen encoding: {msg.encoding}',
                    throttle_duration_sec=5.0)
        except Exception as e:
            self.get_logger().error(f'Image callback hatası: {e}')

    def _orange_cb(self, msg: Detection2DArray):
        """Turuncu duba tespitlerini sakla."""
        self.orange_detections = msg.detections

    def _yellow_cb(self, msg: Detection2DArray):
        """Sarı duba tespitlerini sakla."""
        self.yellow_detections = msg.detections

    # ── Video yazma ───────────────────────────────────────────────────────────

    def _yaz(self):
        """Frame'i bbox overlay ile video'ya yaz."""
        # F-P.11 (robustness taraması, 2026-07-15): bu timer callback'inde
        # try/except YOKTU (_image_cb'de var) — bbox alanlarında NaN/inf
        # (int(float('nan')) → ValueError) ya da bozuk bir frame boyutu
        # (cv2.resize hatası) node'u KALICI öldürürdü, Şartname 4.2 zorunlu
        # Dosya-1 kaydı mp4 tamamlanmadan sessizce dururdu.
        try:
            if self.latest_frame is None:
                # Kamera görüntüsü yoksa siyah frame yaz
                frame = np.zeros((self.height, self.width, 3), dtype=np.uint8)
                cv2.putText(frame, 'Kamera Bekleniyor...', (20, 50),
                            cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 255), 2)
            else:
                frame = self.latest_frame.copy()
                # Boyut uyumu
                if frame.shape[:2] != (self.height, self.width):
                    frame = cv2.resize(frame, (self.width, self.height))

            # ── Turuncu duba bbox'ları ─────────────────────────────────────
            for det in self.orange_detections:
                cx = int(det.bbox.center.position.x)
                cy = int(det.bbox.center.position.y)
                w  = int(det.bbox.size_x)
                h  = int(det.bbox.size_y)
                x1, y1 = cx - w//2, cy - h//2
                x2, y2 = cx + w//2, cy + h//2
                etiket, renk = etiket_ve_renk(det, ('TURUNCU DUBA', (0, 127, 255)))
                cv2.rectangle(frame, (x1, y1), (x2, y2), renk, 2)
                cv2.putText(frame, etiket, (x1, y1-5),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, renk, 1)

            # ── Sarı duba bbox'ları ────────────────────────────────────────
            for det in self.yellow_detections:
                cx = int(det.bbox.center.position.x)
                cy = int(det.bbox.center.position.y)
                w  = int(det.bbox.size_x)
                h  = int(det.bbox.size_y)
                x1, y1 = cx - w//2, cy - h//2
                x2, y2 = cx + w//2, cy + h//2
                etiket, renk = etiket_ve_renk(det, ('SARI DUBA', (0, 255, 255)))
                cv2.rectangle(frame, (x1, y1), (x2, y2), renk, 2)
                cv2.putText(frame, etiket, (x1, y1-5),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, renk, 1)

            # ── Zaman damgası overlay ────────────────────────────────────────
            zaman = datetime.now().strftime('%Y-%m-%d %H:%M:%S')
            cv2.putText(frame, f'IDA/Girdap USV | {zaman}', (10, self.height-10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

            # ── Frame sayacı ─────────────────────────────────────────────────
            self.frame_count += 1
            cv2.putText(frame, f'Frame: {self.frame_count}', (10, 25),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

            self.writer.write(frame)
        except Exception as e:
            self.get_logger().error(
                f'Video yazma hatası, bu frame atlandı: {e}',
                throttle_duration_sec=5.0)

    def destroy_node(self):
        """Node kapatılırken video'yu kaydet."""
        self.writer.release()
        self.get_logger().info(
            f'Video kaydı tamamlandı: {self.video_path} ({self.frame_count} frame)')
        super().destroy_node()


def _sigterm_kapanisi_kur():
    """SIGTERM → KeyboardInterrupt, yani AŞAĞIDAKİ `finally` çalışsın.

    🔴 15.08.2026 arızası: `systemctl stop/restart` **SIGTERM** gönderir ve
    Python bunu işlemezse süreci ANINDA sonlandırır → `finally` çalışmaz →
    `writer.release()` çağrılmaz → mp4'ün `moov` atomu yazılmaz → dosya
    **oynatılamaz** (`ffprobe: moov atom not found`). 14.08'in üç teslim
    dosyası da (kamera, LiDAR, yerel harita) tam bu yüzden bozuktu.
    rclpy yalnız SIGINT için işleyici kurar.

    ⚠ Bu, `girdap_decision/sigterm_kapanis.py`'nin İKİZİDİR — `ida_topics`
    ayrı bir ROS paketi olduğu ve ona bağımlılık eklemek istemediğimiz için
    yerinde tutuluyor. Biri değişirse öteki de değişmeli.
    """
    import signal

    def _isleyici(signum, frame):
        raise KeyboardInterrupt

    try:
        signal.signal(signal.SIGTERM, _isleyici)
    except (ValueError, OSError):
        pass


def main(args=None):
    rclpy.init(args=args)
    _sigterm_kapanisi_kur()
    node = KameraKayitNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass  # launch/systemd SIGINT'i normal kapanıştır (traceback basma)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
