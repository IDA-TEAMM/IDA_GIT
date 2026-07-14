"""
Girdap İDA — Kamera duba tespiti node'u (Layer 2, Sprint 2).

/oak/rgb/image_raw frame'ini ROS-bağımsız çekirdekten (prototype.perception.
camera_buoys) geçirip vision_msgs/Detection2DArray yayınlar. Kaynak-bağımsız
(replaceable design): topic adı sabit, arkasındaki üretici (gerçek OAK-D
sürücüsü / sentetik / rosbag) değişebilir.

Image → numpy dönüşümü girdap_decision.image_codec ile (cv_bridge DEĞİL —
apt cv_bridge numpy 2.x ABI'siyle kırık, gerekçe image_codec docstring'inde).

YOLO katmanı (hedef sınıfı, class 2) MOCK modda (gerçek .pt yok — sabit test
bbox'ı döner). Gerçek model gelince yolo_model_path parametresi verilir; kod
yolu aynı kalır (ultralytics lazy import — mock modda hiç yüklenmez).

F-S.9: turuncu/sarı (class 0/1) için ikinci bir ALTERNATİF yol — eğitilmiş
genel duba lokalizatörü (ör. ida_topics/best.pt, ida_topics/perception_node.py
ile aynı model) + BU node'un ayarlanmış HSV eşikleriyle sınıflandırma
(ida_topics'in kanıtlanmış "YOLO bulur, HSV sınıflar" deseni + girdap'ın
tune edilmiş renk eşiklerinin birleşimi). `use_yolo_localizer:=true` +
`yolo_localizer_model_path` ile açılır; varsayılan kapalı = mevcut saf-HSV
segmentasyonu hiç değişmez.

Sınıf sözleşmesi (class_id string olarak yayınlanır — vision_msgs şeması):
    "0" = parkur_kenari (turuncu)   "1" = engel (sarı)   "2" = hedef (Parkur-3)

Subscribed:
    /oak/rgb/image_raw   sensor_msgs/Image           (SensorDataQoS)
Published:
    /perception/buoys    vision_msgs/Detection2DArray (default RELIABLE)
Header: kaynak frame_id + stamp KORUNUR (bbox'lar görüntü pikseli uzayında).
"""

from __future__ import annotations

from typing import Optional

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from vision_msgs.msg import (
    Detection2D,
    Detection2DArray,
    ObjectHypothesisWithPose,
)

from girdap_decision.image_codec import imgmsg_to_bgr
from girdap_decision.qos_profiles import sensor_data_qos
from prototype.perception.camera_buoys import (
    BuoyLocalizer,
    CameraBuoyConfig,
    Detection,
    YoloInference,
    detect_buoys,
)


class PerceptionCameraNode(Node):
    """Image → CLAHE/HSV segmentasyon (+mock YOLO) → Detection2DArray."""

    def __init__(self) -> None:
        super().__init__("perception_camera_node")

        # --- Parametreler (config perception.camera bloğu) ---
        self.declare_parameter("clahe_clip_limit", 2.0)
        self.declare_parameter("clahe_tile", 8)
        self.declare_parameter("min_area_px", 150)
        self.declare_parameter("morph_kernel_px", 5)
        self.declare_parameter("use_yolo", False)
        self.declare_parameter("yolo_model_path", "")   # boş = mock
        # F-S.9: turuncu/sarı için ALTERNATİF yol — eğitilmiş genel duba
        # lokalizatörü (ör. ida_topics/best.pt) + BU node'un HSV eşikleri.
        # Varsayılan False → mevcut saf-HSV segmentasyonu değişmez.
        self.declare_parameter("use_yolo_localizer", False)
        self.declare_parameter("yolo_localizer_model_path", "")  # boş = mock
        self.declare_parameter("yolo_localizer_min_coverage", 0.15)
        self.declare_parameter("log_period_s", 5.0)
        # HSV aralıkları dizi param — yalnız params.yaml'dan (launch-arg değil)
        self.declare_parameter("hsv_orange_lo", [5, 120, 120])
        self.declare_parameter("hsv_orange_hi", [20, 255, 255])
        self.declare_parameter("hsv_yellow_lo", [21, 120, 120])
        self.declare_parameter("hsv_yellow_hi", [35, 255, 255])

        p = self.get_parameter
        self._cfg = CameraBuoyConfig(
            hsv_orange_lo=tuple(p("hsv_orange_lo").value),
            hsv_orange_hi=tuple(p("hsv_orange_hi").value),
            hsv_yellow_lo=tuple(p("hsv_yellow_lo").value),
            hsv_yellow_hi=tuple(p("hsv_yellow_hi").value),
            clahe_clip_limit=float(p("clahe_clip_limit").value),
            clahe_tile=int(p("clahe_tile").value),
            min_area_px=int(p("min_area_px").value),
            morph_kernel_px=int(p("morph_kernel_px").value),
            use_yolo=bool(p("use_yolo").value),
            yolo_model_path=str(p("yolo_model_path").value),
            use_yolo_localizer=bool(p("use_yolo_localizer").value),
            yolo_localizer_model_path=str(p("yolo_localizer_model_path").value),
            yolo_localizer_min_coverage=float(
                p("yolo_localizer_min_coverage").value
            ),
        )
        # .pt yolu boşsa mock — gerçek model geldiğinde yalnız parametre değişir.
        self._yolo = YoloInference(
            model_path=self._cfg.yolo_model_path,
            mock=not self._cfg.yolo_model_path,
        )
        # F-S.9: turuncu/sarı lokalizatörü (ida_topics/best.pt gibi bir genel
        # duba modeli) — hedef sınıfı YoloInference'dan bağımsız, ayrı model.
        self._localizer = BuoyLocalizer(
            model_path=self._cfg.yolo_localizer_model_path,
            mock=not self._cfg.yolo_localizer_model_path,
        )
        self._log_period_s = float(p("log_period_s").value)
        self._last_log_t: Optional[float] = None

        # --- I/O ---
        self._pub = self.create_publisher(
            Detection2DArray, "/perception/buoys", 10
        )
        self._sub = self.create_subscription(
            Image, "/oak/rgb/image_raw", self._on_image, sensor_data_qos()
        )

        yolo_mode = "mock" if self._yolo.is_mock else self._cfg.yolo_model_path
        loc_mode = (
            "mock" if self._localizer.is_mock
            else self._cfg.yolo_localizer_model_path
        )
        self.get_logger().info(
            "perception_camera_node aktif: /oak/rgb/image_raw → "
            "/perception/buoys "
            f"(min_area={self._cfg.min_area_px} px, "
            f"clahe={self._cfg.clahe_clip_limit}, "
            f"hedef_yolo={'AÇIK:' + yolo_mode if self._cfg.use_yolo else 'kapalı'}, "
            f"turuncu/sarı="
            f"{'YOLO-lokalizatör:' + loc_mode if self._cfg.use_yolo_localizer else 'HSV'})"
        )

    # ------------------------------------------------------------- callback

    def _on_image(self, msg: Image) -> None:
        frame = imgmsg_to_bgr(msg)                 # cv_bridge'siz (docstring)
        detections = detect_buoys(
            frame, self._cfg, self._yolo, self._localizer
        )
        self._pub.publish(self._to_msg(detections, msg))

        self.get_logger().debug(f"{len(detections)} duba tespiti")
        self._periodic_info(len(detections))

    def _to_msg(
        self, detections: list[Detection], src: Image
    ) -> Detection2DArray:
        out = Detection2DArray()
        out.header = src.header                     # frame_id + stamp korunur
        for det in detections:
            d = Detection2D()
            d.header = src.header
            d.bbox.center.position.x = det.center_x
            d.bbox.center.position.y = det.center_y
            d.bbox.size_x = det.width
            d.bbox.size_y = det.height
            hyp = ObjectHypothesisWithPose()
            hyp.hypothesis.class_id = str(det.class_id)
            hyp.hypothesis.score = det.score
            d.results.append(hyp)
            out.detections.append(d)
        return out

    def _periodic_info(self, n_detections: int) -> None:
        """log_period_s'de bir INFO — her frame'de log seli olmasın."""
        now = self.get_clock().now().nanoseconds * 1e-9
        if self._last_log_t is None or now - self._last_log_t >= self._log_period_s:
            self._last_log_t = now
            self.get_logger().info(f"tespit: {n_detections} duba")


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node = PerceptionCameraNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
