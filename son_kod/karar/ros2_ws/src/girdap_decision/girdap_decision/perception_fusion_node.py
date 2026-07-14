"""
Girdap İDA — Kamera-LiDAR bearing füzyonu node'u (Layer 2, Sprint 3).

/perception/obstacle_map (LiDAR, 3D konum + yarıçap, renksiz) ile
/perception/buoys (kamera, 2D bbox + renk) tespitlerini zaman-senkronize edip
ROS-bağımsız çekirdekten (prototype.perception.fusion) geçirir; sonucu
/perception/classified_obstacles (vision_msgs/Detection3DArray) yayınlar.

Zaman senkronizasyonu: message_filters.ApproximateTimeSynchronizer (slop
config'den). QoS bilerek RELIABLE/depth=10 (message_filters varsayılanı) —
her iki kaynak publisher da (perception_lidar_node, perception_camera_node)
aynı varsayılanla yayınlıyor; sensor_data_qos (BEST_EFFORT) burada KULLANMA,
QoS uyuşmazlığı mesajları sessizce düşürür.

⚠ Bearing-only association (kalibrasyon yok) — bkz. prototype.perception.
fusion modül docstring'i. camera_image_width_px/height_px, Detection2D'nin
piksel-uzayı bbox'ını normalize etmek için kullanılan GEÇİCİ sabit (OAK-D
Lite preview çözünürlüğü); gerçek CameraInfo entegrasyonu Sprint 4+.

⚠ STAMP SÖZLEŞMESİ (F7.1): ApproximateTimeSynchronizer iki topic'in header
stamp'lerini eşler; İKİ ÜRETİCİ DE AYNI zaman tabanını kullanmalı. Mevcut
durum: buoys = OAK node'unun yayın-anı saati; obstacle_map = Livox sürücü
stamp'i (config'e göre sistem saati ya da sensör zamanı olabilir). Sapma >
sync_slop_s ise eşleşme HİÇ oluşmaz. Bu yüzden `_sync_watchdog` var: iki
girdi de akarken sync `sync_watchdog_s` boyunca hiç ateşlemediyse WARN basar
(sahada sessiz ölüm yerine görünür arıza). Kalıcı çözüm: Livox sürücüsünü
sistem saatine ayarla ya da slop'u ölçülen sapmaya göre büyüt.

Subscribed:
    /perception/obstacle_map   geometry_msgs/PoseArray        (RELIABLE)
    /perception/buoys          vision_msgs/Detection2DArray   (RELIABLE)
Published:
    /perception/classified_obstacles   vision_msgs/Detection3DArray
Header: frame_id=base_link, stamp = LiDAR mesajının stamp'i (iki kaynaktan
    birini seçmek gerekiyor — 3D konumun kaynağı olduğu için LiDAR baz alınır).
"""

from __future__ import annotations

from typing import Optional

import message_filters
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseArray
from vision_msgs.msg import (
    Detection2DArray,
    Detection3D,
    Detection3DArray,
    ObjectHypothesisWithPose,
)

from prototype.perception.fusion import (
    CameraDetection,
    FusionConfig,
    LidarDetection,
    associate,
)


class PerceptionFusionNode(Node):
    """PoseArray + Detection2DArray (sync) → Detection3DArray (bearing fusion)."""

    def __init__(self) -> None:
        super().__init__("perception_fusion_node")

        # --- Parametreler (config/hardware.yaml perception.fusion bloğu) ---
        self.declare_parameter("bearing_tolerance_rad", 0.15)
        self.declare_parameter("camera_hfov_rad", 1.2)
        self.declare_parameter("camera_image_width_px", 640)
        self.declare_parameter("camera_image_height_px", 480)
        self.declare_parameter("sync_slop_s", 0.1)
        self.declare_parameter("log_period_s", 5.0)

        p = self.get_parameter
        self._cfg = FusionConfig(
            bearing_tolerance_rad=float(p("bearing_tolerance_rad").value),
            camera_hfov_rad=float(p("camera_hfov_rad").value),
        )
        self._image_w = int(p("camera_image_width_px").value)
        self._image_h = int(p("camera_image_height_px").value)
        self._log_period_s = float(p("log_period_s").value)
        self._last_log_t: Optional[float] = None

        # --- I/O ---
        self._pub = self.create_publisher(
            Detection3DArray, "/perception/classified_obstacles", 10
        )
        # message_filters varsayılan QoS (RELIABLE, depth=10) — publisher'larla
        # bilerek eşleşiyor (docstring). sensor_data_qos KULLANMA.
        self._lidar_sub = message_filters.Subscriber(
            self, PoseArray, "/perception/obstacle_map"
        )
        self._camera_sub = message_filters.Subscriber(
            self, Detection2DArray, "/perception/buoys"
        )
        self._sync = message_filters.ApproximateTimeSynchronizer(
            [self._lidar_sub, self._camera_sub],
            queue_size=10,
            slop=float(p("sync_slop_s").value),
        )
        self._sync.registerCallback(self._on_sync)

        # F7.1: sync bekçisi — iki girdi de akarken eşleşme hiç oluşmuyorsa
        # (stamp tabanları uyumsuz / slop dar) sessiz kalmak yerine WARN bas.
        self.declare_parameter("sync_watchdog_s", 10.0)
        self._n_lidar_in = 0
        self._n_camera_in = 0
        self._n_sync = 0
        self._lidar_sub.registerCallback(self._count_lidar)
        self._camera_sub.registerCallback(self._count_camera)
        wd = float(self.get_parameter("sync_watchdog_s").value)
        self._watchdog_timer = self.create_timer(wd, self._on_sync_watchdog)

        self.get_logger().info(
            "perception_fusion_node aktif: obstacle_map + buoys → "
            "classified_obstacles "
            f"(bearing_tol={self._cfg.bearing_tolerance_rad} rad, "
            f"hfov={self._cfg.camera_hfov_rad} rad, "
            f"slop={p('sync_slop_s').value} s)"
        )

    # ------------------------------------------------------------- watchdog

    def _count_lidar(self, _msg) -> None:            # noqa: ANN001
        self._n_lidar_in += 1

    def _count_camera(self, _msg) -> None:           # noqa: ANN001
        self._n_camera_in += 1

    def _on_sync_watchdog(self) -> None:
        """Pencere içinde iki girdi de aktı ama sync hiç ateşlemediyse WARN."""
        if self._n_lidar_in > 0 and self._n_camera_in > 0 and self._n_sync == 0:
            self.get_logger().warn(
                f"sync bekçisi: {self._n_lidar_in} lidar + "
                f"{self._n_camera_in} kamera mesajı geldi ama eşleşme SIFIR — "
                "stamp tabanları uyumsuz olabilir (docstring: STAMP SÖZLEŞMESİ); "
                "sync_slop_s'i büyütmeyi ya da Livox saat kaynağını denetle"
            )
        self._n_lidar_in = self._n_camera_in = self._n_sync = 0

    # ------------------------------------------------------------- callback

    def _on_sync(self, poses: PoseArray, detections: Detection2DArray) -> None:
        self._n_sync += 1
        lidar_list = [
            LidarDetection(x=pose.position.x, y=pose.position.y,
                            radius=abs(pose.orientation.z))
            for pose in poses.poses
        ]
        camera_list = [
            cam
            for det in detections.detections
            if (cam := self._to_camera_detection(det)) is not None
        ]
        fused = associate(lidar_list, camera_list, self._cfg)

        out = Detection3DArray()
        out.header.stamp = poses.header.stamp     # LiDAR referans stamp'i
        out.header.frame_id = "base_link"
        for obs in fused:
            d = Detection3D()
            d.header = out.header
            d.bbox.center.position.x = obs.x
            d.bbox.center.position.y = obs.y
            d.bbox.center.position.z = 0.0
            d.bbox.center.orientation.w = 1.0      # gerçek yönelim yok — kimlik
            d.bbox.size.x = obs.radius * 2.0        # yaklaşık çap (x/y)
            d.bbox.size.y = obs.radius * 2.0
            d.bbox.size.z = 0.0                     # yükseklik bilinmiyor (2D LiDAR cluster)
            hyp = ObjectHypothesisWithPose()
            hyp.hypothesis.class_id = str(obs.class_id)
            hyp.hypothesis.score = obs.score
            d.results.append(hyp)
            out.detections.append(d)
        self._pub.publish(out)

        n_matched = sum(o.matched for o in fused)
        self.get_logger().debug(
            f"{len(lidar_list)} lidar + {len(camera_list)} kamera → "
            f"{len(fused)} füzyon ({n_matched} eşleşti)"
        )
        self._periodic_info(len(fused), n_matched)

    def _to_camera_detection(self, det) -> Optional[CameraDetection]:      # noqa: ANN001
        """Detection2D (piksel bbox) → CameraDetection (normalize [0,1])."""
        if not det.results:                        # savunmacı — olmaması gerekir
            return None
        hyp = det.results[0].hypothesis
        # F7.2: sözleşme class_id="0"/"1"/"2" der ama alan serbest metin
        # taşıyabilir; int() ValueError'ı callback'ten sızarsa node ÖLÜR.
        try:
            class_id = int(hyp.class_id)
        except ValueError:
            self.get_logger().warn(
                f"sayısal olmayan class_id atlandı: {hyp.class_id!r}"
            )
            return None
        return CameraDetection(
            bbox_cx=det.bbox.center.position.x / self._image_w,
            bbox_cy=det.bbox.center.position.y / self._image_h,
            class_id=class_id,
            score=float(hyp.score),
        )

    def _periodic_info(self, n_fused: int, n_matched: int) -> None:
        """log_period_s'de bir INFO — her sync callback'te log seli olmasın."""
        now = self.get_clock().now().nanoseconds * 1e-9
        if self._last_log_t is None or now - self._last_log_t >= self._log_period_s:
            self._last_log_t = now
            self.get_logger().info(
                f"füzyon: {n_fused} engel ({n_matched} eşleşti, "
                f"{n_fused - n_matched} bilinmiyor)"
            )


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node = PerceptionFusionNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
