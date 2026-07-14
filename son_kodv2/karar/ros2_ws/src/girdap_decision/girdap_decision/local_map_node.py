"""
Girdap İDA — Yerel harita PNG dumper node'u (Layer 2).

Şartname 4.2 — Dosya 3:
    Lokal harita / cost map / engel haritası, ≥1 Hz, png seri. Görev bitiminden
    20 dk içinde teslim; her gecikmiş dosya 5 ceza puanı.

planning_node'un yayınladığı `/girdap/map/local` (OccupancyGrid) mesajını 1 Hz'te
grayscale PNG serisine döker. Dönüşüm + oturum dizini mantığı ROS-bağımsız
prototype.mapping.local_map.LocalMapDumper'da; bu node yalnız son mesajı tutup
timer'da diske yazar. Test (test_local_map.py) aynı çekirdeği kullanır.

Subscribed:
    /girdap/map/local   nav_msgs/OccupancyGrid   (SensorDataQoS)
Çıktı:
    ~/girdap_logs/local_map/session_YYYYMMDD_HHMMSS/frame_00000.png ...
"""

from __future__ import annotations

from typing import Optional

import rclpy
from rclpy.node import Node

from nav_msgs.msg import OccupancyGrid

from girdap_decision.qos_profiles import sensor_data_qos
from prototype.mapping.local_map import LocalMapDumper


class LocalMapNode(Node):
    """OccupancyGrid → grayscale PNG serisi (Şartname 4.2 Dosya-3)."""

    def __init__(self) -> None:
        super().__init__("local_map_node")

        # --- Parametreler ---
        self.declare_parameter("dump_rate_hz", 1.0)     # ≥1 Hz şartname garantisi
        self.declare_parameter("output_dir", "")        # boş → ~/girdap_logs/...

        out = str(self.get_parameter("output_dir").value) or None
        self._dumper = LocalMapDumper(base_dir=out)

        self._last: Optional[OccupancyGrid] = None

        # --- Subscriber (map publisher BEST_EFFORT → SensorDataQoS) ---
        self._sub = self.create_subscription(
            OccupancyGrid, "/girdap/map/local", self._on_map, sensor_data_qos()
        )

        # --- 1 Hz dump timer'ı ---
        rate = float(self.get_parameter("dump_rate_hz").value)
        self._timer = self.create_timer(1.0 / rate, self._on_tick)

        self.get_logger().info(
            f"local_map_node aktif → {self._dumper.session_dir} "
            f"(dump={rate} Hz)"
        )

    def _on_map(self, msg: OccupancyGrid) -> None:
        self._last = msg

    def _on_tick(self) -> None:
        if self._last is None:                          # henüz harita gelmedi
            return
        m = self._last
        try:
            path = self._dumper.write_frame(
                list(m.data), m.info.width, m.info.height
            )
        except ValueError as e:
            self.get_logger().error(
                f"Bozuk OccupancyGrid, kare atlandı: {e}",
                throttle_duration_sec=5.0)
            return
        if path is None:
            self.get_logger().error(
                "Dosya-3 PNG yazma hatası (disk dolu olabilir) — kare atlandı",
                throttle_duration_sec=5.0)
            return
        if self._dumper.frame_count % 10 == 1:          # her ~10 karede bir log
            self.get_logger().info(
                f"[Dosya-3] {self._dumper.frame_count} kare → {path.name}"
            )


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node = LocalMapNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
