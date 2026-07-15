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

    def __init__(self, **node_kwargs) -> None:
        # node_kwargs → parameter_overrides passthrough (test enjeksiyonu,
        # diğer node'larla tutarlı — F-P.17 testi bunu gerektirir).
        super().__init__("local_map_node", **node_kwargs)

        # --- Parametreler ---
        self.declare_parameter("dump_rate_hz", 1.0)     # ≥1 Hz şartname garantisi
        self.declare_parameter("output_dir", "")        # boş → ~/girdap_logs/...
        # F-P.17 (robustness taraması, 2026-07-15): /girdap/map/local kesilirse
        # (planning_node çökerse) _on_tick yalnız `_last is None` kontrolü
        # yapıyordu — HİÇ tazelik kontrolü yoktu. Aynı donmuş kareyi sonsuza
        # dek yeni dosya adlarıyla yazmaya devam ederdi: Dosya-3 (Şartname 4.2,
        # zorunlu, 5 ceza puanı) canlı gibi görünür ama gerçekte donmuş olurdu.
        # Frame formatı DEĞİŞTİRİLMEZ (teslim sözleşmesi) — yalnız operatörü
        # sesli uyarır.
        self.declare_parameter("map_timeout_s", 3.0)
        self._map_timeout = float(self.get_parameter("map_timeout_s").value)
        self._last_map_t: Optional[float] = None
        self._stale_warned = False

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
        self._last_map_t = self._now()
        if self._stale_warned:
            self._stale_warned = False
            self.get_logger().info("/girdap/map/local yayını geri geldi")

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _map_stale(self) -> bool:
        """F-P.17: son harita `map_timeout_s`'ten eski mi? Hiç gelmediyse
        False (boot gürültüsü, diğer F-P bekçileriyle aynı ilke)."""
        if self._map_timeout <= 0.0 or self._last_map_t is None:
            return False
        return (self._now() - self._last_map_t) > self._map_timeout

    def _on_tick(self) -> None:
        if self._last is None:                          # henüz harita gelmedi
            return
        if self._map_stale():
            if not self._stale_warned:
                self._stale_warned = True
                age = self._now() - (self._last_map_t or self._now())
                self.get_logger().error(
                    f"/girdap/map/local {age:.1f}s'dir gelmiyor — Dosya-3 "
                    "AYNI DONMUŞ kareyi yazmaya devam ediyor (F-P.17: "
                    "planning_node'u kontrol et)"
                )
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
