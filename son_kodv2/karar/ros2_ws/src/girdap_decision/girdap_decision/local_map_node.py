"""
Girdap İDA — Dosya-3 (lokal harita / cost map) kaydedici node'u (Layer 2).

Şartname 4.2 — Dosya 3:
    "Lokal harita/cost map/engel haritası · En Az 1 Hz"
Teslim: karaya alımdan 20 dk içinde; **4.2'de tanımlanan her bir dosya için**
5 ceza puanı (md 5.5.4.3.5).

════════════════════════════════════════════════════════════════════════════
2026-08-07 TESLİM DENETİMİ — bu node'da DÜZELTİLEN DÖRT ŞEY
════════════════════════════════════════════════════════════════════════════
1) 🔴 **ZAMAN ETİKETİ YOKTU.** Çıktı `frame_00000.png` idi; zaman yalnız
   oturum dizini adında ve dosya mtime'ındaydı. İkisi de **Jetson saatine**
   bağlı ve Jetson saati 07.08'de **~3 saat geri** ölçüldü. Artık damga
   KAREYE yakılıyor (`bev_renderer`), üstelik duvar saatinden değil
   **haritanın kendi `header.stamp`'inden** — kare hangi ana aitse o.
2) 🔴 **FORMAT.** Şartname Dosya-3 için format BELİRTMİYOR (format yazılan
   tek kalem değil: Dosya-1/LiDAR mp4, Dosya-2 csv). Belgenin kendi deseniyle
   tutarlı olsun diye **mp4 birincil** yapıldı; PNG serisi kayıpsız yedek
   olarak KORUNDU (ikisi birden ~5 MB, bedeli yok).
3) 🔴 **TAM 1.0 Hz PAY BIRAKMIYORDU.** Bozuk ızgara ya da disk hatasında kare
   ATLANIYOR (`_on_tick` erken dönüyor) → teslim "En Az 1 Hz"in ALTINA
   düşüyordu. Varsayılan **2 Hz**: bir kare düşse bile şartname sağlanır.
4) 🔴 **KENAR DUBALARI HARİTADA YOKTU.** Turuncu kenar dubaları MPPI'nin
   engel torbasından bilerek çıkarılır (`planning_node._on_classified`), bu
   yüzden occupancy'de hiç görünmüyorlardı → teslim edilen "engel haritası"
   parkurun ANA nesnesini göstermiyordu. `/girdap/planning/edge_buoys`'tan
   (DÜNYA çerçevesi) ayrı katman olarak çiziliyor.

Subscribed:
    /girdap/map/local             nav_msgs/OccupancyGrid   (SensorDataQoS)
    /girdap/fusion/odom           nav_msgs/Odometry        (araç pozu/yaw)
    /girdap/planning/edge_buoys   geometry_msgs/PoseArray  (DÜNYA, çizim katmanı)
Çıktı:
    ~/girdap_logs/local_map/<oturum>/Dosya3_lokal_harita.mp4
    ~/girdap_logs/local_map/<oturum>/png_yedek/frame_00000.png ...
"""

from __future__ import annotations

import math
from datetime import datetime, timezone
from pathlib import Path
from typing import List, Optional, Tuple

import numpy as np
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseArray
from nav_msgs.msg import OccupancyGrid, Odometry

from girdap_decision.qos_profiles import sensor_data_qos
from prototype.mapping.bev_renderer import BevConfig, BevRenderer, Mp4Yazici
from prototype.mapping.local_map import LocalMapDumper


class LocalMapNode(Node):
    """OccupancyGrid → zaman damgalı mp4 (+ PNG yedeği) — Şartname 4.2 Dosya-3."""

    def __init__(self, **node_kwargs) -> None:
        super().__init__("local_map_node", **node_kwargs)

        # --- Parametreler ---
        # ⚠ 1.0 DEĞİL 2.0: şartname "En Az 1 Hz" — tam sınırda koşmak, tek
        # atlanan karede ihlale düşmek demek (yukarıdaki 3. madde).
        self.declare_parameter("dump_rate_hz", 2.0)
        self.declare_parameter("output_dir", "")        # boş → ~/girdap_logs/...
        self.declare_parameter("mp4_enabled", True)     # birincil teslim
        self.declare_parameter("png_yedek", True)       # kayıpsız yedek
        # F-P.17: /girdap/map/local kesilirse aynı DONMUŞ kare yeni adlarla
        # yazılmaya devam eder; Dosya-3 canlı görünür ama değildir. Format
        # DEĞİŞTİRİLMEZ (teslim sözleşmesi) — operatör sesli uyarılır.
        self.declare_parameter("map_timeout_s", 3.0)
        # 🔴 Saat güveni: doğru yolu çekirdeğin `adjtimex` STA_UNSYNC bayrağı
        # (algı katmanında `girdap_ida_algi/saat.py` tam bunu yapıyor).
        # Aynı mantığı ikinci kez YAZMIYORUZ (iki kopya ayrışır — saat.py'nin
        # kendi gerekçesi). Katmanlar ortak modülü paylaşana kadar bu bayrak
        # dışarıdan verilir; False iken damganın yanına "[SAAT?]" basılır.
        self.declare_parameter("saat_guvenilir", True)

        self._map_timeout = float(self.get_parameter("map_timeout_s").value)
        self._saat_guvenilir = bool(self.get_parameter("saat_guvenilir").value)
        self._last_map_t: Optional[float] = None
        self._stale_warned = False

        out = str(self.get_parameter("output_dir").value) or None
        base = (
            Path(out).expanduser() if out
            else Path.home() / "girdap_logs" / "local_map"
        )
        oturum = datetime.now().strftime("oturum_%Y%m%d_%H%M%S")
        self._session_dir = base / oturum
        self._session_dir.mkdir(parents=True, exist_ok=True)

        self._rend = BevRenderer(BevConfig())
        self._last: Optional[OccupancyGrid] = None
        self._arac: Tuple[float, float] = (0.0, 0.0)
        self._yaw: Optional[float] = None
        self._edges: List[Tuple[float, float]] = []

        rate = float(self.get_parameter("dump_rate_hz").value)
        if rate < 1.0:
            # Sessizce ihlale düşmek yerine açılışta patla (F-P.18 ruhu).
            raise ValueError(
                f"dump_rate_hz={rate} < 1 Hz — Şartname 4.2 Dosya-3 "
                "'En Az 1 Hz' ihlali; teslim geçersiz olur."
            )

        self._mp4: Optional[Mp4Yazici] = None
        if bool(self.get_parameter("mp4_enabled").value):
            cfg = self._rend.cfg
            try:
                self._mp4 = Mp4Yazici(
                    self._session_dir / "Dosya3_lokal_harita.mp4",
                    fps=rate, boyut=(cfg.genislik_px, cfg.yukseklik_px),
                )
            except Exception as exc:                 # cv2 yok / codec yok
                # PNG yedeği devam etsin — teslim tamamen kaybolmasın.
                self.get_logger().error(
                    f"Dosya-3 mp4 açılamadı ({exc!r}) → yalnız PNG serisi "
                    "yazılacak. md 5.5.4.3.5: eksik dosya = 5 ceza."
                )

        self._png: Optional[LocalMapDumper] = None
        if bool(self.get_parameter("png_yedek").value):
            self._png = LocalMapDumper(
                base_dir=self._session_dir, session="png_yedek"
            )

        self._kare = 0

        # --- Subscribers ---
        self._sub = self.create_subscription(
            OccupancyGrid, "/girdap/map/local", self._on_map, sensor_data_qos()
        )
        self._sub_odom = self.create_subscription(
            Odometry, "/girdap/fusion/odom", self._on_odom, 10
        )
        # Kenar dubaları DÜNYA çerçevesinde gelir (planning_node dönüştürür —
        # gövde→dünya dönüşümü TEK yerde kalsın diye burada tekrarlanmaz).
        self._sub_edges = self.create_subscription(
            PoseArray, "/girdap/planning/edge_buoys", self._on_edges, 10
        )

        self._timer = self.create_timer(1.0 / rate, self._on_tick)
        self.get_logger().info(
            f"local_map_node aktif → {self._session_dir} "
            f"(Dosya-3: {rate} Hz, mp4={'AÇIK' if self._mp4 else 'kapalı'}, "
            f"png_yedek={'AÇIK' if self._png else 'kapalı'})"
        )

    # ----- callback'ler -----

    def _on_map(self, msg: OccupancyGrid) -> None:
        self._last = msg
        self._last_map_t = self._now()
        if self._stale_warned:
            self._stale_warned = False
            self.get_logger().info("/girdap/map/local yayını geri geldi")

    def _on_odom(self, msg: Odometry) -> None:
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        self._arac = (float(p.x), float(p.y))
        self._yaw = 2.0 * math.atan2(float(q.z), float(q.w))

    def _on_edges(self, msg: PoseArray) -> None:
        # Boş liste de anlamlıdır ("kapı görünmüyor") — bayat duba çizmeyelim.
        self._edges = [
            (float(ps.position.x), float(ps.position.y)) for ps in msg.poses
        ]

    # ----- yardımcılar -----

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _map_stale(self) -> bool:
        if self._map_timeout <= 0.0 or self._last_map_t is None:
            return False
        return (self._now() - self._last_map_t) > self._map_timeout

    @staticmethod
    def _damga(msg: OccupancyGrid) -> str:
        """Kareye yakılacak zaman — HARİTANIN KENDİ stamp'i (duvar saati değil).

        Kare hangi ana aitse damga o olmalı; `_on_tick` duvar saatiyle yazsaydı
        bayat bir kare taze görünürdü.
        """
        s = msg.header.stamp
        t = float(s.sec) + float(s.nanosec) * 1e-9
        if t <= 0.0:                                   # stamp doldurulmamış
            return "STAMP-YOK"
        return (
            datetime.fromtimestamp(t, tz=timezone.utc)
            .strftime("%Y-%m-%dT%H:%M:%S.%f")[:-3] + "Z"
        )

    # ----- kayıt döngüsü -----

    def _on_tick(self) -> None:
        if self._last is None:                         # henüz harita gelmedi
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
        w, h = int(m.info.width), int(m.info.height)
        if w <= 0 or h <= 0 or len(m.data) != w * h:
            self.get_logger().error(
                f"Bozuk OccupancyGrid ({len(m.data)} değer, {w}x{h}), "
                "kare atlandı", throttle_duration_sec=5.0,
            )
            return

        occ = np.asarray(m.data, dtype=np.int16).reshape(h, w)
        res = float(m.info.resolution) or 0.5

        if self._mp4 is not None:
            try:
                kare = self._rend.render_costmap(
                    occ, res, self._arac, yaw=self._yaw,
                    kenar_dubalari=self._edges,
                    zaman_metni=self._damga(m), kare_no=self._kare,
                    saat_guvenilir=self._saat_guvenilir,
                )
            except ValueError as e:
                self.get_logger().error(
                    f"Dosya-3 karesi çizilemedi: {e}",
                    throttle_duration_sec=5.0)
                return
            if not self._mp4.yaz(kare):
                self.get_logger().error(
                    "Dosya-3 mp4 karesi yazılamadı (disk dolu olabilir)",
                    throttle_duration_sec=5.0)

        if self._png is not None:
            if self._png.write_frame(list(m.data), w, h) is None:
                self.get_logger().error(
                    "Dosya-3 PNG yedeği yazılamadı (disk dolu olabilir)",
                    throttle_duration_sec=5.0)

        self._kare += 1
        if self._kare % 20 == 1:
            self.get_logger().info(
                f"[Dosya-3] {self._kare} kare → {self._session_dir.name}"
            )

    def destroy_node(self) -> bool:
        """mp4 yazıcısını KAPAT — kapatılmayan dosya OYNATILAMAZ (moov atomu)."""
        if self._mp4 is not None:
            self._mp4.kapat()
            self._mp4 = None
        return super().destroy_node()


def main(args: Optional[list] = None) -> None:
    rclpy.init(args=args)
    node = LocalMapNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # ⚠ mp4 her hâlükârda kapanmalı: yarım kalan dosya = teslim edilemez
        # dosya = 5 ceza. destroy_node bunu üstleniyor.
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
