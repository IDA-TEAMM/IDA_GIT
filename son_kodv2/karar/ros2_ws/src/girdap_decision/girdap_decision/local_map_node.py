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
from girdap_decision.sigterm_kapanis import sigterm_kapanisi_kur
from girdap_decision.saat_kaynagi import bayatlik_saati
from girdap_decision.yeniden_baslama import ResetAbonesi
from prototype.mapping.bev_renderer import BevConfig, BevRenderer, Mp4Yazici
from prototype.mapping.local_map import LocalMapDumper


class LocalMapNode(Node):
    """OccupancyGrid → zaman damgalı mp4 (+ PNG yedeği) — Şartname 4.2 Dosya-3."""

    def __init__(self, **node_kwargs) -> None:
        super().__init__("local_map_node", **node_kwargs)
        # §0.61: bayatlık tek yönlü saatte. Oturum klasörü adı (datetime.now)
        # duvar saatinde kalır — orası mutlak an.
        self._saat = bayatlik_saati(self)

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
        self._base = base            # madde #11: yeniden baslamada yeni oturum
        self._session_dir = self._yeni_oturum_dizini()

        self._rend = BevRenderer(BevConfig())
        self._last: Optional[OccupancyGrid] = None
        self._arac: Tuple[float, float] = (0.0, 0.0)
        self._yaw: Optional[float] = None
        # 🔴 Odom GELMEDEN kenar dubası ÇİZİLMEZ. `_arac` varsayılanı (0,0)
        # ve occupancy ızgarası araç merkezli KURULDUĞU için harita yine
        # doğru görünür — ama dubalar `dunya_to_px(p, _arac)` ile
        # yerleştirildiğinden MUTLAK dünya koordinatlarına düşer. Ölçüldü:
        # araç (30,40)'ta, duba (32,52) iken odom varken piksel (216,104),
        # odom yokken duba KARE DIŞINA çıkıp hiç çizilmiyor. Harita normal
        # görünürken duba yanlış/yok = SESSİZ YANLIŞ VERİ, bu modülün
        # docstring'inde uyardığı hata sınıfının ta kendisi.
        # Kural: bilmiyorsak ÇİZMEYİZ (yanlış çizmektense).
        self._odom_geldi = False
        self._odom_uyari_t = 0.0
        self._edges: List[Tuple[float, float]] = []

        rate = float(self.get_parameter("dump_rate_hz").value)
        if rate < 1.0:
            # Sessizce ihlale düşmek yerine açılışta patla (F-P.18 ruhu).
            raise ValueError(
                f"dump_rate_hz={rate} < 1 Hz — Şartname 4.2 Dosya-3 "
                "'En Az 1 Hz' ihlali; teslim geçersiz olur."
            )

        # madde #11: yazici kurulumu metoda alindi — yeniden baslamada
        # AYNI kod yeni oturum dizinine tekrar kosuyor (kopyalanmadi).
        self._mp4_rate = rate
        self._mp4: Optional[Mp4Yazici] = None
        self._png: Optional[LocalMapDumper] = None
        self._kare = 0
        self._yazicilari_kur()
        self._reset = ResetAbonesi(self, self._yeniden_basla)

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

    def _yeni_oturum_dizini(self):                     # noqa: ANN202
        """Zaman damgali yeni oturum dizini olustur ve dondur."""
        d = self._base / datetime.now().strftime("oturum_%Y%m%d_%H%M%S")
        d.mkdir(parents=True, exist_ok=True)
        return d

    def _yazicilari_kur(self) -> None:
        """mp4 + PNG yedegini `self._session_dir` icin ac."""
        if bool(self.get_parameter("mp4_enabled").value):
            cfg = self._rend.cfg
            try:
                self._mp4 = Mp4Yazici(
                    self._session_dir / "Dosya3_lokal_harita.mp4",
                    fps=self._mp4_rate,
                    boyut=(cfg.genislik_px, cfg.yukseklik_px),
                )
            except Exception as exc:                 # cv2 yok / codec yok
                # PNG yedeği devam etsin — teslim tamamen kaybolmasın.
                self._mp4 = None
                self.get_logger().error(
                    f"Dosya-3 mp4 açılamadı ({exc!r}) → yalnız PNG serisi "
                    "yazılacak. md 5.5.4.3.5: eksik dosya = 5 ceza."
                )
        if bool(self.get_parameter("png_yedek").value):
            self._png = LocalMapDumper(
                base_dir=self._session_dir, session="png_yedek"
            )

    def _yeniden_basla(self) -> None:
        """md 5.5.3.1 yeniden baslama — Dosya-3'u YENI oturum dizinine al.

        🔴 Neden yeni dizin: yeniden baslama hakki kullanilinca ilk turun
        puanlari sifirlanir, yani PUANLANAN kosu ikincisi. Iki turun karelerini
        ayni mp4/PNG serisinde birlestirmek hakeme gecersiz turu de vermek olur;
        ustelik arac basa dondugu icin harita geriye sicrar.

        mp4 ONCE kapatilir: kapatilmadan birakilan dosya cogu codec'te
        bozuk/oynatilamaz kalir (moov atomu yazilmaz) -> ilk turun kaydi da
        gider. md 5.5.4.3.5: eksik/gecersiz dosya = 5 ceza.
        """
        if self._mp4 is not None:
            try:
                self._mp4.kapat()
            except Exception as exc:                 # noqa: BLE001
                self.get_logger().error(
                    f"ilk tur mp4 kapatilamadi: {exc!r} — dosya bozuk olabilir"
                )
            self._mp4 = None
        self._png = None
        self._kare = 0
        self._session_dir = self._yeni_oturum_dizini()
        self._yazicilari_kur()
        self.get_logger().warn(
            f"Dosya-3 YENI OTURUM (md 5.5.3.1): {self._session_dir}"
        )

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
        self._odom_geldi = True

    def _on_edges(self, msg: PoseArray) -> None:
        # Boş liste de anlamlıdır ("kapı görünmüyor") — bayat duba çizmeyelim.
        self._edges = [
            (float(ps.position.x), float(ps.position.y)) for ps in msg.poses
        ]

    # ----- yardımcılar -----

    def _now(self) -> float:
        """Bayatlık saati — TEK YÖNLÜ (§0.61). Mutlak an olarak kullanılmaz."""
        return self._saat()

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
            # Odom yoksa duba katmanını BOŞ geç (yukarıdaki gerekçe) ve
            # operatörü 10 saniyede bir uyar — sessizce eksik çizmeyelim.
            kenarlar = self._edges if self._odom_geldi else []
            if self._edges and not self._odom_geldi:
                now = self._now()
                if now - self._odom_uyari_t > 10.0:
                    self._odom_uyari_t = now
                    self.get_logger().warn(
                        f"{len(self._edges)} kenar dubası geliyor ama "
                        "/girdap/fusion/odom HİÇ gelmedi → dubalar Dosya-3 "
                        "haritasına ÇİZİLMİYOR (yanlış yere çizmektense boş "
                        "bırakılıyor). fusion_node'u kontrol et."
                    )
            try:
                kare = self._rend.render_costmap(
                    occ, res, self._arac, yaw=self._yaw,
                    kenar_dubalari=kenarlar,
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
    # 🔴 SIGTERM kapısı: `systemctl stop/restart` bu sinyali gönderir ve
    # işlenmezse süreç ANINDA ölür → `finally` çalışmaz → mp4'ün moov atomu
    # yazılmaz → TESLİM DOSYASI OYNATILAMAZ (15.08 arızası, bkz. modül).
    sigterm_kapanisi_kur()
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
