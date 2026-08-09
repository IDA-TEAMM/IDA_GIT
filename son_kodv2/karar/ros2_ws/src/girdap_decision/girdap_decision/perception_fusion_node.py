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
(sahada sessiz ölüm yerine görünür arıza).

🔴 EŞLEŞME HİÇ OLMUYORSA ÖNCE `sync_queue_size`'A BAK, `sync_slop_s`'E DEĞİL
(2026-07-09 tezgah ölçümü, gerçek Livox + OAK ile; 09.08'de algı ekibi
düzeltmenin bu hatta hiç taşınmadığını bulup raporladı):
    Ham sensör damgaları AYNI TABANDA — ölçülen fark ~27 ms, yani slop=0,1 s
    fazlasıyla yetiyor. **Saat sorunu yoktu, gecikme sorunu vardı.** Kapalı
    alanda ~20k yoğun nokta → clustering 1-3,3 s/kare → `obstacle_map`
    damgası DOĞRU ama GEÇ varıyor. Kuyruk, aynı damgalı kamera karesini o
    gecikme boyunca elde tutamazsa kare düşer ve eşleşme hiç oluşmaz →
    `/perception/classified_obstacles` ÜRETİLMEZ.
    Ölçülen: queue=10 (~0,6 s) hiç tutmuyor · queue=50 (~2,9 s) KIL PAYI
    yetmedi · **queue=100 (~6 s) ile füzyon üretmeye başladı.**
    Slop'u büyütmek yanlış ilaçtır: eşleşmeyi zamanca yanlış hâle getirir
    (farklı anların karesi ile bulutu eşlenir), kuyruğu büyütmek eşleşmeyi
    zamanca DOĞRU tutar, yalnız gecikmeli sunar.
    ⚠ Sonucun bedeli tüm P1+P2: sınıf gelmezse `planning_node._edge_buoys`
    boş kalır, kapı takibi ham GPS noktasına düşer.
    ⚖ Bu gecikme kapalı-alan artefaktı olabilir (açık suda seyrek nokta →
    clustering ms'ler); ama açık suda ölçülmedi, o yüzden ölçülmüş değer
    varsayılan bırakıldı.

🆕 H3 (2026-08-09) — KAMERANIN KENDİ MENZİLİ ARTIK KULLANILIYOR.
Ölçüldü (§0.19c): 30 cm duba LiDAR'da ~8 m'de kesiliyor (yeterli nokta
düşmüyor), kapı takibi ise iki direği aynı karede görmek için 8,8-15 m'ye
ihtiyaç duyuyor → İKİ PENCERE ÖRTÜŞMÜYORDU ve model AÇIKKEN P1 çöküyordu
(§0.20c: 0 kapı, 1/4 nokta). Algı tarafı 08.08'den beri bbox genişliğinden
menzil kestirip `/perception/buoys_3d`'ye basıyordu (0,5-15 m); karar tarafı
bu topic'e hiç abone değildi. Artık damgaya göre önbelleğe alınıp füzyona
veriliyor → LiDAR'ın göremediği mesafedeki duba da sınıfıyla birlikte
planlamaya ulaşıyor.

Subscribed:
    /perception/obstacle_map   geometry_msgs/PoseArray        (RELIABLE)
    /perception/buoys          vision_msgs/Detection2DArray   (RELIABLE)
    /perception/buoys_3d       geometry_msgs/PoseArray        (H3, opsiyonel —
                               akmıyorsa eski bearing-only davranış birebir)
Published:
    /perception/classified_obstacles   vision_msgs/Detection3DArray
Header: frame_id=base_link, stamp = LiDAR mesajının stamp'i (iki kaynaktan
    birini seçmek gerekiyor — 3D konumun kaynağı olduğu için LiDAR baz alınır).
"""

from __future__ import annotations

from collections import OrderedDict
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

    def __init__(self, **node_kwargs) -> None:
        # node_kwargs → parameter_overrides passthrough (test enjeksiyonu,
        # diğer node'larla tutarlı — F-P.10 testi bunu gerektirir).
        super().__init__("perception_fusion_node", **node_kwargs)

        # --- Parametreler (config/hardware.yaml perception.fusion bloğu) ---
        self.declare_parameter("bearing_tolerance_rad", 0.15)
        self.declare_parameter("camera_hfov_rad", 1.2)
        # 2026-07-17: oakd_driver_node 1280x720'e çıkarıldı (config-drift
        # riski — bu node'un kod varsayılanı params.yaml/hardware.yaml'daki
        # gerçek değerle her zaman AYNI olmalı, yoksa params.yaml verilmeden
        # elle çalıştırılırsa bearing hesaplaması sessizce yanlış çıkar).
        self.declare_parameter("camera_image_width_px", 1280)
        self.declare_parameter("camera_image_height_px", 720)
        self.declare_parameter("sync_slop_s", 0.1)
        # LiDAR clustering gecikince obstacle_map GEÇ varır (damgası doğru ama
        # geç). Sync'in aynı-damgalı kamera karesini o gecikme boyunca elde
        # tutabilmesi için kuyruk derinliği yeterli olmalı (kamera Hz × gecikme
        # sn). Slop'u DEĞİL bunu büyüt — modül docstring'indeki ölçüme bak.
        self.declare_parameter("sync_queue_size", 100)
        self.declare_parameter("log_period_s", 5.0)

        p = self.get_parameter
        self._sync_queue_size = int(p("sync_queue_size").value)
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
        # 🆕 H3: kameranın KENDİ 3B konum kestirimi. Sync'e ÜÇÜNCÜ girdi
        # olarak EKLENMEZ — 3'lü ApproximateTime eşleşmeyi zorlaştırır ve
        # çalışan LiDAR↔kamera senkronunu riske atar. Bunun yerine damgaya
        # göre önbelleğe alınır: algı node'u `buoys` ile `buoys_3d`'yi AYNI
        # fonksiyonda, AYNI damgayla, ARDIŞIK yayınlıyor (tespit_yayinla) →
        # damga eşleşmesi kesin, indeks sırası da BİREBİR aynı.
        self._buoys3d: "OrderedDict[tuple, list]" = OrderedDict()
        self._sub_buoys3d = self.create_subscription(
            PoseArray, "/perception/buoys_3d", self._on_buoys3d, 10
        )
        self._sync = message_filters.ApproximateTimeSynchronizer(
            [self._lidar_sub, self._camera_sub],
            queue_size=self._sync_queue_size,
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
            f"slop={p('sync_slop_s').value} s, "
            f"queue={p('sync_queue_size').value})"
        )

    # ------------------------------------------------------------- watchdog

    def _count_lidar(self, _msg) -> None:            # noqa: ANN001
        self._n_lidar_in += 1

    def _count_camera(self, _msg) -> None:           # noqa: ANN001
        self._n_camera_in += 1

    def _on_sync_watchdog(self) -> None:
        """Pencere içinde iki girdi de aktı ama sync hiç ateşlemediyse WARN;
        girdilerden biri HİÇ gelmediyse de (F-P.22) ayrı bir WARN bas.

        F-P.22 (2026-07-16/17, gerçek donanım testi): /perception/buoys'u
        hiçbir şey üretmiyordu (perception_camera_node use_onboard_camera=
        false varsayılanıyla hiç açılmamıştı, algı ekibinin ayrı OAK node'u
        da bu ortamda yoktu) — perception_fusion_node bunu SESSİZCE fark
        etmeden bekliyordu, tek belirti /perception/classified_obstacles'ın
        hiç yayınlanmamasıydı ve bu da başka hiçbir yerde loglanmıyordu.
        İkisi de boot'ta 0 olması normaldir (henüz hiçbir sürücü ayağa
        kalkmamış olabilir) — yalnız BİRİ akarken diğeri hiç akmıyorsa uyar.
        """
        if self._n_lidar_in == 0 and self._n_camera_in == 0:
            pass                             # ikisi de henüz başlamamış — boot, normal
        elif self._n_camera_in == 0:
            self.get_logger().warn(
                f"sync bekçisi: {self._n_lidar_in} lidar mesajı geldi ama "
                "/perception/buoys'tan HİÇ mesaj gelmedi — perception_camera_"
                "node çalışmıyor olabilir mi (use_onboard_camera:=false ise "
                "ve algı ekibinin ayrı OAK node'u da çalışmıyorsa hiçbir şey "
                "bu topic'i üretmez, F-P.22) kontrol et"
            )
        elif self._n_lidar_in == 0:
            self.get_logger().warn(
                f"sync bekçisi: {self._n_camera_in} kamera mesajı geldi ama "
                "/perception/obstacle_map'ten HİÇ mesaj gelmedi — "
                "perception_lidar_node/Livox sürücüsü çalışmıyor olabilir"
            )
        elif self._n_sync == 0:
            # ⚠ Reçetenin sırası ÖLÇÜMDEN gelir (2026-07-09 tezgahı): damgalar
            # aynı tabandaydı (~27 ms), arıza kuyruk derinliğiydi. Bu satır
            # eskiden önce slop'u işaret ediyordu — sahada yanlış yere baktırır.
            self.get_logger().warn(
                f"sync bekçisi: {self._n_lidar_in} lidar + "
                f"{self._n_camera_in} kamera mesajı geldi ama eşleşme SIFIR — "
                f"ÖNCE sync_queue_size'a bak (şu an "
                f"{self.get_parameter('sync_queue_size').value}): LiDAR "
                "clustering gecikirse kamera karesi kuyruktan düşer, tezgahta "
                "10 ve 50 yetmemiş 100 tutmuştu. Ancak bu da olmazsa stamp "
                "tabanlarını denetle (docstring: STAMP SÖZLEŞMESİ) — slop'u "
                "büyütmek son çare, eşleşmeyi zamanca yanlış hâle getirir"
            )
        self._n_lidar_in = self._n_camera_in = self._n_sync = 0

    # ------------------------------------------------------------- callback

    def _on_sync(self, poses: PoseArray, detections: Detection2DArray) -> None:
        # F-P.10 (robustness taraması, 2026-07-15): try/except yoktu — ör.
        # camera_image_width/height_px yanlışlıkla 0 verilirse (config/
        # launch-arg yazım hatası) _to_camera_detection'daki bölme
        # ZeroDivisionError fırlatır, callback'ten sızıp node'u KALICI
        # öldürürdü — /perception/classified_obstacles sessizce durur.
        try:
            self._n_sync += 1
            lidar_list = [
                LidarDetection(x=pose.position.x, y=pose.position.y,
                                radius=abs(pose.orientation.z))
                for pose in poses.poses
            ]
            camera_list = []
            ham_idx = []                 # H3: buoys_3d ile indeks hizası
            for k, det in enumerate(detections.detections):
                cam = self._to_camera_detection(det)
                if cam is not None:
                    camera_list.append(cam)
                    ham_idx.append(k)
            n_konumlu = self._konumlari_bagla(detections, camera_list, ham_idx)
            fused = associate(lidar_list, camera_list, self._cfg)
        except Exception as exc:
            self.get_logger().error(
                f"füzyon başarısız, bu senkron atlandı: {exc!r}",
                throttle_duration_sec=5.0,
            )
            return

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
        n_kamera = sum(1 for o in fused if o.source == "kamera")
        self.get_logger().debug(
            f"{len(lidar_list)} lidar + {len(camera_list)} kamera → "
            f"{len(fused)} füzyon ({n_matched} eşleşti, {n_kamera} yalnız kamera)"
        )
        self._periodic_info(len(fused), n_matched, n_kamera, n_konumlu)

    # --------------------------------------------------------- H3: buoys_3d

    @staticmethod
    def _damga(header) -> tuple:                       # noqa: ANN001, ANN205
        return (header.stamp.sec, header.stamp.nanosec)

    def _on_buoys3d(self, msg: PoseArray) -> None:
        """Kameranın 3B konum kestirimini damgaya göre önbelleğe al (H3).

        Önbellek `sync_queue_size` kadar derin: kamera karesi sync kuyruğunda
        ne kadar bekleyebiliyorsa (LiDAR gecikmesi kadar), eşi de o kadar
        beklemeli — yoksa eşleşme anında 3B konum çoktan düşmüş olur ve
        düzeltme sessizce hiç çalışmaz.
        """
        self._buoys3d[self._damga(msg.header)] = [
            (p.position.x, p.position.y, abs(p.orientation.z))
            for p in msg.poses
        ]
        while len(self._buoys3d) > self._sync_queue_size:
            self._buoys3d.popitem(last=False)          # en eskisini at

    def _konumlari_bagla(
        self, detections, camera_list: list, ham_idx: list
    ) -> int:                                          # noqa: ANN001
        """Önbellekteki 3B konumları kamera tespitlerine iliştir (H3).

        🔑 **İNDEKS SÖZLEŞMESİ:** algı node'u `buoys` ile `buoys_3d`'yi tek
        döngüde, aynı sırayla dolduruyor → `detections[k]` ile `poses[k]`
        AYNI tespittir. Ama `camera_list` süzülerek kuruluyor (sayısal
        olmayan class_id atlanır), bu yüzden `ham_idx` orijinal indeksi
        taşır — onsuz sınıf ile konum KAYARDI.

        Uzunluk uyuşmazsa hiçbir şey iliştirilmez: sözleşme bozulmuşsa
        yanlış konum vermektense eski davranışa (bearing-only) düşmek
        güvenlidir.
        """
        konumlar = self._buoys3d.get(self._damga(detections.header))
        if konumlar is None:
            return 0
        if len(konumlar) != len(detections.detections):
            self.get_logger().warn(
                f"buoys_3d uzunluğu ({len(konumlar)}) buoys ile "
                f"({len(detections.detections)}) uyuşmuyor — indeks sözleşmesi "
                "bozulmuş, 3B konumlar YOK SAYILDI (bearing-only'ye düşüldü)",
                throttle_duration_sec=5.0,
            )
            return 0
        n = 0
        for cam, k in zip(camera_list, ham_idx):
            cx, cy, r = konumlar[k]
            cam.x, cam.y, cam.radius = float(cx), float(cy), float(r)
            n += 1
        return n

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

    def _periodic_info(self, n_fused: int, n_matched: int,
                       n_kamera: int = 0, n_konumlu: int = 0) -> None:
        """log_period_s'de bir INFO — her sync callback'te log seli olmasın.

        `yalnız kamera` sayısı H3'ün sahadaki tek görünürlük kanalı: 0 kalıyorsa
        ya LiDAR her şeyi zaten görüyor (iyi) ya da `/perception/buoys_3d`
        akmıyor (kötü — `konumlu` sütunu 0 ise sebep budur).
        """
        now = self.get_clock().now().nanoseconds * 1e-9
        if self._last_log_t is None or now - self._last_log_t >= self._log_period_s:
            self._last_log_t = now
            self.get_logger().info(
                f"füzyon: {n_fused} engel ({n_matched} sınıflı, "
                f"{n_fused - n_matched} bilinmiyor · {n_kamera} yalnız kamera, "
                f"{n_konumlu} tespitte 3B konum var)"
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
