"""
Girdap İDA — LiDAR engel tespiti node'u (Layer 2, Sprint 1).

/livox/lidar point cloud'unu ROS-bağımsız çekirdekten (prototype.perception.
lidar_obstacles) geçirip planning'in beklediği /perception/obstacle_map
sözleşmesine yayınlar. Kaynak-bağımsız (replaceable design): topic adı sabit,
arkasındaki üretici (gerçek Livox sürücüsü / sentetik / Gazebo) değişebilir.

⚠ PLACEHOLDER mesaj şeması (planning_node._on_obstacles ile birebir):
    PoseArray içindeki her Pose:
        position.x / position.y = cluster centroid (engel merkezi, base_link)
        orientation.z           = çevrel yarıçap (m) — quaternion DEĞİL, hack
        orientation.w           = 1.0
    Gerçek quaternion semantiği yok; girdap_msgs custom mesajı gelene kadar
    bu şema korunur (downstream: planning_node, Sprint 3 fusion).

Subscribed:
    /livox/lidar               sensor_msgs/PointCloud2   (SensorDataQoS)
Published:
    /perception/obstacle_map   geometry_msgs/PoseArray   (default RELIABLE —
                               planning depth-10 default QoS ile tüketiyor)
"""

from __future__ import annotations

import time
from typing import Optional

import numpy as np
from numpy.lib.recfunctions import structured_to_unstructured
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Pose, PoseArray
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2

from girdap_decision.qos_profiles import sensor_data_qos
from girdap_decision.saat_kaynagi import bayatlik_saati
from prototype.perception.lidar_obstacles import (
    LidarObstacleConfig,
    detect_obstacles,
)


class PerceptionLidarNode(Node):
    """PointCloud2 → filter/cluster → PoseArray daire engel listesi."""

    def __init__(self, **node_kwargs) -> None:
        # **node_kwargs: fsm_node/planning_node ile aynı sözleşme —
        # parameter_overrides ile test/launch'tan parametre enjekte edilebilir
        # (cfg __init__'te kurulduğu için sonradan set_parameters yetmez).
        super().__init__("perception_lidar_node", **node_kwargs)
        self._saat = bayatlik_saati(self)          # §0.61: tek yönlü saat

        # --- Parametreler (config/hardware.yaml perception.lidar bloğu) ---
        self.declare_parameter("z_min", 0.1)
        self.declare_parameter("z_max", 3.0)
        self.declare_parameter("cluster_tolerance", 0.5)
        self.declare_parameter("min_cluster_size", 5)
        self.declare_parameter("max_cluster_size", 500)
        self.declare_parameter("split_cell_m", 1.0)  # F5.4: büyük küme bölme
        self.declare_parameter("max_range", 25.0)
        self.declare_parameter("voxel_size", 0.1)   # F5.3; 0 = kapalı
        self.declare_parameter("log_period_s", 5.0)
        # F-L.3 (11.08.2026, GERÇEK donanımda ölçüldü) — PAKET BİRLEŞTİRME.
        # livox_ros_driver2 bu sürümde `publish_freq`'i mesaj birleştirmede
        # KULLANMIYOR: `Lddc::publish_period_ns_` hesaplanıp hiçbir yerde
        # okunmuyor (ölü kod), `PublishPointcloud2` kuyruktaki HER paketi tek
        # tek yayınlıyor. Ölçüm: /livox/lidar ~475 Hz, her mesaj width=96
        # (tek Livox ethernet paketi), publish_freq=10.0 olmasına rağmen.
        # Sonuç: kümeleme mesaj başına koştuğu için 30 cm'lik bir duba tek
        # pakette min_cluster_size eşiğini pratikte HİÇ toplayamaz — paketin
        # açısal dilimi çok dar. "obstacle_map akıyor" ama içi boş.
        # Bu yüzden birleştirme BİZİM tarafta yapılır (sürücü başkasının alanı).
        # 0.0 → kapalı, eski davranış BİREBİR (her mesaj ayrı kümelenir).
        # ⚠ VARSAYILAN 0.1 → 0.0 (11.08.2026 öğleden sonra, ölçümle).
        # Yukarıdaki 96-nokta teşhisi sürücünün ARIZALI bir örneğine aitmiş:
        # `girdap-livox` açılışta "bind failed" ile ölmüş, ölçüm o sırada
        # koşan eski örnekten alınmış. Servis düzgün başlatılınca ölçülen
        # /livox/lidar = 10 Hz × ~20 000 nokta, yani sürücü ZATEN doğru kare
        # üretiyor. Birleştirme bu kareler üstünde 2-3 kareyi üst üste bindirdi:
        # 40-60 bin nokta → kümeleme 172-300 ms (bütçe 100 ms) → obstacle_map
        # 9,3 Hz yerine 2,2 Hz. Kazanç yok, bedel kesin; üstelik açıkken
        # kuyruk derinliği 1 → 60 olduğu için F7.3'ün bayat-tarama koruması
        # da kalkıyor. Mekanizma DURUYOR — 96-nokta hali geri gelirse tek
        # parametreyle (>0) açılır, davranışı testlerle kilitli.
        self.declare_parameter("birlestirme_s", 0.0)
        # B0/F5.1 — montaj ofseti = livox_frame'in base_link İÇİNDEKİ konumu
        # (nokta dönüşümü ters yönde: p_base = R(yaw)·p_livox + t).
        # Değerleri hardware.launch `tf.livox_frame` bloğundan geçirir; bu
        # blok static TF yayıncısını da besliyor → TEK kaynak.
        self.declare_parameter("mount_x", 0.0)
        self.declare_parameter("mount_y", 0.0)
        self.declare_parameter("mount_z", 0.0)
        self.declare_parameter("mount_yaw", 0.0)

        p = self.get_parameter
        self._cfg = LidarObstacleConfig(
            z_min=float(p("z_min").value),
            z_max=float(p("z_max").value),
            cluster_tolerance=float(p("cluster_tolerance").value),
            min_cluster_size=int(p("min_cluster_size").value),
            max_cluster_size=int(p("max_cluster_size").value),
            split_cell_m=float(p("split_cell_m").value),
            max_range=float(p("max_range").value),
            voxel_size=float(p("voxel_size").value),
            mount_x=float(p("mount_x").value),
            mount_y=float(p("mount_y").value),
            mount_z=float(p("mount_z").value),
            mount_yaw=float(p("mount_yaw").value),
        )
        self._log_period_s = float(p("log_period_s").value)
        # Livox Mid-360 spesifikasyonu: 200 000 nokta/s @ 10 Hz → kare
        # başına 20 000 nokta. Kümeleme bu periyodu (100 ms) aşarsa
        # obstacle_map geç varır ve füzyon eşleşmesi kesilir.
        self._beklenen_hz = 10.0
        self._last_log_t: Optional[float] = None

        # F-L.3 birleştirme durumu
        self._birlestirme_s = float(p("birlestirme_s").value)
        self._biriken: list = []          # bekleyen (N,3) nokta dizileri
        self._biriktirme_t0: Optional[float] = None

        # --- I/O ---
        self._pub = self.create_publisher(
            PoseArray, "/perception/obstacle_map", 10
        )
        # F7.3: depth=1 — kümeleme (F5.3) 10 Hz'e yetişemezse kuyrukta bayat
        # taramalar birikmesin; her callback ELDEKİ EN YENİ taramayı işlesin.
        # depth=10 ile ~1 s gecikmiş bulutla plan yapılıyordu.
        #
        # F-L.3: birleştirme AÇIKKEN depth=1 YANLIŞ olur — sürücü ~475 Hz'te
        # paket bastığı için pencerenin paketlerinin çoğu kuyruktan DÜŞER ve
        # birleştirme hiçbir şey kazandırmaz. Pencereyi taşıyacak kadar derin
        # kuyruk verilir (475 Hz × pencere, üstüne pay). Bayatlık riski yok:
        # biriken paketler ZATEN aynı pencerenin içindeki taramalardır.
        if self._birlestirme_s > 0.0:
            derinlik = max(10, int(500.0 * self._birlestirme_s) + 10)
        else:
            derinlik = 1
        self._sub = self.create_subscription(
            PointCloud2, "/livox/lidar", self._on_cloud,
            sensor_data_qos(depth=derinlik),
        )

        self.get_logger().info(
            "perception_lidar_node aktif: /livox/lidar → "
            "/perception/obstacle_map "
            f"(montaj xyz=[{self._cfg.mount_x:.3f},{self._cfg.mount_y:.3f},"
            f"{self._cfg.mount_z:.3f}] m yaw={self._cfg.mount_yaw:.3f} rad, "
            f"z=[{self._cfg.z_min},{self._cfg.z_max}] m base_link'e göre, "
            f"tol={self._cfg.cluster_tolerance} m, "
            f"size=[{self._cfg.min_cluster_size},{self._cfg.max_cluster_size}], "
            f"menzil={self._cfg.max_range} m)"
        )
        # B0/F5.1 SESSİZ ARIZA KAPANI: montaj z'si girilmemişse z filtresi
        # fiilen ham LiDAR çerçevesinde uygulanır ve dubalar (LiDAR'ın
        # ALTINDA kaldıkları için) TAMAMEN elenir — hiçbir hata basılmadan
        # obstacle_map boş gelir. Bu tam olarak 04.08'de atölyede yaşanan
        # arızadır; bir daha sessiz olmasın.
        if self._cfg.mount_z == 0.0 and self._cfg.z_min > 0.0:
            self.get_logger().error(
                "MONTAJ Z'Sİ SIFIR — z_min=%.2f m ham LiDAR çerçevesinde "
                "uygulanacak. LiDAR gövde tabanından yukarıdaysa dubalar "
                "eşiğin ALTINDA kalır ve obstacle_map BOŞ döner (B0/F5.1). "
                "Düzeltme: hardware.yaml tf.livox_frame.z ölçülü değeri "
                "(0.41 m) içermeli — launch bu değeri buraya geçirir."
                % self._cfg.z_min
            )

    # ------------------------------------------------------------- callback

    def _on_cloud(self, msg: PointCloud2) -> None:
        # F-L.1: read_points_numpy KULLANMA — gerçek Livox bulutu karışık
        # dtype'lı (x/y/z/intensity float32 + tag/line uint8 + timestamp
        # float64) ve read_points_numpy, field_names'ten BAĞIMSIZ olarak TÜM
        # alanların aynı tipte olmasını assert eder → ilk gerçek mesajda
        # AssertionError. Yapılandırılmış okuma + seçili alanları düz diziye
        # çevirme aynı işi güvenle yapar.
        #
        # F-P.3 (robustness taraması, 2026-07-15): bu blok try/except'siz
        # HİÇ değildi — sürücü yeniden bağlanması/USB hatası gibi tek bir
        # beklenmedik alan şeması (ör. 'z' alanı eksik) node'u KALICI
        # ÖLDÜRÜRDÜ; engel tespiti görevin geri kalanı için sessizce sıfır
        # kalırdı (hiçbir restart supervisor'ı yok). Artık bozuk tarama
        # atlanır, node yaşamaya devam eder.
        try:
            structured = point_cloud2.read_points(
                msg, field_names=("x", "y", "z"), skip_nans=True
            )
            points = structured_to_unstructured(structured).reshape(-1, 3)
        except Exception as exc:
            self.get_logger().error(
                f"bozuk PointCloud2, bu tarama atlandı: {exc!r}",
                throttle_duration_sec=5.0,
            )
            return

        # F-L.3: pencere dolana kadar biriktir, kümelemeyi ERTELE.
        # ⚠ Biriken paketler sensör çerçevesinde toplanır; pencere boyunca
        # tekne hareket ederse iz hafifçe yayılır (1 m/s × 0,1 s = 10 cm,
        # duba çapı 30 cm → kabul edilebilir). Pencereyi büyütmek bu
        # yayılmayı da büyütür; 0,1 s bilinçli üst sınır.
        if self._birlestirme_s > 0.0:
            simdi = time.perf_counter()
            if self._biriktirme_t0 is None:
                self._biriktirme_t0 = simdi
            self._biriken.append(points)
            if simdi - self._biriktirme_t0 < self._birlestirme_s:
                return
            points = np.vstack(self._biriken) if self._biriken else points
            self._biriken = []
            self._biriktirme_t0 = None

        try:
            t0 = time.perf_counter()
            obstacles = detect_obstacles(
                np.asarray(points, dtype=np.float64), self._cfg
            )
            sure_ms = (time.perf_counter() - t0) * 1000.0
        except Exception as exc:
            # Birikim ZATEN boşaltıldı (yukarıda) — hatalı pencere bir
            # sonrakine taşınmaz, tampon sınırsız büyüyemez.
            self.get_logger().error(
                f"kümeleme başarısız, bu pencere atlandı: {exc!r}",
                throttle_duration_sec=5.0,
            )
            return
        self._pub.publish(self._to_pose_array(obstacles, msg))

        self.get_logger().debug(
            f"{len(points)} nokta → {len(obstacles)} engel ({sure_ms:.1f} ms)"
        )
        self._periodic_info(len(obstacles), len(points), sure_ms)

    def _to_pose_array(self, obstacles: list, msg: PointCloud2) -> PoseArray:
        """CircleObstacle listesi → placeholder PoseArray (docstring'e bak)."""
        out = PoseArray()
        out.header.stamp = msg.header.stamp          # kaynak damgasını koru
        # B0: artık yalnız ETİKET değil — noktalar sensor_to_base() ile
        # fiilen base_link'e taşındıktan sonra kümelendi.
        out.header.frame_id = "base_link"
        for obs in obstacles:
            pose = Pose()
            pose.position.x = obs.center_x
            pose.position.y = obs.center_y
            pose.position.z = 0.0
            pose.orientation.z = obs.radius          # ⚠ yarıçap hack'i
            pose.orientation.w = 1.0
            out.poses.append(pose)
        return out

    def _periodic_info(
        self, n_obstacles: int, n_points: int = 0, sure_ms: float = 0.0
    ) -> None:
        """log_period_s'de bir INFO — her callback'te log seli olmasın.

        🔴 **SÜRE NEDEN LOGLANIYOR (2026-08-09 ölçümü).** Kümeleme süresi
        aracın SESSİZ tek darboğazıydı: 09.07 tezgahında 1-3,3 s/kare ölçülmüş
        ve `/perception/obstacle_map` o kadar GEÇ varınca füzyon eşleşmesi hiç
        oluşmamıştı (`classified_obstacles` üretilmiyordu → kapı takibi ham
        GPS'e düşüyordu → P1/P2 puanı gidiyordu). Ama node **hiçbir yerde süre
        yazmıyordu**: sahada "gecikme kaç saniye" sorusunun cevabı yoktu.

        Ölçülen bütçe (bu laptop, üretim config'i tolerance=0,5 · voxel=0,1):

        | sahne | nokta → voxel | çift | süre |
        |---|---|---|---|
        | açık su, 8 kapı dubası | 49 → 35 | 95 | **0,2 ms** |
        | açık su + 10k su dönüşü | 10 049 → 9 828 | 30 206 | **8,5 ms** |
        | kapalı oda 8×6×3 m, 20k | 20 000 → 9 686 | 298 299 | **26,4 ms** |
        | 5 m'lik kapalı hacim (en kötü) | 15 628 | 1 372 707 | **112 ms** |

        🔑 Darboğaz nokta sayısı DEĞİL, `query_pairs`'in döndürdüğü **çift
        sayısı**: yoğunluk arttıkça nokta başına komşu 1,2 → 176'ya çıkıyor ve
        süre onunla büyüyor. Açık suda ışınların çoğu dönmediği için sorun
        yok (0,2-8,5 ms, bütçenin 12-500 katı altında); **risk kıyıya yakın
        olduğumuz an** — yani başlangıç noktası.

        ⚠ Tezgahtaki 1-3,3 s bu makinede ÜRETİLEMEDİ (en kötü 112 ms). Fark
        büyük olasılıkla Jetson'ın CPU'su + o an koşan diğer node'lar + gerçek
        odanın modelimden yoğun olması. Yani **gerçek sayı Jetson'da
        ölçülmeli** — bu log tam onun için var. İlk su testinde bak:
        `journalctl -u girdap-karar | grep "kümeleme"`.
        """
        now = self._saat()
        if self._last_log_t is None or now - self._last_log_t >= self._log_period_s:
            self._last_log_t = now
            butce_ms = 1000.0 / max(self._beklenen_hz, 1e-6)
            uyari = ""
            if sure_ms > butce_ms:
                uyari = (
                    f" 🔴 BÜTÇE AŞILDI (>{butce_ms:.0f} ms): obstacle_map GEÇ "
                    "varır, füzyon eşleşmesi kesilebilir — voxel_size açık mı, "
                    "kıyıya/rıhtıma yakın mıyız kontrol et"
                )
            self.get_logger().info(
                f"tespit: {n_obstacles} engel · {n_points} nokta · "
                f"kümeleme {sure_ms:.1f} ms{uyari}"
            )


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node = PerceptionLidarNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
