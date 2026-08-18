"""
Girdap İDA — SAHTE HAM SENSÖR: sanal gölün ideal algısını LiDAR bulutu +
kamera karesine geri çevirir.

🔑 **Neden gerekli.** `sanal_gol.py` `/perception/obstacle_map` ve
`/perception/classified_obstacles` topic'lerini DOĞRUDAN yayınlar; yani üç
gerçek algı düğümü (`perception_lidar_node`, `perception_camera_node`,
`perception_fusion_node`) sanal göl koşumunda **hiç çalışmaz**. `yarisma_
simulasyonu.py` de aynı baypası yapar (kendi docstring'i söylüyor). Sonuç:
bugüne kadarki bütün yazılım koşumları algı zincirini ATLAMIŞTIR — kümeleme,
renk segmentasyonu ve bearing füzyonu yalnız birim testlerde koşmuştur, canlı
boru hattında hiç.

Bu düğüm o boşluğu kapatır. Sanal gölün **yer gerçeği** çıktısını (remap ile
`/gercek/...` altına alınmış) girdi olarak alır ve gerçek sensörlerin
üreteceği HAM veriyi sentezler:

    /gercek/obstacle_map (PoseArray, base_link, orientation.z=yarıçap)
    /gercek/classified_obstacles (Detection3DArray, sınıf etiketi)
             │
             ├─► /livox/lidar        (PointCloud2, silindirik duba yüzeyi
             │                        + su yüzeyi yansıması)  10 Hz
             └─► /oak/rgb/image_raw  (Image bgr8, iğne-deliği izdüşümü,
                                      sınıfa göre RAL rengi)  10 Hz

Böylece zincir kapanır:

    ham LiDAR → perception_lidar_node   → /perception/obstacle_map
    ham kamera → perception_camera_node → /perception/buoys
                 perception_fusion_node → /perception/classified_obstacles
                                              ↓
                                        planning_node (kapı takibi)

**İzdüşüm fiziksel olarak doğru kurulur** (iğne deliği, optik eksen +x, görüntü
sağı −y): yani `bearing_from_camera`'nın F6.1'de düzeltilen "sol pozitif"
kuralını DOĞRULAR, ona uydurmaz. Üreteç dedektörün ters fonksiyonu olsaydı
F6.1'in maskelediği hatanın aynısı geri gelirdi (bkz. fusion.py:115).

Çalıştır (sanal_gol çıktıları remap'li başlatılmış olmalı):
    python3 scripts/sahte_ham_sensor.py
"""

from __future__ import annotations

import math
import sys

import cv2
import numpy as np

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseArray
from sensor_msgs.msg import Image, PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header
from vision_msgs.msg import Detection3DArray

from girdap_decision.qos_profiles import sensor_data_qos

# camera_buoys varsayılan HSV aralıklarının İÇİNDE kalan BGR renkler
# (synthetic_camera.py ile aynı sabitler — tek kaynak orası, burada yalnız
# sınıf→renk eşlemesi var).
_SINIF_BGR = {
    "0": (0, 140, 255),     # turuncu, RAL 2003 — parkur kenarı (kapı direği)
    "1": (0, 220, 255),     # sarı,    RAL 1026 — engel
    "3": (0, 0, 220),       # kırmızı
    "4": (0, 150, 0),       # yeşil
    # 🔴 18.08.2026: sınıf 5 KAHVERENGİ değil **SİYAH** (RAL 9005). Şartname
    # s.18 hedef renkleri RAL 3026/6037/9005; "kahverengi" hiçbir maddede
    # geçmiyor. `kamikaze_hedef.py` 18.08'de düzeltildi ama gölün sahte
    # kamerası eski rengi basmaya devam ediyordu ⇒ hakem "siyah" dediğinde
    # göl YANLIŞ RENGİ gösteriyordu (P3 zinciri sahte veriyle sınanır).
    "5": (28, 28, 28),      # siyah, RAL 9005 (mat siyah ~#282828)
}
_SU_BGR = (110, 70, 20)
_BILINMEYEN = "99"

_LIDAR_GURULTU_M = 0.02      # Livox Mid-360 ~2 cm menzil gürültüsü
_DUBA_YUKSEKLIK_M = 0.5      # su üstü görünür yükseklik (şartname)


class SahteHamSensor(Node):
    def __init__(self) -> None:
        super().__init__("sahte_ham_sensor")

        self.declare_parameter("kamera_genislik_px", 1280)
        self.declare_parameter("kamera_yukseklik_px", 720)
        self.declare_parameter("kamera_hfov_rad", 1.2)
        self.declare_parameter("kamera_yukseklik_m", 0.28)   # oak_frame z
        self.declare_parameter("nokta_sayisi_duba", 60)
        self.declare_parameter("su_gurultu_nokta", 300)
        self.declare_parameter("seed", 7)

        self._W = int(self.get_parameter("kamera_genislik_px").value)
        self._H = int(self.get_parameter("kamera_yukseklik_px").value)
        self._hfov = float(self.get_parameter("kamera_hfov_rad").value)
        self._kam_z = float(self.get_parameter("kamera_yukseklik_m").value)
        self._n_duba = int(self.get_parameter("nokta_sayisi_duba").value)
        self._n_su = int(self.get_parameter("su_gurultu_nokta").value)
        self._rng = np.random.default_rng(int(self.get_parameter("seed").value))

        # İğne deliği odak uzaklığı — kare oranı korunur (fx = fy).
        self._fx = (self._W / 2.0) / math.tan(self._hfov / 2.0)
        self._cx = self._W / 2.0
        self._cy = self._H / 2.0

        self._son_engeller: list = []      # (bx, by, yaricap)
        self._son_siniflar: dict = {}      # (yuvarlanmış bx,by) → sınıf

        self._pub_cloud = self.create_publisher(
            PointCloud2, "/livox/lidar", sensor_data_qos()
        )
        self._pub_img = self.create_publisher(
            Image, "/oak/rgb/image_raw", sensor_data_qos()
        )
        self.create_subscription(
            PoseArray, "/gercek/obstacle_map", self._on_gercek_engel, 10
        )
        self.create_subscription(
            Detection3DArray, "/gercek/classified_obstacles",
            self._on_gercek_sinif, 10,
        )
        self.create_timer(0.1, self._tick)      # 10 Hz — Livox/OAK kadansı

        self.get_logger().info(
            f"sahte_ham_sensor aktif: /gercek/* → /livox/lidar + "
            f"/oak/rgb/image_raw ({self._W}x{self._H}, "
            f"HFOV {math.degrees(self._hfov):.1f}°, fx={self._fx:.1f} px)"
        )

    # ------------------------------------------------------------ yer gerçeği

    def _on_gercek_engel(self, msg: PoseArray) -> None:
        self._son_engeller = [
            (p.position.x, p.position.y, abs(p.orientation.z))
            for p in msg.poses
        ]

    def _on_gercek_sinif(self, msg: Detection3DArray) -> None:
        siniflar = {}
        for d in msg.detections:
            if not d.results:
                continue
            anahtar = (
                round(d.bbox.center.position.x, 1),
                round(d.bbox.center.position.y, 1),
            )
            siniflar[anahtar] = d.results[0].hypothesis.class_id
        self._son_siniflar = siniflar

    def _sinif_of(self, bx: float, by: float) -> str:
        return self._son_siniflar.get((round(bx, 1), round(by, 1)), _BILINMEYEN)

    # ------------------------------------------------------------- üreteçler

    def _bulut_uret(self) -> np.ndarray:
        """Her engelin silindirik yüzeyinden nokta örnekle + su gürültüsü."""
        parcalar = []
        for (bx, by, yaricap) in self._son_engeller:
            n = self._n_duba
            theta = self._rng.uniform(0.0, 2.0 * math.pi, n)
            z = self._rng.uniform(0.05, _DUBA_YUKSEKLIK_M, n)
            noktalar = np.column_stack((
                bx + yaricap * np.cos(theta),
                by + yaricap * np.sin(theta),
                z,
            ))
            parcalar.append(
                noktalar + self._rng.normal(0.0, _LIDAR_GURULTU_M, noktalar.shape)
            )

        # Su yüzeyi yansıması — z_min=0.1 filtresinin ELEMESİ gereken negatif
        # örnek (filtre gerçekten koşuyor mu, canlı doğrulanır).
        su = np.column_stack((
            self._rng.uniform(1.0, 25.0, self._n_su),
            self._rng.uniform(-12.0, 12.0, self._n_su),
            self._rng.uniform(-0.05, 0.08, self._n_su),
        ))
        parcalar.append(su)

        if not parcalar:
            return np.zeros((0, 3), dtype=np.float32)
        return np.vstack(parcalar).astype(np.float32)

    def _kare_uret(self) -> np.ndarray:
        """İğne deliği izdüşümü — optik eksen +x, görüntü sağı −y, aşağı −z.

        Uzak duba önce çizilir ki yakın duba onu KAPATSIN (z-tamponu yerine
        ressam algoritması; gerçek kameranın oklüzyonunu taklit eder)."""
        kare = np.full((self._H, self._W, 3), _SU_BGR, dtype=np.uint8)

        gorunur = [
            (bx, by, r) for (bx, by, r) in self._son_engeller if bx > 0.3
        ]
        for (bx, by, yaricap) in sorted(gorunur, key=lambda o: -o[0]):
            u = self._cx - self._fx * (by / bx)
            # Duba merkezi su üstü ~yarı yükseklikte; kamera _kam_z'de.
            v = self._cy + self._fx * ((self._kam_z - _DUBA_YUKSEKLIK_M / 2.0) / bx)
            r_px = self._fx * yaricap / bx
            if r_px < 1.0:
                continue                       # menzil dışı — çizilmez
            if u < -r_px or u > self._W + r_px:
                continue                       # kadraj dışı
            renk = _SINIF_BGR.get(self._sinif_of(bx, by))
            if renk is None:
                continue                       # sınıfsız (99) → kamera görmez
            cv2.circle(kare, (int(round(u)), int(round(v))),
                       max(1, int(round(r_px))), renk, thickness=-1)

        gurultulu = kare.astype(np.int16) + self._rng.normal(0.0, 6.0, kare.shape)
        return np.clip(gurultulu, 0, 255).astype(np.uint8)

    # ----------------------------------------------------------------- tick

    def _tick(self) -> None:
        if not self._son_engeller:
            return
        simdi = self.get_clock().now().to_msg()

        h = Header()
        h.stamp = simdi
        h.frame_id = "base_link"
        noktalar = self._bulut_uret()
        self._pub_cloud.publish(
            point_cloud2.create_cloud_xyz32(h, noktalar.tolist())
        )

        kare = self._kare_uret()
        img = Image()
        img.header.stamp = simdi
        img.header.frame_id = "oak_frame"
        img.height, img.width = self._H, self._W
        img.encoding = "bgr8"
        img.is_bigendian = 0
        img.step = self._W * 3
        img.data = kare.tobytes()
        self._pub_img.publish(img)


def main() -> None:
    rclpy.init(args=sys.argv[1:])
    n = SahteHamSensor()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    finally:
        n.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
