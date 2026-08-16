"""
Girdap İDA — LiDAR veri seti kaydedici (Layer 2). ŞARTNAME 4.2, md 487-493.

    "Diğer Otonomi Sensörleri Veri Seti: Kamera dışında kullanılan bir otonomi
     sensörü ya da sensörleri varsa (lidar vs. gibi), HER BİR SENSÖR TİPİ İÇİN
     AYRI AYRI olacak şekilde
        · En az 1 Hz
        · Her bir veri seti ZAMAN ETİKETİNE sahip olacak şekilde mp4 formatında
        · Tespit ve takip işlemleri sonucunda KÜMELEME, AYIRMA vs. gibi bir
          işlem yapıldıysa GÖRÜNECEK şekilde"

🔴 **BU DOSYA 07.08.2026'YA KADAR HİÇ ÜRETİLMİYORDU.** Livox Mid-360
kullandığımız için teslim ZORUNLU; eksik dosya = **5 ceza puanı**
(md 5.5.4.3.5). Denetimde tüm repoda (karar + algı) LiDAR için mp4 yazan tek
satır bulunamadı. Gösterilecek veri zaten yayınlanıyordu — eksik olan çiziciydi.

**"Kümeleme, ayırma görünecek şekilde" nasıl karşılanıyor** (çizim
`prototype.mapping.bev_renderer`, üç katman):
    1. ham nokta bulutu   → kümelemenin GİRDİSİ (madde bir *işlemin*
                            görünmesini istiyor; yalnız daireler çizilseydi
                            kümelemenin YAPILDIĞI görünmezdi)
    2. küme üyeliği       → küme başına AYRI renk  = "ayırma"
    3. sınıf + kimlik     → halka + "K3 KENAR" etiketi

Subscribed:
    /perception/classified_obstacles  vision_msgs/Detection3DArray (GÖVDE)
    /perception/obstacle_map          geometry_msgs/PoseArray      (GÖVDE)
        — sınıflı akış yoksa yedek kaynak (kümeler sınıfsız çizilir)
    /livox/lidar                      sensor_msgs/PointCloud2      (opsiyonel)
    /girdap/fusion/odom               nav_msgs/Odometry
Çıktı:
    ~/girdap_logs/lidar/<oturum>/lidar_kumeleme.mp4
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
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from vision_msgs.msg import Detection3DArray

from girdap_decision.qos_profiles import sensor_data_qos
from girdap_decision.sigterm_kapanis import sigterm_kapanisi_kur
from girdap_decision.saat_kaynagi import bayatlik_saati
from girdap_decision.yeniden_baslama import ResetAbonesi
from prototype.mapping.bev_renderer import (
    BevConfig, BevRenderer, Kume, Mp4Yazici, PngSerisiYazici,
)


def govde_to_dunya(
    px: float, py: float, arac: Tuple[float, float], psi: float
) -> Tuple[float, float]:
    """GÖVDE (x=ileri) → DÜNYA ENU. Dönüşüm + öteleme.

    ⚠️ **`planning_node._body_to_world` ile BİREBİR AYNI olmak zorunda.**
    Bu projenin iki kez yediği hata tam olarak "aynı dönüşümün iki kopyası
    ayrıştı"dır (GIRDAP_DURUM §0.0b). İkisinin uyuştuğu testle donduruldu
    (`test_lidar_kayit_node.py`). Yeni bir kopya AÇMA — buraya çağır.
    """
    c, s = math.cos(psi), math.sin(psi)
    return (arac[0] + px * c - py * s, arac[1] + px * s + py * c)


class LidarKayitNode(Node):
    """LiDAR kümeleme videosu — Şartname 4.2 "Diğer Otonomi Sensörleri"."""

    def __init__(self, **node_kwargs) -> None:
        super().__init__("lidar_kayit_node", **node_kwargs)
        # §0.61: bayatlık tek yönlü saatte. Kareye yakılan damga (_damga) ve
        # oturum klasörü adı duvar saatinde kalır — orası mutlak an.
        self._saat = bayatlik_saati(self)

        # ⚠ 2.0: "En Az 1 Hz"de tam sınırda koşmak, tek atlanan karede ihlal
        # demek (Dosya-3 ile aynı gerekçe).
        self.declare_parameter("dump_rate_hz", 2.0)
        self.declare_parameter("output_dir", "")
        self.declare_parameter("ham_bulut_enabled", True)
        # Ham bulut yalnız GÖRSELLEŞTİRME için; 20k nokta çizmek anlamsız ve
        # pahalı. Her N'inci nokta alınır (karar yoluna DOKUNMAZ).
        self.declare_parameter("ham_bulut_seyreltme", 25)
        self.declare_parameter("saat_guvenilir", True)
        self.declare_parameter("veri_timeout_s", 3.0)

        self._seyrelt = max(1, int(self.get_parameter("ham_bulut_seyreltme").value))
        self._saat_guvenilir = bool(self.get_parameter("saat_guvenilir").value)
        self._timeout = float(self.get_parameter("veri_timeout_s").value)

        out = str(self.get_parameter("output_dir").value) or None
        base = (
            Path(out).expanduser() if out
            else Path.home() / "girdap_logs" / "lidar"
        )
        self._base = base            # madde #11: yeniden baslamada yeni oturum
        self._session_dir = self._yeni_oturum_dizini()

        self._rend = BevRenderer(BevConfig())
        self._arac: Tuple[float, float] = (0.0, 0.0)
        self._psi: float = 0.0
        self._kumeler: List[Kume] = []
        self._ham: List[Tuple[float, float]] = []
        self._son_veri_t: Optional[float] = None
        self._son_stamp: str = "STAMP-YOK"
        self._siniflilar_geldi = False
        self._kare = 0
        self._stale_warned = False

        rate = float(self.get_parameter("dump_rate_hz").value)
        if rate < 1.0:
            raise ValueError(
                f"dump_rate_hz={rate} < 1 Hz — Şartname 4.2 'En az 1 Hz' "
                "ihlali; LiDAR teslimi geçersiz olur."
            )
        cfg = self._rend.cfg
        # 🔴 mp4 AÇILAMAZSA PNG SERİSİNE DÜŞ — teslimi kaybetme.
        # Jetson'ın OpenCV derlemesinde `mp4v` codec'i olmayabilir
        # (`ida_topics/kamera_kayit_node` bu riske karşı zaten F-P.11
        # koruması taşıyor → proje bunu daha önce yaşamış). Ekransız bir
        # makinede burada ölmek, LiDAR teslimini sessizce sıfırlar ve bu
        # ancak hakem masasında anlaşılır (md 5.5.4.3.5: 5 ceza).
        # PNG serisi sonradan `ffmpeg` ile mp4'e çevrilebildiği için
        # (zaman damgası zaten KAREYE yakılı) yedek yol teslimi KURTARIR.
        # `PngSerisiYazici` ile `Mp4Yazici` aynı arayüzü sunar; aşağıdaki
        # kayıt döngüsü hangisini tuttuğunu bilmek zorunda değildir.
        # madde #11: yeniden baslamada ayni yazicilar yeni dizine kurulacak.
        self._mp4_rate = rate
        self._mp4_boyut = (cfg.genislik_px, cfg.yukseklik_px)
        self._yedege_dusuldu = False
        try:
            self._mp4 = Mp4Yazici(
                self._session_dir / "lidar_kumeleme.mp4",
                fps=rate, boyut=(cfg.genislik_px, cfg.yukseklik_px),
            )
        except Exception as exc:                     # cv2 yok / codec yok
            self._yedege_dusuldu = True
            self._mp4 = PngSerisiYazici(
                self._session_dir / "lidar_kumeleme_png", fps=rate
            )
            self.get_logger().error(
                f"LiDAR mp4 AÇILAMADI ({exc!r}) → PNG serisine düşüldü: "
                f"{self._mp4.dizin}. Teslimden ÖNCE mp4'e çevirin — klasördeki "
                "NASIL_MP4_YAPILIR.txt tek satırlık ffmpeg komutunu veriyor. "
                "md 4.2 mp4 istiyor; çevrilmezse 5 ceza (md 5.5.4.3.5)."
            )

        self._reset = ResetAbonesi(self, self._yeniden_basla)

        # --- Subscribers ---
        self._sub_cls = self.create_subscription(
            Detection3DArray, "/perception/classified_obstacles",
            self._on_classified, 10,
        )
        self._sub_raw = self.create_subscription(
            PoseArray, "/perception/obstacle_map", self._on_obstacles, 10
        )
        self._sub_odom = self.create_subscription(
            Odometry, "/girdap/fusion/odom", self._on_odom, 10
        )
        if bool(self.get_parameter("ham_bulut_enabled").value):
            self.create_subscription(
                PointCloud2, "/livox/lidar", self._on_cloud, sensor_data_qos()
            )

        self._timer = self.create_timer(1.0 / rate, self._on_tick)
        self.get_logger().info(
            f"lidar_kayit_node aktif → {self._session_dir} "
            f"({rate} Hz, ham bulut seyreltme 1/{self._seyrelt}) "
            "— Şartname 4.2 'Diğer Otonomi Sensörleri' teslimi"
        )

    def _yeni_oturum_dizini(self):                     # noqa: ANN202
        """Zaman damgali yeni oturum dizini olustur ve dondur."""
        d = self._base / datetime.now().strftime("oturum_%Y%m%d_%H%M%S")
        d.mkdir(parents=True, exist_ok=True)
        return d

    def _yeniden_basla(self) -> None:
        """md 5.5.3.1 yeniden baslama — LiDAR teslimini YENI oturuma al.

        🔴 Neden yeni dizin: yeniden baslama hakki kullanilinca ilk turun
        puanlari sifirlanir, yani PUANLANAN kosu IKINCISI. Iki turun karelerini
        ayni mp4'te birlestirmek hakeme gecersiz turu de vermek olur.

        mp4 ONCE kapatilir: kapatilmadan birakilan dosya cogu codec'te
        oynatilamaz kalir (moov atomu yazilmaz) -> ilk turun kaydi da gider.
        md 5.5.4.3.5: eksik/gecersiz dosya = 5 ceza.

        ⚠ Yazici tipi (mp4 / PNG yedegi) ilk kurulumdaki secime SADIK kalir:
        acilista mp4 acilamadiysa codec yok demektir, ikinci turda da acilmaz.
        Bosuna denemek yerine dogrudan ayni yola devam ediliyor.
        """
        try:
            self._mp4.kapat()
        except Exception as exc:                       # noqa: BLE001
            self.get_logger().error(
                f"ilk tur LiDAR kaydi kapatilamadi: {exc!r} — dosya bozuk olabilir"
            )
        self._kare = 0
        self._session_dir = self._yeni_oturum_dizini()
        if self._yedege_dusuldu:
            self._mp4 = PngSerisiYazici(
                self._session_dir / "lidar_kumeleme_png", fps=self._mp4_rate
            )
        else:
            self._mp4 = Mp4Yazici(
                self._session_dir / "lidar_kumeleme.mp4",
                fps=self._mp4_rate, boyut=self._mp4_boyut,
            )
        self.get_logger().warn(
            f"LiDAR teslimi YENI OTURUM (md 5.5.3.1): {self._session_dir}"
        )

    # ----- callback'ler -----

    def _on_odom(self, msg: Odometry) -> None:
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        self._arac = (float(p.x), float(p.y))
        self._psi = 2.0 * math.atan2(float(q.z), float(q.w))

    def _on_classified(self, msg: Detection3DArray) -> None:
        """Sınıflı akış — birincil kaynak (sınıf + kimlik burada)."""
        self._siniflilar_geldi = True
        self._son_veri_t = self._now()
        self._son_stamp = self._damga(msg.header)
        kumeler: List[Kume] = []
        for i, det in enumerate(msg.detections):
            sinif = None
            if det.results:
                try:
                    sinif = int(det.results[0].hypothesis.class_id)
                except (TypeError, ValueError):
                    sinif = None
            c = det.bbox.center.position
            merkez = govde_to_dunya(float(c.x), float(c.y), self._arac, self._psi)
            kumeler.append(Kume(
                merkez=merkez,
                yaricap=abs(float(det.bbox.size.x)) / 2.0 or 0.15,
                sinif=sinif,
                kume_id=i,
            ))
        self._kumeler = kumeler

    def _on_obstacles(self, msg: PoseArray) -> None:
        """Sınıfsız yedek — sınıflı akış HİÇ gelmediyse kullanılır.

        ⚠ `planning_node`'daki tek yönlü mandalın (`_classified_seen`) aksine
        burada mandal YOK: bu bir KAYIT node'udur, sınıflı akış görev
        ortasında düşerse kaydın da susması teslimi öldürürdü. Kayıtta
        "sınıfsız küme" yazmak, hiç kare yazmamaktan iyidir.
        """
        if self._siniflilar_geldi and not self._veri_bayat():
            return
        self._son_veri_t = self._now()
        self._son_stamp = self._damga(msg.header)
        self._kumeler = [
            Kume(
                merkez=govde_to_dunya(float(p.position.x), float(p.position.y),
                                      self._arac, self._psi),
                yaricap=abs(float(p.orientation.z)) or 0.15,
                sinif=None,
                kume_id=i,
            )
            for i, p in enumerate(msg.poses)
        ]

    def _on_cloud(self, msg: PointCloud2) -> None:
        """Ham bulut — YALNIZ çizim için, seyreltilmiş.

        ⚠ `read_points_numpy` KULLANILMAZ: gerçek Livox alanları karışık
        tiptedir (x/y/z float32, tag/line uint8, timestamp float64) ve o
        fonksiyon hepsinin aynı tip olmasını `assert` eder — sahada tam bu
        yüzden patlamıştı (GIRDAP_DURUM §11.3/1).
        """
        try:
            from sensor_msgs_py import point_cloud2

            s = point_cloud2.read_points(
                msg, field_names=("x", "y", "z"), skip_nans=True
            )
            xs, ys = np.asarray(s["x"]), np.asarray(s["y"])
        except Exception as exc:                       # kayıt teslimi ölmesin
            self.get_logger().warn(
                f"ham bulut okunamadı, kümeler yine çizilecek: {exc!r}",
                throttle_duration_sec=10.0)
            return
        xs, ys = xs[:: self._seyrelt], ys[:: self._seyrelt]
        self._ham = [
            govde_to_dunya(float(x), float(y), self._arac, self._psi)
            for x, y in zip(xs, ys)
        ]

    # ----- yardımcılar -----

    def _now(self) -> float:
        """Bayatlık saati — TEK YÖNLÜ (§0.61). Mutlak an olarak kullanılmaz."""
        return self._saat()

    def _veri_bayat(self) -> bool:
        if self._timeout <= 0.0 or self._son_veri_t is None:
            return False
        return (self._now() - self._son_veri_t) > self._timeout

    @staticmethod
    def _damga(header) -> str:
        """Kareye yakılacak zaman — VERİNİN kendi stamp'i (duvar saati değil)."""
        t = float(header.stamp.sec) + float(header.stamp.nanosec) * 1e-9
        if t <= 0.0:
            return "STAMP-YOK"
        return (
            datetime.fromtimestamp(t, tz=timezone.utc)
            .strftime("%Y-%m-%dT%H:%M:%S.%f")[:-3] + "Z"
        )

    # ----- kayıt döngüsü -----

    def _on_tick(self) -> None:
        if self._son_veri_t is None:                   # henüz algı gelmedi
            return
        if self._veri_bayat():
            if not self._stale_warned:
                self._stale_warned = True
                self.get_logger().error(
                    "algı yayını kesildi — LiDAR teslimi DONMUŞ kare yazmaya "
                    "devam ediyor (kareye yakılı zaman damgası bunu ele verir)"
                )
        elif self._stale_warned:
            self._stale_warned = False
            self.get_logger().info("algı yayını geri geldi")

        kare = self._rend.render_lidar(
            self._arac, yaw=self._psi,
            ham_noktalar=self._ham, kumeler=self._kumeler,
            zaman_metni=self._son_stamp, kare_no=self._kare,
            saat_guvenilir=self._saat_guvenilir,
        )
        if not self._mp4.yaz(kare):
            self.get_logger().error(
                "LiDAR mp4 karesi yazılamadı (disk dolu olabilir)",
                throttle_duration_sec=5.0)
        self._kare += 1
        if self._kare % 20 == 1:
            if self._yedege_dusuldu:
                # ⚠ Açılıştaki tek ERROR journal selinde kaybolur; yedek yola
                # düşüldüyse operatör bunu koşum boyunca görmeli, yoksa
                # teslimden önce çevirmeyi unutur.
                self.get_logger().error(
                    f"[LiDAR veri seti] PNG YEDEĞİNDE {self._kare} kare — "
                    "TESLİMDEN ÖNCE mp4'E ÇEVİRİN (NASIL_MP4_YAPILIR.txt)"
                )
            else:
                self.get_logger().info(
                    f"[LiDAR veri seti] {self._kare} kare, "
                    f"{len(self._kumeler)} küme → {self._session_dir.name}"
                )

    def destroy_node(self) -> bool:
        """mp4'ü KAPAT — kapatılmayan dosya oynatılamaz (moov atomu yazılmaz)."""
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
    node = LidarKayitNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
