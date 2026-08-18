#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""GİRDAP — DOĞRULAMA İZLEYİCİSİ (runtime verification monitor).

`prototype/dogrulama/` kural motorunu **canlı sisteme** bağlar. Kurallar
ROS'suz kalır; bu düğüm yalnız *mesajdan sayı çıkarıp* kurala verir ve
sonucu yayınlar.

## Mimari (NASA Ames / Ogma deseni — araştırıldı)

    veri topic'leri ──► İZLEYİCİ DÜĞÜM ──► /girdap/dogrulama (DiagnosticArray)
      (yalnız abone)         │            └► /girdap/dogrulama/<KURAL> (Float32 marj)
                             └─ yeni veri geldikçe yeniden değerlendirir

Üç tasarım kararı, üçü de gerekçeli:

1. **AYRI DÜĞÜM.** Sınanan sistemin içine kod enjekte edilmez ⇒ gözlemci
   sistemi bozamaz. Yalnız abone olur, hiçbir sistem topic'ine yayın yapmaz.
2. **OLAY GÜDÜMLÜ + periyodik.** Veri geldikçe önbellek tazelenir; kurallar
   sabit kadansta değerlendirilir (yoklama fırtınası olmasın).
3. **KURAL BAŞINA AYRI TOPIC.** Hangi kuralın yandığı topic adından belli
   olur; `rqt_plot` ile marjın zaman serisi doğrudan çizilir — "ne kadar
   payımız kaldı" sorusu grafikten okunur.

## Neden `DiagnosticArray`

ROS'un **standart** sağlık kanalı: `rqt_runtime_monitor` ve mevcut
`/diagnostics` tüketicileri ek iş olmadan görür. `DiagnosticStatus.values`
alanı marjı, birimi ve bağlamı taşır ⇒ özel mesaj paketi gerekmez.

## 🔴 GÖZLEMCİNİN KENDİSİ RİSKTİR
Literatürün açık uyarısı: *"RV alt sistemindeki hatalar görevin tamamını
tehdit eder."* Bu yüzden:
  · her kural çağrısı korumalı (`Kural.olc` patlarsa İHLAL döner, çökmez)
  · düğüm hiçbir sistem topic'ine YAZMAZ
  · veri hiç gelmediyse `STALE` der — "ihlal yok" DEMEZ
"""
from __future__ import annotations

import math
from typing import Callable, Dict, List, Optional, Sequence, Tuple

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.node import Node
from rclpy.time import Time as RclTime
from std_msgs.msg import Float32

from girdap_decision.saat_kaynagi import bayatlik_saati
from prototype.dogrulama import butce, canlilik, fizik, sozlesme
from prototype.dogrulama.kural import Kural, Sonuc, Tur

#: Kuralın hangi topic'lerden beslendiği + sayıyı çıkaran işlev.
#: `cikar(onbellek) -> tuple(args)` — None dönerse kural DEĞERLENDİRİLMEZ
#: (veri henüz yok; "ihlal yok" demek DEĞİL, "bilinmiyor" demek).
Baglanti = Tuple[Kural, Sequence[str], Callable[[dict], Optional[tuple]]]


def _yas(stamp, simdi: RclTime) -> float:
    """ROS damgasının yaşı (s). Damga 0 ise `nan` (doldurulmamış).

    🔑 ROS `Time` aritmetiği kullanılıyor, ham `.nanoseconds` farkı DEĞİL:
      · sim zamanında `get_clock()` `/clock`'u izler, çıkarma tutarlı kalır
      · damga ile "şimdi" AYNI saat tabanında karşılaştırılır — bu ölçüm
        bilerek DUVAR saatinde yapılır, çünkü damga da duvar saatidir.
        (Tek yönlü saatle duvar saatli damgayı çıkarmak, poz tamponunda
        bulduğumuz 57 yıllık taban hatasının ta kendisidir.)
    ⚠ Geçen SÜRE ölçümleri (bayatlık, sıfır itki süresi) bundan ayrı ve
      `bayatlik_saati` ile yapılır — ikisi farklı iştir.
    """
    if stamp.sec <= 0 and stamp.nanosec == 0:
        return math.nan
    return (simdi - RclTime.from_msg(stamp)).nanoseconds * 1e-9


class DogrulamaIzleyici(Node):
    """Kuralları canlı topic'lere bağlayan salt-okur izleyici."""

    def __init__(self) -> None:
        super().__init__("girdap_dogrulama")
        self.declare_parameter("degerlendirme_hz", 5.0)
        self.declare_parameter("bayat_esigi_s", 5.0)

        self._ob: Dict[str, object] = {}      # topic -> son mesaj
        self._t_son: Dict[str, float] = {}    # topic -> varış (monotonic)
        self._grup = MutuallyExclusiveCallbackGroup()
        self._bayat_esigi = float(self.get_parameter("bayat_esigi_s").value)
        # ⏱️ §0.61 TEK YÖNLÜ SAAT — düz monotonik saat DOĞRUDAN kullanılmaz:
        # sim zamanında (`use_sim_time`) `/clock`'u izlemez ve izleyici sahte
        # bayatlık üretir. Bu, poz tamponunda bulduğumuz saat tabanı hatasının
        # birebir aynı sınıfı; ekibin sözleşme testi bu düğümü de yakaladı.
        self._saat = bayatlik_saati(self)

        self._baglantilar: List[Baglanti] = self._baglantilari_kur()

        # Kural başına marj yayıncısı + toplu tanı kanalı
        self._pub_tani = self.create_publisher(DiagnosticArray, "/girdap/dogrulama", 10)
        self._pub_marj = {
            k.ad: self.create_publisher(Float32, f"/girdap/dogrulama/{k.ad}", 10)
            for k, _, _ in self._baglantilar
        }

        for topic, tip in self._abonelikler().items():
            self.create_subscription(
                tip, topic, self._yakala(topic), 10, callback_group=self._grup)

        hz = float(self.get_parameter("degerlendirme_hz").value)
        self.create_timer(1.0 / hz, self._degerlendir, callback_group=self._grup)
        self.get_logger().info(
            f"doğrulama izleyicisi aktif · {len(self._baglantilar)} kural · "
            f"{len(self._abonelikler())} topic · {hz:.0f} Hz")

    # ───────────────────────────── abonelikler ─────────────────────────────
    def _abonelikler(self) -> Dict[str, type]:
        """İzlenecek topic → mesaj tipi. **Karar VE algı** tarafı birlikte."""
        from geometry_msgs.msg import PoseArray
        from nav_msgs.msg import Odometry
        from std_msgs.msg import Float32MultiArray, String
        from vision_msgs.msg import Detection2DArray, Detection3DArray

        return {
            # ── karar tarafı ──
            "/girdap/fusion/odom": Odometry,
            "/girdap/control/thrust": Float32MultiArray,
            "/girdap/mission/state": String,
            "/perception/obstacle_map": PoseArray,
            "/perception/classified_obstacles": Detection3DArray,
            # ── algı tarafı (bizim düğüm) ──
            "/perception/buoys": Detection2DArray,
            "/perception/buoys_3d": PoseArray,
        }

    def _yakala(self, topic: str):
        def _cb(msg):
            self._ob[topic] = msg
            self._t_son[topic] = self._saat()
        return _cb

    # ───────────────────────────── bağlantılar ─────────────────────────────
    def _baglantilari_kur(self) -> List[Baglanti]:
        """Kural ↔ topic ↔ çıkarıcı tablosu. Yeni kural = yeni SATIR, kod değil."""
        ob = self._ob

        def _odom_hiz(_):
            m = ob.get("/girdap/fusion/odom")
            if m is None:
                return None
            return (m.twist.twist.linear.x,)

        def _odom_donus(_):
            m = ob.get("/girdap/fusion/odom")
            return None if m is None else (m.twist.twist.angular.z,)

        def _odom_sonlu(_):
            m = ob.get("/girdap/fusion/odom")
            if m is None:
                return None
            p = m.pose.pose.position
            return (p.x, p.y, m.twist.twist.linear.x)

        def _odom_sicrama(_):
            m = ob.get("/girdap/fusion/odom")
            if m is None:
                return None
            p = m.pose.pose.position
            onceki = getattr(self, "_onceki_poz", None)
            t = self._t_son.get("/girdap/fusion/odom", 0.0)
            self._onceki_poz = (p.x, p.y, t)
            if onceki is None or t <= onceki[2]:
                return None
            return (p.x - onceki[0], p.y - onceki[1], t - onceki[2])

        def _buoys_damga(_):
            m = ob.get("/perception/buoys")
            if m is None:
                return None
            return (_yas(m.header.stamp, self.get_clock().now()),)

        def _buoys_sinif(_):
            m = ob.get("/perception/buoys")
            if m is None:
                return None
            ids = [d.results[0].hypothesis.class_id
                   for d in m.detections if d.results]
            return (ids,)

        def _buoys_cerceve(_):
            m = ob.get("/perception/buoys")
            return None if m is None else (m.header.frame_id, ["oak_rgb", "base_link"])

        def _engel_cerceve(_):
            m = ob.get("/perception/obstacle_map")
            return None if m is None else (m.header.frame_id, ["base_link"])

        def _engel_govde(_):
            m = ob.get("/perception/obstacle_map")
            if m is None or not m.poses:
                return None
            return (min(math.hypot(p.position.x, p.position.y) for p in m.poses),)

        def _itki_sifir(_):
            m = ob.get("/girdap/control/thrust")
            if m is None or not m.data:
                return None
            simdi = self._saat()
            sifir = all(abs(v) < 1e-6 for v in m.data)
            if not sifir:
                self._sifir_basi = None
                return (0.0,)
            if getattr(self, "_sifir_basi", None) is None:
                self._sifir_basi = simdi
            return (simdi - self._sifir_basi,)

        def _durum_sure(_):
            m = ob.get("/girdap/mission/state")
            if m is None:
                return None
            d = m.data
            simdi = self._saat()
            if getattr(self, "_durum_son", None) != d:
                self._durum_son, self._durum_basi = d, simdi
            return (simdi - getattr(self, "_durum_basi", simdi), d)

        def _topic_akis(topic: str, periyot: float):
            def _f(_):
                t = self._t_son.get(topic)
                if t is None:
                    return None
                return (self._saat() - t, periyot)
            return _f

        return [
            (fizik.F1, ["/girdap/fusion/odom"], _odom_sicrama),
            (fizik.F2, ["/girdap/fusion/odom"], _odom_hiz),
            (fizik.F2R, ["/girdap/fusion/odom"], _odom_donus),
            (fizik.F4, ["/girdap/fusion/odom"], _odom_sonlu),
            (fizik.F5, ["/perception/obstacle_map"], _engel_govde),
            (sozlesme.S1, ["/perception/buoys"], _buoys_damga),
            (sozlesme.S2, ["/perception/buoys"], _buoys_sinif),
            (sozlesme.S5, ["/perception/buoys"], _buoys_cerceve),
            (canlilik.C1, ["/girdap/control/thrust"], _itki_sifir),
            (canlilik.C2, ["/girdap/mission/state"], _durum_sure),
            (canlilik.C3, ["/perception/buoys"],
             _topic_akis("/perception/buoys", 0.125)),   # 8 FPS NN
            (canlilik.C3, ["/girdap/fusion/odom"],
             _topic_akis("/girdap/fusion/odom", 0.10)),  # 10 Hz füzyon
        ]

    # ───────────────────────────── değerlendirme ────────────────────────────
    def _degerlendir(self) -> None:
        arr = DiagnosticArray()
        arr.header.stamp = self.get_clock().now().to_msg()
        for kural, topicler, cikar in self._baglantilar:
            arr.status.append(self._tek(kural, topicler, cikar))
        self._pub_tani.publish(arr)

    def _tek(self, kural: Kural, topicler: Sequence[str],
             cikar: Callable) -> DiagnosticStatus:
        st = DiagnosticStatus(name=f"girdap/{kural.ad}", hardware_id=kural.tur.value)

        # Veri hiç gelmediyse / bayatsa: STALE. "İhlal yok" DEMEZ.
        simdi = self._saat()
        eksik = [t for t in topicler if t not in self._t_son]
        bayat = [t for t in topicler
                 if t in self._t_son and simdi - self._t_son[t] > self._bayat_esigi]
        if eksik or bayat:
            st.level = DiagnosticStatus.STALE
            st.message = ("veri yok: " + ", ".join(eksik)) if eksik \
                else ("bayat: " + ", ".join(bayat))
            return st

        args = cikar(self._ob)
        if args is None:
            st.level = DiagnosticStatus.STALE
            st.message = "değer henüz çıkarılamadı"
            return st

        s: Sonuc = kural.olc(*args)
        self._pub_marj[kural.ad].publish(Float32(data=float(s.marj)))

        if s.ihlal:
            st.level = (DiagnosticStatus.ERROR if kural.tur is Tur.ABORT
                        else DiagnosticStatus.WARN)
            st.message = f"İHLAL {s.marj:+.4g} {s.birim} — {kural.aciklama}"
            self.get_logger().warn(f"🔴 {s}", throttle_duration_sec=5.0)
        else:
            st.level = DiagnosticStatus.OK
            st.message = f"marj {s.marj:+.4g} {s.birim}"
        st.values = [
            KeyValue(key="marj", value=f"{s.marj:.6g}"),
            KeyValue(key="birim", value=s.birim),
            KeyValue(key="tur", value=kural.tur.value),
            KeyValue(key="kaynak", value=kural.kaynak),
        ]
        return st


def main() -> None:
    rclpy.init()
    d = DogrulamaIzleyici()
    try:
        rclpy.spin(d)
    except KeyboardInterrupt:
        pass
    finally:
        d.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
