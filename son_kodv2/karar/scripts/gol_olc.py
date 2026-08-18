#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""ÖLÇME GÖLÜ — algı→karar zincirinin Hz / ms / sınıf ölçümü (18.08.2026).

    python3 scripts/gol_olc.py [isinma_s] [olcum_s]      # varsayılan 20 / 60

## Neden bu araç
*"Kapıdan geçmiyor"* sorusunun cevabı tek bir düğümde değil, **zincirde**:

    /perception/obstacle_map (LiDAR)  ─┐
                                        ├─ ApproximateTimeSynchronizer(slop)
    /perception/buoys        (kamera) ─┘        │
                                                ▼
                            /perception/classified_obstacles
                                                │  GateFollower
                                                ▼
                                    /girdap/planning/gate

Halkalardan biri koparsa araç **ham GPS noktasına** gider ve kapıdan geçmez —
üstelik hiçbir hata basılmadan. Bu araç her halkayı ayrı ayrı ölçer.

## Ölçüm yöntemi (literatür + bu projede yakalanan hatalar)
- **Tek, kalıcı, PASİF abone.** `ros2 topic echo --once` ile örneklemek her
  çağrıda ayrı düğüm kurup yıkar; probe etkisi ölçümü bozar (Mytkowicz ve
  ark.: %3'ten az komut artışı bile izi güvenilmez kılabiliyor). Ölçüldü:
  aynı kural echo ile %13,6, bu tezgâhla %2,7 ihlal verdi.
- **Isınma atılır** — açılış rejimi kararlı hâli temsil etmez.
- **YÜZDELİK, ortalama değil** (p50/p95). ROS 2 gecikme yazınının standardı
  (Kronauer ve ark., arXiv:2101.02074).
- **Bitişi düğümün KENDİ timer'ı zamanlar** — dış `timeout` SIGTERM'i
  `finally`'yi atlar ve ölçüm kaybolur (ölçüldü).
- **Yayıncı sayısı denetlenir.** Hayalet düğüm hızı ikiye katlar; 18.08'de
  `/perception/buoys` yayıncı 2 iken 9,88 Hz ölçüldü, gerçek 4,94'tü.

## ⚠ Bu araç NE ÖLÇMEZ
Doğruluk ölçmez, **akış ve zamanlama** ölçer. Duba yerinin doğru olup
olmadığı, modelin isabeti, kapının gerçekten geçilip geçilmediği başka
yollardan sınanır (bant koşumu, kural motoru).
"""
from __future__ import annotations

import collections
import statistics as st
import sys

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray, PoseStamped
from vision_msgs.msg import Detection2DArray, Detection3DArray

#: Füzyonun `sync_slop_s` varsayılanı (params.yaml). Damga farkı bunu aşan
#: kare çifti EŞLEŞMEZ ⇒ o kare `classified_obstacles`'a hiç girmez.
SLOP_MS = 100.0
#: `fusion.py` güvenlik sınıfı: eşleşmeyen LiDAR kümesi atılmaz, UNKNOWN kalır.
CLASS_UNKNOWN = "99"


class OlcmeGolu(Node):
    def __init__(self, isinma_s: float, olcum_s: float) -> None:
        super().__init__("gol_olc")
        self.isinma_s = isinma_s
        self.t0 = self.saat()
        self.alis: dict[str, list[float]] = collections.defaultdict(list)
        self.damga: dict[str, float] = {}
        #: Her topic'in damga DİZİSİ. Eşleşme farkı "en son görülen iki
        #: damga"dan hesaplanamaz — o yalnız GELİŞ SIRASINI ölçer ve iki
        #: kaynak aynı hızda akarken 0↔periyot arasında salınır.
        #: (Ölçüldü: öyle hesaplayınca "slop'u aşan %49,9" çıktı, oysa
        #: `classified` girdiyle AYNI hızda akıyordu ⇒ füzyon her kareyi
        #: eşleştiriyordu. Yani sayı tezgâh kusuruydu.)
        #: ApproximateTimeSynchronizer'ın yaptığı şey EN YAKIN damgayı
        #: bulmaktır; ölçüm de onu yapmalı.
        self.damga_dizi: dict[str, list[float]] = collections.defaultdict(list)
        self.fark_ms: list[float] = []
        self.uctan_uca_ms: list[float] = []
        self.sinif: collections.Counter = collections.Counter()
        self.tespit_n: list[int] = []

        self.create_subscription(PoseArray, "/perception/obstacle_map",
                                 lambda m: self.gelen("omap", m), 10)
        self.create_subscription(Detection2DArray, "/perception/buoys",
                                 lambda m: self.gelen("buoys", m), 10)
        self.create_subscription(Detection3DArray,
                                 "/perception/classified_obstacles",
                                 self.siniflanan, 10)
        self.create_subscription(PoseStamped, "/girdap/planning/gate",
                                 lambda m: self.gelen("kapi", m), 10)
        self.create_timer(isinma_s + olcum_s, self.bitir)

    def saat(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    @staticmethod
    def stamp_s(msg) -> float:
        return msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

    def gelen(self, ad: str, msg) -> None:
        if self.saat() - self.t0 < self.isinma_s:
            return
        self.alis[ad].append(self.saat())
        self.damga[ad] = self.stamp_s(msg)
        if ad in ("buoys", "omap"):
            self.damga_dizi[ad].append(self.stamp_s(msg))

    def siniflanan(self, msg) -> None:
        self.gelen("classified", msg)
        if self.saat() - self.t0 < self.isinma_s:
            return
        self.tespit_n.append(len(msg.detections))
        if "omap" in self.damga:
            self.uctan_uca_ms.append(
                (self.stamp_s(msg) - self.damga["omap"]) * 1000.0)
        for d in msg.detections:
            for r in d.results:
                self.sinif[str(r.hypothesis.class_id)] += 1

    def bitir(self) -> None:
        raise KeyboardInterrupt


def yuzdelik(v: list[float], p: float) -> float:
    if not v:
        return float("nan")
    s = sorted(v)
    return s[min(len(s) - 1, max(0, int(p * len(s)) - 1))]


def eslesme_farki_ms(o: OlcmeGolu) -> list[float]:
    """Her LiDAR damgası için EN YAKIN kamera damgasına uzaklık (ms).

    ApproximateTimeSynchronizer'ın eşleştirme ölçütünün ta kendisi.
    """
    kam = sorted(o.damga_dizi.get("buoys", []))
    if not kam:
        return []
    import bisect
    cik = []
    for z in o.damga_dizi.get("omap", []):
        i = bisect.bisect_left(kam, z)
        adaylar = [kam[j] for j in (i - 1, i) if 0 <= j < len(kam)]
        if adaylar:
            cik.append(min(abs(z - k) for k in adaylar) * 1000.0)
    return cik


def rapor(o: OlcmeGolu) -> None:
    print(f"\nısınma {o.isinma_s:.0f} s atıldı\n")
    print(f"{'topic':13s} {'n':>5s} {'Hz':>7s} {'periyot p50':>12s} {'p95':>9s}")
    for ad in ("omap", "buoys", "classified", "kapi"):
        v = o.alis[ad]
        if len(v) < 3:
            print(f"{ad:13s} {len(v):5d}   🔴 AKMIYOR — zincir burada KOPUK")
            continue
        dt = [b - a for a, b in zip(v, v[1:])]
        print(f"{ad:13s} {len(v):5d} {1.0/st.mean(dt):7.2f} "
              f"{1000*st.median(dt):11.1f}ms {1000*yuzdelik(dt,0.95):8.1f}ms")

    a = eslesme_farki_ms(o)
    if a:
        asan = 100.0 * sum(1 for x in a if x > SLOP_MS) / len(a)
        print(f"\nkamera↔LiDAR damga farkı |Δ|:  p50 {st.median(a):6.1f} ms · "
              f"p95 {yuzdelik(a,0.95):6.1f} ms · max {max(a):6.1f} ms")
        print(f"  füzyon slop {SLOP_MS:.0f} ms → AŞAN örnek: {asan:5.1f} %"
              f"   {'🔴 eşleşme kaybı' if asan > 5 else '✅'}")
    if o.uctan_uca_ms:
        print(f"omap→classified damga gecikmesi: p50 "
              f"{st.median(o.uctan_uca_ms):6.1f} ms · "
              f"p95 {yuzdelik(o.uctan_uca_ms,0.95):6.1f} ms")
    if o.tespit_n:
        print(f"\nclassified tespit/mesaj: ort {st.mean(o.tespit_n):.1f}")
    if o.sinif:
        top = sum(o.sinif.values())
        bilinmeyen = o.sinif.get(CLASS_UNKNOWN, 0)
        print(f"sınıf dağılımı: {dict(o.sinif.most_common(6))}")
        print(f"  UNKNOWN(99) oranı: {100.0*bilinmeyen/top:5.1f} %"
              f"   {'🔴 füzyon eşleştiremiyor' if bilinmeyen/top > 0.25 else '✅'}")
        print("  ⚠ UNKNOWN engel olarak KALIR (güvenli) ama KENAR dubası "
              "SAYILMAZ ⇒ GateFollower'a daha az kapı dubası gider.")


def main() -> None:
    isinma = float(sys.argv[1]) if len(sys.argv) > 1 else 20.0
    olcum = float(sys.argv[2]) if len(sys.argv) > 2 else 60.0
    rclpy.init()
    o = OlcmeGolu(isinma, olcum)
    # Hayalet yayıncı denetimi — ölçümü sessizce ikiye katlayan sınıf.
    o.create_timer(isinma * 0.5, lambda: _yayinci_denetle(o))
    try:
        rclpy.spin(o)
    except BaseException:
        pass
    rapor(o)


def _yayinci_denetle(o: OlcmeGolu) -> None:
    for t in ("/perception/buoys", "/perception/obstacle_map"):
        n = o.count_publishers(t)
        if n > 1:
            print(f"🔴 {t}: YAYINCI {n} — hayalet düğüm var, ölçüm "
                  "GEÇERSİZ. `python3 scripts/gol_temizle.py` çalıştır.")
        elif n == 0:
            print(f"🔴 {t}: yayıncı YOK")


if __name__ == "__main__":
    main()
