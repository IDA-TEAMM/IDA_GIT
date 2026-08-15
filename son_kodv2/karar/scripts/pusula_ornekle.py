#!/usr/bin/env python3
"""
GİRDAP İDA — PUSULA ÖRNEKLEYİCİ (salt okur, §1.07j/0 için)

Neden var (15.08.2026):
    Canlı nöbetçinin `PUSULA-TUTARSIZ` kuralı EŞİK tabanlı: 35°'yi geçince
    bağırır, altına inince susar. Karada taşınırken 1 saniye içinde 49° → 28°
    gidip geldi — yani tek alarm satırı HÜKÜM VERDİRMEZ.

    Bu betik eşiği değil DAĞILIMI kaydeder: pusula başlığı ↔ GPS gidiş yönü
    farkını, hız ve uydu sayısıyla birlikte, saniyede 5 kez CSV'ye yazar.
    Gölde düz sürüşten sonra ortanca/p95 hesaplanır; §0.91'in 41°'lik hatası
    GERÇEKTEN duruyor mu, ancak o zaman söylenebilir.

⚠ SALT OKUR. Hiçbir konuya yayın yapmaz, uçuş kontrolcüsüne yazmaz.

Kullanım:
    source /opt/ros/humble/setup.bash && source ~/ros2_ws/install/setup.bash
    ROS_DOMAIN_ID=42 python3 pusula_ornekle.py
    # çıktı: ~/girdap_logs/pusula/pusula_YYYYMMDD_HHMMSS.csv

Çözümleme (koşum sonrası):
    python3 pusula_ornekle.py --ozet <csv>
"""

from __future__ import annotations

import csv
import math
import sys
from datetime import datetime
from pathlib import Path

# ── Eşikler ────────────────────────────────────────────────────────────────
MIN_HIZ = 0.4        # m/s — altında GPS gidiş yönü gürültüdür (nöbetçiyle aynı)
YAZ_HZ = 5.0

CIKTI_DIZIN = Path.home() / "girdap_logs" / "pusula"


def _aci_farki(a: float, b: float) -> float:
    """İki başlık arasındaki en kısa fark, [-180, 180]."""
    return (a - b + 180.0) % 360.0 - 180.0


# ══ ÖZET KİPİ — ROS gerektirmez ════════════════════════════════════════════
def ozetle(yol: str) -> int:
    farklar: list[float] = []
    hizlar: list[float] = []
    with open(yol, newline="") as f:
        for satir in csv.DictReader(f):
            if satir.get("gecerli") != "1":
                continue
            farklar.append(abs(float(satir["fark_der"])))
            hizlar.append(float(satir["hiz_ms"]))

    if not farklar:
        print("Geçerli örnek YOK — tekne 0,4 m/s üstünde hiç düz gitmemiş.")
        return 1

    farklar.sort()
    n = len(farklar)

    def yuzdelik(p: float) -> float:
        return farklar[min(n - 1, int(p * n))]

    print(f"geçerli örnek : {n}  ({n / YAZ_HZ:.0f} saniye sürüş)")
    print(f"hız           : ortanca {sorted(hizlar)[n // 2]:.2f} m/s")
    print(f"|fark| ortanca: {yuzdelik(0.50):.1f}°")
    print(f"|fark| p95    : {yuzdelik(0.95):.1f}°")
    print(f"|fark| azami  : {farklar[-1]:.1f}°")
    ort = yuzdelik(0.50)
    print()
    if ort < 10.0:
        print("✅ HÜKÜM: pusula SAĞLIKLI (ortanca < 10°) — kalibrasyon gerekmez.")
    elif ort < 20.0:
        print("🟡 HÜKÜM: sınırda — MAGFit önerilir ama acil değil.")
    else:
        print(f"🔴 HÜKÜM: pusula BOZUK (ortanca {ort:.0f}°) — §1.07j/3 MAGFit ŞART.")
    print("⚠ Bu ölçüm yalnız DÜZ sürüşte anlamlıdır; dönüşlerde gidiş yönü")
    print("  ile burun yönü doğal olarak ayrışır.")
    return 0


def main() -> int:
    if len(sys.argv) > 2 and sys.argv[1] == "--ozet":
        return ozetle(sys.argv[2])

    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
    from std_msgs.msg import Float64
    from geometry_msgs.msg import TwistStamped
    from sensor_msgs.msg import NavSatFix

    CIKTI_DIZIN.mkdir(parents=True, exist_ok=True)
    yol = CIKTI_DIZIN / f"pusula_{datetime.now():%Y%m%d_%H%M%S}.csv"

    class Ornekleyici(Node):
        def __init__(self) -> None:
            super().__init__("pusula_ornekleyici")
            self.hdg: float | None = None
            self.vx = self.vy = 0.0
            self.lat = self.lon = 0.0
            self.n = 0
            self.gecerli = 0

            sensor = QoSProfile(
                reliability=ReliabilityPolicy.BEST_EFFORT,
                history=HistoryPolicy.KEEP_LAST, depth=10,
            )
            self.create_subscription(
                Float64, "/mavros/global_position/compass_hdg",
                lambda m: setattr(self, "hdg", m.data), sensor)
            self.create_subscription(
                TwistStamped, "/mavros/global_position/raw/gps_vel",
                self._vel, sensor)
            self.create_subscription(
                NavSatFix, "/mavros/global_position/raw/fix", self._fix, sensor)

            self.dosya = open(yol, "w", newline="")
            self.yaz = csv.writer(self.dosya)
            self.yaz.writerow(["t", "pusula_der", "gidis_der", "fark_der",
                               "hiz_ms", "lat", "lon", "gecerli"])
            self.create_timer(1.0 / YAZ_HZ, self._tik)
            print(f"pusula örnekleyici başladı → {yol}", flush=True)

        def _vel(self, m) -> None:
            self.vx, self.vy = m.twist.linear.x, m.twist.linear.y

        def _fix(self, m) -> None:
            self.lat, self.lon = m.latitude, m.longitude

        def _tik(self) -> None:
            if self.hdg is None:
                return
            hiz = math.hypot(self.vx, self.vy)
            # ENU: x=doğu, y=kuzey → pusula başlığı kuzeyden saat yönünde
            gidis = math.degrees(math.atan2(self.vx, self.vy)) % 360.0
            fark = _aci_farki(self.hdg, gidis)
            ok = hiz >= MIN_HIZ
            self.n += 1
            self.gecerli += int(ok)
            self.yaz.writerow([f"{self.get_clock().now().nanoseconds / 1e9:.2f}",
                               f"{self.hdg:.1f}", f"{gidis:.1f}", f"{fark:.1f}",
                               f"{hiz:.2f}", f"{self.lat:.7f}", f"{self.lon:.7f}",
                               int(ok)])
            if self.n % int(YAZ_HZ * 30) == 0:            # 30 saniyede bir
                self.dosya.flush()
                print(f"  {self.n} örnek · {self.gecerli} geçerli "
                      f"(hız≥{MIN_HIZ}) · son fark {fark:+.0f}° @ {hiz:.2f} m/s",
                      flush=True)

    rclpy.init()
    dugum = Ornekleyici()
    try:
        rclpy.spin(dugum)
    except KeyboardInterrupt:
        pass
    finally:
        dugum.dosya.flush()
        dugum.dosya.close()
        print(f"\nkapandı: {yol}  ({dugum.gecerli} geçerli örnek)", flush=True)
        dugum.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
