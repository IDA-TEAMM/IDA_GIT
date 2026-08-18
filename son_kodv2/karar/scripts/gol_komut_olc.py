#!/usr/bin/env python3
"""SANAL GÖL — KOMUT DAĞILIMI ölçer (F-F.22 A/B kapısı). SALT OKUR.

🔴 NEDEN VAR: 17.08 göl bandında (`session_20260817_193312`, GUIDED+ARMED
227 s, 2136 komut) ölçülen dağılım:

    GERİ (<-0,02 m/s)  %23,1   ortanca -0,744   en geri -1,173
    SIFIR (|x|<=0,02)  %46,5
    İLERİ (>+0,02)     %30,4

Geri komutların %83,4'ünde hedef, %88,6'sında kapı da aracın ARKASINDAYDI —
yani MPPI 130°'lik dönüş yerine geri gitmeyi ucuz buluyordu. Düzeltme
(`mppi_ileri_kisit`, Nav2 `vx_min=0` karşılığı) yazıldı ama **sanal gölde
ölçülemiyordu**: sim teknesi kuzeye bakarak başlıyor, görev noktaları da
kuzeyde ⇒ "hedef arkada" hâli hiç oluşmuyor. `baslangic_yon_derece:=-90`
o sahneyi kurar; bu araç da sonucu sayar.

Bu araç `gol_pdc_olc.py`'nin kardeşi — o KONUMU (parkur dışına çıkma), bu
KOMUTU ölçer. İkisi aynı koşumda birlikte koşabilir.

KULLANIM:
    python3 scripts/gol_komut_olc.py <cikti.csv> [sure_s]

⚠ SALT OKUR: hiçbir konuya yayın yapmaz, hiçbir parametre yazmaz. Aracı
hareket ettirmesi imkânsız (nöbetçi/ayar defteriyle aynı sözleşme).
"""
from __future__ import annotations

import sys
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node

#: Ölü bölge — bu bandın içi "sıfır komut" sayılır. Bant ölçümünde kullanılan
#: eşiğin AYNISI (0,02 m/s); iki tarafın sayıları doğrudan kıyaslanabilsin.
OLU_BOLGE = 0.02


class KomutOlcer(Node):
    def __init__(self, yol: str) -> None:
        super().__init__("gol_komut_olc")
        self.f = open(yol, "w")
        self.f.write("t,linear_x,angular_z\n")
        self.t0 = time.monotonic()
        self.ileri: list[float] = []
        self.donus: list[float] = []
        self.create_subscription(
            Twist, "/mavros/setpoint_velocity/cmd_vel_unstamped", self._on, 10
        )

    def _on(self, m: Twist) -> None:
        t = time.monotonic() - self.t0
        self.ileri.append(m.linear.x)
        self.donus.append(m.angular.z)
        self.f.write(f"{t:.3f},{m.linear.x:.4f},{m.angular.z:.4f}\n")

    def ozet(self) -> str:
        n = len(self.ileri)
        if n == 0:
            return "KOMUT YOK — GUIDED+ARMED hiç olmadı mı? (ölçüm YAPILAMADI)"
        geri = [x for x in self.ileri if x < -OLU_BOLGE]
        sifir = [x for x in self.ileri if abs(x) <= OLU_BOLGE]
        poz = [x for x in self.ileri if x > OLU_BOLGE]
        s = sorted(geri)
        return (
            f"komut {n}\n"
            f"  GERİ  %{100*len(geri)/n:5.1f}  "
            f"ortanca {s[len(s)//2] if s else 0.0:+.3f}  "
            f"en geri {min(self.ileri):+.3f} m/s\n"
            f"  SIFIR %{100*len(sifir)/n:5.1f}\n"
            f"  İLERİ %{100*len(poz)/n:5.1f}  en ileri {max(self.ileri):+.3f} m/s\n"
            f"  🔑 ÖLÇÜT: geri oranı — 17.08 göl tabanı %23,1, hedef <%2"
        )


def main() -> int:
    if len(sys.argv) < 2:
        print(__doc__)
        return 2
    sure = float(sys.argv[2]) if len(sys.argv) > 2 else 0.0
    rclpy.init()
    dugum = KomutOlcer(sys.argv[1])
    bitis = time.monotonic() + sure if sure > 0 else None
    try:
        while rclpy.ok() and (bitis is None or time.monotonic() < bitis):
            rclpy.spin_once(dugum, timeout_sec=0.2)
    except KeyboardInterrupt:
        pass
    finally:
        print(dugum.ozet())
        dugum.f.close()
        dugum.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
