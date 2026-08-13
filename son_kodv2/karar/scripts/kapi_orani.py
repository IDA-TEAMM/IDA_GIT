#!/usr/bin/env python3
"""
Girdap İDA — KAPI GEÇME ORANI ölçüm koşumcusu (13.08.2026).

🔴 NEDEN VAR: 13.08'e kadar her planlama değişikliği **tek koşumla**
değerlendiriliyordu; 5/8 ile 2/8 arasındaki farkın ne kadarı gerçek, ne kadarı
tesadüf bilinmiyordu. Şartname P2 puanı **oransal** (`(G2/KD2)×40`) — yani
"kaç kapıdan geçtik" doğrudan puandır ve **dağılımıyla** ölçülmelidir.

Bu araç kapalı döngüyü **gerçek çekirdeklerle** koşturur (PlanningPipeline +
GateFollower + EdgeBuoyMemory + CatamaranDynamics), ROS'suz — koşum başına
saniyeler, dakikalar değil. ROS'lu `sanal_gol` entegrasyon içindir; bu araç
**istatistik** içindir.

🎯 ZORLAYICI BAŞLANGIÇLAR (`--zor`): 13.08 ölçümünde tekne koridorun yanına
düşünce kapıları kaybediyordu (8 kapıda 5). Gölde bunu dalga/akıntı düzenli
üretecek. Bu kip tekneyi kasten **yanal kaçık + açı hatalı** başlatır ve
**kurtarma oranını** ölçer.

Kullanım:
    python3 scripts/kapi_orani.py                    # taban ölçüm (normal)
    python3 scripts/kapi_orani.py --zor              # zorlayıcı başlangıçlar
    python3 scripts/kapi_orani.py --kosum 24 --zor
    python3 scripts/kapi_orani.py --model-yok        # `.pt` yokken (hepsi engel)

⚠ Koşum BELİRLENİMLİDİR (MPPI tohumu sabit); değişkenlik yalnız başlangıç
pozundan gelir. Aynı kod + aynı bayraklar = aynı sayı.
"""

from __future__ import annotations

import argparse
import math
import statistics
import sys
from pathlib import Path
from typing import List, Optional, Tuple

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from prototype.dynamics.catamaran import CatamaranDynamics       # noqa: E402
from prototype.mission.edge_memory import (                       # noqa: E402
    CLASS_UNKNOWN,
    EdgeBuoyMemory,
)
from prototype.mission.gate_follower import (                     # noqa: E402
    BUOY_RADIUS_M,
    GateFollower,
    GateFollowerConfig,
)
from prototype.mission.parkur_dunyasi import oku as parkuru_oku   # noqa: E402
from prototype.planning.pipeline import (                         # noqa: E402
    PlanningPipeline,
    PlanningPipelineConfig,
)
from prototype.planning.rrt_star import Bounds, CircleObstacle    # noqa: E402

_GC = GateFollowerConfig()
HULL_W, HULL_L = _GC.hull_width_m, _GC.hull_length_m
BUOY_R = BUOY_RADIUS_M
KAMERA_FOV = 1.2               # rad — hardware.yaml perception.fusion
KAMERA_MENZIL = 15.0           # m
EDGE_CLASS_ID = 0
HUNI_TAVANI = 1.4              # planning_node.gate_post_margin_m
VARIS_YARICAP = 2.0            # mission.arrival_radius_m
DWELL_S = 2.0                  # mission.dwell_time_s


def _kesisiyor(a, b, p, q) -> bool:
    """[a,b] yol parçası kapı kirişini [p,q] kesiyor mu (geçiş kanıtı)."""

    def yon(o, u, v):
        return (u[0] - o[0]) * (v[1] - o[1]) - (u[1] - o[1]) * (v[0] - o[0])

    d1, d2 = yon(p, q, a), yon(p, q, b)
    d3, d4 = yon(a, b, p), yon(a, b, q)
    return ((d1 > 0) != (d2 > 0)) and ((d3 > 0) != (d4 > 0))


def kosum(
    parkur,
    *,
    baslangic: Tuple[float, float],
    yon_hatasi_rad: float = 0.0,
    model_var: bool = True,
    sure: float = 400.0,
) -> dict:
    """Kapalı döngüyü bir kez koştur, kapı geçiş metriklerini döndür."""
    kapilar = parkur.kapilar
    dubalar = parkur.dubalar()
    gn = parkur.guzergah
    dyn = CatamaranDynamics()
    pipe = PlanningPipeline(
        Bounds(-20.0, 60.0, -25.0, 25.0),
        PlanningPipelineConfig(
            mppi_K=200, mppi_T=30, mppi_terminal_lookahead_m=3.0
        ),
        dynamics=dyn,
    )
    pipe.set_mission_state("PARKUR1")
    gate = GateFollower(GateFollowerConfig(HULL_W, HULL_L))
    hafiza = EdgeBuoyMemory()

    state = np.array(
        [baslangic[0], baslangic[1], yon_hatasi_rad, 0.0, 0.0, 0.0]
    )
    dt, t, idx, algi_no = 0.1, 0.0, 0, 0
    dwell_t: Optional[float] = None
    gecilen: dict = {}
    en_kucuk_pay = 9.9
    onceki = baslangic

    while t < sure and idx < len(gn):
        x, y, psi = float(state[0]), float(state[1]), float(state[2])
        # `planning_node._on_classified` kuralının aynası: kamera görüş
        # alanında + model varsa turuncu, aksi hâlde CLASS_UNKNOWN.
        tespitler = []
        for bx, by in dubalar:
            d = math.hypot(bx - x, by - y)
            brg = (
                math.atan2(by - y, bx - x) - psi + math.pi
            ) % (2 * math.pi) - math.pi
            gorunur = (
                model_var and d <= KAMERA_MENZIL and abs(brg) <= KAMERA_FOV / 2
            )
            tespitler.append(
                (bx, by, BUOY_R, EDGE_CLASS_ID if gorunur else CLASS_UNKNOWN)
            )
        kenar_mi = (
            hafiza.siniflandir(tespitler, EDGE_CLASS_ID)
            if model_var else [False] * len(tespitler)
        )
        kenar = [
            (bx, by) for (bx, by, _, _), k in zip(tespitler, kenar_mi) if k
        ]
        engeller = [
            CircleObstacle(bx, by, r)
            for (bx, by, r, _), k in zip(tespitler, kenar_mi) if not k
        ]
        # B2 huni: kapı direği kenar KALIR ama torbaya da girer, payı ölçülen
        # açıklıktan türer (`planning_node._huni_payi` aynası).
        for i, (kx, ky) in enumerate(kenar):
            komsu = [
                math.hypot(kx - ox, ky - oy)
                for j, (ox, oy) in enumerate(kenar) if j != i
            ]
            m = HUNI_TAVANI if not komsu else max(
                0.0,
                min(HUNI_TAVANI, (min(komsu) - HULL_W - 2 * BUOY_R) / 2.0),
            )
            engeller.append(CircleObstacle(kx, ky, BUOY_R, margin=m))
        pipe.set_obstacles(engeller)
        algi_no += 1

        if round(t * 10) % 2 == 0:                    # görev katmanı 5 Hz
            if model_var:
                hedef = gate.update(
                    (x, y), gn[idx], kenar,
                    [(o.cx, o.cy, o.r) for o in engeller],
                    gozlem_no=algi_no,
                ).surus_hedefi
            else:
                hedef = gn[idx]
            pipe.set_waypoints([hedef])

        pipe.set_state(state)
        u = pipe.compute_control()
        if u is None:
            u = np.zeros(2)
        if not np.all(np.isfinite(u)):
            break                                     # MPPI çöktü → koşum biter
        for _ in range(2):
            state = dyn.step_rk4(state, u, dt / 2)
        t += dt

        simdi = (float(state[0]), float(state[1]))
        for bx, by in dubalar:
            pay = math.hypot(bx - simdi[0], by - simdi[1]) - BUOY_R - HULL_W / 2
            en_kucuk_pay = min(en_kucuk_pay, pay)
        for ki, (sol, sag) in enumerate(kapilar):
            if ki not in gecilen and _kesisiyor(onceki, simdi, sol, sag):
                orta = ((sol[0] + sag[0]) / 2.0, (sol[1] + sag[1]) / 2.0)
                gecilen[ki] = math.hypot(
                    simdi[0] - orta[0], simdi[1] - orta[1]
                )
        onceki = simdi

        if math.hypot(
            state[0] - gn[idx][0], state[1] - gn[idx][1]
        ) <= VARIS_YARICAP:
            if dwell_t is None:
                dwell_t = t
            elif t - dwell_t >= DWELL_S:
                idx += 1
                dwell_t = None
        else:
            dwell_t = None

    sapmalar = list(gecilen.values())
    return {
        "gecilen": len(gecilen),
        "toplam_kapi": len(kapilar),
        "varilan_gn": idx,
        "toplam_gn": len(gn),
        "sapma_ort": statistics.mean(sapmalar) if sapmalar else float("nan"),
        "en_kucuk_pay": en_kucuk_pay,
        "carpma": en_kucuk_pay <= 0.0,
        "sure": t,
    }


def baslangiclar(parkur, zor: bool, adet: int) -> List[Tuple[Tuple[float, float], float]]:
    """(konum, yön hatası) listesi.

    NORMAL: parkurun kendi başlangıcı çevresinde küçük saçılma.
    ZOR:    koridorun YANINDA + AÇI HATALI — 13.08'de ölçülen arıza deseni
            (tekne yandan kaçınca kapı bir daha seçilemiyordu). Kaçıklık
            KAPI YARI GENİŞLİĞİNDEN türer (öz-ölçekli, uydurma sayı yok).
    """
    b = parkur.baslangic
    yari = (parkur.kapi_genislikleri[0] if parkur.kapi_genislikleri else 12.0) / 2.0
    out: List[Tuple[Tuple[float, float], float]] = []
    if not zor:
        for i in range(adet):
            k = (i % 5 - 2) * 0.5                     # ±1 m yanal
            out.append(((b[0], b[1] + k), 0.0))
        return out
    # Zor: yanal kaçıklık yarı genişliğin {0.5, 0.75, 1.0}'i × iki yön,
    # açı hatası {0, ±30°}. Kaçıklık ≥ yarı genişlik → kapı bandının DIŞI.
    kacikliklar = [yari * f * s for f in (0.5, 0.75, 1.0) for s in (1.0, -1.0)]
    acilar = [0.0, math.radians(30.0), math.radians(-30.0)]
    i = 0
    while len(out) < adet:
        k = kacikliklar[i % len(kacikliklar)]
        a = acilar[(i // len(kacikliklar)) % len(acilar)]
        out.append(((b[0], b[1] + k), a))
        i += 1
    return out


def main() -> None:
    ap = argparse.ArgumentParser(description="Kapı geçme oranı ölçümü")
    ap.add_argument("--kosum", type=int, default=12, help="koşum sayısı")
    ap.add_argument("--zor", action="store_true", help="zorlayıcı başlangıçlar")
    ap.add_argument("--model-yok", action="store_true",
                    help="`.pt` yok: hiçbir duba sınıflanmaz (hepsi engel)")
    ap.add_argument("--sure", type=float, default=400.0)
    a = ap.parse_args()

    parkur = parkuru_oku()
    basl = baslangiclar(parkur, a.zor, a.kosum)
    print(f"parkur: {len(parkur.kapilar)} kapı "
          f"(açıklık {parkur.kapi_genislikleri[0]:.1f} m) · "
          f"{len(parkur.guzergah)} görev noktası")
    print(f"kip: {'ZOR' if a.zor else 'normal'}"
          f"{' · MODEL YOK' if a.model_yok else ''} · {a.kosum} koşum\n")
    print(f"{'#':>3} {'başlangıç':>16} {'açı':>6} {'kapı':>7} {'GN':>6} "
          f"{'sapma':>7} {'pay':>7}")
    oranlar, sapmalar, paylar, carpma = [], [], [], 0
    for i, (poz, aci) in enumerate(basl, 1):
        r = kosum(parkur, baslangic=poz, yon_hatasi_rad=aci,
                  model_var=not a.model_yok, sure=a.sure)
        oran = r["gecilen"] / r["toplam_kapi"]
        oranlar.append(oran)
        paylar.append(r["en_kucuk_pay"])
        if not math.isnan(r["sapma_ort"]):
            sapmalar.append(r["sapma_ort"])
        carpma += int(r["carpma"])
        print(f"{i:3d} ({poz[0]:6.1f},{poz[1]:6.1f}) {math.degrees(aci):5.0f}° "
              f"{r['gecilen']:3d}/{r['toplam_kapi']:<3d} "
              f"{r['varilan_gn']:2d}/{r['toplam_gn']:<3d} "
              f"{r['sapma_ort']:6.2f}m {r['en_kucuk_pay']:6.2f}m")

    oranlar.sort()
    print(f"\n{'='*56}\nKAPI GEÇME ORANI — {len(oranlar)} koşum")
    print(f"  ortalama {statistics.mean(oranlar)*100:5.1f}%"
          f" · ortanca {statistics.median(oranlar)*100:5.1f}%"
          f" · en kötü {oranlar[0]*100:5.1f}% · en iyi {oranlar[-1]*100:5.1f}%")
    if sapmalar:
        print(f"  geçiş sapması ortalaması {statistics.mean(sapmalar):.2f} m")
    print(f"  en küçük gövde payı {min(paylar):.2f} m · ÇARPMA {carpma}/{len(oranlar)}")


if __name__ == "__main__":
    main()
