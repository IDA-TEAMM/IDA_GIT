"""KAPTAN SORUSU (11.08): "bir gate ver ve ötesine bir nokta koy — kod önce
gate'e gidip sonra mı noktaya gidiyor?"

Gerçek zincirle kapalı döngü: GateFollower + PlanningPipeline (RRT*+MPPI) +
CatamaranDynamics (log 58'den tanılanmış). planning_node'un kadansı birebir:
algı 10 Hz → görev 5 Hz (gate.update) → kontrol 10 Hz (compute_control).

İki senaryo:
  A) nokta kapının TAM KARŞISINDA ötesinde  → sıra kendiliğinden doğru olabilir
  B) nokta kapının ötesinde ama YANA KAÇIK  → asıl sınav: kapı takibi yoksa
     araç doğrudan noktaya gider ve kapının DIŞINDAN geçer (puan gitmiş olur)
"""
from __future__ import annotations
import math
import numpy as np

from prototype.dynamics.catamaran import CatamaranDynamics
from prototype.mission.gate_follower import GateFollower, GateFollowerConfig
from prototype.planning.pipeline import PlanningPipeline, PlanningPipelineConfig
from prototype.planning.rrt_star import Bounds, CircleObstacle

BUOY_R = 0.15
HULL_W, HULL_L = 0.785, 1.04
KAPI_X = 20.0
KAPI_W = 12.0                      # gerçek P1 açıklığı (§0.17b, kaptan teyidi)
KAMERA_FOV, KAMERA_MENZIL = 1.2, 15.0
VARIS_R, DWELL = 2.0, 2.0
YARI_BANT = KAPI_W / 2 - BUOY_R - HULL_W / 2      # geçilebilir yarı bant


def _algi(state, dubalar):
    x, y, psi = float(state[0]), float(state[1]), float(state[2])
    kenar, engel = [], []
    for bx, by in dubalar:
        d = math.hypot(bx - x, by - y)
        if d > 25.0:
            continue
        brg = (math.atan2(by - y, bx - x) - psi + math.pi) % (2 * math.pi) - math.pi
        if d <= KAMERA_MENZIL and abs(brg) <= KAMERA_FOV / 2:
            kenar.append((bx, by))
        else:
            engel.append(CircleObstacle(bx, by, BUOY_R))
    return kenar, engel


def _kesisiyor(p1, p2, q1, q2):
    def yon(a, b, c):
        return (b[0]-a[0])*(c[1]-a[1]) - (b[1]-a[1])*(c[0]-a[0])
    d1, d2 = yon(q1, q2, p1), yon(q1, q2, p2)
    d3, d4 = yon(p1, p2, q1), yon(p1, p2, q2)
    return ((d1 > 0) != (d2 > 0)) and ((d3 > 0) != (d4 > 0))


def kosum(nokta, etiket, kapi_takibi=True):
    sol, sag = (KAPI_X, KAPI_W / 2), (KAPI_X, -KAPI_W / 2)
    dubalar = [sol, sag]
    dyn = CatamaranDynamics()
    pipe = PlanningPipeline(
        Bounds(-20.0, 60.0, -25.0, 25.0),
        PlanningPipelineConfig(mppi_K=200, mppi_T=30,
                               mppi_terminal_lookahead_m=3.0),
        dynamics=dyn,
    )
    pipe.set_mission_state("PARKUR1")
    gate = GateFollower(GateFollowerConfig(HULL_W, HULL_L))

    state = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    dt, t, algi_no = 0.1, 0.0, 0
    dwell_t = None
    onceki = (0.0, 0.0)
    t_kapi = None          # kapı düzleminin kesildiği an
    sapma_kapi = None      # o andaki yanal sapma
    t_nokta = None         # noktaya varış (dwell dolduğu an)
    nisan_adim = 0         # hedefin KAPI NİŞANI olduğu tick sayısı
    ham_adim = 0           # hedefin HAM NOKTA olduğu tick sayısı
    ilk_nisan_t = None
    son_nisan_t = None

    while t < 300.0 and t_nokta is None:
        kenar, engeller = _algi(state, dubalar)
        if not kapi_takibi:                    # negatif kontrol: hepsi engel
            engeller = [CircleObstacle(bx, by, BUOY_R) for bx, by in dubalar]
            kenar = []
        pipe.set_obstacles(engeller)
        algi_no += 1
        if round(t * 10) % 2 == 0:             # görev katmanı 5 Hz
            r = gate.update(
                (float(state[0]), float(state[1])), nokta, kenar,
                [(o.cx, o.cy, o.r) for o in engeller], gozlem_no=algi_no,
            )
            pipe.set_waypoints([r.target])
            if r.used_fallback:
                ham_adim += 1
            else:
                nisan_adim += 1
                if ilk_nisan_t is None:
                    ilk_nisan_t = t
                son_nisan_t = t
        pipe.set_state(state)
        u = pipe.compute_control()
        if u is None or not np.all(np.isfinite(u)):
            print(f"  !! MPPI None/NaN t={t:.1f}")
            break
        for _ in range(2):
            state = dyn.step_rk4(state, u, dt / 2)
        t += dt

        simdi = (float(state[0]), float(state[1]))
        if t_kapi is None and _kesisiyor(onceki, simdi, sol, sag):
            t_kapi = t
            sapma_kapi = abs(simdi[1])
        onceki = simdi

        if math.hypot(state[0]-nokta[0], state[1]-nokta[1]) <= VARIS_R:
            if dwell_t is None:
                dwell_t = t
            elif t - dwell_t >= DWELL:
                t_nokta = t
        else:
            dwell_t = None

    print(f"\n{'='*66}\n{etiket}\n{'='*66}")
    print(f"  kapı: x={KAPI_X}, açıklık {KAPI_W} m (direkler y=±{KAPI_W/2})")
    print(f"  nokta: {nokta}   (kapının {nokta[0]-KAPI_X:.0f} m ötesinde, "
          f"yanal kaçıklık {abs(nokta[1]):.0f} m)")
    print(f"  geçilebilir yarı bant: ±{YARI_BANT:.2f} m")
    print(f"  --- hedef neydi ---")
    print(f"  KAPI NİŞANI olduğu tick : {nisan_adim}")
    print(f"  HAM NOKTA olduğu tick   : {ham_adim}")
    if ilk_nisan_t is not None:
        print(f"  nişan penceresi         : t={ilk_nisan_t:.1f}s → {son_nisan_t:.1f}s")
    print(f"  --- SIRA ---")
    if t_kapi is None:
        print(f"  🔴 kapı düzlemi HİÇ kesilmedi")
    else:
        ic = "İÇİNDEN ✅" if sapma_kapi <= YARI_BANT else "DIŞINDAN 🔴"
        print(f"  kapı geçişi : t={t_kapi:6.1f} s · yanal sapma {sapma_kapi:.2f} m → {ic}")
    if t_nokta is None:
        print(f"  🔴 noktaya VARILMADI (300 s)")
    else:
        print(f"  noktaya varış: t={t_nokta:6.1f} s")
    if t_kapi is not None and t_nokta is not None:
        if t_kapi < t_nokta:
            print(f"  ✅ SIRA DOĞRU: önce kapı ({t_kapi:.1f}s), sonra nokta "
                  f"({t_nokta:.1f}s) — arada {t_nokta-t_kapi:.1f} s")
        else:
            print(f"  🔴 SIRA TERS")
    return t_kapi, sapma_kapi, t_nokta


if __name__ == "__main__":
    kosum((35.0, 0.0), "A) NOKTA TAM KARŞIDA — kapının 15 m ötesinde")
    kosum((35.0, 8.0), "B) NOKTA KAÇIK — kapının ötesinde ama 8 m yanda")
    kosum((35.0, 8.0), "C) NEGATİF KONTROL — aynı sahne, KAPI TAKİBİ KAPALI "
                       "(dubalar sınıflanmıyor, hepsi engel)", kapi_takibi=False)
