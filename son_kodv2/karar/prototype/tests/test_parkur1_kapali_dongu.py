"""Parkur-1 ARDIŞIK KAPILI kapalı-döngü testi (2026-08-08).

🔴 **Neden var:** kapı katmanının 40+ testi vardı ama hepsi TEK TICK / TEK
KAPI idi. 06.08 gecesinde ölçülen "geçilmiş kapıya geri kilitlenip sonsuz
salınım" arızasını (GIRDAP_DURUM §0.9b) CI bu yüzden hiç görmedi: arıza ancak
**birden fazla kapı ardışık geçilirken**, gerçek dinamikle ve gerçek kadansla
ortaya çıkıyor. Bu dosya o boşluğu kapatır.

Test edilen zincir `planning_node`'un ta kendisidir (aynı sıra, aynı kadans):
    algı 10 Hz  → sınıflı engel/kenar ayrımı (`_on_classified` kuralı)
    görev 5 Hz  → `GateFollower.update` ile ham GN → kapı nişanı
    kontrol10 Hz→ `PlanningPipeline.compute_control` → `step_rk4`

⚠️ Hız için MPPI küçültüldü (K=200, T=30 — `test_planning_pipeline._fast_cfg`
deseni). Bu test AYAR SEÇMEZ; "zincir ardışık kapılarda ilerliyor mu"yu
dondurur. Ayar ölçümleri `docs/parkur1_kontrol_listesi.md` §4'te.
"""

from __future__ import annotations

import math

import numpy as np

from prototype.dynamics.catamaran import CatamaranDynamics
from prototype.mission.gate_follower import GateFollower, GateFollowerConfig
from prototype.planning.pipeline import PlanningPipeline, PlanningPipelineConfig
from prototype.planning.rrt_star import Bounds, CircleObstacle

BUOY_R = 0.15          # md 5.5.2.1: duba çapı 30 cm
HULL_W, HULL_L = 0.78, 1.04
KAPI_GENISLIK = 4.0
KAMERA_FOV = 1.2       # rad, OAK-D Lite (hardware.yaml perception.fusion)
KAMERA_MENZIL = 15.0   # m


#: GN'nin kapı ortasından kaçıklığı (m) — md 5.5.2.2 "verilen nokta doğrudan
#: iki kenar dubasının arasında OLMAYABİLİR". 1,5 m seçildi çünkü geçilebilir
#: yarı-bant = yarı genişlik − duba yarıçapı − gövde yarı genişliği
#: = 2,00 − 0,15 − 0,39 = 1,46 m: ham GN'ye sürmek bandın DIŞINDA kalır,
#: kapı nişanına sürmek içinde. Testin ayırt ettiği fark tam olarak budur.
GN_KAYMA = 1.5
GECILEBILIR_YARI_BANT = KAPI_GENISLIK / 2 - BUOY_R - HULL_W / 2


def _sahne():
    """Düz hatta 3 kapı + her kapının yanında KAÇIK bir GN."""
    kapilar, gnler, dubalar = [], [], []
    for i, x in enumerate((10.0, 20.0, 30.0)):
        sol, sag = (x, KAPI_GENISLIK / 2), (x, -KAPI_GENISLIK / 2)
        kapilar.append((sol, sag))
        dubalar += [sol, sag]
        gnler.append((x, GN_KAYMA if i % 2 == 0 else -GN_KAYMA))
    return gnler, dubalar, kapilar


def _algi(state, dubalar):
    """`_on_classified` kuralının aynası: FOV+menzil içi → kenar, dışı → engel."""
    x, y, psi = float(state[0]), float(state[1]), float(state[2])
    kenar, engel = [], []
    for bx, by in dubalar:
        d = math.hypot(bx - x, by - y)
        if d > 25.0:                                   # LiDAR menzili
            continue
        bearing = (math.atan2(by - y, bx - x) - psi + math.pi) % (2 * math.pi) - math.pi
        if d <= KAMERA_MENZIL and abs(bearing) <= KAMERA_FOV / 2:
            kenar.append((bx, by))
        else:
            engel.append(CircleObstacle(bx, by, BUOY_R))
    return kenar, engel


def _kesisiyor(p1, p2, q1, q2) -> bool:
    def yon(a, b, c):
        return (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])
    d1, d2 = yon(q1, q2, p1), yon(q1, q2, p2)
    d3, d4 = yon(p1, p2, q1), yon(p1, p2, q2)
    return ((d1 > 0) != (d2 > 0)) and ((d3 > 0) != (d4 > 0))


def test_parkur1_ardisik_kapilardan_ilerler_ve_geri_kilitlenmez() -> None:
    gnler, dubalar, kapilar = _sahne()
    dyn = CatamaranDynamics()
    pipe = PlanningPipeline(
        Bounds(-20.0, 60.0, -20.0, 20.0),
        # lookahead: kodun kendi kuralı (hız × ufuk) — 1.05 m/s × 1.5 s
        PlanningPipelineConfig(
            mppi_K=200, mppi_T=30, mppi_terminal_lookahead_m=3.0
        ),
        dynamics=dyn,
    )
    pipe.set_mission_state("PARKUR1")
    gate = GateFollower(GateFollowerConfig(HULL_W, HULL_L))

    state = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    dt, t, idx, algi_no = 0.1, 0.0, 0, 0
    dwell_t = None
    kenar, engeller = [], []
    fiili: dict[int, float] = {}          # kapı → geçişteki yanal sapma (m)
    en_geri_x = 0.0
    onceki = (0.0, 0.0)

    while t < 220.0 and idx < len(gnler):
        kenar, engeller = _algi(state, dubalar)
        pipe.set_obstacles(engeller)
        algi_no += 1
        if round(t * 10) % 2 == 0:                    # görev katmanı 5 Hz
            hedef = gate.update(
                (float(state[0]), float(state[1])), gnler[idx], kenar,
                [(o.cx, o.cy, o.r) for o in engeller], gozlem_no=algi_no,
            ).target
            pipe.set_waypoints([hedef])
        pipe.set_state(state)
        u = pipe.compute_control()
        assert u is not None and np.all(np.isfinite(u)), "MPPI sayısal çöktü"
        for _ in range(2):
            state = dyn.step_rk4(state, u, dt / 2)
        t += dt

        simdi = (float(state[0]), float(state[1]))
        for ki, (sol, sag) in enumerate(kapilar):
            if ki not in fiili and _kesisiyor(onceki, simdi, sol, sag):
                fiili[ki] = abs(simdi[1] - (sol[1] + sag[1]) / 2.0)
        onceki = simdi
        en_geri_x = min(en_geri_x, float(state[0]))

        if math.hypot(state[0] - gnler[idx][0], state[1] - gnler[idx][1]) <= 2.0:
            if dwell_t is None:
                dwell_t = t
            elif t - dwell_t >= 2.0:
                idx += 1
                dwell_t = None
        else:
            dwell_t = None

    assert idx == len(gnler), (
        f"P1 bitmedi: {idx}/{len(gnler)} GN, {t:.0f} s, "
        f"fiilen geçilen kapı {len(fiili)}"
    )
    # K1 nöbetçisi: geçilmiş kapıya geri kilitlenme = GERİ gitme (§0.9b'de
    # 88 m ölçülmüştü). Başlangıcın 5 m'den fazla gerisine düşmemeli.
    assert en_geri_x > -5.0, f"araç {en_geri_x:.1f} m'ye kadar GERİ gitti (K1)"
    # Kapıların ARASINDAN geçilmeli (md 5.5.4.2 geçiş puanı buradan gelir).
    assert len(fiili) >= 2, (
        f"3 kapının yalnız {len(fiili)}'inden geçildi — kapı nişanı zinciri "
        "kopmuş olabilir"
    )
    # Ve GEÇİLEBİLİR BANT içinden geçilmeli.
    # 🔑 NEGATİF KONTROL (08.08'de koşuldu): aynı sahne, kenar dubaları
    # sınıflanmadan (kamera yokmuş gibi, hepsi engel) → araç **ilk GN'ye bile
    # VARAMADI (0/3)**. Sebep: kaçık GN engel ceza halkasının içinde kalıyor,
    # RRT* "goal engel içinde" diye reddediyor. Kapı takibiyle: 3/3 GN,
    # 3/3 kapı, en kötü sapma **0,04 m**.
    en_kotu = max(fiili.values())
    assert en_kotu <= GECILEBILIR_YARI_BANT, (
        f"kapı düzlemi {en_kotu:.2f} m sapmayla geçildi (bant "
        f"{GECILEBILIR_YARI_BANT:.2f} m) — nişan ham GN'ye kaymış olabilir"
    )
