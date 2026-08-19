#!/usr/bin/env python3
"""
Girdap İDA — PARKUR-2 (Engel Bulunan Ortamda Nokta Takip) ölçüm koşumcusu.

🔴 19.08 — Parkur-2 bu depoda BUGÜNE KADAR HİÇ ÖLÇÜLMEMİŞTİ. `kapi_orani.py`
yalnız Parkur-1'i (parkur_dunyasi.oku()'nun `kapilar` alanı) koşturuyordu;
`.world` dosyasındaki Parkur-2'nin KENDİ kapı çiftleri (`p2_a<i>`/`p2_u<i>`,
11 çift) hiç okunmuyordu (`kapilar_p2` alanı bu yüzden bu gece eklendi).

Şartname (md 5.5.2.4 + Şekil 3): Parkur-2'nin KENDİ görev noktası YOK — tek
istisna kendi SON görev noktası (GN5), o da "Parkur-2'nin son karşılıklı duba
ikilisinden geçmek" ile TANIMLI. Yani araç Parkur-2 boyunca SAF kapı takibiyle
ilerler, GN5'e yalnız son kapı çiftinden geçtikten sonra "varır" — GN'lerle
kapı kapı yönlendirilmez (Parkur-1'in aksine).

Tamamlama şartı (md 5.5.2.4): en az 2 duba ikilisinden geçmek (son GN hariç)
VE son duba ikilisinden geçerek GN5'e ulaşmak.

Kullanım:
    python3 scripts/parkur2_orani.py                 # taban ölçüm
    python3 scripts/parkur2_orani.py --kosum 5
"""
from __future__ import annotations

import argparse
import math
import statistics
import sys
from pathlib import Path
from typing import List, Tuple

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from prototype.dynamics.catamaran import CatamaranDynamics           # noqa: E402
from prototype.mission.edge_memory import CLASS_UNKNOWN, EdgeBuoyMemory  # noqa: E402
from prototype.mission.gate_follower import (                         # noqa: E402
    BUOY_RADIUS_M,
    GateFollower,
    GateFollowerConfig,
)
from prototype.mission.parkur_dunyasi import oku as parkuru_oku       # noqa: E402
from prototype.planning.pipeline import (                             # noqa: E402
    PlanningPipeline,
    PlanningPipelineConfig,
)
from prototype.planning.rrt_star import Bounds, CircleObstacle        # noqa: E402

_GC = GateFollowerConfig()
HULL_W, HULL_L = _GC.hull_width_m, _GC.hull_length_m
BUOY_R = BUOY_RADIUS_M
KAMERA_FOV = 1.2
# 🔴 19.08 — kapi_orani.py'den kopyalanan "15.0" YANLIŞTI: hardware.yaml'da
# ayrı bir "kamera menzili" parametresi YOK. Füzyon LiDAR-menzil +
# kamera-açı modeli (bearing-based association, fusion.py) — mesafeyi LiDAR
# verir (`perception.lidar.max_range=25.0`), kamera yalnız açısal
# konum/renk sağlar. 15.0 kullanınca Parkur-2'nin geniş (20-22 m) kapıları
# için GEREKEN görüş mesafesi (yarı_genişlik/tan(FOV/2) ≈ 16,1 m) menzilin
# DIŞINDA kalıyordu — hiçbir kapıya asla kilitlenilemiyordu (472/472 fallback,
# ölçüldü). Doğru sayı LiDAR'ın gerçek menzili.
KAMERA_MENZIL = 25.0            # perception.lidar.max_range (hardware.yaml)
EDGE_CLASS_ID = 0
HUNI_TAVANI = 1.4
VARIS_YARICAP = 2.0
DWELL_S = 2.0
URETIM_K = 1000
URETIM_T = 50
VARSAYILAN_SURE_S = 900.0
# planning_node._harita_yaricapi = map_width(100)*map_resolution(0.5)/2 (pipeline.py).
HARITA_YARICAPI = 25.0
UNUTMA_MENZILI = HARITA_YARICAPI * 2.0   # edge_unutma_katsayisi varsayılanı


def _sinir_kutusu(kapilar_p2, engeller, baslangic, gn5) -> Bounds:
    xs = ([b[0] for kapi in kapilar_p2 for b in kapi] + [e[0] for e in engeller]
          + [gn5[0], baslangic[0]])
    ys = ([b[1] for kapi in kapilar_p2 for b in kapi] + [e[1] for e in engeller]
          + [gn5[1], baslangic[1]])
    pay = max(10.0, 0.2 * max(max(xs) - min(xs), max(ys) - min(ys)))
    return Bounds(min(xs) - pay, max(xs) + pay, min(ys) - pay, max(ys) + pay)


def kosum(
    kapilar_p2, engel_konumlari, *, baslangic, yon0_rad, gn5_gercek=None,
    sure=VARSAYILAN_SURE_S, mppi_k=URETIM_K, mppi_t=URETIM_T,
) -> dict:
    """PARKUR-2'yi kapalı döngü bir kez koştur.

    Kapı direkleri (`kapilar_p2`) kamera FOV+menzilinde GÖRÜLEBİLİNCE kenar
    sınıfına düşer (P1 testiyle aynı kural) — sarı engel dubaları İSE
    HER ZAMAN engel sayılır (renkle ayrımı P1'in kenar/kenar-değil
    belirsizliğinden farklı, HSV eşiği güvenilir — bkz. camera_buoys.py).
    """
    parkur1_gate = GateFollowerConfig(HULL_W, HULL_L)
    dyn = CatamaranDynamics()
    gn5 = kapilar_p2[-1]
    # Gerçek GN5 (.world dosyasından) verilmişse ONU kullan — üretimde de
    # mission GN5'i kendi başına verir, son kapının orta noktasından
    # TÜRETİLMEZ (ikisi arasında birkaç m fark olabilir).
    gn5_orta = gn5_gercek if gn5_gercek is not None else (
        (gn5[0][0] + gn5[1][0]) / 2.0, (gn5[0][1] + gn5[1][1]) / 2.0)
    # 🔴 19.08 gece — SİMÜLASYON SAATİ ENJEKTE EDİLİYOR (kapi_orani.py'de
    # ekip tarafından bulunan/düzeltilen AYNI kusur, 848f9465). `PlanningPipeline`
    # varsayılan `saat=time.monotonic` kullanır — hem RRT* replan freni
    # (`_replan_frenli`) hem F-P.11 sıkışma kurtarma penceresi
    # (`_sikisma_kurtarmasini_guncelle`) bunu okur. Enjekte edilmezse ikisi de
    # GERÇEK CPU hızına bağlı olur. Ölçüldü: bu betikte kurtarma HİÇBİR ZAMAN
    # tetiklenmiyordu (`kurtarma_aktif` 300+ örnekte hep False) — tekne
    # kilitli-ama-önündeki-kapı-kilitsiz tuzağında 35+ sn hareketsiz kaldı.
    _sim_saat = [0.0]
    pipe = PlanningPipeline(
        _sinir_kutusu(kapilar_p2, engel_konumlari, baslangic, gn5_orta),
        PlanningPipelineConfig(mppi_K=mppi_k, mppi_T=mppi_t,
                                mppi_terminal_lookahead_m=3.0,
                                stuck_recovery_enabled=True),
        dynamics=dyn,
        saat=lambda: _sim_saat[0],
    )
    pipe.set_mission_state("PARKUR2")
    gate = GateFollower(parkur1_gate)
    hafiza = EdgeBuoyMemory()
    kapi_direkleri = [b for kapi in kapilar_p2 for b in kapi]

    state = np.array([baslangic[0], baslangic[1], yon0_rad, 0.0, 0.0, 0.0])
    dt, t = 0.1, 0.0
    dwell_t = None
    gecilen: dict = {}
    en_kucuk_pay = 9.9
    onceki = baslangic
    hedef = gn5_orta

    while t < sure:
        x, y, psi = float(state[0]), float(state[1]), float(state[2])
        # 🔴 19.08 — H1 hafızası (planning_node._on_classified aynası):
        # kapı direği kamera kadrajından çıkınca da (bu parkurda kapılar
        # 20-22 m geniş; hesap: yarım genişlik/menzil > tan(FOV/2) olduğu
        # sürece iki direk AYNI ANDA hiç kadraja sığmaz) EdgeBuoyMemory onu
        # `hatirlananlar()` ile harita yarıçapı içinde TUTAR — üretimin
        # yaptığı TAM budur, atlanırsa GateFollower hiçbir P2 kapısına asla
        # kilitlenemez (ölçüldü: hafızasız sürümde 472/472 örnek fallback).
        tespitler = []
        for bx, by in kapi_direkleri:
            d = math.hypot(bx - x, by - y)
            brg = (math.atan2(by - y, bx - x) - psi + math.pi) % (2 * math.pi) - math.pi
            gorunur = d <= KAMERA_MENZIL and abs(brg) <= KAMERA_FOV / 2
            tespitler.append((bx, by, BUOY_R, EDGE_CLASS_ID if gorunur else CLASS_UNKNOWN))
        kenar_mi = hafiza.siniflandir(tespitler, EDGE_CLASS_ID)
        kenar = [(bx, by) for (bx, by, _, _), k in zip(tespitler, kenar_mi) if k]
        for (bx, by, r, _), k in hafiza.hatirlananlar(
            (x, y), HARITA_YARICAPI, unutma_menzili=UNUTMA_MENZILI
        ):
            if k:
                kenar.append((bx, by))

        # Sarı engeller: menzil+FOV içindeyse HER ZAMAN engel (belirsizlik yok).
        engeller_liste: List[CircleObstacle] = []
        for ex, ey in engel_konumlari:
            d = math.hypot(ex - x, ey - y)
            brg = (math.atan2(ey - y, ex - x) - psi + math.pi) % (2 * math.pi) - math.pi
            if d <= KAMERA_MENZIL and abs(brg) <= KAMERA_FOV / 2:
                engeller_liste.append(CircleObstacle(ex, ey, BUOY_R))

        # B2 huni: kenar direkleri de engel torbasına (komşu-ölçekli pay ile) girer.
        for i, (kx, ky) in enumerate(kenar):
            komsu = [math.hypot(kx - ox, ky - oy)
                     for j, (ox, oy) in enumerate(kenar) if j != i]
            m = HUNI_TAVANI if not komsu else max(
                0.0, min(HUNI_TAVANI, (min(komsu) - HULL_W - 2 * BUOY_R) / 2.0))
            engeller_liste.append(CircleObstacle(kx, ky, BUOY_R, margin=m))
        pipe.set_obstacles(engeller_liste)

        if round(t * 10) % 2 == 0:
            # 🔴 19.08 gece — planning_node._refine_target'ın PARKUR2 baypası
            # aynası: GateFollower burada YANLIŞ model (bkz. dosya başı
            # docstring) — ham GN5'e doğrudan git, MPPI'nin engel kaçınması
            # (yukarıdaki huni'li engeller_liste) işi yapar.
            # 🔴 GN5 VARIŞ NOKTASI DEĞİL, GEÇİLECEK SON KAPININ EŞİĞİDİR
            # (F-K.1'in P2 karşılığı, planning_node._p2_hedefi_oteye_it
            # aynası): ham GN5'e hedeflenince tekne arrival_radius'a girip
            # kapının TAM kirişini geçmeden duruyordu. Referans, araç→GN5
            # doğrultusunda ölçülmüş gövde boyu (HULL_L) kadar ötelenir.
            dxg, dyg = gn5_orta[0] - x, gn5_orta[1] - y
            mesafe_g = math.hypot(dxg, dyg)
            if mesafe_g > 1e-6:
                hedef = (gn5_orta[0] + dxg / mesafe_g * HULL_L,
                         gn5_orta[1] + dyg / mesafe_g * HULL_L)
            else:
                hedef = gn5_orta
            pipe.set_waypoints([hedef])

        pipe.set_state(state)
        u = pipe.compute_control()
        if u is None:
            u = np.zeros(2)
        if not np.all(np.isfinite(u)):
            break
        for _ in range(2):
            state = dyn.step_rk4(state, u, dt / 2)
        t += dt
        _sim_saat[0] = t

        simdi = (float(state[0]), float(state[1]))
        for bx, by in kapi_direkleri:
            pay = math.hypot(bx - simdi[0], by - simdi[1]) - BUOY_R - HULL_W / 2
            en_kucuk_pay = min(en_kucuk_pay, pay)
        for ex, ey in engel_konumlari:
            pay = math.hypot(ex - simdi[0], ey - simdi[1]) - BUOY_R - HULL_W / 2
            en_kucuk_pay = min(en_kucuk_pay, pay)
        for ki, (sol, sag) in enumerate(kapilar_p2):
            if ki not in gecilen:
                a1x, a1y = onceki
                a2x, a2y = simdi
                px, py = sol
                qx, qy = sag

                def yon(o, u_, v):
                    return (u_[0]-o[0])*(v[1]-o[1]) - (u_[1]-o[1])*(v[0]-o[0])
                d1 = yon((px, py), (qx, qy), (a1x, a1y))
                d2 = yon((px, py), (qx, qy), (a2x, a2y))
                d3 = yon((a1x, a1y), (a2x, a2y), (px, py))
                d4 = yon((a1x, a1y), (a2x, a2y), (qx, qy))
                if ((d1 > 0) != (d2 > 0)) and ((d3 > 0) != (d4 > 0)):
                    orta = ((px + qx) / 2.0, (py + qy) / 2.0)
                    gecilen[ki] = math.hypot(simdi[0]-orta[0], simdi[1]-orta[1])
        onceki = simdi

        if math.hypot(state[0]-gn5_orta[0], state[1]-gn5_orta[1]) <= VARIS_YARICAP:
            if dwell_t is None:
                dwell_t = t
            elif t - dwell_t >= DWELL_S:
                break
        else:
            dwell_t = None

    son_kapi_gecildi = (len(kapilar_p2) - 1) in gecilen
    return {
        "gecilen": len(gecilen),
        "gecilen_index": sorted(gecilen),
        "toplam_kapi": len(kapilar_p2),
        "son_kapi_gecildi": son_kapi_gecildi,
        "gn5_varildi": math.hypot(state[0]-gn5_orta[0], state[1]-gn5_orta[1]) <= VARIS_YARICAP,
        "en_kucuk_pay": en_kucuk_pay,
        "carpma": en_kucuk_pay <= 0.0,
        "sure": t,
        # şartname md 5.5.2.4: tamamlama = (>=2 duba ikilisi) VE (son ikiliden geçmek)
        "parkur2_tamamlandi": len(gecilen) >= 2 and son_kapi_gecildi,
    }


def main() -> None:
    ap = argparse.ArgumentParser(description="Parkur-2 kapı geçme ölçümü")
    ap.add_argument("--kosum", type=int, default=3)
    ap.add_argument("--sure", type=float, default=VARSAYILAN_SURE_S)
    ap.add_argument("--K", type=int, default=URETIM_K)
    ap.add_argument("--T", type=int, default=URETIM_T)
    a = ap.parse_args()

    dunya = parkuru_oku()
    if not dunya.kapilar_p2:
        print("HATA: parkur_nihai.world'de Parkur-2 kapıları (p2_a/p2_u) yok")
        sys.exit(1)

    # Başlangıç: Parkur-1'in SON kapısının hemen ötesi (P1→P2 gerçek geçişi).
    son_p1 = dunya.kapilar[-1]
    baslangic = ((son_p1[0][0] + son_p1[1][0]) / 2.0 + 3.0,
                 (son_p1[0][1] + son_p1[1][1]) / 2.0)
    yon0 = 0.0   # +x'e dönük (P2 koridoru +x boyunca uzanıyor)

    print(f"Parkur-2: {len(dunya.kapilar_p2)} kapı çifti · "
          f"{len(dunya.engeller)} sarı engel · başlangıç {baslangic} · "
          f"GN5={dunya.gn5}")
    print(f"MPPI K={a.K} T={a.T}{'  ← ÜRETİM AYARI' if (a.K, a.T)==(1000,50) else ''}\n")

    sonuclar = []
    for i in range(1, a.kosum + 1):
        r = kosum(dunya.kapilar_p2, dunya.engeller, baslangic=baslangic,
                  yon0_rad=yon0, gn5_gercek=dunya.gn5,
                  sure=a.sure, mppi_k=a.K, mppi_t=a.T)
        sonuclar.append(r)
        print(f"#{i}: kapı {r['gecilen']}/{r['toplam_kapi']} {r['gecilen_index']} · "
              f"son kapı geçildi={r['son_kapi_gecildi']} · GN5 varıldı={r['gn5_varildi']} · "
              f"pay={r['en_kucuk_pay']:.2f}m · ÇARPMA={r['carpma']} · "
              f"süre={r['sure']:.0f}s · "
              f"P2 TAMAMLANDI (şartname 5.5.2.4)={r['parkur2_tamamlandi']}")

    n_tamam = sum(1 for r in sonuclar if r["parkur2_tamamlandi"])
    n_carpma = sum(1 for r in sonuclar if r["carpma"])
    print(f"\n{'='*60}\nParkur-2 tamamlama: {n_tamam}/{len(sonuclar)} · "
          f"ÇARPMA: {n_carpma}/{len(sonuclar)} · "
          f"ort kapı {statistics.mean(r['gecilen'] for r in sonuclar):.1f}/"
          f"{len(dunya.kapilar_p2)}")


if __name__ == "__main__":
    main()
