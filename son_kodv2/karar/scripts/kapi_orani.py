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
from prototype.mission.parkur_siniri import (                     # noqa: E402
    ParkurDisiSayaci,
    ParkurSiniri,
)
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
def _erisilebilir_seyir_hizi(dyn) -> float:
    """Modelin ULAŞABİLECEĞİ azami seyir hızı (m/s) — sabit sayı DEĞİL, türev.

    🔑 SCT metriğinin paydası "aracın kendi kısıtları altında erişilebilir
    asgari süre"dir; o da erişilebilir azami hızdan gelir. Sabit bir sayı
    yazmak yerine **dinamik modelin kendisinden** türetilir:

        denge:  2·T_max + Xu·u = 0   →   u_max = 2·T_max / |Xu|

    Böylece `dynamics.yaml` güncellenince metrik kendiliğinden düzelir —
    ölçüm ile plant bir daha ayrışamaz.

    Bugünkü değerlerle: 2×1,455 / 2,48 = **1,17 m/s**. Kaptan teyidi:
    *"1 m/s diyebilirsin, fazladır yüksek ihtimalle"*; log 58'de gerçek tekne
    tam gazda **1,26 m/s** ölçülmüştü — üçü aynı bantta.
    ⚠️ `max_thrust` hâlâ suda doğrulanmadı (dynamics.yaml notu); bu hız
    yükselirse süre verimi sayıları DÜŞER (aynı süre daha kötü sayılır).
    """
    return 2.0 * dyn.p.max_thrust / abs(dyn.p.Xu)

#: 🔴 13.08 — ESKİ VARSAYILAN 400 s ÖLÇÜMÜ KESİYORDU: tekne görevi 574-678 s'de
#: bitiriyor, 400 s'de GN 3/4'te kalıyordu. Tavan artık bitişi kesmez.
VARSAYILAN_SURE_S = 900.0

#: 🔴 13.08 — ARAÇ ARTIK ÜRETİM AYARIYLA ÖLÇER (params.yaml: mppi_K/mppi_T).
#: Önceden hız için K=200/T=30 kullanılıyordu ve **başka bir aracı ölçüyorduk**:
#: AYNI başlangıç noktalarıyla kapı oranı **%87,5 → %100**, en küçük gövde payı
#: **0,15 → 0,27 m**. Sebep tasarımda yazılı: `terminal_lookahead_m=3,0` üretim
#: ufkuna göre seçilmiş (kural: la ≥ seyir × ufuk = 1,05 × 2,5 s); T=30'da ufuk
#: 1,5 s'ye düşüp terminal hedef ufkun ~2 katı öteye kaçıyor.
#: ⚠ Bedeli: adım başına ~8 kat hesap. Hızlı yineleme için `--K 200 --T 30`
#: verilebilir AMA o sayılar üretimle KIYASLANAMAZ.
URETIM_K = 1000
URETIM_T = 50


def gorev_rotasi(parkur) -> List[Tuple[float, float]]:
    """Görev noktaları — **GN5 DAHİL**.

    🔴 13.08 ARTEFAKTI (§0.81c): `parkur.guzergah` yalnız GN1-GN4 döndürüyor;
    `parkur_dunyasi.oku()` beşinciyi ayrı alana (`gn5`) koyuyor. Oysa
    **kapı-8'in ortası (34, −1), son görev noktası GN4'ün (32, 5) ÖTESİNDE** —
    tekne GN4'e varınca görev bitiyor ve kapı-8'e HİÇ gitmiyor. Sonuç: araç
    kusursuz çalışsa bile ölçüm **7/8 = %87,5** diyordu; GN5 eklenince
    **8/8 = %100** çıktı. Yani eksik rota, sistematik bir ölçüm hatasıydı.
    """
    rota = list(parkur.guzergah)
    if parkur.gn5 is not None:
        rota.append(parkur.gn5)
    return rota


def _kesisiyor(a, b, p, q) -> bool:
    """[a,b] yol parçası kapı kirişini [p,q] kesiyor mu (geçiş kanıtı)."""

    def yon(o, u, v):
        return (u[0] - o[0]) * (v[1] - o[1]) - (u[1] - o[1]) * (v[0] - o[0])

    d1, d2 = yon(p, q, a), yon(p, q, b)
    d3, d4 = yon(a, b, p), yon(a, b, q)
    return ((d1 > 0) != (d2 > 0)) and ((d3 > 0) != (d4 > 0))


def _sinir_kutusu(parkur, baslangic, gn) -> Bounds:
    """MPPI sınır kutusu — SAHNEDEN türer, elle yazılmaz (§0.86).

    🔴 14.08.2026'DA BULUNDU: kutu `Bounds(-20, 60, -25, 25)` diye sabit
    yazılmıştı. A maddesiyle eklenen **GN5 = (90, 0)** bu kutunun **30 m
    DIŞINDA** kaldı. `w_boundary=1000` bir DUVAR olduğu için tekne
    `x = 59,9`'da durup 900 s'lik tavana kadar orada bekledi — 26 m
    yakınında hiçbir duba yokken, itkinin %13'üyle. Ölçülen "52 m yol
    678 s, **13× yavaş**" sayısı işte bu bekleyiştir; kontrolcünün
    yavaşlığı DEĞİL (§0.86a).

    👉 Ders §0.81c'nin (GN5 artefaktı) aynısı: **sahneyi tarif eden her
    sayı sahneden türetilir.** Kutu artık duba + görev noktası + başlangıç
    kümesinin sınırlayıcı dikdörtgeni + pay.
    """
    xs = [b[0] for b in parkur.dubalar()] + [p[0] for p in gn] + [baslangic[0]]
    ys = [b[1] for b in parkur.dubalar()] + [p[1] for p in gn] + [baslangic[1]]
    # Pay: RRT*/MPPI'nin manevra için kutunun içinde yer bulması gerekir.
    # Terminal lookahead (3 m) + engel payı (1 m) + tekne boyu mertebesinde
    # bir marj yerine, sahnenin kendi ölçeğinin beşte biri alınır — parkur
    # büyürse pay da büyür, uydurma sabit kalmaz.
    pay = max(10.0, 0.2 * max(max(xs) - min(xs), max(ys) - min(ys)))
    return Bounds(min(xs) - pay, max(xs) + pay, min(ys) - pay, max(ys) + pay)


def kosum(
    parkur,
    *,
    baslangic: Tuple[float, float],
    yon_hatasi_rad: float = 0.0,
    model_var: bool = True,
    sure: float = VARSAYILAN_SURE_S,
    huni_tavani: float = HUNI_TAVANI,
    mppi_k: int = URETIM_K,
    mppi_t: int = URETIM_T,
    iz: Optional[List[dict]] = None,
    koridor_var: bool = False,
) -> dict:
    """Kapalı döngüyü bir kez koştur, kapı geçiş metriklerini döndür.

    `iz` verilirse her adım için bir kayıt eklenir (`--iz` kipi, §0.86).
    🔑 NEDEN İTKİ DE KAYDEDİLİYOR: "yavaşlık" tek başına iki farklı arızayı
    aynı sayıya indirger. Ayıran şey KOMUTun kendisidir —
      · itki YÜKSEK + hız ~0  → kuvvetler dengede: **sıkışma** (yerel minimum)
      · itki ~0    + hız ~0  → kontrolcü gitmemeyi SEÇİYOR: **maliyet işlevi**
    İkisi tamamen farklı iş gerektirir; iz olmadan ayrılamazlar.
    """
    kapilar = parkur.kapilar
    dubalar = parkur.dubalar()
    gn = gorev_rotasi(parkur)
    dyn = CatamaranDynamics()
    pipe = PlanningPipeline(
        _sinir_kutusu(parkur, baslangic, gn),
        # ⚠ VARSAYILAN K=200/T=30 HIZ İÇİNDİR, ÜRETİM DEĞİL. Üretim
        # `params.yaml`: K=1000, T=50 → ufuk 2,5 s. `terminal_lookahead_m=3,0`
        # ÜRETİM ufkuna göre seçilmişti (kural: la ≥ seyir × ufuk =
        # 1,05 × 2,5 = 2,62 m). T=30'da ufuk 1,5 s'ye düşer ve terminal hedef
        # ufkun ~2 katı öteye kaçar — CLAUDE.md bunun MPPI'yi "burnu hedefe
        # çevir" davranışına ittiğini ölçmüştü. Yani düşük K/T ile ölçülen
        # yavaşlık ARACIN KENDİ ARTEFAKTI olabilir → `--K/--T` ile sınanır.
        PlanningPipelineConfig(
            mppi_K=mppi_k, mppi_T=mppi_t, mppi_terminal_lookahead_m=3.0
        ),
        dynamics=dyn,
    )
    pipe.set_mission_state("PARKUR1")
    gate = GateFollower(GateFollowerConfig(HULL_W, HULL_L))
    hafiza = EdgeBuoyMemory()

    # F-S.16 — HAKEM ölçüsü: parkur dışına çıkış. Sınır SAHNENİN gerçek kenar
    # dubalarından kurulur (araç onu algıdan türetecek, ölçüm ise hakem gibi
    # gerçeği bilir). Ayrıntı: `prototype/mission/parkur_siniri.py`.
    pdc = ParkurDisiSayaci(ParkurSiniri.kapilardan(kapilar))

    state = np.array(
        [baslangic[0], baslangic[1], yon_hatasi_rad, 0.0, 0.0, 0.0]
    )
    dt, t, idx, algi_no = 0.1, 0.0, 0, 0
    dwell_t: Optional[float] = None
    gecilen: dict = {}
    en_kucuk_pay = 9.9
    onceki = baslangic
    # B) VERİMLİLİK — literatürün iki ölçüsü (bkz. dönüş sözlüğü):
    alinan_yol = 0.0            # SPL payı: fiilen kat edilen yol
    savrulma_rad = 0.0          # toplam |Δψ| — salınım/geri dönüş göstergesi
    onceki_psi = float(state[2])

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
            m = huni_tavani if not komsu else max(
                0.0,
                min(huni_tavani, (min(komsu) - HULL_W - 2 * BUOY_R) / 2.0),
            )
            engeller.append(CircleObstacle(kx, ky, BUOY_R, margin=m))
        pipe.set_obstacles(engeller)
        algi_no += 1

        if round(t * 10) % 2 == 0:                    # görev katmanı 5 Hz
            if model_var:
                sonuc = gate.update(
                    (x, y), gn[idx], kenar,
                    [(o.cx, o.cy, o.r) for o in engeller],
                    gozlem_no=algi_no,
                )
                hedef = sonuc.surus_hedefi
                # F-S.16 (planning_node._koridoru_besle aynası) — koridoru
                # da besle, yoksa bu araç kaptanın 18.08 06:52 bağlantısını
                # ölçemez (koridor hep boş kalır, terim susar).
                if koridor_var:
                    omurga = [
                        ((gx, gy), yari) for gx, gy, yari in gate.gecilen_kapilar
                    ]
                    if sonuc.gate is not None:
                        omurga.append(
                            (sonuc.gate.midpoint, 0.5 * sonuc.gate.width)
                        )
                    pipe.set_koridor(omurga)
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
        parkur_durumu = pdc.adim(simdi, t)
        alinan_yol += math.hypot(simdi[0] - onceki[0], simdi[1] - onceki[1])
        psi_simdi = float(state[2])
        savrulma_rad += abs(
            (psi_simdi - onceki_psi + math.pi) % (2 * math.pi) - math.pi
        )
        onceki_psi = psi_simdi
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

        if iz is not None:
            iz.append({
                "t": t,
                "x": simdi[0], "y": simdi[1],
                "hiz": math.hypot(float(state[3]), float(state[4])),
                "surge": float(state[3]),
                "itki": float(u[0] + u[1]),        # net ileri komut (N)
                "itki_mutlak": float(abs(u[0]) + abs(u[1])),
                "en_yakin_engel": min(
                    (math.hypot(bx - simdi[0], by - simdi[1]) for bx, by in dubalar),
                    default=float("inf"),
                ),
                "gn": idx,
                "parkur_durumu": parkur_durumu,     # F-S.16
            })

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

    pdc.bitir(t)
    sapmalar = list(gecilen.values())
    # --- VERİMLİLİK ÖLÇÜLERİ (literatür: SPL ve dinamik-farkında türevi SCT) ---
    # `en_kisa`: başlangıç → GN zinciri düz çizgi toplamı (aracın kısıtları
    # altında erişilebilir en kısa yol; engeller onu ancak UZATIR).
    en_kisa = 0.0
    onceki_nokta = baslangic
    for hedef in gn:
        en_kisa += math.hypot(hedef[0] - onceki_nokta[0], hedef[1] - onceki_nokta[1])
        onceki_nokta = hedef
    # SPL benzeri yol verimi: en_kisa / alınan_yol ∈ (0, 1]. 1 = kusursuz.
    yol_verimi = en_kisa / alinan_yol if alinan_yol > 1e-6 else float("nan")
    # SCT benzeri süre verimi: erişilebilir asgari süre / gerçek süre.
    # Asgari süre ÖLÇÜLMÜŞ seyir hızından türer (uydurma değil).
    asgari_sure = en_kisa / _erisilebilir_seyir_hizi(dyn)
    sure_verimi = asgari_sure / t if t > 1e-6 else float("nan")
    return {
        "gecilen": len(gecilen),
        "gecilen_index": sorted(gecilen),        # HANGİ kapılar (teşhis)
        "toplam_kapi": len(kapilar),
        "varilan_gn": idx,
        "toplam_gn": len(gn),
        "sapma_ort": statistics.mean(sapmalar) if sapmalar else float("nan"),
        "en_kucuk_pay": en_kucuk_pay,
        "carpma": en_kucuk_pay <= 0.0,
        "sure": t,
        "alinan_yol": alinan_yol,
        "en_kisa_yol": en_kisa,
        "yol_verimi": yol_verimi,       # SPL benzeri (1 = kusursuz)
        "asgari_sure": asgari_sure,
        "sure_verimi": sure_verimi,     # SCT benzeri (1 = kusursuz)
        "savrulma_derece": math.degrees(savrulma_rad),
        # --- F-S.16 parkur dışına çıkma (şartname s.24-25) ---
        "pdc": pdc.etkin_cikis,              # şartnamenin saydığı PDÇ
        "pdc_ham": pdc.cikis_sayisi,         # epizot adedi (40 sn kuralı öncesi)
        "pdc_sure": pdc.toplam_sure,         # dışarıda geçen toplam süre (s)
        "pdc_en_uzun": pdc.en_uzun_sure,     # en uzun tek epizot (s)
        # 🔑 Derinlik olmadan PDÇ yanıltır: sınırı 0,8 m aşmak (ardışık iki
        # duba arasını düz birleştirme YORUMUNUN payı kadar) ile 6 m dışarı
        # savrulmak aynı sayıyı üretir. §0.86a'nın "SPL %107" dersi.
        "pdc_derinlik": pdc.en_derin,        # en derin aşma (m)
        "pdc_puan": pdc.puan(1),             # P1 formülü: 24 − 6×PDÇ
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


#: `--iz` eşikleri. DURGUN eşiği erişilebilir hızın onda biri — sabit sayı
#: değil, plant'tan türer (§0.85a'nın karar ölçütü: |v| < 0,1 m/s).
DURGUN_ORAN = 0.10          # u_max'ın bu kadarının altı "durgun"
EPIZOT_ASGARI_S = 3.0       # bundan uzun durgunluk bir "duraklama epizodu"


def _iz_kosumu(parkur, a) -> None:
    """TEK koşum + hız/itki izi → yavaşlığın kökünü ayır (§0.86).

    Çıktı, §0.85a'da tanımlanan karar ölçütünü doğrudan verir:
    durgun geçen sürenin oranı **>%50 → H2 (duraklama)**, **<%20 → H1
    (düzgün sürünme)**. Aradaki bant "ikisi de var" demektir ve iz
    epizot tablosuyla hangisinin baskın olduğunu gösterir.
    """
    dyn = CatamaranDynamics()
    u_max = _erisilebilir_seyir_hizi(dyn)
    itki_max = 2.0 * dyn.p.max_thrust
    durgun_esik = DURGUN_ORAN * u_max
    poz, aci = baslangiclar(parkur, a.zor, 1)[0]

    iz: List[dict] = []
    r = kosum(parkur, baslangic=poz, yon_hatasi_rad=aci,
              model_var=not a.model_yok, sure=a.sure, huni_tavani=a.huni,
              mppi_k=a.K, mppi_t=a.T, iz=iz, koridor_var=a.koridor)
    if not iz:
        print("iz boş — koşum hiç adım atmadı")
        return

    dt = iz[1]["t"] - iz[0]["t"] if len(iz) > 1 else 0.1
    hizlar = [k["hiz"] for k in iz]
    durgun = [k for k in iz if k["hiz"] < durgun_esik]
    oran = len(durgun) / len(iz)

    print(f"parkur: {len(parkur.kapilar)} kapı · MPPI K={a.K} T={a.T} "
          f"· başlangıç ({poz[0]:.1f}, {poz[1]:.1f}) {math.degrees(aci):.0f}°")
    print(f"erişilebilir hız {u_max:.2f} m/s · durgun eşiği {durgun_esik:.3f} m/s "
          f"· azami itki {itki_max:.2f} N\n")
    print(f"süre {r['sure']:.0f} s · yol {r['alinan_yol']:.1f} m "
          f"· kapı {r['gecilen']}/{r['toplam_kapi']} · GN {r['varilan_gn']}/{r['toplam_gn']}")
    print(f"hız: ortalama {statistics.mean(hizlar):.3f} · "
          f"ortanca {statistics.median(hizlar):.3f} · "
          f"tepe {max(hizlar):.3f} m/s")

    print(f"\n{'='*60}\nKARAR ÖLÇÜTÜ — durgun geçen süre: "
          f"%{100*oran:.1f}  ({len(durgun)*dt:.0f} / {len(iz)*dt:.0f} s)")
    if oran > 0.50:
        print("  → **H2 baskın**: yavaşlık DURAKLAMA epizotlarından geliyor.")
    elif oran < 0.20:
        print("  → **H1 baskın**: tekne her yerde yavaş (düzgün sürünme).")
    else:
        print("  → KARIŞIK: ikisi de var, epizot tablosuna bak.")

    # Durgunken itki ne yapıyor? Asıl ayırt edici bu.
    if durgun:
        itki_durgun = statistics.mean(k["itki_mutlak"] for k in durgun)
        print(f"\nDURGUNKEN İTKİ: ortalama {itki_durgun:.3f} N "
              f"(azaminin %{100*itki_durgun/itki_max:.0f}'i)")
        if itki_durgun > 0.5 * itki_max:
            print("  → komut VAR ama tekne gitmiyor: **kuvvetler dengede / "
                  "sıkışma** (yerel minimum).")
        elif itki_durgun < 0.2 * itki_max:
            print("  → komut da YOK: kontrolcü gitmemeyi SEÇİYOR → "
                  "**maliyet işlevi** (ilerlemeyi ödüllendiren terim eksik).")
        else:
            print("  → ara değer: kısmi itki; iki mekanizma karışık.")

    # Duraklama epizotları
    epizotlar, bas = [], None
    for k in iz:
        if k["hiz"] < durgun_esik:
            if bas is None:
                bas = k
        elif bas is not None:
            if k["t"] - bas["t"] >= EPIZOT_ASGARI_S:
                epizotlar.append((bas, k))
            bas = None
    if bas is not None and iz[-1]["t"] - bas["t"] >= EPIZOT_ASGARI_S:
        epizotlar.append((bas, iz[-1]))

    print(f"\n{EPIZOT_ASGARI_S:.0f} sn'den uzun duraklama: {len(epizotlar)} adet")
    if epizotlar:
        toplam = sum(b["t"] - a0["t"] for a0, b in epizotlar)
        print(f"  toplam {toplam:.0f} s (koşumun %{100*toplam/r['sure']:.0f}'i)")
        print(f"  {'başlangıç':>10} {'süre':>7} {'konum':>16} "
              f"{'en yakın duba':>14} {'GN':>4}")
        for a0, b in sorted(epizotlar, key=lambda e: e[0]["t"] - e[1]["t"])[:10]:
            print(f"  {a0['t']:9.0f}s {b['t']-a0['t']:6.0f}s "
                  f"({a0['x']:6.1f},{a0['y']:6.1f}) "
                  f"{a0['en_yakin_engel']:13.2f}m {a0['gn']:4d}")

    # Hız dağılımı — sürünme mi, dur-kalk mı, gözle görülür olsun
    print("\nhız dağılımı:")
    kenarlar = [0.0, durgun_esik, 0.3, 0.6, 0.9, u_max, 99.0]
    adlar = ["durgun", f"<0.30", "<0.60", "<0.90", f"<{u_max:.2f}", "üstü"]
    for i, ad in enumerate(adlar):
        n = sum(1 for h in hizlar if kenarlar[i] <= h < kenarlar[i + 1])
        print(f"  {ad:>8} {'█' * max(0, round(40 * n / len(hizlar))):<40} "
              f"%{100*n/len(hizlar):4.1f}")


def main() -> None:
    ap = argparse.ArgumentParser(description="Kapı geçme oranı ölçümü")
    ap.add_argument("--kosum", type=int, default=12, help="koşum sayısı")
    ap.add_argument("--zor", action="store_true", help="zorlayıcı başlangıçlar")
    ap.add_argument("--model-yok", action="store_true",
                    help="`.pt` yok: hiçbir duba sınıflanmaz (hepsi engel)")
    ap.add_argument("--sure", type=float, default=400.0)
    ap.add_argument("--K", type=int, default=URETIM_K,
                    help=f"MPPI rollout sayısı (varsayılan ÜRETİM {URETIM_K})")
    ap.add_argument("--T", type=int, default=URETIM_T,
                    help=f"MPPI ufku (varsayılan ÜRETİM {URETIM_T})")
    ap.add_argument("--huni", type=float, default=HUNI_TAVANI,
                    help="kapı direği huni payı TAVANI (m) — süpürme ekseni")
    ap.add_argument("--iz", action="store_true",
                    help="TEK koşumun hız/itki izini çıkar (yavaşlığın kökü: "
                         "düzgün sürünme mi, duraklama epizotları mı?)")
    ap.add_argument("--koridor", action="store_true",
                    help="F-S.16 koridor terimini besle (planning_node."
                         "_koridoru_besle aynası) — kapatılırsa 17.08 taban "
                         "davranışıyla birebir (A/B için)")
    a = ap.parse_args()

    if a.iz:
        _iz_kosumu(parkuru_oku(), a)
        return

    parkur = parkuru_oku()
    basl = baslangiclar(parkur, a.zor, a.kosum)
    print(f"parkur: {len(parkur.kapilar)} kapı "
          f"(açıklık {parkur.kapi_genislikleri[0]:.1f} m) · "
          f"{len(parkur.guzergah)} görev noktası")
    print(f"huni payı tavanı: {a.huni:.2f} m · MPPI K={a.K} T={a.T}"
          f"{'  ← ÜRETİM AYARI' if (a.K, a.T) == (1000, 50) else ''}")
    print(f"kip: {'ZOR' if a.zor else 'normal'}"
          f"{' · MODEL YOK' if a.model_yok else ''} · {a.kosum} koşum\n")
    print(f"{'#':>3} {'başlangıç':>16} {'açı':>6} {'kapı':>7} {'GN':>6} "
          f"{'sapma':>7} {'pay':>7} {'yol%':>6} {'süre%':>6} {'savrul':>8} "
          f"{'PDÇ':>5} {'dışsn':>6} {'derin':>6}")
    oranlar, sapmalar, paylar, carpma = [], [], [], 0
    yol_v, sure_v, savrul = [], [], []
    pdc_list, pdc_sure, pdc_puan, pdc_uzun, pdc_derin = [], [], [], [], []
    for i, (poz, aci) in enumerate(basl, 1):
        r = kosum(parkur, baslangic=poz, yon_hatasi_rad=aci,
                  model_var=not a.model_yok, sure=a.sure, huni_tavani=a.huni,
                  mppi_k=a.K, mppi_t=a.T, koridor_var=a.koridor)
        oran = r["gecilen"] / r["toplam_kapi"]
        oranlar.append(oran)
        paylar.append(r["en_kucuk_pay"])
        if not math.isnan(r["sapma_ort"]):
            sapmalar.append(r["sapma_ort"])
        carpma += int(r["carpma"])
        if not math.isnan(r["yol_verimi"]):
            yol_v.append(r["yol_verimi"])
        if not math.isnan(r["sure_verimi"]):
            sure_v.append(r["sure_verimi"])
        savrul.append(r["savrulma_derece"])
        pdc_list.append(r["pdc"])
        pdc_sure.append(r["pdc_sure"])
        pdc_puan.append(r["pdc_puan"])
        pdc_uzun.append(r["pdc_en_uzun"])
        pdc_derin.append(r["pdc_derinlik"])
        print(f"{i:3d} ({poz[0]:6.1f},{poz[1]:6.1f}) {math.degrees(aci):5.0f}° "
              f"{r['gecilen']:3d}/{r['toplam_kapi']:<3d} "
              f"{r['varilan_gn']:2d}/{r['toplam_gn']:<3d} "
              f"{r['sapma_ort']:6.2f}m {r['en_kucuk_pay']:6.2f}m "
              f"{100*r['yol_verimi']:5.0f}% {100*r['sure_verimi']:5.0f}% "
              f"{r['savrulma_derece']:7.0f}° "
              f"{r['pdc']:5d} {r['pdc_sure']:6.0f} {r['pdc_derinlik']:5.2f}m")

    oranlar.sort()
    print(f"\n{'='*56}\nKAPI GEÇME ORANI — {len(oranlar)} koşum")
    print(f"  ortalama {statistics.mean(oranlar)*100:5.1f}%"
          f" · ortanca {statistics.median(oranlar)*100:5.1f}%"
          f" · en kötü {oranlar[0]*100:5.1f}% · en iyi {oranlar[-1]*100:5.1f}%")
    if sapmalar:
        print(f"  geçiş sapması ortalaması {statistics.mean(sapmalar):.2f} m")
    print(f"  en küçük gövde payı {min(paylar):.2f} m · ÇARPMA {carpma}/{len(oranlar)}")
    # 🔴 VERİMLİLİK (13.08'de eklendi) — yarışma süresi 20 dk ve ÜÇ parkur
    # için; P1 tek başına bütçeyi yerse kapı oranı anlamsızlaşır.
    if yol_v:
        print(f"  YOL VERİMİ  (SPL): ort %{100*statistics.mean(yol_v):.0f}"
              f" · en kötü %{100*min(yol_v):.0f}   (1 = düz gitti)")
    if sure_v:
        print(f"  SÜRE VERİMİ (SCT): ort %{100*statistics.mean(sure_v):.0f}"
              f" · en kötü %{100*min(sure_v):.0f}"
              f"   (taban: modelin erişilebilir hızı"
              f" {_erisilebilir_seyir_hizi(CatamaranDynamics()):.2f} m/s)")
    print(f"  savrulma (toplam |Δψ|): ortanca {statistics.median(savrul):.0f}°")
    # 🔴 F-S.16 (14.08) — 54 puanlık kalem; kapı oranıyla BİRLİKTE okunmalı,
    # çünkü kapı oranını artıran her değişiklik bunu kötüleştirebilir (§0.80).
    print(f"\n  PARKUR DIŞINA ÇIKMA (şartname s.24-25, hakem ölçüsü)")
    print(f"    PDÇ: ortalama {statistics.mean(pdc_list):.2f}"
          f" · ortanca {statistics.median(pdc_list):.1f}"
          f" · en kötü {max(pdc_list)}"
          f" · hiç çıkmayan koşum {sum(1 for v in pdc_list if v == 0)}"
          f"/{len(pdc_list)}")
    print(f"    dışarıda geçen süre: ortalama {statistics.mean(pdc_sure):.0f} s"
          f" · en uzun tek epizot {max(pdc_uzun):.0f} s"
          f" ({'40 sn kuralı TETİKLENDİ' if max(pdc_uzun) > 40 else '40 sn altı'})")
    print(f"    en derin aşma: {max(pdc_derin):.2f} m"
          f" · koşum ortancası {statistics.median(pdc_derin):.2f} m"
          f"   (gövde yarı genişliği {HULL_W/2:.2f} m — bunun altı SIĞ,"
          f" yorum payı kadar)")
    print(f"    P1 SINIR PUANI (24 üzerinden): ortalama"
          f" {statistics.mean(pdc_puan):.1f} · en kötü {min(pdc_puan):.1f}")


if __name__ == "__main__":
    main()
