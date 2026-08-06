"""
Girdap İDA — RRT* çekirdeği birim testleri (F16.6).

477 satırlık çekirdeğin İLK doğrudan regresyon teli. Faz 10 matematiği elle
doğrulamıştı (rewire maliyet tutarlılığı, informed elips); bu testler o
doğrulamayı deterministik telle dondurur. Pipeline sözleşmesi (F10.1:
start/goal pay içindeyse ValueError, pipeline yakalar) da burada sabitlenir.

Çalıştır: pytest prototype/tests/test_rrt_star.py -v
"""

from __future__ import annotations

import math

import pytest

from prototype.planning.rrt_star import (
    Bounds,
    CircleObstacle,
    RRTStar,
    RRTStarConfig,
)


def _path_length(path) -> float:                         # noqa: ANN001
    return sum(
        math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(path, path[1:])
    )


def _min_clearance(path, obs: CircleObstacle, step: float = 0.05) -> float:
    """Yol poligonunun engel MERKEZİNE en yakın örneklenmiş mesafesi − r."""
    best = math.inf
    for a, b in zip(path, path[1:]):
        seg = math.hypot(b[0] - a[0], b[1] - a[1])
        n = max(2, int(seg / step))
        for i in range(n + 1):
            t = i / n
            x = a[0] + t * (b[0] - a[0])
            y = a[1] + t * (b[1] - a[1])
            best = min(best, math.hypot(x - obs.cx, y - obs.cy) - obs.r)
    return best


def test_plan_empty_map_connects_start_to_goal() -> None:
    """Engelsiz haritada yol bulunmalı; uçlar start/goal'e oturmalı ve
    toplam uzunluk düz çizginin makul katını aşmamalı."""
    rrt = RRTStar(Bounds(0.0, 50.0, 0.0, 50.0), [], RRTStarConfig(seed=1))
    path = rrt.plan((5.0, 5.0), (45.0, 45.0))
    assert path is not None and len(path) >= 2
    assert math.hypot(path[0][0] - 5.0, path[0][1] - 5.0) < 1e-9   # kök = start
    assert math.hypot(path[-1][0] - 45.0, path[-1][1] - 45.0) <= 1.0  # goal_tolerance
    straight = math.hypot(40.0, 40.0)
    assert straight <= _path_length(path) < 1.5 * straight


def test_plan_path_respects_obstacle_clearance() -> None:
    """Köşegen üstündeki engel: yolun her örneklenmiş noktası engel
    yüzeyinin DIŞINDA kalmalı (çarpışma kontrolü teli)."""
    obs = CircleObstacle(25.0, 25.0, 4.0)
    rrt = RRTStar(Bounds(0.0, 50.0, 0.0, 50.0), [obs], RRTStarConfig(seed=1))
    path = rrt.plan((5.0, 5.0), (45.0, 45.0))
    assert path is not None
    # collision_step=0.2 örneklemesiyle segment içi minik sızma teorik olarak
    # mümkün → tel yüzeyin kendisine konur (safety_margin'e değil).
    assert _min_clearance(path, obs) > 0.0


def test_start_or_goal_inside_margin_raises() -> None:
    """F10.1 sözleşmesi: start/goal (r + safety_margin) içinde → ValueError.
    planning pipeline bu istisnayı yakalayıp eski yolu korur — istisna tipi
    değişirse pipeline'ın try/except'i delinir; bu test onu dondurur."""
    obs = CircleObstacle(10.0, 10.0, 1.0)                # pay: 1.0+0.5=1.5 m
    rrt = RRTStar(Bounds(0.0, 50.0, 0.0, 50.0), [obs], RRTStarConfig(seed=0))
    with pytest.raises(ValueError):
        rrt.plan((10.5, 10.0), (45.0, 45.0))             # start pay içinde
    with pytest.raises(ValueError):
        rrt.plan((5.0, 5.0), (10.0, 11.2))               # goal pay içinde
    with pytest.raises(ValueError):
        rrt.plan((-5.0, 5.0), (45.0, 45.0))              # start bounds dışı


def test_same_seed_is_deterministic() -> None:
    """Aynı seed + aynı sahne → birebir aynı yol (saha tekrarlanabilirliği)."""
    scene = dict(
        bounds=Bounds(0.0, 50.0, 0.0, 50.0),
        obstacles=[CircleObstacle(25.0, 25.0, 3.0)],
    )
    p1 = RRTStar(scene["bounds"], scene["obstacles"], RRTStarConfig(seed=7)).plan(
        (5.0, 5.0), (45.0, 45.0)
    )
    p2 = RRTStar(scene["bounds"], scene["obstacles"], RRTStarConfig(seed=7)).plan(
        (5.0, 5.0), (45.0, 45.0)
    )
    assert p1 == p2


def test_best_cost_matches_path_length() -> None:
    """Rewire maliyet tutarlılığı (Faz 10 el doğrulamasının teli):
    raporlanan best_cost, dönen yolun gerçek Öklit uzunluğu olmalı
    (+ goal snap payı). Tutarsızlık = cost propagate hatası."""
    rrt = RRTStar(
        Bounds(0.0, 50.0, 0.0, 50.0),
        [CircleObstacle(25.0, 25.0, 3.0)],
        RRTStarConfig(seed=3),
    )
    path = rrt.plan((5.0, 5.0), (45.0, 45.0))
    assert path is not None
    # best_cost ağaç içi goal düğümüne kadar; yol son segmentte goal'e
    # snap'lenebilir → tolerans goal_tolerance kadar.
    assert abs(_path_length(path) - rrt.best_cost) <= 1.0 + 1e-6


def test_unreachable_goal_returns_none() -> None:
    """Goal'i çevreleyen engel duvarı: çözüm yok → None (istisna DEĞİL).
    Pipeline 'çözüm yok' dalı bu sözleşmeye dayanır."""
    goal = (45.0, 45.0)
    # Goal'in etrafını 8 büyük daireyle kapat (aralıksız halka)
    ring = [
        CircleObstacle(
            goal[0] + 4.0 * math.cos(k * math.pi / 4),
            goal[1] + 4.0 * math.sin(k * math.pi / 4),
            2.2,
        )
        for k in range(8)
    ]
    rrt = RRTStar(
        Bounds(0.0, 50.0, 0.0, 50.0), ring, RRTStarConfig(seed=0, max_iter=400)
    )
    assert rrt.plan((5.0, 5.0), goal) is None


# --------------------------------------------------------------------------- #
# A3 — DURGUNLUK İLE ERKEN ÇIKIŞ (2026-08-07, GIRDAP_DURUM §0.9f)
#
# planning_node tek-thread executor kullanıyor: plan() ne kadar bloklarsa 20 Hz
# kontrol timer'ı o kadar susuyor (ölçüm: P2 sahnesi ort 548 ms, p95 602 ms).
#
# Önce duvar saati bütçesi (time.perf_counter) denendi ve GERİ ALINDI: aynı
# seed + aynı sahne CPU yüküne göre farklı yol veriyordu, yani hem log tekrar
# oynatma hem de "laptopta ölçtüğüm Orin'de de olur" varsayımı ölüyordu.
# Sabit düşük max_iter de elendi: 60 seed × 4 sahne taramasında max_iter=400
# "yoğun" sahnede 8/60, "dar kapı"da 12/60 ÇÖZÜMSÜZ kaldı — plan()→None ise
# pipeline eski referansı korur (sessiz donma).
#
# Kalan kural adaptif ve belirlenimli: ilk çözümden sonra maliyet
# durgunluk_penceresi iterasyon boyunca iyilesme_esigi kadar bile düşmediyse
# dur. Ölçülen bedel (P2, 60 tohum): süre 548 → 67 ms (8,2×), yol +%2,3,
# başarısız plan sayısı DEĞİŞMEDİ (0/60).
# --------------------------------------------------------------------------- #


def test_A3_durgunluk_sureyi_KISAR_yolu_bozmaz() -> None:
    """Erken çıkış süreyi belirgin kısaltmalı, yol bozulmasının KUYRUĞU sınırlı
    kalmalı.

    Tek seed'e bakmıyoruz: ilk denemede tek tohumla ölçüp "+%2,3" diye
    geçmiştik, oysa o ortalamaydı — gerçek dağılımda p95 +%2,8 ve en kötü hâl
    +%8,0. Kuyruğun kaynağı pencere değil, ilk çözümün düştüğü homotopi sınıfı.
    Bu yüzden test ortalamayı VE en kötü hâli ayrı ayrı bağlar.
    """
    import time

    engeller = [CircleObstacle(x, y, 0.15) for x, y in
                [(28.0, 1.0), (33.0, 3.2), (49.0, 2.0), (53.0, -0.5), (66.0, -2.2)]]
    b = Bounds(-40.0, 140.0, -60.0, 60.0)
    start, goal = (0.0, 0.0), (72.0, -3.0)
    TOHUMLAR = range(12)

    sure_kapali = sure_acik = 0.0
    bozulmalar = []
    for s in TOHUMLAR:
        t0 = time.perf_counter()
        r0 = RRTStar(b, engeller, RRTStarConfig(seed=s, durgunluk_penceresi=0))
        y0 = r0.plan(start, goal)
        sure_kapali += time.perf_counter() - t0

        t0 = time.perf_counter()
        r1 = RRTStar(b, engeller, RRTStarConfig(seed=s))       # varsayılan pencere
        y1 = r1.plan(start, goal)
        sure_acik += time.perf_counter() - t0

        assert y0 is not None and y1 is not None, f"seed={s}: çözüm kayboldu"
        assert y1[0] == pytest.approx(start) and y1[-1] == pytest.approx(goal)
        bozulmalar.append(r1.best_cost / r0.best_cost - 1.0)

    assert sure_acik < 0.6 * sure_kapali, (
        f"erken çıkış süreyi kısmadı: {sure_acik*1e3:.0f} ms ↔ "
        f"{sure_kapali*1e3:.0f} ms"
    )
    ort = sum(bozulmalar) / len(bozulmalar)
    assert ort <= 0.03, f"ortalama yol bozulması çok yüksek: %{ort*100:.1f}"
    assert max(bozulmalar) <= 0.12, (
        f"en kötü yol bozulması çok yüksek: %{max(bozulmalar)*100:.1f}"
    )


def test_A3_erken_cikis_COZUMUN_VARLIGINI_kismaz() -> None:
    """🔴 En kritik kural: erken çıkış ilk çözümden ÖNCE uygulanmaz.

    Erken dönmek `None` demek olurdu; `_global_replan` bunu "eski referansı
    koru" diye yorumluyor → araç bayat bir rotayı sürer (sessiz donma). Bu,
    kesmeye çalıştığımız bloklanmadan daha kötüdür.
    """
    engeller = [CircleObstacle(20.0, 0.0, 3.0)]      # rotayı kapatan büyük engel
    b = Bounds(-10.0, 60.0, -30.0, 30.0)
    # Gülünç derecede dar pencere: yine de çözüm dönmeli.
    r = RRTStar(b, engeller, RRTStarConfig(seed=1, durgunluk_penceresi=1))
    yol = r.plan((0.0, 0.0), (40.0, 0.0))
    assert yol is not None
    # Ve yol gerçekten engelin dışından geçmeli (erken çıkış güvenliği bozmaz).
    for x, y in yol:
        assert math.hypot(x - 20.0, y) >= 3.0, "erken çıkışlı yol engelin içinden geçti"


def test_A3_erken_cikis_KAPALIYKEN_eski_davranis_birebir() -> None:
    """durgunluk_penceresi=0 → tam max_iter (geriye uyumluluk)."""
    b = Bounds(-10.0, 60.0, -30.0, 30.0)
    cfg = RRTStarConfig(seed=2, durgunluk_penceresi=0, max_iter=300)
    r = RRTStar(b, [], cfg)
    r.plan((0.0, 0.0), (40.0, 0.0))
    assert r.iterasyon == cfg.max_iter


def test_A3_erken_cikis_BELIRLENIMLI_kalir() -> None:
    """🔴 Duvar saatinin geri alınma sebebi: aynı seed → birebir aynı yol.

    Bu test yalnız `plan()`ın kendini tekrar etmesini değil, erken çıkış
    KARARININ da yalnız seed+sahneye bağlı olmasını doğrular: iterasyon sayısı
    da birebir eşleşmeli. CPU yüküne bakan bir kriter burada çakardı.
    """
    b = Bounds(-40.0, 140.0, -60.0, 60.0)
    engeller = [CircleObstacle(30.0, 2.0, 1.5), CircleObstacle(55.0, -3.0, 2.0)]
    sonuc = []
    for _ in range(3):
        r = RRTStar(b, engeller, RRTStarConfig(seed=11))
        sonuc.append((r.plan((0.0, 0.0), (100.0, 0.0)), r.iterasyon))
    assert sonuc[0] == sonuc[1] == sonuc[2]
    # Ve erken çıkış gerçekten devrede olmalı (yoksa test boş yere geçer).
    assert sonuc[0][1] < RRTStarConfig().max_iter
