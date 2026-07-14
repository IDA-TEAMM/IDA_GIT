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
    obs = CircleObstacle(10.0, 10.0, 1.0)                # pay: 1.0+0.3=1.3 m
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
