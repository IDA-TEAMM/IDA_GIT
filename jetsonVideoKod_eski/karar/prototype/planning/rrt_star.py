"""
Girdap İDA — Informed RRT* Global Planlayıcı (2D düzlem)

Amaç:
    Yarışma alanındaki waypoint'ler arasında engelsiz, asimptotik optimal
    referans yörünge üret. Çıktı (x, y) zinciri MPPI'nin takip referansıdır.
    Heading bilinçli olarak bırakılmaz; ψ kontrolü MPPI iç döngüsüne ait.

Algoritma seçimleri (CLAUDE.md ile uyumlu):
    - Steering: doğrusal segment (deniz yüzeyi engelsiz, dubaları MPPI halleder).
    - Sample biasing: %15 goal-bias.
    - Informed RRT* (Gammell 2014): ilk çözüm bulunduktan sonra örnekleme
      kümesi start↔goal ekseninde, c_best ana ekseni olan elipse daraltılır.
      Yakınsama klasik RRT*'a göre belirgin şekilde hızlanır.
    - Engel modeli: dairesel (CircleObstacle). Yarışma dubaları doğal olarak
      disk; LiDAR cluster'ları da bu temsile düşer.

Çıkış formatı:
    plan() → list[tuple[x, y]] (start ⇒ goal) veya None (çözüm bulunamadı).
    nodes propertysi tüm ağaca erişim verir (görselleştirme için).

Çalıştırma (demo + KTR figürü):
    python -m prototype.planning.rrt_star
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from pathlib import Path
from typing import List, Optional, Tuple

import numpy as np

# NOT: matplotlib yalnızca KTR görselleştirmesi (_draw_panel/_demo) içindir;
# runtime ROS node'u (PlanningPipeline) çekmesin diye modül seviyesinde import
# EDİLMEZ, fonksiyon içinde tembel import edilir. (`from __future__ import
# annotations` sayesinde `plt.Axes` tip notu import gerektirmez.)

_REPO_ROOT = Path(__file__).resolve().parents[2]


# --------------------------------------------------------------------------- #
# Veri tipleri
# --------------------------------------------------------------------------- #


@dataclass(frozen=True)
class Bounds:
    """Çalışma alanı dikdörtgeni (m)."""

    x_min: float
    x_max: float
    y_min: float
    y_max: float


@dataclass(frozen=True)
class CircleObstacle:
    """Dairesel engel: (cx, cy) merkezli, r yarıçaplı disk (m)."""

    cx: float
    cy: float
    r: float


@dataclass
class RRTStarConfig:
    """Planlayıcı parametreleri — tüm sihirli sayılar burada toplanır."""

    max_iter: int = 1500
    step_size: float = 1.5            # m, steering uzunluk üst sınırı
    goal_tolerance: float = 1.0       # m, goal'e bu kadar yakın → çözüm
    goal_bias: float = 0.15           # %15 goal-yönelimli örnekleme
    safety_margin: float = 0.3        # m, engel yarıçapına eklenen tampon
    rewire_gamma: float = 50.0        # rewire yarıçap katsayısı (Karaman 2011)
    collision_step: float = 0.2       # m, segment çarpışma örnekleme adımı
    seed: int = 0
    use_informed: bool = True


class _Node:
    """Ağaç düğümü — slots ile bellek ve erişim hızı."""

    __slots__ = ("x", "y", "parent", "children", "cost")

    def __init__(
        self,
        x: float,
        y: float,
        parent: Optional["_Node"] = None,
        cost: float = 0.0,
    ) -> None:
        self.x = x
        self.y = y
        self.parent = parent
        self.children: List["_Node"] = []
        self.cost = cost


# --------------------------------------------------------------------------- #
# Planlayıcı
# --------------------------------------------------------------------------- #


class RRTStar:
    """
    Tek-amaçlı (start → goal) Informed RRT* planlayıcı, 2D Öklit uzayı.

    Tasarım notları:
        - rewire: kural ihlali olmayan en düşük cost'lu komşuyu parent yap, sonra
          komşulardan herhangi biri yeni node üzerinden daha kısa yol
          alabiliyorsa rewire et + alt ağaca cost propagate et (RRT*-tam).
        - Informed örnekleme yalnız c_best < ∞ olduğunda devreye girer; öncesi
          uniform.
        - Nearest neighbor lineer arama (n ≤ 1500'de NumPy ile hızlı).
    """

    def __init__(
        self,
        bounds: Bounds,
        obstacles: List[CircleObstacle],
        cfg: Optional[RRTStarConfig] = None,
    ) -> None:
        self.bounds = bounds
        self.obstacles = obstacles
        self.cfg = cfg or RRTStarConfig()
        self._rng = np.random.default_rng(self.cfg.seed)
        self.nodes: List[_Node] = []
        self._best_goal: Optional[_Node] = None
        self._c_best: float = math.inf
        # Vektörize çarpışma kontrolü için engel dizileri
        if obstacles:
            self._obs_xy = np.array([[o.cx, o.cy] for o in obstacles])
            self._obs_r2 = np.array(
                [(o.r + self.cfg.safety_margin) ** 2 for o in obstacles]
            )
        else:
            self._obs_xy = np.zeros((0, 2))
            self._obs_r2 = np.zeros(0)

    # -------- public --------

    @property
    def best_cost(self) -> float:
        """Bulunan en kısa start→goal yolun toplam Öklit maliyeti (m)."""
        return self._c_best

    def plan(
        self,
        start: Tuple[float, float],
        goal: Tuple[float, float],
    ) -> Optional[List[Tuple[float, float]]]:
        """Plan çalıştır. Çözüm yoksa None döndürür."""
        if not self._point_free(*start) or not self._point_free(*goal):
            raise ValueError("start veya goal engel/sınır içinde")

        self.nodes = [_Node(start[0], start[1])]
        self._best_goal = None
        self._c_best = math.inf

        c_min = math.hypot(goal[0] - start[0], goal[1] - start[1])
        center = ((start[0] + goal[0]) / 2.0, (start[1] + goal[1]) / 2.0)
        angle = math.atan2(goal[1] - start[1], goal[0] - start[0])
        cos_a, sin_a = math.cos(angle), math.sin(angle)

        for _ in range(self.cfg.max_iter):
            x_rand = self._sample(goal, c_min, center, cos_a, sin_a)
            i_near = self._nearest_idx(x_rand)
            nearest = self.nodes[i_near]
            x_new = self._steer((nearest.x, nearest.y), x_rand)

            if not self._segment_free(nearest.x, nearest.y, x_new[0], x_new[1]):
                continue

            near_idx = self._near_indices(x_new)

            # Choose parent: yakın komşular arasında en düşük "cost + segment"
            best_parent = nearest
            best_cost = nearest.cost + math.hypot(
                x_new[0] - nearest.x, x_new[1] - nearest.y
            )
            for idx in near_idx:
                node = self.nodes[idx]
                d = math.hypot(x_new[0] - node.x, x_new[1] - node.y)
                if (
                    node.cost + d < best_cost
                    and self._segment_free(node.x, node.y, x_new[0], x_new[1])
                ):
                    best_parent = node
                    best_cost = node.cost + d

            new_node = _Node(x_new[0], x_new[1], best_parent, best_cost)
            best_parent.children.append(new_node)
            self.nodes.append(new_node)

            # Rewire: yakın komşular yeni node üzerinden daha ucuza gidebilir mi?
            for idx in near_idx:
                node = self.nodes[idx]
                if node is best_parent:
                    continue
                d = math.hypot(node.x - new_node.x, node.y - new_node.y)
                cand = new_node.cost + d
                if cand < node.cost and self._segment_free(
                    new_node.x, new_node.y, node.x, node.y
                ):
                    self._reattach(node, new_node, cand)

            # Goal-yakınsama kontrolü + en iyi maliyeti güncelle
            d_goal = math.hypot(new_node.x - goal[0], new_node.y - goal[1])
            if d_goal <= self.cfg.goal_tolerance:
                total = new_node.cost + d_goal
                if total < self._c_best and self._segment_free(
                    new_node.x, new_node.y, goal[0], goal[1]
                ):
                    self._best_goal = new_node
                    self._c_best = total

            # Mevcut best_goal'in cost'u rewire ile düşmüş olabilir
            if self._best_goal is not None:
                self._c_best = self._best_goal.cost + math.hypot(
                    self._best_goal.x - goal[0], self._best_goal.y - goal[1]
                )

        if self._best_goal is None:
            return None
        return self._extract_path(goal)

    # -------- sampling --------

    def _sample(
        self,
        goal: Tuple[float, float],
        c_min: float,
        center: Tuple[float, float],
        cos_a: float,
        sin_a: float,
    ) -> Tuple[float, float]:
        if self._rng.random() < self.cfg.goal_bias:
            return goal
        if (
            self.cfg.use_informed
            and self._c_best < math.inf
            and self._c_best > c_min
        ):
            return self._sample_informed(c_min, center, cos_a, sin_a)
        return self._sample_uniform()

    def _sample_uniform(self) -> Tuple[float, float]:
        b = self.bounds
        return (
            float(self._rng.uniform(b.x_min, b.x_max)),
            float(self._rng.uniform(b.y_min, b.y_max)),
        )

    def _sample_informed(
        self,
        c_min: float,
        center: Tuple[float, float],
        cos_a: float,
        sin_a: float,
    ) -> Tuple[float, float]:
        # Birim daireye uniform örnek (rejection — 2D'de ~%79 kabul oranı)
        while True:
            u, v = self._rng.uniform(-1.0, 1.0, size=2)
            if u * u + v * v <= 1.0:
                break
        a = self._c_best / 2.0                                # ana yarı-eksen
        b = math.sqrt(self._c_best ** 2 - c_min ** 2) / 2.0    # yan yarı-eksen
        # Lokal koord → global rotation + translation
        x = center[0] + cos_a * (a * u) - sin_a * (b * v)
        y = center[1] + sin_a * (a * u) + cos_a * (b * v)
        bnd = self.bounds
        return (
            min(max(x, bnd.x_min), bnd.x_max),
            min(max(y, bnd.y_min), bnd.y_max),
        )

    # -------- tree ops --------

    def _nearest_idx(self, x_rand: Tuple[float, float]) -> int:
        # Ağaç boyu küçük; np ile vektörize lineer arama yeterince hızlı
        n = len(self.nodes)
        arr = np.empty((n, 2))
        for i, nd in enumerate(self.nodes):
            arr[i, 0] = nd.x
            arr[i, 1] = nd.y
        d2 = (arr[:, 0] - x_rand[0]) ** 2 + (arr[:, 1] - x_rand[1]) ** 2
        return int(np.argmin(d2))

    def _steer(
        self,
        from_xy: Tuple[float, float],
        to_xy: Tuple[float, float],
    ) -> Tuple[float, float]:
        dx = to_xy[0] - from_xy[0]
        dy = to_xy[1] - from_xy[1]
        d = math.hypot(dx, dy)
        if d <= self.cfg.step_size or d < 1e-9:
            return to_xy
        ratio = self.cfg.step_size / d
        return (from_xy[0] + dx * ratio, from_xy[1] + dy * ratio)

    def _near_indices(self, x_new: Tuple[float, float]) -> List[int]:
        n = len(self.nodes)
        # Karaman & Frazzoli: r_n = γ · (log n / n)^(1/d), d=2
        r = min(
            self.cfg.rewire_gamma * math.sqrt(math.log(n + 1) / (n + 1)),
            self.cfg.step_size * 4.0,
        )
        r2 = r * r
        out: List[int] = []
        for i, nd in enumerate(self.nodes):
            if (nd.x - x_new[0]) ** 2 + (nd.y - x_new[1]) ** 2 <= r2:
                out.append(i)
        return out

    def _reattach(self, node: _Node, new_parent: _Node, new_cost: float) -> None:
        """node'u new_parent'a bağla, alt ağaca cost değişimini propagate et."""
        if node.parent is not None:
            node.parent.children.remove(node)
        node.parent = new_parent
        new_parent.children.append(node)
        delta = new_cost - node.cost
        node.cost = new_cost
        # İterasyonel propagation — recursion derinliğini patlatma
        stack = list(node.children)
        while stack:
            ch = stack.pop()
            ch.cost += delta
            stack.extend(ch.children)

    # -------- collision --------

    def _point_free(self, x: float, y: float) -> bool:
        b = self.bounds
        if not (b.x_min <= x <= b.x_max and b.y_min <= y <= b.y_max):
            return False
        if self._obs_xy.shape[0] == 0:
            return True
        d2 = (self._obs_xy[:, 0] - x) ** 2 + (self._obs_xy[:, 1] - y) ** 2
        return bool(np.all(d2 > self._obs_r2))

    def _segment_free(
        self, x1: float, y1: float, x2: float, y2: float
    ) -> bool:
        b = self.bounds
        if not (b.x_min <= x2 <= b.x_max and b.y_min <= y2 <= b.y_max):
            return False
        d = math.hypot(x2 - x1, y2 - y1)
        if d < 1e-9:
            return self._point_free(x1, y1)
        n_samples = max(2, int(d / self.cfg.collision_step) + 1)
        ts = np.linspace(0.0, 1.0, n_samples)
        xs = x1 + (x2 - x1) * ts
        ys = y1 + (y2 - y1) * ts
        if self._obs_xy.shape[0] == 0:
            return True
        # Tüm sample noktaları × tüm engelleri broadcast karşılaştır
        dx = xs[:, None] - self._obs_xy[None, :, 0]
        dy = ys[:, None] - self._obs_xy[None, :, 1]
        d2 = dx * dx + dy * dy
        return bool(np.all(d2 > self._obs_r2[None, :]))

    # -------- output --------

    def _extract_path(
        self, goal: Tuple[float, float]
    ) -> List[Tuple[float, float]]:
        path: List[Tuple[float, float]] = [goal]
        node = self._best_goal
        while node is not None:
            path.append((node.x, node.y))
            node = node.parent
        path.reverse()
        return path


# --------------------------------------------------------------------------- #
# KTR demosu
# --------------------------------------------------------------------------- #


def _draw_panel(
    ax: plt.Axes,
    planner: RRTStar,
    path: Optional[List[Tuple[float, float]]],
    start: Tuple[float, float],
    goal: Tuple[float, float],
    obstacles: List[CircleObstacle],
    bounds: Bounds,
    title: str,
) -> None:
    from matplotlib.patches import Circle    # tembel: yalnız görselleştirme

    for o in obstacles:
        ax.add_patch(Circle((o.cx, o.cy), o.r,
                            color="tab:red", alpha=0.45, zorder=2))
        ax.add_patch(Circle((o.cx, o.cy), o.r + planner.cfg.safety_margin,
                            color="tab:red", fill=False, ls=":",
                            lw=0.8, alpha=0.5, zorder=2))

    # Tüm ağaç kenarları (tek scatter yerine LineCollection daha hızlı olur,
    # ama düğüm sayısı küçük; doğrudan plot)
    for nd in planner.nodes:
        if nd.parent is not None:
            ax.plot([nd.x, nd.parent.x], [nd.y, nd.parent.y],
                    color="lightgray", lw=0.4, zorder=1)

    if path is not None:
        xs = [p[0] for p in path]
        ys = [p[1] for p in path]
        ax.plot(xs, ys, color="tab:purple", lw=2.4, zorder=4,
                label=f"Yol: {planner.best_cost:.2f} m, "
                      f"{len(path)} waypoint")

    ax.scatter(*start, c="tab:green", marker="X",
               s=130, zorder=5, label="Start")
    ax.scatter(*goal, c="black", marker="*",
               s=180, zorder=5, label="Goal")

    ax.set_xlim(bounds.x_min, bounds.x_max)
    ax.set_ylim(bounds.y_min, bounds.y_max)
    ax.set_aspect("equal")
    ax.set_xlabel("x (m)")
    ax.set_ylabel("y (m)")
    ax.set_title(f"{title} — {planner.cfg.max_iter} iter, "
                 f"{len(planner.nodes)} düğüm")
    ax.grid(True, alpha=0.3)
    ax.legend(loc="lower right", fontsize=9)


def _demo() -> None:
    import matplotlib.pyplot as plt          # tembel: yalnız görselleştirme

    bounds = Bounds(0.0, 50.0, 0.0, 50.0)
    # Yarışma duba düzeni temsili: dar geçit + dağınık engeller
    obstacles = [
        CircleObstacle(15.0, 20.0, 3.0),
        CircleObstacle(25.0, 25.0, 4.0),
        CircleObstacle(35.0, 15.0, 3.0),
        CircleObstacle(20.0, 35.0, 3.0),
        CircleObstacle(35.0, 35.0, 3.5),
        CircleObstacle(10.0, 40.0, 2.5),
    ]
    start = (5.0, 5.0)
    goal = (45.0, 45.0)
    out_path = _REPO_ROOT / "docs" / "KTR" / "rrt_star_demo.png"

    fig, axes = plt.subplots(1, 2, figsize=(13, 6.2))
    for ax, use_informed, title in zip(
        axes,
        [False, True],
        ["Standart RRT*", "Informed RRT*"],
    ):
        cfg = RRTStarConfig(use_informed=use_informed, seed=0)
        planner = RRTStar(bounds, obstacles, cfg)
        path = planner.plan(start, goal)
        _draw_panel(ax, planner, path, start, goal, obstacles, bounds, title)
        kind = "Informed" if use_informed else "Standart"
        cost_str = (f"{planner.best_cost:.2f} m"
                    if planner.best_cost < math.inf else "yok")
        print(f"[demo] {kind:9s} → cost = {cost_str},  "
              f"düğüm = {len(planner.nodes)}")

    fig.suptitle("RRT* Global Planlayıcı — yarışma duba düzeni temsili",
                 fontsize=12)
    fig.tight_layout()

    out_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out_path, dpi=150, bbox_inches="tight")
    plt.close(fig)
    print(f"[demo] kaydedildi: {out_path.relative_to(_REPO_ROOT)}")


if __name__ == "__main__":
    _demo()
