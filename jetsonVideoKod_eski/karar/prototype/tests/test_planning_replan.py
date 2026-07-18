"""
Girdap İDA — RRT* replan çağrı sözleşmesi testleri (F10.1 / F10.2).

test_planning_pipeline.py gtsam gerektirir (FusionPipeline importu); bu dosya
yalnız PlanningPipeline + rrt_star kullanır → gtsam'sız ortamda da koşar.

Kapsam:
    F10.1 — start/goal engel payı içindeyken replan istisna fırlatmamalı
            (önceki davranış: ValueError → rclpy callback → planning_node
            görev ortasında ölür).
    F10.2 — start/goal statik bounds dışında (negatif çeyrek) olsa da plan
            üretilmeli (bounds start+goal zarfıyla genişletilir).

Çalıştır: pytest prototype/tests/test_planning_replan.py -v
"""

from __future__ import annotations

import math

import numpy as np

from prototype.planning.pipeline import (
    PlanningPipeline,
    PlanningPipelineConfig,
)
from prototype.planning.rrt_star import Bounds, CircleObstacle


def _fast_cfg() -> PlanningPipelineConfig:
    return PlanningPipelineConfig(mppi_K=200, mppi_T=30)


def _bounds() -> Bounds:
    return Bounds(0.0, 50.0, 0.0, 50.0)


def test_replan_start_inside_obstacle_margin_does_not_raise() -> None:
    """F10.1: araç engel payının (r+safety_margin) içindeyken tetiklenen
    replan istisna FIRLATMAMALI; eski referans yol korunmalı."""
    pipe = PlanningPipeline(_bounds(), _fast_cfg())
    pipe.set_state(np.array([5.0, 5.0, 0.0, 0.0, 0.0, 0.0]))
    pipe.set_waypoints([(40.0, 40.0)])
    old_path = pipe.global_path
    assert old_path is not None
    # Engel tam aracın üstünde → start payın içinde → plan() reddeder
    pipe.set_obstacles([CircleObstacle(5.0, 5.0, 1.0)])   # istisna YOK
    assert pipe.global_path == old_path, "eski referans yol korunmalı"


def test_goal_inside_obstacle_margin_does_not_raise() -> None:
    """F10.1 (goal tarafı): hedef bir engelin payı içindeyse set_waypoints
    çökmemeli (görev callback'i de aynı ölüm zincirini tetikliyordu)."""
    pipe = PlanningPipeline(_bounds(), _fast_cfg())
    pipe.set_state(np.array([5.0, 5.0, 0.0, 0.0, 0.0, 0.0]))
    pipe.set_obstacles([CircleObstacle(40.0, 40.0, 1.0)])
    pipe.set_waypoints([(40.0, 40.0)])                    # istisna YOK
    assert pipe.global_path is None                       # plan üretilemedi


def test_replan_outside_static_bounds_succeeds() -> None:
    """F10.2: start/goal statik bounds'un ([0,200]²) DIŞINDA (negatif çeyrek)
    olsa da plan üretilmeli — bounds start+goal zarfıyla genişletilir."""
    pipe = PlanningPipeline(Bounds(0.0, 200.0, 0.0, 200.0), _fast_cfg())
    pipe.set_state(np.array([-20.0, -10.0, 0.0, 0.0, 0.0, 0.0]))
    pipe.set_waypoints([(-40.0, -30.0)])                  # istisna YOK
    path = pipe.global_path
    assert path is not None, "negatif çeyrekte plan üretilmeliydi"
    assert math.hypot(path[-1][0] + 40.0, path[-1][1] + 30.0) < 2.0
