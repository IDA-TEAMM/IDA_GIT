"""
Girdap İDA — Cascade Heading PID kontrolcü (ROS-bağımsız, MPPI'ye alternatif).

F-S.10: son_kodv2 karar katmanı sentezi — ida_topics/decision_node.py'nin
GERÇEK DONANIMDA kanıtlanmış cascade PID mantığı (dış döngü heading→yaw_rate,
iç döngü yaw_rate→açısal düzeltme) buraya taşındı, ama iki iyileştirmeyle:

    1. Engel kaçınma artık kamera bbox yakınlık sezgisi DEĞİL, girdap_decision'ın
       LiDAR tabanlı `CircleObstacle` verisiyle (potansiyel alan itmesi) —
       gerçek dünya/araç çerçevesinde, kamera FOV'una bağımlı değil.
    2. Çıktı arayüzü `MPPIController.step()` ile BİREBİR AYNI: (2,) [T_left,
       T_right] Newton. `PlanningPipeline.compute_control()` iki kontrolcü
       arasında şeffaf geçiş yapabilir (bkz. PlanningPipelineConfig.control_mode).

Neden bir PID yedeği: MPPI/RRT* mimarisi daha yetenekli ama saha/donanımda
uçtan uca hiç koşmadı (F-S.6 bulgusu — /girdap/mission/waypoints hiç publish
edilmiyordu). Bu kontrolcü, MPPI saha kalibrasyonu tamamlanana kadar (ya da
MPPI beklenmedik davranırsa) aynı güvenlik/FSM/mavros_bridge çatısı altında
donanımda kanıtlanmış bir düşme-güvenli (fallback) seçenek sağlar.

Kullanım: PlanningPipeline zaten kurulu obstacles/state/mission_state'i besler;
bu modül yalnız `step()` çağrılır, RRT* global path'e ihtiyaç duymaz (hedefe
düz çizgi + engelden kaçınma — ida_topics'in doğrudan hedefe seyir mantığı).
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import List, Optional, Tuple

import numpy as np

from prototype.planning.rrt_star import CircleObstacle


def _wrap_pi(angle: float) -> float:
    """Açıyı (-π, π] aralığına sar (ida_topics decision_node.py ile aynı)."""
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


@dataclass(frozen=True)
class PidControllerConfig:
    """ida_topics/decision_node.py sabitleriyle aynı varsayılanlar (OUTER_KP,
    INNER_KP/KD, MAX_YAW_RATE/MAX_ANGULAR) — donanımda doğrulanmış değerler.
    """

    outer_kp: float = 1.0            # heading hatası → istenen yaw_rate
    inner_kp: float = 1.0            # yaw_rate hatası → açısal düzeltme
    inner_kd: float = 0.0
    max_yaw_rate: float = 1.0        # rad/s
    max_angular: float = 1.0         # normalize açısal komut sınırı
    cruise_thrust_n: float = 15.0    # sabit ileri itki (N, max_thrust'ın yarısı)
    max_diff_thrust_n: float = 15.0  # dönüş için ayrılabilecek maks fark (N)
    target_smoothing: float = 0.15   # ida_topics smoothed_target_yaw katsayısı
    # F-S.10: LiDAR engel kaçınma (potansiyel alan) — girdap'ın CircleObstacle
    # verisiyle, kamera bbox sezgisinin (ida_topics) yerini alır. Kapalı-döngü
    # fiziksel simülasyonla (test_planning_pipeline.py) ölçülerek kalibre
    # edildi — dönüş ivmesi/menzil düşünülmeden seçilen küçük bir pay gerçek
    # çarpışmaya yol açtı (bkz. F-S.10 test geçmişi).
    obstacle_safety_margin_m: float = 8.0   # engel yarıçapına ek güvenlik payı
    obstacle_avoid_gain: float = 2.5
    # Radyal itmeye eklenen teğetsel (90°) bileşenin oranı — yalnız engel TAM
    # ÖNDEYKEN (radyal, hedef vektörünü doğrudan iptal eder) potansiyel-alan
    # tekil noktasını KARARLI biçimde kırmak için; küçük tutulur ki normal
    # (öndeşik olmayan) engellerde radyal kaçış yönünü EZMESİN.
    obstacle_tangential_ratio: float = 0.3


class CascadeHeadingPidController:
    """Hedefe doğrudan seyir + LiDAR engel kaçınma — MPPI'ye TDD alternatifi.

    `step()` imzası `MPPIController.step()` ile aynıdır: durum vektörü alır,
    (T_left, T_right) Newton döner. `PlanningPipeline` ikisi arasında hiçbir
    çağıran kodu değiştirmeden geçiş yapabilir.
    """

    def __init__(self, cfg: Optional[PidControllerConfig] = None) -> None:
        self.cfg = cfg or PidControllerConfig()
        self._smoothed_target_yaw: Optional[float] = None
        self._prev_yaw_rate_err = 0.0

    def reset(self) -> None:
        """Parkur/hedef değişince yumuşatma+türev geçmişini temizle (soğuk
        başlangıç zikzağı önlenir — MPPI'nin warm-start korumasıyla aynı ruh,
        ama PID durumsuz olduğundan yalnız yumuşatma geçmişi sıfırlanır)."""
        self._smoothed_target_yaw = None
        self._prev_yaw_rate_err = 0.0

    def step(
        self,
        state: np.ndarray,
        target_xy: Tuple[float, float],
        obstacles: Optional[List[CircleObstacle]] = None,
    ) -> np.ndarray:
        """Tek kontrol adımı. `state` = [x, y, ψ, u, v, r] (PlanningPipeline
        ile aynı durum vektörü sözleşmesi). Dönüş (2,) [T_left, T_right] N.
        """
        cfg = self.cfg
        x, y, yaw, _u, _v, yaw_rate = state[:6]
        tx, ty = target_xy

        # ── Dış döngü: hedefe heading (potansiyel-alan: hedef çekimi +
        # engel itmesi VEKTÖREL toplanır — F-S.10 düzeltmesi: saf açısal
        # override, engel TAM ÖNDEYKEN [bearing≈0] işaret belirsizliği
        # (klasik potansiyel-alan tekil noktası) yüzünden kapalı-döngü
        # simülasyonunda gerçek çarpışmaya yol açtı, bkz. test_pid_controller
        # closed-loop testleri). Vektör toplamı bu tekil noktayı ortadan
        # kaldırır: dead-ahead engelde bile tutarlı bir sapma üretir.
        target_yaw_raw = self._resultant_heading(x, y, tx, ty, obstacles or [])
        if self._smoothed_target_yaw is None:
            self._smoothed_target_yaw = target_yaw_raw
        diff = _wrap_pi(target_yaw_raw - self._smoothed_target_yaw)
        self._smoothed_target_yaw += diff * cfg.target_smoothing
        target_yaw = self._smoothed_target_yaw

        yaw_err = _wrap_pi(target_yaw - yaw)
        desired_yaw_rate = max(
            -cfg.max_yaw_rate, min(cfg.max_yaw_rate, yaw_err * cfg.outer_kp)
        )

        # ── İç döngü: yaw_rate → açısal düzeltme (PD) ──
        yaw_rate_err = desired_yaw_rate - yaw_rate
        yaw_rate_err_deriv = yaw_rate_err - self._prev_yaw_rate_err
        self._prev_yaw_rate_err = yaw_rate_err
        nav_angular = max(
            -cfg.max_angular,
            min(
                cfg.max_angular,
                cfg.inner_kp * yaw_rate_err + cfg.inner_kd * yaw_rate_err_deriv,
            ),
        )

        # ── Diferansiyel itki (Newton) — MPPI ile aynı çıktı sözleşmesi ──
        turn = nav_angular * cfg.max_diff_thrust_n
        t_left = cfg.cruise_thrust_n - turn
        t_right = cfg.cruise_thrust_n + turn
        return np.array([t_left, t_right])

    def _resultant_heading(
        self,
        x: float,
        y: float,
        tx: float,
        ty: float,
        obstacles: List[CircleObstacle],
    ) -> float:
        """Hedef-çekim + engel-itme vektörlerinin toplamından istenen heading.

        Her tehdit eden engel için RADYAL (engelden uzağa) BİLEŞENE, sabit
        elle bir TEĞETSEL (radyalin 90° döndürülmüşü) bileşen eklenir — bu,
        engel TAM ÖNDEYKEN (radyal tam ters yönde, hedef vektörünü doğrudan
        iptal ederdi) potansiyel-alan yöntemlerinin bilinen "tekil nokta"
        sorununu KARARLI biçimde çözer (her zaman aynı yöne sapma, salınım
        yok). Yön: teğetsel bileşen radyali +90° döndürür (saat yönü tersi).
        """
        cfg = self.cfg
        goal_dx, goal_dy = tx - x, ty - y
        goal_dist = math.hypot(goal_dx, goal_dy)
        if goal_dist < 1e-6:
            return math.atan2(goal_dy, goal_dx)
        result_x, result_y = goal_dx / goal_dist, goal_dy / goal_dist

        for obs in obstacles:
            dx, dy = x - obs.cx, y - obs.cy         # engelden araca (radyal)
            dist = math.hypot(dx, dy)
            safety = obs.r + cfg.obstacle_safety_margin_m
            if dist >= safety or dist < 1e-6:
                continue
            strength = (safety - dist) / safety * cfg.obstacle_avoid_gain
            rad_x, rad_y = dx / dist, dy / dist
            tan_x, tan_y = -rad_y, rad_x            # +90° döndürülmüş (CCW)
            r = cfg.obstacle_tangential_ratio
            result_x += (rad_x + tan_x * r) * strength
            result_y += (rad_y + tan_y * r) * strength

        if math.hypot(result_x, result_y) < 1e-6:   # tam iptal (çok nadir)
            return math.atan2(goal_dy, goal_dx)
        return math.atan2(result_y, result_x)
