"""
Girdap İDA — Fusion bypass (MAVROS EKF pass-through, ROS-bağımsız).

Video modu (use_isam2=false): iSAM2/GTSAM optimizasyonu ATLANIR; MAVROS'un
kendi EKF çıktısı (/mavros/local_position/pose) doğrudan poz olarak iletilir.
Bu modül **gtsam import ETMEZ** — bypass modunda GTSAM hiç yüklenmez.

Yarışma modu (use_isam2=true) prototype.fusion.pipeline.FusionPipeline kullanır.
"""

from __future__ import annotations

import math
from typing import Tuple


def quat_to_yaw(qx: float, qy: float, qz: float, qw: float) -> float:
    """Quaternion → yaw (ψ) rad, ENU/ZYX. Yüzey aracı için roll/pitch küçük."""
    siny = 2.0 * (qw * qz + qx * qy)
    cosy = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.atan2(siny, cosy)


class PosePassthrough:
    """Son MAVROS EKF pozunu tutup doğrudan ileten geçiş (iSAM2 bypass)."""

    def __init__(self) -> None:
        self._x = 0.0
        self._y = 0.0
        self._psi = 0.0
        self._has_pose = False

    def update(self, x: float, y: float, psi: float) -> None:
        """Yeni EKF pozunu kaydet (ENU x, y, yaw)."""
        self._x = float(x)
        self._y = float(y)
        self._psi = float(psi)
        self._has_pose = True

    def current_pose(self) -> Tuple[float, float, float]:
        """(x, y, ψ) döndür. Henüz poz gelmediyse RuntimeError.

        FusionPipeline.current_pose ile aynı sözleşme → node tek arayüzle
        her iki modu da yayınlar.
        """
        if not self._has_pose:
            raise RuntimeError("henüz poz yok (bypass)")
        return self._x, self._y, self._psi

    @property
    def has_pose(self) -> bool:
        return self._has_pose
