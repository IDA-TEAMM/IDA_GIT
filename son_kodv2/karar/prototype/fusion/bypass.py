"""
Girdap İDA — Fusion bypass (MAVROS EKF pass-through, ROS-bağımsız).

Video modu (use_isam2=false): iSAM2/GTSAM optimizasyonu ATLANIR; MAVROS'un
kendi EKF çıktısı (/mavros/local_position/pose) doğrudan poz olarak iletilir.
Bu modül **gtsam import ETMEZ** — bypass modunda GTSAM hiç yüklenmez.

Yarışma modu (use_isam2=true) prototype.fusion.pipeline.FusionPipeline kullanır.
"""

from __future__ import annotations

import math
from typing import Optional, Tuple


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
        # 🕰️ §1.57 — besleyen mesajın ÖLÇÜM zamanı (saniye). `FusionPipeline`
        # ile AYNI sözleşme: iki kol da `son_olcum_zamani` sunar, böylece
        # `fusion_node` hangi kolda olduğunu bilmek zorunda kalmaz.
        self._son_t: Optional[float] = None

    def update(self, x: float, y: float, psi: float,
               t: Optional[float] = None) -> None:
        """Yeni EKF pozunu kaydet (ENU x, y, yaw)."""
        self._x = float(x)
        self._y = float(y)
        self._psi = float(psi)
        self._has_pose = True
        # `t=None` → eski çağrı biçimi; damga taşınmaz, `fusion_node` yayın
        # anına düşer (bilinen ve loglanan hâl).
        if t is not None:
            self._son_t = float(t)

    def current_pose(self) -> Tuple[float, float, float]:
        """(x, y, ψ) döndür. Henüz poz gelmediyse RuntimeError.

        FusionPipeline.current_pose ile aynı sözleşme → node tek arayüzle
        her iki modu da yayınlar.
        """
        if not self._has_pose:
            raise RuntimeError("henüz poz yok (bypass)")
        return self._x, self._y, self._psi

    @property
    def son_olcum_zamani(self) -> Optional[float]:
        """Besleyen pozun ölçüm zamanı (saniye), yoksa None."""
        return self._son_t

    @property
    def has_pose(self) -> bool:
        return self._has_pose
