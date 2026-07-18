"""
Girdap İDA — Fusion bypass (MAVROS EKF pass-through) testleri.

use_isam2=false modu: /mavros/local_position/pose doğrudan /girdap/pose'a
iletilir. Çekirdek PosePassthrough'u doğrular + video modunda GTSAM'ın hiç
yüklenmediğini (import bile edilmediğini) garanti eder.

Çalıştır: pytest prototype/tests/test_fusion_bypass.py -v
"""

from __future__ import annotations

import inspect
import math
import re

import pytest

from prototype.fusion.bypass import PosePassthrough, quat_to_yaw


def test_passthrough_forwards_same_pose() -> None:
    pp = PosePassthrough()
    with pytest.raises(RuntimeError):        # poz gelmeden erişim hatası
        pp.current_pose()
    assert pp.has_pose is False
    pp.update(3.0, -2.0, 0.5)                # fake EKF pose
    assert pp.current_pose() == (3.0, -2.0, 0.5)   # aynı veri ~1 tick sonra
    assert pp.has_pose is True


def test_passthrough_latest_wins() -> None:
    pp = PosePassthrough()
    pp.update(1.0, 1.0, 0.0)
    pp.update(5.0, 6.0, 1.2)
    assert pp.current_pose() == (5.0, 6.0, 1.2)


def test_quat_to_yaw_reference() -> None:
    assert abs(quat_to_yaw(0.0, 0.0, 0.0, 1.0)) < 1e-9      # kimlik → 0
    s = math.sqrt(0.5)
    assert abs(quat_to_yaw(0.0, 0.0, s, s) - math.pi / 2) < 1e-6   # +90° yaw


def test_bypass_module_imports_no_gtsam() -> None:
    """Video modu: bypass modülü GTSAM import ETMEMELİ (import bile şart değil).

    Docstring GTSAM'dan bahsedebilir; asıl kontrol gerçek import satırıdır.
    """
    import prototype.fusion.bypass as bypass_mod
    src = inspect.getsource(bypass_mod)
    assert not re.search(r"^\s*import\s+gtsam", src, re.MULTILINE)
    assert not re.search(r"^\s*from\s+gtsam", src, re.MULTILINE)
