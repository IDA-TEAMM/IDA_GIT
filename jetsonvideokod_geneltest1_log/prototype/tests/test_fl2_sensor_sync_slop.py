"""
Girdap İDA — F-L.2 kamera-LiDAR zaman toleransı testi (hata_defteri 2026-07-12).

Canlı ölçüm (12.07): Livox stamp'i Jetson saatinden ~0.2 s GERİDE (Livox saat
kaynağı ≠ Jetson saati). ApproximateTimeSynchronizer toleransı 0.1 s bunun
ALTINDA — kayma büyürse eşleşme tamamen durur (perception_fusion_node'un
_sync_watchdog uyarısı bile "sync_slop_s'i büyütmeyi" öneriyor). Defterdeki
düzeltme adayları: restamp / slop 0.3 / PTP → düşük riskli olan seçildi:
slop ≥ 0.3 s (ölçülen 0.2 s + pay). Yanlış eşleşme riskini bearing_tolerance
(0.15 rad) kapılamaya devam eder; duba statik, 0.3 s'de hareket etmez.

Dosya-tabanlı test: tüm konfig kaynakları (hardware.yaml, params.yaml, node
varsayılanı, launch fallback'i) aynı toleransı taşımalı — tek kaynak bayat
kalırsa hangi yoldan açılırsa açılsın davranış aynı olsun.
"""

from __future__ import annotations

import re
from pathlib import Path

import yaml

_REPO = Path(__file__).resolve().parents[2]
_PKG = _REPO / "ros2_ws" / "src" / "girdap_decision"

# Ölçülen Livox kayması ~0.2 s (F-L.2) + pay → tolerans en az 0.3 s olmalı.
_MIN_SLOP_S = 0.3


def test_fl2_hardware_yaml_slop_livox_kaymasini_kapsar() -> None:
    cfg = yaml.safe_load((_PKG / "config" / "hardware.yaml").read_text())
    slop = float(cfg["perception"]["fusion"]["sync_slop_s"])
    assert slop >= _MIN_SLOP_S, (
        f"hardware.yaml sync_slop_s={slop} < {_MIN_SLOP_S} — ölçülen 0.2 s "
        "Livox kaymasını karşılamaz (F-L.2)"
    )


def test_fl2_params_yaml_slop_livox_kaymasini_kapsar() -> None:
    cfg = yaml.safe_load((_PKG / "config" / "params.yaml").read_text())
    slop = float(
        cfg["perception_fusion_node"]["ros__parameters"]["sync_slop_s"]
    )
    assert slop >= _MIN_SLOP_S, (
        f"params.yaml sync_slop_s={slop} < {_MIN_SLOP_S} (F-L.2)"
    )


def test_fl2_node_varsayilani_livox_kaymasini_kapsar() -> None:
    src = (_PKG / "girdap_decision" / "perception_fusion_node.py").read_text()
    m = re.search(r'declare_parameter\("sync_slop_s",\s*([0-9.]+)\)', src)
    assert m, "perception_fusion_node.py sync_slop_s declare_parameter bulunamadı"
    assert float(m.group(1)) >= _MIN_SLOP_S, (
        f"node varsayılanı sync_slop_s={m.group(1)} < {_MIN_SLOP_S} (F-L.2)"
    )


def test_fl2_launch_fallback_livox_kaymasini_kapsar() -> None:
    src = (_PKG / "launch" / "hardware.launch.py").read_text()
    m = re.search(r'"sync_slop_s":\s*\(([0-9.]+),', src)
    assert m, "hardware.launch.py sync_slop_s fallback'i bulunamadı"
    assert float(m.group(1)) >= _MIN_SLOP_S, (
        f"launch fallback sync_slop_s={m.group(1)} < {_MIN_SLOP_S} (F-L.2)"
    )
