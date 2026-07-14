"""
Girdap İDA — hardware.launch.py config yükleyici testi (F-V.5 / F3.3).

hardware.yaml okunamadığında launch SESSİZCE yarışma-modu varsayılanlarına
(use_isam2/use_rrt=True) düşüyordu — video günü YAML yazım hatası, bypass'ı
fark edilmeden kapatırdı (md 3.3.1.1 istemsiz-hareket riski: kalibrasyonsuz
iSAM2+RRT*). Düzeltme: fallback KALIR ama stderr'e gürültülü uyarı basılır.

launch/launch_ros gerektirir (ROS ortamı); yoksa SKIP (CI ROS'suz job'ı).
"""

from __future__ import annotations

import importlib.util
from pathlib import Path

import pytest

pytest.importorskip("launch_ros", reason="launch_ros yok — ROS ortamında koş")

_LAUNCH_FILE = (
    Path(__file__).resolve().parents[2]
    / "ros2_ws" / "src" / "girdap_decision" / "launch" / "hardware.launch.py"
)


def _load_module():
    spec = importlib.util.spec_from_file_location("hw_launch_test", _LAUNCH_FILE)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def test_config_okunamazsa_uyari_basar_ve_varsayilana_duser(
    monkeypatch, capsys
) -> None:
    """F-V.5: hardware.yaml yüklenemezse stderr'e GÜRÜLTÜLÜ uyarı + fallback."""
    mod = _load_module()

    def _patlat(_pkg):
        raise FileNotFoundError("paket share dizini yok (test)")

    monkeypatch.setattr(mod, "get_package_share_directory", _patlat)
    cfg = mod._load_hardware_config()

    # Fallback davranışı korunur (yarışma-modu varsayılanları).
    assert cfg["use_isam2"] is True and cfg["use_rrt"] is True
    assert cfg["mission_source"] == "file"

    # Ama artık SESSİZ değil: stderr'de video-modu riskini söyleyen uyarı var.
    err = capsys.readouterr().err
    assert "hardware.yaml" in err
    assert "use_isam2" in err or "yarışma" in err.lower()


def test_config_normal_yolda_uyari_basmaz(capsys) -> None:
    """Kaynak ağaçtaki gerçek hardware.yaml ile uyarı ÜRETİLMEZ (yanlış alarm yok).

    Not: share dizini kurulu değilse bu test yine uyarı yolunu tetikleyebilir;
    o durumda kaynak yaml'ı share'den okunamıyor demektir — ortam işareti.
    """
    mod = _load_module()
    try:
        mod.get_package_share_directory(mod._PKG)
    except Exception:
        pytest.skip("girdap_decision share dizini yok — install edilmemiş ortam")
    mod._load_hardware_config()
    assert "UYARI" not in capsys.readouterr().err
