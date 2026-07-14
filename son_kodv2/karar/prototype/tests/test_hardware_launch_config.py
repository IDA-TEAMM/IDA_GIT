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


# ----- B1/B2 + F-M.6: video blokları hardware.yaml'dan node'lara geçmeli -----


def test_video_bloklari_yamldan_okunur() -> None:
    """hardware.yaml fsm/bridge/telemetry blokları AUTO video değerlerini verir."""
    mod = _load_module()
    try:
        mod.get_package_share_directory(mod._PKG)
    except Exception:
        pytest.skip("girdap_decision share dizini yok — install edilmemiş ortam")
    cfg = mod._load_hardware_config()

    # B1 — görevi FC uçurur: AUTO'da başlar, köprü GUIDED'a zorlamaz.
    assert cfg["fsm"]["start_on_mode"] == "AUTO"
    # F-V.6: "önce AUTO sonra ARM" akışında da görev başlamalı (yoksa Ekran-2
    # setpoint eğrileri boş çıkar).
    assert cfg["fsm"]["start_on_arm_in_mode"] is True
    assert cfg["bridge"]["auto_guided"] is False
    # mode_name GUIDED KALMALI: planning geçidi GUIDED beklediği için AUTO'da
    # cmd_vel yayınlanmaz (MPPI ile FC kavga etmez).
    assert cfg["mode_name"] == "GUIDED"

    # B2 — Ekran-2 kuvvet isteği FC servo çıkışından.
    assert cfg["telemetry"]["setpoint_source"] == "fc"
    assert cfg["telemetry"]["fc_thrust_left_ch"] == 1
    assert cfg["telemetry"]["fc_thrust_right_ch"] == 3

    # F-M.6 — FC 1 Hz sorunu: bağlantıda akış hızı istenir. ALT SINIR 5 Hz:
    # fusion_node pose_timeout_s=1.0 → 1-2 Hz'de odom yayını KESİLİR.
    assert cfg["bridge"]["stream_rate_hz"] >= 5


def test_yaml_okunamazsa_yarisma_varsayilanina_duser(monkeypatch) -> None:
    """Fallback = YARIŞMA modu (GUIDED + MPPI thrust'ı) — video değerleri değil."""
    mod = _load_module()

    def _patlat(_pkg):
        raise FileNotFoundError("paket share dizini yok (test)")

    monkeypatch.setattr(mod, "get_package_share_directory", _patlat)
    cfg = mod._load_hardware_config()

    assert cfg["fsm"]["start_on_mode"] == "GUIDED"
    assert cfg["fsm"]["start_on_arm_in_mode"] is False   # yarışma güvenliği
    assert cfg["bridge"]["auto_guided"] is True
    assert cfg["telemetry"]["setpoint_source"] == "girdap"


def test_fv7_auto_videoda_dwell_sifir() -> None:
    """AUTO'da FC waypoint'te durmaz → sahte bekleme yon_setpoint'i yanıltır."""
    mod = _load_module()
    try:
        mod.get_package_share_directory(mod._PKG)
    except Exception:
        pytest.skip("girdap_decision share dizini yok — install edilmemiş ortam")
    cfg = mod._load_hardware_config()
    assert cfg["mission_timing"]["dwell_time_s"] == 0.0
    assert cfg["mission_timing"]["arrival_radius_m"] > 0.0


def test_gorev_kaynagi_fc_varsayilani() -> None:
    """md 3.3.1(2): görev YKİ'de tanımlanıp İDA'ya YÜKLENİR → kaynak "fc".

    Elle launch edildiğinde sessizce araç-üstü YAML'a düşmek video şartını
    ihlal eder (görev İDA'da hazır beklemiş olur).
    """
    mod = _load_module()
    try:
        mod.get_package_share_directory(mod._PKG)
    except Exception:
        pytest.skip("girdap_decision share dizini yok — install edilmemiş ortam")
    assert mod._load_hardware_config()["mission_source"] == "fc"
