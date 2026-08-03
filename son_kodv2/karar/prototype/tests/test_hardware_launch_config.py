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


def test_with_drivers_node_lari_launch_descriptiona_eklenir() -> None:
    """F-S.2 regresyonu: with_drivers=true olsa da driver_nodes listesi
    (livox_driver_node/oakd_driver_node/kamera_kayit_node) generate_launch_
    description()'ın döndürdüğü LaunchDescription'a hiç EKLENMEMİŞ bulundu
    (gerçek donanım testinde node list'te hiç görünmediler, 2026-07-16).
    Liste inşa ediliyordu ama `return LaunchDescription([...])` içinde
    `*driver_nodes` unutulmuştu — with_drivers flag'i sessizce hiçbir şey
    yapmıyordu.
    """
    mod = _load_module()
    try:
        mod.get_package_share_directory(mod._PKG)
    except Exception:
        pytest.skip("girdap_decision share dizini yok — install edilmemiş ortam")

    ld = mod.generate_launch_description()
    executables = {
        getattr(entity, "_Node__node_executable", None)
        for entity in ld.entities
        if type(entity).__name__ == "Node"
    }
    for exe in ("livox_driver_node", "oakd_driver_node", "kamera_kayit_node"):
        assert exe in executables, (
            f"{exe} generate_launch_description() çıktısında yok — "
            "driver_nodes listesi LaunchDescription'a eklenmemiş"
        )


def test_mavros_respawn_true() -> None:
    """F-P.20 (2026-07-16, gerçek donanım testi): FTDI/seri bağlantı bir
    anlığına EOF verdiğinde mavros_node yakalanmamış bir istisnayla çöküyordu
    (SIGABRT) ve hiçbir şey onu geri başlatmıyordu — karar yığını (planning_
    node/mission_manager_node kendi stale-guard'larıyla GÜVENLİ kalıyor ama
    sistem asla toparlanmıyordu).

    apm.launch/node.launch'ın kendi `respawn_mavros` argümanı VAR ama
    node.launch'taki <node> etiketine hiç bağlanmamış (mavros paketinin
    kendi hatası — canlı testte respawn_mavros:=true geçmesine rağmen
    ikinci çökmede mavros_node bir daha hiç dönmedi, doğrulandı). Bu yüzden
    apm.launch include'u bypass edilip mavros_node doğrudan Node() ile
    respawn=True olarak açılıyor (launch_ros'ta bu gerçekten çalışıyor).
    """
    mod = _load_module()
    try:
        mod.get_package_share_directory(mod._PKG)
    except Exception:
        pytest.skip("girdap_decision share dizini yok — install edilmemiş ortam")

    ld = mod.generate_launch_description()
    mavros_nodes = [
        e for e in ld.entities
        if type(e).__name__ == "Node"
        and getattr(e, "_Node__node_executable", None) == "mavros_node"
    ]
    assert mavros_nodes, (
        "mavros_node Node() eylemi bulunamadı — apm.launch include'u hâlâ "
        "kullanılıyor olabilir (respawn çalışmaz, F-P.20 rejeksiyonu)"
    )
    assert mavros_nodes[0]._ExecuteLocal__respawn is True, (
        "mavros_node respawn=True ile açılmıyor — çökerse sistem kalıcı "
        "olarak kör kalır (F-P.20)"
    )


# ----- iSAM2: kök `fusion:` bloğu (keyframe throttle + robust GPS) -----


def test_isam2_blogu_yamldan_okunur() -> None:
    """hardware.yaml kök `fusion:` bloğu fusion_node parametrelerine düşmeli.

    ⚠ `perception.fusion` (kamera-LiDAR bearing) ile AYNI ada sahip iki ayrı
    blok var; yükleyici bunları karıştırırsa iSAM2 ayarları sessizce
    varsayılanda kalır (throttle/robust hiç devreye girmez).
    """
    mod = _load_module()
    try:
        mod.get_package_share_directory(mod._PKG)
    except Exception:
        pytest.skip("girdap_decision share dizini yok — install edilmemiş ortam")
    cfg = mod._load_hardware_config()

    assert cfg["isam2"]["keyframe_rate_hz"] > 0.0
    assert cfg["isam2"]["gps_robust_enabled"] is True
    assert cfg["isam2"]["gps_huber_k"] == pytest.approx(1.345)

    # gps_sigma_by_status alt sözlüğü düzleştirilmiş param adlarına açılmalı
    assert cfg["isam2"]["gps_sigma_gbas_fix"] == pytest.approx(0.05)
    assert cfg["isam2"]["gps_sigma_sbas_fix"] == pytest.approx(0.50)
    assert cfg["isam2"]["gps_sigma_fix"] == pytest.approx(2.50)

    # RTK ile tek-nokta arasında ciddi ağırlık farkı olmalı (yoksa status
    # okumanın anlamı kalmaz)
    assert (
        cfg["isam2"]["gps_sigma_fix"] / cfg["isam2"]["gps_sigma_gbas_fix"]
    ) >= 10.0

    # perception.fusion BOZULMAMALI (ayrı blok, ayrı anahtar)
    assert cfg["fusion"]["camera_image_width_px"] == 1280


def test_isam2_blogu_yoksa_guvenli_varsayilana_duser() -> None:
    """yaml okunamazsa robust GPS AÇIK + throttle AÇIK kalmalı (güvenli taraf)."""
    mod = _load_module()

    def _patlat(_pkg):
        raise FileNotFoundError("paket share dizini yok (test)")

    monkeypatch = pytest.MonkeyPatch()
    monkeypatch.setattr(mod, "get_package_share_directory", _patlat)
    try:
        cfg = mod._load_hardware_config()
    finally:
        monkeypatch.undo()

    assert cfg["isam2"]["gps_robust_enabled"] is True
    assert cfg["isam2"]["keyframe_rate_hz"] == pytest.approx(5.0)


def test_isam2_launch_argumanlari_fusion_nodea_gecer() -> None:
    """fusion.* launch-arg'ları DECLARE edilip fusion_node'a bağlanmalı.

    F-S.2 sınıfı regresyon: blok okunur ama Node(parameters=...) listesine
    hiç eklenmezse ayar sessizce params.yaml'da kalır.
    """
    mod = _load_module()
    try:
        mod.get_package_share_directory(mod._PKG)
    except Exception:
        pytest.skip("girdap_decision share dizini yok — install edilmemiş ortam")

    ld = mod.generate_launch_description()
    declared = {
        e.name for e in ld.entities
        if type(e).__name__ == "DeclareLaunchArgument"
    }
    for key in mod._ISAM2_DEFAULTS:
        assert f"fusion.{key}" in declared, f"fusion.{key} declare edilmemiş"

    fusion_nodes = [
        e for e in ld.entities
        if type(e).__name__ == "Node"
        and getattr(e, "_Node__node_executable", None) == "fusion_node"
    ]
    assert fusion_nodes, "fusion_node launch açıklamasında yok"
    param_keys: set[str] = set()
    for block in fusion_nodes[0]._Node__parameters or []:
        if isinstance(block, dict):
            param_keys |= {str(k) for k in block}
    for key in mod._ISAM2_DEFAULTS:
        assert key in param_keys, f"{key} fusion_node parametrelerine geçmiyor"


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
