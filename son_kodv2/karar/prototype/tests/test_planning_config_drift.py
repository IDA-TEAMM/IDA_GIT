"""
Girdap İDA — MPPI saha tuning yüzeyinin config-drift kapıları (ROS'SUZ).

Aynı varsayılan üç yerde yaşıyor:
    prototype/planning/mppi.py      MPPIConfig            (TEK GERÇEK KAYNAK)
    launch/hardware.launch.py       _MPPI_DEFAULTS        (launch/CLI kopyası)
    config/params.yaml              planning_node bloğu   (saha tuning yüzeyi)

Launch dosyası `prototype`'ı import ETMEZ (--show-args ROS'suz/numpy'siz
makinede de çalışmalı) → kopya kaçınılmaz. Sessiz drift ise kabul edilemez:
kod varsayılanı değişip launch'ta kalırsa `ros2 launch` ile `ros2 run` FARKLI
davranır ve bu sahada fark edilmez. Bu testler kopyaları bağlar.

launch/launch_ros GEREKTİRMEZ (dosya `ast` ile okunur) — CI'ın ROS'suz
çekirdek job'ında da koşar (F16.2 kapılama dersi: gerçekten koşabilen test).
"""

from __future__ import annotations

import ast
from pathlib import Path

import pytest
import yaml

from prototype.planning.mppi import MPPIConfig

_PKG_DIR = (
    Path(__file__).resolve().parents[2] / "ros2_ws" / "src" / "girdap_decision"
)
_LAUNCH_FILE = _PKG_DIR / "launch" / "hardware.launch.py"
_PARAMS_FILE = _PKG_DIR / "config" / "params.yaml"
_HARDWARE_FILE = _PKG_DIR / "config" / "hardware.yaml"

# launch anahtarı → MPPIConfig alanı (λ hariç: nöbetçi değerli, ayrı test)
_ESLEME = {
    "mppi_sigma_u": "sigma_u",
    "mppi_obstacle_margin": "obstacle_margin",
    "mppi_terminal_mode": "terminal_mode",
    "mppi_terminal_lookahead_m": "terminal_lookahead_m",
    "mppi_ref_window_size": "ref_window_size",
    "mppi_ref_window_enabled": "ref_window_enabled",
}


# Launch sabitleri `(değer, tip)` demetleri tutuyor; `float`/`int`/... ast'te
# literal DEĞİL (Name) → literal_eval tek başına yetmez, tip adlarını çözeriz.
_TIPLER = {"float": float, "int": int, "str": str, "bool": bool}


def _coz(dugum: ast.AST):
    if isinstance(dugum, ast.Name) and dugum.id in _TIPLER:
        return _TIPLER[dugum.id]
    if isinstance(dugum, ast.Tuple):
        return tuple(_coz(e) for e in dugum.elts)
    if isinstance(dugum, ast.Dict):
        return {_coz(k): _coz(v) for k, v in zip(dugum.keys, dugum.values)}
    return ast.literal_eval(dugum)


def _sabit(ad: str) -> dict:
    """Launch dosyasındaki modül düzeyi `<ad> = {...}` sabitini ast ile oku."""
    agac = ast.parse(_LAUNCH_FILE.read_text(encoding="utf-8"))
    for dugum in agac.body:
        hedefler = (
            dugum.targets if isinstance(dugum, ast.Assign)
            else [dugum.target] if isinstance(dugum, ast.AnnAssign)
            else []
        )
        for hedef in hedefler:
            if isinstance(hedef, ast.Name) and hedef.id == ad:
                return _coz(dugum.value)
    raise AssertionError(f"{ad} launch dosyasında bulunamadı")


def _yaml(path: Path) -> dict:
    return yaml.safe_load(path.read_text(encoding="utf-8")) or {}


def test_launch_varsayilanlari_mppiconfig_ile_ayni() -> None:
    """`_MPPI_DEFAULTS` ↔ `MPPIConfig`: değer VE tip birebir."""
    kod = MPPIConfig()
    launch = _sabit("_MPPI_DEFAULTS")
    for launch_key, cfg_key in _ESLEME.items():
        deger, tip = launch[launch_key]
        beklenen = getattr(kod, cfg_key)
        assert deger == beklenen, (
            f"{launch_key}: launch={deger!r} ≠ MPPIConfig.{cfg_key}={beklenen!r}"
            " — kod varsayılanı değiştiyse launch'ı da güncelle"
        )
        assert isinstance(beklenen, tip), f"{launch_key} tipi {tip} olmalı"


def test_lambda_nobetcisi_profili_kazandirir() -> None:
    """λ launch/yaml varsayılanı 0.0 = "parkur profili kazansın" nöbetçisi.
    Buraya gerçek bir λ yazılırsa üç parkurun profil değeri sessizce ezilir."""
    from prototype.planning.pipeline import _PARKUR_PROFILES

    assert _sabit("_MPPI_DEFAULTS")["mppi_lambda"][0] == 0.0
    assert _yaml(_PARAMS_FILE)["planning_node"]["ros__parameters"][
        "mppi_lambda"
    ] == 0.0
    assert _yaml(_HARDWARE_FILE)["planning"]["mppi_lambda"] == 0.0
    # Nöbetçi anlamlı olsun diye profillerin λ'sı pozitif olmalı
    assert all(p.lambda_ > 0.0 for p in _PARKUR_PROFILES.values())


@pytest.mark.parametrize("dosya", ["params", "hardware"])
def test_yaml_anahtarlari_launch_ile_ayni(dosya: str) -> None:
    """yaml'daki mppi_* anahtar KÜMESİ launch/node ile birebir olmalı.

    ROS bilinmeyen yaml anahtarını SESSİZCE atar → `mppi_lambdaa: 50` yazımı
    hiçbir uyarı üretmeden yok sayılırdı (sahada "değiştirdim ama değişmedi").
    """
    if dosya == "params":
        blok = _yaml(_PARAMS_FILE)["planning_node"]["ros__parameters"]
    else:
        blok = _yaml(_HARDWARE_FILE)["planning"]
    anahtarlar = {
        k for k in blok
        if k.startswith("mppi_") and k not in ("mppi_K", "mppi_T")
    }
    assert anahtarlar == set(_sabit("_MPPI_DEFAULTS"))


@pytest.mark.parametrize("dosya", ["params", "hardware"])
def test_yaml_degerleri_kod_varsayilaniyla_ayni(dosya: str) -> None:
    """Sevk edilen yaml'lar kod varsayılanını YANSITMALI — yaml'ı okuyan
    operatör aracın gerçekte hangi ayarla uçtuğunu görebilsin."""
    kod = MPPIConfig()
    if dosya == "params":
        blok = _yaml(_PARAMS_FILE)["planning_node"]["ros__parameters"]
    else:
        blok = _yaml(_HARDWARE_FILE)["planning"]
    for yaml_key, cfg_key in _ESLEME.items():
        assert blok[yaml_key] == getattr(kod, cfg_key), (
            f"{dosya}.yaml {yaml_key}={blok[yaml_key]!r} ≠ "
            f"MPPIConfig.{cfg_key}={getattr(kod, cfg_key)!r}"
        )


def test_launch_argumanlari_aciklamali() -> None:
    """`--show-args` çıktısında her mppi_* argümanı açıklamalı görünsün —
    operatör sınırları (geçit tavanı, λ dejenerasyonu) orada okuyabilmeli."""
    tanimlar = _sabit("_MPPI_DEFAULTS")
    aciklamalar = _sabit("_MPPI_ARG_DESC")
    assert set(tanimlar) == set(aciklamalar)
    assert all(len(v) > 20 for v in aciklamalar.values())


def test_terminal_mode_yaml_degeri_gecerli() -> None:
    """yaml'daki terminal_mode MPPIConfig doğrulamasından geçmeli (geçersiz
    değer node'da WARN + varsayılana düşüş üretir, sessiz kalmaz)."""
    for blok in (
        _yaml(_PARAMS_FILE)["planning_node"]["ros__parameters"],
        _yaml(_HARDWARE_FILE)["planning"],
    ):
        MPPIConfig(terminal_mode=blok["mppi_terminal_mode"])   # ValueError = geçersiz


# --------------------------------------------------------------------------- #
# Kapı takibi (gate following, 2026-08-03) — aynı drift kapısı
# --------------------------------------------------------------------------- #

# launch anahtarı → GateFollowerConfig alanı. Üç anahtar (enabled / class_id /
# use_classified) node'a özgüdür, çekirdek config'te karşılığı YOKTUR.
_GATE_ESLEME = {
    "hull_width_m": "hull_width_m",
    "hull_length_m": "hull_length_m",
}
_GATE_NODE_ONLY = {
    "gate_following_enabled",
    "edge_buoy_class_id",
    "use_classified_obstacles",
}


def test_gate_launch_varsayilanlari_kodla_ayni() -> None:
    """`_GATE_DEFAULTS` ↔ `GateFollowerConfig`: değer VE tip birebir."""
    from prototype.mission.gate_follower import GateFollowerConfig

    kod = GateFollowerConfig()
    launch = _sabit("_GATE_DEFAULTS")
    for launch_key, cfg_key in _GATE_ESLEME.items():
        deger, tip = launch[launch_key]
        beklenen = getattr(kod, cfg_key)
        assert deger == beklenen, (
            f"{launch_key}: launch={deger!r} ≠ "
            f"GateFollowerConfig.{cfg_key}={beklenen!r} — çekirdek varsayılanı "
            "değiştiyse launch'ı da güncelle"
        )
        assert isinstance(beklenen, tip), f"{launch_key} tipi {tip} olmalı"


def test_gate_takibi_varsayilan_ACIK() -> None:
    """Kapı takibi varsayılan AÇIK olmalı — md 5.5.2.2'ye göre Parkur-1/2
    puanı kapıdan geçmekten gelir; sessizce kapalı kalması puan kaybettirir.
    (Kapı görünmüyorken çekirdek zaten ham GN'ye düşer, yani açık olmak
    kapısız senaryoda davranışı DEĞİŞTİRMEZ.)"""
    assert _sabit("_GATE_DEFAULTS")["gate_following_enabled"][0] is True
    assert _yaml(_PARAMS_FILE)["planning_node"]["ros__parameters"][
        "gate_following_enabled"
    ] is True
    assert _yaml(_HARDWARE_FILE)["planning"]["gate_following_enabled"] is True


@pytest.mark.parametrize("dosya", ["params", "hardware"])
def test_gate_yaml_anahtarlari_launch_ile_ayni(dosya: str) -> None:
    """yaml'daki kapı anahtar KÜMESİ launch/node ile birebir (yazım hatası
    kapanı — ROS bilinmeyen anahtarı sessizce atar)."""
    if dosya == "params":
        blok = _yaml(_PARAMS_FILE)["planning_node"]["ros__parameters"]
    else:
        blok = _yaml(_HARDWARE_FILE)["planning"]
    anahtarlar = {
        k for k in blok if k.startswith("gate_") or k.startswith("hull_")
        or k in _GATE_NODE_ONLY
    }
    assert anahtarlar == set(_sabit("_GATE_DEFAULTS"))


@pytest.mark.parametrize("dosya", ["params", "hardware"])
def test_gate_yaml_degerleri_kod_varsayilaniyla_ayni(dosya: str) -> None:
    """Sevk edilen yaml'lar çekirdek varsayılanını YANSITMALI."""
    from prototype.mission.gate_follower import GateFollowerConfig

    kod = GateFollowerConfig()
    if dosya == "params":
        blok = _yaml(_PARAMS_FILE)["planning_node"]["ros__parameters"]
    else:
        blok = _yaml(_HARDWARE_FILE)["planning"]
    for yaml_key, cfg_key in _GATE_ESLEME.items():
        assert blok[yaml_key] == getattr(kod, cfg_key), (
            f"{dosya}.yaml {yaml_key}={blok[yaml_key]!r} ≠ "
            f"GateFollowerConfig.{cfg_key}={getattr(kod, cfg_key)!r}"
        )


def test_kapi_seciminde_ayarlanabilir_esik_KALMADI() -> None:
    """🔑 Tasarım kuralı (2026-08-03): kapı seçiminde tahmine dayalı sayı yok.

    Kapı geometrisi önceden bilinemez (şartname: mesafeler yarışma alanına göre
    değişir + dubalar deniz şartlarıyla yer değiştirir), dolayısıyla "sahada
    ölçüp gir" diye bir eşik OLAMAZ. Geriye yalnız ÖLÇÜLMÜŞ tekne boyutları
    kalmalı; genişlik bandı / menzil / derinlik toleransı / bırakma mesafesi /
    eşleşme yarıçapı hepsi geometriden türetilir.

    Bu test o kuralı DONDURUR: biri geri eklenirse CI kırmızı olur.
    """
    from prototype.mission.gate_follower import GateFollowerConfig

    yasak = {
        "gate_width_min", "gate_width_max", "max_lookahead",
        "pair_depth_tol", "release_distance", "match_radius",
    }
    alanlar = set(GateFollowerConfig.__dataclass_fields__)
    sizan = yasak & alanlar
    assert not sizan, (
        f"GateFollowerConfig'e tahmine dayalı eşik geri gelmiş: {sorted(sizan)}. "
        "Kapı geometrisi önceden bilinemez — türetilmiş geometri kullan."
    )
    assert alanlar == {"hull_width_m", "hull_length_m"}, (
        f"Beklenen alanlar yalnız ölçülmüş tekne boyutları; bulunan: {sorted(alanlar)}"
    )
    # Launch/yaml tarafında da hiçbir eşik kalmamalı.
    launch = set(_sabit("_GATE_DEFAULTS"))
    assert not (yasak & {k.replace("gate_", "") for k in launch})
