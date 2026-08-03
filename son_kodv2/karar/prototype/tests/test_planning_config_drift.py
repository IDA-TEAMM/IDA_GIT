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
