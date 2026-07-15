"""
Katamaran dinamik modeli birim testleri.
Çalıştır: pytest prototype/tests/test_dynamics.py -v
"""

import math

import numpy as np
import pytest
from prototype.dynamics.catamaran import (
    CatamaranDynamics,
    CatamaranParams,
    WaveDisturbance,
)


@pytest.fixture
def model() -> CatamaranDynamics:
    return CatamaranDynamics()


def _make_params(wave: WaveDisturbance) -> CatamaranParams:
    """Test fikstürü — Mitras YAML değerleri + verilen wave."""
    return CatamaranParams(
        mass=30.0,
        inertia_z=5.0,
        Xu=-8.0,
        Yv=-12.0,
        Nr=-3.0,
        thruster_spacing=0.596,
        max_thrust=30.0,
        wave=wave,
    )


def test_fp18_sifir_mass_reddedilir() -> None:
    """F-P.18 (robustness taraması, 2026-07-15): mass<=0 (configs/dynamics.yaml
    yazım hatası) öncesinde derivatives()'ta SESSİZCE inf/nan üretirdi —
    artık __post_init__ anında net ValueError verir."""
    with pytest.raises(ValueError, match="mass"):
        CatamaranParams(
            mass=0.0, inertia_z=5.0, Xu=-8.0, Yv=-12.0, Nr=-3.0,
            thruster_spacing=0.596, max_thrust=30.0,
        )


def test_fp18_sifir_inertia_reddedilir() -> None:
    with pytest.raises(ValueError, match="inertia_z"):
        CatamaranParams(
            mass=30.0, inertia_z=0.0, Xu=-8.0, Yv=-12.0, Nr=-3.0,
            thruster_spacing=0.596, max_thrust=30.0,
        )


def test_fp18_sifir_thruster_spacing_reddedilir() -> None:
    """thruster_spacing=0 → yaw torku tamamen kaybolur (direksiyon yetkisi
    yok) — bu da sessizce geçmemeli."""
    with pytest.raises(ValueError, match="thruster_spacing"):
        CatamaranParams(
            mass=30.0, inertia_z=5.0, Xu=-8.0, Yv=-12.0, Nr=-3.0,
            thruster_spacing=0.0, max_thrust=30.0,
        )


def test_fp18_normal_degerler_kabul_edilir() -> None:
    """Regresyon: normal (pozitif) değerler hâlâ sorunsuz kurulmalı."""
    CatamaranParams(
        mass=30.0, inertia_z=5.0, Xu=-8.0, Yv=-12.0, Nr=-3.0,
        thruster_spacing=0.596, max_thrust=30.0,
    )


def test_duragan_arac_hareketsiz(model: CatamaranDynamics) -> None:
    """Sıfır kontrol → sıfır türev (durağan araç yerinde kalır)."""
    state = np.zeros(6)
    control = np.zeros(2)
    derivs = model.derivatives(state, control)
    np.testing.assert_allclose(derivs, np.zeros(6), atol=1e-10)


def test_esit_thrust_duz_ileri(model: CatamaranDynamics) -> None:
    """Eşit thrust → yaw değişmemeli, araç ileri gitmeli."""
    state0 = np.zeros(6)  # psi=0, düz ileri
    control = np.array([10.0, 10.0])
    history = model.simulate(state0, np.tile(control, (200, 1)), dt=0.05)
    final = history[-1]
    # İleri gitmiş olmalı
    assert final[0] > 1.0, "x artmalıydı"
    # Yön değişmemeli
    np.testing.assert_allclose(final[2], 0.0, atol=0.01)


def test_asimetrik_thrust_donus(model: CatamaranDynamics) -> None:
    """Asimetrik thrust → yaw hızı oluşmalı."""
    state = np.zeros(6)
    control = np.array([5.0, 15.0])  # sağ > sol → sola dönüş
    derivs = model.derivatives(state, control)
    assert derivs[5] > 0, "r_dot pozitif olmalıydı (sola dönüş)"


def test_yerinde_donus(model: CatamaranDynamics) -> None:
    """Ters thrust → yerinde dönüş (x,y sabit kalmalı)."""
    state0 = np.zeros(6)
    control = np.array([-10.0, 10.0])
    history = model.simulate(state0, np.tile(control, (100, 1)), dt=0.05)
    final = history[-1]
    # Konum fazla değişmemeli
    assert abs(final[0]) < 1.0, "x fazla kaymış"
    assert abs(final[1]) < 1.0, "y fazla kaymış"
    # Yaw dönmüş olmalı
    assert abs(final[2]) > 0.5, "psi dönmemiş"


def test_thrust_siniri(model: CatamaranDynamics) -> None:
    """Max thrust sınırı aşılmamalı."""
    state = np.zeros(6)
    control = np.array([999.0, 999.0])  # aşırı değer
    derivs = model.derivatives(state, control)
    # Sınırlanmış thrust ile hesaplanmış olmalı
    max_accel = 2 * model.p.max_thrust / model.p.mass
    assert derivs[3] <= max_accel + 1e-6


# --------------------------------------------------------------------------- #
# Wave bozucusu testleri
# --------------------------------------------------------------------------- #


def test_wave_disabled_yaml_default() -> None:
    """YAML'dan yüklendiğinde wave varsayılan kapalı (geriye uyumluluk)."""
    params = CatamaranParams.from_yaml()
    assert params.wave.enabled is False


def test_wave_disabled_t_invariant() -> None:
    """wave.enabled=False iken derivatives t'den bağımsız."""
    dyn = CatamaranDynamics(_make_params(WaveDisturbance(enabled=False)))
    state = np.zeros(6)
    ctrl = np.zeros(2)
    d_t0 = dyn.derivatives(state, ctrl, t=0.0)
    d_t1 = dyn.derivatives(state, ctrl, t=1.0)
    d_t100 = dyn.derivatives(state, ctrl, t=100.0)
    np.testing.assert_allclose(d_t0, d_t1, atol=1e-12)
    np.testing.assert_allclose(d_t0, d_t100, atol=1e-12)
    # Sıfır kontrol + sıfır state → tamamen sıfır türev
    np.testing.assert_allclose(d_t0, np.zeros(6), atol=1e-12)


def test_wave_enabled_oscillates() -> None:
    """wave.enabled=True iken Fx_wave sinüsoidal — beklenen genlik/faz."""
    Fx_amp = 10.0
    wave = WaveDisturbance(
        enabled=True, Fx_amp=Fx_amp, Fx_freq=1.0,  # 1 Hz
        Mz_amp=0.0, Mz_freq=0.0, phase=0.0,
    )
    dyn = CatamaranDynamics(_make_params(wave))
    state = np.zeros(6)
    ctrl = np.zeros(2)

    # Sin başlangıçta 0 → u_dot = 0
    d_t0 = dyn.derivatives(state, ctrl, t=0.0)
    np.testing.assert_allclose(d_t0[3], 0.0, atol=1e-12)

    # Çeyrek periyot (t=0.25 @ 1Hz) → sin(π/2)=1 → Fx_wave = Fx_amp
    # u_dot = Fx_wave / mass
    d_quarter = dyn.derivatives(state, ctrl, t=0.25)
    expected_u_dot = Fx_amp / dyn.p.mass
    np.testing.assert_allclose(d_quarter[3], expected_u_dot, atol=1e-9)

    # Yarım periyot → sin(π)=0 → tekrar 0
    d_half = dyn.derivatives(state, ctrl, t=0.5)
    np.testing.assert_allclose(d_half[3], 0.0, atol=1e-9)


def test_wave_enabled_yaw_disturbance() -> None:
    """Mz_wave yaw eksenine bozucu uygular — r_dot oscillates."""
    wave = WaveDisturbance(
        enabled=True, Fx_amp=0.0, Fx_freq=0.0,
        Mz_amp=2.0, Mz_freq=0.5, phase=0.0,
    )
    dyn = CatamaranDynamics(_make_params(wave))
    state = np.zeros(6)
    ctrl = np.zeros(2)

    # 0.5 Hz @ t=0.5 → sin(π/2)=1 → Mz_wave=2.0 → r_dot = 2.0/inertia_z
    d = dyn.derivatives(state, ctrl, t=0.5)
    expected_r_dot = 2.0 / dyn.p.inertia_z
    np.testing.assert_allclose(d[5], expected_r_dot, atol=1e-9)


def test_wave_simulate_propagates_time() -> None:
    """simulate(t0) wave fazını adım adım yürütmeli — toplam dürtü > 0."""
    wave = WaveDisturbance(
        enabled=True, Fx_amp=10.0, Fx_freq=0.5,
        Mz_amp=0.0, Mz_freq=0.0, phase=math.pi / 2,  # cos faz → t=0'da maks
    )
    dyn = CatamaranDynamics(_make_params(wave))
    # Sıfır kontrol — tüm hareket dalgadan
    n = 20  # 1 s @ dt=0.05
    history = dyn.simulate(np.zeros(6), np.zeros((n, 2)), dt=0.05, t0=0.0)
    # x ekseninde net hareket olmalı (yarım periyot 1 s → faz π → bir yönde itki)
    assert abs(history[-1, 0]) > 0.01, "Wave hareketi etkisiz kaldı"


def test_wave_yaml_roundtrip() -> None:
    """YAML'a wave eklendiğinde alanlar doğru yüklenmeli."""
    # Mevcut dynamics.yaml'da wave: enabled=false; saha YAML override edilince
    # alanlar erişilebilir olmalı
    params = CatamaranParams.from_yaml()
    assert hasattr(params.wave, "Fx_amp")
    assert hasattr(params.wave, "Mz_freq")
    # Tipler doğru
    assert isinstance(params.wave.enabled, bool)
    assert isinstance(params.wave.Fx_amp, (int, float))