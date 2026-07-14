"""
Girdap İDA — Offline görselleştirici testleri (Sprint 4.5).

Görsel regresyon DEĞİL, çalışma doğrulaması: senaryo koşuyor mu, state doluyor
mu, füzyon renk ataması doğru mu, draw_frame başsız (Agg) hata vermeden çiziyor
mu, MPPI yörüngesi alınıyor mu, GIF kaydı üretiliyor mu.

Çalıştır: pytest prototype/tests/test_viz.py -v
"""

from __future__ import annotations

import pytest

# F16.2b: sistem matplotlib'i numpy-1 ABI'siyle derli — numpy 2.x altında
# AttributeError(_ARRAY_API) fırlatır; importorskip yakalayamaz → elle kapıla.
try:
    import matplotlib
    import matplotlib.transforms  # noqa: F401 — ABI kırığı burada patlar
except Exception as exc:  # ImportError VEYA ABI AttributeError
    pytest.skip(f"matplotlib kullanılamıyor: {exc}", allow_module_level=True)

matplotlib.use("Agg", force=True)                   # başsız — pyplot'tan ÖNCE

from prototype.viz.plotter import (  # noqa: E402
    FUSION_COLORS,
    _color_for,
    draw_frame,
    make_figure,
    save_gif,
)
from prototype.viz.scenario import (  # noqa: E402
    SCENARIOS,
    Buoy,
    VizScenario,
    run_scenario,
    scenario_fusion,
    scenario_parkur1,
    scenario_parkur2,
)

# ---------------------------------------------------------------- senaryo motoru

def test_run_scenario_produces_populated_frames() -> None:
    frames = run_scenario(scenario_parkur1())
    assert len(frames) > 5
    f0 = frames[0]
    assert f0.waypoints                              # waypoint listesi dolu
    assert isinstance(f0.parkur_state, str)
    assert f0.mission_phase in ("ACTIVE", "COMPLETE")
    assert f0.cost_grid is not None                  # cost map üretildi


def test_all_registered_scenarios_run() -> None:
    for builder in SCENARIOS.values():
        frames = run_scenario(builder())
        assert len(frames) > 1
        assert frames[-1].mission_phase == "COMPLETE"


def test_scenario_is_deterministic() -> None:
    a = run_scenario(scenario_fusion())
    b = run_scenario(scenario_fusion())
    assert len(a) == len(b)
    assert a[-1].boat_x == pytest.approx(b[-1].boat_x)
    assert a[-1].boat_y == pytest.approx(b[-1].boat_y)
    assert a[len(a) // 2].parkur_state == b[len(b) // 2].parkur_state


def test_fusion_scenario_completes_full_parkur_chain() -> None:
    # fusion senaryosu labels [1,2,3] → 1→2→3→COMPLETED tam zincir
    frames = run_scenario(scenario_fusion())
    seen = []
    for f in frames:
        if not seen or seen[-1] != f.parkur_state:
            seen.append(f.parkur_state)
    assert seen == ["PARKUR_1", "PARKUR_2", "PARKUR_3", "COMPLETED"]


def test_parkur2_transitions_to_parkur2() -> None:
    frames = run_scenario(scenario_parkur2())
    states = {f.parkur_state for f in frames}
    assert "PARKUR_2" in states                      # 1→2 geçişi görüldü


def test_fusion_assigns_colored_and_unknown_classes() -> None:
    # Orta frame: FOV içi dubalar renkli (0/1/2), yan/arka olanlar 99 (unknown)
    frames = run_scenario(scenario_fusion())
    all_classes = set()
    for f in frames:
        for o in f.obstacles:
            all_classes.add(o.class_id)
    assert 99 in all_classes                          # yan dubalar → unknown
    assert all_classes & {0, 1, 2}                    # en az bir renkli eşleşme


# ---------------------------------------------------------------- renk eşlemesi

def test_fusion_color_mapping() -> None:
    assert _color_for(0) == FUSION_COLORS[0]          # turuncu
    assert _color_for(1) == FUSION_COLORS[1]          # sarı
    assert _color_for(2) == FUSION_COLORS[2]          # kırmızı hedef
    assert _color_for(99) == FUSION_COLORS[99]        # gri unknown
    assert _color_for(12345) == FUSION_COLORS[99]     # bilinmeyen → unknown


# ---------------------------------------------------------------- MPPI yörünge

def test_mppi_trajectory_obtained_when_enabled() -> None:
    # fusion show_mppi=True → gerçek MPPI öngörü yörüngesi (>2 nokta) alınmalı
    frames = run_scenario(scenario_fusion())
    got_real = any(
        f.mppi_traj is not None and len(f.mppi_traj) > 2 for f in frames
    )
    assert got_real


def test_straight_line_fallback_when_mppi_disabled() -> None:
    # parkur1 show_mppi=False → düz çizgi (2 nokta: boat → hedef)
    frames = run_scenario(scenario_parkur1())
    active = [f for f in frames if f.mission_phase == "ACTIVE"]
    assert active
    assert active[0].mppi_traj is not None
    assert len(active[0].mppi_traj) == 2


# ---------------------------------------------------------------- çizim (başsız)

def test_draw_frame_headless_no_error() -> None:
    scenario = scenario_fusion()
    frames = run_scenario(scenario)
    fig, ax = make_figure()
    try:
        for idx in (0, len(frames) // 2, len(frames) - 1):
            draw_frame(ax, frames[idx], scenario)     # hata vermemeli
    finally:
        import matplotlib.pyplot as plt
        plt.close(fig)


def test_save_gif_creates_nonempty_file(tmp_path) -> None:  # noqa: ANN001
    # Küçük hızlı senaryo → GIF kaydı (Pillow, Agg)
    scenario = VizScenario(
        name="mini",
        buoys=[Buoy(6.0, 1.0, true_class=0), Buoy(6.0, -6.0, true_class=1)],
        waypoints=[(8.0, 0.0)],
        parkur_labels=[1],
        boat_start=(0.0, 0.0, 0.0),
        max_frames=12,
        show_cost_map=True,
    )
    frames = run_scenario(scenario)
    out = tmp_path / "mini.gif"
    path = save_gif(frames, scenario, out, fps=10)
    assert path.exists()
    assert path.stat().st_size > 0
