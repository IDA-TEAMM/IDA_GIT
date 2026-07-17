"""
Girdap İDA — Ekran-2 panel üretici testleri (T0 video montaj aracı).

Şartname md 3.3.1.1: video Ekran-2 üç senkron sinyal ister — (a) gerçek hız +
hız setpoint, (b) heading/yaw + yaw setpoint, (c) thrusterlardan kuvvet isteği.
Bu testler grafik CSV'sinden (telemetry_node, GRAPH_CSV_HEADER) panel üreten
offline aracın çekirdeğini doğrular: CSV ayrıştırma (ISO zaman → saniye,
"" → NaN, rad → derece), figür kurulumu (3 eksen), PNG/MP4 çıktısı.

Çalıştır: pytest prototype/tests/test_ekran2.py -v
"""

from __future__ import annotations

import math
import shutil

import numpy as np
import pytest

# F16.2b: sistem matplotlib'i numpy-1 ABI'siyle derli — numpy 2.x altında
# AttributeError(_ARRAY_API) fırlatır; importorskip yakalayamaz → elle kapıla.
try:
    import matplotlib
    import matplotlib.transforms  # noqa: F401 — ABI kırığı burada patlar
except Exception as exc:  # ImportError VEYA ABI AttributeError
    pytest.skip(f"matplotlib kullanılamıyor: {exc}", allow_module_level=True)

matplotlib.use("Agg", force=True)                   # başsız — pyplot'tan ÖNCE

from prototype.telemetry.csv_logger import (  # noqa: E402
    GRAPH_CSV_HEADER,
    GraphSample,
    TelemetryCsvLogger,
)
from prototype.viz.ekran2 import (  # noqa: E402
    break_wraps,
    find_latest_graph_csv,
    load_graph_csv,
    make_figure,
    save_mp4,
    save_png,
)


def _write_graph_csv(tmp_path, rows):
    """Gerçek yazıcıyla (tek doğruluk kaynağı) örnek grafik CSV'si üret.

    rows: (zaman_iso, GraphSample) listesi.
    """
    logger = TelemetryCsvLogger(
        tmp_path, filename="grafik_test.csv", header=GRAPH_CSV_HEADER
    )
    for zaman, sample in rows:
        logger.write_sample(zaman, sample)
    logger.close()
    return logger.path


@pytest.fixture()
def sample_csv(tmp_path):
    """3 satırlık örnek: 10 Hz, 2. satırda thrust yok (bayat öncesi boşluk)."""
    rows = [
        (
            "2026-07-10T12:00:00.000+00:00",
            GraphSample(
                hiz=1.0, hiz_setpoint=1.5, heading=math.pi / 2,
                yon_setpoint=math.pi / 2, thrust_sol=12.3, thrust_sag=-7.0,
            ),
        ),
        (
            "2026-07-10T12:00:00.100+00:00",
            GraphSample(
                hiz=1.1, hiz_setpoint=1.5, heading=math.pi / 2,
                yon_setpoint=math.pi / 2, thrust_sol=None, thrust_sag=None,
            ),
        ),
        (
            "2026-07-10T12:00:00.200+00:00",
            GraphSample(
                hiz=1.2, hiz_setpoint=1.5, heading=0.0,
                yon_setpoint=0.1, thrust_sol=10.0, thrust_sag=10.0,
            ),
        ),
    ]
    return _write_graph_csv(tmp_path, rows)


# ---------------------------------------------------------------- ayrıştırma


def test_load_time_axis_starts_at_zero_seconds(sample_csv):
    data = load_graph_csv(sample_csv)
    assert data.t[0] == pytest.approx(0.0)
    assert data.t[1] == pytest.approx(0.1, abs=1e-6)   # 10 Hz aralık
    assert data.t[2] == pytest.approx(0.2, abs=1e-6)
    assert len(data.t) == 3


def test_load_converts_heading_to_degrees(sample_csv):
    data = load_graph_csv(sample_csv)
    assert data.heading_deg[0] == pytest.approx(90.0, abs=0.01)
    assert data.yon_setpoint_deg[2] == pytest.approx(math.degrees(0.1), abs=0.1)


def test_load_missing_values_become_nan_not_zero(sample_csv):
    """"" → NaN: 0 sanılırsa thrust grafiği sahte sıfır çizer."""
    data = load_graph_csv(sample_csv)
    assert np.isnan(data.thrust_sol[1])
    assert np.isnan(data.thrust_sag[1])
    assert data.thrust_sol[0] == pytest.approx(12.3, abs=0.01)
    assert data.thrust_sag[0] == pytest.approx(-7.0, abs=0.01)


def test_load_rejects_wrong_header(tmp_path):
    """Dosya-2 CSV'si yanlışlıkla verilirse net hata (sessiz çöp grafik değil)."""
    bad = tmp_path / "telemetri_yanlis.csv"
    bad.write_text("zaman,lat,lon\n2026-07-10T12:00:00.000+00:00,41.0,29.0\n")
    with pytest.raises(ValueError, match="GRAPH_CSV_HEADER"):
        load_graph_csv(bad)


def test_load_rejects_empty_csv(tmp_path):
    empty = _write_graph_csv(tmp_path, [])
    with pytest.raises(ValueError, match="veri satırı"):
        load_graph_csv(empty)


def test_break_wraps_inserts_nan_at_pi_jump():
    """±180° sarım sıçraması çizgide dikey artefakt bırakmasın diye NaN'lanır."""
    deg = np.array([170.0, 175.0, -178.0, -170.0])
    out = break_wraps(deg)
    assert np.isnan(out[2])                       # sıçramanın vardığı nokta
    assert out[0] == 170.0 and out[3] == -170.0   # kalanı dokunulmamış
    assert not np.isnan(break_wraps(np.array([10.0, 20.0, 30.0]))).any()


def test_find_latest_graph_csv(tmp_path):
    """Kayıt düzeni (16.07): kayit/<N>/grafik.csv adayları DOSYA ZAMANINA göre
    seçilir — numara boşluk-doldurduğu için en büyük numara ≠ en yeni; eski
    düzen (grafik_*.csv) geçiş için aday kalır, telemetri.csv karışmaz."""
    import os

    d1 = tmp_path / "1"
    d2 = tmp_path / "2"
    d1.mkdir(), d2.mkdir()
    f1 = d1 / "grafik.csv"
    f2 = d2 / "grafik.csv"
    f1.write_text("x"), f2.write_text("x")
    (d2 / "telemetri.csv").write_text("x")               # Dosya-2, karışmasın
    # numara 1 SONRA açıldı (silinen numara yeniden kullanıldı senaryosu)
    os.utime(f2, ns=(1 * 10**9, 1 * 10**9))
    os.utime(f1, ns=(2 * 10**9, 2 * 10**9))
    assert find_latest_graph_csv(tmp_path) == f1

    old = tmp_path / "grafik_20260711T090000Z.csv"       # eski düzen adayı
    old.write_text("x")
    os.utime(old, ns=(3 * 10**9, 3 * 10**9))
    assert find_latest_graph_csv(tmp_path) == old

    with pytest.raises(FileNotFoundError):
        find_latest_graph_csv(tmp_path / "yok")


# ------------------------------------------------------------------- çıktılar


def test_make_figure_has_three_panels(sample_csv):
    data = load_graph_csv(sample_csv)
    fig = make_figure(data)
    try:
        assert len(fig.axes) == 3
        # her panelde en az bir çizgi (gerçek + setpoint → hız panelinde 2)
        assert len(fig.axes[0].lines) >= 2   # hız + hız_setpoint
        assert len(fig.axes[1].lines) >= 2   # heading + yön_setpoint
        assert len(fig.axes[2].lines) >= 2   # thrust sol + sağ
    finally:
        import matplotlib.pyplot as plt

        plt.close(fig)


def test_save_png_creates_nonempty_file(sample_csv, tmp_path):
    data = load_graph_csv(sample_csv)
    out = tmp_path / "ekran2.png"
    save_png(data, out)
    assert out.exists() and out.stat().st_size > 0


@pytest.mark.skipif(shutil.which("ffmpeg") is None, reason="ffmpeg yok")
def test_save_mp4_creates_nonempty_file(sample_csv, tmp_path):
    data = load_graph_csv(sample_csv)
    out = tmp_path / "ekran2.mp4"
    save_mp4(data, out, fps=5)               # 3 satır × küçük fps = hızlı test
    assert out.exists() and out.stat().st_size > 0


def test_thrust_ekseni_birim_alir() -> None:
    """B2: fc modunda thrust % — eksen 'N' yazarsa Ekran-2 yalan söyler."""
    matplotlib = pytest.importorskip("matplotlib")
    matplotlib.use("Agg")
    from prototype.viz.ekran2 import Ekran2Data, make_figure

    n = 5
    data = Ekran2Data(
        t=np.arange(n, dtype=float),
        hiz=np.zeros(n), hiz_setpoint=np.zeros(n),
        heading_deg=np.zeros(n), yon_setpoint_deg=np.zeros(n),
        thrust_sol=np.zeros(n), thrust_sag=np.zeros(n),
        kaynak="test",
    )
    fig = make_figure(data)                                  # varsayılan
    assert "(N)" in fig.axes[2].get_ylabel()
    fig = make_figure(data, thrust_birim="%")                # fc modu
    assert "(%)" in fig.axes[2].get_ylabel()
