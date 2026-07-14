"""
Girdap İDA — Yerel harita PNG dumper testi (Şartname 4.2 Dosya-3).

local_map_node ROS-bağımsız çekirdeği (LocalMapDumper) üzerinden doğrular:
    - 3 kare dökümü → oturum dizininde 3 PNG (3±1)
    - Son PNG: 100×100, mode 'L' (grayscale)
    - Bilinen hücre eşlemesi: OG 100 → PNG 0, OG 0 → 255, OG -1 → 128
    - PlanningPipeline.local_cost_grid engel/serbest/bilinmiyor üretimi

Çalıştır: pytest prototype/tests/test_local_map.py -v
"""

from __future__ import annotations

import numpy as np
import pytest
from PIL import Image

from prototype.mapping.local_map import (
    LocalMapDumper,
    UNKNOWN_GRAY,
    occupancy_to_gray,
)
from prototype.planning.pipeline import PlanningPipeline, PlanningPipelineConfig
from prototype.planning.rrt_star import Bounds, CircleObstacle


# --------------------------------------------------------------------------- #
# Sahte OccupancyGrid (100×100, karışık değerler)
# --------------------------------------------------------------------------- #


def _fake_grid(w: int = 100, h: int = 100):
    """Serbest (0) zemin + engel bloğu (100) + bilinmiyor bloğu (-1)."""
    arr = np.zeros((h, w), dtype=np.int16)
    arr[10:15, 20:25] = 100         # engel bloğu (güneyden 10. satır)
    arr[80:85, 80:85] = -1          # bilinmiyor bloğu
    return arr, arr.reshape(-1).tolist()


# --------------------------------------------------------------------------- #
# Skaler eşleme
# --------------------------------------------------------------------------- #


def test_occupancy_to_gray_reference() -> None:
    assert occupancy_to_gray(0) == 255       # serbest → beyaz
    assert occupancy_to_gray(100) == 0       # engel → siyah
    assert occupancy_to_gray(-1) == UNKNOWN_GRAY   # bilinmiyor → gri (128)
    assert occupancy_to_gray(50) in (127, 128)     # lineer orta


# --------------------------------------------------------------------------- #
# Dumper — kare üretimi + PNG doğrulaması
# --------------------------------------------------------------------------- #


def test_three_frames_written(tmp_path) -> None:
    """3 tick → oturum dizininde 3 PNG (3±1 toleransı)."""
    dumper = LocalMapDumper(base_dir=tmp_path, session="session_test")
    _, flat = _fake_grid()
    for _ in range(3):
        dumper.write_frame(flat, 100, 100)

    assert dumper.frame_count == 3
    pngs = sorted(dumper.session_dir.glob("frame_*.png"))
    assert 2 <= len(pngs) <= 4               # 3±1
    assert len(pngs) == 3
    assert pngs[0].name == "frame_00000.png"
    assert pngs[-1].name == "frame_00002.png"


def test_last_png_size_and_mode(tmp_path) -> None:
    """Son PNG 100×100 ve grayscale (mode 'L')."""
    dumper = LocalMapDumper(base_dir=tmp_path, session="s")
    _, flat = _fake_grid()
    dumper.write_frame(flat, 100, 100)
    img = Image.open(sorted(dumper.session_dir.glob("*.png"))[-1])
    assert img.size == (100, 100)
    assert img.mode == "L"


def test_known_cell_mapping(tmp_path) -> None:
    """Bilinen hücreler doğru gri tona (kuzey-yukarı flip dahil)."""
    dumper = LocalMapDumper(base_dir=tmp_path, session="s")
    _, flat = _fake_grid(100, 100)
    dumper.write_frame(flat, 100, 100)
    img = Image.open(sorted(dumper.session_dir.glob("*.png"))[-1])

    # flipud: ROS satır r → görüntü satırı (h-1-r)
    # engel hücresi (ROS satır 10, sütun 20) → görüntü (x=20, y=89) = 0 (siyah)
    assert img.getpixel((20, 89)) == 0
    # serbest hücre (ROS satır 0, sütun 0) → görüntü (x=0, y=99) = 255 (beyaz)
    assert img.getpixel((0, 99)) == 255
    # bilinmiyor hücre (ROS satır 80, sütun 80) → görüntü (x=80, y=19) = 128
    assert img.getpixel((80, 19)) == UNKNOWN_GRAY


# --------------------------------------------------------------------------- #
# F-S.5: boyut uyuşmazlığı + disk-dolu koruması
# --------------------------------------------------------------------------- #


def test_write_frame_boyut_uyusmazligi_value_error(tmp_path) -> None:
    """values uzunluğu width*height ile uyuşmuyorsa net ValueError yükselir."""
    dumper = LocalMapDumper(base_dir=tmp_path, session="s")
    _, flat = _fake_grid(100, 100)
    with pytest.raises(ValueError, match="boyut uyuşmazlığı"):
        dumper.write_frame(flat, 50, 50)      # 10000 değer ama 50x50=2500 bekleniyor
    assert dumper.frame_count == 0            # başarısız deneme sayılmadı


def test_write_frame_disk_dolu_none_doner_exception_yok(tmp_path, monkeypatch) -> None:
    """F-S.5: disk-dolu (OSError) exception sızdırmaz, None döner."""
    dumper = LocalMapDumper(base_dir=tmp_path, session="s")
    _, flat = _fake_grid(100, 100)

    import PIL.Image as PILImage

    def _patlayan_save(self, path, *a, **kw):  # noqa: ANN001
        raise OSError("disk dolu (simüle)")

    monkeypatch.setattr(PILImage.Image, "save", _patlayan_save)
    result = dumper.write_frame(flat, 100, 100)
    assert result is None
    assert dumper.frame_count == 0            # başarısız kare sayılmadı


# --------------------------------------------------------------------------- #
# Pipeline cost grid üretimi
# --------------------------------------------------------------------------- #


def test_local_cost_grid_obstacle_free_unknown() -> None:
    """local_cost_grid: engel merkezi ~100, uzak hücre 0, arena dışı -1."""
    bounds = Bounds(0.0, 200.0, 0.0, 200.0)
    pipe = PlanningPipeline(bounds, PlanningPipelineConfig())
    # Aracı arena köşesine yakın koy → pencerenin bir kısmı arena dışı (-1).
    pipe.set_state(np.array([10.0, 10.0, 0.0, 0.0, 0.0, 0.0]))
    pipe.set_obstacles([CircleObstacle(10.0, 10.0, 3.0)])   # araç konumunda engel

    cg = pipe.local_cost_grid()
    grid = np.asarray(cg.data, dtype=np.int16).reshape(cg.height, cg.width)

    assert cg.width == 100 and cg.height == 100
    assert grid.max() == 100                 # engel merkezi doygun
    assert (grid == 0).any()                 # serbest su hücreleri var
    assert (grid == -1).any()                # arena dışı bilinmiyor hücreler
    # Değerler sözleşme aralığında
    assert grid.min() >= -1 and grid.max() <= 100
