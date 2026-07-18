"""
Girdap İDA — Yerel harita PNG dumper (ROS-bağımsız çekirdek).

Şartname 4.2 — Dosya 3:
    Lokal harita / cost map / engel haritası, ≥1 Hz, png seri. Görev bitiminden
    20 dk içinde teslim; her gecikmiş dosya 5 ceza puanı.

Layer 2 `local_map_node` bu çekirdeği sarar: OccupancyGrid mesajını alır,
`write_frame` ile diske grayscale PNG serisi yazar. rclpy bağımsız olduğundan
dönüşüm mantığı pytest ile .venv altında doğrulanır.

OccupancyGrid → 8-bit grayscale eşlemesi:
    değer  0   → PNG 255  (beyaz = serbest su)
    değer 100  → PNG   0   (siyah = engel)
    değer -1   → PNG 128  (gri   = bilinmiyor)
    0..100 arası → lineer (255 → 0)

ROS OccupancyGrid satır 0'ı güney (min y) tutar; kuzey-yukarı PNG için satırlar
dikey çevrilir (flipud).
"""

from __future__ import annotations

from datetime import datetime
from pathlib import Path
from typing import Optional, Sequence, Union

import numpy as np
from PIL import Image

# Bilinmiyor (-1) hücrenin sabit gri tonu.
UNKNOWN_GRAY = 128


def occupancy_to_gray(value: int) -> int:
    """Tek OccupancyGrid değeri → 8-bit gri ton (0-255)."""
    if value < 0:                                # -1 (ve olası diğer negatifler)
        return UNKNOWN_GRAY
    v = 100 if value > 100 else value
    return int(round(255.0 * (100 - v) / 100.0))


def _grid_to_gray(arr: np.ndarray) -> np.ndarray:
    """(H, W) int OccupancyGrid → (H, W) uint8 gri ton (vektörize)."""
    known = np.clip(arr, 0, 100).astype(np.float64)
    gray = np.rint(255.0 * (100.0 - known) / 100.0)
    gray = np.where(arr < 0, UNKNOWN_GRAY, gray)
    return gray.astype(np.uint8)


class LocalMapDumper:
    """OccupancyGrid → grayscale PNG serisi yazıcı (oturum dizinli).

    Boot'ta ~/girdap_logs/local_map/session_YYYYMMDD_HHMMSS/ oluşturur;
    her `write_frame` çağrısı frame_00000.png, frame_00001.png ... yazar.
    """

    def __init__(
        self,
        base_dir: Optional[Union[str, Path]] = None,
        session: Optional[str] = None,
    ) -> None:
        base = (
            Path(base_dir).expanduser()
            if base_dir
            else Path.home() / "girdap_logs" / "local_map"
        )
        if session is None:
            session = datetime.now().strftime("session_%Y%m%d_%H%M%S")
        self.session_dir = base / session
        self.session_dir.mkdir(parents=True, exist_ok=True)
        self._count = 0

    def write_frame(
        self, values: Sequence[int], width: int, height: int
    ) -> Path:
        """Bir OccupancyGrid'i (satır-major, ROS) PNG'ye çevirip diske yaz."""
        arr = np.asarray(values, dtype=np.int16).reshape(height, width)
        # ROS satır 0 = güney → PNG üst satır kuzey olsun diye dikey çevir.
        arr = np.flipud(arr)
        gray = _grid_to_gray(arr)
        img = Image.fromarray(gray, mode="L")
        path = self.session_dir / f"frame_{self._count:05d}.png"
        img.save(path)
        self._count += 1
        return path

    @property
    def frame_count(self) -> int:
        return self._count
