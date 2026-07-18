"""
Girdap İDA — local_map_node dayanıklılık testi (F-S.5 hattı, Dosya-3).

Yahya sonkodv3 denetimi (2026-07-15) local_map yazma yolunda korumasızlık
işaret etti. Bizim çekirdeğimizde `write_frame` None DÖNMEZ (dönüş tipi Path)
— onun `path is None` guard'ı burada ölü kod olurdu. Gerçek risk exception:
    - bozuk OccupancyGrid (data uzunluğu != width*height) → np.reshape ValueError
    - disk dolu / IO hatası                               → Image.save OSError
Timer callback'inde yakalanmayan exception node'u düşürür → Dosya-3 (md 4.2)
görev ortasında sessizce kesilir. Kare atlanmalı, node YAŞAMALI.

rclpy gerektirir → sistem python3.10 + ROS Humble; .venv'de SKIP.
"""

from __future__ import annotations

import pytest

rclpy = pytest.importorskip("rclpy", reason="rclpy yok (.venv) — ROS ortamında koş")

from rclpy.parameter import Parameter                    # noqa: E402
from nav_msgs.msg import OccupancyGrid                   # noqa: E402

girdap = pytest.importorskip(
    "girdap_decision.local_map_node",
    reason="girdap_decision source'lanmamış (ros2_ws/install/setup.bash)",
)


@pytest.fixture(scope="module")
def ros_context():                                       # noqa: ANN201
    rclpy.init()
    yield
    rclpy.shutdown()


def _node(tmp_path):                                     # noqa: ANN001, ANN201
    return girdap.LocalMapNode(
        parameter_overrides=[
            Parameter("output_dir", Parameter.Type.STRING, str(tmp_path)),
        ]
    )


def _grid(width: int, height: int, n_cells: int) -> OccupancyGrid:
    m = OccupancyGrid()
    m.info.width = width
    m.info.height = height
    m.data = [0] * n_cells
    return m


def test_bozuk_grid_kareyi_atlar_node_yasar(ros_context, tmp_path) -> None:
    """data uzunluğu width*height ile uyuşmuyorsa (reshape ValueError) kare
    atlanır; node ayakta kalır ve SONRAKİ sağlam kare yazılır."""
    node = _node(tmp_path)
    try:
        node._on_map(_grid(4, 4, 9))                     # 9 != 16 → bozuk
        node._on_tick()                                  # patlamamalı
        assert node._dumper.frame_count == 0, "bozuk kare yazıldı"

        node._on_map(_grid(4, 4, 16))                    # sağlam kare
        node._on_tick()
        assert node._dumper.frame_count == 1, (
            "bozuk kareden sonra node yazmayı sürdürmedi (Dosya-3 kesildi)"
        )
    finally:
        node.destroy_node()


def test_disk_hatasi_kareyi_atlar_node_yasar(ros_context, tmp_path, monkeypatch) -> None:
    """Diske yazma OSError verirse (disk dolu) node ölmemeli — Dosya-3 kaydı
    görev ortasında sessizce durmamalı."""
    node = _node(tmp_path)
    try:
        def _boom(*_a, **_k):
            raise OSError(28, "No space left on device")

        monkeypatch.setattr(node._dumper, "write_frame", _boom)
        node._on_map(_grid(4, 4, 16))
        node._on_tick()                                  # patlamamalı
    finally:
        node.destroy_node()
