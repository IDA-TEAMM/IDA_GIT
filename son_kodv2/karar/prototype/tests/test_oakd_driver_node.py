"""
Girdap İDA — oakd_driver_node yeniden bağlanma testleri (F-S.8, 18.08.2026).

SAHA BULGUSU: `_capture_loop`, ilk bağlantı başarısız olduğunda (soğuk
açılışta USB enumeration yarışı) ya da çalışırken cihaz koptuğunda
`self.queue`'yu `None` bırakıyordu ama `_init_depthai()`'a hiçbir zaman
TEKRAR gitmiyordu — kamera görevin geri kalanında sessizce kör kalıyordu.
Bu dosya iki şeyi kilitler: (1) kaynak sözleşmesi — boş kuyruk dalı gerçekten
yeniden bağlanmayı DENİYOR mu (2) davranış — `_init_depthai()` art arda
çağrılınca (başarısız → başarılı) `self.queue` gerçekten kurtarılıyor mu.

rclpy gerektirir → ROS ortamında koş:
    source /opt/ros/humble/setup.bash && source ros2_ws/install/setup.bash
    python3 -m pytest prototype/tests/test_oakd_driver_node.py -v

.venv'de (rclpy yok) otomatik SKIP edilir.
"""

from __future__ import annotations

import re
import sys
import types
from pathlib import Path
from typing import Optional

import pytest

rclpy = pytest.importorskip("rclpy", reason="rclpy yok (.venv) — ROS ortamında koş")

oakd = pytest.importorskip(
    "ida_topics.oakd_driver_node",
    reason="ida_topics source'lanmamış (ros2_ws/install/setup.bash)",
)

_SRC = Path(oakd.__file__).read_text(encoding="utf-8")


# --------------------------------------------------------------------------- #
# 1) Kaynak sözleşmesi — REGRESYON KİLİDİ
# --------------------------------------------------------------------------- #


def test_bos_kuyrukta_yeniden_baglanma_DENENIYOR() -> None:
    """`_capture_loop`'un `queue is None` dalı `_init_depthai()`'a gitmeli.

    Regresyon: biri bu çağrıyı silip eski "1s uyu, hiçbir şey yapma" hâline
    dönerse kamera bir daha asla geri gelmez — testin adı bunu söylüyor.
    """
    dal = re.search(
        r"if self\.queue is None:(.*?)\n    (?:def |\Z)", _SRC, re.S,
    )
    assert dal is not None, "_capture_loop'ta 'if self.queue is None:' dalı bulunamadı"
    assert "self._init_depthai()" in dal.group(1), (
        "boş kuyruk dalı artık yeniden bağlanmayı DENEMİYOR — kamera kalıcı "
        "kör kalır"
    )


def test_cihaz_kaybi_queue_i_sifirlar() -> None:
    """Kare yakalama hatasında `self.queue = None` yapılmalı — aksi halde

    aynı bozuk queue nesnesi sonsuza dek yeniden denenir, yeniden bağlanma
    dalı hiç tetiklenmez.
    """
    except_bloku = re.search(
        r"except Exception as e:\s*\n(.*?)\Z", _SRC, re.S,
    )
    assert except_bloku is not None
    gövde = except_bloku.group(1)
    assert "self.queue = None" in gövde, (
        "capture except bloğu queue'yu sıfırlamıyor — cihaz koptuğunda "
        "yeniden bağlanma hiç tetiklenmez"
    )


# --------------------------------------------------------------------------- #
# 2) Davranış — _init_depthai() gerçekten kurtarıyor mu
# --------------------------------------------------------------------------- #


class _FakeCamera:
    def setResolution(self, *a, **k) -> None: ...
    def setPreviewSize(self, *a, **k) -> None: ...
    def setInterleaved(self, *a, **k) -> None: ...
    def setFps(self, *a, **k) -> None: ...
    def setBoardSocket(self, *a, **k) -> None: ...

    class _Preview:
        def link(self, *a, **k) -> None: ...

    preview = _Preview()


class _FakeXout:
    def setStreamName(self, *a, **k) -> None: ...
    input = object()


class _FakePipeline:
    def createColorCamera(self) -> _FakeCamera:
        return _FakeCamera()

    def createXLinkOut(self) -> _FakeXout:
        return _FakeXout()


class _FakeQueue:
    def tryGet(self) -> Optional[object]:
        return None


class _FakeDevice:
    def __init__(self, pipeline) -> None:  # noqa: ANN001
        pass

    def getOutputQueue(self, **k):          # noqa: ANN201
        return _FakeQueue()

    def close(self) -> None: ...


def _install_fake_depthai(*, cihaz_hazir: bool) -> None:
    """`import depthai as dai` çağrısını sahte modülle karşılar.

    cihaz_hazir=False → `dai.Device(...)` RuntimeError fırlatır (USB henüz
    enumerate olmadı senaryosu); True → gerçek bağlantı gibi başarılı olur.
    """
    fake = types.ModuleType("depthai")
    fake.Pipeline = _FakePipeline

    class _ColorCameraProperties:
        class SensorResolution:
            THE_1080_P = object()

    class _CameraBoardSocket:
        CAM_A = object()

    fake.ColorCameraProperties = _ColorCameraProperties
    fake.CameraBoardSocket = _CameraBoardSocket

    if cihaz_hazir:
        fake.Device = _FakeDevice
    else:
        def _patlayan_device(pipeline):  # noqa: ANN001, ANN202
            raise RuntimeError("X_LINK_DEVICE_NOT_FOUND (simüle)")
        fake.Device = _patlayan_device

    sys.modules["depthai"] = fake


@pytest.fixture(scope="module")
def ros_context():                                       # noqa: ANN201
    rclpy.init()
    yield
    rclpy.shutdown()


@pytest.fixture
def temiz_depthai_modulu():                              # noqa: ANN201
    onceki = sys.modules.pop("depthai", None)
    yield
    if onceki is not None:
        sys.modules["depthai"] = onceki
    else:
        sys.modules.pop("depthai", None)


def test_ilk_baglanti_basarisiz_sonra_yeniden_deneme_kurtarir(
    ros_context, temiz_depthai_modulu,                     # noqa: ANN001
) -> None:
    """Soğuk açılışta USB henüz hazır değil → queue None; cihaz sonradan

    enumerate olunca `_init_depthai()`'ın TEKRAR çağrılması queue'yu kurtarmalı
    (bu, `_capture_loop`'un arka planda yaptığı şeyin doğrudan doğrulaması).
    """
    _install_fake_depthai(cihaz_hazir=False)
    node = oakd.OakdDriverNode()
    try:
        assert node.queue is None, "ilk bağlantı 'başarısız' simülasyonunda queue None olmalıydı"

        _install_fake_depthai(cihaz_hazir=True)
        node._init_depthai()

        assert node.queue is not None, (
            "_init_depthai() yeniden çağrılınca kurtarmadı — "
            "_capture_loop'taki yeniden bağlanma dalı da işe yaramaz"
        )
    finally:
        node.destroy_node()
