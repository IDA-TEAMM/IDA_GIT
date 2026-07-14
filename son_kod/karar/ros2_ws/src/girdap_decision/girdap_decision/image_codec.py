"""
Girdap İDA — sensor_msgs/Image ↔ numpy BGR dönüştürücü (cv_bridge'siz).

Neden cv_bridge değil: apt cv_bridge'in derlenmiş boost modülü numpy 1.x
ABI'siyle bağlı; pip numpy 2.x kurulu sistemde (gtsam/scipy zinciri) import
anında `_ARRAY_API not found` ile sakatlanıyor ve cv2_to_imgmsg KeyError
veriyor. Karar yığını yalnız 8-bit 3-kanal frame kullanır — dönüşüm birkaç
satır, kırılgan bağımlılık gereksiz. Aynı tuzak Jetson'da da geçerli.
"""

from __future__ import annotations

from typing import Optional

import numpy as np

from sensor_msgs.msg import Image
from std_msgs.msg import Header


def imgmsg_to_bgr(msg: Image) -> np.ndarray:
    """bgr8/rgb8 Image → HxWx3 uint8 BGR (contiguous kopya)."""
    if msg.encoding not in ("bgr8", "rgb8"):
        raise ValueError(f"desteklenmeyen encoding: {msg.encoding!r}")
    buf = np.frombuffer(msg.data, dtype=np.uint8)
    # step ≥ width*3 olabilir (satır dolgusu) — önce satırlara, sonra kırp.
    frame = buf.reshape(msg.height, msg.step)[:, : msg.width * 3]
    frame = frame.reshape(msg.height, msg.width, 3)
    if msg.encoding == "rgb8":
        frame = frame[:, :, ::-1]                 # RGB → BGR
    return np.ascontiguousarray(frame)            # cv2 contiguous ister


def bgr_to_imgmsg(frame: np.ndarray, header: Optional[Header] = None) -> Image:
    """HxWx3 uint8 BGR → bgr8 Image."""
    if frame.ndim != 3 or frame.shape[2] != 3 or frame.dtype != np.uint8:
        raise ValueError(f"HxWx3 uint8 bekleniyor, gelen: {frame.shape} {frame.dtype}")
    msg = Image()
    if header is not None:
        msg.header = header
    msg.height, msg.width = int(frame.shape[0]), int(frame.shape[1])
    msg.encoding = "bgr8"
    msg.is_bigendian = 0
    msg.step = msg.width * 3
    msg.data = np.ascontiguousarray(frame).tobytes()
    return msg
