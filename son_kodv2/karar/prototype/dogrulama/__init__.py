"""Girdap İDA — çalışma zamanı doğrulama (runtime verification) çekirdeği.

ROS'suz, `cv2`'siz: yalnız sayılar üzerinde çalışır ⇒ pytest'te koşar.
"""
from prototype.dogrulama.kural import (  # noqa: F401
    Kural,
    Sonuc,
    Tur,
    bir_ara,
    degil,
    her_zaman,
    ve,
    veya,
)
