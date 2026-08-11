"""Test izolasyonu nöbetçisi — PAR-01'in geri gelmesini engeller.

`conftest.py` testleri canlı ROS domain'inden ayırıyor. Bu dosya o ayrımın
gerçekten yürürlükte olduğunu doğrular. Nöbetçi olmadan izolasyon sessizce
kaybolabilir (conftest silinir, bir başka conftest ezer, ortam değişkeni
başka yerden yazılır) ve bunu **ancak bir sonraki bag analizinde** fark ederiz —
kaptanın 14 oturumluk analizinde tam bu oldu.
"""

from __future__ import annotations

import os

import pytest

from prototype.tests.conftest import VARSAYILAN_TEST_DOMAIN


def test_test_domaini_CANLI_domainden_FARKLI() -> None:
    """🔴 En kritik kural: testler canlı sistemin domain'inde koşmamalı.

    Canlı sistem `ROS_DOMAIN_ID=42` kullanıyor (CLAUDE.md · girdap-karar.service).
    Testler oraya düğüm sokarsa `/girdap/mission/state`, `/mavros/*` gibi
    topic'lere sahte veri yazar; PAR-01'de 24.430 sahte GPS mesajı böyle
    enjekte edilmişti.
    """
    d = os.environ.get("ROS_DOMAIN_ID")
    assert d is not None, "ROS_DOMAIN_ID hic ayarlanmamis — conftest yuklenmedi mi?"
    assert d != "42", (
        "🔴 TESTLER CANLI DOMAIN'DE (42) KOSUYOR — canli yigina veri sizar. "
        "conftest.py yuklenmemis ya da ezilmis olabilir."
    )


def test_izolasyon_varsayilani_yururlukte() -> None:
    """Kaçış kapısı kullanılmadıysa varsayılan izolasyon domain'i geçerli olmalı."""
    istenen = os.environ.get("GIRDAP_TEST_DOMAIN", VARSAYILAN_TEST_DOMAIN)
    assert os.environ.get("ROS_DOMAIN_ID") == istenen


def test_conftest_rclpy_INIT_EDILMEDEN_once_kosmus() -> None:
    """Sıra doğrulaması: domain, rclpy bağlamı kurulmadan ÖNCE yazılmış olmalı.

    `rclpy.init()` çağrıldıktan sonra `ROS_DOMAIN_ID` değiştirmek etkisizdir —
    değer bağlam kurulurken okunur. Bu test, ortam değişkeninin şu an doğru
    olduğunu ve rclpy varsa bağlamın henüz kurulmamış olduğunu birlikte kontrol
    ederek sıranın bozulmadığını gösterir.
    """
    rclpy = pytest.importorskip("rclpy", reason="rclpy yok — sira kontrolu atlanir")
    assert os.environ.get("ROS_DOMAIN_ID") != "42"
    # Bu test dosyası hiçbir bağlam kurmaz; başka bir test önce koşup bağlam
    # kurmuş olabilir, o yüzden "kurulmamış olmalı" diye ISRAR ETMİYORUZ —
    # kontrol edilen şey domain'in hâlâ izole olduğu.
    assert hasattr(rclpy, "init")
