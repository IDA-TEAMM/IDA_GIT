"""
Girdap İDA — GPS fix kalitesi → sigma eşlemesi testleri.

Çekirdek ROS-bağımsız (düz int tablo) → rclpy VE gtsam olmadan koşar.

Çalıştır: pytest prototype/tests/test_gps_quality.py -v
"""

from __future__ import annotations

import pytest

from prototype.fusion.gps_quality import (
    DEFAULT_SIGMA_BY_STATUS,
    STATUS_FIX,
    STATUS_GBAS_FIX,
    STATUS_NO_FIX,
    STATUS_SBAS_FIX,
    sigma_for_status,
    status_name,
)


# ------------------------------------------------------------------ sabitler

def test_sabitler_navsatstatus_mesaj_tanimiyla_ayni() -> None:
    """sensor_msgs/NavSatStatus değerleri (çekirdek rclpy'siz koşsun diye
    yeniden tanımlandı — mesaj tanımından SAPMAMALI)."""
    assert (STATUS_NO_FIX, STATUS_FIX, STATUS_SBAS_FIX, STATUS_GBAS_FIX) == (
        -1, 0, 1, 2
    )


# ------------------------------------------------------- varsayılan eşleme

@pytest.mark.parametrize(
    "status, beklenen",
    [
        (STATUS_GBAS_FIX, 0.05),   # RTK fixed
        (STATUS_SBAS_FIX, 0.50),
        (STATUS_FIX, 2.50),        # tek nokta
    ],
)
def test_fix_kalitesi_sigmaya_cevrilir(status: int, beklenen: float) -> None:
    assert sigma_for_status(status) == pytest.approx(beklenen)


def test_no_fix_reddedilir() -> None:
    """STATUS_NO_FIX → None: çağıran taraf add_gps'i ÇAĞIRMAMALI.

    Fix yokken NavSatFix'in lat/lon alanı tanımsızdır (0/0 ya da son geçerli
    değer); prior olarak eklenirse grafiği kalıcı olarak bozar.
    """
    assert sigma_for_status(STATUS_NO_FIX) is None


def test_no_fixten_kucuk_statuler_de_reddedilir() -> None:
    """Bazı sürücüler -2/-3 gibi satıcıya özgü hata kodları basar."""
    assert sigma_for_status(-2) is None
    assert sigma_for_status(-99) is None


def test_rtk_tek_noktadan_en_az_bir_buyukluk_daha_hassas() -> None:
    """Tablonun ANLAMI: fix kaliteleri arasında ciddi ağırlık farkı olmalı;
    hepsi birbirine yakınsa status okumanın bir faydası kalmaz."""
    rtk = DEFAULT_SIGMA_BY_STATUS[STATUS_GBAS_FIX]
    tek = DEFAULT_SIGMA_BY_STATUS[STATUS_FIX]
    assert tek / rtk >= 10.0


# ----------------------------------------------------------- tablo override

def test_ozel_tablo_varsayilani_ezer() -> None:
    """hardware.yaml fusion.gps_sigma_by_status → çağrıya tablo olarak geçer."""
    tablo = {STATUS_GBAS_FIX: 0.02, STATUS_SBAS_FIX: 0.8, STATUS_FIX: 5.0}
    assert sigma_for_status(STATUS_GBAS_FIX, tablo) == pytest.approx(0.02)
    assert sigma_for_status(STATUS_FIX, tablo) == pytest.approx(5.0)
    # override edilmiş tablo NO_FIX kuralını değiştirmez
    assert sigma_for_status(STATUS_NO_FIX, tablo) is None


def test_bilinmeyen_status_en_kotumser_sigmayla_kabul_edilir() -> None:
    """Tabloda olmayan pozitif status (yeni fix tipi) sessizce RTK sanılmamalı."""
    assert sigma_for_status(9) == pytest.approx(max(DEFAULT_SIGMA_BY_STATUS.values()))


def test_bos_tablo_olcumu_reddeder() -> None:
    """Yanlış konfigürasyonda (boş blok) sessizce varsayılana düşme —
    ölçümü reddet: kalibrasyonsuz güven, drift'ten kötüdür."""
    assert sigma_for_status(STATUS_GBAS_FIX, {}) is None


def test_pozitif_olmayan_sigma_reddedilir() -> None:
    """σ=0 sonsuz güven demek — yaml'da yazım hatası grafiği kilitlemesin."""
    assert sigma_for_status(STATUS_GBAS_FIX, {STATUS_GBAS_FIX: 0.0}) is None


# ------------------------------------------------------------------ logging

def test_status_name_okunabilir() -> None:
    assert "RTK" in status_name(STATUS_GBAS_FIX)
    assert status_name(STATUS_NO_FIX) == "NO_FIX"
    assert "7" in status_name(7)          # bilinmeyen değeri de göstersin


# ------------------------------------------------------- video modu güvencesi

def test_cekirdek_gtsam_yuklemez() -> None:
    """CLAUDE.md güvencesi: use_isam2=false → fusion_node GTSAM'ı HİÇ yüklemez.

    fusion_node bu modülü MODÜL DÜZEYİNDE import ediyor (kalite kapısı her iki
    modda da gerekli). Buraya bir gün gtsam sızarsa video modu sessizce GTSAM'a
    bağımlı hale gelir — video günü kurulu olmayan bir makinede node hiç
    açılmaz. Alt süreçte izole ölç: aynı oturumdaki başka testler gtsam'ı
    zaten yüklemiş olabilir.
    """
    import subprocess
    import sys

    kod = (
        "import sys; import prototype.fusion.gps_quality as m; "
        "print('gtsam' in sys.modules)"
    )
    out = subprocess.run(
        [sys.executable, "-c", kod], capture_output=True, text=True, check=True
    )
    assert out.stdout.strip() == "False", (
        "gps_quality dolaylı olarak gtsam yüklüyor — video modu bozulur"
    )
