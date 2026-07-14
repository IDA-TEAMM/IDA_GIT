"""
Girdap İDA — TelemetryCsvLogger testi (Şartname 4.2 Dosya-2).

10 saniyelik mock sensör akışını 2 Hz'te CSV'ye yazar ve doğrular:
    - Dosya adı deseni (telemetri_<UTC>.csv)
    - Satır sayısı (header + 20 veri satırı)
    - Başlık sütun sözleşmesi (CSV_HEADER birebir)
    - Her satır 10 sütun, eksik alan "" (boş)
    - Sayısal format (sabit ondalık)
    - fsync sonrası dosya akış ortasında okunabilir (durability)

Çalıştır: pytest prototype/tests/test_telemetry_logger.py -v
"""

from __future__ import annotations

import csv
import math
import re

import pytest

from prototype.telemetry.csv_logger import (
    CSV_HEADER,
    GRAPH_CSV_HEADER,
    GraphSample,
    TelemetryCsvLogger,
    TelemetrySample,
    pwm_to_thrust_pct,
    quat_to_rpy,
)


# --------------------------------------------------------------------------- #
# Mock sensör akışı
# --------------------------------------------------------------------------- #


def _mock_stream(n: int) -> list[TelemetrySample]:
    """
    n adet telemetri örneği — düz seyir + orta noktada dönüş + durum değişimi.
    Marmaris civarı sabit origin etrafında küçük hareket.
    """
    samples = []
    lat0, lon0 = 36.85, 28.27
    for k in range(n):
        heading = 0.0 if k < n // 2 else math.pi / 2
        state = "PARKUR1" if k < n // 2 else "PARKUR2"
        samples.append(
            TelemetrySample(
                lat=lat0 + k * 1e-5,
                lon=lon0 + k * 1e-5,
                hiz=1.0 + 0.1 * k,
                roll=0.01 * math.sin(k),
                pitch=0.01 * math.cos(k),
                heading=heading,
                hiz_setpoint=1.5,
                yon_setpoint=heading,
                mission_state=state,
            )
        )
    return samples


# --------------------------------------------------------------------------- #
# Testler
# --------------------------------------------------------------------------- #


def test_10s_stream_row_count(tmp_path) -> None:
    """10 s @ 2 Hz → 20 veri satırı + 1 header = 21 satır."""
    rate_hz = 2.0
    duration_s = 10.0
    n = int(rate_hz * duration_s)               # 20

    logger = TelemetryCsvLogger(tmp_path)
    for s in _mock_stream(n):
        logger.write_sample("2026-07-03T12:00:00.000+00:00", s)
    logger.close()

    assert logger.row_count == n

    with open(logger.path, newline="", encoding="utf-8") as f:
        rows = list(csv.reader(f))
    assert len(rows) == n + 1, "header + 20 satır bekleniyordu"


def test_filename_pattern(tmp_path) -> None:
    """Dosya adı telemetri_<UTC>.csv desenine uymalı."""
    logger = TelemetryCsvLogger(tmp_path)
    logger.close()
    assert re.fullmatch(
        r"telemetri_\d{8}T\d{6}Z\.csv", logger.path.name
    ), f"beklenmeyen dosya adı: {logger.path.name}"


def test_header_matches_contract(tmp_path) -> None:
    """Başlık satırı Şartname 4.2 sütun sözleşmesiyle birebir aynı olmalı."""
    logger = TelemetryCsvLogger(tmp_path)
    logger.write_sample("t0", TelemetrySample(lat=36.0, lon=28.0))
    logger.close()

    with open(logger.path, newline="", encoding="utf-8") as f:
        header = next(csv.reader(f))
    assert header == CSV_HEADER
    assert header[0] == "zaman"
    assert "mission_state" in header
    assert len(header) == 10


def test_column_format_and_missing_fields(tmp_path) -> None:
    """Her satır 10 sütun; None alanlar boş; sayısal format sabit ondalık."""
    logger = TelemetryCsvLogger(tmp_path)
    # Sadece lat/lon dolu, geri kalan None
    logger.write_sample(
        "2026-07-03T12:00:00.000+00:00",
        TelemetrySample(lat=36.8512345, lon=28.2698765),
    )
    logger.close()

    with open(logger.path, newline="", encoding="utf-8") as f:
        rows = list(csv.reader(f))
    data = rows[1]
    assert len(data) == 10, "her satır 10 sütun olmalı"
    # lat/lon 7 ondalık
    assert data[1] == "36.8512345"
    assert data[2] == "28.2698765"
    # hiz, roll, pitch, heading, setpoint'ler None → boş
    for i in range(3, 9):
        assert data[i] == "", f"sütun {i} boş olmalıydı"
    # mission_state boş string
    assert data[9] == ""


def test_numeric_precision(tmp_path) -> None:
    """Sayısal sütunlar sözleşmedeki ondalık hassasiyetiyle yazılmalı."""
    logger = TelemetryCsvLogger(tmp_path)
    logger.write_sample(
        "t",
        TelemetrySample(
            hiz=1.23456, roll=0.123456, pitch=-0.98765,
            heading=1.5707963, hiz_setpoint=2.0, yon_setpoint=-0.5,
        ),
    )
    logger.close()

    with open(logger.path, newline="", encoding="utf-8") as f:
        data = list(csv.reader(f))[1]
    assert data[3] == "1.235"          # hiz, 3 ondalık
    assert data[4] == "0.1235"         # roll, 4 ondalık
    assert data[5] == "-0.9877"        # pitch, 4 ondalık (yuvarlama)
    assert data[6] == "1.5708"         # heading, 4 ondalık
    assert data[7] == "2.000"          # hiz_setpoint, 3 ondalık
    assert data[8] == "-0.500"         # yon_setpoint, 3 ondalık


def test_fsync_durability_mid_stream(tmp_path) -> None:
    """
    fsync her satırda çalıştığından, logger kapanmadan bile yazılmış satırlar
    diskten okunabilmeli (güç kesintisi senaryosu).
    """
    logger = TelemetryCsvLogger(tmp_path)
    for s in _mock_stream(5):
        logger.write_sample("t", s)
    # KAPATMADAN oku — fsync sayesinde 5 satır + header diskte olmalı
    with open(logger.path, newline="", encoding="utf-8") as f:
        rows = list(csv.reader(f))
    assert len(rows) == 6, "fsync'lenmiş satırlar kapatılmadan görülmeli"
    logger.close()


# --------------------------------------------------------------------------- #
# Grafik CSV (T0-g — Şartname md 3.3.1.1 Ekran-2)
# --------------------------------------------------------------------------- #


def test_dosya2_header_frozen() -> None:
    """
    Dosya-2 sözleşmesi DONMUŞ — md 4.2 thrust İSTEMİYOR, Ekran-2 sinyalleri
    ayrı grafik CSV'sine gider. Bu test CSV_HEADER'a yanlışlıkla sütun
    eklenmesini yakalar.
    """
    assert CSV_HEADER == [
        "zaman", "lat", "lon", "hiz", "roll", "pitch", "heading",
        "hiz_setpoint", "yon_setpoint", "mission_state",
    ]


def test_graph_header_contract() -> None:
    """md 3.3.1.1 Ekran-2 üç sinyali: hız+sp, heading+sp, thrust isteği."""
    assert GRAPH_CSV_HEADER == [
        "zaman", "hiz", "hiz_setpoint", "heading", "yon_setpoint",
        "thrust_sol", "thrust_sag",
    ]


def test_graph_csv_write_and_format(tmp_path) -> None:
    """Grafik CSV: özel header + GraphSample satır formatı (thrust 2 ondalık)."""
    logger = TelemetryCsvLogger(
        tmp_path, filename="grafik_test.csv", header=GRAPH_CSV_HEADER
    )
    logger.write_sample(
        "2026-07-10T12:00:00.000+00:00",
        GraphSample(
            hiz=1.23456, hiz_setpoint=1.5,
            heading=0.123456, yon_setpoint=-0.5,
            thrust_sol=12.3456, thrust_sag=-7.0,
        ),
    )
    logger.close()

    with open(logger.path, newline="", encoding="utf-8") as f:
        rows = list(csv.reader(f))
    assert rows[0] == GRAPH_CSV_HEADER
    data = rows[1]
    assert len(data) == 7
    assert data[1] == "1.235"          # hiz, 3 ondalık (Dosya-2 ile tutarlı)
    assert data[2] == "1.500"
    assert data[3] == "0.1235"         # heading, 4 ondalık
    assert data[4] == "-0.500"
    assert data[5] == "12.35"          # thrust (N), 2 ondalık
    assert data[6] == "-7.00"


def test_graph_sample_missing_fields(tmp_path) -> None:
    """Henüz mesaj gelmemiş alanlar boş string olmalı (7 sütun korunur)."""
    logger = TelemetryCsvLogger(
        tmp_path, filename="grafik_bos.csv", header=GRAPH_CSV_HEADER
    )
    logger.write_sample("t", GraphSample())
    logger.close()

    with open(logger.path, newline="", encoding="utf-8") as f:
        data = list(csv.reader(f))[1]
    assert len(data) == 7
    assert data[0] == "t"
    assert all(v == "" for v in data[1:])


def test_default_header_unchanged_for_dosya2(tmp_path) -> None:
    """header parametresi verilmezse eski davranış birebir korunmalı."""
    logger = TelemetryCsvLogger(tmp_path)
    logger.close()
    with open(logger.path, newline="", encoding="utf-8") as f:
        header = next(csv.reader(f))
    assert header == CSV_HEADER


def test_quat_to_rpy_known_values() -> None:
    """quat_to_rpy referans değerlerde doğru RPY üretmeli."""
    # Kimlik quaternion → sıfır RPY
    r, p, y = quat_to_rpy(0.0, 0.0, 0.0, 1.0)
    assert abs(r) < 1e-9 and abs(p) < 1e-9 and abs(y) < 1e-9
    # +90° yaw (z ekseni): qz=sin(45°), qw=cos(45°)
    s = math.sqrt(0.5)
    r, p, y = quat_to_rpy(0.0, 0.0, s, s)
    assert abs(y - math.pi / 2) < 1e-6
    assert abs(r) < 1e-6 and abs(p) < 1e-6


# --------------------------------------------------------------------------- #
# pwm_to_thrust_pct — AUTO video modu (fc kaynağı) Ekran-2c eşlemesi
# --------------------------------------------------------------------------- #


def test_pwm_to_thrust_pct_known_values() -> None:
    """PWM → % eşlemesi referans noktalarda doğru olmalı (1500=0, ±500=±100)."""
    assert pwm_to_thrust_pct(1500) == 0.0
    assert pwm_to_thrust_pct(2000) == 100.0
    assert pwm_to_thrust_pct(1000) == -100.0
    assert pwm_to_thrust_pct(1750) == 50.0
    assert pwm_to_thrust_pct(1250) == -50.0


def test_pwm_to_thrust_pct_clamp_and_missing_channel() -> None:
    """Aralık dışı kırpılır; pwm<=0 (kanal kapalı/yok) None → CSV'de boş.

    Sahte-sıfır YASAK: çıkış kapalıyken 0% (nötr itki) yazmak Ekran-2c'de
    'motor duruyor' ile 'veri yok'u ayırt edilemez kılar (run_ekran2 NaN kuralı).
    """
    assert pwm_to_thrust_pct(2600) == 100.0     # üst kırpma
    assert pwm_to_thrust_pct(400) == -100.0     # alt kırpma (>0 ama aralık dışı)
    assert pwm_to_thrust_pct(0) is None         # kanal yayında yok
    assert pwm_to_thrust_pct(-5) is None


def test_pwm_to_thrust_pct_custom_neutral_range() -> None:
    """Özel neutral/range parametreleri (FC trim'e göre) doğru ölçeklenmeli."""
    # neutral=1520 (trim kaymış), range=400 → 1920 = +100%, 1720 = +50%
    assert pwm_to_thrust_pct(1920, neutral=1520, pwm_range=400) == 100.0
    assert pwm_to_thrust_pct(1720, neutral=1520, pwm_range=400) == 50.0


# ---------------------------------------------------------------------------
# F-S.5 — disk-dolu (OSError) koruması: bu proje GERÇEK disk-dolu krizi
# yaşadı; timer callback'inden sızan OSError node'u öldürür → Dosya-2 üretimi
# tamamen durur (5 ceza + tek örnek değil TÜM kayıt kaybı).
# ---------------------------------------------------------------------------

def test_write_sample_disk_hatasinda_false_doner(tmp_path, monkeypatch) -> None:  # noqa: ANN001
    """OSError (disk dolu) write_sample'dan SIZMAMALI — False dönmeli."""
    from prototype.telemetry.csv_logger import TelemetryCsvLogger, TelemetrySample

    logger = TelemetryCsvLogger(tmp_path)
    ornek = TelemetrySample(lat=1.0, lon=2.0, hiz=0.5, roll=0.0,
                            pitch=0.0, heading=90.0)
    assert logger.write_sample("2026-07-14T12:00:00", ornek) is True

    def _disk_dolu(_fd):  # noqa: ANN001, ANN202
        raise OSError(28, "No space left on device")

    monkeypatch.setattr("prototype.telemetry.csv_logger.os.fsync", _disk_dolu)
    assert logger.write_sample("2026-07-14T12:00:01", ornek) is False, (
        "disk hatası exception olarak sızdı ya da True döndü (F-S.5)"
    )

    # Disk açılınca kayıt kaldığı yerden devam etmeli (logger ölmez).
    monkeypatch.undo()
    assert logger.write_sample("2026-07-14T12:00:02", ornek) is True
    logger.close()
