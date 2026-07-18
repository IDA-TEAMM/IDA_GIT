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


def test_next_kayit_num_bosluk_doldurur(tmp_path) -> None:
    """GÖREV 1 çekirdeği (rev. Eyüp 16.07): kayıt numarası dizindeki EN KÜÇÜK
    boş pozitif — silinen numara yeniden kullanılır ('log 2 silinirse gene
    log 2'den devam etsin'); sayı-dışı adlar yok sayılır."""
    from prototype.telemetry.csv_logger import next_kayit_num

    assert next_kayit_num(tmp_path / "yok") == 1         # kök henüz yok
    assert next_kayit_num(tmp_path) == 1                 # boş kök
    (tmp_path / "1").mkdir()
    (tmp_path / "3").mkdir()
    assert next_kayit_num(tmp_path) == 2                 # boşluk doldurulur
    (tmp_path / "2").mkdir()
    assert next_kayit_num(tmp_path) == 4
    (tmp_path / "abc").mkdir()                           # sayı-dışı yok sayılır
    (tmp_path / "not5.csv").write_text("x")              # dosya yok sayılır
    assert next_kayit_num(tmp_path) == 4


def test_prune_old_kayit_dirs(tmp_path) -> None:
    """'Eski logları da silsin' çekirdeği: en yeni `keep` kayıt kalır (dosya
    zamanına göre), keep_dirs asla silinmez, keep<=0 = kapalı."""
    import os

    from prototype.telemetry.csv_logger import prune_old_kayit_dirs

    for i, n in enumerate(["1", "2", "3"]):
        d = tmp_path / n
        d.mkdir()
        (d / "telemetri.csv").write_text("x")
        os.utime(d, ns=((i + 1) * 10**9, (i + 1) * 10**9))

    assert prune_old_kayit_dirs(tmp_path, keep=0) == []  # kapalı
    assert prune_old_kayit_dirs(tmp_path, keep=5) == []  # sığıyor

    # 1 en eski ama korunuyor → sıradaki en eski (2) silinir
    deleted = prune_old_kayit_dirs(tmp_path, keep=2, keep_dirs=(tmp_path / "1",))
    assert [p.name for p in deleted] == ["2"]
    assert (tmp_path / "1").exists() and (tmp_path / "3").exists()

    deleted = prune_old_kayit_dirs(tmp_path, keep=1)
    assert [p.name for p in deleted] == ["1"]            # mtime en eski
    assert (tmp_path / "3").exists()
