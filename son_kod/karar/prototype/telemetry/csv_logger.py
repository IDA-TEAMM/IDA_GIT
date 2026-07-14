"""
Girdap İDA — Telemetri CSV Logger (ROS-bağımsız çekirdek)

Şartname 4.2 — Dosya 2:
    Telemetri CSV, ≥1 Hz, header satırlı. Görev bitiminden 20 dk içinde
    teslim; her gecikmiş dosya 5 ceza puanı. Bu yüzden yazım güç kesintisine
    dayanıklı olmalı → her satırda fsync.

Layer 2 telemetry_node ve Layer 0 testleri ortak bu modülü kullanır. rclpy
bağımsız olduğundan CSV formatı/fsync davranışı pytest ile doğrulanabilir.

Sütun sözleşmesi (CSV_HEADER) değiştirilmemeli — post-process script'leri ve
yarışma değerlendirmesi bu sıraya bağlı.
"""

from __future__ import annotations

import csv
import math
import os
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import List, Optional, TextIO, Tuple, Union


# Şartname 4.2 Dosya-2 başlık sırası — DEĞİŞTİRME.
CSV_HEADER: List[str] = [
    "zaman",
    "lat",
    "lon",
    "hiz",
    "roll",
    "pitch",
    "heading",
    "hiz_setpoint",
    "yon_setpoint",
    "mission_state",
]

# Video grafik CSV'si (T0-g) — Şartname md 3.3.1.1 Ekran-2 üç sinyali:
# (a) gerçek hız + hız setpoint, (b) heading + yaw setpoint,
# (c) thrusterlardan kuvvet isteği. Dosya-2'ye GİRMEZ (md 4.2 istemiyor).
GRAPH_CSV_HEADER: List[str] = [
    "zaman",
    "hiz",
    "hiz_setpoint",
    "heading",
    "yon_setpoint",
    "thrust_sol",
    "thrust_sag",
]

# Her sayısal sütun için sabit ondalık — post-process kolaylığı.
_DECIMALS = {
    "lat": 7,
    "lon": 7,
    "hiz": 3,
    "roll": 4,
    "pitch": 4,
    "heading": 4,
    "hiz_setpoint": 3,
    "yon_setpoint": 3,
    "thrust": 2,
}


def pwm_to_thrust_pct(
    pwm: int, neutral: int = 1500, pwm_range: int = 500
) -> Optional[float]:
    """ESC PWM (µs) → normalize itki isteği yüzdesi [-100, +100].

    AUTO video modunda (telemetry `setpoint_source: fc`) Ekran-2c thrust'ı
    MPPI'den DEĞİL, FC'nin gerçek çıkışından (/mavros/rc/out) türetilir —
    AUTO'da aracı FC sürer, MPPI thrust'ı sahte veri olur (md 3.3.1.1
    "grafikler araç hareketiyle senkron" şartı). Eşleme: neutral=0%,
    neutral±pwm_range=±100%, dışı kırpılır. pwm<=0 = kanal kapalı/yok →
    None (CSV'de boş kalır — sahte sıfır basılmaz, run_ekran2 NaN kuralı).
    """
    if pwm <= 0:
        return None
    pct = (float(pwm) - float(neutral)) / float(pwm_range) * 100.0
    return max(-100.0, min(100.0, pct))


def quat_to_rpy(
    qx: float, qy: float, qz: float, qw: float
) -> Tuple[float, float, float]:
    """Quaternion → (roll, pitch, yaw) ENU, ZYX sıralaması (rad)."""
    # roll
    sinr = 2.0 * (qw * qx + qy * qz)
    cosr = 1.0 - 2.0 * (qx * qx + qy * qy)
    roll = math.atan2(sinr, cosr)
    # pitch (gimbal clamp)
    sinp = 2.0 * (qw * qy - qz * qx)
    sinp = max(-1.0, min(1.0, sinp))
    pitch = math.asin(sinp)
    # yaw
    siny = 2.0 * (qw * qz + qx * qy)
    cosy = 1.0 - 2.0 * (qy * qy + qz * qz)
    yaw = math.atan2(siny, cosy)
    return roll, pitch, yaw


def _fmt(value: Optional[float], decimals: int) -> str:
    """None → boş string; float → sabit ondalık."""
    if value is None:
        return ""
    return f"{value:.{decimals}f}"


def utc_timestamp() -> str:
    """Dosya adı için sıralanabilir UTC damgası (telemetri_<...>.csv)."""
    return datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")


def utc_isoformat() -> str:
    """Satır 'zaman' sütunu için ISO-8601 UTC, milisaniye çözünürlük."""
    return datetime.now(timezone.utc).isoformat(timespec="milliseconds")


@dataclass
class TelemetrySample:
    """
    Tek telemetri satırının veri alanları. Eksik alanlar None → CSV'de "".
    Değerler SI: hiz m/s, roll/pitch/heading rad, setpoint'ler ilgili birim.
    """

    lat: Optional[float] = None
    lon: Optional[float] = None
    hiz: Optional[float] = None
    roll: Optional[float] = None
    pitch: Optional[float] = None
    heading: Optional[float] = None
    hiz_setpoint: Optional[float] = None
    yon_setpoint: Optional[float] = None
    mission_state: str = ""

    def to_row(self, zaman: str) -> List[str]:
        """CSV_HEADER sırasında formatlanmış satır üret."""
        return [
            zaman,
            _fmt(self.lat, _DECIMALS["lat"]),
            _fmt(self.lon, _DECIMALS["lon"]),
            _fmt(self.hiz, _DECIMALS["hiz"]),
            _fmt(self.roll, _DECIMALS["roll"]),
            _fmt(self.pitch, _DECIMALS["pitch"]),
            _fmt(self.heading, _DECIMALS["heading"]),
            _fmt(self.hiz_setpoint, _DECIMALS["hiz_setpoint"]),
            _fmt(self.yon_setpoint, _DECIMALS["yon_setpoint"]),
            self.mission_state,
        ]


@dataclass
class GraphSample:
    """
    Video grafik CSV'sinin (GRAPH_CSV_HEADER) tek satırı — Ekran-2 sinyalleri.
    Birimler: hiz m/s, heading/yon rad, thrust diferansiyel sol/sağ —
    kaynak "girdap" (GUIDED/MPPI) ise Newton, "fc" (AUTO, /mavros/rc/out)
    ise normalize % [-100, +100] (pwm_to_thrust_pct).
    """

    hiz: Optional[float] = None
    hiz_setpoint: Optional[float] = None
    heading: Optional[float] = None
    yon_setpoint: Optional[float] = None
    thrust_sol: Optional[float] = None
    thrust_sag: Optional[float] = None

    def to_row(self, zaman: str) -> List[str]:
        """GRAPH_CSV_HEADER sırasında formatlanmış satır üret."""
        return [
            zaman,
            _fmt(self.hiz, _DECIMALS["hiz"]),
            _fmt(self.hiz_setpoint, _DECIMALS["hiz_setpoint"]),
            _fmt(self.heading, _DECIMALS["heading"]),
            _fmt(self.yon_setpoint, _DECIMALS["yon_setpoint"]),
            _fmt(self.thrust_sol, _DECIMALS["thrust"]),
            _fmt(self.thrust_sag, _DECIMALS["thrust"]),
        ]


class TelemetryCsvLogger:
    """
    Header'lı CSV yazıcı — her satırda flush + os.fsync (güç kesintisi güvenli).

    Tipik kullanım:
        logger = TelemetryCsvLogger("data/telemetry")
        logger.write_sample(utc_isoformat(), TelemetrySample(lat=..., ...))
        ...
        logger.close()
    """

    def __init__(
        self,
        output_dir: Union[str, Path],
        filename: Optional[str] = None,
        header: Optional[List[str]] = None,
    ) -> None:
        out = Path(output_dir).expanduser()
        out.mkdir(parents=True, exist_ok=True)
        if filename is None:
            filename = f"telemetri_{utc_timestamp()}.csv"
        self.path = out / filename

        self._fp: TextIO = open(
            self.path, "w", newline="", encoding="utf-8"
        )
        self._writer = csv.writer(self._fp)
        # header verilmezse Dosya-2 sözleşmesi (CSV_HEADER) — eski davranış.
        self._writer.writerow(CSV_HEADER if header is None else header)
        self._sync()
        self._row_count = 0

    def write_sample(
        self, zaman: str, sample: Union[TelemetrySample, "GraphSample"]
    ) -> bool:
        """Bir telemetri satırı yaz ve diske senkronize et.

        F-S.5: disk-dolu (OSError) timer callback'ine SIZMAZ — node ölürse
        Dosya-2 üretimi tamamen durur (bu proje gerçek disk-dolu krizi yaşadı).
        Dönüş: True = yazıldı, False = disk hatası (örnek atlandı, logger
        yaşamaya devam eder; disk açılınca kayıt kaldığı yerden sürer).
        """
        try:
            self._writer.writerow(sample.to_row(zaman))
            self._sync()
        except OSError:
            return False
        self._row_count += 1
        return True

    def _sync(self) -> None:
        """flush + fsync: buffer'ı OS'a, OS'u fiziksel diske indir."""
        self._fp.flush()
        os.fsync(self._fp.fileno())

    @property
    def row_count(self) -> int:
        """Yazılmış veri satırı sayısı (header hariç)."""
        return self._row_count

    def close(self) -> None:
        if not self._fp.closed:
            self._fp.flush()
            os.fsync(self._fp.fileno())
            self._fp.close()
