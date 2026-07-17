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
import shutil
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Iterable, List, Optional, TextIO, Tuple, Union


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


def next_kayit_num(root: Union[str, Path]) -> int:
    """Kayıt klasörü numarası: kökteki sayı-adlı klasörlerin DIŞINDAKİ en
    küçük pozitif sayı.

    GÖREV 1 (rev. Eyüp 16.07): her kayıt kendi numaralı klasöründe
    (kayit/1, kayit/2, …) ve numara kalıcı sayaç DEĞİL — 'log 2 silinirse
    gene log 2'den devam etsin'. Sayı-dışı adlar ve dosyalar yok sayılır.
    """
    root = Path(root).expanduser()
    used = set()
    if root.is_dir():
        for p in root.iterdir():
            if p.is_dir() and p.name.isdigit():
                used.add(int(p.name))
    n = 1
    while n in used:
        n += 1
    return n


def prune_old_kayit_dirs(
    root: Union[str, Path],
    keep: int,
    keep_dirs: Iterable[Union[str, Path]] = (),
) -> List[Path]:
    """'Eski logları da silsin' (Eyüp 16.07): sayı-adlı kayıt klasörlerinden
    en yeni `keep` tanesi kalır (dosya zamanına göre — numara boşluk
    doldurduğu için numaraya bakılamaz), gerisi silinir. `keep_dirs`
    (aktif kayıt) asla silinmez; `keep <= 0` → silme kapalı.
    Silinenleri (eski→yeni) döner.
    """
    if keep <= 0:
        return []
    root = Path(root).expanduser()
    if not root.is_dir():
        return []
    protected = {Path(p) for p in keep_dirs}
    dirs = [p for p in root.iterdir() if p.is_dir() and p.name.isdigit()]
    dirs.sort(key=lambda p: (p.stat().st_mtime_ns, int(p.name)))
    deleted: List[Path] = []
    fazla = len(dirs) - keep
    for p in dirs:
        if fazla <= 0:
            break
        if p in protected:
            continue
        shutil.rmtree(p, ignore_errors=True)
        deleted.append(p)
        fazla -= 1
    return deleted


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
    Birimler: hiz m/s, heading/yon rad, thrust N (diferansiyel sol/sağ).
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
    ) -> None:
        """Bir telemetri satırı yaz ve diske senkronize et."""
        self._writer.writerow(sample.to_row(zaman))
        self._sync()
        self._row_count += 1

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
