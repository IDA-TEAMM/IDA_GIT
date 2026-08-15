"""fc_referans_uret.py — pck_coz() çözümleyicisinin nöbetçisi.

🔴 15.08 gerçek donanımda ilk çalıştırıldığında `pck_coz` (o zaman elle
yazılmış bir çözücüydü) İLK kayıtta bile çöktü. Kök neden ÜÇ ayrı bit/nibble
hatasıydı — biri bile yeterince ölümcül:
  1. kayıtlar arasındaki dolgu (pad, 0x00) baytları hiç atlanmıyordu
  2. "varsayılan değer de var" biti `0x40` sanılmıştı, gerçeği `0x10`
  3. ad-uzunluğu / ortak-önek-uzunluğu nibble'ları TERS okunuyordu

Bu bir REFERANS DOSYASI üretiyor — Pixhawk'a geri yüklenecek. Yanlış ada
yanlış değer yazdırmak "çalışmıyor" değil "yanlış çalışıyor" üretir, bu
yüzden çözücü artık MAVProxy'nin kendi `param_ftp.ftp_param_decode`'una
devredildi (`pck_coz`). Bu test dosyası KENDİ elle-kodlanmış pck bayt
dizileriyle çözümü sınar — gerçek donanıma bağlı değil, CI'da koşar.
"""

from __future__ import annotations

import importlib.util
import struct
import sys
from pathlib import Path

import pytest

pytest.importorskip("MAVProxy", reason="MAVProxy yok — pck çözümü atlanır")

_yol = Path(__file__).resolve().parents[2] / "scripts" / "fc_referans_uret.py"
_spec = importlib.util.spec_from_file_location("fc_referans_uret", _yol)
fru = importlib.util.module_from_spec(_spec)
sys.modules["fc_referans_uret"] = fru
_spec.loader.exec_module(fru)


# --------------------------------------------------------------------------- #
# sentetik param.pck üreticisi — AP_Param::pack() biçiminin AYNISI
# --------------------------------------------------------------------------- #

_TIP_FLOAT = 4
_TIP_INT16 = 2


def _kayit_uret(ad: bytes, deger: float, onceki_ad: bytes,
                 varsayilan: float | None, tip: int = _TIP_FLOAT) -> bytes:
    """Tek bir param kaydını (ptype + plen + ad + değer[+varsayılan]) üret."""
    ortak = 0
    for a, b in zip(ad, onceki_ad):
        if a != b:
            break
        ortak += 1
    ortak = min(ortak, 15)
    kalan_ad = ad[ortak:]
    assert 1 <= len(kalan_ad) <= 16
    has_default = varsayilan is not None
    ptype = tip | (0x10 if has_default else 0x00)
    plen = ((len(kalan_ad) - 1) << 4) | ortak
    fmt = "f" if tip == _TIP_FLOAT else "h"
    gövde = struct.pack("<BB", ptype, plen) + kalan_ad
    if tip == _TIP_FLOAT:
        gövde += struct.pack("<f", deger)
        if has_default:
            gövde += struct.pack("<f", varsayilan)
    else:
        gövde += struct.pack("<h", int(deger))
        if has_default:
            gövde += struct.pack("<h", int(varsayilan))
    return gövde


def _pck_uret(kayitlar: list[tuple[str, float, float | None]],
              pad_araya: int = 0) -> bytes:
    """[(ad, deger, varsayilan|None), ...] → tam param.pck bayt dizisi."""
    magic = 0x671C          # withdefaults
    gövde = b""
    onceki = b""
    for ad, deger, vars_ in kayitlar:
        ad_b = ad.encode("ascii")
        gövde += _kayit_uret(ad_b, deger, onceki, vars_)
        onceki = ad_b
        gövde += b"\x00" * pad_araya
    return struct.pack("<HHH", magic, len(kayitlar), len(kayitlar)) + gövde


# --------------------------------------------------------------------------- #
# testler
# --------------------------------------------------------------------------- #


def test_TEK_kayit_varsayilanli_dogru_cozulur() -> None:
    ham = _pck_uret([("FORMAT_VERSION", 120.0, 120.0)])
    d = fru.pck_coz(ham)
    assert d == {"FORMAT_VERSION": (120.0, 120.0)}


def test_COK_kayit_ORTAK_ONEK_SIKISTIRMASI_dogru_coziliyor() -> None:
    """🔴 bug #3'ün doğrudan sınavı: 'SERVO1_MIN' → 'SERVO1_TRIM' arasında
    'SERVO1_' ortak önekini paylaşır; ortak/ad-uzunluğu nibble'ları ters
    okunsaydı ikinci adın ilk harfleri kaybolur ya da bozuk çıkardı."""
    ham = _pck_uret([
        ("SERVO1_MIN", 1000.0, 1100.0),
        ("SERVO1_TRIM", 1500.0, 1500.0),
        ("SERVO1_MAX", 2000.0, 1900.0),
    ])
    d = fru.pck_coz(ham)
    assert set(d) == {"SERVO1_MIN", "SERVO1_TRIM", "SERVO1_MAX"}
    assert d["SERVO1_MIN"] == (1000.0, 1100.0)
    assert d["SERVO1_TRIM"] == (1500.0, 1500.0)
    assert d["SERVO1_MAX"] == (2000.0, 1900.0)


def test_TEK_DEGERLI_kayit_zaten_varsayilanda_sayilir() -> None:
    """🔴 bug #2'nin doğrudan sınavı: has-default biti 0x10'da, 0x40'ta
    DEĞİL. Protokol: bit KAPALIYSA tek değer gönderilir ve bu "şu an
    varsayılanda" demektir (current==default) — `None` DEĞİL. Yanlış bitte
    okunursa (0x40) bu kayıt hep 'varsayılansız' sanılır ve sonraki kaydın
    baytlarını yanlış konumdan okumaya başlar (desenkron)."""
    ham = _pck_uret([
        ("STAT_BOOTCNT", 7.0, None),          # tek-değerli: zaten varsayılan
        ("CRUISE_SPEED", 1.05, 2.0),          # iki-değerli: default'tan sapmış
    ])
    d = fru.pck_coz(ham)
    assert d["STAT_BOOTCNT"] == (7.0, 7.0)
    assert d["CRUISE_SPEED"] == pytest.approx((1.05, 2.0))  # float32 yuvarlama


def test_DOLGU_BAYTLARI_kayitlar_arasinda_ATLANIYOR() -> None:
    """🔴 bug #1'in doğrudan sınavı: MAVFTP burst chunk'ları arasına ArduPilot
    bazen 0x00 dolgu koyar; atlanmazsa sonraki kaydın ptype'ı 0 okunur ve
    (tip=0 tanımsız olduğu için) çözüm PATLAR."""
    ham = _pck_uret([
        ("AHRS_EKF_TYPE", 3.0, 3.0),
        ("GPS_TYPE", 1.0, 1.0),
    ], pad_araya=3)
    d = fru.pck_coz(ham)
    assert d["AHRS_EKF_TYPE"] == (3.0, 3.0)
    assert d["GPS_TYPE"] == (1.0, 1.0)


def test_INT16_tip_de_dogru_cozuluyor() -> None:
    ham = _pck_uret([("RC9_OPTION", 16.0, 0.0)])
    # int16 tip için manuel üret (yardımcı varsayılan float kullanıyor)
    ham = struct.pack("<HHH", 0x671C, 1, 1) + _kayit_uret(
        b"RC9_OPTION", 16.0, b"", 0.0, tip=_TIP_INT16
    )
    d = fru.pck_coz(ham)
    assert d == {"RC9_OPTION": (16.0, 0.0)}


def test_909_KAYITLIK_GERCEKCI_HACIMDE_DOGRU_KALIYOR() -> None:
    """Gerçek FC dökümüyle aynı büyüklük mertebesi (909) — desenkron olsaydı
    ilk birkaç kayıttan sonra ortaya çıkardı."""
    kayitlar = [(f"P{i:04d}_ABCDEFG", float(i), float(i) + 0.5)
                for i in range(909)]
    ham = _pck_uret(kayitlar)
    d = fru.pck_coz(ham)
    assert len(d) == 909
    assert d["P0000_ABCDEFG"] == (0.0, 0.5)
    assert d["P0908_ABCDEFG"] == (908.0, 908.5)


def test_BOZUK_MAGIC_None_ve_hata_veriyor() -> None:
    ham = struct.pack("<HHH", 0xDEAD, 1, 1) + b"\x00" * 8
    with pytest.raises(ValueError):
        fru.pck_coz(ham)


def test_EKSIK_KAYIT_SAYISI_None_ve_hata_veriyor() -> None:
    """total_params ile gerçek çözülen kayıt sayısı uyuşmazsa (kesik indirme)
    sessizce yarım sonuç DÖNMEMELİ."""
    ham = struct.pack("<HHH", 0x671C, 5, 5) + _kayit_uret(
        b"ONLY_ONE", 1.0, b"", 1.0
    )
    with pytest.raises(ValueError):
        fru.pck_coz(ham)
