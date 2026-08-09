#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""`girdap_saat_kur.system_time_ayikla` — bağımsız MAVLink ayrıştırıcı testleri.

Bu ayrıştırıcı NEDEN var: Jetson'da **internet yok** (09.08 ölçümü: varsayılan
rota yok, DNS yok) ve bu md 4.1'in sonucu (WiFi yasak) → `pymavlink`
KURULAMAZ. Hedef makinede kurulamayan bir bağımlılık yarışma-kritik bir açılış
servisinde olmamalı, o yüzden çerçeveleme elle yapılıyor.

Elle yazılmış çerçeve ayrıştırıcısı **sessizce yanlış** olmaya çok müsaittir
(yanlış ofset → çöp saat → md 4.2 damgaları bozulur). Bu yüzden testler gerçek
bayt dizileri üretip hem v1 hem v2'yi, hem CRC reddini hem parça parça gelen
akışı zorluyor.
"""
from __future__ import annotations

import importlib.util
from pathlib import Path

import pytest

_YOL = (Path(__file__).resolve().parents[2] / "scripts" / "girdap_saat_kur.py")
_spec = importlib.util.spec_from_file_location("girdap_saat_kur", _YOL)
gsk = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(gsk)


def _cerceve(usec: int, *, v2: bool = True, boot_ms: int = 1234,
             crc_boz: bool = False) -> bytes:
    """Gerçek bir SYSTEM_TIME çerçevesi üretir (CRC dahil)."""
    yuk = usec.to_bytes(8, "little") + boot_ms.to_bytes(4, "little")
    if v2:
        govde = bytes([len(yuk), 0, 0, 7, 1, 1]) + (2).to_bytes(3, "little") + yuk
        stx = 0xFD
    else:
        govde = bytes([len(yuk), 7, 1, 1, 2]) + yuk
        stx = 0xFE
    crc = gsk._crc_hesapla(govde, gsk._SYSTEM_TIME_CRC_EXTRA)
    if crc_boz:
        crc ^= 0xFFFF
    return bytes([stx]) + govde + crc.to_bytes(2, "little")


_USEC = 1_786_000_000_000_000  # 2026-08-09 civarı


@pytest.mark.parametrize("v2", [True, False])
def test_temiz_cerceve_ayristirilir(v2: bool) -> None:
    """v2 (0xFD) ve v1 (0xFE) ikisi de okunmalı — FC hangisini konuşursa."""
    t = bytearray(_cerceve(_USEC, v2=v2))
    assert gsk.system_time_ayikla(t) == _USEC
    assert len(t) == 0, "cerceve tuketilmeliydi"


def test_onunde_cop_varken_bulunur() -> None:
    """Seri hat ortasından dinlemeye başlanır; baştaki yarım veri atılmalı."""
    t = bytearray(b"\x00\x11\x22rastgele\x99" + _cerceve(_USEC))
    assert gsk.system_time_ayikla(t) == _USEC


def test_CRC_BOZUKSA_KABUL_EDILMEZ() -> None:
    """🔴 En kritik test: rastgele bayt dizisi geçerli çerçeve gibi görünüp
    ÇÖP BİR SAAT yazdırmamalı. CRC bunun tek yapısal savunması."""
    t = bytearray(_cerceve(_USEC, crc_boz=True))
    assert gsk.system_time_ayikla(t) is None


def test_baska_mesajlar_atlanir() -> None:
    """Hatta 10+ mesaj tipi akıyor; yalnız msgid=2 alınmalı."""
    yabanci = bytes([0xFD, 4, 0, 0, 9, 1, 1]) + (30).to_bytes(3, "little") \
        + b"\x01\x02\x03\x04" + b"\xAA\xBB"          # ATTITUDE benzeri, CRC uydurma
    t = bytearray(yabanci + _cerceve(_USEC))
    assert gsk.system_time_ayikla(t) == _USEC


def test_YARIM_cerceve_beklenir_veri_ATILMAZ() -> None:
    """Seri okuma parça parça gelir. Yarım çerçeve gelince None dönüp
    tamponu KORUMALI; atarsa mesaj sonsuza kadar kaçırılır."""
    tam = _cerceve(_USEC)
    t = bytearray(tam[:9])
    assert gsk.system_time_ayikla(t) is None
    assert len(t) == 9, "yarim cerceve ATILMAMALI"
    t += tam[9:]
    assert gsk.system_time_ayikla(t) == _USEC


def test_imzali_v2_cercevesi_uzunlugu_dogru_atlanir() -> None:
    """v2 imzalı çerçeve (incompat 0x01) 13 bayt fazla taşır. Uzunluk yanlış
    hesaplanırsa SONRAKİ mesajın başı kaçar → ayrıştırıcı senkron kaybeder."""
    yuk = (0).to_bytes(8, "little") + (5).to_bytes(4, "little")
    govde = bytes([len(yuk), 0x01, 0, 7, 1, 1]) + (2).to_bytes(3, "little") + yuk
    crc = gsk._crc_hesapla(govde, gsk._SYSTEM_TIME_CRC_EXTRA)
    imzali = bytes([0xFD]) + govde + crc.to_bytes(2, "little") + bytes(13)
    t = bytearray(imzali + _cerceve(_USEC))
    # İlk çerçeve usec=0 (fix yok) → 0 döner, sonra ikinciden gerçek değer.
    assert gsk.system_time_ayikla(t) == 0
    assert gsk.system_time_ayikla(t) == _USEC


def test_crc_algoritmasi_bilinen_degeri_veriyor() -> None:
    """CRC uygulaması X.25/CCITT (crc16-mcrf4xx) olmalı. Bilinen vektör:
    '123456789' → 0x6F91. Algoritma kayarsa TÜM çerçeveler reddedilir ve
    servis sessizce hiç saat kurmaz."""
    crc = 0xFFFF
    for b in b"123456789":
        crc = gsk._crc_accumulate(b, crc)
    assert crc == 0x6F91
