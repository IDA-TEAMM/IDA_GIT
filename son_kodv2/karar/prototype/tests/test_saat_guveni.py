#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""`saat_guveni` — ölçütü ve struct hizalamasını donduran testler.

Bu testler bir DEĞERİ değil bir SÖZLEŞMEYİ koruyor: ölçüt algı katmanındaki
`girdap_ida_algi/saat.py` ile aynı olmak zorunda (iki teslim ailesi aynı
gerçeği söylemeli), ama `algi/` bu repoda ayna olduğu ve CI'da bulunmadığı
için import edilemiyor. Sabitler burada çivilenir → sessiz ayrışma CI'da kırılır.
"""
from __future__ import annotations

import ctypes

from prototype.telemetry.saat_guveni import (
    MODES_SALT_OKUNUR,
    STA_UNSYNC,
    TIME_ERROR,
    Timex,
    saat_guvenilir_mi,
)


def test_cekirdek_sabitleri_DONDURULDU() -> None:
    """🔴 Değiştirilirse ölçüt sessizce yanlışa döner.

    Kaynak: çekirdek `include/linux/timex.h` + `kernel/time/ntp.c`. Bunlar
    ABI, "iyileştirilecek" sayılar değil. Algı katmanındaki `saat.py` de
    birebir aynılarını kullanıyor.
    """
    assert TIME_ERROR == 5
    assert STA_UNSYNC == 0x0040
    assert MODES_SALT_OKUNUR == 0, "modes!=0 → salt-okunur olmaz, SAAT DEĞİŞİR"


def test_timex_struct_boyutu_dogru() -> None:
    """🔴 Yanlış hizalama `status` alanını kaydırır → SESSİZCE yanlış sonuç.

    Struct çok kısa olursa çekirdek kalan alanları yazmaz ve `status`
    çöp okunur; çok uzun olması sorun değil ama boyut sapması hizalama
    hatasının en iyi göstergesi. 64-bit `long`'da beklenen: 208 byte.
    """
    assert ctypes.sizeof(Timex()) == 208


def test_salt_okunur_cagri_SAATI_DEGISTIRMEZ() -> None:
    """Ölçüm yan etkisiz olmalı — yoksa her kontrol saati oynatırdı."""
    import time

    once = time.time()
    for _ in range(5):
        saat_guvenilir_mi()
    sonra = time.time()
    # 5 çağrı milisaniyeler sürer; 1 s'lik pay sıçrama olmadığını gösterir.
    assert abs((sonra - once)) < 1.0


def test_sozlesme_bool_ve_gerekce_dondurur() -> None:
    """Çağıranlar `(bool, str)` bekliyor; gerekçe log'a basılıyor."""
    guvenilir, gerekce = saat_guvenilir_mi()
    assert isinstance(guvenilir, bool)
    assert isinstance(gerekce, str) and gerekce, "gerekce BOS olmamali"


def test_bu_makinede_kosuyor_ve_bir_cevap_veriyor() -> None:
    """Duman testi: platform ne olursa olsun istisna FIRLATMAZ.

    `saat_guvenilir_mi` hata hâlinde `False` döner (fail-safe) — ölçemediğimiz
    şeyi "güvenilir" saymak tam da kapatmaya çalıştığımız sessiz arızadır.
    """
    guvenilir, gerekce = saat_guvenilir_mi()
    assert guvenilir in (True, False)
    print(f"bu makine: guvenilir={guvenilir} · {gerekce}")
