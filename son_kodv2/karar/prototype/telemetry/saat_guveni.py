#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Saat güvenilir mi? — ROS-bağımsız tek kaynak (md 4.2 teslim damgaları).

`hardware.launch.py` bunu **bir kez** çağırıp sonucu üç teslim node'una
`saat_guvenilir` parametresi olarak geçirir:
`telemetry_node` (Dosya-2) · `local_map_node` (Dosya-3) · `lidar_kayit_node`
(Dosya-1b). Tek yerde hesaplanır → üç teslim birbirinden AYRIŞMAZ.

═══════════════════════════════════════════════════════════════════════════
NEDEN GEREKLİ — bu tesisat 09.08'e kadar YARIM BAĞLIYDI
═══════════════════════════════════════════════════════════════════════════
`local_map_node` ve `lidar_kayit_node` `saat_guvenilir` parametresini
**okuyordu** ama **hiçbir yerden beslenmiyordu** (ne launch'ta ne yaml'da) →
varsayılan `True`'da kalıyordu, yani sistem **saat 3 saat yanlışken de
"güvenilir" diyordu**. `telemetry_node`'da (Dosya-2) parametre **hiç yoktu**:
dosya adı ve her satırın `zaman` sütunu doğrudan `datetime.now(timezone.utc)`.

Ölçülen arıza: Jetson 06.08'de ~15 saat, 07.08'de ~3 saat geri açıldı.
Geçersiz/yanlış damgalı teslim = **5 ceza puanı** (md 5.5.4.3.5).

═══════════════════════════════════════════════════════════════════════════
ÖLÇÜT: çekirdeğin senkron bayrağı (`adjtimex`, STA_UNSYNC)
═══════════════════════════════════════════════════════════════════════════
Doğru soru **"tarih makul mü?" DEĞİL**, "bu saat bir referansa göre
DÜZELTİLDİ mi?". Mutlak eşik (`epoch >= 2026-01-01`) saatler mertebesindeki
geriliği **tanım gereği göremez** — 3 saat geri bir saat de 2026'nın içindedir.
Cevap çekirdekte: `adjtimex(2)` salt-okunur çağrısı (`modes=0`) yetki istemez;
dönüş `TIME_ERROR(5)` ya da `status & STA_UNSYNC` ise saat disipline
EDİLMEMİŞTİR.

⚠️ Bu ölçüt algı katmanındaki `girdap_ida_algi/saat.py` ile **AYNI** — bilinçli:
iki teslim ailesi (Dosya-1 vs Dosya-2/3) aynı gerçeği söylemeli. Neden import
etmiyoruz: `algi/` bu repoda **ayna** (`algi/KAYNAK.md`: "burada düzenleme
yapılmaz") ve karar paketinin CI'ında o paket **yok** → import zinciri kırılırdı.
Bu yüzden ölçüt burada yeniden yazıldı; `test_saat_guveni.py` sabitleri
(TIME_ERROR=5, STA_UNSYNC=0x0040, modes=0) donduruyor ki iki kopya sessizce
ayrışmasın.

⏳ **Bilinen sınır:** bayrak sonsuza kadar temiz kalmaz. Çekirdek
`second_overflow()` her saniye `time_maxerror += 500 µs` yapıyor;
`NTP_PHASE_LIMIT` (16.000.000 µs) aşılınca STA_UNSYNC geri gelir → **~8,9
saat**. Ağsız teknede bu KABUL EDİLEBİLİR bir yanlış-negatiftir: saat hâlâ
doğrudur ama "kanıtım yok" deriz. Yarışma koşusu 20 dk.

🔴 **Bilinen yanlış-negatif:** `sudo date -s` saati düzeltir ama STA_UNSYNC'i
**temizlemez**. Bu yüzden `scripts/girdap_saat_kur.py` (GPS'ten kuran servis)
bayrağı `ADJ_STATUS` ile ayrıca temizliyor — saat gerçekten bir referansa
(GPS) göre disipline edildiği için bu dürüstlüktür, chrony/ntpd de aynısını
yapar. Elle `date -s` yapan operatör bunu yapmazsa teslimler "güvenilmez"
damgalanır: veri KAYBOLMAZ, yalnız mutlak saat iddia edilmez.
"""
from __future__ import annotations

import ctypes
import ctypes.util
from typing import Tuple

#: `adjtimex(2)` dönüşü: saat senkronize DEĞİL (çekirdek: kernel/time/ntp.c).
TIME_ERROR = 5
#: `include/linux/timex.h` — saat disipline edilmemiş bayrağı.
STA_UNSYNC = 0x0040
#: Salt-okunur çağrı: hiçbir şey değiştirme, yalnız durumu al (yetki istemez).
MODES_SALT_OKUNUR = 0


class Timex(ctypes.Structure):
    """`struct timex` — glibc, `long` = 64 bit (aarch64/Jetson + x86_64).

    Boyut 208 byte olmalı; `test_saat_guveni` bunu doğruluyor. Yanlış hizalama
    `status` alanını kaydırır ve **sessizce yanlış** sonuç verir.
    """

    _fields_ = [
        ("modes", ctypes.c_uint),
        ("offset", ctypes.c_long),
        ("freq", ctypes.c_long),
        ("maxerror", ctypes.c_long),
        ("esterror", ctypes.c_long),
        ("status", ctypes.c_int),
        ("constant", ctypes.c_long),
        ("precision", ctypes.c_long),
        ("tolerance", ctypes.c_long),
        ("time_sec", ctypes.c_long),
        ("time_usec", ctypes.c_long),
        ("tick", ctypes.c_long),
        ("ppsfreq", ctypes.c_long),
        ("jitter", ctypes.c_long),
        ("shift", ctypes.c_int),
        ("stabil", ctypes.c_long),
        ("jitcnt", ctypes.c_long),
        ("calcnt", ctypes.c_long),
        ("errcnt", ctypes.c_long),
        ("stbcnt", ctypes.c_long),
        ("tai", ctypes.c_int),
        ("_pad", ctypes.c_int * 11),
    ]


def saat_guvenilir_mi() -> Tuple[bool, str]:
    """(güvenilir_mi, gerekçe). Yetki istemez, ağ istemez, saniyenin altı sürer.

    Hata hâlinde **GÜVENİLMEZ** döner (fail-safe): ölçemediğimiz bir şeyi
    "güvenilir" saymak, tam da kapatmaya çalıştığımız sessiz arızadır.
    """
    try:
        libc = ctypes.CDLL(
            ctypes.util.find_library("c") or "libc.so.6", use_errno=True
        )
        tx = Timex()
        tx.modes = MODES_SALT_OKUNUR
        rc = libc.adjtimex(ctypes.byref(tx))
        if rc < 0:
            return False, f"adjtimex hatasi (errno={ctypes.get_errno()})"
        if rc == TIME_ERROR:
            return False, "cekirdek TIME_ERROR: saat hic disipline edilmemis"
        if tx.status & STA_UNSYNC:
            return False, "STA_UNSYNC set: saat bir referansa gore duzeltilmemis"
        return True, f"cekirdek senkron (maxerror={tx.maxerror} us)"
    except Exception as e:  # pragma: no cover - platforma bagli
        return False, f"olculemedi ({e}) — guvenli tarafta guvenilmez sayildi"
