#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""GİRDAP İDA — Jetson sistem saatini Pixhawk'ın GPS saatinden kurar.

Açılışta **bir kez** koşar (`girdap-saat.service`, oneshot), saati kurar, çıkar.

═══════════════════════════════════════════════════════════════════════════
NEDEN VAR
═══════════════════════════════════════════════════════════════════════════
Jetson yanlış saatle açılıyor — ÖLÇÜLDÜ, tahmin değil: 06.08'de ~15 saat,
07.08'de ~3 saat geri. md 4.2 teslimleri (Dosya-1/2/3) **zaman etiketli** olmak
zorunda; geçersiz dosya başına **5 ceza puanı** (md 5.5.4.3.5). Yanlış saat
SESSİZDİR: dosyalar üretilir, damgalar makul görünür, kimse fark etmez.

NTP çözüm DEĞİL: md 4.1 tüm bilgisayarlarda WiFi'yi kapatmayı zorunlu kılıyor
→ yarışma günü internet yok. Ağ gerektirmeyen tek mutlak zaman kaynağı GPS.

Klasik `gpsd` + chrony da çalışmaz: F9P Rover'ın bağımsız UART'ı yok, tek
birleşik konnektörle yalnız Pixhawk GPS1'e bağlı (FTDI ile tapping denendi,
tanımlanamayan ikili protokol geldi). Zaman ancak **FC üzerinden** alınabilir.

═══════════════════════════════════════════════════════════════════════════
TASARIM KARARLARI (üçü de tuzaktan kaçınmak için)
═══════════════════════════════════════════════════════════════════════════
1) 🔴 **MAVROS ÜZERİNDEN DEĞİL, doğrudan seri porttan.** MAVROS'u
   `girdap-karar.service` başlatıyor; saat kurucu MAVROS'u bekleseydi karar
   servisinin de saat kurucudan sonra başlaması gerekirdi → DAİRESEL
   bağımlılık. Doğrudan pymavlink ile bağlanmak bunu keser.

2) 🔴 **Oneshot olmak ZORUNDA — seri port tekildir.** MAVROS `/dev/ttyUSB0`'ı
   açık tutar; iki süreç aynı portu açamaz. Bu script MAVROS'tan ÖNCE koşar,
   portu bırakır, çıkar (`Before=girdap-karar.service`).

3) 🔴 **SYSTEM_TIME İSTENİR, gelmesi UMULMAZ.** Mesaj ArduPilot'ta bir EXTRA
   stream grubunda; varsayılan hızda gelmeyebilir. `SET_MESSAGE_INTERVAL`
   (MAV_CMD 511) ile açıkça isteniyor, ayrıca eski FC'ler için
   `REQUEST_DATA_STREAM` yedeği var.

Saat neden karar servisinden ÖNCE doğru olmalı: `telemetry_node`,
`local_map_node` ve `lidar_kayit_node` oturum dizinini/dosya adını
**başlangıçta** üretiyor (`datetime.now().strftime(...)`). Saat sonradan
düzelse bile **dosya adları yanlış kalır**.

═══════════════════════════════════════════════════════════════════════════
STA_UNSYNC — neden ayrıca temizliyoruz
═══════════════════════════════════════════════════════════════════════════
Algı katmanı (`girdap_ida_algi/saat.py`) saat güvenini çekirdeğin `adjtimex`
STA_UNSYNC bayrağından okuyor — doğru ölçüt, çünkü "tarih makul mü" saatler
mertebesindeki geriliği göremez. Ama `clock_settime`/`date -s` o bayrağı
**temizlemez**: saat artık doğru olduğu hâlde kareler "güvenilmez" damgalanır
(Eyüp bunu kendi dosyasında bilinen yanlış-negatif olarak yazmış).

Biz saati GERÇEK bir referansa (GPS) göre kurduğumuz için bayrağı temizlemek
**dürüsttür** — chrony/ntpd de senkron olunca tam bunu yapar (ADJ_STATUS).

⚠️ Bayrak sonsuza kadar temiz kalmaz: çekirdek `second_overflow()` her saniye
`time_maxerror += 500 µs` yapıyor, `NTP_PHASE_LIMIT` (16.000.000 µs) aşılınca
STA_UNSYNC geri gelir → **~8,9 saat**. Ağsız teknede bu beklenen davranış;
oturum 8,9 saati geçerse damgalar "güvenilmez"e döner. Yarışma koşusu 20 dk,
sorun değil.

═══════════════════════════════════════════════════════════════════════════
ÇIKIŞ KODLARI (systemd ve operatör için)
═══════════════════════════════════════════════════════════════════════════
  0  saat kuruldu (ya da zaten doğruydu, dokunulmadı)
  1  FC'ye bağlanılamadı        → saat kurulMADI
  2  zaman aşımı, geçerli SYSTEM_TIME gelmedi (GPS fix yok?) → kurulMADI
  3  yetki yok (root değil)     → kurulMADI
  4  pymavlink kurulu değil     → kurulMADI
Sıfır dışı her kod "saat güvenilmez" demektir; `girdap-karar.service` yine de
başlar (görev başlamazsa hiç puan yok) — teslimler `saat_guvenilir=false` ile
damgalanır.
"""
from __future__ import annotations

import argparse
import ctypes
import ctypes.util
import os
import sys
import time
from datetime import datetime, timezone

# --- adjtimex(2) sabitleri (çekirdek: include/linux/timex.h) ---
ADJ_STATUS = 0x0010
STA_UNSYNC = 0x0040
TIME_ERROR = 5

#: Makul zaman penceresi. GPS'ten gelen değer bunun dışındaysa GÜVENİLMEZ:
#: FC fix'siz iken 0 ya da çöp gönderebilir, ve yanlış saati "GPS'ten geldi"
#: diye yazmak mevcut durumdan DAHA kötü olur (yanlışlığı gizler).
_EN_ERKEN = datetime(2026, 1, 1, tzinfo=timezone.utc).timestamp()
_EN_GEC = datetime(2030, 1, 1, tzinfo=timezone.utc).timestamp()

#: Bu farkın altındaysa saate DOKUNMUYORUZ. Sebep: gereksiz sıçrama ROS
#: stamp'lerinde ve süre ölçümlerinde tutarsızlık yaratır.
_TOLERANS_S = 2.0


class _Timex(ctypes.Structure):
    """`struct timex` (glibc/aarch64 + x86_64: `long` = 64 bit)."""

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
        # çekirdeğin ayırdığı padding (11 int) — boyut tutmalı
        ("_pad", ctypes.c_int * 11),
    ]


def _libc() -> ctypes.CDLL:
    return ctypes.CDLL(ctypes.util.find_library("c") or "libc.so.6", use_errno=True)


def sta_unsync_temizle(log) -> bool:
    """Çekirdeğin STA_UNSYNC bayrağını temizler. Root ister.

    Önce salt-okunur (`modes=0`) mevcut status alınır, sonra yalnız o bit
    düşürülüp `ADJ_STATUS` ile geri yazılır — diğer bitlere dokunulmaz.
    """
    try:
        libc = _libc()
        tx = _Timex()
        tx.modes = 0
        libc.adjtimex(ctypes.byref(tx))
        if not (tx.status & STA_UNSYNC):
            log("STA_UNSYNC zaten temiz, dokunulmadi")
            return True
        yeni = _Timex()
        yeni.modes = ADJ_STATUS
        yeni.status = tx.status & ~STA_UNSYNC
        rc = libc.adjtimex(ctypes.byref(yeni))
        if rc < 0:
            log(f"STA_UNSYNC temizlenemedi (adjtimex rc={rc}, errno="
                f"{ctypes.get_errno()}) — saat DOGRU ama bayrak kirli kalir")
            return False
        log("STA_UNSYNC temizlendi (saat GPS ile disipline edildi)")
        return True
    except Exception as e:  # pragma: no cover - platforma bagli
        log(f"STA_UNSYNC temizleme hatasi: {e}")
        return False


def gps_saati_al(port: str, baud: int, zaman_asimi: float, log):
    """FC'den GPS kaynaklı UNIX zamanını (saniye, float) okur; yoksa None.

    SYSTEM_TIME açıkça İSTENİR (bkz. modül docstring'i, tasarım kararı 3).
    """
    try:
        from pymavlink import mavutil
    except ImportError:
        log("pymavlink kurulu degil: pip3 install pymavlink")
        return None, 4

    # Portun BELIRMESINI bekle. Acilista udev/FTDI enumerate'i gecikebilir ve
    # `mavlink_connection` yok olan porta istisna atar. systemd device unit'ine
    # (`After=dev-...device`) guvenmek yerine burada bekliyoruz: symlink
    # kullanildiginda o unit adi da degisir, bu yontem ikisinde de calisir.
    _bekleme_bitis = time.monotonic() + min(30.0, zaman_asimi / 3)
    while not os.path.exists(port) and time.monotonic() < _bekleme_bitis:
        time.sleep(0.5)
    if not os.path.exists(port):
        log(f"port belirmedi: {port} — kablo/udev kurali? (ls -l /dev/serial/by-id)")
        return None, 1

    log(f"FC'ye baglaniliyor: {port} @ {baud}")
    try:
        m = mavutil.mavlink_connection(port, baud=baud)
    except Exception as e:
        log(f"seri port acilamadi: {e}")
        return None, 1

    bitis = time.monotonic() + zaman_asimi
    try:
        if m.wait_heartbeat(timeout=max(5.0, zaman_asimi / 3)) is None:
            log("heartbeat gelmedi — FC bagli mi, port dogru mu?")
            return None, 1
        log(f"heartbeat alindi (sys={m.target_system} comp={m.target_component})")

        # SYSTEM_TIME'i ACIKCA iste — 1 Hz (1e6 us). Iki yol: modern
        # SET_MESSAGE_INTERVAL, ve eski FC'ler icin REQUEST_DATA_STREAM.
        try:
            m.mav.command_long_send(
                m.target_system, m.target_component,
                mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
                mavutil.mavlink.MAVLINK_MSG_ID_SYSTEM_TIME, 1_000_000,
                0, 0, 0, 0, 0)
        except Exception as e:
            log(f"SET_MESSAGE_INTERVAL gonderilemedi ({e}) — yedege geciliyor")
        try:
            m.mav.request_data_stream_send(
                m.target_system, m.target_component,
                mavutil.mavlink.MAV_DATA_STREAM_EXTRA3, 1, 1)
        except Exception:
            pass

        while time.monotonic() < bitis:
            msg = m.recv_match(type="SYSTEM_TIME", blocking=True, timeout=2.0)
            if msg is None:
                continue
            t = msg.time_unix_usec / 1e6
            if t <= 0:
                log("SYSTEM_TIME geldi ama time_unix_usec=0 (GPS fix yok) — beklenilyor")
                continue
            if not (_EN_ERKEN <= t <= _EN_GEC):
                # Yanlis saati "GPS'ten geldi" diye yazmak, yanlisligi
                # GIZLEDIGI icin mevcut durumdan daha kotudur.
                log(f"SYSTEM_TIME makul aralikta DEGIL ({_iso(t)}) — reddedildi")
                continue
            log(f"GPS saati alindi: {_iso(t)}")
            return t, 0
        log("zaman asimi — gecerli SYSTEM_TIME gelmedi (GPS fix yok?)")
        return None, 2
    finally:
        try:
            m.close()
        except Exception:
            pass


def _iso(t: float) -> str:
    return datetime.fromtimestamp(t, tz=timezone.utc).isoformat(timespec="seconds")


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--port", default="/dev/ttyUSB0",
                    help="FC seri portu (hardware.yaml fcu_url ile AYNI olmali)")
    ap.add_argument("--baud", type=int, default=57600)
    ap.add_argument("--zaman-asimi", type=float, default=90.0,
                    help="gecerli SYSTEM_TIME icin beklenecek azami sure (s)")
    ap.add_argument("--kuru", action="store_true",
                    help="saati KURMA, yalnizca oku ve farki bildir (testte)")
    a = ap.parse_args(argv)

    def log(s: str) -> None:
        # systemd journald'a gider; tarih YAZMIYORUZ (saat henuz yanlis olabilir,
        # yaniltici olur) — journald kendi damgasini basar.
        print(f"[girdap-saat] {s}", flush=True)

    t, kod = gps_saati_al(a.port, a.baud, a.zaman_asimi, log)
    if t is None:
        log(f"SAAT KURULMADI (kod {kod}) — teslimler saat_guvenilir=false "
            "ile damgalanmali")
        return kod

    simdi = time.time()
    fark = t - simdi
    log(f"sistem saati {_iso(simdi)} · fark {fark:+.1f} s")

    if abs(fark) <= _TOLERANS_S:
        log(f"fark {_TOLERANS_S} s toleransinin altinda — saate DOKUNULMADI")
        sta_unsync_temizle(log)
        return 0

    if a.kuru:
        log("--kuru verildi: saat KURULMADI (yalniz olcum)")
        return 0

    if os.geteuid() != 0:
        log("root degil — saat kurulamaz (systemd servisi root kosar)")
        return 3

    try:
        time.clock_settime(time.CLOCK_REALTIME, t)
    except Exception as e:
        log(f"clock_settime basarisiz: {e}")
        return 3
    log(f"SAAT KURULDU: {_iso(time.time())} (duzeltme {fark:+.1f} s)")

    sta_unsync_temizle(log)

    # RTC'ye de yaz: pil varsa saat guc kesintisini ATLATIR, boylece bir
    # sonraki acilista GPS fix'ini beklemek zorunda kalmayiz.
    # (Orin Nano BBAT pini CR1225 SARJ EDILEMEZ pil ister; sarjli ML1220 +
    #  sarj devresi uyumsuz — bkz. docs/saat_gps_senkron_plani.md.)
    if os.system("hwclock --systohc >/dev/null 2>&1") == 0:
        log("RTC guncellendi (hwclock --systohc)")
    else:
        log("RTC yazilamadi — pil yoksa beklenen, saat yine de dogru")
    return 0


if __name__ == "__main__":
    sys.exit(main())
