#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""GİRDAP İDA — Jetson sistem saatini Pixhawk'ın GPS saatinden kurar.

Açılışta **bir kez** koşar (`girdap-saat.service`, oneshot), saati kurar, çıkar.

═══════════════════════════════════════════════════════════════════════════
NEDEN VAR
═══════════════════════════════════════════════════════════════════════════
Jetson yanlış saatle açılıyor — ÖLÇÜLDÜ, tahmin değil: 06.08'de ~15 saat,
07.08'de ~3 saat, **09.08'de 25 saat 57 dakika** geri. md 4.2 teslimleri
(Dosya-1/2/3) **zaman etiketli** olmak zorunda; geçersiz dosya başına **5 ceza
puanı** (md 5.5.4.3.5). Yanlış saat SESSİZDİR: dosyalar üretilir, damgalar
makul görünür, kimse fark etmez.

🔴 **KÖK NEDEN (09.08'de Jetson'da ölçüldü):** `timedatectl` →
`RTC time: Thu 1970-01-01 00:13:54` · `System clock synchronized: no`.
RTC **pilsiz** — her güç kesintisinde 1970'e düşüyor (13 dakika = açılıştan
beri geçen süre), sistem saati de son kapanış zamanından devam ediyor. Yani
sapma "son ne zaman kapatıldıysa o kadar" — bu yüzden her ölçümde farklı.
Tamamlayıcı donanım önlemi: **CR1225 (şarj edilemez)** saat pili; şarjlı
ML1220 + şarj devresi Orin Nano BBAT ile uyumsuz. Bkz.
`docs/saat_gps_senkron_plani.md`.

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

2) 🔴 **Oneshot olmak ZORUNDA — seri port tekildir.** MAVROS `/dev/pixhawk`'ı
   açık tutar; iki süreç aynı portu açamaz. Bu script MAVROS'tan ÖNCE koşar,
   portu bırakır, çıkar (`Before=girdap-karar.service`).

Port adı `/dev/pixhawk`: Jetson'da **iki FTDI** var ve `ttyUSBn` numarası
enumerate sırasına göre verilir. Sabit adı Eyüp'ün `99-girdap-fc.rules`
kuralı üretiyor (seri no `DU0EFEA7`, ayrıca `ID_MM_DEVICE_IGNORE=1` ile
ModemManager'ın portu problayıp MAVROS'u ~30 s geciktirmesini de engelliyor —
F-M.8). Kural 09.08'de Jetson'da **kurulu ve çalışır** bulundu; `fcu_url` de
zaten bu symlink'e bakıyor.

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
ADJ_MAXERROR = 0x0004
ADJ_ESTERROR = 0x0008
ADJ_STATUS = 0x0010
STA_UNSYNC = 0x0040
TIME_ERROR = 5

#: `clock_settime` sonrası çekirdek `time_maxerror`'u AZAMİYE çıkarır
#: (`NTP_PHASE_LIMIT` = 16.000.000 µs). O sınırın üstünde kaldığı sürece
#: `second_overflow()` her saniye `STA_UNSYNC`'i GERİ KOYAR — yani bayrağı
#: tek başına temizlemek İŞE YARAMAZ (2026-08-11'de sahada ölçüldü: bit
#: temizlendi, `timedatectl` yine "synchronized: no" dedi, `maxerror` tam
#: 16.000.000'daydı). chrony/ntpd de bu yüzden `maxerror`'u birlikte yazar.
#:
#: Değer GPS zamanının GERÇEK belirsizliğini yansıtmalı — sıfır yazmak yalan
#: olur. MAVLink `SYSTEM_TIME` 57600 baud seri hat üzerinden geliyor; çerçeve
#: gecikmesi + FC'nin kendi damgalama gecikmesi ~onlarca ms. 100 ms dürüst bir
#: üst sınır. Yan fayda: 500 µs/s büyüme hızıyla
#: (16.000.000 − 100.000) / 500 ≈ 31.800 s ≈ **8,8 saat** boyunca "senkron"
#: kalır — belgelediğimiz ~8,9 saatlik pencereyle birebir tutuyor.
MAXERROR_US = 100_000

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
    """`STA_UNSYNC`'i temizler **ve `maxerror`'u sıfırlar**. Root ister.

    🔴 İKİSİ BİRLİKTE YAPILMAK ZORUNDA. İlk sürüm yalnız biti temizliyordu ve
    2026-08-11'de sahada işe yaramadığı ölçüldü: bit temizlendi ama
    `clock_settime`'ın azamiye çıkardığı `maxerror` (16.000.000 µs) sınırın
    üstünde kaldığı için çekirdek bayrağı bir sonraki saniyede geri koydu.
    `timedatectl` "synchronized: no" demeye devam etti → Eyüp'ün `saat.py`'si
    teslimleri "güvenilmez" damgalayacaktı, saat DOĞRU olduğu hâlde.

    Sonra da **doğrular** — yazıp sonucuna bakmamak bugünün dersi.
    """
    try:
        libc = _libc()
        tx = _Timex()
        tx.modes = 0
        libc.adjtimex(ctypes.byref(tx))

        yeni = _Timex()
        yeni.modes = ADJ_STATUS | ADJ_MAXERROR | ADJ_ESTERROR
        yeni.status = tx.status & ~STA_UNSYNC
        yeni.maxerror = MAXERROR_US
        yeni.esterror = MAXERROR_US
        rc = libc.adjtimex(ctypes.byref(yeni))
        if rc < 0:
            log(f"adjtimex yazilamadi (rc={rc}, errno={ctypes.get_errno()}) — "
                "saat DOGRU ama bayrak kirli kalir")
            return False

        # --- DOGRULAMA: gerçekten tuttu mu? ---
        kon = _Timex()
        kon.modes = 0
        rc2 = libc.adjtimex(ctypes.byref(kon))
        if (kon.status & STA_UNSYNC) or rc2 == TIME_ERROR:
            log(f"STA_UNSYNC GERI GELDI (status=0x{kon.status:04x} rc={rc2} "
                f"maxerror={kon.maxerror} us) — saat dogru ama 'guvenilir' "
                "damgalanmayacak")
            return False
        log(f"STA_UNSYNC temizlendi + maxerror={kon.maxerror} us "
            "(saat GPS ile disipline edildi, DOGRULANDI)")
        return True
    except Exception as e:  # pragma: no cover - platforma bagli
        log(f"STA_UNSYNC temizleme hatasi: {e}")
        return False


#: SYSTEM_TIME mesaj kimliği ve MAVLink CRC_EXTRA'sı (mavlink common.xml).
_SYSTEM_TIME_ID = 2
_SYSTEM_TIME_CRC_EXTRA = 137


def _crc_accumulate(b: int, crc: int) -> int:
    """MAVLink X.25/CCITT CRC birikimi (crc16-mcrf4xx)."""
    tmp = b ^ (crc & 0xFF)
    tmp = (tmp ^ (tmp << 4)) & 0xFF
    return ((crc >> 8) ^ (tmp << 8) ^ (tmp << 3) ^ (tmp >> 4)) & 0xFFFF


def _crc_hesapla(govde: bytes, crc_extra: int) -> int:
    crc = 0xFFFF
    for b in govde:
        crc = _crc_accumulate(b, crc)
    return _crc_accumulate(crc_extra, crc)


def system_time_ayikla(tampon: bytearray):
    """Tampondan DOĞRULANMIŞ bir SYSTEM_TIME varsa `time_unix_usec` döndürür.

    Tüketilen baytları tampondan siler. Bulamazsa None.

    NEDEN ELLE AYRIŞTIRIYORUZ — pymavlink'e bağlanamayız:
    Jetson'da **internet YOK** (09.08'de ölçüldü: varsayılan rota yok, DNS yok)
    ve bu bir arıza değil, md 4.1'in sonucu (WiFi yasak). Hedef makinede
    KURULAMAYAN bir bağımlılık, yarışma-kritik bir açılış servisinde olmamalı.
    Elle ayrıştırma bu servisi **bağımlılıksız** yapar.

    Mesajı İSTEMEK gerekmiyor: `SR2_EXTRA3 = 10` (canlı param dökümü) ve
    ArduPilot'ta SYSTEM_TIME **EXTRA3** grubunda → TELEM2'de 10 Hz kendiliğinden
    akıyor. Yani yalnız OKUYUP ayrıştırmak yeterli, çerçeve ÜRETMEK gerekmez.

    CRC **doğrulanıyor** (CRC_EXTRA=137): rastgele bayt dizisinin geçerli
    çerçeve gibi görünüp çöp bir saat yazdırmasına karşı. Zaman aralığı
    kontrolü (çağıran tarafta) ikinci süzgeç.
    """
    while True:
        # v2 (0xFD) ya da v1 (0xFE) başlangıcını bul; öncesindeki çöpü at.
        i2, i1 = tampon.find(0xFD), tampon.find(0xFE)
        adaylar = [i for i in (i2, i1) if i >= 0]
        if not adaylar:
            del tampon[:]
            return None
        i = min(adaylar)
        if i:
            del tampon[:i]
        v2 = tampon[0] == 0xFD
        # Başlık: v2 10 bayt (msgid 3 bayt), v1 6 bayt (msgid 1 bayt)
        bas = 10 if v2 else 6
        if len(tampon) < bas + 2:
            return None                      # başlık tamamlanmadı, daha bekle
        uzunluk = tampon[1]
        imza = 13 if (v2 and (tampon[2] & 0x01)) else 0
        toplam = bas + uzunluk + 2 + imza
        if len(tampon) < toplam:
            return None                      # çerçeve tamamlanmadı, daha bekle
        cerceve = bytes(tampon[:toplam])
        msgid = (int.from_bytes(cerceve[7:10], "little") if v2 else cerceve[5])
        if msgid == _SYSTEM_TIME_ID:
            govde = cerceve[1:bas + uzunluk]  # STX hariç, CRC hariç
            gelen = int.from_bytes(
                cerceve[bas + uzunluk:bas + uzunluk + 2], "little")
            if _crc_hesapla(govde, _SYSTEM_TIME_CRC_EXTRA) == gelen:
                yuk = cerceve[bas:bas + uzunluk]
                if len(yuk) >= 8:            # time_unix_usec = ilk alan
                    del tampon[:toplam]
                    return int.from_bytes(yuk[:8], "little")
        # Bu çerçeve bizim değil ya da CRC tutmadı → 1 bayt kaydır, tekrar ara.
        del tampon[:1]


def gps_saati_al_bagimsiz(port: str, zaman_asimi: float, log):
    """pymavlink OLMADAN: portu oku, SYSTEM_TIME ayıkla. (t, kod) döner."""
    try:
        import serial  # pyserial — Jetson'da kurulu (3.5, 09.08 teyidi)
        sp = serial.Serial(port, 57600, timeout=1.0)
        kapat = sp.close
        oku = lambda: sp.read(4096) or b""     # noqa: E731
    except ImportError:
        # pyserial de yoksa stdlib termios ile aç (garantili mevcut).
        import termios
        fd = os.open(port, os.O_RDONLY | os.O_NOCTTY)
        a = termios.tcgetattr(fd)
        a[4] = a[5] = termios.B57600                     # ispeed / ospeed
        a[2] = termios.CS8 | termios.CREAD | termios.CLOCAL
        a[0] = a[1] = a[3] = 0                           # iflag/oflag/lflag ham
        a[6][termios.VMIN], a[6][termios.VTIME] = 0, 10  # 1 s zaman aşımı
        termios.tcsetattr(fd, termios.TCSANOW, a)
        kapat = lambda: os.close(fd)                     # noqa: E731
        oku = lambda: os.read(fd, 4096)                  # noqa: E731

    tampon = bytearray()
    bitis = time.monotonic() + zaman_asimi
    sifir_uyarildi = False
    try:
        while time.monotonic() < bitis:
            veri = oku()
            if not veri:
                continue
            tampon += veri
            if len(tampon) > 65536:                      # sınırsız büyümesin
                del tampon[:-4096]
            while True:
                usec = system_time_ayikla(tampon)
                if usec is None:
                    break
                t = usec / 1e6
                if t <= 0:
                    if not sifir_uyarildi:
                        log("SYSTEM_TIME geldi ama time_unix_usec=0 "
                            "(GPS fix yok) — beklenilyor")
                        sifir_uyarildi = True
                    continue
                if not (_EN_ERKEN <= t <= _EN_GEC):
                    log(f"SYSTEM_TIME makul aralikta DEGIL ({_iso(t)}) — reddedildi")
                    continue
                log(f"GPS saati alindi (bagimsiz ayristirici): {_iso(t)}")
                return t, 0
        log("zaman asimi — gecerli SYSTEM_TIME gelmedi (GPS fix yok?)")
        return None, 2
    finally:
        try:
            kapat()
        except Exception:
            pass


def gps_saati_al(port: str, baud: int, zaman_asimi: float, log):
    """FC'den GPS kaynaklı UNIX zamanını (saniye, float) okur; yoksa None."""
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

    # BAĞIMSIZ YOL BİRİNCİL: Jetson'da pymavlink YOK ve internet olmadığı için
    # kurulamaz (md 4.1). pymavlink varsa (geliştirme makinesi) onu kullanırız —
    # daha olgun çerçeveleme — ama üretimde koşan yol bağımsız ayrıştırıcıdır.
    try:
        from pymavlink import mavutil
    except ImportError:
        log("pymavlink yok → bagimsiz ayristirici kullaniliyor (SR2_EXTRA3=10, "
            "SYSTEM_TIME kendiliginden akiyor; istek gerekmez)")
        return gps_saati_al_bagimsiz(port, zaman_asimi, log)

    log(f"FC'ye baglaniliyor (pymavlink): {port} @ {baud}")
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
    ap.add_argument("--port", default="/dev/pixhawk",
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

    # RTC'ye de yaz: pil varsa saat guc kesintisini ATLATIR, boylece bir
    # sonraki acilista GPS fix'ini beklemek zorunda kalmayiz.
    # (Orin Nano BBAT pini CR1225 SARJ EDILEMEZ pil ister; sarjli ML1220 +
    #  sarj devresi uyumsuz — bkz. docs/saat_gps_senkron_plani.md.)
    # 🔴 2026-08-11'de olculdu: RTC PILSIZ — guc kesilince saat 1970'e dustu.
    # Yani bu yazma su an yalniz ayni guc oturumu icinde ise yariyor.
    if os.system("hwclock --systohc >/dev/null 2>&1") == 0:
        log("RTC guncellendi (hwclock --systohc)")
    else:
        log("RTC yazilamadi — pil yoksa beklenen, saat yine de dogru")

    # 🔴 SIRA ONEMLI: bayrak temizleme EN SONDA. `clock_settime` maxerror'u
    # azamiye cikariyor, `hwclock` da cekirdegin saat durumuna dokunabiliyor;
    # ikisinden ONCE temizlemek bosa gider. 2026-08-11'de ilk surumde
    # temizleme ortada yapiliyordu ve bayrak geri geliyordu.
    sta_unsync_temizle(log)
    return 0


if __name__ == "__main__":
    sys.exit(main())
