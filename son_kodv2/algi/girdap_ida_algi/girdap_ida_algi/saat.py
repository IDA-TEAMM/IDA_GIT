#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""GİRDAP İDA — saat güvenilirliği (SAF katman, ROS/depthai/ağ bağımsız).

NEDEN AYRI MODÜL: saatin doğruluğu İKİ ayrı yerde puana dokunuyor ve ikisi de
bizim alanımız (algı):
  1) **Veri seti manifest'i** — hangi karenin hangi saatte/ışıkta çekildiği.
     Veri setinin asıl değeri çeşitlilik denetimi; yanlış saat bunu zehirler.
  2) **Dosya-1 (şartname md 4.2, s.14)** — *"her bir frame zaman etiketine
     sahip olacak şekilde mp4"*. Teslim edilmeyen/geçersiz dosya başına
     **5 ceza puanı** (md 5.5.4.3.5).
Aynı mantığın iki dosyada iki kez yazılması = ikisinin ayrışması; buraya alındı.

---------------------------------------------------------------------------
🔴 BUNU DOĞURAN ARIZA (2026-08-06, Jetson'da KANITLI — tahmin değil)
Jetson ~15 saat BAYAT saatle açıldı (gerçek ~11:07, sistem saati 05.08 19:50).
O sabah çekilen 18 kare (kare_02587–02604) manifest'e **dünün tarihiyle
(05.08 21:34-21:36) ve `saat_guvenilir=1`** yazıldı; NTP saati ancak dakikalar
sonra düzeltti. Yani bayrak "güvenilir" derken saat yalan söylüyordu.

Sebep: eski ölçüt **mutlak eşikti** (`epoch >= 2026-01-01`). O eşik "RTC pilsiz
Jetson 1970'e/2 ay geriye düşer" senaryosu için tasarlanmıştı; **saatler
mertebesindeki geriliği tanım gereği göremez** — 05.08 19:50 de 2026'nın
içinde, eşiği rahatça geçiyor.

⇒ Doğru soru "tarih makul mü?" değil, **"bu saat bir referansa göre
DÜZELTİLDİ mi?"**. Bunun cevabı çekirdekte duruyor.

---------------------------------------------------------------------------
🔬 YÖNTEM: çekirdeğin kendi senkron bayrağı (`adjtimex`, birinci kaynak)
`adjtimex(2)` salt-okunur çağrısı (`modes=0`) **yetki istemez** ve çekirdeğin
saat disiplin durumunu verir:
  · dönüş `TIME_ERROR (5)` ya da `status & STA_UNSYNC` → saat **disipline
    EDİLMEMİŞ** (hiçbir NTP/PTP kaynağı saati düzeltmemiş)
  · ikisi de temizse → saat bir referansa göre düzeltilmiş
Bu ölçüt **daemon'dan bağımsız**: systemd-timesyncd de chrony de ntpd de
senkron olunca STA_UNSYNC'i temizler. `timedatectl` de zaten aynı çekirdek
bayrağını okur — ama o bir alt süreç açar; biz tek syscall ile alıyoruz.

⏳ ÖNEMLİ SINIR — bayrak SONSUZA KADAR "senkron" kalmaz (kaynak: çekirdek):
`kernel/time/ntp.c::second_overflow()` her saniye
`time_maxerror += MAXFREQ / NSEC_PER_USEC` yapıyor ve
`time_maxerror > NTP_PHASE_LIMIT` olunca `time_status |= STA_UNSYNC` diyor.
`include/linux/timex.h`: `MAXFREQ 500000` (ns/s) · `MAXPHASE 500000000L` (ns) ·
`NTP_PHASE_LIMIT ((MAXPHASE / NSEC_PER_USEC) << 5)`.
Hesap: 16.000.000 µs / 500 µs-per-s = **32.000 s ≈ 8,9 saat**.
⇒ Kıyıda senkron olup denize açılan Jetson, ağsız **~8,9 saat sonra** yeniden
"senkronsuz" damgalanır. Bu bizim için KABUL EDİLEBİLİR bir yanlış-negatif:
saat hâlâ ±saniyeler doğrudur ama biz "kanıtım yok" deriz. Oturum 8,9 saati
geçerse manifest'te bir kırılma görülür — panik değil, beklenen davranış.
(PC'de canlı doğrulandı: senkrondan 43 sn sonra maxerror = 21.500 µs,
 yani 500 µs/s artış birebir tutuyor.)

🔴 BİLİNEN YANLIŞ-NEGATİF: `sudo date -s '...'` ile saati ELLE ayarlamak
STA_UNSYNC'i **temizlemez** (çekirdeğe "bu saat disipline edildi" demez).
Deniz günü geçici önlemi tam da buydu. O yüzden insan doğrulaması ayrı bir
kanıt olarak kabul edilir: `--saat-elle-dogrulandi` bayrağı (toplayıcıda),
`elle_dogrulandi=True` (burada). Bayrak verilmezse kareler "güvenilmez"
damgalanır — veri KAYBOLMAZ, yalnız iddia edilmez.

📌 Neden "hepsini güvenilmez damgalamak" veriyi çöpe atmaz: kareler arası
GÖRELİ sıra bozulmuyor (dosya adı + manifest sırası), yalnız mutlak saat
iddiası düşüyor. Işık/saat çeşitliliği analizi için tek bir doğru çapa
(kıyıda not edilen gerçek saat) yeterlidir.
"""
import ctypes
import ctypes.util

# adjtimex(2) dönüş kodu: saat senkronize DEĞİL
TIME_ERROR = 5
# include/linux/timex.h — çekirdeğin "senkron değil" durum biti
STA_UNSYNC = 0x0040

# Mutlak alt sınır: RTC pilsiz Jetson boot'ta çok geriye düşebilir (bu projede
# ~2 ay ölçüldü). Çekirdek bayrağının YERİNE geçmez, YANINDA durur — ucuz bir
# akıl sağlığı sınırı.
SAAT_ALT_SINIR = 1767225600.0        # 2026-01-01T00:00:00Z


class _Timex(ctypes.Structure):
    """`struct timex` — yalnız `status` alanı okunuyor; gerisi hizalama için.

    Alan sırası ve tipleri `include/uapi/linux/timex.h` ile aynı olmalı, yoksa
    `status` yanlış ofsetten okunur. `long` genişliğini ctypes platforma göre
    ayarlar (Jetson aarch64 = 64 bit).
    """
    _fields_ = [
        ("modes", ctypes.c_uint), ("offset", ctypes.c_long),
        ("freq", ctypes.c_long), ("maxerror", ctypes.c_long),
        ("esterror", ctypes.c_long), ("status", ctypes.c_int),
        ("constant", ctypes.c_long), ("precision", ctypes.c_long),
        ("tolerance", ctypes.c_long), ("tv_sec", ctypes.c_long),
        ("tv_usec", ctypes.c_long), ("tick", ctypes.c_long),
        ("ppsfreq", ctypes.c_long), ("jitter", ctypes.c_long),
        ("shift", ctypes.c_int), ("stabil", ctypes.c_long),
        ("jitcnt", ctypes.c_long), ("calcnt", ctypes.c_long),
        ("errcnt", ctypes.c_long), ("stbcnt", ctypes.c_long),
        ("tai", ctypes.c_int), ("_pad", ctypes.c_int * 11),
    ]


def cekirdek_senkron_mu():
    """Çekirdek saati bir referansa göre DİSİPLİNE EDİLMİŞ mi?

    Returns:
        True  — senkron (STA_UNSYNC temiz)
        False — senkronsuz (hiç düzeltilmemiş ya da ~8,9 saattir düzeltilmemiş)
        None  — OKUNAMADI (libc yok / çağrı hatası). Ölçemediğimiz şey için
                alarm da üretmeyiz, "güvenilir" de demeyiz — çağıran karar verir.
    """
    try:
        libc = ctypes.CDLL(ctypes.util.find_library("c"), use_errno=True)
        t = _Timex()
        sonuc = libc.adjtimex(ctypes.byref(t))
        if sonuc < 0:
            return None
        return sonuc != TIME_ERROR and not (t.status & STA_UNSYNC)
    except Exception:
        return None


def saat_guvenilir_mi(epoch_s, alt_sinir=SAAT_ALT_SINIR,
                      senkron=None, elle_dogrulandi=False):
    """Bu zaman damgası KANITLANABİLİR biçimde doğru mu? (saf fonksiyon)

    Args:
        epoch_s: damgalanacak an (unix epoch, saniye).
        alt_sinir: mutlak akıl sağlığı sınırı.
        senkron: `cekirdek_senkron_mu()` sonucu (True/False/None).
            **None = bilinmiyor** → yalnız mutlak eşiğe bakılır (eski davranış;
            saf birim testleri makineden bağımsız kalsın diye varsayılan bu).
        elle_dogrulandi: insan kıyıda saati gözle doğruladıysa True. `date -s`
            çekirdek bayrağını temizlemediği için tek telafi yolu budur.

    Karar tablosu:
        mutlak eşiğin altı              -> False (her hâlde)
        senkron is False & elle değil   -> False  ← 06.08 arızasını yakalayan dal
        senkron is False & elle doğru   -> True
        senkron is True / None          -> True
    """
    if float(epoch_s) < alt_sinir:
        return False
    if senkron is False and not elle_dogrulandi:
        return False
    return True


def saat_raporu(senkron):
    """Journal/log için tek satırlık insan-okur durum metni."""
    if senkron is None:
        return ("saat senkron durumu OKUNAMADI (adjtimex) — damgalar mutlak "
                "eşiğe göre işaretlenecek")
    if senkron:
        return ("saat NTP/PTP ile senkron (çekirdek STA_UNSYNC temiz) — "
                "damgalar güvenilir; ağsız ~8,9 saat sonra bu bayrak düşer")
    return ("🔴 saat SENKRON DEĞİL — hiçbir referans saati düzeltmemiş. "
            "Zaman damgaları YANLIŞ olabilir (tarih makul görünse bile). "
            "Kıyıda düzelt: tethering ile NTP, ya da `sudo date -s '...'` + "
            "toplayıcıya --saat-elle-dogrulandi ver")
