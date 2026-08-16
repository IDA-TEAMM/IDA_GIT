#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""OAK-D bağlantı dayanıklılığı — USB2'ye zorlama, kilit kurtarma, termal denetim.

NEDEN VAR (2026-08-05'te bu Jetson'da ÖLÇÜLDÜ, tahmin değil)
------------------------------------------------------------
1) **USB3 linki bu platformda kararsız.** Cihaz firmware'i yükleyip
   SuperSpeed'e (`03e7:f63b`, bus2 5000M) geçiyor, ardından link
   `tegra-xusb`'un U1/U2 güç durumu pazarlığında dağılıyor ve cihaz ROM'a
   düşüyor → `X_LINK_DEVICE_NOT_FOUND`. Kernel logu 2 saatte:
       100× "Disable of device-initiated U1 failed."
       100× "Disable of device-initiated U2 failed."
       110× "Device not responding to setup address."  (+ error -71)
   Hataların TAMAMI SuperSpeed yolunda; high-speed yolunda SIFIR hata.
   NVIDIA forumunda aynı platformda aynı belirti kayıtlı, Luxonis suçu
   `xusb-tegra`'ya atıyor (konu çözümsüz).

   ⇒ ÇÖZÜM: `maxUsbSpeed=HIGH`. Ölçüm: **5/5 açılış** (otomatik pazarlıkta
   ~6 denemede 1). Bant genişliği kaybı YOK — deploy akışı ~15 MB/s,
   USB2 pratik tavanı ~35-40 MB/s.

2) **Kilit yazılımdan açılabiliyor.** `/etc/udev/rules.d/80-movidius.rules`
   `MODE="0666"` verdiği için USB düğümüne sudo'suz `USBDEVFS_RESET`
   atılabiliyor; cihaz 0,5 sn'de geri geliyor. Bu, "teknede kameraya fiziksel
   erişim olmayacak" açık riskinin tek yazılımsal çaresi.

3) **Otomatik termal kısma YOK.** Luxonis dokümanı kısmadan bahsetmiyor;
   cihaz yavaşlamıyor, sıcaklık yükselince doğrudan ÇÖKÜYOR (çip anma sınırı
   105 °C, gözlenen çökme 125 °C). OAK-D **Lite** küçük soğutuculu, azami
   ortam ~40 °C. Güvenlik ağını BİZ kurmak zorundayız → `sicaklik_durumu()`.

Sürümden bağımsız: v2 `dai.Device(pipeline, hiz)`, v3 `dai.Device(hiz)` +
`dai.Pipeline(dev)` şeklinde açar. Bu yüzden modül açılışı kendisi yapmaz,
çağıranın verdiği `acici()` fonksiyonunu dayanıklı biçimde ÇAĞIRIR.
"""
import fcntl
import glob
import os
import time

# _IO('U', 20) — Linux usbdevfs port reset ioctl'i
USBDEVFS_RESET = ord("U") << 8 | 20

OAK_VENDOR = "03e7"

# Termal eşikler (°C). Çip anma sınırı 105, gözlenen çökme 125.
# 11 FPS'te iç mekânda ölçülen plato ~69 °C → 85 uyarı, 95 kritik makul pay bırakır.
SICAKLIK_UYARI = 85.0
SICAKLIK_KRITIK = 95.0


# ───────────────────────────────── USB düğümü ─────────────────────────────
def usb_dugum_yolu():
    """Takılı OAK'ın `/dev/bus/usb/BBB/DDD` yolu; yoksa None.

    Cihaz ROM modunda (`2485`) ya da bootlanmış (`f63b`) olabilir — ikisi de
    aynı vendor id'yi taşır, ayrım gerekmez.
    """
    for d in glob.glob("/sys/bus/usb/devices/*/"):
        try:
            if open(d + "idVendor").read().strip() != OAK_VENDOR:
                continue
            bus = int(open(d + "busnum").read())
            dev = int(open(d + "devnum").read())
            return f"/dev/bus/usb/{bus:03d}/{dev:03d}"
        except (OSError, ValueError):
            continue
    return None


def usb_reset(oturma_payi=2.5, zaman_asimi=10.0):
    """OAK'a USBDEVFS_RESET gönder (fişi çıkar-tak'ın yazılımsal karşılığı).

    sudo GEREKMEZ — 80-movidius.rules düğüme 0666 veriyor. Kural yoksa
    PermissionError yerine sessizce False döner (çağıran yine de deneyebilsin).

    Returns: reset gönderildi ve cihaz USB'de geri göründüyse True.
    """
    yol = usb_dugum_yolu()
    if not yol:
        return False
    try:
        fd = os.open(yol, os.O_WRONLY)
    except OSError:
        return False
    try:
        fcntl.ioctl(fd, USBDEVFS_RESET, 0)
    except OSError:
        return False
    finally:
        os.close(fd)

    bas = time.monotonic()          # SÜRE: duvar saati sıçramasından etkilenmesin
    while time.monotonic() - bas < zaman_asimi:
        if usb_dugum_yolu():
            time.sleep(oturma_payi)     # enumerasyon otursun
            return True
        time.sleep(0.2)
    return False


# ─────────────────────────────── dayanıklı açılış ─────────────────────────
def dayanikli_ac(acici, deneme=4, kaydet=None, bekleme=1.0):
    """`acici()` ile cihazı aç; X_LINK kilidinde USB reset atıp yeniden dene.

    Args:
        acici:  argümansız çağrılabilir → `dai.Device` döndürür.
                Sürüm farkını çağıran kapatır:
                    v2: lambda: dai.Device(pipeline, dai.UsbSpeed.HIGH)
                    v3: lambda: dai.Device(dai.UsbSpeed.HIGH)
        deneme: toplam açılış denemesi (ilk deneme dâhil).
        kaydet: opsiyonel log fonksiyonu (ROS logger / print).
        bekleme: reset sonrası tekrar denemeden önceki bekleme (sn).

    Returns: açılmış cihaz.
    Raises:  son denemenin hatası (deneme tükenirse).
    """
    def _log(m):
        if kaydet:
            kaydet(m)

    # 🔴 F-A.4 (15.08.2026) — CİHAZ YOKKEN ÇÖKMEK ÇÖZÜM DEĞİL, BEKLEMEK ÇÖZÜM.
    #
    # Ölçüldü (göl günü): kamera USB'den düşünce bu fonksiyon 4 denemeyi
    # tüketip `raise` ediyordu → düğüm ölüyor → ROS launch `respawn` ediyor →
    # ~30 saniyede bir yeniden çöküyor. Bir saat boyunca ONLARCA kez
    # döndü (`process has died [exit code 1]` günlüğü dolu). Her tur depthai'yi
    # sıfırdan kuruyor; yığın zaten işlemci bütçesini aşmışken (§1.11c,
    # load 9,31/6 çekirdek) bu **boşuna yakılan işlemci**.
    #
    # 🔑 AYRIM: "cihaz USB'de YOK" bir ARIZA değil, beklenecek bir DURUMDUR
    # (kaptan kabloyu henüz takmamıştır). "Cihaz VAR ama açılmıyor" ise gerçek
    # arızadır — X_LINK kilidi; orada eski davranış (USB reset + 4 deneme +
    # `raise`) aynen korunur, çünkü sessizce beklemek arızayı GİZLERDİ.
    #
    # Kaptan: *"kamera nodu otomatik başlamıyor galiba çıkarıp atınca…
    # otomatik her şey başlamalı."* Bu düzeltmeden sonra düğüm ölmez: cihaz
    # takıldığı anda kendi yakalar.
    son_hata = None
    i = 0
    while i < deneme:
        try:
            return acici()
        except Exception as e:                       # RuntimeError + X_LINK türevleri
            son_hata = e
            i += 1
            _log(f"OAK açılamadı ({i}/{deneme}): {type(e).__name__}: {e}")
            if i < deneme:
                ok = usb_reset()
                _log("USB reset gönderildi, tekrar deneniyor"
                     if ok else "USB reset BAŞARISIZ (cihaz USB'de yok?)")
                time.sleep(bekleme)
    raise son_hata


def cihazi_bekle(kaydet=None, aralik=2.0, bildirim_araligi=30.0):
    """OAK USB'de görünene kadar BEKLE (süresiz). Çökmeden, sessiz kalmadan.

    Neden süresiz: teknede kameraya fiziksel erişim koşum sırasında yok; bir
    zaman aşımı koyup çökmek, kaptanın kabloyu takmasını beklemekten daha
    kötüdür (§F-A.4). Yığın ayakta kalır, kamera gelince kendi devam eder.

    Neden `aralik=2.0`: yoklama ucuz (`/sys` okuması) ama boşuna sık dönmenin
    anlamı yok — cihaz insan eliyle takılıyor. `bildirim_araligi` ile 30
    saniyede bir tek satır basılır; **sessizlik başarı değildir** kuralı
    (canlı nöbetçi tasarımıyla aynı ilke) burada da geçerli.
    """
    def _log(m):
        if kaydet:
            kaydet(m)

    t0 = time.monotonic()
    son_bildirim = -bildirim_araligi
    while usb_dugum_yolu() is None:
        gecen = time.monotonic() - t0
        if gecen - son_bildirim >= bildirim_araligi:
            son_bildirim = gecen
            _log(f"OAK kamera USB'de YOK — bekleniyor ({gecen:.0f} s). "
                 "Düğüm ayakta, cihaz takılınca kendi devam edecek "
                 "(F-A.4: çökme döngüsü yok)")
        time.sleep(aralik)
    if time.monotonic() - t0 > aralik:
        _log(f"OAK kamera USB'de göründü ({time.monotonic() - t0:.0f} s bekledi)")


# ───────────────────────────────── termal denetim ─────────────────────────
def sicaklik_durumu(c, uyari=SICAKLIK_UYARI, kritik=SICAKLIK_KRITIK):
    """VPU sıcaklığını sınıflandır: 'normal' | 'uyari' | 'kritik'.

    Cihazda otomatik kısma OLMADIĞI için bu denetimi uygulama yapmak zorunda.
    `c` None ise (okuma başarısız) 'normal' döner — ölçemediğimiz şey için
    alarm üretmeyiz, ama çağıran bunu loglamalı.
    """
    if c is None:
        return "normal"
    if c >= kritik:
        return "kritik"
    if c >= uyari:
        return "uyari"
    return "normal"


def vpu_sicakligi(dev):
    """Cihazdan VPU ortalama sıcaklığını oku; okunamazsa None."""
    try:
        return float(dev.getChipTemperature().average)
    except Exception:
        return None
