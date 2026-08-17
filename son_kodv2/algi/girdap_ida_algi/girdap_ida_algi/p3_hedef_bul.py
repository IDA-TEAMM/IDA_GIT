# ============================================================================
# 🔴🔴 BU DOSYA BİR **KOPYA**.
#   kanonik kaynak : girdap-ida-p3/p3_hedef/hedef_bul.py
#   kaynak sha256  : 191d29466097e0ba334e2f612f575a2962daae960f5dc5fc1d639dac8930dd6b
#   kopyalandı     : 2026-08-17
#
# NEDEN KOPYA (ve neden `import p3_hedef` DEĞİL):
#  · Algı node'u Jetson'da systemd ile açılıyor; `girdap-ida-p3` ayrı bir repo
#    ve o makinede kurulu bir paket değil. `sys.path`'e elle dizin eklemek
#    "kurulum adımı unutulursa P3 sessizce ölür" demek olurdu — bu zincirde
#    tam bu sınıftan üç arıza yaşandı (blob yolu · WorkingDirectory yokluğu ·
#    `girdap-algi` disabled). Hepsi belirtisizdi.
#  · Yarışma alanında internet/pip YOK (md 4.1) ⇒ tek `git pull` ile gelen kod
#    çalışmak zorunda.
#  · Aynı desen karar reposunda zaten var: `prototype/mission/renk_kodu.py`
#    kanonik kaynağın kopyasıdır ve ayrışma denetimiyle korunur.
#
# 🔴 DEĞİŞİKLİK BURAYA YAPILMAZ — kanonik dosyada yapılır, sonra bu dosya
#    yeniden kopyalanır ve yukarıdaki sha256 güncellenir.
#    `test_p3_hedef_bul_kopya.py` ayrışmayı yakalar (kanonik repo bu makinede
#    yoksa test SKIP eder — sessizce yeşil vermez).
# ============================================================================
"""PARKUR-3 hedef dubası — **saf OpenCV** tespiti (İDA tarafı).

Neden burada: algı bizde. Karar reposundaki `prototype/perception/camera_buoys.py`
Sprint-2'de, ortada eğitilmiş model yokken yazılmış bir yedek; üretimde **kapalı**
(`use_onboard_camera=false`) ve tek OAK'ı biz tuttuğumuz için o node'a kare zaten
ulaşmıyor. O dosya SİLİNMEDİ (ortak alan kuralı: başkasının koduna habersiz
dokunulmaz) — buraya **kodu değil, ölçülmüş bilgisini** taşıdık:

  · CLAHE ön işleme (su parlaması dengesi)
  · **Doygunluk germesi** — F-P.21, 2026-07-16 GERÇEK DONANIM testi: akşamüstü/
    bulutlu ışıkta gerçek dubanın doygunluğu **S≈29-83** ölçüldü, sabit eşik 120'nin
    çok altında ⇒ **hiç tespit edilemedi**. CLAHE yetmedi (S 75→84); global
    yüzdelik germe S 75→255 yaptı. Bu tuzak bizim için de geçerli.
  · Kırmızı OpenCV'de **iki aralık** ister (H=0/179 sarmalı).
  · Sahnede zaten doygun renk varsa germe UYGULANMAZ (yoksa doğal doygunluk
    farkları bozulur).

🔴 EKLENEN: **SİYAH (RAL 9005)**. Karar tarafının dosyasında siyah sınıfı YOK
(3=kırmızı, 4=yeşil, 5=kahverengi) ⇒ şartnamenin üç hedef renginden biri
görülemiyordu. Siyah **mutlak** V eşiğiyle bulunamaz (eşik zemine bağımlıdır);
İHA plaka modülünün ölçülmüş deseni kullanılıyor: **koyuluk oranı** — adayın
parlaklığı, çevresindeki suya GÖRE ne kadar koyu.

Şartname s.18 hedef renkleri: RAL 3026 (kırmızı) · RAL 6037 (yeşil) · RAL 9005 (siyah).
Sayısal kod sözleşmesi `renk_kodu.py` ile AYNI: 1=kırmızı · 2=yeşil · 3=siyah.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Optional

import cv2
import numpy as np

KOD = {"kirmizi": 1, "yesil": 2, "siyah": 3}


@dataclass
class Ayar:
    """HSV OpenCV ölçeğinde: H∈[0,179], S/V∈[0,255]."""

    # RAL 3026 floresan kırmızı — iki aralık (H sarmalı)
    kirmizi1_lo: tuple = (0, 120, 70)
    kirmizi1_hi: tuple = (8, 255, 255)
    kirmizi2_lo: tuple = (170, 120, 70)
    kirmizi2_hi: tuple = (179, 255, 255)
    # RAL 6037 saf yeşil
    yesil_lo: tuple = (40, 80, 60)
    yesil_hi: tuple = (85, 255, 255)
    # RAL 9005 siyah — mutlak eşik DEĞİL, çevreye göre koyuluk (aşağıya bak)
    siyah_s_max: int = 90
    siyah_v_max: int = 90
    #: Aday, çevresindeki suyun parlaklığının en fazla bu katı olabilir.
    #: 🔑 Mutlak eşik zemine bağımlı: aynı siyah duba parlak suda V=40,
    #: gölgeli suda V=85 okur. Oran zemini böler, sabit kalır (İHA plaka
    #: modülünde aynı desen ölçümle doğrulandı).
    siyah_koyuluk_orani: float = 0.55

    # ── ŞEKİL KAPILARI — RENGE GÖRE AYRI, 16.08.2026'da ÖLÇÜLDÜ ──────────
    #: 🪤 İlk sürümde İHA plaka modülünün eşikleri (doluluk 0,72) kopyalanmıştı
    #: ve hedefin **%81'ini eliyordu**: plaka DÜZ DİKDÖRTGEN, duba ise yuvarlak
    #: ve su hattı gövdeyi kesiyor ⇒ doluluk doğal olarak 0,5-0,7 (kusursuz
    #: dairenin teorik tavanı bile 0,785). Eşik ölçümle yeniden oturtuldu.
    #:
    #: SÜPÜRME (600 sentetik hedef karesi ↔ 110 hedefsiz gerçek kare):
    #:   kırmızı  dol 0,45 → **%91 bulma / 0,12 yanlış**  · 0,55 → %77/0,08
    #:   yeşil    dol 0,45 → %95/0,58 · **0,55 → %88/0,30** · 0,62 → %73/0,10
    #:   siyah    dol 0,45 → %86/1,81 · 0,55 → %73/0,51 · **0,62+alan500 → %52/0,03**
    #: 🔑 Renkler ayrı davranıyor: yeşilin yanlış alarmı kıyı bitkisinden,
    #: siyahınki gölge ve koyu sudan. Kırmızı en temizi (sahnede karşılığı yok).
    #: 🔴 SİYAH bilerek MUHAFAZAKÂR: yanlış kilit kalıcıdır ve yanlış temas
    #: TS3'te 100→50 puan; kaçırmanın bedeli ise zamansal onayla telafi
    #: edilebilir (3 ardışık kare gerekiyor). Gölge SÜREKLİ olduğu için
    #: yüksek yanlış alarm zamansal onaydan da GEÇER — asıl tehlike bu.
    #: ⚠️ Siyah eğrisi **kendi ürettiğim sentetik** siyahtan çıktı (V×0,20);
    #: gerçek RAL 9005 yüzeyinde doğrulanmadı — fiziksel duba gelince yenilenmeli.
    kapi: dict = field(default_factory=lambda: {
        "kirmizi": dict(min_alan=300, doluluk=0.45, en_boy=2.4),
        "yesil":   dict(min_alan=300, doluluk=0.55, en_boy=2.4),
        "siyah":   dict(min_alan=500, doluluk=0.62, en_boy=2.4),
    })

    clahe_clip: float = 2.0
    clahe_kare: int = 8
    germe_alt_yuzde: float = 1.0
    germe_ust_yuzde: float = 99.0
    #: Sahnede zaten doygun renk varsa germe yapma (doğal farkları bozar).
    germe_atla_ustu: float = 150.0
    morf_px: int = 3


@dataclass
class Aday:
    renk: str
    cx: float
    cy: float
    w: float
    h: float
    alan: float
    doluluk: float
    en_boy: float

    @property
    def kod(self) -> int:
        return KOD[self.renk]


def _clahe(bgr, a: Ayar):
    lab = cv2.cvtColor(bgr, cv2.COLOR_BGR2LAB)
    c = cv2.createCLAHE(clipLimit=a.clahe_clip, tileGridSize=(a.clahe_kare, a.clahe_kare))
    lab[:, :, 0] = c.apply(lab[:, :, 0])
    return cv2.cvtColor(lab, cv2.COLOR_LAB2BGR)


def _doygunluk_germe(hsv, a: Ayar):
    """Düşük ışıkta doygunluğu ger — yoksa sabit eşikler hiç tetiklenmez."""
    s = hsv[:, :, 1]
    ust = float(np.percentile(s, a.germe_ust_yuzde))
    if ust >= a.germe_atla_ustu:           # sahnede zaten doygun renk var
        return hsv
    alt = float(np.percentile(s, a.germe_alt_yuzde))
    if ust - alt < 1e-3:
        return hsv
    hsv = hsv.copy()
    hsv[:, :, 1] = np.clip((s.astype(np.float32) - alt) * 255.0 / (ust - alt), 0, 255)
    return hsv


def _maske(hsv, lo, hi, morf):
    m = cv2.inRange(hsv, np.array(lo, np.uint8), np.array(hi, np.uint8))
    k = np.ones((morf, morf), np.uint8)
    m = cv2.morphologyEx(m, cv2.MORPH_OPEN, k)
    return cv2.morphologyEx(m, cv2.MORPH_CLOSE, k)


def _siyah_maske(hsv, a: Ayar):
    """Koyu + doygunsuz **ve** çevresine göre belirgin koyu bölgeler.

    Mutlak eşik tek başına yetmez: gölgeli su da koyudur. Sahne referansı
    olarak kareyi alt yarısının (su) medyan parlaklığı alınır.
    """
    s, v = hsv[:, :, 1], hsv[:, :, 2]
    su_v = float(np.median(v[v.shape[0] // 2:, :]))
    tavan = min(a.siyah_v_max, max(12.0, su_v * a.siyah_koyuluk_orani))
    m = ((s <= a.siyah_s_max) & (v <= tavan)).astype(np.uint8) * 255
    k = np.ones((a.morf_px, a.morf_px), np.uint8)
    m = cv2.morphologyEx(m, cv2.MORPH_OPEN, k)
    return cv2.morphologyEx(m, cv2.MORPH_CLOSE, k)


def _adaylar(maske, renk, a: Ayar):
    out = []
    k = a.kapi[renk]
    cnt, _ = cv2.findContours(maske, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    for c in cnt:
        alan = cv2.contourArea(c)
        if alan < k["min_alan"]:
            continue
        (_, _), (w, h), _ = cv2.minAreaRect(c)
        if w < 1 or h < 1:
            continue
        doluluk = alan / (w * h)
        if doluluk < k["doluluk"]:
            continue
        en_boy = max(w, h) / min(w, h)
        if en_boy > k["en_boy"]:
            continue
        x, y, bw, bh = cv2.boundingRect(c)
        out.append(Aday(renk, x + bw / 2.0, y + bh / 2.0, float(bw), float(bh),
                        float(alan), float(doluluk), float(en_boy)))
    return out


def _ortusuyor(a: "Aday", kutu) -> bool:
    """Aday ile veto kutusu (cx, cy, w, h — piksel) çakışıyor mu?"""
    bx, by, bw, bh = kutu
    return (abs(bx - a.cx) < max(bw, a.w) and
            abs(by - a.cy) < max(bh, a.h) * 1.5)


def hedef_bul(bgr, ayar: Optional[Ayar] = None, veto_kutulari=None) -> list:
    """Karedeki P3 hedef dubası adayları (kırmızı/yeşil/siyah).

    `veto_kutulari`: YOLO'nun **kendi dubalarımız** için verdiği kutular
    (cx, cy, w, h — piksel). Örtüşen aday ELENİR.

    🔑 NEDEN (16.08.2026, ÖLÇÜLDÜ). Stereo yokken boyut kapısı işlevsizdir:
    menzil Ø0,64 varsayılarak hesaplandığı için yayınlanan çap **daireseldir**,
    `cap_makul_mu` her adayı geçirir. Geriye tek ayırt edici renk kalıyor ve
    KIRMIZI ayırmıyor — gölgedeki kendi turuncu kenar dubamız RAL 3026 ile
    aynı bantta okunuyor (ölçüm: eski boyada %32,4, doğru RAL 2003'te %5,0).
    Uçtan uca simülasyonda bedeli görüldü: hedefsiz gerçek dizilerde hakem
    *"kırmızı"* deseydi **6 bloğun 2'sinde YANLIŞ KİLİT** kuruluyordu
    (yeşil/siyahta 0). Yanlış temas TS3'te 100 → 50 puan.

    Çözüm ölçümle bulundu: gerçek karelerdeki 22 kırmızı adayın **21'i (%95)**
    YOLO kutusuyla örtüşüyor — yani onlar zaten BİZİM dubalarımız. Ve yeni
    model P3 hedefini **görmemeyi** öğrendiği (etiketsiz P3 negatifleri) için
    gerçek hedef bu vetoya takılmaz. İki dedektör birbirini tamamlıyor:
    YOLO *"bizim duba"*yı bilir, OpenCV hedef rengini bulur.

    Boş/geçersiz girişte boş liste — tanı kodu görevi öldürmez.
    """
    a = ayar or Ayar()
    if bgr is None or getattr(bgr, "size", 0) == 0:
        return []
    hsv = cv2.cvtColor(_clahe(bgr, a), cv2.COLOR_BGR2HSV)
    hsv = _doygunluk_germe(hsv, a)
    kir = cv2.bitwise_or(_maske(hsv, a.kirmizi1_lo, a.kirmizi1_hi, a.morf_px),
                         _maske(hsv, a.kirmizi2_lo, a.kirmizi2_hi, a.morf_px))
    yes = _maske(hsv, a.yesil_lo, a.yesil_hi, a.morf_px)
    siy = _siyah_maske(hsv, a)
    adaylar = (_adaylar(kir, "kirmizi", a) + _adaylar(yes, "yesil", a)
               + _adaylar(siy, "siyah", a))
    if veto_kutulari:
        adaylar = [d for d in adaylar
                   if not any(_ortusuyor(d, k) for k in veto_kutulari)]
    return adaylar


# ── PARKUR-3 KAPISI (16.08.2026, Eyüp kararı) ─────────────────────────────
#: Hedef tespiti YALNIZ bu durumda koşar.
P3_DURUMU = "PARKUR3"


def hedef_bul_p3(bgr, mission_state: str, ayar: Optional[Ayar] = None,
                 veto_kutulari=None) -> list:
    """PARKUR-3 dışında **hiç çalışmaz** — boş liste döner.

    🔑 NEDEN KAPI VAR:
      · P1/P2'de hedef tespiti **anlamsız**: `nisan_hedefi` zaten PARKUR3 dışında
        `None` döndürüyor ⇒ hesap yapılıp atılıyordu. CPU bedava değil
        (Jetson'da 15-23 ms/kare ölçülmüştü).
      · Yüzey daralır: koşmayan kod yanlış pozitif üretemez.

    🔴 KARIŞTIRILMAMASI GEREKEN ŞEY: **büyük cisim süzgeci kapıya TABİ DEĞİL.**
    O, algı tarafında `/perception/buoys`'u korur ve **her zaman açıktır** —
    çünkü Ø64 cm hedef 25 m'den 7,7 px eder, yani tekne daha **Parkur-2'nin
    içindeyken** hedefi görür; süzülmezse füzyon `EdgeBuoyMemory`'ye KALICI
    kenar kaydı açar (orada unutma yok) → hayalet kapı → P2 rotası bozulur.
    (Eyüp'ün gözlemi: *"parkurlar tek yön değil, P1'deyken P3'ün dubasını da
    görebiliriz."*)

    ⇒ Kapı YALNIZ hedef tespiti/nişan zincirine uygulanır.
    """
    if (mission_state or "").strip().upper() != P3_DURUMU:
        return []
    return hedef_bul(bgr, ayar, veto_kutulari)
