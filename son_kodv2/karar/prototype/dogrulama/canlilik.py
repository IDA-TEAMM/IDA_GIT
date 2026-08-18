# -*- coding: utf-8 -*-
"""CANLILIK DEĞİŞMEZLERİ — "iyi bir şey olmalı", ama **SINIRLI SÜREDE**.

## 🔑 Neden hepsinde bir süre sınırı var

Klasik ayrım (Lamport) ve çalışma zamanı doğrulamasının temel kısıtı:

  · **Güvenlik** (safety): *"kötü bir şey ASLA olmasın"* — ihlal **sonlu**
    bir izde görülür, parmakla gösterilir. Kolayca izlenir. (F/S/B kuralları.)
  · **Canlılık** (liveness): *"iyi bir şey ER GEÇ olsun"* — sonlu bir izde
    **asla** ihlal edilmiş sayılamaz; her an "belki bir sonraki adımda olur"
    denebilir. ⇒ **İZLENEBİLİR DEĞİLDİR.**

Literatürün hükmü net: *"Yalnız **sınırlı** canlılık özellikleri çalışma
anında denetlenebilir — 'iyi bir şey **belirli sayıda adım içinde** olacak'."*

⇒ Bu yüzden buradaki hiçbir kural *"sonsuza kadar"* demiyor. Hepsinin bir
**süre sınırı** var ve o sınır — fizik eşiklerinde olduğu gibi — **türetiliyor**,
uydurulmuyor.

## Sınırların kaynağı

| kural | sınır | nereden |
|---|---|---|
| C1 itki | 3,0 s | ArduPilot komut kesme eşiği — o kadar süre komutsuzsak FC zaten durdurdu |
| C2 durum | 20 dk | şartname s.22 görev süresi — görevden uzun süre bir durumda kalmak saçma |
| C3 topic | periyot × tolerans | topic'in **kendi** nominal kadansı |
| C5 kapanış | 20 s | `girdap-algi.service` `TimeoutStopSec=20` |
"""
from __future__ import annotations

import math
from typing import Iterable, Optional, Sequence

from prototype.dogrulama.butce import ARDUPILOT_KOMUT_KESME_S
from prototype.dogrulama.kural import Kural, Tur

#: Şartname s.22 — görev süresi. C2'nin sınırı; bilerek CÖMERT.
#: (Şartname s.6'da "DSB" diyor, s.22'de 20 dk — çelişki kayıtlı, cömert olan alındı.)
GOREV_SURESI_S = 20 * 60.0

#: `girdap-algi.service` TimeoutStopSec — kapanışa tanınan süre.
KAPANIS_TAVANI_S = 20.0


# ───────────────────────── C1 — sessiz felç ─────────────────────────
def c1_itki_sifir_kalmasin(
    sifir_sureleri_s: float,
    tavan_s: float = ARDUPILOT_KOMUT_KESME_S,
) -> float:
    """Marj (s): itkinin KESİNTİSİZ sıfır kaldığı süreye kalan pay.

    KAR-04: `/girdap/control/thrust` incelenen **hiçbir** oturumda sıfırdan
    farklı olmadı — 21.000+ mesajın tamamı `[0.0, 0.0]`. Araç hiç tahrik
    komutu almadı ve bu ancak akşam bant analiziyle görüldü.

    Sınır neden 3,0 s: o kadar süre komut akmadıysa ArduPilot tekneyi zaten
    durdurmuş demektir (bkz. `butce.ARDUPILOT_KOMUT_KESME_S`). Yani bu sınır
    bizim seçimimiz değil, eyleyicinin davranışı.

    ⚠ Bu kural yalnız **ARM + GUIDED + görev aktif** iken anlamlıdır; çağıran
    kapıyı kendisi uygular (BEKLEMEDE'de sıfır itki DOĞRU davranıştır).
    """
    if sifir_sureleri_s < 0 or math.isnan(sifir_sureleri_s):
        return math.nan
    return tavan_s - sifir_sureleri_s


C1 = Kural(
    "C1", "ARM+GUIDED iken itki sınırlı süreden uzun sıfır kalamaz",
    kaynak="ArduPilot komut kesme eşiği 3,0 s; KAR-04'te 21.000+ mesajın TAMAMI [0,0]",
    birim="s", fn=c1_itki_sifir_kalmasin, tur=Tur.ABORT, olcek=0.50,
)


# ─────────────────────── C2 — durum makinesi ilerlemesi ───────────────────
def c2_durum_ilerlesin(
    ayni_durumda_sure_s: float,
    durum: Optional[str] = None,
    tavan_s: float = GOREV_SURESI_S,
    bekleyen_durumlar: Sequence[str] = ("BEKLEMEDE", "TAMAMLANDI", "KILL"),
) -> float:
    """Marj (s): aynı durumda kalınabilecek süreye pay.

    KAR-03: **25 dakika** BOOT'ta kalındı ve bu sürede 10 Hz komut yayınlamaya
    devam edildi. KAR-08: görev hiç PARKUR'a geçemedi.

    `bekleyen_durumlar` muaf: BEKLEMEDE'de saatlerce durmak DOĞRU davranıştır
    (YKİ komutu bekleniyor), TAMAMLANDI ve KILL de terminal durumlardır.
    Muafiyet olmadan kural her koşuda yanar ⇒ özgüllük sıfırlanır.
    """
    if durum is not None and durum.upper() in {d.upper() for d in bekleyen_durumlar}:
        return tavan_s          # muaf — tam pay
    if math.isnan(ayni_durumda_sure_s):
        return math.nan
    return tavan_s - ayni_durumda_sure_s


C2 = Kural(
    "C2", "Görev durumu görev süresinden uzun sabit kalamaz",
    kaynak="şartname s.22 görev süresi 20 dk; KAR-03'te 25 dk BOOT kilitlenmesi",
    birim="s", fn=c2_durum_ilerlesin, olcek=60.0,
)


# ─────────────────────── C3 — kritik topic akışı ───────────────────────
def c3_topic_akiyor(
    son_mesajdan_gecen_s: float,
    nominal_periyot_s: float,
    tolerans: float = 3.0,
) -> float:
    """Marj (s): topic'in **kendi** kadansından türeyen sessizlik payı.

    Sabit bir "5 saniye" eşiği yanlış olurdu: 50 Hz IMU için 5 s felaket,
    1 Hz görev yayını için normaldir. Sınır her topic'in kendi periyodundan
    çıkar (varsayılan 3 periyot).

    PAR-04: `/mavros/state` 0,17 Hz'e düştü (nominal 2 Hz) ⇒ oturumun %86'sı
    KILL. ALG-05: LiDAR 5 saatte 39 mesaj.
    """
    if not (nominal_periyot_s > 0.0):
        return math.nan
    return nominal_periyot_s * tolerans - son_mesajdan_gecen_s


C3 = Kural(
    "C3", "Kritik topic kendi kadansının toleransı içinde akmalı",
    kaynak="topic'in nominal periyodu × 3; PAR-04 (state 0,17 Hz) ve ALG-05 (LiDAR 39 mesaj/5 saat)",
    birim="s", fn=c3_topic_akiyor, olcek=0.50,
)


# ─────────────────────── C5 — temiz kapanış ───────────────────────
def c5_kapanis_temiz(
    kapanis_suresi_s: float,
    dosyalar_sonlandi: bool,
    tavan_s: float = KAPANIS_TAVANI_S,
) -> float:
    """Marj (s): kapanışa kalan pay. Dosya sonlanmadıysa doğrudan ihlal.

    PAR-10: 14 bag'in **13'ü** sonlandırılmamış. Aynı sınıf Dosya-1'i de
    vurur: mp4'ün `moov` atomu yazılmazsa dosya **oynatılamaz** ⇒ md 4.2
    gereği teslim edilmemiş sayılır ⇒ **5 ceza puanı**.
    """
    if not dosyalar_sonlandi:
        return -1.0
    if math.isnan(kapanis_suresi_s):
        return math.nan
    return tavan_s - kapanis_suresi_s


C5 = Kural(
    "C5", "Kapanış süre içinde ve dosyalar sonlandırılmış olmalı",
    kaynak="girdap-algi.service TimeoutStopSec=20; PAR-10'da 13/14 bag sonlandırılmamış",
    birim="s", fn=c5_kapanis_temiz, olcek=5.0,
)


KURALLAR = (C1, C2, C3, C5)
