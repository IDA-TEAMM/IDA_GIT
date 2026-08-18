# -*- coding: utf-8 -*-
"""SÖZLEŞME DEĞİŞMEZLERİ — katmanların birbirine verdiği söz.

Dayanak ROS'un kendi standartları:
  · **REP 103** — SI birimleri, sağ el kuralı, x ileri / z yukarı
  · **REP 105** — `base_link` ⊂ `odom` ⊂ `map` ⊂ `earth` ve ÇERÇEVE GARANTİLERİ:

        `odom` : sınırsız SÜRÜKLENEBİLİR ama **süreklidir** —
                 *"pose … always evolves in a smooth way, without discrete jumps"*
        `map`  : belirgin sürüklenmez ama **sıçrayabilir**
                 (küresel düzeltme geldiğinde ayrık atlama normaldir)

İkisi **zıt** garanti veriyor; hangisini seçtiğin tüketicinin ne
varsayabileceğini belirler.

🔴 BULUNAN SÖZLEŞME BOŞLUĞU (18.08.2026)
`/girdap/fusion/odom` adı `odom` ama **`frame_id="map"`** ile yayınlanıyor
(`fusion_node.py:677`). Yani yayıncı *"sıçrayabilirim"* diyor. Ama tüketiciler
**süreklilik** varsayıyor:
  · `planning_node._poz_damgada` iki poz örneği arasında **interpolasyon**
    yapıyor — ayrık sıçrama üzerinden interpolasyon anlamsızdır
  · geçit geçme doğrulaması pencere boyunca çizgi kuruyor
⇒ İki yoldan biri seçilmeli: ya çerçeve `odom` olmalı (süreklilik garanti
edilir, sıçrama gerçekten arıza olur), ya tüketiciler sıçramayı **algılayıp
interpolasyonu kesmeli**. Bugün ikisi de yapılmıyor.
Bu bir RAPOR maddesidir; kod burada değiştirilmez.
"""
from __future__ import annotations

import math
from typing import Iterable, Optional, Sequence

from prototype.dogrulama.kural import Kural, Tur

#: REP-105 çerçeve garantileri: süreklilik sözü veren çerçeveler.
SUREKLI_CERCEVELER = frozenset({"odom", "base_link"})
#: Ayrık sıçramaya izin veren çerçeveler.
SICRAYABILIR_CERCEVELER = frozenset({"map", "earth"})

#: `/perception/buoys` sınıf kimliği sözleşmesi (camera_buoys.py).
GECERLI_SINIFLAR = frozenset({0, 1, 2, 3, 4, 5, 99})

#: Tüketicinin bbox'ı normalize ettiği piksel uzayı (hardware/params.yaml).
BBOX_UZAYI = (1280, 720)


# ────────────────────────── S1 — damga yaşı ──────────────────────────
def s1_damga_yasi(yas_s: float, tavan_s: float = 2.0) -> float:
    """Marj (s): kabul edilebilir yaşa kalan pay.

    İki yönlü: gelecekten damga (yaş < 0) da ihlaldir — ALG-06'da damgalar
    **56 yıl** bayattı; saat sıçraması her iki yöne de gidebilir.
    Tavan 2,0 s: ölçülen p95 0,233 s'nin ~8 katı (`DAMGA_MAKUL_TAVAN_S`).
    """
    if math.isnan(yas_s):
        return math.nan
    return min(yas_s, tavan_s - yas_s)


S1 = Kural("S1", "Damga yaşı [0, tavan] aralığında olmalı",
           kaynak="ölçüm: çekim→yayın p95 0,233 s ⇒ tavan 2,0 s (8× pay); ALG-06'da 56 yıl bayattı",
           birim="s", fn=s1_damga_yasi, olcek=0.20)


# ──────────────────────── S2 — alan sözleşmesi ────────────────────────
def s2_sinif_gecerli(sinif_idleri: Iterable) -> float:
    """Marj: sözleşme dışı sınıf sayısı (negatif). Hepsi geçerliyse +1."""
    liste = list(sinif_idleri)
    if not liste:
        return 1.0
    disarda = 0
    for s in liste:
        try:
            if int(s) not in GECERLI_SINIFLAR:
                disarda += 1
        except (TypeError, ValueError):
            disarda += 1
    return 1.0 if disarda == 0 else -float(disarda)


def s2_bbox_uzayi(genislik: int, yukseklik: int) -> float:
    """Marj: tüketicinin beklediği uzayla birebir mi (+1 / −1).

    E-1 regresyonu: bbox 640×480 yayınlanıp 1280×720 varsayılınca karenin
    sağ %75'indeki her duba bearing toleransını aşıyordu — **hiçbir hata
    basılmadan** P1+P2 sıfırlanıyordu.
    """
    return 1.0 if (int(genislik), int(yukseklik)) == BBOX_UZAYI else -1.0


S2 = Kural("S2", "class_id sözleşmedeki kümede olmalı",
           kaynak="camera_buoys.py sınıf kimliği sözleşmesi (0,1,2,3,4,5,99)",
           birim="—", fn=s2_sinif_gecerli)

S2B = Kural("S2B", "bbox piksel uzayı tüketiciyle aynı olmalı",
            kaynak="E-1 regresyonu: 640×480 ↔ 1280×720 uyuşmazlığı P1+P2'yi sessizce sıfırlamıştı",
            birim="—", fn=s2_bbox_uzayi, tur=Tur.ABORT)


# ───────────────────────── S3 — QoS uyumluluğu ─────────────────────────
#: DDS kuralı: abone yayıncıdan KATI olamaz.
_REL = {"BEST_EFFORT": 0, "RELIABLE": 1}
_DUR = {"VOLATILE": 0, "TRANSIENT_LOCAL": 1}


def s3_qos_uyumu(yayinci: tuple, abone: tuple) -> float:
    """Marj: uyumluysa +1, uyumsuzsa −1 (bağlantı HİÇ kurulmaz, hata basılmaz).

    `yayinci`/`abone`: ("RELIABLE"|"BEST_EFFORT", "VOLATILE"|"TRANSIENT_LOCAL")
    """
    try:
        pr, pd = _REL[yayinci[0]], _DUR[yayinci[1]]
        sr, sd = _REL[abone[0]], _DUR[abone[1]]
    except (KeyError, IndexError, TypeError):
        return math.nan
    return -1.0 if (sr > pr or sd > pd) else 1.0


S3 = Kural("S3", "Abone QoS'u yayıncıdan katı olamaz",
           kaynak="DDS uyumluluk kuralı — uyumsuzlukta bağlantı kurulmaz ve HATA BASILMAZ",
           birim="—", fn=s3_qos_uyumu, tur=Tur.ABORT)


# ──────────────────── S5 — çerçeve sözleşmesi (REP-105) ────────────────────
def s5_cerceve_beklenen(gelen: Optional[str], beklenen: Sequence[str]) -> float:
    """Marj: beklenen çerçeve kümesindeyse +1, değilse −1.

    03.08 canlı hatası: `/perception/obstacle_map` `base_link` (GÖVDE) ile
    yayınlanıyordu ama planlama koordinatları **dünya** sanıp olduğu gibi
    kullanıyordu. Araç origin'de ve ψ=0 iken tesadüfen doğru, başka her
    durumda engeller yanlış yere düşüyordu.
    """
    if not gelen:
        return math.nan
    return 1.0 if gelen in set(beklenen) else -1.0


def s5_sureklilik_sozu(cerceve: Optional[str], interpolasyon_yapiliyor: bool) -> float:
    """Marj: tüketici interpolasyon yapıyorsa çerçeve SÜREKLİLİK sözü vermeli.

    REP-105: `odom` sürekli (sıçramaz), `map` sıçrayabilir. Sıçrayabilen bir
    çerçevede iki örnek arasında interpolasyon yapmak, atlamanın ortasında
    var olmayan bir poz uydurmaktır.
    """
    if cerceve is None:
        return math.nan
    if not interpolasyon_yapiliyor:
        return 1.0
    return 1.0 if cerceve in SUREKLI_CERCEVELER else -1.0


S5 = Kural("S5", "frame_id beklenen çerçeve kümesinde olmalı",
           kaynak="REP-105 çerçeve ağacı; 03.08 canlı hatası (gövde↔dünya karışması)",
           birim="—", fn=s5_cerceve_beklenen, tur=Tur.ABORT)

S5C = Kural("S5C", "İnterpolasyon yapan tüketici SÜREKLİ çerçeve ister",
            kaynak="REP-105: odom sürekli (sıçramaz) ↔ map sıçrayabilir",
            birim="—", fn=s5_sureklilik_sozu)


KURALLAR = (S1, S2, S2B, S3, S5, S5C)
