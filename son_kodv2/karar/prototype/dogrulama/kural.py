# -*- coding: utf-8 -*-
"""KURAL MOTORU — nicel (marjlı) çalışma zamanı doğrulama çekirdeği.

## Neden ikili değil MARJ

Bir kuralın "geçti/kaldı" demesi yetmiyor. Gövde payı **+0,03 m** ile
**+0,62 m** ikili bir kuralda ikisi de "geçti"dir; oysa biri kıl payı
kurtulmuştur ve bir sonraki dalgada çarpar. Sinyal Zamansal Mantığı'nın
(STL) *gürbüzlük* semantiği tam bunu çözer:

    marj = sinyalin, boolean sonucu DEĞİŞTİRMEDEN bozulabileceği
           **işaretli mesafe**

  · marj > 0  → sağlandı, ve sayı "ne kadar payımız var" demek
  · marj < 0  → ihlal, ve sayı "ne kadar kötü" demek
  · marj = 0  → tam sınırda

Bu sayede aynı kural hem **alarm** hem **A/B ölçütü** olur: "geçti" diyen
iki ayarın hangisinin daha güvenli olduğu marjdan okunur.

## Neden üç TÜR

Literatürdeki ayrım (guard / invariant / abort) davranışı belirler:

  · `GUARD`    — eylemden **önce** bakılır (önkoşul). İhlalde eylem YAPILMAZ,
                 sistem çalışmaya devam eder. Örn. `degistirilebilir_mi`.
  · `DEGISMEZ` — **sürekli** doğru kalmalı. İhlal bir teşhis sinyalidir.
  · `ABORT`    — ihlalde yürütme **anında durmalı**. Örn. `control_gate`.

Üçü aynı kefeye konursa tepki de tek tip olur; oysa bir GUARD ihlali
"bekle", bir ABORT ihlali "dur" demektir.

## 🔴 BİRİM KARIŞTIRMA YASAĞI

STL literatürü tek bir sinyal uzayı varsayar; gerçek sistemde marjlar
**farklı birimlerde** olur (metre, saniye, m/s). `min(0,3 m, 0,2 s)`
matematiksel olarak hesaplanır ama **anlamsızdır**. Bu yüzden `ve`/`veya`
farklı birimleri birleştirmeyi REDDEDER. Karşılaştırmak gerekiyorsa
`normalize()` ile önce boyutsuzlaştırılır (marj / o kuralın tolerans ölçeği).
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from enum import Enum
from typing import Callable, Iterable, Optional, Sequence


class Tur(Enum):
    """Kuralın ihlal edildiğinde ne yapılması gerektiğini söyler."""

    GUARD = "guard"
    DEGISMEZ = "degismez"
    ABORT = "abort"


#: Birimsiz (normalize edilmiş) marjların etiketi.
BIRIMSIZ = "—"


@dataclass(frozen=True)
class Sonuc:
    """Tek bir kural değerlendirmesinin çıktısı.

    `marj` ve `birim` birlikte anlamlıdır: `-0.29` tek başına bir şey
    söylemez, `-0.29 m` "gövde dubaya 29 cm girdi" demektir.
    """

    kural: str
    marj: float
    birim: str
    tur: Tur
    baglam: dict = field(default_factory=dict)

    @property
    def ihlal(self) -> bool:
        """Marj negatifse ihlal. NaN da ihlal sayılır (bilinmiyor ≠ iyi)."""
        return not (self.marj >= 0.0)

    def __str__(self) -> str:
        isaret = "IHLAL" if self.ihlal else "ok"
        return f"[{isaret}] {self.kural}: {self.marj:+.4g} {self.birim}"


class Kural:
    """Bir değişmez/guard/abort — durumdan **marj** hesaplar.

    Args:
        ad: kısa kimlik (F1, B3, S1 …). Yayın topic adı bundan türetilir.
        aciklama: insan için tek cümle.
        kaynak: 🔴 **ZORUNLU** — eşiğin NEREDEN geldiği. "elle seçildi"
            yazılamaz; ya fizik, ya sözleşme, ya ölçüm, ya şartname maddesi.
            Bu alan, uydurulmuş eşiklerin sisteme sızmasını engelleyen kapıdır.
        birim: marjın birimi ("m", "s", "m/s", "Hz", BIRIMSIZ).
        fn: `durum -> float` (marj). İçeride hesaplanamıyorsa `nan` döndürür
            ve bu **ihlal** sayılır — "ölçemedim" asla "iyi" demek değildir.
        tur: guard / değişmez / abort.
        olcek: normalize ederken kullanılacak tolerans ölçeği (birim başına).
    """

    def __init__(
        self,
        ad: str,
        aciklama: str,
        kaynak: str,
        birim: str,
        fn: Callable[..., float],
        tur: Tur = Tur.DEGISMEZ,
        olcek: Optional[float] = None,
    ) -> None:
        if not kaynak or not kaynak.strip():
            raise ValueError(
                f"{ad}: `kaynak` boş olamaz — her eşik türetilmiş olmalı "
                "(fizik / sözleşme / ölçüm / şartname maddesi)"
            )
        if olcek is not None and not (olcek > 0.0):
            raise ValueError(f"{ad}: `olcek` pozitif olmalı, geldi {olcek}")
        self.ad = ad
        self.aciklama = aciklama
        self.kaynak = kaynak
        self.birim = birim
        self.tur = tur
        self.olcek = olcek
        self._fn = fn

    def olc(self, *a, **kw) -> Sonuc:
        """Kuralı değerlendir. Fonksiyon patlarsa İHLAL döner, çökmez.

        Gerekçe: gözlemcinin kendisi görevi düşüremez (NASA'nın RV uyarısı).
        Ama sessizce de geçemez — hata `nan` marja çevrilir, yani ihlal.
        """
        try:
            m = float(self._fn(*a, **kw))
        except Exception as e:  # noqa: BLE001
            return Sonuc(self.ad, math.nan, self.birim, self.tur,
                         {"hata": f"{type(e).__name__}: {e}"})
        return Sonuc(self.ad, m, self.birim, self.tur)

    def normalize(self, sonuc: Sonuc) -> Sonuc:
        """Marjı boyutsuzlaştır (marj / ölçek) — farklı birimler kıyaslanabilsin.

        `olcek` verilmemişse normalize edilemez; bu bilinçli bir kısıt:
        ölçeği olmayan bir kural başka birimle karşılaştırılamaz.
        """
        if self.olcek is None:
            raise ValueError(f"{self.ad}: `olcek` yok, normalize edilemez")
        return Sonuc(sonuc.kural, sonuc.marj / self.olcek, BIRIMSIZ,
                     sonuc.tur, dict(sonuc.baglam, olcek=self.olcek))

    def __repr__(self) -> str:
        return f"<Kural {self.ad} [{self.tur.value}] {self.birim}>"


# ─────────────────────────── birleştiriciler ────────────────────────────
# STL gürbüzlük semantiği: ve→min · veya→max · değil→işaret çevirme
# her_zaman (G) → pencere üstünde min · bir_ara (F) → pencere üstünde max


def _birim_denetle(sonuclar: Sequence[Sonuc], islem: str) -> str:
    if not sonuclar:
        raise ValueError(f"{islem}: en az bir sonuç gerekli")
    birimler = {s.birim for s in sonuclar}
    if len(birimler) > 1:
        raise ValueError(
            f"{islem}: FARKLI BİRİMLER birleştirilemez {sorted(birimler)} — "
            "önce `Kural.normalize()` ile boyutsuzlaştır"
        )
    return birimler.pop()


def ve(*sonuclar: Sonuc, ad: str = "ve") -> Sonuc:
    """Mantıksal VE — gürbüzlükte **min** (en zayıf halka belirler)."""
    birim = _birim_denetle(sonuclar, "ve")
    en = min(sonuclar, key=lambda s: (math.inf if math.isnan(s.marj) else s.marj))
    zayif = min(sonuclar, key=lambda s: s.marj if not math.isnan(s.marj) else -math.inf)
    secilen = zayif if any(math.isnan(s.marj) for s in sonuclar) else en
    return Sonuc(ad, secilen.marj, birim, secilen.tur,
                 {"belirleyen": secilen.kural,
                  "hepsi": {s.kural: s.marj for s in sonuclar}})


def veya(*sonuclar: Sonuc, ad: str = "veya") -> Sonuc:
    """Mantıksal VEYA — gürbüzlükte **max** (en güçlü halka yeter)."""
    birim = _birim_denetle(sonuclar, "veya")
    gecerli = [s for s in sonuclar if not math.isnan(s.marj)]
    if not gecerli:
        return Sonuc(ad, math.nan, birim, sonuclar[0].tur,
                     {"hepsi_olculemedi": True})
    en = max(gecerli, key=lambda s: s.marj)
    return Sonuc(ad, en.marj, birim, en.tur,
                 {"belirleyen": en.kural,
                  "hepsi": {s.kural: s.marj for s in sonuclar}})


def degil(sonuc: Sonuc, ad: Optional[str] = None) -> Sonuc:
    """Mantıksal DEĞİL — gürbüzlükte işaret çevirme."""
    return Sonuc(ad or f"degil({sonuc.kural})", -sonuc.marj, sonuc.birim,
                 sonuc.tur, dict(sonuc.baglam))


def her_zaman(sonuclar: Iterable[Sonuc], ad: str = "her_zaman") -> Sonuc:
    """G (always) — pencere boyunca **min**. Bir kez bile ihlal → ihlal."""
    liste = list(sonuclar)
    return ve(*liste, ad=ad)


def bir_ara(sonuclar: Iterable[Sonuc], ad: str = "bir_ara") -> Sonuc:
    """F (eventually) — pencere boyunca **max**. Bir kez sağlanması yeter.

    Canlılık kuralları bunun üstüne kurulur: *"ARM'lıyken itki sonsuza kadar
    sıfır kalamaz"* = pencerede **bir ara** sıfırdan farklı olmalı.
    """
    liste = list(sonuclar)
    return veya(*liste, ad=ad)
