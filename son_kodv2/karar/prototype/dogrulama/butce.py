# -*- coding: utf-8 -*-
"""BÜTÇE DEĞİŞMEZLERİ — zamanın merdiveni.

Bir otonomi yığınında **en kısa halka kazanır**. Bizim bekçilerimiz ne kadar
iyi olursa olsun, uçuş kontrolcüsünün kendi eşiğinden uzunlarsa geç kalırlar:
ArduPilot `cmd_vel` **3 s** görmezse tekneyi durdurur ve bunu kimseye söylemez.

Bu modül üç şeyi sınar:
  · **B1** — bir döngünün gerçek periyodu nominalin toleransı içinde mi
  · **B2** — uçtan uca gecikme eyleyici eşiğinin altında mı
  · **B3** — bekçi merdiveni monoton mu ve **en dıştaki halka** eyleyicinin
    eşiğinden KISA mı
  · **B4** — tüketici kadansı üreticininkinden hızlı mı

🔴 B3 bu motorun ilk gerçek bulgusu: dağıtım config'inde
`heartbeat_timeout_s = 5,0 s` ve ArduPilot eşiği **3,0 s**. Yani 3-5 saniye
arasında tekne durmuşken yığın hâlâ "sağlıklı" der. Kimse bunu bildirmedi —
kural buldu.
"""
from __future__ import annotations

import math
from typing import Sequence, Tuple

from prototype.dogrulama.kural import Kural, Tur

#: ArduPilot GUIDED modunda komut akışı kesilirse failsafe süresi.
#: 🔴 Bizim ayarımız DEĞİL — uçuş kontrolcüsünün davranışı. Bunu biz
#: değiştiremeyiz, ona GÖRE ayarlanırız.
ARDUPILOT_KOMUT_KESME_S = 3.0


def b1_dongu_periyodu(olculen_s: float, nominal_s: float,
                      tolerans: float = 1.5) -> float:
    """Marj (s): izin verilen periyot − ölçülen.

    KAR-11: kontrol döngüsü 117 ms → 1.062 ms'e çıkmıştı (9×). Tolerans 1,5
    yani %50 sarkma kabul; ötesi "komut hesaplandığı ana ait değil" demek.
    """
    if not (nominal_s > 0.0):
        return math.nan
    return nominal_s * tolerans - olculen_s


def b2_uctan_uca(gecikme_s: float,
                 eyleyici_esigi_s: float = ARDUPILOT_KOMUT_KESME_S) -> float:
    """Marj (s): eyleyici eşiği − ölçülen uçtan uca gecikme."""
    return eyleyici_esigi_s - gecikme_s


def b3_bekci_merdiveni(
    merdiven: Sequence[Tuple[str, float]],
    eyleyici_esigi_s: float = ARDUPILOT_KOMUT_KESME_S,
) -> float:
    """Marj (s): eyleyici eşiği − **en uzun** bekçi süresi.

    `merdiven`: [(ad, saniye), …] — sıralı olması gerekmez, en büyüğü alınır.

    Negatif marj = bizim en dıştaki bekçimiz, uçuş kontrolcüsü tekneyi çoktan
    durdurduktan SONRA uyanıyor demektir.
    """
    if not merdiven:
        return math.nan
    en_uzun = max(v for _, v in merdiven)
    return eyleyici_esigi_s - en_uzun


def b4_kadans(tuketici_hz: float, uretici_hz: float) -> float:
    """Marj (Hz): tüketici − üretici.

    Tüketici yavaşsa kuyruk birikir ve veri bayatlar. Algı düğümü 15 Hz döner
    ama NN 8 FPS üretir ⇒ marj +7 Hz, sağlıklı.
    """
    if not (uretici_hz > 0.0):
        return math.nan
    return tuketici_hz - uretici_hz


B1 = Kural("B1", "Döngü periyodu nominalin toleransı içinde",
           kaynak="KAR-11 ölçümü: 117 ms → 1.062 ms (9× sarkma)",
           birim="s", fn=b1_dongu_periyodu, olcek=0.05)

B2 = Kural("B2", "Uçtan uca gecikme eyleyici eşiğinin altında",
           kaynak="ArduPilot: 3 s komut gelmezse tekneyi DURDURUR",
           birim="s", fn=b2_uctan_uca, tur=Tur.ABORT, olcek=0.50)

B3 = Kural("B3", "En dıştaki bekçi eyleyici eşiğinden kısa olmalı",
           kaynak="ArduPilot komut kesme eşiği 3,0 s (FC davranışı, bizim ayarımız değil)",
           birim="s", fn=b3_bekci_merdiveni, tur=Tur.ABORT, olcek=0.50)

B4 = Kural("B4", "Tüketici kadansı üreticiden hızlı olmalı",
           kaynak="kuyruk birikmesi ⇒ komut hesaplandığı ana ait olmaz",
           birim="Hz", fn=b4_kadans, olcek=1.0)

KURALLAR = (B1, B2, B3, B4)


# ───────────────────── dağıtım config'ini okuyup denetle ──────────────────
def dagitim_merdiveni(hardware_yaml: str) -> list:
    """`hardware.yaml`'dan bekçi merdivenini çıkar — sayılar ELLE yazılmaz."""
    import io

    import yaml

    hw = yaml.safe_load(io.open(hardware_yaml, encoding="utf-8")) or {}
    pl = hw.get("planning", {}) or {}
    fu = hw.get("fusion", {}) or {}
    m = []
    if "pose_timeout_s" in fu:
        m.append(("fusion.pose_timeout_s", float(fu["pose_timeout_s"])))
    if "obstacle_timeout_s" in pl:
        m.append(("planning.obstacle_timeout_s", float(pl["obstacle_timeout_s"])))
    if "heartbeat_timeout_s" in hw:
        m.append(("heartbeat_timeout_s", float(hw["heartbeat_timeout_s"])))
    return m
