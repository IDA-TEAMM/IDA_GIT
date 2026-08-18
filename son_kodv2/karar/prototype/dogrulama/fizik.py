# -*- coding: utf-8 -*-
"""FİZİK DEĞİŞMEZLERİ — eşikler tekne dinamiğinden TÜRETİLİR, yazılmaz.

🔑 Tasarımın çekirdeği: buradaki hiçbir sayı elle girilmez. Hepsi
`CatamaranParams`'tan çıkar. Tekne parametresi değişirse (yeni itici, yeni
gövde) eşikler **kendiliğinden** güncellenir — güncellenmeyi unutmak
imkânsızdır.

Türetimler (kararlı durum, `catamaran.py` denklemleri):

    u̇ = (Fx + Xu·u)/m = 0        ⇒  u_max = −2·T_max / Xu
    ṙ = (Mz + Nr·r)/Iz = 0       ⇒  r_max = −T_max·B / Nr
    u̇(u=0)                       ⇒  a_max = 2·T_max / m

Dağıtım değerleriyle (T=1,455 N · Xu=−2,48 · Nr=−3,0 · B=0,596 · m=11,8):
    u_max ≈ 1,173 m/s · r_max ≈ 0,289 rad/s (16,6 °/s) · a_max ≈ 0,247 m/s²
Üçü de 120 s'lik tam gaz / tam pivot simülasyonuyla **birebir** doğrulandı.

⚠️ ÇEVRESEL PAY: bu tavanlar **itkiden** gelir; akıntı ve dalga aracı ayrıca
sürükler. Kurallar `pay` çarpanıyla kullanılır (varsayılan 1,5) — sayı
uydurulmuş değil, "bozucu itkinin yarısı kadar olabilir" varsayımının açık
hâli ve sahada ölçülünce **düşürülmelidir**.
"""
from __future__ import annotations

import math
from typing import Optional, Sequence, Tuple

from prototype.dogrulama.kural import Kural, Tur

#: Çevresel bozucu payı (akıntı/dalga). 1,0 = pay yok.
#: 🔴 Bu tek "ölçülmemiş" katsayı; sahada akıntı ölçülünce daraltılacak.
CEVRESEL_PAY = 1.5


def _params():
    from prototype.dynamics.catamaran import CatamaranDynamics
    return CatamaranDynamics().p


def tavanlar(p=None) -> Tuple[float, float, float]:
    """(u_max m/s, r_max rad/s, a_max m/s²) — dinamik modelden türetilir."""
    p = p or _params()
    u_max = -2.0 * p.max_thrust / p.Xu
    r_max = -p.max_thrust * p.thruster_spacing / p.Nr
    a_max = 2.0 * p.max_thrust / p.mass
    return u_max, r_max, a_max


# ───────────────────────────── F1 — ışınlanma ─────────────────────────────
def f1_konum_sicramasi(dx: float, dy: float, dt: float,
                       pay: float = CEVRESEL_PAY) -> float:
    """Marj (m): izin verilen yer değiştirme − gerçekleşen.

    KAR-06'da ölçülen 25 ms'de 6,54 m = 261,6 m/s, tavanın **223 katı**.
    Bu oranda eşiğin hassasiyeti önemsiz — kural ayar gerektirmiyor.
    """
    if not (dt > 0.0):
        return math.nan
    u_max, _, _ = tavanlar()
    return u_max * pay * dt - math.hypot(dx, dy)


F1 = Kural(
    "F1", "Konum sıçraması fiziksel tavanı aşamaz",
    kaynak="dinamik model: u_max = −2·T_max/Xu (simülasyonla doğrulandı)",
    birim="m", fn=f1_konum_sicramasi, tur=Tur.DEGISMEZ, olcek=0.10,
)


# ────────────────────────────── F2 — hız/dönüş ─────────────────────────────
def f2_hiz(u: float, pay: float = CEVRESEL_PAY) -> float:
    """Marj (m/s): tavan − |sürat|."""
    u_max, _, _ = tavanlar()
    return u_max * pay - abs(u)


def f2_donus(r: float, pay: float = CEVRESEL_PAY) -> float:
    """Marj (rad/s): tavan − |dönüş hızı|."""
    _, r_max, _ = tavanlar()
    return r_max * pay - abs(r)


F2 = Kural("F2", "Sürat fiziksel tavanı aşamaz",
           kaynak="dinamik model: −2·T_max/Xu", birim="m/s",
           fn=f2_hiz, olcek=0.20)
F2R = Kural("F2R", "Dönüş hızı fiziksel tavanı aşamaz",
            kaynak="dinamik model: −T_max·B/Nr", birim="rad/s",
            fn=f2_donus, olcek=0.05)


# ─────────────────────────────── F4 — sonluluk ─────────────────────────────
def f4_poz_sonlu(*bilesenler: float) -> float:
    """Marj: NaN/∞ varsa −1 (ihlal), hepsi sonluysa +1.

    İkili bir kural — burada "ne kadar sonlu" diye bir şey yok. Marj
    semantiği korunuyor (negatif = ihlal) ama ölçek anlamsız olduğu için
    `olcek` verilmedi ⇒ normalize edilemez, bilerek.
    """
    return 1.0 if all(math.isfinite(b) for b in bilesenler) else -1.0


F4 = Kural("F4", "Poz sonlu olmalı (NaN/∞ yok)",
           kaynak="sayısal geçerlilik — KAR-05'te (0,0,0), füzyonda 10^149",
           birim="—", fn=f4_poz_sonlu, tur=Tur.ABORT)


# ─────────────────────────── F5 — kendi gövdesi ────────────────────────────
def f5_kendi_govdesi(mesafe: float, govde_yari_genislik: float = 0.785 / 2) -> float:
    """Marj (m): tespit mesafesi − gövde yarıçapı.

    ALG-02'de engel bulutunun %27'si aracın arkasındaydı, en yakını 1,3 mm:
    LiDAR kendi gövdesini görüyordu. Gövde yarıçapından yakın hiçbir "engel"
    gerçek olamaz.
    """
    return mesafe - govde_yari_genislik


F5 = Kural("F5", "Gövde yarıçapından yakın tespit sensörün kendisidir",
           kaynak="ölçülen gövde genişliği 0,785 m (09.08)",
           birim="m", fn=f5_kendi_govdesi, olcek=0.10)


KURALLAR = (F1, F2, F2R, F4, F5)
