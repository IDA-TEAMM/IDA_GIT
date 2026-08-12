"""Parkur-3 hedef SEÇİMİ — ROS'suz saf mantık (FAZ 3, 2026-08-13).

Algı katmanı `/perception/targets`'a **gördüğü tüm** hedef adaylarını basar
(renk kodu + ölçülen çap + konum). Burada tek soru: **hangisine nişan alacağız?**

🔴 **Yanlış hedef pahalıdır.** Şartname s.25 (TS3): doğru hedefe temastan önce
başka hedefe temas → **100 → 50**; iki yanlış temas → **100 → 5**. Bu yüzden
kurallar "en yakını al" değil, "**emin değilsek dokunma**" tarafına eğimli.
"""
from __future__ import annotations

import math
from typing import Iterable, Optional

#: Renk kodu sözleşmesi — algı (`gecit_mantik.HEDEF_RENK_KODU`), karar
#: (`renk_kodu.KOD_RENK`) ve İHA (`cikis.RENK_KODU`) ile **AYNI**.
#: 0 = renk çözülemedi.
RENK_BILINMIYOR = 0

#: Hedef bu yarıçapın dışındaysa "aynı hedef" sayılmaz — nişanı sıçratmamak
#: için değil, **bayat/uçuk tespiti elemek** için. Ölçülen çap bandı zaten
#: algı tarafında (`buyuk_cisim_mi`) uygulanıyor; buradaki ikinci kapı
#: 0,64 m varsayımının makul olduğunu doğrular.
CAP_BANDI = (0.40, 1.00)


class Hedef:
    """`/perception/targets` tespitinin saf karşılığı (dünya çerçevesinde)."""

    __slots__ = ("x", "y", "renk_kodu", "cap_m", "skor")

    def __init__(self, x: float, y: float, renk_kodu: int,
                 cap_m: float = 0.0, skor: float = 0.0) -> None:
        self.x = float(x)
        self.y = float(y)
        self.renk_kodu = int(renk_kodu)
        self.cap_m = float(cap_m)
        self.skor = float(skor)


def cap_makul_mu(cap_m: float, bant=CAP_BANDI) -> bool:
    """Ölçülen çap bir hedef dubası olabilir mi? (0 / bilinmiyor → True)

    Çap 0 gelirse (ölçülemedi) **iddia etmiyoruz** — kör eleme yapmayız;
    aynı kural algı tarafındaki `buyuk_cisim_mi`'de de var.
    """
    if not cap_m or cap_m <= 0.0:
        return True
    return bant[0] <= cap_m <= bant[1]


def hedef_sec(hedefler: Iterable[Hedef], istenen_renk_kodu: Optional[int],
              arac_xy: tuple[float, float]) -> Optional[Hedef]:
    """İstenen renkteki EN YAKIN geçerli hedefi seç. Yoksa `None`.

    Kurallar (hepsi TS3 cezasından türüyor):
      · **İstenen renk yoksa seçim YOK.** Hakem rengi vermemişse hedefe
        kendi kendimize karar vermeyiz.
      · **Renk kodu 0 (çözülemedi) EŞLEŞMEZ.** Konumu bilinen ama rengi
        bilinmeyen bir cisme nişan almak, yanlış hedefe temas riskidir.
        (Algı yine de yayınlar — varlık bilgisi başka amaçlarla değerli.)
      · **Çap bandı dışındaki aday elenir.**
      · Kalanlardan **en yakını** — temas hedefi, yaklaşma ne kadar kısaysa
        sapma o kadar az.
    """
    if not istenen_renk_kodu:                      # None ya da 0
        return None
    ax, ay = arac_xy
    # 📌 Kod 0 (renk çözülemedi) burada AYRICA elenmiyor — gerek yok:
    # `istenen_renk_kodu` 0 ise fonksiyon zaten yukarıda döndü, değilse
    # eşitlik 0'ı tutmaz. (13.08 mutasyon turu: ayrı bir `!= 0` kontrolü
    # eklemiştim, hiçbir testi düşürmedi ⇒ **ulaşılamaz koddu**. Güvenlik
    # gibi görünen ölü kod yanlış güven verir, kaldırıldı.)
    uygun = [h for h in hedefler
             if h.renk_kodu == istenen_renk_kodu
             and cap_makul_mu(h.cap_m)]
    if not uygun:
        return None
    return min(uygun, key=lambda h: math.hypot(h.x - ax, h.y - ay))


#: `/perception/targets` bu süreden eskiyse hedef YOK sayılır. Algı 2 Hz
#: yayınlıyor ⇒ 1,5 sn üç periyot: tek kare kaybı nişanı düşürmez, ama algı
#: ölürse (kamera koptu) bayat konuma sürmeye devam etmeyiz.
HEDEF_BAYATLIK_S = 1.5


def nisan_hedefi(mission_state, istenen_renk_kodu, hedefler, arac_xy,
                 hedef_yasi_s, bayatlik_s: float = HEDEF_BAYATLIK_S):
    """PARKUR-3'te nişan alınacak hedef — yoksa `None`.

    `None` = *"bu çağrıda P3 yolu devrede değil"*; çağıran o zaman **bugünkü**
    davranışına (kapı takibi ya da ham görev noktası) düşer. Dört kapı:

    1. **PARKUR3'te miyiz** — P1/P2'de bu yol tanım gereği kapalı.
    2. **Hedef rengi yüklü mü** — hakem rengi vermemişse hedefe kendi
       kendimize karar vermeyiz (TS3: 100→50→**5**).
    3. **Tespit TAZE mi** — algı sussa bayat konuma nişan almak, olmayan bir
       şeye sürmektir.
    4. **İstenen renkte, çapı makul, en yakın aday var mı** (`hedef_sec`).

    Saf tutuldu ki ROS kurulu olmayan makinede de test edilebilsin — node
    tarafı yalnız bunu çağıran ince bir sarmalayıcıdır.
    """
    if mission_state != "PARKUR3" or not istenen_renk_kodu:
        return None
    if arac_xy is None:
        return None                      # poz yok → gövde→dünya anlamsız
    if not hedefler or hedef_yasi_s > bayatlik_s:
        return None
    return hedef_sec(hedefler, istenen_renk_kodu, arac_xy)
