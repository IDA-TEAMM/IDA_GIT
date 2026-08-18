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


# ─────────────────────── HEDEF KİLİDİ (2026-08-13) ────────────────────────
#: Kilitlemeden önce hedefin kaç ARDIŞIK karede görülmesi gerekir.
#: Tek karelik yanlış tespite kilitlenmek, yanlış hedefe sürmek demektir
#: (TS3: 100→50→**5**). 2 Hz'te 3 kare = 1,5 sn — angajman bütçesinde hiçbir şey.
KILIT_ONAY_KARE = 3

#: Kilitliyken gelen yeni tespit, kilitli konuma bu mesafeden yakınsa **aynı
#: hedef** sayılır ve konum tazelenir. Uzaktaysa **yok sayılır** — hedefler
#: yan yana duruyor (Şekil 3) ve saldırı ortasında hedef değiştirmek, iki
#: hedefe birden temas riskidir.
KILIT_TAZELEME_M = 3.0


class HedefKilidi:
    """Bir hedefe kilitlen ve **görüntü kesilse de** nişanı koru.

    🔴 **NEDEN GEREKLİ.** Nişan yalnız taze tespitle sürülürse, hedef bir an
    kaybolduğunda (dalga, serpinti, kadraj taşması) `nisan_hedefi` `None`
    döner ve çağıran **ham görev noktasına** düşer — o da Parkur-2'nin son
    noktası, yani **arkamız**. Sonuç: saldırı ortasında tekne hedeften döner.

    🔑 Bu, karar tarafının kapı hafızasıyla (`edge_memory`) **aynı desen**:
    *"kapıya yaklaşırken direkler kaçınılmaz olarak kadrajdan çıkar"* ⇒
    hatırlamak tahmin değil, **fiziksel gerçeğin korunması**. Bizde daha da
    güçlü: hedef **şamandıra, demirli** — yer değiştirmiyor. Kaybolan şey
    cismin kendisi değil, o karede **görülebilirliği**.

    ÖLÇÜLDÜ (13.08): hedef **0,7 m**'ye kadar kadrajda kalıyor, ama **0,3 m**
    altında stereo ölüyor (`setDepthLowerThreshold(300)`) ⇒ son yarım metrede
    tespit **kesinlikle** kesilir. Kilit olmadan tam temas anında nişan düşer.

    ⚠️ Konum **dünya çerçevesinde** saklanır; odometri sürüklenmesi kadar
    hata birikir. Yaklaşma kısa (10 m @1,5 m/s ≈ 7 sn) olduğu için ihmal
    edilebilir, ama **suda doğrulanmalı**.
    """

    def __init__(self, onay_kare: int = KILIT_ONAY_KARE,
                 tazeleme_m: float = KILIT_TAZELEME_M) -> None:
        self._onay_kare = int(onay_kare)
        self._tazeleme_m = float(tazeleme_m)
        self._aday: Optional[Hedef] = None
        self._sayac = 0
        self._kilitli: Optional[Hedef] = None

    def guncelle(self, secilen: Optional[Hedef]) -> Optional[Hedef]:
        """Bu karenin seçimini ver, nişan alınacak hedefi al (`None` = yok).

        · Kilitli DEĞİLKEN: aynı hedef `onay_kare` kez üst üste görülürse kilitlenir.
        · Kilitliyken: yakındaki yeni tespit konumu **tazeler**; tespit yoksa
          **kilitli konum korunur** (asıl amaç bu).
        """
        if self._kilitli is not None:
            if secilen is not None and self._yakin(secilen, self._kilitli):
                self._kilitli = secilen          # taze ölçümle konumu güncelle
            return self._kilitli

        if secilen is None:
            self._aday, self._sayac = None, 0    # ardışıklık bozuldu
            return None
        if self._aday is not None and self._yakin(secilen, self._aday):
            self._sayac += 1
        else:
            self._aday, self._sayac = secilen, 1
        self._aday = secilen
        if self._sayac >= self._onay_kare:
            self._kilitli = secilen
            return self._kilitli
        return None                              # henüz onaylanmadı

    def sifirla(self) -> None:
        """Parkur-3'ten çıkıldı / yeniden başlama — kilit bırakılır."""
        self._aday, self._sayac, self._kilitli = None, 0, None

    def _yakin(self, a: Hedef, b: Hedef) -> bool:
        return math.hypot(a.x - b.x, a.y - b.y) <= self._tazeleme_m

    @property
    def kilitli(self) -> Optional[Hedef]:
        return self._kilitli

    @property
    def onay_sayaci(self) -> int:
        return self._sayac
