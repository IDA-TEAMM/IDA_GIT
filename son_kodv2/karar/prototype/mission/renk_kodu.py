"""Parkur-3 hedef rengi — **SAYISAL KOD** sözleşmesi (ROS'suz, cv2'siz).

Neden sayı, neden metin değil (Eyüp, 13.08.2026: *"direkt renk kodunu atalım,
ona göre aralığı İDA belirler"*):

1. **Taşıma kanalı sayı taşıyor.** Rengi YKİ'den İDA'ya geçirmenin şartnameye
   en uygun yolu YKİ **arayüzünden** geçmek (s.21: *"görev yükleme aşamasında
   … YKİ'de sadece YKİ arayüzü açık olacak"*). Mission Planner'ın parametre
   ekranı bir arayüzdür ama parametreler **float**tur — içine "siyah" yazılamaz.
2. **Metin tuzakları biter.** 13.08'de ölçüldü: Python `.lower()` Türkçe `İ`yi
   `i`+U+0307 yapıyor ⇒ `"SİYAH"`/`"YEŞİL"` sözlükte eşleşmiyordu. Sayıda
   büyük/küçük harf, aksan, klavye düzeni sorunu **yok**.
3. **İHA tarafıyla zaten aynı.** `girdap-iha-plaka/plaka/cikis.py` çıktısındaki
   `renk_kodu` **birebir bu tablo** — iki uç aynı sayıyı konuşuyor.

🔴 **DRIFT UYARISI:** bu tablo İKİ AYRI REPODA yaşıyor (burada + İHA reposunda).
Değiştiren, ikisini birden değiştirmek zorundadır; yoksa İHA "3" der, İDA başka
rengi avlar ve bu **sessiz** olur — yanlış hedefe angajman TS3'ü artırır
(şartname s.25: 1 yanlış temas 100→50, 2 yanlış temas 100→**5**).

⚠️ **İHA'nın ÖLÇTÜĞÜ HSV DEĞERLERİ GÖNDERİLMEZ — yalnız KATEGORİ.** Plaka
karada, setin içinde, farklı ışıkta; hedef dubası suda, farklı zeminde. İHA'nın
gördüğü ton İDA'nın göreceği tonu temsil etmez. İDA, koda karşılık gelen
**kendi kalibre edilmiş** HSV bandını kullanır.
"""
from __future__ import annotations

from typing import Optional

from prototype.mission.kamikaze_hedef import HedefRengiHatasi, _anahtarla

#: Kod → renk adı. **0 = KARAR YOK** bilerek ayrı bir değer: operatör
#: *"sistem çalışmıyor"* ile *"renk belirsiz"*i ayırt edebilmeli, ve İHA emin
#: değilse **susmalı** (yanlış renk bildirmek hiç bildirmemekten pahalı).
#: Renkler şartname s.18: RAL 3026 (kırmızı) · RAL 6037 (yeşil) · RAL 9005 (siyah).
KOD_RENK: dict[int, Optional[str]] = {
    0: None,        # karar yok / hedef atanmamış
    1: "kirmizi",   # RAL 3026 floresan kırmızı
    2: "yesil",     # RAL 6037 saf yeşil
    3: "siyah",     # RAL 9005 siyah
}

#: Ters yön — renk adı → kod. Adlar `kamikaze_hedef.RENK_SINIFLARI` ile aynı
#: normalleştirmeden geçer (Türkçe İ tuzağı orada çözülüyor).
RENK_KOD: dict[str, int] = {
    "kirmizi": 1, "kırmızı": 1, "red": 1,
    "yesil": 2, "yeşil": 2, "green": 2,
    "siyah": 3, "siyahi": 3, "black": 3,
}


def kod_to_renk(kod) -> Optional[str]:
    """Sayısal kod → renk adı (`kamikaze_target_color` için).

    `0` → `None` (hedef atanmamış). Tanınmayan kod **hata**dır: sessizce
    "hedef yok"a düşmek, operatörün yanlış değer girdiğini gizlerdi.

    Kayan noktalı gelen değerler kabul edilir (uçuş kontrolcüsü parametreleri
    **float**tur: `SCR_USER1 = 3.0`), ama tam sayıya yakın olmalıdır —
    `2.5` bir renk değildir, yazım hatasıdır.

    Raises:
        HedefRengiHatasi: kod tablo dışında ya da tam sayı değilse.
    """
    try:
        f = float(kod)
    except (TypeError, ValueError) as exc:
        raise HedefRengiHatasi(f"renk kodu sayı değil: {kod!r}") from exc
    if abs(f - round(f)) > 1e-6:
        raise HedefRengiHatasi(
            f"renk kodu tam sayı olmalı, {f!r} geldi (yazım hatası?)"
        )
    i = int(round(f))
    if i not in KOD_RENK:
        raise HedefRengiHatasi(
            f"bilinmeyen renk kodu {i} — kabul edilenler: "
            + ", ".join(f"{k}={v or 'karar yok'}" for k, v in sorted(KOD_RENK.items()))
        )
    return KOD_RENK[i]


def renk_to_kod(ad: Optional[str]) -> int:
    """Renk adı → sayısal kod. Boş/None → 0 (karar yok).

    Raises:
        HedefRengiHatasi: ad tanınmıyorsa. (turuncu/sarı burada da YOK —
        onlar hedef olarak seçilemez, bkz. `kamikaze_hedef`.)
    """
    if ad is None:
        return 0
    anahtar = _anahtarla(ad)
    if not anahtar:
        return 0
    if anahtar not in RENK_KOD:
        raise HedefRengiHatasi(
            f"renk adı koda çevrilemedi: {ad!r} — kabul edilenler: "
            + ", ".join(sorted(RENK_KOD))
        )
    return RENK_KOD[anahtar]
