"""Parkur-3 hedef rengi — SAYISAL KOD SÖZLEŞMESİ (karar tarafı kopyası).

🔴 KANONİK KAYNAK: `girdap-ida-p3/p3_hedef/renk_kodu.py`. Burası **kopya**;
farklı repo/makine olduğu için import edilemiyor. Bu yüzden dosya kendi
kendini denetliyor: `IMZA` tutmuyorsa **açılışta patlar**.

NEDEN BU KADAR SERT (16.08.2026): aynı tablo dört yerde elle kopyalanmıştı ve
**biri ters yazılmıştı** —

    arkadaşın `iha_renk_gonder.py`/`ida_renk_al.py`: 1=siyah 2=kırmızı 3=yeşil
    bizim repolarımız:                              1=kırmızı 2=yeşil 3=siyah

İHA "1" gönderip *siyah* demek isterken İDA "1" okuyup **kırmızı hedefe**
saldırırdı, hiçbir hata mesajı olmadan. Şartname s.25: 1 yanlış temas
100 → **50**, 2 yanlış temas 100 → **5**.

🔴 Bu dosya 16.08'de EKSİKTİ: `planning_node._on_hedef_rengi` onu import
ediyordu ama modül yalnız `girdap-ida-p3`'te vardı ⇒ renk mesajı gelince
`ImportError`. Callback `@_guard`'lı olduğu için node ölmüyordu — **renk
sessizce hiç uygulanmıyordu** ve P3 nişanı hep kapalı kalıyordu.

Renkler şartname s.18: RAL 3026 (kırmızı) · RAL 6037 (yeşil) · RAL 9005 (siyah).
"""
from __future__ import annotations

import unicodedata
from typing import Optional

#: 0 = KARAR YOK, bilerek ayrı değer: operatör *"sistem çalışmıyor"* ile
#: *"renk belirsiz"*i ayırt edebilmeli; İHA emin değilse **susmalı**
#: (yanlış renk bildirmek hiç bildirmemekten pahalı).
KOD_RENK: dict[int, Optional[str]] = {
    0: None,        # karar yok / hedef atanmamış
    1: "kirmizi",   # RAL 3026
    2: "yesil",     # RAL 6037
    3: "siyah",     # RAL 9005
}
RENK_KOD: dict[str, int] = {v: k for k, v in KOD_RENK.items() if v}

#: İnsan-okunur etiket (telemetri/log). Kod ile BİRLİKTE gönderilir ki alıcı
#: **çapraz doğrulayabilsin** — sürüm ayrışması sessiz kalmasın.
ETIKET: dict[int, str] = {
    0: "YOK",
    1: "KIRMIZI-RAL3026",
    2: "YESIL-RAL6037",
    3: "SIYAH-RAL9005",
}

#: Tablonun parmak izi — kanonik kaynakla BİREBİR aynı olmalı.
IMZA = "p3renk-v1:0=yok,1=kirmizi,2=yesil,3=siyah"


def _anahtarla(ad: str) -> str:
    """Serbest metni tablo anahtarına indirge.

    Operatör `ros2 param set` ile *"Kırmızı"*, *"KIRMIZI"* ya da *" kirmizi "*
    yazabilir; telsizden gelen etiket de büyük harfli olabilir. Türkçe
    karakterler ASCII'ye düşürülür (ı→i, ş→s, …) — aksi hâlde "kırmızı"
    tabloda BULUNAMAZ ve renk sessizce atanmamış sayılırdı.
    """
    s = (ad or "").strip().lower()
    # Türkçe'ye özel: 'ı' NFKD ile ayrışmaz, elle eşlenir.
    s = s.replace("ı", "i").replace("İ", "i")
    s = unicodedata.normalize("NFKD", s)
    return "".join(c for c in s if not unicodedata.combining(c))


def dogrula(kod_renk: dict) -> None:
    """Kopyalanmış tabloyu kanonikle karşılaştır; ayrışıksa PATLA.

    Sessiz yanlış eşleme, gürültülü çökmeden **çok** daha pahalıdır.
    """
    if kod_renk != KOD_RENK:
        raise ValueError(
            "RENK KODU SÖZLEŞMESİ AYRIŞMIŞ!\n"
            f"  beklenen: {KOD_RENK}\n"
            f"  gelen   : {kod_renk}\n"
            "  ⇒ İHA ile İDA farklı renk konuşuyor. Yanlış hedefe angajman "
            "100→50 puan (şartname s.25). Tabloyu p3_hedef/renk_kodu.py'ye eşitle."
        )


def kod_dogru_mu(kod: int, etiket: str) -> bool:
    """Gelen (kod, etiket) çifti kendi içinde tutarlı mı?

    🔑 Asıl koruma bu: sayı tek başına sürüm ayrışmasını yakalayamaz, ama
    yanına insan-okunur etiket konursa alıcı çelişkiyi **görebilir**.
    Çelişkide karar: **REDDET** — yanlış renkle angajman, angajman
    yapmamaktan pahalı.
    """
    return ETIKET.get(int(kod)) == (etiket or "").strip()
