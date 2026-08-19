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

from prototype.mission.kamikaze_hedef import HedefRengiHatasi

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


# ───────────────────────── KOD ↔ RENK ÇEVRİMİ ─────────────────────────
# 🔴 19.08 — `renk_kodu_koprusu` için geri getirildi (13.08'de `f6d8847e` ile
# P3 işiyle birlikte main'den çıkmıştı). Köprü, uçuş kontrolcüsünden okunan
# **float** parametreyi renk adına çeviriyor.


def kod_to_renk(kod) -> Optional[str]:
    """Sayısal kod → renk adı (`kamikaze_target_color` için).

    `0` → `None` (hedef atanmamış). Tanınmayan kod **hata**dır: sessizce
    "hedef yok"a düşmek, operatörün yanlış değer girdiğini gizlerdi.

    Kayan noktalı değerler kabul edilir (uçuş kontrolcüsü parametreleri
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
            + ", ".join(f"{k}={v or 'karar yok'}"
                        for k, v in sorted(KOD_RENK.items()))
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


class RenkUygulamaDurumu:
    """*"Hangi rengi hâlâ uygulamamız gerekiyor?"* — ROS'suz durum makinesi.

    🔴 **Neden var (13.08 kusur avında bulundu):** köprünün ilk hâli okunan
    kodu **uygulamadan ÖNCE** önbelleğe yazıyordu. Hedef node'un parametre
    servisi o anda hazır değilse (açılışta ÇOK muhtemel) uygulama sessizce
    atlanıyor, bir sonraki yoklamada kod *"değişmedi"* görünüp erken
    dönülüyordu ⇒ **renk bir daha HİÇ uygulanmıyordu**. Belirti yok, log yok,
    Parkur-3 sessizce sıfır.

    Kural: bir kod ancak **başarıyla uygulandıktan sonra** işlenmiş sayılır.
    Geçici hatalar (servis hazır değil, çağrı düştü, ret) yeniden denenir.
    """

    def __init__(self) -> None:
        self._son_kod: Optional[float] = None
        self._uygulanan: Optional[str] = None
        self._bekleyen: Optional[str] = None

    def kod_geldi(self, kod) -> tuple[Optional[str], bool]:
        """FC'den kod okundu. Döner: (uygulanacak_renk, ilk_kez_görüldü_mü).

        `uygulanacak_renk is None` → yapılacak bir şey yok (kod 0 ya da zaten
        uygulanmış). İkinci değer yalnız **loglama** içindir: aynı kod her
        yoklamada tekrar loglanmasın, ama **denenmeye devam etsin**.

        Raises:
            HedefRengiHatasi: kod tanınmıyorsa (çağıran loglar).
        """
        yeni = self._son_kod is None or abs(float(kod) - self._son_kod) > 1e-6
        self._son_kod = float(kod)
        renk = kod_to_renk(kod)            # geçersizse yukarı fırlar
        if renk is None:
            self._bekleyen = None
            return None, yeni
        if renk == self._uygulanan:
            self._bekleyen = None
            return None, yeni
        self._bekleyen = renk              # 🔑 uygulanana kadar bekler
        return renk, yeni

    def uygulandi(self, renk: str) -> None:
        """Hedef node parametreyi KABUL etti — ancak şimdi işlenmiş sayılır."""
        self._uygulanan = renk
        self._bekleyen = None

    def basarisiz(self) -> None:
        """Geçici hata: bir sonraki yoklamada YENİDEN denenir."""
        self._bekleyen = None
        self._son_kod = None               # kod "değişmedi" diye atlanmasın

    @property
    def bekleyen(self) -> Optional[str]:
        return self._bekleyen

    @property
    def uygulanan(self) -> Optional[str]:
        return self._uygulanan
