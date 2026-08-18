"""Parkur-3 hedef renk mekanizması — 12'lik madde #4, şartname md 5.5.3.1.

**Şartnamenin dediği:** *"hedef bilgisi görev başlangıcı için komut verilmeden
önce aktarılabilir, harekete başladıktan sonra aktarılamaz"* (md 5.5.3.1).
Yani hakem Parkur-3'ün hedef duba rengini koşudan **önce** söyler; bizim işimiz
o rengi **arm'dan önce** sisteme geçirmek ve görev başladıktan sonra
**değiştirilemez** kılmak. Aktarımın kendisi yasak değil — YANLIŞ ZAMANDA
aktarım yasak.

**Mekanizma:** kamera node'u zaten her rengi ayrı sınıfla yayınlıyor
(`camera_buoys.py`: turuncu=0, sarı=1, hedef=2, kırmızı=3, yeşil=4,
siyah=5). Hakem "kırmızı" derse yapılacak tek şey, kırmızı tespitlerini
**`CLASS_HEDEF=2`** olarak yeniden etiketlemek — aşağı akıştaki hiçbir şey
(MPPI `w_kamikaze` çekicisi, FSM Parkur-3 mantığı) değişmez, çünkü hepsi
sınıf 2'ye bakıyor.

Bu modül **ROS'suz** ve `cv2`'siz: yalnız sınıf kimlikleri üzerinde çalışır,
böylece kamera/ROS kurulu olmayan makinede de test edilir.

⚠ **Neden turuncu ve sarı REDDEDİLİYOR** (bu modülün asıl değeri):
- **turuncu = 0 = parkur kenarı.** `gate_follower` kapıları TAM bu sınıftan
  buluyor (`edge_buoy_class_id`). Turuncuyu hedefe taşımak iki felaketi birden
  yapar: (1) kapılar görünmez olur → Parkur-1/2 puanı gider, (2) MPPI
  kamikaze çekicisi tekneyi **kapı dubalarının üstüne sürer** → çarpma cezası.
- **sarı = 1 = engel.** Engelleri hedefe çevirmek, kaçınılması gereken şeye
  nişan almak demektir.
Şartname md 5.5.2.1 bu iki rengi navigasyon anlamı için ayırdığı için hakemin
hedef olarak bunları söylemesi beklenmez; ama bir yazım/duyma hatası koşuyu
bitirebileceği için kod bunu **kabul etmiyor**.
"""

from __future__ import annotations

from typing import Iterable, Optional, Protocol

# camera_buoys ile AYNI sözleşme. Oradan import EDİLMİYOR: o modül `cv2`
# çekiyor ve bu modülün ROS'suz/cv2'siz koşabilmesi test edilebilirliğin
# kendisi. Sürüklenme riski `test_kamikaze_hedef.py` içinde bağlanıyor —
# iki taraf ayrışırsa CI kırmızı.
CLASS_PARKUR_KENARI = 0
CLASS_ENGEL = 1
CLASS_HEDEF = 2
CLASS_KIRMIZI = 3
CLASS_YESIL = 4
CLASS_SIYAH = 5          # RAL 9005 — şartname s.18 (18.08: kahverengi'nin yerine)

#: Hakemin söyleyebileceği renk adı → kamera sınıfı. Türkçe karakterli ve
#: karaktersiz yazımlar, İngilizce karşılıkları da kabul edilir; operatör
#: koşu sabahı klavye ayarıyla uğraşmasın.
RENK_SINIFLARI: dict[str, int] = {
    "kirmizi": CLASS_KIRMIZI,
    "kırmızı": CLASS_KIRMIZI,
    "red": CLASS_KIRMIZI,
    "yesil": CLASS_YESIL,
    "yeşil": CLASS_YESIL,
    "green": CLASS_YESIL,
    "siyah": CLASS_SIYAH,
    "black": CLASS_SIYAH,
    # Aşağıdakiler TANINIR ama REDDEDİLİR — bilinmeyen renk ile yasak renk
    # farklı hatalar, operatöre farklı şey söylenmeli.
    "turuncu": CLASS_PARKUR_KENARI,
    "orange": CLASS_PARKUR_KENARI,
    "sari": CLASS_ENGEL,
    "sarı": CLASS_ENGEL,
    "yellow": CLASS_ENGEL,
}

#: Hedef olarak seçilebilen sınıflar. Kasıtlı olarak 0 ve 1 YOK.
SECILEBILIR_SINIFLAR: frozenset[int] = frozenset(
    {CLASS_KIRMIZI, CLASS_YESIL, CLASS_SIYAH}
)

_YASAK_GEREKCE: dict[int, str] = {
    CLASS_PARKUR_KENARI: (
        "turuncu = parkur KENAR dubasi (sinif 0); gate_follower kapilari tam "
        "bu siniftan buluyor. Hedefe tasinirsa kapilar gorunmez olur VE tekne "
        "kapi dubalarina surer"
    ),
    CLASS_ENGEL: (
        "sari = ENGEL (sinif 1); engeli hedefe cevirmek kacinilmasi gereken "
        "seye nisan almak demektir"
    ),
}


class _Tespit(Protocol):
    """`camera_buoys.Detection` ile uyumlu en küçük arayüz."""

    class_id: int


class HedefRengiHatasi(ValueError):
    """Renk adı tanınmadı ya da hedef olarak seçilmesi yasak."""


def kanonik_ad(sinif: Optional[int]) -> str:
    """Sınıf kimliği → `renk_kodu` tablosunun anladığı **kanonik** renk adı.

    🔴 18.08.2026 — NEDEN VAR: `kamikaze_param._renk_yayinla` operatörün
    YAZDIĞI ham metni yayınlıyordu, `planning_node._on_hedef_rengi` ise onu
    `renk_kodu.RENK_KOD`'da arıyor. İki tablo aynı adları tutmadığı için
    takma adlar sessizce düşüyordu (ölçüldü: `RENK_KOD.get("red", 0)` → **0**
    = "hedef atanmamış") ⇒ operatör *"red"* yazarsa renk kabul edilmiş
    görünür ama **P3 nişanı hiç açılmaz**. Aynı delik "green"/"black"te de var.

    Yeni tablo EKLENMEDİ (elle yazılan tablo zaten bu arızanın kaynağıydı):
    kanonik ad, `RENK_SINIFLARI` ile `RENK_KOD`'un **kesişiminden** türetiliyor.
    Kesişim tam olarak bir ad vermiyorsa `test_kanonik_ad_*` kırmızı yanar.

    GERİ ALINIRSA: hakemin rengini İngilizce ya da takma adla girmek P3'ü
    sessizce kapatır (145 puan).
    """
    if sinif is None:
        return ""
    from prototype.mission.renk_kodu import RENK_KOD
    for ad, s in RENK_SINIFLARI.items():
        if s == sinif and ad in RENK_KOD:
            return ad
    raise HedefRengiHatasi(
        f"sinif {sinif} icin kanonik ad yok — RENK_SINIFLARI ile "
        f"renk_kodu.RENK_KOD ayrismis (bkz. kanonik_ad docstring)"
    )


def renk_to_class(ad: Optional[str]) -> Optional[int]:
    """Hakemin söylediği renk adını kamera sınıfına çevir.

    `None` / boş dize → `None` (**hedef atanmamış**, varsayılan hâl). Bu
    kasıtlı: parametre boşken hiçbir şey yeniden etiketlenmez, yani mekanizma
    hakem konuşmadan **hareketsiz** durur.

    Raises:
        HedefRengiHatasi: ad tanınmıyorsa, ya da tanınıyor ama hedef olarak
            seçilmesi yasaksa (turuncu/sarı — modül docstring'ine bakın).
    """
    if ad is None:
        return None
    anahtar = ad.strip().lower()
    if not anahtar:
        return None
    if anahtar not in RENK_SINIFLARI:
        secilebilir = sorted(
            k for k, v in RENK_SINIFLARI.items() if v in SECILEBILIR_SINIFLAR
        )
        raise HedefRengiHatasi(
            f"bilinmeyen hedef rengi {ad!r} — kabul edilenler: "
            f"{', '.join(secilebilir)} (bos dize = hedef atanmamis)"
        )
    sinif = RENK_SINIFLARI[anahtar]
    if sinif not in SECILEBILIR_SINIFLAR:
        raise HedefRengiHatasi(
            f"{ad!r} hedef olarak SECILEMEZ: {_YASAK_GEREKCE[sinif]}"
        )
    return sinif


def hedef_isaretle(
    tespitler: Iterable[_Tespit], hedef_class: Optional[int]
) -> int:
    """`hedef_class` sınıfındaki tespitleri `CLASS_HEDEF`e taşı (yerinde).

    `hedef_class is None` → hiçbir şey yapılmaz, 0 döner (varsayılan hâl).

    Returns:
        Yeniden etiketlenen tespit sayısı. Sıfır dönmesi kamera o rengi o
        karede görmedi demektir — hata değil, ama operatörün Parkur-3'e
        girmeden önce sıfırdan farklı bir sayı görmesi gerekir.
    """
    if hedef_class is None:
        return 0
    if hedef_class not in SECILEBILIR_SINIFLAR:
        # Buraya düşmek bir programlama hatası: renk_to_class zaten eliyor.
        raise HedefRengiHatasi(
            f"hedef sinifi {hedef_class} secilebilir degil "
            f"(izinli: {sorted(SECILEBILIR_SINIFLAR)})"
        )
    sayi = 0
    for t in tespitler:
        if t.class_id == hedef_class:
            t.class_id = CLASS_HEDEF
            sayi += 1
    return sayi


#: Hedef rengin DEĞİŞTİRİLEBİLDİĞİ görev durumları. md 5.5.3.1: harekete
#: başladıktan sonra aktarım yasak → yalnız hareket ÖNCESİ durumlar.
#: (`prototype/fsm/mission_fsm.py` MissionState değerleriyle birebir; string
#: tutuluyor ki bu modül FSM'i import etmek zorunda kalmasın.)
DEGISTIRILEBILIR_DURUMLAR: frozenset[str] = frozenset(
    {"BOOT", "ARM", "BEKLEMEDE", "KILL", "TAMAMLANDI"}
)


def degistirilebilir_mi(gorev_durumu: Optional[str]) -> tuple[bool, str]:
    """Hedef rengi ŞU AN değiştirilebilir mi (md 5.5.3.1)?

    `None` (görev durumu henüz bilinmiyor) → **izin verilir**: FSM daha
    yayına başlamamışsa hareket de başlamamıştır. Emniyetsiz taraf bu değil;
    tersi (bilinmiyorken reddetmek) koşu sabahı ayar yapılmasını engellerdi.

    `KILL`/`TAMAMLANDI` izinli çünkü ikisi de hareketin BİTTİĞİ durumlar —
    yeniden başlama hakkı (md 5.5.3.1, bkz. `/girdap/mission/reset`)
    kullanıldığında renk yeniden verilebilmeli.
    """
    if gorev_durumu is None:
        return True, "gorev durumu bilinmiyor (FSM henuz yayin yapmadi)"
    d = gorev_durumu.strip().upper()
    if d in DEGISTIRILEBILIR_DURUMLAR:
        return True, f"gorev durumu {d} — hareket baslamadi"
    return False, (
        f"gorev durumu {d} — HAREKET BASLADI, md 5.5.3.1 hedef bilgisi "
        f"aktarimini yasakliyor. Once /girdap/mission/kill ya da "
        f"/girdap/mission/reset"
    )
