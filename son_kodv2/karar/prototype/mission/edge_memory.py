r"""
Girdap İDA — KENAR DUBASI HAFIZASI.

🔑 **Bir kez turuncu sınıflanan duba, rengi artık görünmese de kenar kalır.**

🔴 **NEDEN GEREKLİ — geometri (2026-08-09 ölçümü, GIRDAP_DURUM §0.17d/e)**

Kamera 69° (1,2 rad) ve 15 m menzilli; LiDAR (Livox Mid-360) **360°** ve 25 m.
Bir kapının **iki direğini aynı karede** görebilmek için:

    mesafe ≥ yarı_açıklık / tan(FOV/2)

| açıklık | ikisi birden görünür | pencere | 1,05 m/s'de |
|---|---|---|---|
| **12 m** (gerçek P1) | 8,8 – 15,0 m | **6,2 m** | ~5,9 s |
| 18 m | 13,2 – 15,0 m | 1,8 m | ~1,7 s |

Yani kapıya yaklaşırken direkler **kaçınılmaz olarak** kadrajdan çıkar. O anda
füzyon onları eşleşmemiş LiDAR kümesi olarak `CLASS_UNKNOWN=99` ile geçirir
(güvenlik kuralı: bilinmeyeni atma) → `planning_node._on_classified` onları
**engel torbasına** koyar → MPPI'nin `obstacle_margin` ceza halkası **tam kapı
ağzında** aracı dışarı iter. Kilit hedefi korur, itme kalır.

**Kaybolan şey konum DEĞİL, RENK.** Konum Livox'tan akmaya devam ediyor; renk
bir kez öğrenildi ve **duba renk değiştirmez**. Hatırlamak bu yüzden bir tahmin
değil, fiziksel bir gerçeğin kullanılmasıdır.

**Ölçüldü** (`parkur_nihai.world`, 8 kapı, 3 tohum, kapalı döngü, gerçek dinamik):

| açıklık | kol | GN | kapı | gövde payı |
|---|---|---|---|---|
| 18 m | hafızasız | 4/4 | 6/8 | +0,456 |
| 18 m | hafızalı | 4/4 | 7/8 | +0,434 |
| **12 m** | **hafızasız** | 🔴 **1/4** | 4/8 | −0,492 |
| **12 m** | **hafızalı** | ✅ **4/4** | 6/8 | −0,283 |

18 m'de marjinal, **12 m'de belirleyici: hafızasız kod Parkur-1'i BİTİREMİYOR.**

---

🔑 **EŞLEŞME ÖLÇÜSÜ: AYARLANABİLİR EŞİK YOK — İKİ DAİRENİN ÇAKIŞMASI**

Modülün donmuş tasarım kuralı (bkz. `GateFollowerConfig` docstring'i +
`test_kapi_seciminde_ayarlanabilir_esik_KALMADI`): *her sayı ya ölçülmüş bir
tekne boyutudur ya da saf geometridir.* Kapı geometrisi önceden bilinemez, o
yüzden "sahada ölçüp gir" diye bir eşleşme yarıçapı olamaz.

Burada da yok. Ölçüt: **iki katı cisim aynı yeri kaplayamaz.**

    aynı duba  ⟺  merkez_mesafesi ≤ r_hatırlanan + r_yeni

İki yarıçap da tespitin **kendi mesajından** gelir (`bbox.size.x` = çap,
`perception_fusion_node` sözleşmesi) → öz-ölçekli, ayarsız, ölçek-bağımsız.
Belirsizlik hâlinde (birden çok kayıt çakışıyorsa) **en yakını** seçilir; bu bir
eşik değil, tekil atama kuralıdır.

⚠ Ölçüt neden yeterli: 10 Hz'te 1,05 m/s ile araç kare başına ~0,10 m yol alır,
duba yarıçapı 0,15 m → çakışma kaybolmaz. Kapı direkleri 12 m, ardışık kapılar
4 m aralıklı — yani 0,30 m'lik çakışma bandında **başka aday yok**.

---

⚠️ **HAFIZANIN GÜVENLİK BEDELİ VE ÇELİŞKİ KURALI**

Hatırlanan duba engel torbasından ÇIKARILIR → orada kaçınma yapılmaz. O yüzden
hafıza yanlışsa bedeli çarpmadır. Tek gerçek risk şartnamenin kendi uyardığı
durumdur: *"kenar dubaları ve engeller deniz şartlarından dolayı yer
değiştirebilir"* → turuncu duba kayar, yerine SARI bir engel gelir.

Kural: **bilinen ve farklı bir sınıf, hafızayı iptal eder.**
- `CLASS_UNKNOWN` (99) ya da sınıfsız → **çelişki DEĞİL** (zaten düzeltmek
  istediğimiz hâl: LiDAR görüyor, kamera rengi veremiyor).
- Başka bir bilinen sınıf (sarı engel=1, hedef duba=2) → kayıt **silinir**,
  tespit engel olur. Kamera "orada sarı var" diyorsa hafızadan daha tazedir.

Hafızada süreye/mesafeye bağlı unutma **yoktur** — ölçülen kol (C) da öyleydi.
Sınırsız büyüme riski yok çünkü kayıtlar yerinde GÜNCELLENİR: hareket eden duba
yeni kayıt doğurmaz (`test_hareket_eden_duba_YENI_kayit_dogurmaz`).
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import List, Optional, Sequence, Tuple

# Füzyon sözleşmesi: eşleşmeyen LiDAR kümesi bu sınıfla geçer (güvenlik).
# prototype.perception.fusion.CLASS_UNKNOWN ile aynı olmalı — import etmiyoruz
# ki bu modül algı katmanına bağımlı olmasın; nöbetçi test ikisini bağlıyor.
CLASS_UNKNOWN = 99

# (x, y, yarıçap, sınıf) — dünya (ENU) çerçevesinde, metre. Sınıf None olabilir
# (sayısal olmayan class_id → sınıfsız sayılır).
Tespit = Tuple[float, float, float, Optional[int]]


@dataclass
class HatirlananKenar:
    """Haritada tutulan tek cismin en son bilinen hâli.

    🆕 H1 (2026-08-09): `sinif` alanı eklendi — hafıza artık YALNIZ turuncu
    kapı direklerini değil, **görülen her cismi** tutuyor (sarı engel, iskele,
    tekne, sınıfsız LiDAR kümesi…). `sinif is None` → hiç sınıflanmamış.
    """

    x: float
    y: float
    r: float
    # Kaç algı karesinde teyit edildi — salt teşhis (sahada "hafıza tutuyor mu").
    gorulme: int = 1
    #: En son bilinen sınıf. `edge_class_id` ise kenar dubası; CLASS_UNKNOWN
    #: ya da None ise sınıfsız (engel olarak kalır, güvenlik kuralı).
    sinif: Optional[int] = None
    #: Bu karede taze görüldü mü — `siniflandir()` her çağrıda tazeler.
    #: `hatirlananlar()` yalnız GÖRÜLMEYENLERİ döndürmek için buna bakar.
    taze: bool = False

    def cakisiyor(self, x: float, y: float, r: float) -> bool:
        """İki daire çakışıyor mu = aynı fiziksel cisim mi (ayarsız ölçüt)."""
        return math.hypot(self.x - x, self.y - y) <= (self.r + r)

    def uzaklik(self, x: float, y: float) -> float:
        return math.hypot(self.x - x, self.y - y)


class EdgeBuoyMemory:
    """Kenar dubası hafızası — modül docstring'ine bak.

    Kullanım (`planning_node._on_classified`):

        kenar_bayraklari = hafiza.siniflandir(tespitler, edge_class_id)

    Dönen liste girdiyle **aynı sırada** ve her elemanı "bu tespit kenar dubası
    mı" sorusunun cevabıdır. True olanlar engel torbasından çıkarılıp
    `GateFollower`'a beslenir, False olanlar engel kalır.
    """

    def __init__(self) -> None:
        self._kayitlar: List[HatirlananKenar] = []
        #: Son `siniflandir` çağrısındaki kenar sınıfı — `hatirlananlar()`
        #: bunu kullanır. Ayrı bir parametre DEĞİL (donmuş kural: ayar yok).
        self._edge_id: Optional[int] = None
        self._hatirlanarak_kurtarilan = 0      # teşhis: hafızanın kazandırdığı
        self._celiskiyle_silinen = 0           # teşhis: sınıf çelişkisi sayısı

    # ------------------------------------------------------------- teşhis

    @property
    def boyut(self) -> int:
        return len(self._kayitlar)

    @property
    def hatirlanarak_kurtarilan(self) -> int:
        """Rengi görünmezken hafıza sayesinde kenar kalan tespit sayısı.

        Sahada 0 ise hafıza hiç iş görmüyor demektir (ya kapıya hiç yaklaşılmadı
        ya da eşleşme tutmuyor); yüksekse beklenen davranış işliyor.
        """
        return self._hatirlanarak_kurtarilan

    @property
    def celiskiyle_silinen(self) -> int:
        """Bilinen farklı sınıf yüzünden iptal edilen kayıt sayısı."""
        return self._celiskiyle_silinen

    def kayitlar(self) -> List[Tuple[float, float, float]]:
        """Hatırlanan kenarlar (x, y, r) — RViz/teşhis için kopya."""
        return [(k.x, k.y, k.r) for k in self._kayitlar]

    def temizle(self) -> None:
        """Parkur geçişinde/yeniden başlamada hafızayı sıfırla."""
        self._kayitlar.clear()

    # ------------------------------------------------------------- çekirdek

    def siniflandir(
        self, tespitler: Sequence[Tespit], edge_class_id: int
    ) -> List[bool]:
        """Her tespit için "kenar dubası mı" bayrağı üret (girdiyle aynı sıra).

        İki geçiş, sırası ÖNEMLİ:
          1. Rengi ŞU AN görünenler — hafızayı kurar/günceller. Önce çalışır ki
             aynı karede taze renk, bayat kayda göre öncelikli olsun.
          2. Rengi görünmeyenler — sınıfı bilinen ve farklıysa çelişki (kaydı
             siler, engel olur); sınıfsız/UNKNOWN ise hafızada aranır.
        """
        self._edge_id = edge_class_id
        kenar = [False] * len(tespitler)
        kullanilan: set[int] = set()          # bir kayıt en fazla bir tespite
        for k in self._kayitlar:              # H1: tazelik her karede sıfırlanır
            k.taze = False

        # --- 1. geçiş: renk ŞU AN görünüyor ---------------------------------
        for i, (x, y, r, cls) in enumerate(tespitler):
            if cls != edge_class_id:
                continue
            kenar[i] = True
            j = self._eslesen_kayit(x, y, r, haric=kullanilan)
            if j is None:
                self._kayitlar.append(
                    HatirlananKenar(x, y, r, sinif=cls, taze=True)
                )
                kullanilan.add(len(self._kayitlar) - 1)
            else:
                self._tasi(j, x, y, r, cls)
                kullanilan.add(j)

        # --- 2. geçiş: renk görünmüyor --------------------------------------
        silinecek: set[int] = set()
        for i, (x, y, r, cls) in enumerate(tespitler):
            if kenar[i]:
                continue
            bilinen_farkli = (
                cls is not None
                and cls != edge_class_id
                and cls != CLASS_UNKNOWN
            )
            j = self._eslesen_kayit(x, y, r, haric=kullanilan | silinecek)
            if j is None:
                # 🆕 H1: eşleşen kayıt yok → YENİ kayıt aç. Hafıza artık yalnız
                # turuncu direkleri değil GÖRÜLEN HER CİSMİ tutuyor (sarı engel,
                # iskele, tekne, sınıfsız küme). Kenar bayrağı False kalır:
                # sınıfsız cisim engel olarak sürer (güvenlik kuralı bozulmadı).
                self._kayitlar.append(
                    HatirlananKenar(x, y, r, sinif=cls, taze=True)
                )
                kullanilan.add(len(self._kayitlar) - 1)
                continue
            if bilinen_farkli:
                # Kamera "orada sarı/hedef var" diyor — hafızadan taze bilgi.
                # Kayıt SİLİNMEZ, SINIFI GÜNCELLENİR (H1): cisim hâlâ orada,
                # yalnız ne olduğu değişti. Silmek onu haritadan düşürürdü.
                self._tasi(j, x, y, r, cls)
                self._celiskiyle_silinen += 1
                kullanilan.add(j)
                continue
            kenar[i] = self._kayitlar[j].sinif == edge_class_id
            self._tasi(j, x, y, r, cls)
            self._kayitlar[j].gorulme += 1
            if kenar[i]:
                self._hatirlanarak_kurtarilan += 1
            kullanilan.add(j)

        return kenar

    def hatirlananlar(self) -> List[Tuple[Tespit, bool]]:
        """🆕 H1 — bu karede GÖRÜLMEYEN kayıtlar `((x, y, r, sınıf), kenar_mı)`.

        🔴 **Çözdüğü arıza (GIRDAP_DURUM §0.21/H1, ölçüm §0.22d).** `siniflandir`
        yalnız **gelen** tespiti sınıflandırıyordu; **kaybolan** cismi haritada
        tutmuyordu. Sonuç: planlayıcının engel torbası her karede sıfırdan
        kuruluyordu ve o an görülmeyen cisim **yok** sayılıyordu.

        Ölçüldü: kapıya yaklaşınca direkler önce kameranın 69°'lik kadrajından,
        sonra LiDAR'ın (30 cm duba için ~8 m) menzilinden çıkıyor. O anda
        hiçbir tespit gelmiyor → kapı ortadan kayboluyor → araç nişanı
        kaybediyor. H3 (kamera menzili) bunu 0/3'ten 2/3'e çıkardı ama
        kapatamadı; kalan boşluk tam burası.

        🔑 **UNUTMA YOK — kaptan kararı (09.08).** Kayıt ne süreyle ne de
        "LiDAR oraya bakıyor ama boş dönüyor" kuralıyla silinir. Gerekçe:
        LiDAR o dubayı zaten 8 m'den uzakta göremiyor, yani böyle bir silme
        kuralı cismi **yok olduğu için değil menzil yetmediği için** silerdi —
        düzeltilen arızayı geri getirirdi. Ayrıca asimetrik bedel: hayalet
        duba yalnız gereksiz kaçınma yaratır, unutulan gerçek duba ise
        **çarpma** (Ç1: P1'de 16 puan).

        ⚠ Konum doğruluğu odometriye bağlı. RTK sabit fix'te ~5 cm (H-RTK
        F9P) — duba çapının (30 cm) çok altında, yani hafıza pratikte
        kesindir. RTK kaybında 2,5 m'ye çıkar; o hâlde eski kayıtla eşleşme
        tutmaz ve **yeni** kayıt açılır (çift kayıt), silme değil.
        """
        return [
            ((k.x, k.y, k.r, k.sinif), k.sinif is not None and k.sinif == self._edge_id)
            for k in self._kayitlar
            if not k.taze
        ]

    # ------------------------------------------------------------- yardımcı

    def _eslesen_kayit(
        self, x: float, y: float, r: float, *, haric: set
    ) -> Optional[int]:
        """Çakışan kayıtlardan EN YAKINI (yoksa None). Eşik değil, tekil atama."""
        en_iyi, en_yakin = None, math.inf
        for j, k in enumerate(self._kayitlar):
            if j in haric or not k.cakisiyor(x, y, r):
                continue
            d = k.uzaklik(x, y)
            if d < en_yakin:
                en_iyi, en_yakin = j, d
        return en_iyi

    def _tasi(self, j: int, x: float, y: float, r: float,
              sinif: Optional[int] = None) -> None:
        """Kaydı en son ölçüme taşı — duba (ve odometri) kayar, kayıt takip eder.

        Yerinde güncelleme, hafızanın sınırsız büyümesini de engeller: hareket
        eden duba yeni kayıt doğurmaz.
        """
        k = self._kayitlar[j]
        k.x, k.y, k.r = x, y, r
        k.taze = True
        # Sınıf yalnız BİLİNEN bir değerle güncellenir: UNKNOWN/None gelmesi
        # "artık bilmiyoruz" demek değil, "bu karede kamera veremedi" demektir.
        # Öğrenilmiş rengi silmek H1'in çözdüğü arızayı geri getirirdi.
        if sinif is not None and sinif != CLASS_UNKNOWN:
            k.sinif = sinif
