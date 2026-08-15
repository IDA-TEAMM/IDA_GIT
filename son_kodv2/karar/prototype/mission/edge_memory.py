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

Burada da yok. Ölçüt: **iki katı cisim aynı yeri kaplayamaz + kısmi görüş payı.**

    aynı duba  ⟺  merkez_mesafesi ≤ r_hatırlanan + r_yeni + duba_çapı

İki yarıçap tespitin **kendi mesajından** gelir (`bbox.size.x` = çap,
`perception_fusion_node` sözleşmesi); üçüncü terim **şartnamenin verdiği tek
kesin boyut** (duba çapı 0,30 m). Belirsizlik hâlinde (birden çok kayıt
çakışıyorsa) **en yakını** seçilir; bu bir eşik değil, tekil atama kuralıdır.

🔴 **Üçüncü terim neden var (FAZ 1, GIRDAP_DURUM §1.10c/§1.13, 15.08 göl
ölçümü):** LiDAR dubanın yalnız kendine bakan yüzünü görür; küme merkezi bu
yüzden gerçek merkezin değil YAKIN YÜZEYİN üstüne oturur ve bakış açısı
değiştikçe aynı dubanın ölçülen merkezi **bir çapa kadar** oynar (hakemli
karşılığı: kısmi yaydan daire uydurma yanlılığı — GIRDAP_DURUM §1.15b).
Yalın `r+r` bandı bunu saymıyordu; göl bandında sahte kayıtların %63,5'i
bandın hemen dışında ([0,30–0,60) m) doğdu, hafıza 26 dakikada 3 573 kayda
şişti ve ikizler `_huni_payi`'yi direklerin %84,8'inde sıfırladı (çarpışma
koruması söküldü). Aynı arıza MIT Arcturus'un RoboBoat 2026 raporunda da var:
*"identifying the buoy pairs was not working properly in the presence of
duplicates"* (§1.15a).

⚠ Ölçüt neden hâlâ güvenli: kapı direkleri 12 m, ardışık kapılar 4 m
aralıklı — 0,60 m'lik bantta **başka gerçek aday yok**. Bedel, F-A.1'in
"kalan risk" penceresinin 0,30→0,60 m'ye büyümesi (iki bağımsız parlama aynı
banda düşerse onay dolar); izleme kanalı aynı: `onaylanan` sayacı.

---

⚠️ **HAFIZANIN GÜVENLİK BEDELİ VE ÇELİŞKİ KURALI**

Hatırlanan duba engel torbasından ÇIKARILIR → orada kaçınma yapılmaz. O yüzden
hafıza yanlışsa bedeli çarpmadır. Tek gerçek risk şartnamenin kendi uyardığı
durumdur: *"kenar dubaları ve engeller deniz şartlarından dolayı yer
değiştirebilir"* → turuncu duba kayar, yerine SARI bir engel gelir.

Kural: **bilinen ve farklı bir sınıf, hafızayı iptal eder.**
- `CLASS_UNKNOWN` (99) ya da sınıfsız → **çelişki DEĞİL** (zaten düzeltmek
  istediğimiz hâl: LiDAR görüyor, kamera rengi veremiyor).
- Başka bir bilinen sınıf (sarı engel=1, hedef duba=2) → kaydın **SINIFI
  GÜNCELLENİR** (H1'den beri; eskiden silinirdi). Cisim hâlâ oradadır, yalnız ne
  olduğu değişmiştir — silmek onu haritadan düşürürdü.

Hafızada süreye bağlı unutma **yoktur** — ölçülen kol (C) da öyleydi.

---

🔴 **YAYIM MENZİLİ (2026-08-10) — `hatirlananlar(arac_xy, menzil)`**

Eski not *"sınırsız büyüme riski yok çünkü kayıtlar yerinde GÜNCELLENİR"* **eksikti**:
yerinde güncelleme yalnız **eşleşen** kayıt için geçerli. Ölçüldü (GIRDAP_DURUM
§0.26b): kare arası konum sıçraması çakışma bandının (`r₁+r₂`=0,30 m) üçte birini
geçince aynı duba için **ikinci kayıt** açılıyor —

    sıçrama 0,08 m → 8 kayıt · 0,10 m → 24 · 0,12 m → **220** · 0,15 m → **1 362**

…ve bedeli iki yerde ödeniyor: `planning_node._huni_payi` **O(n²) saf Python**
(481 kenarda 10 Hz bütçesinin %25'i, laptopta) ve MPPI'nin `(K, T+1, N)` engel
tensörü (N=2000'de 1,6 GB — Orin Nano'da bellek işlemciyle PAYLAŞIMLI).

**Çözüm silme DEĞİL, YAYIM MENZİLİ:** kayıt haritada kalır (kaptan kararı korunur),
yalnız araçtan `menzil` ötesindekiler engel torbasına **konmaz**. Sayı uydurulmadı:
çağıran `PlanningPipeline`'ın yerel maliyet haritası penceresinin yarıçapını verir
(`map_width × map_resolution / 2` = 25 m) — planlayıcının zaten akıl yürüttüğü alan.
Aynı sayı LiDAR'ın `max_range`'i ile de örtüşüyor: o menzilin ötesinde zaten taze
tespit gelmiyor.

⚠ Geriye tam uyumlu: `menzil=None` → eski davranış (hepsi döner).
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import List, Optional, Sequence, Tuple

# Füzyon sözleşmesi: eşleşmeyen LiDAR kümesi bu sınıfla geçer (güvenlik).
# prototype.perception.fusion.CLASS_UNKNOWN ile aynı olmalı — import etmiyoruz
# ki bu modül algı katmanına bağımlı olmasın; nöbetçi test ikisini bağlıyor.
CLASS_UNKNOWN = 99

# Şartnamenin verdiği tek kesin boyut: duba çapı 30 cm (yükseklik 50 cm).
# Eşleşme bandındaki kısmi-görüş payı ve kayıt↔kayıt konsolidasyonu bu
# sabitten türer (FAZ 1, GIRDAP_DURUM §1.10c/§1.13) — ayar parametresi değil.
DUBA_CAPI_M = 0.30

# Kayıt↔kayıt konsolidasyon kadansı: her N `hatirlananlar()` çağrısında bir
# (≈2 s @10 Hz). Neden her karede değil: tarama O(n²) ve tespit↔kayıt bandı
# genişletildiği için ikizler zaten nadiren doğar; konsolidasyon yalnız
# birbirinin bandına SONRADAN sürüklenen kayıtları toplar. Neden ilk çağrıda
# hemen: şişmiş bir hafızayla açılan oturum (bant replay, §1.13) ilk karede
# temizlensin.
_BIRLESTIRME_KADANSI = 20

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
    #: 🔴 F-A.1 (13.08.2026) — TEKRAR SAYACI. Bu konum kaç KARede turuncu
    #: görüldü. Kayıt, sayaç **2**'ye ulaşana kadar kenar dubası SAYILMAZ →
    #: engel torbasında kalır (güvenli varsayılan). 2 ayarlanabilir bir eşik
    #: DEĞİL, "tekrar"ın mantıksal asgarisidir: bir kez görülmek tekrar
    #: değildir. Ayar parametresi YOK (donmuş kural §0.0d korunur).
    turuncu_sayaci: int = 0

    def cakisiyor(self, x: float, y: float, r: float) -> bool:
        """Aynı fiziksel cisim mi — daire çakışması + kısmi görüş payı.

        FAZ 1 (§1.13): yalın `r+r` bandı, LiDAR'ın kısmi görüşünden gelen
        merkez oynamasını (bir çapa kadar, §1.10c ölçümü) saymıyordu ve göl
        koşumunda ikiz kayıt patlamasına yol açtı. `DUBA_CAPI_M` şartname
        sabitidir, ayar değil (modül docstring'i: "eşleşme ölçüsü").
        """
        return math.hypot(self.x - x, self.y - y) <= (self.r + r + DUBA_CAPI_M)

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
        self._onaylanan = 0                    # teşhis: onaya ulaşan konum
        self._onay_bekleyen_kare = 0           # teşhis: onaysız turuncu kare
        self._kayitlar: List[HatirlananKenar] = []
        #: Son `siniflandir` çağrısındaki kenar sınıfı — `hatirlananlar()`
        #: bunu kullanır. Ayrı bir parametre DEĞİL (donmuş kural: ayar yok).
        self._edge_id: Optional[int] = None
        self._hatirlanarak_kurtarilan = 0      # teşhis: hafızanın kazandırdığı
        self._celiskiyle_silinen = 0           # teşhis: sınıf çelişkisi sayısı
        #: Teşhis (2026-08-10): kaç kayıt AÇILDI ve son yayımda kaçı menzil
        #: dışında kaldı. Sahada duplikasyonun tek görünürlük kanalı — `boyut`
        #: sürekli artarken `menzil_ici` sabit kalıyorsa konum sıçraması
        #: çakışma bandını aşıyordur (§0.26b).
        self._acilan_kayit = 0
        self._son_menzil_disi = 0
        #: KAR-11 (12.08): unutma ile silinen toplam kayit — teshis.
        self._unutulan = 0
        #: FAZ 1 (15.08): kayıt↔kayıt konsolidasyonuyla ERİTİLEN toplam kayıt
        #: (teşhis) ve kadans sayacı. Sayaç kadans değerinden başlar ki ilk
        #: `hatirlananlar()` çağrısı hemen bir tarama yapsın.
        self._birlestirilen = 0
        self._birlestirme_sayaci = _BIRLESTIRME_KADANSI

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

    @property
    def acilan_kayit(self) -> int:
        """Şimdiye kadar AÇILAN kayıt sayısı (yerinde güncellenenler hariç).

        `boyut` ile aynı büyür; ayrı tutuluyor çünkü sahada anlamlı olan
        **hız**: iki log penceresi arasındaki fark sıfıra inmiyorsa yeni
        cisim görülmüyor demektir, **duplikasyon** oluyordur (§0.26b).
        """
        return self._acilan_kayit

    @property
    def onaylanan(self) -> int:
        """F-A.1 teşhisi: M-of-N onayını geçip kenar dubası olan konum sayısı."""
        return self._onaylanan

    @property
    def onay_bekleyen_kare(self) -> int:
        """F-A.1 teşhisi: turuncu görülüp ONAYA ULAŞMADIĞI için engel olarak
        bırakılan kare sayısı. Sahada bu sayı yüksek ve `onaylanan` sıfırsa,
        kameranın turuncuları tek kare parlamalarıdır (yanlış pozitif)."""
        return self._onay_bekleyen_kare

    @property
    def unutulan(self) -> int:
        """Menzil dışına düştüğü için SİLİNEN toplam kayıt (KAR-11 teşhisi).

        `boyut` sabitlenirken bu sayaç artıyorsa unutma çalışıyor demektir.
        İkisi birlikte artıyorsa unutma menzili çok geniş; `boyut` artarken bu
        sıfır kalıyorsa unutma hiç devreye girmemiş (parametre verilmemiş).
        """
        return self._unutulan

    @property
    def son_menzil_disi(self) -> int:
        """Son `hatirlananlar()` çağrısında menzil dışı kaldığı için engel
        torbasına KONMAYAN kayıt sayısı."""
        return self._son_menzil_disi

    @property
    def birlestirilen(self) -> int:
        """FAZ 1 teşhisi: konsolidasyonla eritilen toplam kayıt sayısı.

        Sahada sürekli artıyorsa ikizler hâlâ doğuyor demektir (eşleşme bandı
        yetmiyor → odometri sıçramasına bak, KAR-06); sıfır kalıyorsa hafıza
        zaten temiz."""
        return self._birlestirilen

    def kayitlar(self) -> List[Tuple[float, float, float]]:
        """Hatırlanan kenarlar (x, y, r) — RViz/teşhis için kopya."""
        return [(k.x, k.y, k.r) for k in self._kayitlar]

    def temizle(self) -> None:
        """Parkur geçişinde/yeniden başlamada hafızayı sıfırla.

        md 5.5.3.1 yeniden başlamada ZORUNLU: araç fiilen başa döndüğü için
        eski kayıtlar artık aracın önünde değil arkasında/yanlış yerde;
        taşınırsa ikinci turda hayalet kenar dubaları kapı üretir.

        ⚠ Teşhis sayaçları (`hatirlanarak_kurtarilan`, `celiskiyle_silinen`)
        bilerek KORUNUR: koşum boyu kümülatif ölçüm, sıfırlanırsa hafızanın
        toplam katkısı kayıt dışı kalır. `_edge_id` de korunur — o yalnız bir
        önbellek, her `siniflandir()` çağrısında yeniden yazılıyor.
        """
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
        # 🔴 F-A.1: 1. geçişte İŞLENEN tespitler. `kenar` yetmez — onay
        # dolmadan kenar bayrağı False kalır ve tespit 2. geçişe düşüp AYNI
        # cisim için İKİNCİ kayıt açardı (ölçüldü: 2 direk → 3 kayıt).
        islenen: set[int] = set()
        for k in self._kayitlar:              # H1: tazelik her karede sıfırlanır
            k.taze = False

        # --- 1. geçiş: renk ŞU AN görünüyor ---------------------------------
        # 🔴 F-A.1: "şu an turuncu" TEK BAŞINA yetmez. Kayıt açılır/taşınır ama
        # kenar bayrağı ancak ONAY dolunca True olur; o ana kadar cisim engel
        # olarak kalır (güvenli varsayılan bozulmaz).
        for i, (x, y, r, cls) in enumerate(tespitler):
            if cls != edge_class_id:
                continue
            j = self._eslesen_kayit(x, y, r, haric=kullanilan)
            if j is None:
                self._kayitlar.append(
                    HatirlananKenar(x, y, r, sinif=None, taze=True)
                )
                j = len(self._kayitlar) - 1
                self._acilan_kayit += 1
            else:
                self._tasi(j, x, y, r, None)
            kayit = self._kayitlar[j]
            kayit.turuncu_sayaci += 1
            # 🔑 EŞİK DEĞİL, TEKRARIN ASGARİSİ. Ölçüldü (13.08, canlı kamera,
            # 80 kare / 8 945 tespit): kameranın ürettiği turuncuların tamamı
            # TEK KARE parlamasıydı — aynı konumda sonraki 3/5/10 karede
            # tekrar sayısı **0**. Yani "≥2 kez" bütün yanlış pozitifleri eler.
            # Gerçek dubaya bedeli bir kare (~0,12 s @8 kare/s), buna karşılık
            # §0.17e'nin ölçtüğü görünürlük penceresi ~6 s.
            if kayit.turuncu_sayaci >= 2:
                if kayit.sinif != edge_class_id:
                    self._onaylanan += 1
                kayit.sinif = edge_class_id       # ONAYLANDI → kenar dubası
                kenar[i] = True
            else:
                self._onay_bekleyen_kare += 1     # henüz engel olarak kalıyor
            kullanilan.add(j)
            islenen.add(i)

        # --- 2. geçiş: renk görünmüyor --------------------------------------
        for i, (x, y, r, cls) in enumerate(tespitler):
            if i in islenen:                  # 1. geçişte ele alındı
                continue
            bilinen_farkli = (
                cls is not None
                and cls != edge_class_id
                and cls != CLASS_UNKNOWN
            )
            j = self._eslesen_kayit(x, y, r, haric=kullanilan)
            if j is None:
                # 🆕 H1: eşleşen kayıt yok → YENİ kayıt aç. Hafıza artık yalnız
                # turuncu direkleri değil GÖRÜLEN HER CİSMİ tutuyor (sarı engel,
                # iskele, tekne, sınıfsız küme). Kenar bayrağı False kalır:
                # sınıfsız cisim engel olarak sürer (güvenlik kuralı bozulmadı).
                self._kayitlar.append(
                    HatirlananKenar(x, y, r, sinif=cls, taze=True)
                )
                self._acilan_kayit += 1
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
            # 🔴 F-A.1 — SÖNÜM YOK, BİLİNÇLİ. İki aday kural denendi:
            #  · "peş peşe" (turuncusuz karede sayacı sıfırla) ve
            #  · "±1 erime"
            # ikisi de kapalı alanda ÖLÇÜLEBİLEN yanlış pozitifi eliyor ama
            # ikisi de GÖLDE ölçülemeyen bir şeyi riske atıyor: kamera gerçek
            # dubayı aralıklı sınıflandırırsa onay HİÇ dolmaz. Test bunu
            # gösterdi: %50 turuncu görülen duba ±1 ile 1,0,1,0 salınıp asla
            # onaylanmıyor → §0.17e'nin ölçülmüş kazancı (12 m'de 1/4 → 4/4)
            # yok olurdu. Kapı geçişi PUANDIR; yanlış pozitifin bedeli ise
            # gereksiz bir kaçınmadır. Bu yüzden kanıt ERİMEZ.
            # ⚠ KALAN RİSK: birbirinden bağımsız İKİ parlama aynı konuma
            # (eşleşme bandı = tespitin kendi yarıçapı) düşerse onay dolar.
            # Ölçümde bunun izi YOK (80 karede tek turuncu, 0 tekrar) ama
            # imkânsız değil. Saha izleme kanalı: `onaylanan` sayacı —
            # ortalıkta gerçek turuncu duba yokken artıyorsa desen budur.
            self._kayitlar[j].gorulme += 1
            if kenar[i]:
                self._hatirlanarak_kurtarilan += 1
            kullanilan.add(j)

        return kenar

    def hatirlananlar(
        self,
        arac_xy: Optional[Tuple[float, float]] = None,
        menzil: Optional[float] = None,
        unutma_menzili: Optional[float] = None,
    ) -> List[Tuple[Tespit, bool]]:
        """🆕 H1 — bu karede GÖRÜLMEYEN kayıtlar `((x, y, r, sınıf), kenar_mı)`.

        `arac_xy` + `menzil` verilirse yalnız **araçtan `menzil` içindeki**
        kayıtlar döner (2026-08-10, modül docstring'i "YAYIM MENZİLİ").
        Kayıt SİLİNMEZ — yalnız engel torbasına konmaz; araç yaklaşınca
        kendiliğinden geri gelir. İkisinden biri `None` ise eski davranış.

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
        # 🔴 KAR-11 (12.08 ÖLÇÜLDÜ): unutma yoksa torba SINIRSIZ büyür.
        # Canlı Jetson'da `boyut` **2404 kayıt**a çıkmıştı ve hâlâ artıyordu;
        # `siniflandir()` her algı karesinde her tespiti HER kayda karşı test
        # ettiği için maliyet doğrusal büyüyor → kontrol döngüsü 117 ms'den
        # **1062 ms**'e çıkıyor (9×, ölçüldü). Tekne hareketsizken bile büyümesi,
        # odometri sıçramasının (KAR-06: 25 ms'de 6,54 m) aynı dubayı tekrar
        # tekrar kaydettirmesinden.
        # `unutma_menzili` verilirse o menzilin ÖTESİNDEKİ kayıtlar SİLİNİR —
        # süzmek yetmez, çünkü maliyet yayımda değil TARAMADA.
        # ⚠ 09.08'de "unutma YOK" bilinçli bir karardı: duba geçici olarak
        # görünmez olunca (LiDAR menzili ~8 m) hafıza onu kurtarıyor. O karara
        # dokunmuyoruz — unutma menzili yayım menzilinden BELİRGİN BÜYÜK
        # seçilir, yani "hâlâ işimize yarayabilecek" hiçbir kayıt silinmez;
        # yalnız aracın çok gerisinde kalmış, bir daha kullanılmayacak
        # kopyalar düşer.
        # FAZ 1 (§1.13c): kayıt↔kayıt konsolidasyonu — kadanslı, tarama zaten
        # bu çağrının işi olduğu için burada. Tespit↔kayıt bandı genişletilse
        # de iki kayıt SONRADAN birbirinin bandına sürüklenebilir (odometri
        # oynaması, kısmi görüş); literatürde de kopya işaretçi yönetimi ayrı
        # bir katmandır (§1.15c). Aksiyom aynı: çakışan uyumlu kayıtlar tek
        # cisimdir.
        self._birlestirme_sayaci += 1
        if self._birlestirme_sayaci >= _BIRLESTIRME_KADANSI:
            self._birlestirme_sayaci = 0
            self._birlestir()
        if arac_xy is not None and unutma_menzili is not None:
            ax0, ay0 = arac_xy
            kalan = [
                k for k in self._kayitlar
                if math.hypot(k.x - ax0, k.y - ay0) <= unutma_menzili
            ]
            self._unutulan += len(self._kayitlar) - len(kalan)
            self._kayitlar = kalan
        gorulmeyenler = [k for k in self._kayitlar if not k.taze]
        if arac_xy is None or menzil is None:
            self._son_menzil_disi = 0
            secilen = gorulmeyenler
        else:
            ax, ay = arac_xy
            secilen = [
                k for k in gorulmeyenler
                if math.hypot(k.x - ax, k.y - ay) <= menzil
            ]
            self._son_menzil_disi = len(gorulmeyenler) - len(secilen)
        return [
            ((k.x, k.y, k.r, k.sinif), k.sinif is not None and k.sinif == self._edge_id)
            for k in secilen
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

    def _birlestir(self) -> int:
        """FAZ 1 (§1.13) — çakışan uyumlu kayıtları tek kayda erit.

        Ölçüt `cakisiyor` ile AYNI (r_a + r_b + duba_çapı): tespit↔kayıt için
        doğru olan aksiyom kayıt↔kayıt için de doğrudur — iki katı cisim aynı
        yeri kaplayamaz. Göl bandında ölçüldü (§1.13h): bu birleştirme
        yayımlanan kayıtları 230→104'e, paysız direk oranını %78→%45'e indirir
        (kalanı `_huni_payi` düzeltmesi kapatır, FAZ 2).

        Kurallar:
          · Bilinen ve FARKLI iki sınıf → **birleşmez** (çelişki güvenliği:
            turuncu direğin dibindeki sarı engel yutulmaz, ikisi de kalır).
          · Merkez `gorulme` ağırlıklı ortalama (çok görülen kayıt daha
            güvenilir), yarıçap büyük olan (güvenli taraf), `gorulme` ve
            `turuncu_sayaci` toplanır (ikizler aynı dubanın kareleridir).
          · Tek geçiş, açgözlü: zincirler kadanslı tekrar taramalarla çöker.
        """
        eritilen = 0
        yeni: List[HatirlananKenar] = []
        for k in self._kayitlar:
            hedef = None
            for m in yeni:
                if not m.cakisiyor(k.x, k.y, k.r):
                    continue
                bilinen_m = m.sinif is not None and m.sinif != CLASS_UNKNOWN
                bilinen_k = k.sinif is not None and k.sinif != CLASS_UNKNOWN
                if bilinen_m and bilinen_k and m.sinif != k.sinif:
                    continue                      # sınıf çelişkisi: ikisi de kalır
                hedef = m
                break
            if hedef is None:
                yeni.append(k)
                continue
            w = hedef.gorulme + k.gorulme
            hedef.x = (hedef.x * hedef.gorulme + k.x * k.gorulme) / w
            hedef.y = (hedef.y * hedef.gorulme + k.y * k.gorulme) / w
            hedef.r = max(hedef.r, k.r)
            hedef.gorulme = w
            hedef.turuncu_sayaci += k.turuncu_sayaci
            hedef.taze = hedef.taze or k.taze
            if k.sinif is not None and k.sinif != CLASS_UNKNOWN and (
                hedef.sinif is None or hedef.sinif == CLASS_UNKNOWN
            ):
                hedef.sinif = k.sinif             # bilinen sınıf kazanır
            eritilen += 1
        self._kayitlar = yeni
        self._birlestirilen += eritilen
        return eritilen

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
