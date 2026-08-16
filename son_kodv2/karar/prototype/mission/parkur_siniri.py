r"""
Girdap İDA — PARKUR SINIRI: kenar duba zincirinden türetilen koridor (F-S.16).

🔑 **Neden bu modül var — 54 puan ve sistemde kavram olarak yoktu.**
Şartname s.24-25: Parkur-1 `(−24×PDÇ1)/4 + 24`, Parkur-2 `(−30×PDÇ2)/5 + 30`
→ her **parkur dışına çıkış 6 puan**, dışarıda **40 saniyeden** uzun kalmak
**2 çıkış** sayılıyor. Kodda `parkur_disi` diye bir kavram yoktu: sistem
koridorun içinde mi dışında mı olduğunu bilmiyordu (GIRDAP_DURUM §0.80).

🔑 **Neden sabit bir geofence DEĞİL.** md 5.5.2.2: *"Yarışma parkurlarının
önceden haritalanmasına izin verilmeyecektir"* + *"Engellerin konum bilgileri
paylaşılmayacaktır."* ⇒ sınır **o an görülen kenar dubalarından** türetilmek
zorunda. Bu modül tam olarak onu yapar ve **hiçbir ayarlanabilir eşik
içermez** (§0.0d kuralı): girdisi yalnız kapı çiftleridir, ölçüsü kapıların
kendi genişliğidir.

📐 **Geometri — koridor, orta çizgi TÜPÜ değil, İKİ KENAR ZİNCİRİ arasıdır.**
Kapı `i` iki kenar dubası verir: `sol_i` (+yanal) ve `sag_i` (−yanal). Ardışık
kapıların aynı taraflı dubaları birleştirilince iki kırık çizgi çıkar; parkur
bu ikisinin ARASIDIR:

    sol_1 ── sol_2 ── sol_3 ── … ── sol_n        ← sol kenar zinciri
      │                                 │
      │        P A R K U R   İ Ç İ      │
      │                                 │
    sag_1 ── sag_2 ── sag_3 ── … ── sag_n        ← sağ kenar zinciri

Orta çizgi + yarı genişlik "tüpü" (§0.80c'nin ilk önerisi) zigzag parkurda
kapı kirişlerine göre **eğik** durur; kenar zinciri ise dubaların kendisinden
geçtiği için parkurun **tanımının kendisidir**. Yarı genişlik ölçüsü zaten
`Gate.passed_between`'in kullandığı ölçüyle aynı: kiriş üzerinde
`|yanal| ≤ genişlik/2` içeridedir. Yani yeni bir ölçü icat edilmiyor.

⚠ **KAPSAM DIŞI üçüncü bir durumdur, "dışarıda" DEĞİL.** İlk kapının önünde
ve son kapının ötesinde kenar dubası yoktur → orada sınır da yoktur. Bu ayrım
şart: aksi hâlde fırlatma noktasındaki (henüz parkura girmemiş) araç sürekli
"parkur dışı" sayılırdı — ve F-S.16'nın MPPI ayağı bağlandığında araç daha
başlamadan duvara toslardı (§0.86a'nın `w_boundary` duvarı dersi).

⚠ **VARSAYIM — ardışık kenar dubaları arası DÜZ çizgi.** Şartname "parkur
dışı"nın kenar dubaları arasında tam olarak hangi eğriden geçtiğini
söylemiyor; iki komşu duba arasını düz birleştirmek en dar (en kötümser)
makul yorumdur. Ölçüm bu yüzden PDÇ'yi **abartabilir**, eksik saymaz —
kötümser taraf doğru taraftır. Hakem daha geniş yorumlarsa gerçek puan bu
ölçümden yalnız İYİ çıkar.

⚠ **Sıralama çağıranın sorumluluğu:** `kapilar` parkur boyunca sıralı ve
sol/sağ etiketleri tutarlı olmalı. Ölçüm tarafında sahne dosyası zaten sıralı
verir; araç tarafında `GateFollower` sol/sağ'ı kursa göre etiketler.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import List, Optional, Sequence, Tuple

Point = Tuple[float, float]
Kapi = Tuple[Point, Point]          # (sol, sağ)

ICERIDE = "ICERIDE"
DISARIDA = "DISARIDA"
KAPSAM_DISI = "KAPSAM_DISI"

# Boylamsal kapsam konumu (bkz. `ParkurSiniri.kapsam_konumu`)
ONCE = "ONCE"
ICINDE = "ICINDE"
SONRA = "SONRA"

# Şartname s.25 — dışarıda bu süreden uzun kalmak İKİ çıkış sayılır.
# Ayar değil, şartnamenin sayısı; sahada değişmez.
UZUN_KALIS_S = 40.0


def _birim(dx: float, dy: float) -> Optional[Point]:
    n = math.hypot(dx, dy)
    if n < 1e-9:
        return None
    return (dx / n, dy / n)


def _parcaya_uzaklik(p: Point, a: Point, b: Point) -> float:
    """Noktanın [a,b] parçasına en kısa uzaklığı (m)."""
    abx, aby = b[0] - a[0], b[1] - a[1]
    apx, apy = p[0] - a[0], p[1] - a[1]
    uz2 = abx * abx + aby * aby
    t = 0.0 if uz2 < 1e-12 else max(0.0, min(1.0, (apx * abx + apy * aby) / uz2))
    return math.hypot(apx - t * abx, apy - t * aby)


def _nokta_parcada(p: Point, a: Point, b: Point, tol: float = 1e-9) -> bool:
    """Nokta [a,b] parçasının üstünde mi (sayısal tolerans içinde)."""
    apx, apy = p[0] - a[0], p[1] - a[1]
    abx, aby = b[0] - a[0], b[1] - a[1]
    uzunluk2 = abx * abx + aby * aby
    if uzunluk2 < tol:
        return math.hypot(apx, apy) <= tol
    t = (apx * abx + apy * aby) / uzunluk2
    if t < -tol or t > 1.0 + tol:
        return False
    return abs(apx * aby - apy * abx) / math.sqrt(uzunluk2) <= tol


def tutarli_yonlendir(kapilar: Sequence[Kapi]) -> List[Kapi]:
    """Ardışık kapıların sol/sağ etiketlerini ÇAPRAZLANMAYACAK şekilde hizala.

    🔴 **Neden gerekli (algı tarafının tuzağı):** `GateFollower` sol/sağ'ı
    **aracın o kapıya yaklaşma yönüne** göre etiketler (`select_gate`:
    *"+lateral = sol; Δyanal'ın işareti hangi dubanın solda olduğunu doğrudan
    verir, kurs eksenine göre değil"*). Bu kapı seçimi için doğrudur ama
    zincir için değil: kapıya ters/eğik yaklaşılan tek bir kare, o kapının
    etiketlerini komşularına göre **ters** yazar. Ters çift koridor çokgenini
    **kelebeğe** çevirir ve içeride/dışarıda testi anlamsızlaşır — üstelik
    sessizce.

    🔑 **Ölçüt eşik değil, saf geometri:** iki komşu kapıyı bağlamanın iki
    yolu vardır (düz ve çapraz); çaprazın toplam uzunluğu daha kısaysa
    etiketler terstir. Ölçek-bağımsız, ayarlanacak sayı yok (§0.0d).
    """
    duzeltilmis: List[Kapi] = []
    for i, (sol, sag) in enumerate(kapilar):
        if i == 0:
            duzeltilmis.append((sol, sag))
            continue
        onceki_sol, onceki_sag = duzeltilmis[-1]
        duz = math.dist(sol, onceki_sol) + math.dist(sag, onceki_sag)
        capraz = math.dist(sol, onceki_sag) + math.dist(sag, onceki_sol)
        duzeltilmis.append((sag, sol) if capraz < duz else (sol, sag))
    return duzeltilmis


@dataclass(frozen=True)
class ParkurSiniri:
    """Sıralı kapı çiftlerinden kurulan parkur koridoru.

    En az **iki** kapı gerekir: tek kapı bir kiriştir, alanı yoktur.
    """

    kapilar: Tuple[Kapi, ...]

    # ---- kurucu ----

    @classmethod
    def kapilardan(cls, kapilar: Sequence[Kapi]) -> Optional["ParkurSiniri"]:
        """Sınırı kur; iki kapıdan azsa **None** (koridor tanımsız).

        None dönmesi bir hata değil, geçerli bir hâldir: kapı görülmeden
        sınır da yoktur, ceza da uygulanmamalıdır (§0.80c).
        """
        if len(kapilar) < 2:
            return None
        return cls(tuple((tuple(s), tuple(g)) for s, g in kapilar))  # type: ignore[misc]

    # ---- geometri ----

    @property
    def ortalar(self) -> List[Point]:
        return [((s[0] + g[0]) / 2.0, (s[1] + g[1]) / 2.0) for s, g in self.kapilar]

    @property
    def cokgen(self) -> List[Point]:
        """Koridor çokgeni: sol zincir ileri, sağ zincir geri (kapalı halka)."""
        sol = [s for s, _ in self.kapilar]
        sag = [g for _, g in self.kapilar]
        return sol + sag[::-1]

    def _kapi_normali(self, i: int, kurs: Point) -> Optional[Point]:
        """Kapı `i`'nin KENDİ normali (kirişe dik), kurs yönüne göre işaretli.

        ⚠ **Kapı ortalarının yönü DEĞİL.** İlk denemede kapsam düzlemleri
        komşu kapı ortalarından türetilmişti; zigzag parkurda son ayak
        (30,5)→(34,−1) diyagonal olduğu için o düzlem geriye doğru süpürüp
        kapı-5'in **yanındaki** noktayı "kapsam dışı" ilan etti — yani
        gerçek bir parkur dışı çıkış ölçüme hiç girmiyordu. Kapının kendi
        kirişine dik normal kurstan bağımsızdır; yalnız İŞARETİNİ kurstan
        alır (aynı gerekçe `Gate.signed_distance`'ta da yazılı, B4/B6).
        """
        sol, sag = self.kapilar[i]
        c = _birim(sol[0] - sag[0], sol[1] - sag[1])
        if c is None:
            return None
        n = (c[1], -c[0])
        return n if (n[0] * kurs[0] + n[1] * kurs[1]) >= 0.0 else (-n[0], -n[1])

    def kapsamda_mi(self, p: Point) -> bool:
        """Nokta ilk ve son kapı DÜZLEMLERİNİN arasında mı (boylamsal kapsam)."""
        return self.kapsam_konumu(p) == ICINDE

    def kapsam_konumu(self, p: Point) -> str:
        """Boylamsal konum: `ONCE` (girişten önce) · `ICINDE` · `SONRA` (bitişten sonra).

        🔑 **İki kapsam dışı hâli AYNI ŞEY DEĞİL** — 14.08 ölçümünde bu ayrım
        eksikken zorlayıcı koşum-2 gerçekte 43 s dışarıdayken **107 s**
        yazıldı: tekne son kapıyı geçip GN5'e (Parkur-2) giderken açık kalan
        epizot koşum sonuna kadar süre biriktirdi. `SONRA` parkurun BİTİŞİDİR,
        dışarıda kalmak değildir. `ONCE` ise gerçekten kaçıştır: yanından
        çıkıp bir daha dönmeyen araç hâlâ parkur dışındadır.

        Kapsam dışında sınır TANIMSIZDIR; ne içeride ne dışarıda sayılır.
        """
        ortalar = self.ortalar
        ilk_kurs = _birim(ortalar[1][0] - ortalar[0][0], ortalar[1][1] - ortalar[0][1])
        son_kurs = _birim(
            ortalar[-1][0] - ortalar[-2][0], ortalar[-1][1] - ortalar[-2][1]
        )
        if ilk_kurs is None or son_kurs is None:
            return ONCE
        giris = self._kapi_normali(0, ilk_kurs)
        cikis = self._kapi_normali(len(self.kapilar) - 1, son_kurs)
        if giris is None or cikis is None:
            return ONCE
        onde = (p[0] - ortalar[0][0]) * giris[0] + (p[1] - ortalar[0][1]) * giris[1]
        gecti = (p[0] - ortalar[-1][0]) * cikis[0] + (p[1] - ortalar[-1][1]) * cikis[1]
        if gecti > 0.0:
            return SONRA
        return ICINDE if onde >= 0.0 else ONCE

    def iceride_mi(self, p: Point) -> bool:
        """Nokta koridor çokgeninin içinde mi (ışın atma; KENAR içeri sayılır).

        Kenarın içeri sayılması bilinçli: sınır çizgisi kenar dubalarının
        merkezlerinden geçer, tam üstünde olmak "dışarı çıkmış" değildir.
        """
        x, y = p
        cok = self.cokgen
        icinde = False
        n = len(cok)
        for i in range(n):
            x1, y1 = cok[i]
            x2, y2 = cok[(i + 1) % n]
            if _nokta_parcada(p, (x1, y1), (x2, y2)):
                return True                      # tam sınırın üstünde
            if (y1 > y) != (y2 > y):
                kesim_x = x1 + (y - y1) * (x2 - x1) / (y2 - y1)
                if x < kesim_x:
                    icinde = not icinde
        return icinde

    def kenara_uzaklik(self, p: Point) -> float:
        """Noktanın koridor kenarına en kısa uzaklığı (m) — işaretsiz.

        🔑 **Neden şart:** PDÇ ikili bir sayıdır; sınırı 20 cm aşmakla 5 m
        dışarı savrulmak aynı görünür. Ardışık iki duba arasını düz çizgiyle
        birleştirmek bir YORUMDUR (modül docstring'i); yorumun payından küçük
        bir aşma hakemin "parkur dışı" diyeceği şey değildir. Derinlik bu
        ikisini ayırır — §0.86a'daki *"SPL %107"* dersi: tek başına anlamsız
        bir sayı iki kez tuzağa düşürdü.
        """
        cok = self.cokgen
        return min(
            _parcaya_uzaklik(p, cok[i], cok[(i + 1) % len(cok)])
            for i in range(len(cok))
        )

    def durum(self, p: Point) -> str:
        """`ICERIDE` · `DISARIDA` · `KAPSAM_DISI` (bkz. modül docstring'i)."""
        if not self.kapsamda_mi(p):
            return KAPSAM_DISI
        return ICERIDE if self.iceride_mi(p) else DISARIDA


@dataclass
class ParkurDisiSayaci:
    """Şartnamenin PDÇ kuralını uygulayan sayaç (s.24-25).

    Kurallar tek yerde: çıkış = **içeriden dışarıya** geçiş; dışarıda
    `UZUN_KALIS_S`'ten uzun kalınan her epizot **iki** çıkış sayılır.

    ⚠ **Parkura girmeden önce sayılmaz.** Araç fırlatma noktasında koridorun
    boylamsal kapsamı dışındadır; ilk kez içeri girene kadar sayaç uyur.
    Aksi hâlde her koşum daha başlamadan bir "çıkış" yazardı.
    """

    sinir: Optional[ParkurSiniri]
    _girdi: bool = False
    _disarida_mi: bool = False
    _epizot_basi: float = 0.0
    _epizot_derinlik: float = 0.0
    epizotlar: List[float] = field(default_factory=list)
    derinlikler: List[float] = field(default_factory=list)

    def adim(self, p: Point, t: float) -> str:
        """Bir zaman adımı işle; o adımın durumunu döndür."""
        if self.sinir is None:
            return KAPSAM_DISI
        d = self.sinir.durum(p)
        if d == ICERIDE:
            self._kapat(t)
            self._girdi = True
        elif d == KAPSAM_DISI and self.sinir.kapsam_konumu(p) == SONRA:
            # Parkur BİTTİ (son kapı düzleminin ötesi) — açık epizot burada
            # kapanır, koşumun kalanı "dışarıda" sayılmaz.
            self._kapat(t)
        elif d == DISARIDA and self._girdi:
            if not self._disarida_mi:
                self._disarida_mi = True
                self._epizot_basi = t
                self._epizot_derinlik = 0.0
            self._epizot_derinlik = max(
                self._epizot_derinlik, self.sinir.kenara_uzaklik(p)
            )
        return d

    def _kapat(self, t: float) -> None:
        if self._disarida_mi:
            self.epizotlar.append(t - self._epizot_basi)
            self.derinlikler.append(self._epizot_derinlik)
            self._disarida_mi = False

    def bitir(self, t: float) -> None:
        """Koşum sonunda açık epizodu kapat (dışarıda bitmiş olabilir)."""
        self._kapat(t)

    # ---- ölçüler ----

    @property
    def cikis_sayisi(self) -> int:
        """Ham çıkış sayısı (epizot adedi)."""
        return len(self.epizotlar)

    @property
    def etkin_cikis(self) -> int:
        """Şartnamenin saydığı PDÇ — 40 sn'den uzun epizot İKİ sayılır."""
        return sum(2 if s > UZUN_KALIS_S else 1 for s in self.epizotlar)

    @property
    def toplam_sure(self) -> float:
        return sum(self.epizotlar)

    @property
    def en_uzun_sure(self) -> float:
        return max(self.epizotlar) if self.epizotlar else 0.0

    @property
    def en_derin(self) -> float:
        """En derin aşma (m) — sığ aşmayı gerçek savrulmadan ayırır."""
        return max(self.derinlikler) if self.derinlikler else 0.0

    def puan(self, parkur: int) -> float:
        """Şartnamenin formülü: P1 `(−24×PDÇ)/4+24`, P2 `(−30×PDÇ)/5+30`.

        Şartname PDÇ'yi P1'de [0:4], P2'de [0:5] aralığında tanımlıyor; iki
        formül de o tavanda tam sıfır verdiği için ayrıca kırpmaya gerek yok —
        `max(0, …)` tavanın kendisidir. (Kırpma satırı yazılmıştı, mutasyon
        turunda **ölü kod** olduğu görüldü ve atıldı.)
        """
        if parkur == 1:
            tam, tavan = 24.0, 4
        elif parkur == 2:
            tam, tavan = 30.0, 5
        else:
            raise ValueError("PDÇ yalnız Parkur-1 ve Parkur-2 için tanımlı")
        return max(0.0, (-tam * self.etkin_cikis) / tavan + tam)
