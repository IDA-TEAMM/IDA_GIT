"""
Girdap İDA — Duba kapısı takibi (gate-following) çekirdeği, ROS-bağımsız.

PROBLEM (Şartname md 5.5.2.2 / Şekil 2): Parkur-1 ve Parkur-2'de puan,
GPS görev noktasına (GN) tam basmaktan DEĞİL, karşılıklı KENAR dubası ikilisinin
ARASINDAN geçmekten gelir (G1/G2 = geçiş sayısı). Üstelik hakemin verdiği görev
noktası "doğrudan iki kenar dubasının arasında bir nokta olmayabilir" (md
5.5.2.2) ve dubaların konumu paylaşılmaz / önceden haritalanamaz. Yani ham GPS
noktasına yönelmek YETMEZ: araç, kamera/füzyonla algıladığı kenar dubalarından
kapı orta noktasını KENDİSİ hesaplayıp oradan geçmelidir.

BU MODÜL: algılanan turuncu KENAR dubalarından (class "0") öndeki kapıyı seçer
ve geçilecek noktayı "rafine hedef" olarak döndürür. Kapı görünmüyorsa ham GPS
görev noktasına düşer (fallback) — böylece kapı görüş menziline girene kadar
araç yine de doğru genel yöne ilerler.

🔑 HEDEF KÖR ORTA NOKTA DEĞİL: geçilecek nokta kapı kirişi üzerinde
ENGELLERE GÖRE kaydırılır (`aim_point`). Sebebi mimari: kenar dubaları MPPI'nin
engel torbasından çıkarılmak ZORUNDA (ceza halkası geçidin içini kaplar ve araç
kapıdan geçmeyi dolanmaktan pahalı bulur) — ama çıkarılınca geçitte dubadan
İTEN hiçbir kuvvet kalmaz, yalnız ortanın çekimi vardır. İtme bu yüzden MPPI
maliyetine değil HEDEFİN KENDİSİNE gömülür. Engel yokken nişan tam ortadadır,
yani kapısız/engelsiz senaryoda davranış birebir eskisi gibidir.

MİMARİ YERİ: mission_manager'ın `current_target`'ını (ham GN) RAFİNE eder;
RRT*/MPPI zinciri değişmez, sadece daha doğru bir hedef alır. SARI engel
dubaları (class "1") bu modüle GİRMEZ — onlar planning'de CircleObstacle olarak
kaçınma nesnesi olarak kalır. Turuncu kenar dubaları ise "kapı" olur (arasından
geçilir), engel torbasından çıkarılır (bkz. Layer 2 node entegrasyon notu).

FRAME: tek bir 2D dünya ENU çerçevesi (x=doğu, y=kuzey). Tüm girdiler AYNI
çerçevede olmalı; base_link→ENU dönüşümü Layer 2 node'un işidir (çekirdek
frame-bağımsız kalır, pytest ile rclpy'siz doğrulanır — parkur_fsm.py deseni).
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import List, Optional, Sequence, Tuple

Point = Tuple[float, float]
# Dairesel engel: (merkez_x, merkez_y, yarıçap) — dünya ENU, metre.
Circle = Tuple[float, float, float]

# Şartname md 5.5.2.1: kenar/engel dubası çapı 30 cm. Şekil 3'ün aksine bu
# ÖLÇÜLMÜŞ değil ŞARTNAME sabiti — metinde kesin verilen iki sayıdan biri
# (diğeri 50 cm yükseklik), dolayısıyla "tahmine dayalı sayı yok" kuralını
# bozmaz. Dubalar merkezlerinden algılandığı için serbest açıklık hesabında
# yarıçapın iki kez düşülmesi gerekir.
BUOY_RADIUS_M = 0.15

# Nişan noktası aramasında kiriş başına en fazla kaç adım denenir (hesap
# kapağı, davranış ayarı DEĞİL): çok geniş kapıda örnek sayısı patlamasın.
# Adım gövde genişliğinden türer, bu yalnız üst sınır.
_AIM_MAX_ADIM = 64


@dataclass(frozen=True)
class GateFollowerConfig:
    r"""Kapı seçimi — **TAHMİNE DAYALI TEK BİR SAYI YOK.**

    🔴 **Neden:** kapı geometrisi önceden bilinemez. Şartname üç yerde söylüyor:
      · *"Parkurların uzunlukları, parkurlarda kullanılan duba sayıları ve
        KENAR DUBALARI ARASINDAKİ MESAFELER yarışma alanına göre değişkenlik
        gösterecektir."*
      · *"Dubalar arasındaki mesafeler, duba sayıları, parkur uzunluğu yarışma
        alanına göre BELİRLENECEKTİR. Duba sayılarına göre bir akış
        tasarlanmaması tavsiye edilmektedir."*
      · *"kenar dubaları ve engeller de DENİZ ŞARTLARINDAN DOLAYI YER
        DEĞİŞTİREBİLİR."* → koşu SIRASINDA bile sabit değil.
    Parkur önceden görülemez, önceden haritalama zaten yasak. Dolayısıyla
    "sahada ölçüp gir" diye bir eşik OLAMAZ — ölçülecek bir şey yok.

    **Tasarım kuralı (2026-08-03): her sayı ya ÖLÇÜLMÜŞ bir tekne boyutudur ya
    da saf geometridir. Serbest/ayarlanabilir eşik bırakılmadı.**

    | eski (tahmin) | yerine geçen |
    |---|---|
    | `gate_width_max` = 20 m | **YOK** — genişlik hiç kısıt değil |
    | `gate_width_min` = 1.0 m | **gövde genişliği + 2×duba yarıçapı** (fizik: dar kapıdan geçilemez; dubalar merkezden algılanır) |
    | `max_lookahead` = 25 m | **YOK** — menzili algı katmanı belirler |
    | `pair_depth_tol` = 3.0 m | **\|Δileri\| < \|Δyanal\|** (45° geometrik ayrım) |
    | `min_forward` = 0.5 m | **gövde yarı boyu** (burnun önü) |
    | `release_distance` = 0.8 m | **ileri mesafe ≤ 0** (kapı düzlemi geçildi) |
    | `match_radius` = 2.5 m | **kapının kendi genişliğinin yarısı** (öz-ölçekli) |

    🔑 **"Yan yana mı" testi neden `|Δileri| < |Δyanal|`:** bir kapı, kursa
    DİK duran bir duba çiftidir; ardışık iki kapının dubaları ise kurs boyunca
    dizilir. Bu eşitsizlik "aradaki doğru paralel olmaktan çok dike yakın mı"
    demektir — kesişim noktası tam 45°, yani **ayarlanan bir eşik değil, iki
    hâl arasındaki geometrik ayrım noktası**. Ölçek-bağımsız: kapı 2 m de olsa
    40 m de olsa aynı çalışır. Aynı zamanda tek dubası görünmeyen bir kapının
    yanlışlıkla komşu kapıyla eşleşmesini de eler (onlar ileri yönde ayrıktır).

    Tekne boyutları `~/Desktop/GIRDAP_DURUM.md` §1'den (ölçülmüş):
    0,78 × 1,04 × 0,52 m.
    """

    # ÖLÇÜLMÜŞ tekne boyutları — tek girdi bunlar, tahmin değil.
    # Genişlik: bundan dar bir açıklıktan tekne fiziksel olarak GEÇEMEZ, o
    # yüzden öyle bir çift kapı değildir (büyük olasılıkla tek dubanın iki
    # tespite bölünmesidir). Bu bir eşik ayarı değil, geçilebilirlik testi.
    hull_width_m: float = 0.78
    # Boy: dubanın "önümüzde" sayılması için burnun ötesinde olması gerekir.
    hull_length_m: float = 1.04

    @property
    def min_forward(self) -> float:
        """Duba en az bu kadar önde olmalı = gövde yarı boyu (burun hattı)."""
        return self.hull_length_m / 2.0

    @property
    def half_beam(self) -> float:
        """Gövde yarı genişliği — kiriş üzerindeki ölü bandın bir bileşeni."""
        return self.hull_width_m / 2.0

    @property
    def min_passable_width(self) -> float:
        """Geçilebilir en dar kapı: MERKEZDEN merkeze mesafe.

        🔴 Dubalar MERKEZLERİNDEN algılanır ama gövde duba YÜZEYLERİ arasından
        geçer: serbest açıklık = `merkez_mesafe − 2r`. Yalnız `hull_width_m`
        ile karşılaştırmak duba çapı kadar (30 cm) GEÇ kapanan bir süzgeçtir →
        `[0.78 ; 1.08)` aralığındaki çiftler geçilemez oldukları hâlde kapı
        sayılır, araç sığmayacağı bir orta noktaya nişan alır ve **iki dubaya
        birden çarpar**. Üstelik tam bu aralık sahte çiftlerin (tek dubanın iki
        kümeye bölünmesi, su yansıması) düştüğü yerdir.
        """
        return self.hull_width_m + 2.0 * BUOY_RADIUS_M


@dataclass(frozen=True)
class Gate:
    """Seçilmiş bir kapı: iki kenar dubası + orta nokta (hepsi dünya ENU).

    `midpoint` KİMLİKTİR, `aim` HEDEFTİR — ikisi bilerek ayrı:
      · `midpoint` = saf geometrik orta. Kapının değişmez kimliği; "aynı kapı
        mı" eşleşmesi ve "geçildi mi" testi buna bakar. Engel gelip gidince
        kaymaz, yoksa kilit her tick'te kendi kendine kırılırdı.
      · `aim` = fiilen sürülecek nokta; kiriş üzerinde engellerden en açık yer
        (bkz. `aim_point`). Engel yokken `midpoint` ile BİREBİR aynıdır.
    """

    left: Point       # kursa göre SOL kenar dubası (+lateral)
    right: Point      # kursa göre SAĞ kenar dubası (-lateral)
    midpoint: Point   # geometrik orta — kapının kimliği
    # Engele göre ayarlanmış nişan noktası; None → engel bilgisi yok, orta kullan.
    aim: Optional[Point] = None
    # Kapının KENDİ ileri normali (birim, kirişe dik). Kilitlenme anındaki
    # gidiş yönüne göre işaretlenir. None → eski kurs-ekseni testine düşülür.
    normal: Optional[Point] = None

    @property
    def width(self) -> float:
        """Kenar dubaları arası mesafe (m) — MERKEZDEN merkeze."""
        return math.hypot(self.left[0] - self.right[0], self.left[1] - self.right[1])

    @property
    def drive_target(self) -> Point:
        """Sürülecek nokta: nişan varsa o, yoksa geometrik orta."""
        return self.aim if self.aim is not None else self.midpoint

    @property
    def aim_shift(self) -> float:
        """Nişanın geometrik ortadan kayma miktarı (m) — saha teşhisi."""
        ax, ay = self.drive_target
        return math.hypot(ax - self.midpoint[0], ay - self.midpoint[1])

    def signed_distance(self, vehicle: Point) -> Optional[float]:
        r"""Aracın kapı DÜZLEMİNE işaretli mesafesi (m). Normal yoksa None.

        `> 0` → kapı düzlemi geçildi (araç kapının ötesinde),
        `< 0` → kapıya hâlâ gidiliyor.

        🔑 **Neden kurs ekseni DEĞİL, kapının kendi normali** (B4/B6):
        eski test `(orta − araç)·f` idi; `f` araçtan ham görev noktasına (GN)
        bakan birim vektör. İki ayrı arızası vardı:
          1. Şartname *"görev noktası doğrudan iki kenar dubasının arasında bir
             nokta OLMAYABİLİR"* diyor (md 5.5.2.2). **GN yana kaçıksa** araç
             kapıdan fiilen geçtiği hâlde izdüşüm pozitif kalabilir → kilit
             çözülmez, araç geçtiği kapıya GERİ DÖNER.
          2. GN'ye yaklaşırken `f` hızla döner (1 m kala 0,3 m yanal hata ≈ 17°)
             → eşik kayar, seçim tick'ler arasında zıplar.
        Kapının kendi normali kurstan da ölçekten de bağımsızdır; yalnız
        İŞARETİ kilitlenme anındaki gidiş yönünden alınır ve araç 90°'den
        fazla dönmedikçe sabit kalır.
        """
        if self.normal is None:
            return None
        nx, ny = self.normal
        return (vehicle[0] - self.midpoint[0]) * nx + (
            vehicle[1] - self.midpoint[1]
        ) * ny

    def passed_between(self, vehicle: Point) -> bool:
        """Araç kapının ARASINDAN mı geçti? (düzlemi aşmak YETMEZ)

        İki şart: (1) düzlem geçildi, (2) yanal sapma kapının yarı
        genişliğini aşmıyor — yani direklerin ARASINDAN geçildi. (2) olmazsa
        kapıyı yandan DOLAŞAN araç da "geçti" sayılırdı; oysa şartnameye göre
        bu geçiş değildir (G1/G2 sayılmaz) ve üstüne parkur-dışı cezası gelir.

        ⚠ Bu, kilidi bırakma testinden AYRIDIR: dolaşarak geçilen kapı da
        bırakılır (arkada kaldı), ama SAYILMAZ.
        """
        d = self.signed_distance(vehicle)
        if d is None or d <= 0.0:
            return False
        nx, ny = self.normal                     # type: ignore[misc]
        tx, ty = -ny, nx                         # kiriş boyunca birim
        yanal = abs(
            (vehicle[0] - self.midpoint[0]) * tx
            + (vehicle[1] - self.midpoint[1]) * ty
        )
        return yanal <= self.width / 2.0


@dataclass
class GateDiagnostics:
    """Neden kapı bulunamadı? — sahada bandı düzeltebilmek için.

    Kapı genişliği önceden bilinemediği için (bkz. GateFollowerConfig) en
    tehlikeli arıza SESSİZ RET'tir: turuncu dubalar pekâlâ görülüyordur ama
    genişlikleri banda girmediği için hiç kapı oluşmaz ve araç ham görev
    noktasına gider — hiçbir hata basılmadan, puan kaybederek.
    Bu sayaçlar node tarafından loglanır; operatör ÖLÇÜLEN genişlikleri görüp
    `planning.gate_width_max:=14` gibi bir launch-arg ile anında düzeltebilir.
    """

    n_edge_buoys: int = 0            # gelen turuncu duba sayısı
    n_in_range: int = 0              # burun hattının önünde kalanlar
    n_pairs_checked: int = 0         # değerlendirilen çift sayısı
    # Gövdenin sığmadığı (geçilemez) çiftlerin ÖLÇÜLEN açıklığı.
    reddedilen_genislik: List[float] = None
    # "Yan yana değil" (kursa dik değil) diye elenen çift sayısı — bunlar
    # neredeyse hep ardışık kapıların dubalarıdır, normal ve beklenen.
    reddedilen_derinlik: int = 0
    secilen_genislik: Optional[float] = None
    # Nişan noktasının geometrik ortadan kayması (m). 0.0 → engel kirişe
    # dokunmuyor, tam ortadan geçiliyor. Büyük değer sahada "kapının içinde/
    # ağzında bir şey var" demektir; operatör buna bakarak duba mı hayalet mi
    # ayırt eder.
    aim_kaymasi_m: float = 0.0

    def __post_init__(self) -> None:
        if self.reddedilen_genislik is None:
            self.reddedilen_genislik = []


@dataclass(frozen=True)
class GateResult:
    """update() çıktısı: kullanılacak hedef + bağlam."""

    target: Point            # rafine hedef (kapı ortası) ya da fallback (ham GN)
    gate: Optional[Gate]     # seçilen kapı; fallback'te None
    used_fallback: bool      # True → kapı yok, ham GN'ye düşüldü


def _forward_left_axes(
    vehicle: Point, coarse_target: Point
) -> Optional[Tuple[float, float, float, float]]:
    """Araçtan ham GN'ye "ileri" birim vektörü + "sol" dik birim vektörü.

    Kurs yönü = araçtan görev noktasına doğru (aracın anlık heading'i değil —
    GN kursun gittiği yönü verir, momentary heading sapmalarına dayanıklı).
    Araç GN'nin üstündeyse (yön tanımsız) None döner.
    """
    dx = coarse_target[0] - vehicle[0]
    dy = coarse_target[1] - vehicle[1]
    norm = math.hypot(dx, dy)
    if norm < 1e-6:
        return None
    fx, fy = dx / norm, dy / norm      # ileri birim
    lx, ly = -fy, fx                   # sol dik birim (90° CCW)
    return fx, fy, lx, ly


def _gate_normal(
    left: Point, right: Point, forward: Point
) -> Optional[Point]:
    """Kirişe dik birim vektör, `forward` ile aynı yarı düzleme işaretlenmiş.

    `forward` yalnız İŞARET içindir (hangi taraf "ileri"); vektörün yönü
    tamamen kapı geometrisinden gelir. Kiriş dejenere ise None.
    """
    dx, dy = right[0] - left[0], right[1] - left[1]
    w = math.hypot(dx, dy)
    if w <= 1e-9:
        return None
    nx, ny = -dy / w, dx / w                  # kirişe dik (90° CCW)
    if nx * forward[0] + ny * forward[1] < 0.0:
        nx, ny = -nx, -ny                     # ters yarı düzlem → çevir
    return (nx, ny)


def _clearance(px: float, py: float, circles: Sequence[Circle]) -> float:
    """Noktanın en yakın engel YÜZEYİNE mesafesi (m). Engel yoksa +sonsuz.

    Merkeze değil yüzeye bakılır: farklı yarıçaplı nesneler (30 cm duba ile
    bölünmüş büyük bir küme) aynı ölçüte girsin.
    """
    best = math.inf
    for cx, cy, r in circles:
        d = math.hypot(px - cx, py - cy) - r
        if d < best:
            best = d
    return best


def aim_point(
    left: Point,
    right: Point,
    circles: Sequence[Circle],
    cfg: GateFollowerConfig,
) -> Optional[Point]:
    r"""Kapı kirişi üzerinde engellerden EN AÇIK nişan noktası.

    🔑 **NEDEN GEREKLİ.** Kenar dubaları MPPI'nin engel torbasından bilerek
    çıkarılır (`obstacle_margin`=1.0 m'lik ceza halkası geçidin içini kaplar,
    ölçüldü: 1.5 m'de araç geçitten HİÇ geçmiyor). Bedeli şu: geçitte dubadan
    **iten hiçbir kuvvet kalmaz**, yalnız orta noktanın çekimi vardır. Dalga
    sürüklenmesi ya da tek taraflı bir tespit hatası doğrudan TEMAS demektir
    (`Ç1`/`Ç2`; md 815-818: aynı dubaya 30 sn temas = 2 çarpma).

    Çözüm dubayı engele geri çevirmek DEĞİL (geçit kapanır), **nişanı kirişin
    üzerinde kaydırmak**: itme MPPI'ye değil hedefe gömülür. Engel yokken
    nişan zaten tam ortadır → geriye birebir uyumlu.

    **Ölü bant:** kirişin iki ucunda `r_duba + gövde/2` kadar yer gövdeye
    yasaktır (duba yüzeyi + yarım gövde). Bu bant kirişten uzunsa kapı
    geçilemez → `None`. Yani `min_passable_width` testi burada da kendini
    doğrular.

    **Neden tarama, neden kapalı form yok:** açıklık = tüm dairelerin üzerinde
    `min(|p−c| − r)`; bunun maksimumu parçalı hiperbolik, analitik çözümü yok.
    Izgara **merkezden dışa** kurulur → engel bağlamadığında argmax tam merkez
    çıkar (kayan ızgarada merkez örneklenmeyip sonuç 1-2 cm kayabilirdi).
    Adım gövde genişliğinin dörtte biri: ayarlanan eşik değil, "tekne bu
    çözünürlüğün altındaki farkı zaten süremez" ölçüsü.

    Girdi/çıktı dünya ENU. `circles` kirişi ilgilendirmeyen uzak engelleri de
    içerebilir — uzak engel `min`'i bağlamaz, sonucu değiştirmez.
    """
    lx, ly = left
    rx, ry = right
    w = math.hypot(rx - lx, ry - ly)
    if w <= 1e-9:
        return None
    tx, ty = (rx - lx) / w, (ry - ly) / w          # kiriş boyunca birim vektör
    olu_bant = BUOY_RADIUS_M + cfg.half_beam
    s_lo, s_hi = olu_bant, w - olu_bant
    if s_lo > s_hi:
        return None                                # gövde sığmıyor → kapı değil
    merkez = 0.5 * w                               # s_lo ≤ merkez ≤ s_hi (garanti)

    # Merkezden dışa doğru simetrik ızgara + bandın iki ucu.
    adaylar = [merkez]
    adim = max(cfg.hull_width_m / 4.0, 1e-3)
    for k in range(1, _AIM_MAX_ADIM + 1):
        o = k * adim
        geri, ileri = merkez - o, merkez + o
        if geri < s_lo and ileri > s_hi:
            break
        if geri >= s_lo:
            adaylar.append(geri)
        if ileri <= s_hi:
            adaylar.append(ileri)
    adaylar.append(s_lo)
    adaylar.append(s_hi)

    en_iyi_s = merkez
    en_iyi_a = -math.inf
    for s in adaylar:
        px, py = lx + tx * s, ly + ty * s
        a = _clearance(px, py, circles)
        # Eşitlikte MERKEZE en yakın aday kazanır: engel simetrikse ya da hiç
        # yoksa sonuç geometrik ortadır (belirlenimci, tarama sırasından bağımsız).
        if a > en_iyi_a + 1e-9 or (
            abs(a - en_iyi_a) <= 1e-9 and abs(s - merkez) < abs(en_iyi_s - merkez)
        ):
            en_iyi_s, en_iyi_a = s, a
    return (lx + tx * en_iyi_s, ly + ty * en_iyi_s)


def select_gate(
    vehicle: Point,
    coarse_target: Point,
    edge_buoys: Sequence[Point],
    cfg: GateFollowerConfig,
    diag: Optional[GateDiagnostics] = None,
    obstacles: Sequence[Circle] = (),
) -> Optional[Gate]:
    """Öndeki en yakın geçerli kapıyı seç (durumsuz, saf fonksiyon).

    Adımlar:
      1. Kurs eksenini (ileri/sol) ham GN yönünden kur.
      2. Her kenar dubasını (ileri, lateral) uzayına projelendir; yalnız burun
         hattının ÖNÜNDE olanları tut (üst menzil kapağı YOK — menzili algı
         katmanı belirler).
      3. Çiftleri değerlendir: (a) merkez mesafesi ≥ `min_passable_width`
         (= gövde + 2r, gövde fiilen sığıyor mu) ve (b) `|Δileri| < |Δyanal|`
         (kursa dik mi, yani kapı mı yoksa ardışık kapıların dubaları mı).
      4. Geçerli çiftlerden orta noktası EN ÖNDE (en yakın) olanı seç; eşitlikte
         orta noktası kurs çizgisine en yakın olan (küçük |lateral|).
      5. Kazanan kapı için `aim_point` ile engellerden en açık nişanı hesapla.

    Dönüş: Gate ya da geçerli kapı yoksa None. Kenar dubaları frame'i = girdi
    frame'i (dünya ENU); left/right kursa göre etiketlenir (+lateral = sol).
    """
    if diag is not None:
        diag.n_edge_buoys = len(edge_buoys)
    if len(edge_buoys) < 2:
        return None
    axes = _forward_left_axes(vehicle, coarse_target)
    if axes is None:
        return None
    fx, fy, lx, ly = axes
    vx, vy = vehicle

    # (buoy, ileri, lateral) — sadece öndeki menzil içi dubalar.
    projected: List[Tuple[Point, float, float]] = []
    for bx, by in edge_buoys:
        rx, ry = bx - vx, by - vy
        fwd = rx * fx + ry * fy
        # Yalnız burun hattının ÖNÜNDEKİLER. Üst menzil kapağı YOK: menzili
        # algı katmanı belirler (LiDAR/kamera zaten ne görüyorsa onu verir);
        # buraya ikinci bir sayı koymak tahmin olurdu.
        if fwd < cfg.min_forward:
            continue
        lat = rx * lx + ry * ly
        projected.append(((bx, by), fwd, lat))

    if diag is not None:
        diag.n_in_range = len(projected)
    if len(projected) < 2:
        return None

    best: Optional[Tuple[float, float, Gate]] = None   # (mid_fwd, |mid_lat|, Gate)
    n = len(projected)
    for i in range(n):
        pi, fi, li = projected[i]
        for j in range(i + 1, n):
            pj, fj, lj = projected[j]
            if diag is not None:
                diag.n_pairs_checked += 1
            sep = math.hypot(pi[0] - pj[0], pi[1] - pj[1])
            # (a) GEÇİLEBİLİRLİK — gövde sığmıyorsa bu bir kapı değildir.
            # `sep` MERKEZDEN merkeze; serbest açıklık `sep − 2r` olduğu için
            # eşik gövde genişliği DEĞİL `hull + 2r`'dir (bkz.
            # `min_passable_width`). Üst sınır YOK: kapı ne kadar geniş olursa
            # olsun geçilebilir.
            if sep < cfg.min_passable_width:
                if diag is not None:
                    diag.reddedilen_genislik.append(sep)
                continue
            # (b) KURSA DİK Mİ — bir kapı kursa dik duran çifttir; ardışık
            # kapıların dubaları ise kurs boyunca dizilir. |Δileri| < |Δyanal|
            # tam olarak "paralel olmaktan çok dike yakın" demek (ayrım noktası
            # 45°: ayarlanan eşik değil, iki hâl arasındaki geometrik sınır).
            # Ölçek-bağımsız → kapı genişliğini bilmeyi GEREKTİRMEZ.
            if abs(fi - fj) >= abs(li - lj):
                if diag is not None:
                    diag.reddedilen_derinlik += 1
                continue
            mid_fwd = 0.5 * (fi + fj)
            mid_lat = 0.5 * (li + lj)
            # +lateral = sol; büyük lateral olan SOL dubadır.
            left, right = (pi, pj) if li >= lj else (pj, pi)
            midpoint = (0.5 * (pi[0] + pj[0]), 0.5 * (pi[1] + pj[1]))
            gate = Gate(left=left, right=right, midpoint=midpoint)
            key = (mid_fwd, abs(mid_lat))
            if best is None or key < (best[0], best[1]):
                best = (mid_fwd, abs(mid_lat), gate)

    if best is None:
        return None

    # Nişan noktası YALNIZ kazanan çift için hesaplanır (her aday için değil —
    # tarama gereksiz maliyet olurdu). Açıklık dairelerine kapının kendi
    # direkleri de girer: MPPI'de engel olmadıkları için iten tek kuvvet budur.
    # Diğer kenar dubaları da katılır — koridora sarkan üçüncü bir duba
    # nişanı ondan uzağa iter (yanlış eşleşmenin sessiz zararını azaltır).
    gate = best[2]
    circles: List[Circle] = [(bx, by, BUOY_RADIUS_M) for bx, by in edge_buoys]
    circles.extend(obstacles)
    aim = aim_point(gate.left, gate.right, circles, cfg)
    # Kapının KENDİ ileri normali: kirişe dik iki adaydan gidiş yönüyle aynı
    # tarafta olanı. İşaret BİR KEZ burada belirlenir; sonrasında "geçildi"
    # testi kurstan da GN'nin yerinden de bağımsızdır (B4/B6).
    # Adayın işareti kesin: çift (b) testini geçtiği için kiriş kursa
    # PARALEL olmaktan çok DİK — yani n·f sıfıra yakın olamaz.
    normal = _gate_normal(gate.left, gate.right, (fx, fy))
    gate = Gate(
        left=gate.left, right=gate.right, midpoint=gate.midpoint,
        aim=aim, normal=normal,
    )
    if diag is not None:
        diag.secilen_genislik = gate.width
        diag.aim_kaymasi_m = gate.aim_shift
    return gate


class GateFollower:
    """Durumlu kapı takipçisi — seçime histerezis (kapıya kilitlen, geçince bırak).

    Saf `select_gate` her tick'te öndeki en yakın kapıyı verir; bu yeterince
    kararlıdır ama (a) kapı anlık görüşten çıkınca (oklüzyon/dalga) hedef zıplar,
    (b) geçiş anında bir sonraki kapıya erken atlayabilir. Bu sınıf seçilen
    kapıya KİLİTLENİR, taze algıyla günceller, ancak orta noktasını GEÇİNCE
    serbest bırakıp sonraki kapıyı seçer. Kapı hiç yoksa ham GN'ye düşer.

    Durum minimaldir (tek bir kilitli kapı); MPPI'nin warm-start'ı + FSM
    güvenlik çatısı zaten üstte — burada aşırı mühendislik yok.
    """

    def __init__(self, cfg: Optional[GateFollowerConfig] = None) -> None:
        self._cfg = cfg or GateFollowerConfig()
        self._committed: Optional[Gate] = None
        # Son update()'in teşhis sayaçları — node bunu loglar (sessiz ret kapanı).
        self.last_diagnostics = GateDiagnostics()
        # Şartname G1/G2 = "FARKLI karşılıklı kenar dubaları arasından geçiş
        # sayısı". Düzlem-kesişimi testi (B4/B6) bu sayacı bedavaya veriyor:
        # aynı hesap hem kilidi bırakıyor hem geçişi sayıyor.
        self._passed_midpoints: List[Point] = []

    def reset(self) -> None:
        """Kilitli kapıyı temizle (parkur geçişi / yeniden başlama).

        ⚠ Geçiş SAYACINA dokunmaz: sayaç puan kanıtıdır, parkur geçişinde
        sıfırlanmaz. Yeniden başlama için `reset_passed_gates()` kullan.
        """
        self._committed = None

    def reset_passed_gates(self) -> None:
        """Geçiş sayacını sıfırla — YALNIZ yeniden başlamada.

        Şartname md 5.5.3.1: *"Yeniden başlama hakkını kullanan takımın
        topladığı puanlar SIFIRLANACAKTIR."* İkinci turda aynı geçitlerden
        yeniden geçilir; hafıza temizlenmezse hepsi "zaten geçildi" sayılır
        ve HİÇBİRİ sayılmaz.
        """
        self._passed_midpoints.clear()

    @property
    def committed_gate(self) -> Optional[Gate]:
        return self._committed

    @property
    def passed_gate_count(self) -> int:
        """Geçilen FARKLI kapı sayısı (şartname G1/G2 tanımı)."""
        return len(self._passed_midpoints)

    def _say_gecis(self, gate: Gate) -> None:
        """Geçilen kapıyı say — daha önce sayılanlardan FARKLIYSA.

        Ayırt etme ölçüsü kapının KENDİ yarı genişliği: manevra/geri dönüş
        yüzünden aynı kapıdan tekrar geçmek sayılmaz, ama komşu bir kapı
        (en az bir kapı genişliği ötede) ayrı sayılır. Öz-ölçekli — dışarıdan
        bir mesafe eşiği GEREKTİRMEZ (modülün tasarım kuralı).
        """
        yari = gate.width / 2.0
        for px, py in self._passed_midpoints:
            if math.hypot(gate.midpoint[0] - px, gate.midpoint[1] - py) <= yari:
                return                      # aynı kapı — tekrar sayma
        self._passed_midpoints.append(gate.midpoint)

    def update(
        self,
        vehicle: Point,
        coarse_target: Point,
        edge_buoys: Sequence[Point],
        obstacles: Sequence[Circle] = (),
    ) -> GateResult:
        """Bir kontrol tick'i: rafine hedefi (kapı nişanı ya da ham GN) üret.

        `obstacles`: dünya ENU dairesel engeller (sarı duba, UNKNOWN küme…).
        Verilmezse nişan geometrik ortaya düşer — eski davranış birebir.
        """
        self.last_diagnostics = GateDiagnostics()
        fresh = select_gate(
            vehicle, coarse_target, edge_buoys, self._cfg,
            self.last_diagnostics, obstacles,
        )

        if self._committed is not None:
            axes = _forward_left_axes(vehicle, coarse_target)
            # Geçildi mi? — ÖNCE kapının KENDİ normali (B4/B6), yalnız o yoksa
            # eski kurs ekseni. "Geçildi" = kapı düzlemi ARKADA kaldı; tam
            # geometrik tanım (eski `release_distance`=0.8 m bir tahmindi ve
            # kapıyı erken bırakıyordu).
            d = self._committed.signed_distance(vehicle)
            if d is not None:
                passed = d > 0.0
                # Direklerin ARASINDAN mı geçtik yoksa yandan mı DOLAŞTIK?
                # Kilit her iki hâlde de bırakılır (kapı arkada kaldı), ama
                # yalnız aradan geçiş şartnamenin saydığı geçiştir (G1/G2).
                if passed and self._committed.passed_between(vehicle):
                    self._say_gecis(self._committed)
            else:
                # Normal yok (elle kurulmuş Gate) → geriye uyumlu eski test.
                passed = True
                if axes is not None:
                    fx, fy, _, _ = axes
                    mx, my = self._committed.midpoint
                    fwd = (mx - vehicle[0]) * fx + (my - vehicle[1]) * fy
                    passed = fwd <= 0.0
            if not passed:
                # Kapıya hâlâ gidiyoruz. Taze algı aynı kapıyı görüyorsa
                # drift'i güncelle; görmüyorsa (oklüzyon) kilitli kapıyı KORU.
                # "Aynı kapı mı" ölçütü kapının KENDİ genişliğinin yarısı:
                # öz-ölçekli, dolayısıyla kapı 2 m de olsa 40 m de olsa
                # çalışır ve dışarıdan bir sayı gerektirmez (eski
                # `match_radius`=2.5 m bir tahmindi ve geniş kapılarda drift
                # güncellemesini kaçırırdı).
                if fresh is not None and _dist(
                    fresh.midpoint, self._committed.midpoint
                ) <= self._committed.width / 2.0:
                    self._committed = fresh
                return GateResult(
                    target=self._committed.drive_target,
                    gate=self._committed,
                    used_fallback=False,
                )
            # Geçildi → serbest bırak, aşağıda yeniden seç.
            self._committed = None

        # Kilitli kapı yok: taze kapı varsa kilitlen, yoksa ham GN'ye düş.
        if fresh is not None:
            self._committed = fresh
            return GateResult(
                target=fresh.drive_target, gate=fresh, used_fallback=False
            )
        return GateResult(target=coarse_target, gate=None, used_fallback=True)


def _dist(a: Point, b: Point) -> float:
    return math.hypot(a[0] - b[0], a[1] - b[1])
