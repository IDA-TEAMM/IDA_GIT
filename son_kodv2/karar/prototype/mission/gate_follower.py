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
ve orta noktasını "rafine hedef" olarak döndürür. Kapı görünmüyorsa ham GPS
görev noktasına düşer (fallback) — böylece kapı görüş menziline girene kadar
araç yine de doğru genel yöne ilerler.

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
    | `gate_width_min` = 1.0 m | **gövde genişliği** (fizik: dar kapıdan geçilemez) |
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


@dataclass(frozen=True)
class Gate:
    """Seçilmiş bir kapı: iki kenar dubası + orta nokta (hepsi dünya ENU)."""

    left: Point       # kursa göre SOL kenar dubası (+lateral)
    right: Point      # kursa göre SAĞ kenar dubası (-lateral)
    midpoint: Point   # geçilecek hedef nokta

    @property
    def width(self) -> float:
        """Kenar dubaları arası mesafe (m)."""
        return math.hypot(self.left[0] - self.right[0], self.left[1] - self.right[1])


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


def select_gate(
    vehicle: Point,
    coarse_target: Point,
    edge_buoys: Sequence[Point],
    cfg: GateFollowerConfig,
    diag: Optional[GateDiagnostics] = None,
) -> Optional[Gate]:
    """Öndeki en yakın geçerli kapıyı seç (durumsuz, saf fonksiyon).

    Adımlar:
      1. Kurs eksenini (ileri/sol) ham GN yönünden kur.
      2. Her kenar dubasını (ileri, lateral) uzayına projelendir; yalnız
         `min_forward ≤ ileri ≤ max_lookahead` olanları tut.
      3. Çiftleri değerlendir: kenar mesafesi `[gate_width_min, gate_width_max]`
         içinde VE iki dubanın ileri-mesafe farkı `≤ pair_depth_tol` (yan yana).
      4. Geçerli çiftlerden orta noktası EN ÖNDE (en yakın) olanı seç; eşitlikte
         orta noktası kurs çizgisine en yakın olan (küçük |lateral|).

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
            # Üst sınır YOK: kapı ne kadar geniş olursa olsun geçilebilir.
            if sep < cfg.hull_width_m:
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

    if best is not None and diag is not None:
        diag.secilen_genislik = best[2].width
    return best[2] if best is not None else None


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

    def reset(self) -> None:
        """Kilitli kapıyı temizle (parkur geçişi / yeniden başlama)."""
        self._committed = None

    @property
    def committed_gate(self) -> Optional[Gate]:
        return self._committed

    def update(
        self,
        vehicle: Point,
        coarse_target: Point,
        edge_buoys: Sequence[Point],
    ) -> GateResult:
        """Bir kontrol tick'i: rafine hedefi (kapı ortası ya da ham GN) üret."""
        self.last_diagnostics = GateDiagnostics()
        fresh = select_gate(
            vehicle, coarse_target, edge_buoys, self._cfg, self.last_diagnostics
        )

        if self._committed is not None:
            axes = _forward_left_axes(vehicle, coarse_target)
            # Kilitli kapının orta noktasına ileri-mesafe: geçildi mi?
            passed = True
            if axes is not None:
                fx, fy, _, _ = axes
                mx, my = self._committed.midpoint
                fwd = (mx - vehicle[0]) * fx + (my - vehicle[1]) * fy
                # "Geçildi" = kapı düzlemi ARKADA kaldı. Tam geometrik tanım;
                # eski `release_distance` (0.8 m) bir tahmindi ve kapıyı erken
                # bırakıyordu.
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
                    target=self._committed.midpoint,
                    gate=self._committed,
                    used_fallback=False,
                )
            # Geçildi → serbest bırak, aşağıda yeniden seç.
            self._committed = None

        # Kilitli kapı yok: taze kapı varsa kilitlen, yoksa ham GN'ye düş.
        if fresh is not None:
            self._committed = fresh
            return GateResult(target=fresh.midpoint, gate=fresh, used_fallback=False)
        return GateResult(target=coarse_target, gate=None, used_fallback=True)


def _dist(a: Point, b: Point) -> float:
    return math.hypot(a[0] - b[0], a[1] - b[1])
