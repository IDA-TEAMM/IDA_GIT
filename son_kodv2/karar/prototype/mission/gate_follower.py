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
    """Kapı seçimi ve takip histerezis parametreleri (saha testinde kalibre).

    Değerler temsili başlangıç tahminleridir; gerçek duba aralıkları yarışma
    anına göre değişir (md 5.5.2.2) — sahada ölçülüp güncellenmeli.
    """

    # Geçerli bir kapının kenar dubaları arası mesafe aralığı (m). Bu aralık
    # dışındaki çiftler kapı sayılmaz (çok yakın = tek dubanın gürültüsü;
    # çok uzak = ayrı kapıların dubaları).
    gate_width_min: float = 1.5
    gate_width_max: float = 8.0
    # Yalnız araçtan en fazla bu kadar ÖNDEKİ dubalar dikkate alınır (m).
    max_lookahead: float = 25.0
    # Duba en az bu kadar önde olmalı (m) — aracın hizasındaki/geçilmiş
    # dubaları eler (0 çok küçük; negatif = arkada, zaten elenir).
    min_forward: float = 0.5
    # Bir kapının iki dubası "yan yana" olmalı: ileri-mesafe farkı bu eşiği
    # aşarsa (biri yakın biri uzak) çift kapı sayılmaz (m).
    pair_depth_tol: float = 3.0
    # --- histerezis (GateFollower sınıfı) ---
    # Kilitlenmiş kapının orta noktasına ileri-mesafe bunun altına inince
    # kapı "geçildi" sayılır ve serbest bırakılır (m).
    release_distance: float = 0.8
    # Yeniden algılanan kapı, kilitli kapının orta noktasına bu yarıçap
    # içindeyse "aynı kapı" kabul edilir ve taze algıyla güncellenir (m).
    match_radius: float = 2.5


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
        if fwd < cfg.min_forward or fwd > cfg.max_lookahead:
            continue
        lat = rx * lx + ry * ly
        projected.append(((bx, by), fwd, lat))

    if len(projected) < 2:
        return None

    best: Optional[Tuple[float, float, Gate]] = None   # (mid_fwd, |mid_lat|, Gate)
    n = len(projected)
    for i in range(n):
        pi, fi, li = projected[i]
        for j in range(i + 1, n):
            pj, fj, lj = projected[j]
            sep = math.hypot(pi[0] - pj[0], pi[1] - pj[1])
            if sep < cfg.gate_width_min or sep > cfg.gate_width_max:
                continue
            if abs(fi - fj) > cfg.pair_depth_tol:      # yan yana değil → kapı değil
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
        fresh = select_gate(vehicle, coarse_target, edge_buoys, self._cfg)

        if self._committed is not None:
            axes = _forward_left_axes(vehicle, coarse_target)
            # Kilitli kapının orta noktasına ileri-mesafe: geçildi mi?
            passed = True
            if axes is not None:
                fx, fy, _, _ = axes
                mx, my = self._committed.midpoint
                fwd = (mx - vehicle[0]) * fx + (my - vehicle[1]) * fy
                passed = fwd <= self._cfg.release_distance
            if not passed:
                # Kapıya hâlâ gidiyoruz. Taze algı aynı kapıyı görüyorsa
                # (orta nokta match_radius içinde) drift'i güncelle; görmüyorsa
                # (oklüzyon) kilitli kapıyı KORU — hedef zıplamaz.
                if fresh is not None and _dist(
                    fresh.midpoint, self._committed.midpoint
                ) <= self._cfg.match_radius:
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
