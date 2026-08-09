r"""
Girdap İDA — GERÇEK PARKUR GEOMETRİSİ (`parkur_nihai.world`) okuyucusu.

🔑 **Neden bu modül var: sabit koordinat GÖMÜLMESİN.**

09.08'de parkur dosyası kapı açıklığı **18 m** ile geldi, kaptan teyidiyle
**12 m**'ye indirildi (*"orası kesin"*). Aynı gün yazılan testler 18 m'lik
sayıları dosyaya GÖMDÜĞÜ için 12 m'ye çevrilmedi ve kırmızı kaldı; ölçüm
betiği ise `.world`'ü doğrudan okuduğu için kendiliğinden doğru koştu.
**Ders: parkuru tek yerden oku.** (GIRDAP_DURUM §0.17b/§0.17f)

Dosya repoda: `prototype/configs/parkur_nihai.world`. Repoya kondu çünkü
`~/Downloads` altında dururken (a) test edilemiyordu, (b) makine değişince
kayboluyordu, (c) kodun içindeki kopyaların doğruluğu denetlenemiyordu —
`yarisma_simulasyonu._REAL_ENGEL_BUOYS_ENU` tam bu yüzden 45 gün boyunca
bayat (y=±3,5) kaldı.

İçerik (09.08 hâli): 38 turuncu duba (16 Parkur-1 kapı direği + 22 Parkur-2
sınırı) · 9 sarı engel · 3 hedef duba (Parkur-3) · GN1-GN5 · araç başlangıcı.

⚠ Bu modül parkurun **simülasyon** tanımıdır; koşu günü gerçek noktalar
uçuş kontrolcüsünden (`mission_source: fc`) gelir. Buradaki koordinatlar
test/ölçüm içindir, kontrol yolunda kullanılmaz.
"""

from __future__ import annotations

import xml.etree.ElementTree as ET
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List, Optional, Tuple

Point = Tuple[float, float]

VARSAYILAN_DOSYA = (
    Path(__file__).resolve().parents[1] / "configs" / "parkur_nihai.world"
)


@dataclass(frozen=True)
class ParkurDunyasi:
    """`.world` dosyasından okunan parkur geometrisi (dünya ENU, metre)."""

    kapilar: List[Tuple[Point, Point]]      # Parkur-1 kapı çiftleri (sol, sağ)
    guzergah: List[Point]                   # GN1..GN4 (Parkur-1/2 görev noktaları)
    engeller: List[Point]                   # Parkur-2 sarı engelleri
    hedefler: Dict[str, Point]              # Parkur-3 hedef dubaları (renk → konum)
    baslangic: Point                        # aracın başlangıç konumu
    gn5: Optional[Point] = None             # Parkur-2 bitişi (varsa)

    @property
    def kapi_genislikleri(self) -> List[float]:
        return [
            ((s[0] - g[0]) ** 2 + (s[1] - g[1]) ** 2) ** 0.5
            for s, g in self.kapilar
        ]

    @property
    def kapi_ortalari(self) -> List[Point]:
        return [((s[0] + g[0]) / 2.0, (s[1] + g[1]) / 2.0) for s, g in self.kapilar]

    def dubalar(self) -> List[Point]:
        """Tüm kapı direkleri, tek listede (kapı sırasıyla sol, sağ)."""
        return [b for kapi in self.kapilar for b in kapi]


def _poz(eleman: ET.Element) -> Optional[Point]:
    parcalar = (eleman.findtext("pose") or "").split()
    if len(parcalar) < 2:
        return None
    return (float(parcalar[0]), float(parcalar[1]))


def oku(yol: Optional[Path] = None) -> ParkurDunyasi:
    """`.world` dosyasını oku.

    ⚠ İki ayrı ad kaynağı var ve KARIŞTIRILMAMALI: `<include>` bloklarında ad
    **alt eleman** (`<name>`), `<model>` bloklarında ise **attribute**
    (`<model name="...">`). 09.08'de bu ayrım gözden kaçınca GN1-GN4'ün
    pose satırları bozulmuştu (önceki dubanın kuralı sızdı, yedekten geri
    alındı) — o yüzden ikisi ayrı ayrı ele alınıyor.
    """
    yol = Path(yol) if yol is not None else VARSAYILAN_DOSYA
    kok = ET.parse(yol).getroot()
    dunya = kok.find("world") if kok.find("world") is not None else kok

    dahil: Dict[str, Point] = {}
    uriler: Dict[str, str] = {}
    for e in dunya.iter("include"):
        ad = e.findtext("name") or ""
        p = _poz(e)
        if ad and p is not None:
            dahil[ad] = p
            uriler[ad] = e.findtext("uri") or ""

    modeller: Dict[str, Point] = {}
    for e in dunya.iter("model"):
        ad = e.get("name", "")
        p = _poz(e)
        if ad and p is not None:
            modeller[ad] = p

    # Kapılar: p1_l<i> / p1_r<i> çiftleri, i 1'den başlayarak kesintisiz.
    kapilar: List[Tuple[Point, Point]] = []
    i = 1
    while f"p1_l{i}" in dahil and f"p1_r{i}" in dahil:
        kapilar.append((dahil[f"p1_l{i}"], dahil[f"p1_r{i}"]))
        i += 1

    guzergah: List[Point] = []
    j = 1
    while f"GN{j}" in modeller:
        guzergah.append(modeller[f"GN{j}"])
        j += 1
    gn5 = guzergah.pop() if len(guzergah) > 4 else None

    engeller = [
        dahil[ad] for ad in sorted(
            (a for a in dahil if a.startswith("engel")),
            key=lambda a: int(a[5:]) if a[5:].isdigit() else 0,
        )
    ]
    hedefler = {
        ad.replace("hedef_", ""): dahil[ad]
        for ad in dahil if ad.startswith("hedef_")
    }
    baslangic = dahil.get("IDA") or modeller.get("IDA") or (0.0, 0.0)

    return ParkurDunyasi(
        kapilar=kapilar,
        guzergah=guzergah,
        engeller=engeller,
        hedefler=hedefler,
        baslangic=baslangic,
        gn5=gn5,
    )
