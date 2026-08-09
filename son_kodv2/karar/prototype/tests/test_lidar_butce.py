"""
Girdap İDA — LiDAR kümeleme ZAMAN BÜTÇESİ nöbetçileri (ROS'SUZ).

🔴 **NEDEN (2026-08-09 ölçümü + Livox spesifikasyonu).**

Livox Mid-360: **200 000 nokta/s, 10 Hz** → kare başına **20 000 nokta**, her
zaman. Yani "20k nokta" kapalı alana özgü bir durum değil, sensörün sabit
çıktısı; kapalı alanda değişen şey noktaların NEREYE düştüğü.

`cluster_points`'in maliyeti nokta sayısına DEĞİL, `cKDTree.query_pairs`'in
döndürdüğü **çift sayısına** bağlı — o da yoğunlukla büyüyor:

| sahne | nokta → voxel | çift | nokta başına komşu | süre |
|---|---|---|---|---|
| açık su, 8 kapı dubası | 49 → 35 | 95 | 2,7 | **0,2 ms** |
| açık su + 10k su dönüşü | 10 049 → 9 828 | 30 206 | 6,1 | **8,5 ms** |
| kapalı oda 8×6×3 m | 20 000 → 9 686 | 298 299 | 61,6 | **26,4 ms** |
| 5 m'lik kapalı hacim | 20 000 → 15 628 | 1 372 707 | 175,7 | **112 ms** 🔴 |

Bunun bedeli sessiz: kümeleme 100 ms'i aşarsa `/perception/obstacle_map` GEÇ
varır → füzyonun `ApproximateTimeSynchronizer`'ı eşleşme bulamaz →
`/perception/classified_obstacles` üretilmez → `planning_node._edge_buoys`
boş kalır → kapı takibi ham GPS noktasına düşer → P1/P2 puanı gider.
(09.07 tezgahında tam bu yaşandı: ölçülen gecikme 1-3,3 s.)

⚠ O 1-3,3 s bu makinede ÜRETİLEMEDİ (en kötü 112 ms). Fark muhtemelen
Jetson'ın CPU'su + eşzamanlı koşan diğer node'lar + gerçek odanın modelden
yoğun olması. Gerçek sayı **Jetson'da ölçülmeli**; node artık her 5 saniyede
süreyi logluyor ve bütçe aşılırsa uyarı basıyor.

Bu dosya iki şeyi donduruyor:
  1. Açık su sahnesi bütçenin çok altında kalmalı (performans gerilemesi kapanı)
  2. Yükü taşıyan emniyet (`voxel_size`) node'da AÇIK kalmalı
"""

from __future__ import annotations

import ast
import time
from pathlib import Path

import numpy as np

from prototype.perception.lidar_obstacles import (
    LidarObstacleConfig,
    cluster_points,
    detect_obstacles,
    voxel_downsample,
)

_NODE_FILE = (
    Path(__file__).resolve().parents[2] / "ros2_ws" / "src" / "girdap_decision"
    / "girdap_decision" / "perception_lidar_node.py"
)

#: Livox Mid-360 spesifikasyonu — 200 000 nokta/s @ 10 Hz
LIVOX_NOKTA_PER_KARE = 20_000
BUTCE_MS = 100.0                     # 10 Hz periyodu

#: `perception_lidar_node`'un DECLARE ettiği üretim değerleri (çekirdek
#: varsayılanı DEĞİL — çekirdekte voxel_size=0.0, node'da 0.1).
URETIM = LidarObstacleConfig(
    z_min=0.1, z_max=3.0, cluster_tolerance=0.5, min_cluster_size=5,
    max_cluster_size=500, split_cell_m=1.0, max_range=25.0, voxel_size=0.1,
)


def _node_varsayilanlari() -> dict:
    """Node'daki `self.declare_parameter("ad", <literal>)` çağrılarını topla."""
    bulunan: dict = {}
    for dugum in ast.walk(ast.parse(_NODE_FILE.read_text(encoding="utf-8"))):
        if not isinstance(dugum, ast.Call):
            continue
        fn = dugum.func
        if not (isinstance(fn, ast.Attribute) and fn.attr == "declare_parameter"):
            continue
        if len(dugum.args) != 2:
            continue
        try:
            bulunan[ast.literal_eval(dugum.args[0])] = ast.literal_eval(dugum.args[1])
        except ValueError:
            continue
    return bulunan


def _acik_su_sahnesi(tohum: int = 7, n_su_donusu: int = 10_000) -> np.ndarray:
    """Gerçekçi açık su karesi: kapı dubaları + su yüzeyinden dağınık dönüşler.

    Duba nokta sayısı katı açıdan türer (Livox ~5400 nokta/sr): 0,3×0,5 m'lik
    bir duba 10 m'de ~8 nokta verir — `min_cluster_size=5` bu yüzden kritik.
    """
    rng = np.random.default_rng(tohum)
    kapilar = [(6, -11), (6, 1), (10, -6), (10, 6),
               (14, -1), (14, 11), (18, -6), (18, 6)]
    parcalar = []
    for bx, by in kapilar:
        d = float(np.hypot(bx, by))
        n = max(3, int(5400 * 0.15 / max(d, 1.0) ** 2))
        parcalar.append(rng.normal(0, 0.07, size=(n, 3)) + np.array([bx, by, 0.25]))
    r = rng.uniform(1.0, 25.0, n_su_donusu)
    th = rng.uniform(0, 2 * np.pi, n_su_donusu)
    z = rng.normal(0.15, 0.05, n_su_donusu)
    parcalar.append(np.stack([r * np.cos(th), r * np.sin(th), z], axis=1))
    return np.vstack(parcalar)


# ----------------------------------------------------------- performans

def test_acik_su_karesi_butcenin_COK_altinda() -> None:
    """Açık su 10 Hz bütçesinin çok altında kalmalı (ölçülen 8,5 ms).

    Eşik 50 ms = ölçülenin ~6 katı; amaç kesin süreyi dondurmak değil,
    BÜYÜKLÜK MERTEBESİ gerilemesini yakalamak (ör. voxel'in kapatılması,
    tolerance'ın büyütülmesi, query_pairs'in daha pahalı bir şeyle
    değiştirilmesi). Yüklü CI'da bile 6 kat pay var.
    """
    p = voxel_downsample(_acik_su_sahnesi(), URETIM.voxel_size)
    sureler = []
    for _ in range(3):                       # en iyi koşum = makine gürültüsü dışı
        t0 = time.perf_counter()
        cluster_points(p, URETIM)
        sureler.append((time.perf_counter() - t0) * 1000.0)
    en_iyi = min(sureler)
    assert en_iyi < 50.0, (
        f"açık su kümelemesi {en_iyi:.1f} ms — ölçülen 8,5 ms'in çok "
        "üstünde; voxel kapanmış ya da tolerance büyümüş olabilir"
    )


def test_yakin_duba_tespit_ediliyor() -> None:
    """Hız anlamlı olsun diye: boru hattı yakın dubayı GERÇEKTEN buluyor.

    (Boş çıktı da hızlıdır — süre testini tek başına bırakmak yanıltıcı olur.)
    5 ve 8 m modelden bağımsız güvenli: oralarda duba 21 ve 8 nokta veriyor,
    `min_cluster_size=5`'in rahatça üstünde.
    """
    rng = np.random.default_rng(11)
    for d, n in ((5.0, 21), (8.0, 8)):
        pts = rng.normal(0, 0.07, size=(n, 3)) + np.array([d, 0.0, 0.25])
        obstacles = detect_obstacles(pts, URETIM)
        assert obstacles, f"{d:.0f} m'deki duba ({n} nokta) tespit edilemedi"


def test_MENZIL_KISITI_belgeleniyor_min_cluster_size_buyutulemez() -> None:
    r"""🔴 **AÇIK RİSK — kapı hafızasının ihtiyaç duyduğu menzil ölçülmedi.**

    30×50 cm'lik duba mesafeyle hızla kararıyor (katı açı ∝ 1/d²). Livox'un
    ortalama açısal yoğunluğu 3 498 nokta/sr (200 000 nokta/s ÷ 10 Hz ÷ 5,72 sr):

    | mesafe | dubaya düşen nokta | voxel sonrası | boru hattı tespit ediyor mu |
    |---|---|---|---|
    | 5 m | 21 | 16 | ✅ |
    | 8 m | 8 | 7 | ✅ |
    | **10 m** | **5** | **4** | 🔴 **HAYIR** |
    | 12 m | 4 | 4 | 🔴 hayır |
    | 15 m | 2 | 2 | 🔴 hayır |

    🔑 İki ayar birbirine bağlı ve bağ YAZILI DEĞİL: `voxel_size=0.1` aynı
    10 cm hücreye düşen noktaları teke indiriyor, yani `min_cluster_size`
    ham nokta sayısını değil **voxel sonrası** sayıyı süzüyor. 10 m'de duba
    5 ham noktadan 4 voxel noktasına düşüp eşiğe takılıyor → CPU'yu kurtaran
    voxel, tespit menzilini kısaltıyor.

    ⚠️ **Neden bu bir "düzelt" değil "ÖLÇ" maddesi:** yukarıdaki tablo
    açısal yoğunluğun FOV boyunca DÜZGÜN dağıldığını varsayıyor. Mid-360
    tekrarsız taramalı, yoğunluk düzgün değil; ayrıca dubanın suyun üstünde
    kalan/LiDAR'a bakan yüzeyi 0,15 m²'den küçük olabilir. Model 2 kat yanılsa
    menzil 7 m de olur 14 m de. **Sahada ölçülmeli** — node artık nokta
    sayısını logluyor, tek yapılacak dubadan 5/10/15/20 m'de durup okumak.

    Bu test bir DAVRANIŞI değil, bir SINIRI donduruyor: `min_cluster_size`
    büyütülürse menzil daha da kısalır ve kapı hafızası hiç dolmayabilir
    (12 m'lik kapıda iki direği aynı karede görme penceresi 8,8-15 m).
    """
    mcs = _node_varsayilanlari()["min_cluster_size"]
    assert mcs <= 5, (
        f"min_cluster_size={mcs} — 5'te bile tespit menzili ~8 m ölçüldü, "
        "büyütmek kapı hafızasının penceresini (8,8-15 m) tamamen kapatır. "
        "Büyütmek gerekiyorsa ÖNCE sahada menzil ölçülmeli."
    )
    # Menzil kısıtı gerçek: 12 m'deki duba sevk edilen eşikle elenmeli.
    # (Kırmızıya dönerse menzil İYİLEŞMİŞ demektir — o zaman bu testi ve
    # yukarıdaki tabloyu yeni ölçümle güncelle, silme.)
    rng = np.random.default_rng(11)
    uzak = rng.normal(0, 0.07, size=(4, 3)) + np.array([12.0, 0.0, 0.25])
    assert not detect_obstacles(uzak, URETIM), (
        "12 m'deki 4 noktalı duba artık tespit ediliyor — menzil modeli "
        "değişmiş, docstring'deki tabloyu yeniden ölç"
    )


def test_maliyet_nokta_sayisina_DEGIL_yogunluga_bagli() -> None:
    """Bütçe belgesinin dayandığı iddia: darboğaz N değil, çift sayısı.

    Aynı nokta sayısı (20k) iki farklı hacimde: seyrek olan hızlı, yoğun olan
    yavaş olmalı. Bu ilişki bozulursa yukarıdaki bütçe tablosu geçersizdir.
    """
    rng = np.random.default_rng(3)
    n = LIVOX_NOKTA_PER_KARE

    def sure_ms(kenar: float) -> float:
        pts = rng.uniform(-kenar / 2, kenar / 2, size=(n, 3)) * np.array([1, 1, 0.3])
        p = voxel_downsample(pts, URETIM.voxel_size)
        t0 = time.perf_counter()
        cluster_points(p, URETIM)
        return (time.perf_counter() - t0) * 1000.0

    seyrek = sure_ms(30.0)       # ölçüm: ~20 ms
    yogun = sure_ms(8.0)         # ölçüm: ~61 ms
    assert yogun > seyrek, (
        f"yoğun sahne ({yogun:.1f} ms) seyrekten ({seyrek:.1f} ms) hızlı çıktı "
        "— maliyet modeli değişmiş, bütçe tablosu yeniden ölçülmeli"
    )


# ------------------------------------------------------------- nöbetçi

def test_node_voxel_downsample_ACIK_sevk_ediyor() -> None:
    """🔴 Yükü taşıyan emniyet: `voxel_size` node'da AÇIK olmalı.

    Çekirdek varsayılanı 0.0 (kapalı, "davranış birebir" gerekçesiyle) ama
    node 0.1 declare ediyor — sevk edilen davranış node'unkidir. Kapatılırsa
    en kötü hâl İKİYE KATLANIYOR (ölçüm: 5 m'lik hacimde 112 → 204 ms), yani
    10 Hz bütçesi kesin aşılır ve arıza sessizdir.
    """
    voxel = _node_varsayilanlari()["voxel_size"]
    assert voxel > 0.0, (
        "perception_lidar_node voxel_size=0 sevk ediyor — kümeleme en kötü "
        "hâlde 204 ms (bütçe 100 ms). Kapatma gerekçesi ölçümle yazılmalı."
    )
    assert voxel <= URETIM.cluster_tolerance / 2.0, (
        f"voxel_size={voxel} m, cluster_tolerance={URETIM.cluster_tolerance} m'in "
        "yarısından büyük — downsample kümeleri koparabilir"
    )


def test_node_kumeleme_SURESINI_olcup_logluyor() -> None:
    """Sahadaki tek görünürlük kanalı: süre ölçülüp loglanmalı.

    09.07'de gecikme arızası yaşandı ama node hiçbir yerde süre yazmıyordu →
    "kaç saniye" sorusunun cevabı yoktu. Bu kaldırılırsa aynı körlük döner.
    """
    kaynak = _NODE_FILE.read_text(encoding="utf-8")
    assert "perf_counter" in kaynak, "kümeleme süresi ölçülmüyor"
    assert "sure_ms" in kaynak, "ölçülen süre loglanmıyor"
    assert "BÜTÇE AŞILDI" in kaynak, "bütçe aşımında uyarı basılmıyor"
    # Uyarı eşiği gerçek periyottan türemeli, elle yazılmış bir sayı olmamalı
    assert "1000.0 / max(self._beklenen_hz" in kaynak, (
        "bütçe eşiği beklenen kare hızından türetilmiyor"
    )
