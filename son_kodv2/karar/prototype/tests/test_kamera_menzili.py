"""
Girdap İDA — H3: kameranın kendi 3B konumu füzyona giriyor mu (ROS'SUZ).

🔴 **ÇÖZDÜĞÜ ARIZA (§0.19c + §0.20c).** İki menzil penceresi örtüşmüyordu:

| pencere | aralık | neden |
|---|---|---|
| LiDAR duba görebiliyor | **0 – ~8 m** | 30 cm duba 10 m'de 5 nokta veriyor, voxel sonrası 4 kalıyor, `min_cluster_size=5` eliyor |
| Kamera 12 m kapının İKİ direğini birden görebiliyor | **8,8 – 15 m** | 69° FOV geometrisi (§0.17d) |

Kesişim ≈ yok. Ölçülen sonuç: model AÇIKKEN P1 **çöküyordu** (0/8 kapı,
1/4 güzergah noktası, tamamlama şartı sağlanmıyor) — model KAPALIYKEN
sorunsuz bitiyordu (53,75 puan). Yani `.pt` aracı iyileştirmiyor, bozuyordu.

Çözüm yeni bir algoritma değil: algı tarafı 08.08'den beri bbox
genişliğinden menzil kestirip (`gecit_mantik.menzil_coz`, duba çapı şartname
sabiti Ø30 cm, geçerli bant 0,5-15 m) `/perception/buoys_3d`'ye yayınlıyordu.
Karar tarafı bu topic'e **hiç abone değildi**.

Bu dosya üç şeyi donduruyor:
  1. Konumu olan kamera tespiti artık ATILMIYOR (asıl kazanç)
  2. Konumu OLMAYAN tespitte eski davranış BİREBİR korunuyor (geri uyum)
  3. Konumsal eşleştirme bearing'in kaçırdığını yakalıyor (çift sayma yok)
"""

from __future__ import annotations

import math

from prototype.perception.fusion import (
    CLASS_UNKNOWN,
    CameraDetection,
    FusionConfig,
    LidarDetection,
    associate,
)

CFG = FusionConfig()
KENAR, ENGEL = 0, 1
R = 0.15                      # şartname md 5.5.2.1: duba çapı 30 cm


def _kam(bearing_rad: float, cls: int = KENAR, *, konum=None) -> CameraDetection:
    """Bearing'i verilen kamera tespiti (`bearing_from_camera`'nın tersi)."""
    cx = 0.5 - bearing_rad / CFG.camera_hfov_rad
    x, y, r = konum if konum else (None, None, None)
    return CameraDetection(bbox_cx=cx, bbox_cy=0.5, class_id=cls, score=0.9,
                           x=x, y=y, radius=r)


# --------------------------------------------------------------- asıl kazanç

def test_LiDARIN_GOREMEDIGI_duba_artik_planlamaya_ULASIYOR() -> None:
    """§0.19c'nin ta kendisi: 12 m'deki duba LiDAR'da yok, kamerada var."""
    lidar = []                                    # LiDAR o mesafede kümeleyemedi
    kamera = [_kam(0.1, KENAR, konum=(12.0, 1.2, R))]

    fused = associate(lidar, kamera, CFG)

    assert len(fused) == 1, "konumu olan kamera tespiti hâlâ atılıyor"
    assert fused[0].source == "kamera"
    assert fused[0].class_id == KENAR, "sınıf taşınmadı → kapı takibi çalışmaz"
    assert math.isclose(fused[0].x, 12.0) and math.isclose(fused[0].y, 1.2)


def test_kapinin_IKI_DIREGI_de_ulasiyor() -> None:
    """Kapı takibi tek direkle kurulamaz — ikisi birden gelmeli (§0.17d)."""
    kamera = [
        _kam(+0.45, KENAR, konum=(11.0, +6.0, R)),      # sol direk
        _kam(-0.45, KENAR, konum=(11.0, -6.0, R)),      # sağ direk
    ]
    fused = associate([], kamera, CFG)
    assert len(fused) == 2
    assert all(f.class_id == KENAR and f.source == "kamera" for f in fused)


# ------------------------------------------------------------- geri uyumluluk

def test_konumu_YOKSA_eski_davranis_BIREBIR() -> None:
    """3B konum yoksa eşleşmemiş kamera tespiti yine atılır (eski sözleşme)."""
    fused = associate([], [_kam(0.1, KENAR)], CFG)
    assert fused == [], "konumsuz kamera tespiti sonuca sızdı"


def test_eslesen_ciftte_konum_LIDARDAN_gelir() -> None:
    """LiDAR varken onun konumu kazanır — kamera menzili yalnız YEDEK.

    Gerekçe ölçülmüş: bbox genişliğinden menzil 15 m'de ~%17 hatalı
    (duba ~6 px, ±1 px), LiDAR ise ≤2 cm (Livox spesifikasyonu @10 m).
    """
    lidar = [LidarDetection(x=8.0, y=0.5, radius=R)]
    kamera = [_kam(0.06, KENAR, konum=(8.6, 0.55, R))]   # kestirim biraz sapık

    fused = associate(lidar, kamera, CFG)

    assert len(fused) == 1, "aynı duba İKİ KEZ sayıldı"
    assert fused[0].source == "fused"
    assert math.isclose(fused[0].x, 8.0), "konum kameradan alınmış (LiDAR daha hassas)"
    assert fused[0].class_id == KENAR


def test_eslesmeyen_LIDAR_hala_UNKNOWN_engel() -> None:
    """Güvenlik kuralı bozulmadı: renksiz LiDAR kümesi engel olarak kalır."""
    fused = associate([LidarDetection(x=5.0, y=0.0, radius=R)], [], CFG)
    assert len(fused) == 1
    assert fused[0].class_id == CLASS_UNKNOWN and fused[0].source == "lidar"


# --------------------------------------------------- konumsal eşleştirme (2. geçiş)

def test_bearingin_KACIRDIGI_cift_konumdan_yakalaniyor() -> None:
    """Bearing kalibrasyonsuz ve kaba (modül docstring'i) — konum onu tamamlar.

    Burada bearing farkı toleransın (0,15 rad) ÜSTÜNDE ama iki daire
    çakışıyor: aynı fiziksel duba. Eskiden çift sayılırdı (biri UNKNOWN
    engel, biri kamera tespiti) → MPPI aynı cisme iki ceza uygulardı.
    """
    lidar = [LidarDetection(x=6.0, y=0.0, radius=0.30)]
    kamera = [_kam(0.40, KENAR, konum=(6.1, 0.05, 0.20))]   # bearing çok uzak

    fused = associate(lidar, kamera, CFG)

    assert len(fused) == 1, "aynı duba iki kez sonuca girdi"
    assert fused[0].source == "fused" and fused[0].class_id == KENAR


def test_UZAKTAKI_kamera_tespiti_yanlis_LIDARA_YAPISMAZ() -> None:
    """Çakışma ölçütü tekil atama yapmalı — uzaktaki ayrı bir cisimdir."""
    lidar = [LidarDetection(x=5.0, y=0.0, radius=R)]
    kamera = [_kam(0.02, ENGEL, konum=(14.0, 0.3, R))]      # 9 m ötede

    fused = associate(lidar, kamera, CFG)

    kaynaklar = sorted(f.source for f in fused)
    assert kaynaklar == ["kamera", "lidar"], f"beklenmedik: {kaynaklar}"


def test_bir_kamera_tespiti_EN_FAZLA_bir_kez_kullanilir() -> None:
    """Ne bearing ne konumsal geçiş aynı tespiti iki LiDAR'a bağlayamaz."""
    lidar = [
        LidarDetection(x=6.0, y=0.0, radius=0.30),
        LidarDetection(x=6.2, y=0.1, radius=0.30),
    ]
    kamera = [_kam(0.0, KENAR, konum=(6.1, 0.05, 0.20))]

    fused = associate(lidar, kamera, CFG)

    assert len(fused) == 2, "LiDAR tespiti kaybolmuş"
    assert sum(f.matched for f in fused) == 1, "tek kamera tespiti iki kez kullanıldı"


def test_bos_girdi_coke_MEZ() -> None:
    assert associate([], [], CFG) == []


# ----------------------------------------------------------- node sözleşmesi

def test_node_buoys3d_ye_abone_ve_INDEKS_hizasi_koruyor() -> None:
    """🔑 `buoys` ile `buoys_3d` indeksleri BİREBİR eşlenmeli.

    Algı node'u ikisini tek döngüde aynı sırayla dolduruyor → `detections[k]`
    ile `poses[k]` aynı tespit. Ama `camera_list` süzülerek kuruluyor
    (sayısal olmayan class_id atlanır), o yüzden ham indeks ayrıca taşınmalı.
    Bu kaybolursa **sınıf ile konum kayar** ve arıza sessizdir: turuncu
    dubanın sınıfı sarı dubanın konumuna yapışır.
    """
    from pathlib import Path

    kaynak = (
        Path(__file__).resolve().parents[2] / "ros2_ws" / "src"
        / "girdap_decision" / "girdap_decision" / "perception_fusion_node.py"
    ).read_text(encoding="utf-8")

    assert "/perception/buoys_3d" in kaynak, "buoys_3d aboneliği yok"
    assert "ham_idx" in kaynak, "ham indeks taşınmıyor — sınıf/konum kayar"
    assert "_konumlari_bagla" in kaynak
    # Uzunluk uyuşmazlığında sessizce yanlış konum verilmemeli
    govde = kaynak.split("def _konumlari_bagla")[1].split("def _to_camera_detection")[0]
    assert "len(konumlar) != len(detections.detections)" in govde, (
        "indeks sözleşmesi doğrulanmıyor — bozulursa sınıf yanlış konuma yapışır"
    )
    assert "return 0" in govde, "uyuşmazlıkta bearing-only'ye düşülmüyor"
    # Önbellek derinliği sync kuyruğuyla aynı olmalı (yoksa eşleşme anında düşer)
    assert "self._sync_queue_size" in kaynak


def test_YANLIS_SINIF_YAPISMASI_engellendi() -> None:
    """🔴 H3'ün yan kazancı — bearing tek başına ÇARPMAYA yol açabiliyordu.

    Senaryo: 5 m'de SARI ENGEL (LiDAR görüyor, kamera sınıflayamıyor çünkü
    o an kadrajda değil ya da küçük), 14 m'de TURUNCU KAPI DİREĞİ (kamera
    görüyor). İkisi aynı hizada. Bearing menzil bilmediği için engele
    "kapı direği" sınıfı yapışırdı → `planning_node._on_classified` onu
    engel torbasından ÇIKARIR → araç doğrudan engele sürer (Ç1/Ç2).

    Artık menzil tutarlılığı bunu eliyor: iki cisim de ayrı ayrı sonuca
    girer, engel UNKNOWN kalır (güvenlik kuralı) ve torbadan çıkmaz.
    """
    lidar = [LidarDetection(x=5.0, y=0.0, radius=R)]        # sarı engel
    kamera = [_kam(0.0, KENAR, konum=(14.0, 0.0, R))]        # uzaktaki direk

    fused = associate(lidar, kamera, CFG)

    yakin = [f for f in fused if abs(f.x - 5.0) < 1.0]
    assert len(yakin) == 1
    assert yakin[0].class_id == CLASS_UNKNOWN, (
        "5 m'deki engele 14 m'deki dubanın SINIFI yapıştı — engel torbasından "
        "çıkarılır ve araç ona sürer (Ç1/Ç2 çarpma puanı)"
    )
    assert len(fused) == 2, "uzaktaki kapı direği kayboldu"
