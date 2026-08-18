"""
Girdap İDA — Kamera-LiDAR bearing füzyonu çekirdeği (Sprint 3, ROS-bağımsız).

LiDAR (3D, renksiz cluster) ile kamera (2D bbox, renkli) tespitlerini ORTAK
KALİBRASYON OLMADAN eşleştirir: her iki sensörün bearing'i (yatay açısı)
hesaplanır, en yakın bearing'li çift greedy olarak birleştirilir.

⚠ TASARIM SINIRI (bilinçli, Sprint 4+'a bırakıldı):
    Gerçek intrinsic/extrinsic kamera projeksiyonu YOK. `bearing_from_camera`
    kamera bbox'ının yatay konumunu HFOV ile orantılı bir açıya çevirir —
    kaba bir yaklaşım. Kamera optik çerçevesi/base_link hizası varsayımına
    bağlı bir İŞARET KURALI içerir; gerçek donanımda sol/sağ ters çıkarsa
    `bearing_from_camera` içindeki işareti çevirmek yeterli olur (çağıran
    kod / çıktı sözleşmesi değişmez — bkz. CLAUDE.md Perception bölümü).

Eşleşmeyen LiDAR tespiti GÜVENLİK NEDENİYLE atılmaz — class_id=CLASS_UNKNOWN
(99) ile korunur (MPPI cost map'te hâlâ engel olarak sayılmalı).

🆕 Eşleşmeyen kamera tespiti ARTIK ATILMIYOR (2026-08-09, H3): algı tarafı
08.08'den beri `/perception/buoys_3d` ile kendi 3B konum kestirimini de
yayınlıyor (stereo, yoksa bbox genişliğinden pinhole — duba çapı şartname
sabiti Ø30 cm; geçerli bant 0,5-15 m). Konumu olan tespit `source="kamera"`
ile sonuca girer. Konumu YOKSA eski davranış: atılır.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Optional

#: LiDAR-only (eşleşmemiş, renksiz) tespit sınıfı — güvenlik: engel olarak tut.
#: Kamera sınıfları (0=parkur_kenari, 1=engel, 2=hedef) camera_buoys'ta tanımlı;
#: bu modül yalnız kendi placeholder sınıfını (99) ekler.
CLASS_UNKNOWN = 99


@dataclass
class LidarDetection:
    """/perception/obstacle_map'ten çıkarılan tek daire engel (base_link)."""

    x: float
    y: float
    radius: float


@dataclass
class CameraDetection:
    """/perception/buoys'tan çıkarılan tek bbox (normalize görüntü uzayı)."""

    bbox_cx: float     # yatay merkez, normalize [0, 1] (0=sol kenar, 1=sağ kenar)
    bbox_cy: float
    class_id: int
    score: float
    # 🆕 H3 (2026-08-09): kameranın KENDİ 3B konum kestirimi
    # (`/perception/buoys_3d`) — base_link, x=ileri, y=sol, metre.
    # None ise eski davranış (yalnız bearing) BİREBİR korunur.
    x: Optional[float] = None
    y: Optional[float] = None
    radius: Optional[float] = None

    @property
    def konumu_var(self) -> bool:
        return self.x is not None and self.y is not None


@dataclass
class FusedObstacle:
    """Birleştirilmiş çıktı — /perception/classified_obstacles ön-hali."""

    x: float
    y: float
    radius: float
    class_id: int
    score: float
    matched: bool      # True: sınıf kameradan geldi
    #: Konumun KAYNAĞI (H3) — teşhis + aşağı akış kararları için:
    #:   "lidar"  : LiDAR kümesi, sınıf yok (CLASS_UNKNOWN)
    #:   "fused"  : LiDAR konumu + kamera sınıfı — en güvenilir
    #:   "kamera" : yalnız kamera; LiDAR o mesafede kümeleyemedi. Konum
    #:              bbox genişliğinden kestirim → uzakta menzil hatası
    #:              büyük (15 m'de duba ~6 px, ±1 px ≈ %17), ama YANAL
    #:              hassasiyet iyi (12 m'de ~2 cm) — kapı nişanı için
    #:              gereken de odur.
    source: str = "lidar"


@dataclass
class FusionConfig:
    """Füzyon parametreleri (config/hardware.yaml perception.fusion bloğu)."""

    bearing_tolerance_rad: float = 0.15   # ~8.6° — eşleşme kabul eşiği
    camera_hfov_rad: float = 1.2          # OAK-D Lite yatay FOV yaklaşık değeri
    #: Kameranın gövdeye göre YAW'ı (rad, sol pozitif). Optik eksen pruva
    #: hattıyla çakışmıyorsa TÜM kamera bearing'leri sabit miktarda kayar.
    #: ÖLÇÜLDÜ 2026-08-11: +0.0415 rad (+2.38°, iskeleye dönük) —
    #: `docs/olcum_formu.md §3b`. Varsayılan 0.0 = eski davranış birebir.
    #: Değer `hardware.yaml tf.oak_frame.yaw`'dan launch ile beslenir.
    camera_yaw_rad: float = 0.0
    #: 🔴 KAPTAN KARARI 18.08.2026 — *"bilinmeyen engelleri füzyona sokma".*
    #: `False` → eşleşmeyen LiDAR kümesi çıkışa HİÇ girmez (CLASS_UNKNOWN
    #: üretilmez). Gerekçe sahadan: kameranın göremediği kapı direkleri
    #: `CLASS_UNKNOWN` ile engel torbasında kalıyor, `obstacle_margin`
    #: halkaları kapı açıklığının içini kaplıyor ve hedef "engel içinde"
    #: sayılıyor → RRT* her döngüde `goal engel/sınır içinde` ile başarısız,
    #: yeni yol üretilemiyor, araç bayat referansla yerinde dönüyor
    #: (18.08 Gazebo koşumu: kapı 0/8, 280 sn).
    #:
    #: ⚠️ EMNİYET BEDELİ AÇIK YAZILIYOR: bu bayrak `False` iken kameranın
    #: sınıflayamadığı GERÇEK bir engel (kütük, bot, sınıfsız duba, ağ)
    #: maliyet haritasına HİÇ girmez — araç ona doğru sürebilir. Kamera
    #: 69° görüyor, LiDAR 360°; yani yanda/arkada kalan her şey bilinmeyendir.
    #: Varsayılan bu yüzden `True` (eski, emniyetli davranış) bırakıldı;
    #: kararı uygulamak için `hardware.yaml`'da açıkça `false` verilir ve
    #: TEKNOFEST öncesi denetim listesinde gözden geçirilir.
    bilinmeyen_engelleri_tut: bool = True


def bearing_from_lidar(det: LidarDetection) -> float:
    """LiDAR cluster'ının base_link'e göre yatay açısı (rad, atan2(y,x))."""
    return math.atan2(det.y, det.x)


def bearing_from_camera(
    det: CameraDetection, hfov: float, yaw_rad: float = 0.0
) -> float:
    """Bbox yatay merkezinden kaba bearing yaklaşımı (rad).

    İşaret kuralı = `bearing_from_lidar` (atan2) ile AYNI: **sol pozitif.**
    Görüntünün sol yarısındaki nesne (bbox_cx<0.5) fiziksel olarak soldadır
    → + bearing. merkez (0.5) → 0; sol kenar → +hfov/2; sağ → −hfov/2.
    (F6.1 düzeltmesi: eski `(cx−0.5)·hfov` LiDAR'a göre TERSTİ; sentetik
    üretecin ters fonksiyonu hatayı maskeliyordu.) ⚠ Kamera ters/aynalı
    monte edilirse sahada yine burası tek değişim noktasıdır.

    `yaw_rad`: kameranın gövdeye göre dönüklüğü (sol pozitif). Bbox'tan gelen
    açı KAMERANIN kendi eksenine göredir; gövde eksenine çevirmek için yaw
    EKLENİR. Doğrulama (2026-08-11 saha ölçümü): duba gerçekte +1.65°'de,
    bbox −0.73° veriyordu, ölçülen yaw +2.38° → −0.73 + 2.38 = +1.65 ✓.
    Varsayılan 0.0 → eski davranış birebir korunur.
    """
    return (0.5 - det.bbox_cx) * hfov + yaw_rad


def _circular_diff(a: float, b: float) -> float:
    """İki açı arası [-π, π] farkı — atan2 sarımı (CLAUDE.md heading kuralı)."""
    return math.atan2(math.sin(a - b), math.cos(a - b))


#: Bearing eşleşmesinin menzil tutarlılığı payı (H3 yan kazancı, 2026-08-09).
#:
#: 🔴 **Kapattığı tehlike:** bearing eşleştirmesi MENZİL BİLMEZ — aynı yöndeki
#: her şeyi eşleştirir. 5 m'deki SARI ENGEL ile 14 m'deki TURUNCU KAPI DİREĞİ
#: aynı hizadaysa, engel "kapı direği" sınıfını alır, `planning_node` onu engel
#: torbasından ÇIKARIR ve araç doğrudan engele sürer (Ç1/Ç2 çarpma puanı).
#: Eskiden bu kontrol EDİLEMİYORDU: kameranın menzili yoktu. Artık var.
#:
#: Oran ölçüsü, kameranın kendi belirsizlik modelinden türer. Pinhole menzil
#: `D = K/w_px` olduğu için hata menzille KARESEL büyür: `dD = D²·dw/K`.
#: Algı tarafının kalibrasyonuyla (`w_px = 90,1/Z`) 15 m'de ±2 px ≈ ±5 m,
#: yani **%33**. 0,5 bunun üstünde güvenli pay bırakır → doğru eşleşmeyi
#: reddetmez, ama yukarıdaki 5 m ↔ 14 m karışmasını (%180) yakalar.
#: ⚠ Kameranın gerçek menzil hatası SUDA ölçülmedi; bu pay ölçümle
#: güncellenmeli (kamera menzili sistematik saparsa oran büyütülür).
_MENZIL_TUTARLILIK_ORANI = 0.5


def _konum_celisiyor(lid: LidarDetection, cam: CameraDetection) -> bool:
    """Bearing eşleşmesi iki sensörün KONUMUYLA çelişiyor mu.

    Kameranın 3B konumu yoksa kontrol edilemez → False (eski davranış birebir).
    """
    if not cam.konumu_var:
        return False
    ayrik = math.hypot(lid.x - cam.x, lid.y - cam.y)
    kamera_menzili = math.hypot(cam.x, cam.y)
    return ayrik > kamera_menzili * _MENZIL_TUTARLILIK_ORANI


def associate(
    lidar_list: list[LidarDetection],
    camera_list: list[CameraDetection],
    cfg: FusionConfig,
) -> list[FusedObstacle]:
    """Greedy en-yakın-bearing eşleştirme.

    ÜÇ GEÇİŞ (2. ve 3. 2026-08-09'da eklendi, H3):
      1. **Bearing** eşleştirme — greedy en yakın açı (eski davranış).
      2. **Konumsal** eşleştirme — kameranın 3B konumu varsa, bearing'in
         kaçırdığı çiftleri daire çakışmasıyla yakala (ayar eşiği yok).
      3. **Yalnız kamera** — hâlâ eşleşmemiş ama konumu olan tespitler
         sonuca `source="kamera"` ile girer (LiDAR o mesafede kümeleyemiyor).

    - Her LiDAR tespiti sonuca girer (eşleşmesin bile — CLASS_UNKNOWN ile).
    - Her kamera tespiti EN FAZLA bir LiDAR'a eşlenir (double-match yok).
    - Konumu OLMAYAN eşleşmemiş kamera tespiti yine atılır (eski davranış).
    - Aday çiftler bearing farkına göre küçükten büyüğe işlenir → global
      olarak en yakın çiftler önce kilitlenir (greedy, optimal değil ama
      kalibrasyonsuz bearing-only senaryoda yeterli).
    """
    candidates: list[tuple[float, int, int]] = []
    for i, lidar_det in enumerate(lidar_list):
        lidar_bearing = bearing_from_lidar(lidar_det)
        for j, camera_det in enumerate(camera_list):
            camera_bearing = bearing_from_camera(
                camera_det, cfg.camera_hfov_rad, cfg.camera_yaw_rad
            )
            diff = abs(_circular_diff(lidar_bearing, camera_bearing))
            if diff > cfg.bearing_tolerance_rad:
                continue
            # H3 yan kazancı: aynı YÖNDE ama çok farklı MENZİLDE olan çift
            # aynı cisim değildir (`_MENZIL_TUTARLILIK_ORANI` gerekçesi).
            if _konum_celisiyor(lidar_det, camera_det):
                continue
            candidates.append((diff, i, j))
    candidates.sort(key=lambda c: c[0])

    matched_camera_for_lidar: dict[int, int] = {}
    used_camera_idx: set[int] = set()
    for _diff, i, j in candidates:
        if i in matched_camera_for_lidar or j in used_camera_idx:
            continue                                  # biri zaten kilitlendi
        matched_camera_for_lidar[i] = j
        used_camera_idx.add(j)

    # 🆕 H3 — 2. GEÇİŞ: KONUMSAL eşleştirme (bearing kaçırdıysa kurtar).
    # Kameranın 3B konumu varsa, bearing toleransına takılmayan çiftleri
    # daire çakışmasıyla yakalarız: `d ≤ r_lidar + r_kamera`. Ayarlanabilir
    # eşik DEĞİL — iki katı cisim aynı yeri kaplayamaz (aynı ölçüt
    # `EdgeBuoyMemory`'de de kullanılıyor). Bearing yaklaşımı kalibrasyonsuz
    # ve kaba olduğu için (modül docstring'i) bu ikinci geçiş onu tamamlar.
    for j, cam in enumerate(camera_list):
        if j in used_camera_idx or not cam.konumu_var:
            continue
        en_iyi, en_yakin = None, math.inf
        for i, lid in enumerate(lidar_list):
            if i in matched_camera_for_lidar:
                continue
            d = math.hypot(lid.x - cam.x, lid.y - cam.y)
            if d <= lid.radius + (cam.radius or 0.0) and d < en_yakin:
                en_iyi, en_yakin = i, d
        if en_iyi is not None:
            matched_camera_for_lidar[en_iyi] = j
            used_camera_idx.add(j)

    fused: list[FusedObstacle] = []
    for i, lidar_det in enumerate(lidar_list):
        if i in matched_camera_for_lidar:
            camera_det = camera_list[matched_camera_for_lidar[i]]
            fused.append(
                FusedObstacle(
                    x=lidar_det.x, y=lidar_det.y, radius=lidar_det.radius,
                    class_id=camera_det.class_id, score=camera_det.score,
                    matched=True, source="fused",
                )
            )
        elif cfg.bilinmeyen_engelleri_tut:
            fused.append(
                FusedObstacle(
                    x=lidar_det.x, y=lidar_det.y, radius=lidar_det.radius,
                    class_id=CLASS_UNKNOWN, score=0.0, matched=False,
                    source="lidar",
                )
            )
        # else: kaptan kararı (18.08) — sınıflanmamış küme çıkışa girmez.
        # Bedeli `FusionConfig.bilinmeyen_engelleri_tut` docstring'inde.

    # 🆕 H3 — 3. GEÇİŞ: eşleşmeyen ama KONUMU OLAN kamera tespitleri.
    #
    # 🔴 Bu satırlar §0.19c/§0.20c'deki menzil boşluğunu kapatıyor. 30 cm duba
    # LiDAR'da ~8 m'de kesiliyor (yeterli nokta düşmüyor); kapı takibi ise
    # iki direği aynı karede görmek için 8,8-15 m'ye ihtiyaç duyuyor →
    # pencereler ÖRTÜŞMÜYORDU ve ölçümde model AÇIKKEN P1 çöküyordu
    # (0 kapı, 1/4 nokta). Kameranın kendi menzil kestirimi (bbox
    # genişliğinden, duba çapı şartname sabiti Ø30 cm) 0,5-15 m bandında
    # geçerli → tam o boşluğu dolduruyor.
    #
    # Eski davranış (`camera_positions` yoksa) BİREBİR korunur: konumu
    # olmayan eşleşmemiş kamera tespiti yine atılır.
    for j, cam in enumerate(camera_list):
        if j in used_camera_idx or not cam.konumu_var:
            continue
        fused.append(
            FusedObstacle(
                x=cam.x, y=cam.y,
                radius=cam.radius if cam.radius is not None else 0.0,
                class_id=cam.class_id, score=cam.score,
                matched=True, source="kamera",
            )
        )
    return fused
