"""Bilinmeyen (sınıflanmamış) engellerin füzyon çıkışına girip girmemesi.

🔴 KAPTAN KARARI 18.08.2026: *"bilinmeyen engelleri füzyona sokma doldurma."*

Gerekçe sahadan ölçüldü (Gazebo, tam yığın): kameranın göremediği kapı
direkleri `CLASS_UNKNOWN` ile engel torbasında kalıyor, `obstacle_margin`
halkaları kapı açıklığının içini kaplıyor, hedef "engel içinde" sayılıyor →

    RRT* ValueError('goal engel/sınır içinde') — eski referans korunuyor

her döngüde. Yeni yol üretilemiyor, araç bayat referansla yerinde dönüyor
(kapı 0/8, 280 sn).

⚠️ Bu dosya AYNI ZAMANDA emniyet bedelinin nöbetçisidir: bayrak `False` iken
kameranın sınıflayamadığı GERÇEK engel de düşer. Bu yüzden **varsayılan
`True`** (eski davranış) ve testler bunu donduruyor — kararın açıkça
`hardware.yaml`da verilmesi gerekir, sessizce sızmamalı.
"""

from __future__ import annotations

import pytest

from prototype.perception.fusion import (
    CLASS_UNKNOWN,
    CameraDetection,
    FusionConfig,
    LidarDetection,
    associate,
)


def _lidar(x: float, y: float, r: float = 0.15) -> LidarDetection:
    return LidarDetection(x=x, y=y, radius=r)


def test_VARSAYILAN_eski_emniyetli_davranis() -> None:
    """Varsayılan değişirse emniyet kuralı sessizce düşer — o yüzden kilitli."""
    assert FusionConfig().bilinmeyen_engelleri_tut is True, (
        "varsayılan `False`a çekilmiş: sınıflanmamış gerçek engeller (kütük, "
        "bot, ağ) maliyet haritasından sessizce düşer. Karar `hardware.yaml`da "
        "AÇIKÇA verilmeli."
    )


def test_ACIKKEN_eslesmeyen_kume_KORUNUR() -> None:
    cfg = FusionConfig(bilinmeyen_engelleri_tut=True)
    sonuc = associate([_lidar(5.0, 0.0)], [], cfg)
    assert len(sonuc) == 1
    assert sonuc[0].class_id == CLASS_UNKNOWN
    assert sonuc[0].matched is False


def test_KAPALIYKEN_eslesmeyen_kume_DUSER() -> None:
    """Kaptanın istediği davranış: sınıfsız küme çıkışa hiç girmez."""
    cfg = FusionConfig(bilinmeyen_engelleri_tut=False)
    assert associate([_lidar(5.0, 0.0)], [], cfg) == []


def test_KAPALIYKEN_bile_SINIFLI_engel_KORUNUR() -> None:
    """Bayrak yalnız BİLİNMEYENİ eler; sarı engel (sınıf 1) elenmemeli.

    Aksi hâlde kapı açılırken engel kaçınması da kapanırdı — bu, kaptanın
    istediği şey değil ve Parkur-2'yi imkânsız kılardı.
    """
    cfg = FusionConfig(bilinmeyen_engelleri_tut=False)
    # Kamera aynı bearing'de sınıf 1 görüyor → eşleşir, korunur.
    kam = CameraDetection(bbox_cx=0.5, bbox_cy=0.5,
                          class_id=1, score=0.9)
    sonuc = associate([_lidar(5.0, 0.0)], [kam], cfg)
    assert len(sonuc) == 1
    assert sonuc[0].class_id == 1
    assert sonuc[0].matched is True


@pytest.mark.parametrize("tut", [True, False])
def test_SINIFLI_tespit_iki_ayarda_da_AYNI(tut: bool) -> None:
    """Bayrak sınıflı yolu HİÇ değiştirmemeli — yoksa kapı takibi de kayar."""
    cfg = FusionConfig(bilinmeyen_engelleri_tut=tut)
    kam = CameraDetection(bbox_cx=0.5, bbox_cy=0.5,
                          class_id=0, score=0.8)
    sonuc = associate([_lidar(8.0, 0.0)], [kam], cfg)
    assert [(o.class_id, o.matched, o.source) for o in sonuc] == [
        (0, True, "fused")
    ]
