"""ENU orijini respawn'a dayanıklı mı — kaptanın F-M.12 değişikliğiyle etkileşim.

Kaptan `21621af` ile `fusion_node`'u `respawn=True` yaptı (§0.42d: düğüm iSAM2
tekilleşmesiyle ölmüş ve launch onu bir daha başlatmamıştı). Doğru bir karar,
ama bir yan etkisi var: orijin İLK GPS FIX'inden alındığı için yeniden doğan
süreç dünya çerçevesini aracın YENİ konumuna çakar.

Görev hedefleri bundan etkilenmez (mission topic'leri ENU-hizalı ÖTELEME taşır,
odom xy ile birlikte kayar) ama `planning_node`'da DÜNYA çerçevesinde biriken
her şey bozulur: `EdgeBuoyMemory`, geçilmiş kapı kayıtları (md 5.5.3.1 puan
sayacı), RRT* referansı, MPPI warm-start. O düğüm eşzamanlı yeniden başlamaz.
KAR-11'de tam bu tür bir kayma kenar hafızasını şişirmişti.
"""

from __future__ import annotations

import math

import pytest

gtsam = pytest.importorskip("gtsam", reason="gtsam yok — çekirdek füzyon testi")

from prototype.fusion.pipeline import FusionPipeline    # noqa: E402

LAT, LON = 36.85, 28.27


def test_orijin_ACIKCA_cakilabiliyor() -> None:
    """`set_origin` ilk fix'in yerine geçer — respawn sonrası geri yükleme yolu."""
    fp = FusionPipeline()
    assert fp.origin_latlon() is None
    fp.set_origin(LAT, LON)
    assert fp.origin_latlon() == (LAT, LON)


def test_cakili_orijin_ilk_fixle_EZILMIYOR() -> None:
    """🔑 Asıl kural: orijin çakılıysa gelen fix onu değiştirmemeli.

    Respawn'dan sonra araç başlangıç noktasından metrelerce uzakta olabilir;
    ilk fix orijini ezerse tam da engellemeye çalıştığımız kayma olur.
    """
    fp = FusionPipeline()
    fp.set_origin(LAT, LON)
    fp.on_gps(LAT + 0.001, LON + 0.001)       # ~111 m kuzey/doğu
    assert fp.origin_latlon() == (LAT, LON), "gelen fix orijini ezdi"


def test_cakili_orijin_KORUNURSA_konum_dogru_kaliyor() -> None:
    """Sayısal doğrulama: aynı orijinle iki ayrı süreç aynı ENU'yu üretir.

    Bu, "çerçeve korundu" iddiasının ölçülebilir hâli.
    """
    hedef_lat, hedef_lon = LAT + 0.0009, LON + 0.0012

    ilk = FusionPipeline()
    ilk.on_gps(LAT, LON)                       # orijin = ilk fix
    x1, y1 = ilk._latlon_to_enu(hedef_lat, hedef_lon)

    # "Respawn": yeni süreç, orijin KAYITTAN geri yuklendi
    sonra = FusionPipeline()
    sonra.set_origin(LAT, LON)
    x2, y2 = sonra._latlon_to_enu(hedef_lat, hedef_lon)

    assert x1 == pytest.approx(x2) and y1 == pytest.approx(y2)


def test_orijin_KORUNMAZSA_kayma_METRELERCE() -> None:
    """🔴 Düzeltmenin neden gerekli olduğunun sayısal kanıtı.

    Respawn olan süreç orijini yeniden alırsa, aynı fiziksel nokta iki farklı
    ENU koordinatına düşer. Fark = başlangıçtan respawn anına kadar kat edilen
    mesafe. `planning_node` bu iki çerçeveyi ayırt edemez.
    """
    hedef_lat, hedef_lon = LAT + 0.0009, LON + 0.0012

    ilk = FusionPipeline()
    ilk.on_gps(LAT, LON)
    x1, y1 = ilk._latlon_to_enu(hedef_lat, hedef_lon)

    # Araç 100 m kuzeye gitmişken respawn oldu ve orijini YENİDEN aldı
    kayan = FusionPipeline()
    kayan.on_gps(LAT + 0.0009, LON)
    x2, y2 = kayan._latlon_to_enu(hedef_lat, hedef_lon)

    kayma = math.hypot(x1 - x2, y1 - y2)
    assert kayma > 50.0, (
        f"kayma yalnizca {kayma:.1f} m — senaryo temsil etmiyor"
    )


def test_kosu_ORTASINDA_orijin_degistirilemez() -> None:
    """Orijin bir kez çakıldıktan sonra değiştirmek, engellemeye çalıştığımız
    sıçramayı üretir — sessizce izin verilmemeli."""
    fp = FusionPipeline()
    fp.on_gps(LAT, LON)
    with pytest.raises(RuntimeError, match="zaten cakili"):
        fp.set_origin(LAT + 0.01, LON)
