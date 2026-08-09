"""
Girdap İDA — kenar dubası hafızası testleri (ROS'SUZ).

Kapsanan davranışlar (hepsi `edge_memory.py` docstring'indeki gerekçelere bağlı):
  · rengi kaybolan duba kenar KALIR (asıl kazanç, §0.17e)
  · rengi hiç görülmemiş duba kenar OLMAZ (hafıza uydurmuyor)
  · bilinen farklı sınıf hafızayı İPTAL eder (deniz şartlarıyla yer değiştirme)
  · hareket eden duba yeni kayıt doğurmaz (sınırsız büyüme yok)
  · ayarlanabilir eşik YOK (donmuş tasarım kuralı)
"""

from __future__ import annotations

import inspect
import math

from prototype.mission.edge_memory import (
    CLASS_UNKNOWN,
    EdgeBuoyMemory,
    HatirlananKenar,
)

TURUNCU = 0        # kenar dubası (edge_buoy_class_id varsayılanı)
SARI = 1           # engel dubası
HEDEF = 2          # Parkur-3 hedef dubası
R = 0.15           # şartname: duba çapı 30 cm


# --------------------------------------------------------------- asıl kazanç

def test_rengi_kaybolan_duba_KENAR_KALIR() -> None:
    """§0.17e'nin ta kendisi: kapıya yaklaşınca direk kadrajdan çıkar,
    füzyon UNKNOWN geçirir — hafıza olmasa engel torbasına düşerdi."""
    hafiza = EdgeBuoyMemory()

    # Kare 1: kamera 12 m'lik kapının iki direğini de görüyor (8,8-15 m penceresi)
    kenar = hafiza.siniflandir(
        [(10.0, 6.0, R, TURUNCU), (10.0, -6.0, R, TURUNCU)], TURUNCU
    )
    assert kenar == [True, True]

    # Kare 2: araç yaklaştı, direkler 69°'lik kadrajdan çıktı → UNKNOWN
    kenar = hafiza.siniflandir(
        [(10.0, 6.0, R, CLASS_UNKNOWN), (10.0, -6.0, R, CLASS_UNKNOWN)], TURUNCU
    )
    assert kenar == [True, True], "rengi görünmeyen direk engel torbasına düştü"
    assert hafiza.hatirlanarak_kurtarilan == 2


def test_sinifsiz_tespit_de_hatirlanir() -> None:
    """class_id sayısal değilse `cls=None` gelir — o da UNKNOWN gibi ele alınır."""
    hafiza = EdgeBuoyMemory()
    hafiza.siniflandir([(5.0, 2.0, R, TURUNCU)], TURUNCU)
    assert hafiza.siniflandir([(5.0, 2.0, R, None)], TURUNCU) == [True]


def test_hic_gorulmemis_duba_KENAR_OLMAZ() -> None:
    """Hafıza uydurmuyor: rengi hiç sınıflanmamış küme engel kalır."""
    hafiza = EdgeBuoyMemory()
    assert hafiza.siniflandir([(3.0, 1.0, R, CLASS_UNKNOWN)], TURUNCU) == [False]
    assert hafiza.boyut == 0


def test_uzaktaki_UNKNOWN_hatirlanan_dubaya_yapismaz() -> None:
    """Çakışma ölçütü: 0,30 m'lik banttan uzaktaki tespit başka bir cisimdir."""
    hafiza = EdgeBuoyMemory()
    hafiza.siniflandir([(10.0, 6.0, R, TURUNCU)], TURUNCU)
    # 1 m ötede bir LiDAR kümesi — aynı duba olamaz
    assert hafiza.siniflandir([(11.0, 6.0, R, CLASS_UNKNOWN)], TURUNCU) == [False]


# --------------------------------------------------------- çelişki / güvenlik

def test_bilinen_farkli_sinif_hafizayi_IPTAL_eder() -> None:
    """Şartname: dubalar deniz şartlarıyla YER DEĞİŞTİREBİLİR. Turuncunun eski
    yerine sarı engel gelirse hafıza silinmeli, yoksa engele körleşiriz."""
    hafiza = EdgeBuoyMemory()
    hafiza.siniflandir([(8.0, 3.0, R, TURUNCU)], TURUNCU)
    assert hafiza.boyut == 1

    kenar = hafiza.siniflandir([(8.0, 3.0, R, SARI)], TURUNCU)
    assert kenar == [False], "sarı engel kenar sayıldı — çarpma riski"
    assert hafiza.boyut == 0
    assert hafiza.celiskiyle_silinen == 1

    # Silindiği için sonraki UNKNOWN de kurtarılmamalı
    assert hafiza.siniflandir([(8.0, 3.0, R, CLASS_UNKNOWN)], TURUNCU) == [False]


def test_parkur3_hedef_dubasi_kenar_sayilmaz() -> None:
    """Hedef duba (class 2) kenar olmamalı — P3'te ona ÇARPMAK gerekiyor."""
    hafiza = EdgeBuoyMemory()
    hafiza.siniflandir([(20.0, 0.0, R, TURUNCU)], TURUNCU)
    assert hafiza.siniflandir([(20.0, 0.0, R, HEDEF)], TURUNCU) == [False]


def test_temizle_hafizayi_sifirlar() -> None:
    hafiza = EdgeBuoyMemory()
    hafiza.siniflandir([(1.0, 1.0, R, TURUNCU)], TURUNCU)
    hafiza.temizle()
    assert hafiza.boyut == 0
    assert hafiza.siniflandir([(1.0, 1.0, R, CLASS_UNKNOWN)], TURUNCU) == [False]


# ------------------------------------------------------------ büyüme / atama

def test_hareket_eden_duba_YENI_kayit_dogurmaz() -> None:
    """Kayıt yerinde güncellenir → sınırsız büyüme yok.

    Duba 10 Hz'te kare başına ~0,10 m kayıyor (araç 1,05 m/s); 40 karede 4 m
    yol alsa bile tek kayıt kalmalı.
    """
    hafiza = EdgeBuoyMemory()
    x = 10.0
    for _ in range(40):
        hafiza.siniflandir([(x, 5.0, R, TURUNCU)], TURUNCU)
        x += 0.10
    assert hafiza.boyut == 1
    kx, ky, _ = hafiza.kayitlar()[0]
    assert math.isclose(kx, 13.9, abs_tol=1e-6) and ky == 5.0


def test_bir_kayit_ayni_karede_IKI_tespite_verilmez() -> None:
    """Tekil atama: iki ayrı küme tek hafıza kaydını paylaşamaz."""
    hafiza = EdgeBuoyMemory()
    hafiza.siniflandir([(10.0, 6.0, R, TURUNCU)], TURUNCU)
    # Aynı karede çakışma bandında iki tespit — yalnız en yakını kurtarılır
    kenar = hafiza.siniflandir(
        [(10.05, 6.0, R, CLASS_UNKNOWN), (10.20, 6.0, R, CLASS_UNKNOWN)], TURUNCU
    )
    assert kenar.count(True) == 1
    assert kenar[0] is True, "en yakın tespit kurtarılmalı"


def test_taze_renk_bayat_kayittan_ONCE_islenir() -> None:
    """1. geçiş renk, 2. geçiş hafıza: aynı karede turuncu görünen tespit
    kaydı kapar, komşu UNKNOWN ona yapışamaz."""
    hafiza = EdgeBuoyMemory()
    hafiza.siniflandir([(10.0, 6.0, R, TURUNCU)], TURUNCU)
    kenar = hafiza.siniflandir(
        [(10.20, 6.0, R, CLASS_UNKNOWN), (10.05, 6.0, R, TURUNCU)], TURUNCU
    )
    assert kenar[1] is True                    # renk görünen
    assert kenar[0] is False                   # kayıt kapılmış, UNKNOWN engel
    assert hafiza.boyut == 1                   # yeni kayıt açılmadı


def test_yaricap_mesajdan_gelir_oz_olcekli() -> None:
    """Eşleşme bandı tespitin KENDİ yarıçapından türer — büyük duba, büyük band."""
    hafiza = EdgeBuoyMemory()
    hafiza.siniflandir([(10.0, 0.0, 1.0, TURUNCU)], TURUNCU)   # r=1 m
    # 1,5 m ötesi: 1,0 + 0,5 = 1,5 → tam sınırda çakışıyor
    assert hafiza.siniflandir([(11.5, 0.0, 0.5, CLASS_UNKNOWN)], TURUNCU) == [True]
    # Aynı ayrım küçük dubalarda kenar SAYILMAZDI (0,15+0,15 = 0,30 m)
    hafiza2 = EdgeBuoyMemory()
    hafiza2.siniflandir([(10.0, 0.0, R, TURUNCU)], TURUNCU)
    assert hafiza2.siniflandir([(11.5, 0.0, R, CLASS_UNKNOWN)], TURUNCU) == [False]


# ------------------------------------------------------------- donmuş kural

def test_hafizada_ayarlanabilir_esik_YOK() -> None:
    """🔑 Donmuş tasarım kuralı (§0.0d): kapı yolunda tahmine dayalı sayı yok.

    `EdgeBuoyMemory` hiçbir eşik/tolerans parametresi ALMAMALI — eşleşme
    ölçüsü tespitin kendi yarıçapıdır. Buraya bir `match_radius` eklenirse
    ("sahada ayarlarız") kural delinmiş olur ve ayarlanacak bir şey YOKTUR:
    kapı geometrisi önceden bilinemez.
    """
    imza = inspect.signature(EdgeBuoyMemory.__init__)
    assert list(imza.parameters) == ["self"], (
        f"EdgeBuoyMemory'ye ayar parametresi eklenmiş: {list(imza.parameters)[1:]}"
    )
    imza_sinif = inspect.signature(EdgeBuoyMemory.siniflandir)
    assert list(imza_sinif.parameters) == ["self", "tespitler", "edge_class_id"]
    # Kayıt alanları: konum + yarıçap + teşhis sayacı; tolerans alanı yok.
    assert set(HatirlananKenar.__dataclass_fields__) == {"x", "y", "r", "gorulme"}


def test_class_unknown_fuzyon_sozlesmesiyle_AYNI() -> None:
    """Sabit iki yerde yaşıyor (modül algı katmanına bağımlı olmasın diye) —
    ayrışırsa hafıza, UNKNOWN'ı çelişki sanıp kaydı siler ve arıza geri gelir."""
    from prototype.perception.fusion import CLASS_UNKNOWN as FUZYON_UNKNOWN

    assert CLASS_UNKNOWN == FUZYON_UNKNOWN
