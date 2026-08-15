"""
Girdap İDA — kenar dubası hafızası testleri (ROS'SUZ).

Kapsanan davranışlar (hepsi `edge_memory.py` docstring'indeki gerekçelere bağlı):
  · rengi kaybolan duba kenar KALIR (asıl kazanç, §0.17e)
  · rengi hiç görülmemiş duba kenar OLMAZ (hafıza uydurmuyor)
  · bilinen farklı sınıf hafızayı İPTAL eder (deniz şartlarıyla yer değiştirme)
  · hareket eden duba yeni kayıt doğurmaz (sınırsız büyüme yok)
  · ayarlanabilir eşik YOK (donmuş tasarım kuralı)
"""

# 🔴 F-A.1 (13.08.2026) — SÖZLEŞME İNCELDİ: "bir kez turuncu" YETMEZ, İKİ KEZ.
# Ölçüldü (canlı kamera, 80 kare / 8 945 tespit): kameranın ürettiği
# turuncuların tamamı TEK KARE parlamasıydı; aynı konumda sonraki 3/5/10
# karede tekrar sayısı **0**. Buna karşılık tek kare, konumu KALICI kenar
# dubası yapıp onu engel torbasından düşürüyordu (sahada 18 dakikada 76
# kalıcı kayıt biriktiği ölçüldü — her biri kaçınılmayan gerçek bir cisim).
# Onaya kadar cisim ENGEL olarak kalır (güvenli varsayılan). Aşağıdaki
# testlere eklenen ikinci kare bu onaydır; korudukları güvence aynen duruyor.

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
    assert kenar == [False, False], "F-A.1: tek kare turuncu ONAY DEĞİL"
    # Kare 1b: aynı direkler yine turuncu → onay doldu (bkz. F-A.1 notu)
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
    hafiza.siniflandir([(5.0, 2.0, R, TURUNCU)], TURUNCU)      # F-A.1 onayı
    assert hafiza.siniflandir([(5.0, 2.0, R, None)], TURUNCU) == [True]


def test_hic_gorulmemis_duba_KENAR_OLMAZ() -> None:
    """Hafıza uydurmuyor: rengi hiç sınıflanmamış küme ENGEL kalır.

    🆕 H1: cisim artık HARİTAYA yazılıyor (boyut 1) ama kenar SAYILMIYOR —
    ikisi ayrı şey. Haritada durması kaçınma içindir; kenar olmak kapı
    takibine gitmek demektir ve o yalnız RENK görüldüğünde olur.
    """
    hafiza = EdgeBuoyMemory()
    assert hafiza.siniflandir([(3.0, 1.0, R, CLASS_UNKNOWN)], TURUNCU) == [False]
    assert hafiza.boyut == 1, "H1: görülen cisim haritaya yazılmalı"


def test_uzaktaki_UNKNOWN_hatirlanan_dubaya_yapismaz() -> None:
    """Çakışma ölçütü: 0,30 m'lik banttan uzaktaki tespit başka bir cisimdir."""
    hafiza = EdgeBuoyMemory()
    hafiza.siniflandir([(10.0, 6.0, R, TURUNCU)], TURUNCU)
    # 1 m ötede bir LiDAR kümesi — aynı duba olamaz
    assert hafiza.siniflandir([(11.0, 6.0, R, CLASS_UNKNOWN)], TURUNCU) == [False]


# --------------------------------------------------------- çelişki / güvenlik

def test_bilinen_farkli_sinif_KENARLIGI_iptal_eder() -> None:
    """Şartname: dubalar deniz şartlarıyla YER DEĞİŞTİREBİLİR. Turuncunun eski
    yerine sarı engel gelirse o cisim kenar sayılmamalı, yoksa engele körleşiriz.

    🆕 H1: kayıt SİLİNMİYOR, SINIFI güncelleniyor. Silmek cismi haritadan
    düşürürdü — oysa cisim hâlâ orada, yalnız ne olduğu değişti. Silme,
    kaçınmayı da kaldırdığı için çarpma riskini ARTIRIRDI.
    """
    hafiza = EdgeBuoyMemory()
    hafiza.siniflandir([(8.0, 3.0, R, TURUNCU)], TURUNCU)
    assert hafiza.boyut == 1

    kenar = hafiza.siniflandir([(8.0, 3.0, R, SARI)], TURUNCU)
    assert kenar == [False], "sarı engel kenar sayıldı — çarpma riski"
    assert hafiza.boyut == 1, "cisim haritadan düştü — kaçınma da kaybolur"
    assert hafiza.celiskiyle_silinen == 1

    # Sınıf güncellendiği için sonraki UNKNOWN de kenara DÖNMEMELİ
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
    hafiza.siniflandir([(10.0, 6.0, R, TURUNCU)], TURUNCU)     # F-A.1 onayı
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
    # 🆕 H1: kapılan UNKNOWN artık KENDİ kaydını açıyor (haritaya girer,
    # kenar olmaz). Eskiden düşürülüyordu — yani kaçınması da kayboluyordu.
    assert hafiza.boyut == 2


def test_yaricap_mesajdan_gelir_oz_olcekli() -> None:
    """Eşleşme bandı tespitin KENDİ yarıçapından türer — büyük duba, büyük band."""
    hafiza = EdgeBuoyMemory()
    hafiza.siniflandir([(10.0, 0.0, 1.0, TURUNCU)], TURUNCU)   # r=1 m
    hafiza.siniflandir([(10.0, 0.0, 1.0, TURUNCU)], TURUNCU)   # F-A.1 onayı
    # 1,5 m ötesi: 1,0 + 0,5 = 1,5 → tam sınırda çakışıyor
    assert hafiza.siniflandir([(11.5, 0.0, 0.5, CLASS_UNKNOWN)], TURUNCU) == [True]
    # Aynı ayrım küçük dubalarda kenar SAYILMAZDI (0,15+0,15 = 0,30 m)
    hafiza2 = EdgeBuoyMemory()
    hafiza2.siniflandir([(10.0, 0.0, R, TURUNCU)], TURUNCU)
    hafiza2.siniflandir([(10.0, 0.0, R, TURUNCU)], TURUNCU)    # F-A.1 onayı
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
    # Kayıt alanları: konum + yarıçap + teşhis + sınıf/tazelik; tolerans YOK.
    # 🔴 F-A.1: `turuncu_sayaci` eklendi — bu bir EŞİK DEĞİL, DURUMDUR
    # (bu konum kaç karede turuncu görüldü). Onay çıtası kodda sabit **2**;
    # "tekrar"ın mantıksal asgarisi, ayarlanabilir bir sayı değil. Yukarıdaki
    # iki imza denetimi kuralın kendisini (parametre YOK) korumaya devam ediyor.
    assert set(HatirlananKenar.__dataclass_fields__) == {
        "x", "y", "r", "gorulme", "sinif", "taze", "turuncu_sayaci"
    }


def test_class_unknown_fuzyon_sozlesmesiyle_AYNI() -> None:
    """Sabit iki yerde yaşıyor (modül algı katmanına bağımlı olmasın diye) —
    ayrışırsa hafıza, UNKNOWN'ı çelişki sanıp kaydı siler ve arıza geri gelir."""
    from prototype.perception.fusion import CLASS_UNKNOWN as FUZYON_UNKNOWN

    assert CLASS_UNKNOWN == FUZYON_UNKNOWN


# ------------------------------------------------- H1: kalıcı dünya haritası

def test_H1_gorulmeyen_cisim_HARITADA_kalir() -> None:
    """🔴 H1'in ta kendisi (§0.21): kaybolan cisim planlayıcıdan düşmemeli.

    Eskiden engel torbası her karede sıfırdan kuruluyordu → o an görülmeyen
    cisim **yok** sayılıyordu. Kapıya yaklaşırken direkler önce kameranın
    69°'lik kadrajından, sonra LiDAR'ın (30 cm duba için ~8 m) menzilinden
    çıkıyor ve kapı ortadan kayboluyordu.
    """
    hafiza = EdgeBuoyMemory()
    for _ in range(2):                                        # F-A.1 onayı
        hafiza.siniflandir(
            [(10.0, 6.0, R, TURUNCU), (10.0, -6.0, R, TURUNCU)], TURUNCU
        )
    assert hafiza.hatirlananlar() == [], "ilk karede hepsi taze, hatırlanan yok"

    # Kare 2: yalnız SOL direk görünüyor, sağ direk menzil dışına çıktı
    hafiza.siniflandir([(10.0, 6.0, R, TURUNCU)], TURUNCU)

    hatirlanan = hafiza.hatirlananlar()
    assert len(hatirlanan) == 1, "görülmeyen direk haritadan düştü"
    (x, y, r, sinif), kenar = hatirlanan[0]
    assert (x, y) == (10.0, -6.0) and kenar is True
    assert sinif == TURUNCU, "sınıf unutulmuş — kapı takibine gidemez"


def test_H1_sinifsiz_engel_de_haritada_kalir_ama_KENAR_OLMAZ() -> None:
    """Sarı engel / iskele / tekne de hatırlanır — kaçınma kaybolmasın.

    Ama kenar bayrağı False: engel torbasında KALIR (güvenlik kuralı).
    """
    hafiza = EdgeBuoyMemory()
    hafiza.siniflandir([(20.0, 0.0, R, SARI)], TURUNCU)
    hafiza.siniflandir([], TURUNCU)                      # cisim görünmez oldu

    hatirlanan = hafiza.hatirlananlar()
    assert len(hatirlanan) == 1
    (_, _, _, sinif), kenar = hatirlanan[0]
    assert kenar is False, "sarı engel kenar sayıldı — torbadan çıkar, çarpılır"
    assert sinif == SARI


def test_H1_UNUTMA_YOK_kayit_sonsuza_kadar_kalir() -> None:
    """🔑 Kaptan kararı (09.08): süreye bağlı unutma YOK.

    Gerekçe: LiDAR dubayı zaten ~8 m'den uzakta göremiyor, yani "görmüyorum
    demek ki yok" kuralı cismi **yok olduğu için değil menzil yetmediği
    için** silerdi. Ayrıca bedel asimetrik: hayalet duba yalnız gereksiz
    kaçınma, unutulan gerçek duba **çarpma** (Ç1, P1'de 16 puan).
    """
    hafiza = EdgeBuoyMemory()
    hafiza.siniflandir([(15.0, 2.0, R, TURUNCU)], TURUNCU)
    for _ in range(500):                                 # 50 saniye @10 Hz
        hafiza.siniflandir([], TURUNCU)
    assert len(hafiza.hatirlananlar()) == 1, "kayıt zamanla silinmiş"


def test_H1_geri_gelen_cisim_CIFT_sayilmaz() -> None:
    """Cisim tekrar görünürse hafıza kaydı tazelenmeli, ikinci kopya olmamalı."""
    hafiza = EdgeBuoyMemory()
    hafiza.siniflandir([(10.0, 6.0, R, TURUNCU)], TURUNCU)
    hafiza.siniflandir([], TURUNCU)                       # kayboldu
    assert len(hafiza.hatirlananlar()) == 1

    hafiza.siniflandir([(10.05, 6.0, R, TURUNCU)], TURUNCU)   # geri geldi
    assert hafiza.boyut == 1, "aynı cisim iki kez haritaya girdi"
    assert hafiza.hatirlananlar() == [], "taze cisim hâlâ 'görülmeyen' sayılıyor"


def test_H1_temizle_haritayi_da_sifirlar() -> None:
    hafiza = EdgeBuoyMemory()
    hafiza.siniflandir([(1.0, 1.0, R, TURUNCU)], TURUNCU)
    hafiza.temizle()
    assert hafiza.hatirlananlar() == []


# ------------------------------------------- KAR-11: menzil tabanlı unutma


def test_unutma_VERILMEZSE_eski_davranis_BIREBIR() -> None:
    """Geriye uyum: `unutma_menzili` yoksa hiçbir kayıt silinmez (09.08 kararı)."""
    m = EdgeBuoyMemory()
    m.siniflandir([(0.0, 0.0, 0.15, 0), (100.0, 0.0, 0.15, 0)], edge_class_id=0)
    n = m.boyut
    m.hatirlananlar((0.0, 0.0), 10.0)          # yalnız SÜZER
    assert m.boyut == n, "unutma istenmediği hâlde kayıt silinmiş"
    assert m.unutulan == 0


def test_unutma_menzil_disini_SILIYOR() -> None:
    """🔴 KAR-11'in düzeltmesi: süzmek yetmiyor, SİLMEK gerekiyor.

    Maliyet yayımda değil TARAMADA: `siniflandir()` her tespiti hafızadaki HER
    kayda karşı test ediyor. Canlı ölçüm: 2404 kayıt → döngü 117→1062 ms (9×).
    """
    m = EdgeBuoyMemory()
    m.siniflandir(
        [(0.0, 0.0, 0.15, 0), (5.0, 0.0, 0.15, 0), (100.0, 0.0, 0.15, 0)],
        edge_class_id=0,
    )
    assert m.boyut == 3
    m.hatirlananlar((0.0, 0.0), 10.0, unutma_menzili=20.0)
    assert m.boyut == 2, "100 m'deki kayıt unutulmalıydı"
    assert m.unutulan == 1


def test_unutma_menzili_YAKINDAKINI_KORUYOR() -> None:
    """Pay bırakılıyor: yayım menzilinin ötesindeki ama unutma menzilinin
    içindeki kayıt YAŞAR — araç dönünce hâlâ işimize yarayabilir. 09.08'in
    'unutma yok' gerekçesi (LiDAR ~8 m'de dubayı kaybediyor) böyle korunuyor.
    """
    m = EdgeBuoyMemory()
    m.siniflandir([(15.0, 0.0, 0.15, 0)], edge_class_id=0)
    m.hatirlananlar((0.0, 0.0), 10.0, unutma_menzili=20.0)   # 15 m: yayım DIŞI
    assert m.boyut == 1, "yayım menzili dışı diye SİLİNMEMELİ"
    assert m.unutulan == 0


def test_unutma_torbayi_SINIRLIYOR() -> None:
    """Asıl amaç: tekrarlanan çağrılarda torba sınırsız büyümesin.

    Odometri sıçraması (KAR-06: 25 ms'de 6,54 m) aynı dubayı tekrar tekrar
    kaydettiriyordu; hareketsiz teknede bile 2404 kayda çıkmıştı.
    """
    m = EdgeBuoyMemory()
    for i in range(60):
        m.siniflandir([(50.0 + i, 0.0, 0.15, 0)], edge_class_id=0)
        m.hatirlananlar((0.0, 0.0), 10.0, unutma_menzili=20.0)
    assert m.boyut == 0, f"torba sinirlanmadi: {m.boyut} kayit"
    assert m.unutulan >= 60


# ------------------------------- KAR-11 kök neden: konum gürültüsü toleransı


def _sahne(n: int = 30, gurultu: float = 0.0, rnd=None):
    """n cisimli sabit sahne; `gurultu` kadar konum oynaması eklenir.

    ⚠️ `rnd` KARELER ARASI PAYLAŞILMALI. Her çağrıda yeni bir `Random(tohum)`
    kurulursa her kare AYNI gürültüyü alır, konumlar hiç oynamaz ve test
    ölçmek istediği şeyi ölçmez (12.08'de bu hataya düşüldü).
    """
    import random
    rnd = rnd or random.Random(1)
    return [
        (float(i % 6) * 2.0 + rnd.uniform(-gurultu, gurultu),
         float(i // 6) * 2.0 + rnd.uniform(-gurultu, gurultu),
         0.3, CLASS_UNKNOWN)
        for i in range(n)
    ]


def test_SABIT_sahne_100_karede_hafizayi_BUYUTMEZ() -> None:
    """🔴 NÖBETÇİ: aynı sahne tekrar tekrar işlenince hafıza sabit kalmalı.

    12.08'de canlı Jetson'da hafıza 4 dakikada 964→1574 kayda çıkmıştı ve
    kontrol döngüsü 10 Hz'den **2,49 Hz**'e düşmüştü (A/B ile kanıtlandı:
    hafıza boşaltılınca 10,02 Hz'e döndü). Bu test o davranışın temel hâlini
    dondurur — tekilleştirme bozulursa CI kırmızı.
    """
    import random
    rnd = random.Random(1)
    m = EdgeBuoyMemory()
    for _ in range(100):
        m.siniflandir(_sahne(rnd=rnd), edge_class_id=0)
    assert m.boyut == 30, f"sabit sahnede hafiza buyudu: {m.boyut}"


def test_kucuk_gurultu_TOLERE_EDILIYOR() -> None:
    """Ölçülen tolerans: cisim yarıçapı (0,3 m) mertebesine kadar dayanıyor."""
    import random
    for g in (0.05, 0.1, 0.2):
        rnd = random.Random(1)
        m = EdgeBuoyMemory()
        for _ in range(100):
            m.siniflandir(_sahne(gurultu=g, rnd=rnd), edge_class_id=0)
        assert m.boyut <= 40, f"gurultu ±{g} m'de hafiza {m.boyut} kayda cikti"


def test_BUYUK_gurultu_hafizayi_PATLATIYOR_belgelenmis_kisit() -> None:
    """🔴 BİLİNEN KISIT — düzeltme burada DEĞİL, odometride.

    Ölçüm (12.08, laptop, 30 cisim / 100 kare):
        ±0,00-0,20 m →  30 kayıt   (kararlı)
        ±0,30 m      →  59
        ±0,50 m      → 132   ← patlama başlıyor
        ±1,00 m      → 259
        ±2,00 m      → 357

    Tolerans ~cisim yarıçapı kadar. KAR-06 ise **25 ms'de 6,54 m** sıçrama
    belgeliyor — toleransın **20 katı**. Yani hafızanın patlaması KAR-05/06'nın
    (füzyonun geçersiz/ışınlanan odometri yayınlaması) **semptomudur**;
    kaptanın ayrı listelediği KAR-11 ile KAR-05/06 tek zincirdir:

        bozuk odometri → dünya konumları metrelerce oynar → aynı cisim her
        karede yeni kayıt → hafıza büyür → tarama maliyeti artar →
        kontrol döngüsü 10 Hz'i tutturamaz

    Bu test kısıtı **dondurur**: birisi hafızaya yama yapıp "çözdüm" sanmasın.
    Gerçek çözüm füzyon tarafında.
    """
    import random
    rnd = random.Random(1)
    m = EdgeBuoyMemory()
    for _ in range(100):
        m.siniflandir(_sahne(gurultu=1.0, rnd=rnd), edge_class_id=0)
    assert m.boyut > 100, (
        "buyuk gurultude hafiza artik patlamiyor — odometri duzeldiyse bu test "
        "guncellenmeli, yoksa tekillestirme sessizce degismis olabilir"
    )


# ═══════════════════════════════════════════════════════════════════════════
# F-A.1 (13.08.2026) — TEK KARE TURUNCU KENAR DUBASI YAPMAZ
#
# 🔴 SAHA ARIZASI: 13.08 01:18-01:36 koşumunda kalıcı haritadaki "kenar"
# sayısı 1 → 25 → 39 → 54 → 76 diye **hiç azalmadan** büyüdü (`unutulan = 0`).
# Kamera kare başına ~1 yanlış turuncu üretiyordu ve TEK kare, o konumu KALICI
# kenar dubası yapıyordu. Kenar dubaları engel torbasından ÇIKARILDIĞI için
# (kapıdan geçebilmek adına, bilinçli tasarım) bu, 76 gerçek cismin MPPI'nin
# kaçınma listesinden sessizce düşmesi demekti.
#
# 📏 ÖLÇÜM (canlı kamera, 80 kare, 8 945 tespit): UNKNOWN 8 945 · sarı 7 ·
# turuncu 1. O tek turuncunun aynı konumda sonraki 3/5/10 karede tekrar
# sayısı **0**. → "≥2 kez" bütün yanlış pozitifleri eler.
# ⚠ SÖNÜM (zamanla unutma) BİLEREK EKLENMEDİ: aynı ölçüm sınıflandırmanın
# ne kadar seyrek olduğunu da gösteriyor (8 945'te 1); sönüm gerçek dubayı
# da silerdi. Doğru kaldıraç GİRİŞTE onay, çıkışta sönüm değil.
# ═══════════════════════════════════════════════════════════════════════════


def test_FA1_TEK_kare_turuncu_kenar_YAPMAZ() -> None:
    """Güvenli varsayılan: onay dolana kadar cisim ENGEL olarak kalır."""
    hafiza = EdgeBuoyMemory()
    assert hafiza.siniflandir([(10.0, 3.0, R, TURUNCU)], TURUNCU) == [False]
    assert hafiza.onaylanan == 0
    assert hafiza.onay_bekleyen_kare == 1


def test_FA1_IKI_kare_turuncu_ONAYLAR() -> None:
    hafiza = EdgeBuoyMemory()
    hafiza.siniflandir([(10.0, 3.0, R, TURUNCU)], TURUNCU)
    assert hafiza.siniflandir([(10.0, 3.0, R, TURUNCU)], TURUNCU) == [True]
    assert hafiza.onaylanan == 1


def test_FA1_OLCULEN_yanlis_pozitif_deseni_HIC_kenar_uretmez() -> None:
    """Sahada ölçülen desen: tek kare parlamaları, hep BAŞKA konumda.

    80 karelik canlı ölçümde tekrar sayısı 0'dı; bu test o deseni taklit eder
    ve hafızanın tek bir kenar dubası bile üretmemesini bekler.
    """
    hafiza = EdgeBuoyMemory()
    gecmis: list = []
    for i in range(40):                       # her karede başka yerde parlama
        # Ölçümdeki desen: konumlar TEKRARLAMIYOR (80 karede 0 tekrar).
        x = 5.0 + 0.9 * i
        y = -6.0 + 0.31 * i
        # Sahadaki gibi: LiDAR eski konumları HER karede görmeye devam eder,
        # yalnız sınıfsız olarak (ölçüm: 8 945 UNKNOWN / 1 turuncu).
        kare = [(gx, gy, R, CLASS_UNKNOWN) for (gx, gy) in gecmis if (gx, gy) != (x, y)]
        kare.append((x, y, R, TURUNCU))
        hafiza.siniflandir(kare, TURUNCU)
        if (x, y) not in gecmis:
            gecmis.append((x, y))
    assert hafiza.onaylanan == 0, "gezici yanlış pozitif kenar dubası üretti"
    assert hafiza.hatirlananlar() == [], "onaysız kayıt engel torbasından düştü"
    assert hafiza.onay_bekleyen_kare == 40


def test_FA1_GERCEK_duba_onay_sonrasi_rengi_kaybolsa_da_KENAR_KALIR() -> None:
    """§0.17e'nin kazancı korunuyor: onay dolduktan sonra hafıza eskisi gibi."""
    hafiza = EdgeBuoyMemory()
    for _ in range(2):
        hafiza.siniflandir([(12.0, 4.0, R, TURUNCU)], TURUNCU)
    assert hafiza.siniflandir([(12.0, 4.0, R, CLASS_UNKNOWN)], TURUNCU) == [True]
    assert hafiza.siniflandir([(12.0, 4.0, R, None)], TURUNCU) == [True]


def test_FA1_onaysiz_kayit_ENGEL_olarak_kalir() -> None:
    """🔴 Güvenlik: onaysız turuncu, kaçınma listesinden DÜŞMEMELİ."""
    hafiza = EdgeBuoyMemory()
    hafiza.siniflandir([(10.0, 3.0, R, TURUNCU)], TURUNCU)
    # Kayıt açıldı ama kenar değil → `hatirlananlar` kenar olarak vermez
    assert hafiza.boyut == 1
    assert hafiza.hatirlananlar() == []


def test_FA1_onay_TEK_kayda_baglidir_komsu_parlama_saymaz() -> None:
    """İki ayrı konumdaki birer parlama, birbirini onaylayamaz."""
    hafiza = EdgeBuoyMemory()
    hafiza.siniflandir([(10.0, 3.0, R, TURUNCU)], TURUNCU)
    assert hafiza.siniflandir([(30.0, -8.0, R, TURUNCU)], TURUNCU) == [False]
    assert hafiza.onaylanan == 0


def test_FA1_ARALIKLI_siniflandirilan_GERCEK_duba_yine_ONAYLANIR() -> None:
    """🌊 GÖL SENARYOSU (kapalı alanda ÖLÇÜLEMEZ, bu yüzden teste yazıldı).

    Kaptanın sorusu: *"şu an kapalı alandayız, gölde durum değişmez mi?"*
    Değişir: gölde GERÇEK turuncu dubalar var ve kamera onları her karede
    yakalayamayabilir. Onay kuralı "peş peşe" olsaydı, kare atlayan bir duba
    onayı HİÇ dolduramaz ve §0.17e'nin ölçülmüş kazancı (12 m'de 1/4 → 4/4
    güzergah noktası) yok olurdu.

    Bu yüzden kanıt SIFIRLANMAZ, ±1 ERİR: bir karede turuncu (+1), sınıfsız
    karede (−1). %50 turuncu gören bir duba yine onaylanır; tek kare parlaması
    ise aradaki sınıfsız karelerde erir (bir üstteki test).
    """
    hafiza = EdgeBuoyMemory()
    kenar = [False]
    for i in range(8):                        # dönüşümlü: turuncu, sınıfsız…
        cls = TURUNCU if i % 2 == 0 else CLASS_UNKNOWN
        kenar = hafiza.siniflandir([(12.0, 4.0, R, cls)], TURUNCU)
    assert hafiza.onaylanan == 1, "aralıklı sınıflandırılan gerçek duba onaylanmadı"
    assert kenar == [True]


# ═══════════════════════════════════════════════════════════════════════════
# FAZ 1 (15.08.2026, GIRDAP_DURUM §1.13/§1.14) — İKİZ KAYIT PATLAMASI
#
# 🔴 SAHA ARIZASI (göl bandı 15.08): hafıza 26 dakikada 3 573 kayda şişti;
# sahte kayıtların %63,5'i eşleşme bandının hemen dışında ([0,30–0,60) m)
# doğuyordu — LiDAR'ın kısmi görüşü aynı dubanın merkezini bakış açısına göre
# bir çapa kadar oynatıyor (§1.10c). İkizler `planning_node._huni_payi`'nin
# W'sunu düşürüp kaçınma payını direklerin %84,8'inde SIFIRLADI → tekne
# dubanın üstüne nişan aldı. Aynı arıza MIT Arcturus RB2026 raporunda (§1.15a).
#
# İki düzeltme: ① eşleşme bandı `r+r` → `r+r+duba_çapı` (kısmi görüş payı,
# şartname sabiti) ② `hatirlananlar()` içinde kadanslı kayıt↔kayıt
# konsolidasyonu (`_birlestir`).
# ═══════════════════════════════════════════════════════════════════════════


def test_FAZ1_kismi_gorus_ikizi_ACILMAZ() -> None:
    """§1.10b'nin ölçtüğü popülasyon: [0,30–0,60) m'de doğan kayıtlar.

    Aynı duba, bakış açısı değişince merkezi 0,45 m oynamış görünür —
    eski bant (0,30) yeni kayıt açardı; yeni bant (0,60) aynı kayda taşır.
    """
    hafiza = EdgeBuoyMemory()
    hafiza.siniflandir([(10.0, 6.0, R, TURUNCU)], TURUNCU)
    hafiza.siniflandir([(10.45, 6.0, R, TURUNCU)], TURUNCU)
    assert hafiza.boyut == 1, "kısmi görüş oynaması ikiz kayıt açtı (FAZ 1 bandı delik)"
    assert hafiza.acilan_kayit == 1


def test_FAZ1_gercek_iki_direk_BIRLESMEZ() -> None:
    """Bant güvenliği: geçilebilir en dar kapının direkleri bile (1,085 m)
    bandın (0,60 m) dışında — gerçek çift asla tek kayda inmez."""
    hafiza = EdgeBuoyMemory()
    hafiza.siniflandir(
        [(10.0, 6.0, R, TURUNCU), (10.0, 4.915, R, TURUNCU)], TURUNCU
    )
    assert hafiza.boyut == 2
    hafiza.hatirlananlar()                      # konsolidasyon da koşsun
    assert hafiza.boyut == 2, "konsolidasyon gerçek kapı direklerini yuttu"


def test_FAZ1_konsolidasyon_ikizleri_ERITIYOR() -> None:
    """Kayıt↔kayıt: iki kayıt SONRADAN birbirinin bandına sürüklenirse
    (odometri oynaması) ilk `hatirlananlar()` taramasında tek kayda iner."""
    hafiza = EdgeBuoyMemory()
    # 1,2 m arayla iki ayrı kayıt (bandın dışında, ikisi de meşru)
    hafiza.siniflandir([(10.0, 6.0, R, TURUNCU)], TURUNCU)
    hafiza.siniflandir([(11.2, 6.0, R, TURUNCU)], TURUNCU)
    assert hafiza.boyut == 2
    # ikincisi sürüklenip ilkinin bandına girdi (0,40 m)
    hafiza._kayitlar[1].x = 10.40
    hafiza.hatirlananlar()
    assert hafiza.boyut == 1, "çakışan kayıtlar konsolidasyonda birleşmedi"
    assert hafiza.birlestirilen == 1


def test_FAZ1_konsolidasyon_SINIF_CELISKISINI_yutmaz() -> None:
    """Güvenlik: bilinen ve FARKLI sınıflı iki kayıt banda girse de
    BİRLEŞMEZ — sarı engel turuncu direğe yutulursa MPPI ondan kaçınmayı
    bırakır (Ç1/Ç2 cezası). (Tespit↔kayıt yolundaki çelişki kuralı ayrı:
    orada H1 gereği sınıf GÜNCELLENİR — bu test yalnız `_birlestir`'i bağlar.)
    """
    hafiza = EdgeBuoyMemory()
    hafiza.siniflandir([(10.0, 6.0, R, TURUNCU)], TURUNCU)
    hafiza.siniflandir([(10.0, 6.0, R, TURUNCU)], TURUNCU)      # onay → TURUNCU
    hafiza.siniflandir([(11.0, 6.0, R, SARI)], TURUNCU)          # bandın dışında
    assert hafiza.boyut == 2
    hafiza._kayitlar[1].x = 10.40                # sarı kayıt banda sürüklendi
    hafiza.hatirlananlar()
    assert hafiza.boyut == 2, "farklı sınıflı komşu kayıt yutuldu (güvenlik ihlali)"


def test_FAZ1_konsolidasyon_onaylari_TOPLAR() -> None:
    """İkizler aynı dubanın kareleridir: `gorulme`/`turuncu_sayaci` birleşimde
    toplanır — kanıt kaybolmaz, kayıt kenarlığını koruyabilir."""
    hafiza = EdgeBuoyMemory()
    hafiza.siniflandir([(10.0, 6.0, R, TURUNCU)], TURUNCU)
    hafiza.siniflandir([(10.0, 6.0, R, TURUNCU)], TURUNCU)      # onaylı kenar
    # bandın dışında meşru ikinci kayıt açılır, sonra sürüklenir
    hafiza.siniflandir([(10.8, 6.0, R, TURUNCU)], TURUNCU)
    hafiza._kayitlar[1].x = 10.3
    hafiza.hatirlananlar()
    assert hafiza.boyut == 1
    k = hafiza._kayitlar[0]
    assert k.sinif == TURUNCU, "onaylı kenar sınıfı birleşimde kayboldu"
    # `gorulme` 1. geçişte artmaz (yalnız 2. geçişte) → iki kayıt 1+1 = 2;
    # `turuncu_sayaci` her turuncu karede artar → 2+1 = 3. Kanıt TOPLANDI.
    assert k.gorulme == 2 and k.turuncu_sayaci == 3


def test_FAZ1_bant_replay_senaryosu_hafiza_TEMIZLENIR() -> None:
    """§1.13'ün makro doğrulaması: aynı dubanın ±0,25 m oynayan tespitleri
    100 karede TEK kayıt kalmalı (eskiden 0,30 m bandı aşan her sıçrama
    yeni kayıt açıyordu → 26 dakikada 3 573)."""
    import random
    rnd = random.Random(7)
    hafiza = EdgeBuoyMemory()
    for _ in range(100):
        x = 10.0 + rnd.uniform(-0.25, 0.25)
        y = 6.0 + rnd.uniform(-0.25, 0.25)
        hafiza.siniflandir([(x, y, R, TURUNCU)], TURUNCU)
        hafiza.hatirlananlar()
    assert hafiza.boyut == 1, f"±0,25 m oynayan tek duba {hafiza.boyut} kayıt üretti"
