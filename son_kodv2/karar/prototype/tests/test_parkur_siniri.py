"""F-S.16 — parkur sınırı (kenar duba zinciri) nöbetçi testleri.

Bu testler üç şeyi donduruyor:
  1. Koridor **iki kenar zinciri arasıdır**, orta çizgi tüpü DEĞİL.
  2. Kapsam dışı üçüncü bir durumdur — parkura girmeden çıkış sayılmaz.
  3. PDÇ ve puan şartnamenin formülü (s.24-25), yorum değil.
"""

import math

import pytest

from prototype.mission.parkur_dunyasi import oku
from prototype.mission.parkur_siniri import (
    DISARIDA,
    ICERIDE,
    KAPSAM_DISI,
    ParkurDisiSayaci,
    ParkurSiniri,
    tutarli_yonlendir,
)


@pytest.fixture(scope="module")
def sinir() -> ParkurSiniri:
    s = ParkurSiniri.kapilardan(oku().kapilar)
    assert s is not None
    return s


def test_iki_kapidan_azinda_sinir_YOK():
    """Tek kapı bir kiriştir — alanı yok, sınır da yok (ceza uygulanmamalı)."""
    assert ParkurSiniri.kapilardan([]) is None
    assert ParkurSiniri.kapilardan([((0.0, 1.0), (0.0, -1.0))]) is None


def test_kapi_ortalari_ICERIDE(sinir: ParkurSiniri):
    for orta in sinir.ortalar:
        assert sinir.durum(orta) == ICERIDE, orta


def test_kenar_dubasinin_disi_DISARIDA(sinir: ParkurSiniri):
    """Kenar dubasının 1 m ötesi parkur dışıdır — sınır dubadan geçer."""
    for i, (sol, sag) in enumerate(sinir.kapilar):
        if i in (0, len(sinir.kapilar) - 1):
            continue                      # uç kapılar: kapsam kenarı, ayrı test
        orta = sinir.ortalar[i]
        for duba in (sol, sag):
            yon = (duba[0] - orta[0], duba[1] - orta[1])
            n = math.hypot(*yon)
            disari = (duba[0] + yon[0] / n, duba[1] + yon[1] / n)
            assert sinir.durum(disari) == DISARIDA, (i, disari)


def test_baslangic_noktasi_KAPSAM_DISI(sinir: ParkurSiniri):
    """Fırlatma noktası ilk kapının önünde — orada sınır tanımsız."""
    assert sinir.durum(oku().baslangic) == KAPSAM_DISI


def test_son_kapinin_otesi_KAPSAM_DISI(sinir: ParkurSiniri):
    """GN5 (Parkur-2) son kapının ötesinde — P1 sınırı orayı yargılamaz."""
    gn5 = oku().gn5
    assert gn5 is not None
    assert sinir.durum(gn5) == KAPSAM_DISI


def test_koridor_ORTA_CIZGI_TUPU_DEGIL(sinir: ParkurSiniri):
    """🔑 Ayırt edici: orta çizgiye yakın ama kenar zincirinin DIŞINDA nokta.

    (7,5, 4,0) noktası kapı-2 ortasına 4,7 m — yani "orta çizgi ± yarı
    genişlik (6 m)" tüpünün İÇİNDE. Ama sol kenar zincirinin
    (6,1)→(10,6) dışında kaldığı için parkur DIŞIDIR. Tüp yaklaşımı bunu
    kaçırırdı; zigzag parkurda tüp kirişlere göre eğik durur.
    """
    p = (7.5, 4.0)
    orta2 = sinir.ortalar[1]
    assert math.hypot(p[0] - orta2[0], p[1] - orta2[1]) < 6.0
    assert sinir.durum(p) == DISARIDA


def test_parkura_girmeden_cikis_SAYILMAZ(sinir: ParkurSiniri):
    """🔑 Araç parkura HİÇ girmeden koridorun yanından geçerse çıkış YAZILMAZ.

    Ölçü noktası bilerek `DISARIDA` olan bir yer (kapsam dışı değil): yalnız
    fırlatma noktasıyla sınanırsa "önce içeri girmiş olmalı" kapısı silinse
    bile test yeşil kalır — mutasyon turu bunu yakaladı.
    """
    sayac = ParkurDisiSayaci(sinir)
    assert sinir.durum((7.5, 4.0)) == DISARIDA
    for t in range(10):
        sayac.adim((7.5, 4.0), float(t))
    for t in range(10, 15):                   # sonra kapsam dışına savrul
        sayac.adim((-10.0, 0.0), float(t))
    sayac.bitir(15.0)
    assert sayac.cikis_sayisi == 0
    assert sayac.puan(1) == pytest.approx(24.0)


def test_ic_dis_ic_TEK_cikis(sinir: ParkurSiniri):
    sayac = ParkurDisiSayaci(sinir)
    sayac.adim(sinir.ortalar[1], 0.0)         # içeri
    sayac.adim((7.5, 4.0), 1.0)               # dışarı
    sayac.adim((7.5, 4.0), 2.0)               # hâlâ dışarı — tek epizot
    sayac.adim(sinir.ortalar[2], 3.0)         # geri içeri
    sayac.bitir(4.0)
    assert sayac.cikis_sayisi == 1
    assert sayac.etkin_cikis == 1
    assert sayac.toplam_sure == pytest.approx(2.0)
    assert sayac.puan(1) == pytest.approx(18.0)


def test_40_saniyeden_uzun_kalis_IKI_cikis(sinir: ParkurSiniri):
    sayac = ParkurDisiSayaci(sinir)
    sayac.adim(sinir.ortalar[1], 0.0)
    sayac.adim((7.5, 4.0), 1.0)
    sayac.adim(sinir.ortalar[2], 42.0)        # 41 sn dışarıda
    sayac.bitir(43.0)
    assert sayac.cikis_sayisi == 1
    assert sayac.etkin_cikis == 2
    assert sayac.puan(1) == pytest.approx(12.0)


def test_kosum_disarida_biterse_epizot_KAPANIR(sinir: ParkurSiniri):
    sayac = ParkurDisiSayaci(sinir)
    sayac.adim(sinir.ortalar[1], 0.0)
    sayac.adim((7.5, 4.0), 1.0)
    sayac.bitir(6.0)
    assert sayac.cikis_sayisi == 1
    assert sayac.toplam_sure == pytest.approx(5.0)


def test_puan_formulu_sartnamenin_kendisi(sinir: ParkurSiniri):
    """P1: 24 − 6×PDÇ (tavan 4) · P2: 30 − 6×PDÇ (tavan 5)."""
    sayac = ParkurDisiSayaci(sinir)
    for pdc, beklenen_p1, beklenen_p2 in [
        (0, 24.0, 30.0), (1, 18.0, 24.0), (2, 12.0, 18.0),
        (4, 0.0, 6.0), (7, 0.0, 0.0),
    ]:
        sayac.epizotlar = [1.0] * pdc
        assert sayac.puan(1) == pytest.approx(beklenen_p1)
        assert sayac.puan(2) == pytest.approx(beklenen_p2)
    with pytest.raises(ValueError):
        sayac.puan(3)


def test_BITIS_epizodu_kapatir_KACIS_kapatmaz(sinir: ParkurSiniri):
    """İki kapsam dışı hâli aynı şey değil (14.08'de ölçümü 43→107 s şişirdi).

    · son kapı düzleminin ötesi = parkur BİTTİ → açık epizot kapanır
    · giriş düzleminin gerisi   = KAÇIŞ → epizot açık kalır, süre birikir
    """
    gn5 = oku().gn5
    bitis = ParkurDisiSayaci(sinir)
    bitis.adim(sinir.ortalar[1], 0.0)
    bitis.adim((7.5, 4.0), 1.0)               # dışarı
    bitis.adim(gn5, 5.0)                      # son kapının ötesi = bitiş
    bitis.bitir(400.0)
    assert bitis.toplam_sure == pytest.approx(4.0)

    kacis = ParkurDisiSayaci(sinir)
    kacis.adim(sinir.ortalar[1], 0.0)
    kacis.adim((7.5, 4.0), 1.0)
    kacis.adim((-10.0, 0.0), 5.0)             # girişin gerisine kaçtı
    kacis.bitir(400.0)
    assert kacis.toplam_sure == pytest.approx(399.0)
    assert kacis.etkin_cikis == 2             # 40 sn kuralı


def test_derinlik_SIG_asmayi_gercek_savrulmadan_ayirir(sinir: ParkurSiniri):
    """PDÇ ikili; derinlik olmadan 20 cm ile 5 m aynı görünür."""
    # (7,5 · 4,0) sol kenar zincirinin (6,1)→(10,6) 0,70 m dışında.
    assert sinir.kenara_uzaklik((7.5, 4.0)) == pytest.approx(0.7028, abs=1e-3)
    # Kapı ortası kenardan uzaktır — ama YARI GENİŞLİK kadar değil: zigzagda
    # en yakın kenar kirişin ucu değil, komşu kapıya giden eğik zincirdir.
    assert sinir.kenara_uzaklik(sinir.ortalar[3]) == pytest.approx(3.75, abs=0.05)

    sayac = ParkurDisiSayaci(sinir)
    sayac.adim(sinir.ortalar[1], 0.0)
    sayac.adim((7.5, 4.0), 1.0)
    sayac.adim((9.0, 9.0), 2.0)               # daha derin
    sayac.adim(sinir.ortalar[2], 3.0)
    sayac.bitir(4.0)
    assert sayac.cikis_sayisi == 1
    assert len(sayac.derinlikler) == 1
    assert sayac.en_derin > 2.0               # tek epizodun EN DERİN anı
    assert sayac.en_derin == pytest.approx(sinir.kenara_uzaklik((9.0, 9.0)))


def test_ters_etiketli_kapi_HIZALANIR(sinir: ParkurSiniri):
    """Algı bir kapıyı ters etiketlerse koridor kelebeğe döner — hizalayıcı şart.

    `GateFollower` sol/sağ'ı yaklaşma yönüne göre verir; eğik yaklaşılan tek
    kare etiketi ters yazabilir. Ters çiftle kurulmuş sınır kapı ortasını
    "dışarıda" sanar; `tutarli_yonlendir` bunu düzeltir.
    """
    bozuk = list(sinir.kapilar)
    bozuk[3] = (bozuk[3][1], bozuk[3][0])              # kapı-4 ters
    bozuk_sinir = ParkurSiniri.kapilardan(bozuk)
    assert bozuk_sinir is not None
    # Kelebek: kapı-3 ile kapı-5 arasındaki gerçek koridor "dışarı" oluyor.
    # (Kapı ORTALARI yanıltıcı — çift ters çevrilince orta nokta değişmez.)
    for p in [(16.0, 0.0), (16.0, 2.0), (20.0, 0.0), (16.0, -3.0)]:
        assert sinir.durum(p) == ICERIDE
        assert bozuk_sinir.durum(p) == DISARIDA, p

    duzeltilmis = ParkurSiniri.kapilardan(tutarli_yonlendir(bozuk))
    assert duzeltilmis is not None
    for orta in duzeltilmis.ortalar:
        assert duzeltilmis.durum(orta) == ICERIDE
    assert duzeltilmis.cokgen == sinir.cokgen


def test_hizalayici_DOGRU_zinciri_BOZMAZ(sinir: ParkurSiniri):
    assert tutarli_yonlendir(list(sinir.kapilar)) == list(sinir.kapilar)


def test_sinir_YOKSA_sayac_sessiz():
    """Kapı görülmeden sınır yok — sayaç hiçbir şey saymaz, çökmez."""
    sayac = ParkurDisiSayaci(None)
    assert sayac.adim((0.0, 0.0), 0.0) == KAPSAM_DISI
    sayac.bitir(1.0)
    assert sayac.cikis_sayisi == 0
