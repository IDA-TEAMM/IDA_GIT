#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""gecit_mantik.py — saf geometri testleri (ROS/depthai/kamera GEREKMEZ).

Neden ayrı modül: geçit seçimi puanın doğrudan kaynağı ((G/KD)×10 ve ×40,
şartname 5.5.4.2) ama navigator.py depthai+rclpy import ettiği için kamerasız
test edilemiyordu. Saf mantık buraya alındı → masada test edilir.

Koşum: python3 -m pytest girdap_ida_algi/test/test_gecit_mantik.py -q
"""
import math
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from girdap_ida_algi import gecit_mantik as gm  # noqa: E402


# --------------------------------------------------- odak / pinhole menzil
def test_odak_px_hfov_ile_tutarli():
    """f = (W/2)/tan(HFOV/2). IMX214 HFOV 69°, NN 416 px → ~302,6 px."""
    f = gm.odak_px(416, math.radians(69.0))
    assert 300.0 < f < 306.0


def test_mesafe_genislikten_bilinen_deger():
    """D = f·W/w_px. 30 cm duba, f=302,6 → 15 m'de ~6 px, 5 m'de ~18 px."""
    f = gm.odak_px(416, math.radians(69.0))
    assert gm.mesafe_genislikten(6.05, 0.30, f) == pytest.approx(15.0, rel=0.02)
    assert gm.mesafe_genislikten(18.2, 0.30, f) == pytest.approx(5.0, rel=0.02)


def test_mesafe_genislikten_gecersiz_piksel_None():
    f = gm.odak_px(416, math.radians(69.0))
    assert gm.mesafe_genislikten(0.0, 0.30, f) is None
    assert gm.mesafe_genislikten(-3.0, 0.30, f) is None


# ------------------------------------------------ stereo ↔ mono tutarlılığı
def test_stereo_mono_uyumluysa_tutarli():
    assert gm.menzil_tutarli(5.0, 5.4, bagil_tol=0.35) is True


def test_stereo_mono_celisirse_tutarsiz():
    """OAK-D Lite baseline 7,5 cm → ~8 m ötesinde stereo Z metrelerce şaşar.
    Mono 6 m derken stereo 14 m diyorsa bu çift GÜVENİLMEZ."""
    assert gm.menzil_tutarli(14.0, 6.0, bagil_tol=0.35) is False


def test_menzil_tutarli_eksik_olcumde_reddetmez():
    """Ölçümlerden biri yoksa çelişki İDDİA EDİLEMEZ — kör reddetme yapma."""
    assert gm.menzil_tutarli(None, 6.0) is True
    assert gm.menzil_tutarli(6.0, None) is True


# ------------------------------------------- A-5: yanlış (aynı taraf) çift
def test_arada_duba_yoksa_cift_gecerli():
    """Karşılıklı çift: aralarında başka kenar dubası olmamalı."""
    assert gm.arada_duba_var(-1.5, 6.0, 1.5, 6.0, []) is False


def test_aralarinda_duba_varsa_cift_REDDEDILIR():
    """(-4,6) ile (+4,6) 'çift' sanılırsa aslında koridorun İKİ YANI seçilmiştir;
    aralarındaki (0,6) dubası bunu ele verir → yanlış geçitten sürüş (parkur
    dışı cezası + geçiş sayılmaz)."""
    assert gm.arada_duba_var(-4.0, 6.0, 4.0, 6.0, [(0.0, 6.0)]) is True


def test_arada_ama_cok_uzaktaki_duba_sayilmaz():
    """Bearing'i arada ama çok daha uzaktaki duba bir sonraki geçide aittir."""
    assert gm.arada_duba_var(-1.5, 6.0, 1.5, 6.0, [(0.0, 14.0)]) is False


# ----------------------------------------- A-3: FARKLI geçit sayımı (dedupe)
def test_ilk_gecit_her_zaman_yeni():
    assert gm.yeni_gecit_mi(3.0, 1.0, [], ayirt_m=3.0) is True


def test_ayni_gecitten_tekrar_gecmek_SAYILMAZ():
    """Şartname G tanımı: 'FARKLI karşılıklı kenar dubaları arasından geçiş
    sayısı' — manevrada aynı geçitten dönülürse ikinci kez sayılmamalı."""
    assert gm.yeni_gecit_mi(3.2, 1.1, [(3.0, 1.0)], ayirt_m=3.0) is False


def test_uzaktaki_yeni_gecit_sayilir():
    assert gm.yeni_gecit_mi(12.0, 4.0, [(3.0, 1.0)], ayirt_m=3.0) is True


def test_dedupe_tum_gecilenlere_bakar():
    gecilenler = [(3.0, 1.0), (12.0, 4.0), (20.0, 2.0)]
    assert gm.yeni_gecit_mi(12.5, 4.2, gecilenler, ayirt_m=3.0) is False
    assert gm.yeni_gecit_mi(28.0, 3.0, gecilenler, ayirt_m=3.0) is True


# ------------------------- ARADAN mı geçti, yandan mı dolaştı? (kritik)
# Geçit: orta nokta (0,0), duba çizgisi y ekseni boyunca (teğet 0,1),
# ileri normal (1,0), yarı genişlik 1.5 m (yani dubalar (0,-1.5) ve (0,+1.5)).
GECIT = dict(mx=0.0, my=0.0, nx=1.0, ny=0.0, tx=0.0, ty=1.0,
             yari_genislik=1.5, ek_yol=1.0)


def test_ortadan_gecerse_SAYILIR():
    assert gm.gecitten_gecti(px=1.2, py=0.2, **GECIT) is True


def test_duzlemi_gecmediyse_sayilmaz():
    """Henüz geçide varmadı (ileri projeksiyon ek_yol'un altında)."""
    assert gm.gecitten_gecti(px=0.5, py=0.0, **GECIT) is False


def test_YANDAN_DOLASIRSA_SAYILMAZ():
    """🔴 Asıl hata buydu: düzlemi aşmak yetiyordu. Tekne dubaların DIŞINDAN
    dolaşırsa şartnameye göre geçiş DEĞİL (üstelik parkur dışına çıkma cezası),
    bizim sayacımız 'geçti' saymamalı."""
    assert gm.gecitten_gecti(px=1.2, py=4.0, **GECIT) is False


def test_duba_hizasinda_sinir():
    """Duba merkezinin tam hizası sınır kabul (içeride sayılır)."""
    assert gm.gecitten_gecti(px=1.2, py=1.5, **GECIT) is True
    assert gm.gecitten_gecti(px=1.2, py=1.8, **GECIT) is False


def test_geri_yonde_gecerse_sayilmaz():
    """Normal, geçidi görürken burun yönüne kilitli — geriye geçiş sayılmaz."""
    assert gm.gecitten_gecti(px=-2.0, py=0.0, **GECIT) is False


def test_yari_genislik_bilinmiyorsa_eski_davranis():
    """Geçit FOV'dan kaybolup tek bearing'den kurulduysa genişlik bilinmez;
    o durumda yalnız düzlem testi yapılır (bilgi yokken kör reddetme yok)."""
    g = dict(GECIT); g["yari_genislik"] = None
    assert gm.gecitten_gecti(px=1.2, py=4.0, **g) is True


# ===================== LETTERBOX PAYI (varsayım yerine hesap) ===============
def test_letterbox_4_3_kaynaktan_kareye():
    """4:3 kaynak (640x480 / 1440x1080) → 416x416 NN: üst/alt şerit %12,5.
    Eski kod bu 0,125'i SABİT yazıyordu; artık gerçek boyutlardan hesaplanıyor."""
    assert gm.letterbox_payi(416, 416, 640, 480) == pytest.approx(0.125)
    assert gm.letterbox_payi(416, 416, 1440, 1080) == pytest.approx(0.125)


def test_letterbox_16_9_kaynakta_pay_BUYUR():
    """16:9 kaynak kareye sığdırılırsa şerit büyür (%21,9) — sabit 0,125
    kullanmak bbox'ları dikeyde kaydırırdı."""
    assert gm.letterbox_payi(416, 416, 1920, 1080) == pytest.approx(0.21875)


def test_letterbox_ayni_oranda_pay_YOK():
    """Kaynak ve NN aynı orandaysa şerit yoktur."""
    assert gm.letterbox_payi(416, 416, 480, 480) == pytest.approx(0.0)
    assert gm.letterbox_payi(640, 480, 1440, 1080) == pytest.approx(0.0)


def test_letterbox_gecersiz_boyutta_None():
    """Cihaz dönüşüm bilgisi vermezse çağıran taraf yedeğe düşsün."""
    assert gm.letterbox_payi(0, 416, 640, 480) is None
    assert gm.letterbox_payi(416, 416, 640, 0) is None


# ===== son_kodv2/gate_follower'dan ALINAN daha iyi ölçütler (2026-08-04) =====
# Gerekçe (şartname, birinci kaynak):
#  · Şekil 3 notu (s.20): "parkurlarda kullanılan duba sayıları ve KENAR
#    DUBALARI ARASINDAKİ MESAFELER yarışma alanına göre değişkenlik gösterecektir"
#  · s.23: "Dubalar arasındaki mesafeler, duba sayıları, parkur uzunluğu yarışma
#    alanına göre belirlenecektir. DUBA SAYILARINA GÖRE BİR AKIŞ TASARLANMAMASI
#    tavsiye edilmektedir."
# ⇒ metre cinsinden sabit eşik (eski GECIT_MAX_DZ=4 m, GECIT_MAX_GEN=10 m) TAHMİNDİR.
# Yerine: ölçek-bağımsız geometri + fiziksel geçilebilirlik.

def test_yan_yana_cift_KAPI_sayilir():
    """Kapı = kursa DİK duran çift → |Δileri| < |Δyanal|."""
    # a=(ileri 6.0, yanal -1.5), b=(ileri 6.2, yanal +1.5): Δileri 0.2 < Δyanal 3.0
    assert gm.yan_yana_mi(6.0, -1.5, 6.2, 1.5) is True


def test_ardisik_kapilarin_dubalari_KAPI_DEGIL():
    """Kurs boyunca dizilen iki duba (biri 6 m, biri 12 m önde) kapı değildir —
    eski 4 m'lik sabit eşik bunu 'kapı' sayabiliyordu."""
    assert gm.yan_yana_mi(6.0, 1.4, 12.0, 1.6) is False


def test_yan_yana_olcek_bagimsiz():
    """Kapı 2 m de olsa 40 m de olsa aynı ölçüt — sabit metre eşiği YOK."""
    assert gm.yan_yana_mi(5.0, -1.0, 5.1, 1.0) is True      # dar kapı
    assert gm.yan_yana_mi(5.0, -20.0, 8.0, 20.0) is True    # çok geniş kapı


def test_45_derece_ayrim_noktasi():
    """Ayrım noktası tam 45° — ayarlanan eşik değil, geometrik sınır."""
    assert gm.yan_yana_mi(0.0, 0.0, 3.0, 3.0) is False      # |Δi| == |Δy| → kapı değil
    assert gm.yan_yana_mi(0.0, 0.0, 2.9, 3.0) is True


def test_gecilebilirlik_govdeye_gore():
    """Alt sınır fizik: dubalar MERKEZDEN ölçülür, serbest açıklık = mesafe − çap.
    Gövde 0,78 m + duba çapı 0,30 m ⇒ merkez-merkez ≥ 1,08 m olmalı."""
    assert gm.gecilebilir_mi(1.20, hull_en=0.78, duba_cap=0.30) is True
    assert gm.gecilebilir_mi(1.00, hull_en=0.78, duba_cap=0.30) is False


def test_cok_dar_cift_muhtemelen_TEK_dubadir():
    """0,4 m'lik 'çift' fiziksel kapı olamaz — büyük olasılıkla tek duba iki
    tespite bölünmüştür; kapı sayılmamalı."""
    assert gm.gecilebilir_mi(0.40, hull_en=0.78, duba_cap=0.30) is False


def test_genislik_UST_siniri_YOK():
    """Kapı ne kadar geniş olursa olsun geçilebilir — üst sınır koymak tahmindir
    (mesafeler alana göre değişiyor, şartname)."""
    assert gm.gecilebilir_mi(25.0, hull_en=0.78, duba_cap=0.30) is True


# ------------------- B6: yeniden başlama hakkı (md 5.5.3.1) sayacı sıfırlar
def test_parkurdan_beklemeye_donunce_SIFIRLANIR():
    """Takımın 1 kereye mahsus yeniden başlama hakkı var ve PUANLAR SIFIRLANIR.
    Geçilen geçit hafızası temizlenmezse yeni turda aynı geçitler 'zaten
    geçildi' sanılır ve hiç sayılmaz."""
    assert gm.sifirlama_gerekir("PARKUR2", "BEKLEMEDE") is True
    assert gm.sifirlama_gerekir("PARKUR1", "ARM") is True
    assert gm.sifirlama_gerekir("PARKUR3", "BOOT") is True


def test_normal_parkur_ilerlemesi_SIFIRLAMAZ():
    assert gm.sifirlama_gerekir("PARKUR1", "PARKUR2") is False
    assert gm.sifirlama_gerekir("PARKUR2", "PARKUR3") is False


def test_gorev_bitisi_ve_kill_sifirlamaz():
    """TAMAMLANDI/KILL yeniden başlama değildir — sayaç korunur (rapor/log)."""
    assert gm.sifirlama_gerekir("PARKUR2", "TAMAMLANDI") is False
    assert gm.sifirlama_gerekir("PARKUR2", "KILL") is False


def test_ilk_durum_bilgisi_yokken_sifirlamaz():
    assert gm.sifirlama_gerekir(None, "BEKLEMEDE") is False


# ------------------------------- M2: geçide yönlendirecek hedef noktası
def test_gecit_hedefi_ortanin_OTESINDE():
    """Hedef geçit ortasında olursa tekne geçidin İÇİNDE durur; ötesine
    konur ki komple geçsin (kıç dahil)."""
    hx, hy = gm.gecit_hedefi(ox=5.0, oy=0.0, px=1.0, py=0.0, oteleme=2.0)
    assert (hx, hy) == pytest.approx((7.0, 0.0))


def test_gecit_hedefi_egik_gecitte_dike_gider():
    """Geçit çizgisine DİK yönde ötelenir (çapraz girip dubaya sürtmesin)."""
    s = math.sqrt(0.5)
    hx, hy = gm.gecit_hedefi(ox=0.0, oy=0.0, px=s, py=s, oteleme=2.0)
    assert (hx, hy) == pytest.approx((2 * s, 2 * s))


# --------------------------------------------------------- P1/P2 eşikleri
def test_p1_orantisal_sart():
    """Parkur-1 tamamlama: (G1/KD1)×10 ≥ 5 → G1/KD1 ≥ 0,5 (md 5.5.2.3)."""
    assert gm.p1_tamam(g=3, kd=5) is True      # 0.6
    assert gm.p1_tamam(g=2, kd=5) is False     # 0.4
    assert gm.p1_tamam(g=1, kd=2) is True      # 0.5 tam sınır


def test_p1_kd_bilinmiyorsa_karar_verilemez():
    """KD saha verisi; bilinmiyorsa 'tamamlandı' İDDİA EDİLEMEZ."""
    assert gm.p1_tamam(g=5, kd=0) is False
    assert gm.p1_tamam(g=5, kd=None) is False


def test_p2_mutlak_sart():
    """Parkur-2: en az 2 duba ikilisi (md 5.5.2.4)."""
    assert gm.p2_gecit_sarti(1) is False
    assert gm.p2_gecit_sarti(2) is True
