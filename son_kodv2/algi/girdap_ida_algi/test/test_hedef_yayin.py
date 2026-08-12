"""FAZ 2 — `/perception/targets` yayını (Parkur-3 hedef dubası).

🔑 **Bu dosyanın asıl işi P1/P2'yi KORUMAK.** Hedef tespitleri
`/perception/buoys`'a **asla** karışmamalı: karışırsa füzyon
`EdgeBuoyMemory`'ye kalıcı kenar kaydı açar → iki hedef arasında **hayalet
kapı** → tekne kapı sanıp aralarından geçer, P2 rotası bozulur (11.08).

Ve Dosya-1 (md 4.2, eksik dosya **5 ceza puanı**) ile P3 birbirini
düşürmemeli — burada iki yönlü de sınanıyor.

Sahte `self` ile ROS'suz koşar; **gerçek fonksiyonlar** çağrılır.
"""
import os
import sys
import types

import numpy as np
import pytest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

pytest.importorskip("depthai", reason="depthai kurulu değil")
pytest.importorskip("rclpy", reason="rclpy kurulu değil")
pytest.importorskip("vision_msgs", reason="vision_msgs kurulu değil")
cv2 = pytest.importorskip("cv2", reason="opencv yok")

from girdap_ida_algi import duba_gecis_navigator as dgn  # noqa: E402
from girdap_ida_algi import gecit_mantik as gm  # noqa: E402

RAL = {"kirmizi": (2, 235, 215), "yesil": (72, 205, 150), "siyah": (0, 20, 38)}


def _duba(cls, x, z, cx, cy, cap_m, h=0.10, conf=0.9):
    """Verilen GERÇEK çapa göre tutarlı bbox genişliği üretir."""
    w = cap_m * gm.odak_px(1.0) / z
    return dgn.Duba(cls=cls, x=x, z=z, conf=conf, cx=cx, cy=cy, w=w, h=h)


class _Pub:
    def __init__(self, kutu, ad):
        self.kutu, self.ad = kutu, ad

    def publish(self, msg):
        self.kutu[self.ad] = msg


def _ns(dubalar, kare=None):
    kutu = {}
    ns = types.SimpleNamespace(
        dubalar=dubalar, _lb_pay=0.0, sinif_esleme={0: "0", 1: "1"},
        buoys_pub=_Pub(kutu, "2d"), buoys3d_pub=_Pub(kutu, "3d"),
        targets_pub=_Pub(kutu, "hedef"),
        _f_norm=gm.odak_px(1.0), _hedef_adaylari=[],
        _tani={"buyuk_cisim": 0, "hedef_yayin": 0},
        _son_kare=kare, _kare_no=1, _hedef_kare_no=-1, _son_hedef_t=-99.0,
        _kayit_bozuk=False,
        get_logger=lambda: types.SimpleNamespace(
            warn=lambda *a, **k: None, info=lambda *a, **k: None,
            error=lambda *a, **k: None),
        get_clock=lambda: types.SimpleNamespace(
            now=lambda: types.SimpleNamespace(
                to_msg=lambda: dgn.Detection2DArray().header.stamp)),
    )
    # `hedef_adimi` içinden çağrılan yardımcıyı sahte self'e BAĞLA — yoksa
    # AttributeError kendi try'ında yutulur ve test sessizce "yayın yok" görür.
    ns._hedef_rengi_coz = lambda kare, d: dgn.DubaNavigator._hedef_rengi_coz(
        ns, kare, d)
    ns._kutu = kutu
    return ns


def _kare_ile_hedef(renk_hsv, cap_m=0.64, z=6.0, boyut=512):
    """Ortasında verilen renkte hedef olan sahte kare + o tespiti üret."""
    kare = np.zeros((boyut, boyut, 3), np.uint8)
    kare[:, :] = cv2.cvtColor(np.uint8([[[100, 90, 140]]]),
                              cv2.COLOR_HSV2BGR)[0, 0]          # su
    d = _duba(0, 0.0, z, cx=0.5, cy=0.5, cap_m=cap_m)
    w_px = int(d.w * boyut); h_px = int(d.h * boyut)
    x1, y1 = boyut // 2 - w_px // 2, boyut // 2 - h_px // 2
    kare[y1:y1 + h_px, x1:x1 + w_px] = cv2.cvtColor(
        np.uint8([[list(renk_hsv)]]), cv2.COLOR_HSV2BGR)[0, 0]
    return kare, d


# ───────────────── P1/P2 KORUMASI (en kritik) ─────────────────
def test_HEDEF_buoys_a_KARISMIYOR():
    """🔴 Sözleşme değişmedi: 0,64 m'lik cisim `/perception/buoys`'a girmez."""
    ns = _ns([_duba(0, 0.0, 6.0, 0.5, 0.5, cap_m=0.64)])
    dgn.DubaNavigator.tespit_yayinla(ns)
    assert len(ns._kutu["2d"].detections) == 0, "hedef buoys'a sızdı!"
    assert len(ns._kutu["3d"].poses) == 0
    assert len(ns._hedef_adaylari) == 1, "hedef adayı toplanmadı"


def test_NORMAL_duba_buoys_ta_KALIR():
    """0,30 m'lik gerçek duba etkilenmemeli — P1/P2 aynen çalışır."""
    ns = _ns([_duba(0, 0.0, 6.0, 0.5, 0.5, cap_m=0.30)])
    dgn.DubaNavigator.tespit_yayinla(ns)
    assert len(ns._kutu["2d"].detections) == 1
    assert ns._hedef_adaylari == []


def test_aday_listesi_HER_KAREDE_sifirlanir():
    """Temizlenmezse tekne çoktan geçtiği hedefe nişan almaya devam eder."""
    ns = _ns([_duba(0, 0.0, 6.0, 0.5, 0.5, cap_m=0.64)])
    dgn.DubaNavigator.tespit_yayinla(ns)
    dgn.DubaNavigator.tespit_yayinla(ns)
    assert len(ns._hedef_adaylari) == 1, "adaylar birikiyor (bayat hedef)"


# ───────────────── /perception/targets içeriği ─────────────────
@pytest.mark.parametrize("ad", list(RAL))
def test_hedef_RENK_KODUYLA_yayinlaniyor(ad):
    kare, d = _kare_ile_hedef(RAL[ad])
    ns = _ns([d], kare=kare)
    dgn.DubaNavigator.tespit_yayinla(ns)
    dgn.DubaNavigator.hedef_adimi(ns, 100.0)
    arr = ns._kutu["hedef"]
    assert len(arr.detections) == 1
    assert arr.detections[0].results[0].hypothesis.class_id == \
        str(gm.HEDEF_RENK_KODU[ad])
    assert arr.header.frame_id == dgn.BASE_FRAME     # perception = GÖVDE


def test_renk_cozulemezse_KOD_0_ama_KONUM_yayinlanir():
    """Renk bilinmese de hedefin VARLIĞI değerli — bilgi atılmaz."""
    d = _duba(0, 1.5, 6.0, 0.5, 0.5, cap_m=0.64)
    ns = _ns([d], kare=None)                          # kare yok → renk yok
    dgn.DubaNavigator.tespit_yayinla(ns)
    dgn.DubaNavigator.hedef_adimi(ns, 100.0)
    det = ns._kutu["hedef"].detections[0]
    assert det.results[0].hypothesis.class_id == "0"
    assert det.bbox.center.position.x == pytest.approx(6.0 + dgn.KAMERA_OFSET_ILERI)
    assert det.bbox.center.position.y == pytest.approx(-1.5)


def test_hedef_yokken_BOS_yayinlanir():
    """'Hedef yok' ile 'node ölmüş' ayırt edilebilmeli."""
    ns = _ns([_duba(0, 0.0, 6.0, 0.5, 0.5, cap_m=0.30)])
    dgn.DubaNavigator.tespit_yayinla(ns)
    dgn.DubaNavigator.hedef_adimi(ns, 100.0)
    assert "hedef" in ns._kutu and len(ns._kutu["hedef"].detections) == 0


# ───────────────── Dosya-1 ile KARŞILIKLI BAĞIMSIZLIK ─────────────────
def test_P3_HATASI_Dosya1i_OLDURMEZ():
    """🔴🔴 Kaydın `except`'i `_kayit_bozuk=True` yapıp Dosya-1'i KALICI
    kapatıyor (5 ceza puanı). P3 kodu o bloğun DIŞINDA olmalı."""
    d = _duba(0, 0.0, 6.0, 0.5, 0.5, cap_m=0.64)
    ns = _ns([d], kare="bu bir numpy dizisi DEĞİL")   # renk çözümü patlayacak
    dgn.DubaNavigator.tespit_yayinla(ns)
    dgn.DubaNavigator.hedef_adimi(ns, 100.0)          # çökmemeli
    assert ns._kayit_bozuk is False, "P3 hatası Dosya-1'i devre dışı bıraktı!"


def test_KAYIT_BOZUKKEN_hedef_yayini_SURER():
    """Dosya-1 ölürse Parkur-3 onunla birlikte ölmemeli."""
    kare, d = _kare_ile_hedef(RAL["kirmizi"])
    ns = _ns([d], kare=kare)
    ns._kayit_bozuk = True
    dgn.DubaNavigator.tespit_yayinla(ns)
    dgn.DubaNavigator.hedef_adimi(ns, 100.0)
    assert len(ns._kutu["hedef"].detections) == 1


def test_HEDEF_HZ_kisitlaniyor():
    """Aynı anda iki kez çağrılırsa ikincisi atlanmalı (kare zaten 2 Hz)."""
    kare, d = _kare_ile_hedef(RAL["kirmizi"])
    ns = _ns([d], kare=kare)
    dgn.DubaNavigator.tespit_yayinla(ns)
    dgn.DubaNavigator.hedef_adimi(ns, 100.0)
    ns._kutu.pop("hedef")
    dgn.DubaNavigator.hedef_adimi(ns, 100.0 + 0.1)    # periyot dolmadı
    assert "hedef" not in ns._kutu


# ───────────── KOPYA ŞARTI: kayıt, hedefin karesini BOYAMAMALI ─────────────
def test_KAYIT_saklanan_kareyi_BOYAMIYOR():
    """🔴🔴 `kayit_adimi` görüntünün ÜSTÜNE çiziyor (bbox dikdörtgenleri +
    yeşil zaman etiketi). numpy dizisi REFERANSTIR — kopyalanmazsa `_son_kare`
    de boyanır ve Parkur-3 renk analizi **kendi çizdiğimiz turuncu/sarı
    çerçeveyi** okur. Ölçüldü (13.08): kopya 0,05 ms (Jetson ~0,25 ms) =
    500 ms kayıt bütçesinin %0,05'i — güvenli olanı seçmek bedavaya yakın.
    """
    kare, d = _kare_ile_hedef(RAL["kirmizi"])
    onceki = kare.copy()
    yazilan = {}

    ns = types.SimpleNamespace(
        _son_kare=kare, _kare_no=1, _kayit_kare_no=-1, _kayit_bozuk=False,
        dubalar=[d], kenar_cls=0, _siniflar=["kenar_dubasi", "engel_dubasi"],
        durum="ARAMA", gecit_sayisi=0, olculen_fps=8.0,
        _kayit_yaz=lambda img, t: yazilan.update(img=img),
        get_logger=lambda: types.SimpleNamespace(
            warn=lambda *a, **k: None, info=lambda *a, **k: None,
            error=lambda *a, **k: None),
    )
    dgn.DubaNavigator.kayit_adimi(ns)

    assert ns._kayit_bozuk is False, "kayıt hata verdi"
    assert "img" in yazilan, "kayıt hiç yazmadı"
    # Kaydedilen karede çizim OLMALI, saklanan karede OLMAMALI
    assert not np.array_equal(yazilan["img"], onceki), "kayıt hiç çizmemiş?"
    assert np.array_equal(ns._son_kare, onceki), \
        "🔴 kayıt saklanan kareyi boyadı — P3 renk analizi bozulur"


def test_ayni_kare_IKI_KEZ_kaydedilmez():
    """Eskiden 'kuyruk boşsa atla' davranışı vardı; kare numarası onu korur."""
    kare, d = _kare_ile_hedef(RAL["kirmizi"])
    sayac = {"n": 0}
    ns = types.SimpleNamespace(
        _son_kare=kare, _kare_no=1, _kayit_kare_no=-1, _kayit_bozuk=False,
        dubalar=[], kenar_cls=0, _siniflar=["a", "b"],
        durum="ARAMA", gecit_sayisi=0, olculen_fps=8.0,
        _kayit_yaz=lambda img, t: sayac.update(n=sayac["n"] + 1),
        get_logger=lambda: types.SimpleNamespace(
            warn=lambda *a, **k: None, info=lambda *a, **k: None,
            error=lambda *a, **k: None),
    )
    dgn.DubaNavigator.kayit_adimi(ns)
    dgn.DubaNavigator.kayit_adimi(ns)          # yeni kare gelmedi
    assert sayac["n"] == 1
