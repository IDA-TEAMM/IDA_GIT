"""PARKUR-3 SAF OpenCV hattı — `/perception/targets`'ın YOLO'dan bağımsız ayağı.

🔴 NEDEN VAR: 16.08'de yazılan OpenCV hedef tespiti (`p3_hedef/hedef_bul.py`)
kütüphane olarak duruyordu ama **hiçbir node onu çağırmıyordu** (arama: 0
sonuç). Hedef adayları hâlâ YOLO kutularından türüyordu — oysa modelimiz iki
sınıflı ve **P3 hedefini görmemeyi öğrendi** (eğitimde etiketsiz P3
negatifleri). YOLO hedefi kutulamazsa `/perception/targets` boş kalır,
`hedef_sec` seçim yapamaz ⇒ **P3 = 0**, hiçbir hata basılmadan.

Bu dosya o hattın **iki yönünü de** donduruyor: PARKUR3 dışında hiç koşmaması
ve PARKUR3'te gerçekten aday üretmesi.

Sahte `self` ile ROS yayıncısı olmadan koşar; **gerçek metotlar** çağrılır.
"""
import os
import sys
import types

import numpy as np
import pytest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

pytest.importorskip("cv2", reason="OpenCV kurulu değil")
pytest.importorskip("depthai", reason="depthai kurulu değil")
pytest.importorskip("rclpy", reason="rclpy kurulu değil")
pytest.importorskip("vision_msgs", reason="vision_msgs kurulu değil")

from girdap_ida_algi import duba_gecis_navigator as dgn  # noqa: E402
from girdap_ida_algi import gecit_mantik as gm  # noqa: E402
from girdap_ida_algi import p3_hedef_bul as p3  # noqa: E402

SU = (150, 140, 120)          # BGR — mavi-gri su


def _kare(renk_bgr=None, merkez=(256, 320), yaricap=26, boyut=512):
    """Deploy karesinin taklidi: 512×512 BGR (passthrough boyutu)."""
    im = np.zeros((boyut, boyut, 3), np.uint8)
    im[:, :] = SU
    if renk_bgr is not None:
        import cv2
        cv2.circle(im, merkez, yaricap, renk_bgr, -1)
    return im


SAHTE_SIMDI_NS = 1_800_000_000 * 10**9      # sabit, gercekci bir ROS ani


def _stamp(ns_toplam):
    from builtin_interfaces.msg import Time as _T
    t = _T()
    t.sec = int(ns_toplam // 10**9)
    t.nanosec = int(ns_toplam % 10**9)
    return t


def _sahte_saat():
    """`_damga` hem `.to_msg()` hem `.nanoseconds` ister (RclTime aritmetigi).

    Sabit bir an dondurur ki damga testleri deterministik olsun.
    """
    return types.SimpleNamespace(
        now=lambda: types.SimpleNamespace(
            nanoseconds=SAHTE_SIMDI_NS,
            to_msg=lambda: _stamp(SAHTE_SIMDI_NS)))


def _ns(kare, durum="PARKUR3", yayinlanan=(), hedef_adaylari=(),
        kare_yasi_s=None):
    """`hedef_adimi` + `_p3_opencv_adaylari` için sahte self."""
    kutu = {}

    class _Pub:
        def publish(self, msg):
            kutu["targets"] = msg

    ns = types.SimpleNamespace(
        _son_kare=kare, _kare_no=1, _hedef_kare_no=-1, _son_hedef_t=-1e9,
        _f_norm=gm.odak_px(1.0), _hedef_adaylari=list(hedef_adaylari),
        _yayinlanan_kutular=list(yayinlanan), son_gorev_durumu=durum,
        _tani={"hedef_yayin": 0, "p3_opencv": 0, "p3_veto": 0,
               "p3_su_hatti": 0},
        targets_pub=_Pub(),
        get_logger=lambda: types.SimpleNamespace(
            warn=lambda *a, **k: None, info=lambda *a, **k: None),
        get_clock=lambda: _sahte_saat(),
    )
    ns._tani["damga_yedek"] = 0
    ns._kare_yasi_s = kare_yasi_s
    ns._mesaj_yasi = dgn.DubaNavigator._mesaj_yasi
    for ad in ("_p3_opencv_adaylari", "_hedef_rengi_coz", "hedef_adimi",
               "_damga"):
        setattr(ns, ad, types.MethodType(getattr(dgn.DubaNavigator, ad), ns))
    return ns, kutu


# ---------------------------------------------------------------- KAPI ----
def test_PARKUR3_DISINDA_hic_kosmaz():
    """CPU bedava değil ve koşmayan kod yanlış pozitif üretemez."""
    ns, _ = _ns(_kare((60, 200, 60)), durum="PARKUR2")
    assert ns._p3_opencv_adaylari(ns._son_kare) == []


def test_gorev_durumu_YOKKEN_kosmaz():
    """FSM susuyorsa kör kabul yok — yanlış temas TS3'te 100 → 50 puan."""
    ns, _ = _ns(_kare((60, 200, 60)), durum=None)
    assert ns._p3_opencv_adaylari(ns._son_kare) == []


def test_ANA_SALTER_kapaliyken_targets_HIC_yayinlanmaz(monkeypatch):
    """`GIRDAP_P3_HEDEF=0` iken OpenCV yolu da dahil hiçbir şey koşmaz."""
    monkeypatch.setattr(dgn, "P3_HEDEF_YAYINI", False)
    ns, kutu = _ns(_kare((60, 200, 60)))
    ns.hedef_adimi(0.0)
    assert "targets" not in kutu, "şalter kapalıyken yayın olmamalı"
    assert ns._tani["p3_opencv"] == 0, "şalter kapalıyken OpenCV koşmamalı"


# ------------------------------------------------------------- ÜRETİM ----
@pytest.fixture
def p3_acik(monkeypatch):
    monkeypatch.setattr(dgn, "P3_HEDEF_YAYINI", True)


def test_YESIL_hedef_bulunur_ve_yayinlanir(p3_acik):
    ns, kutu = _ns(_kare((60, 200, 60)))          # BGR yeşil
    ns.hedef_adimi(0.0)
    arr = kutu["targets"]
    assert arr.detections, "yeşil hedef bulunamadı"
    d = arr.detections[0]
    assert d.results[0].hypothesis.class_id == str(gm.HEDEF_RENK_KODU["yesil"])
    assert ns._tani["p3_opencv"] >= 1


def test_KIRMIZI_hedef_de_bulunur(p3_acik):
    """Kırmızı mono YOLO yolundan GEÇEMEZ (`MONO_HEDEF_RENKLERI`); OpenCV
    yolu onu bulabilen tek üretici — yoksa hakem 'kırmızı' derse P3 = 0."""
    ns, kutu = _ns(_kare((40, 40, 220)))
    ns.hedef_adimi(0.0)
    kodlar = [d.results[0].hypothesis.class_id for d in kutu["targets"].detections]
    assert str(gm.HEDEF_RENK_KODU["kirmizi"]) in kodlar


def test_cap_SIFIR_yayinlanir_sahte_olcum_degil(p3_acik):
    """Menzil Ø0,64 VARSAYARAK kuruluyor ⇒ ondan türetilen çap dairesel olur.
    0,0 = 'ölçemedim'; tüketici (`cap_makul_mu`) bunu kör elemiyor."""
    ns, kutu = _ns(_kare((60, 200, 60)))
    ns.hedef_adimi(0.0)
    det = kutu["targets"].detections[0]
    assert det.bbox.size.x == 0.0 and det.bbox.size.y == 0.0
    assert det.results[0].hypothesis.score == 0.0, \
        "saf OpenCV'nin model güveni yoktur — doluluk 'skor' diye yazılmamalı"


def test_konum_ileri_x_sol_y_sozlesmesi(p3_acik):
    """base_link: x=ileri (+kamera ofseti), y=sol. Sağdaki hedef y<0 olmalı."""
    ns, kutu = _ns(_kare((60, 200, 60), merkez=(400, 320)))   # sağda
    ns.hedef_adimi(0.0)
    det = kutu["targets"].detections[0]
    assert det.bbox.center.position.x > dgn.KAMERA_OFSET_ILERI
    assert det.bbox.center.position.y < 0.0, "sağdaki hedef sol(+y) tarafta çıktı"


# --------------------------------------------------------------- VETO ----
def test_YOLO_vetosu_kendi_dubamizi_eler(p3_acik):
    """Yayınlanmış duba kutusuyla örtüşen aday elenir (ölçüm: kırmızı yanlış
    adayların %95'i buraya düşüyor)."""
    kare = _kare((40, 40, 220), merkez=(256, 320))
    vetosuz, _ = _ns(kare)
    n_vetosuz = len(vetosuz._p3_opencv_adaylari(kare))
    assert n_vetosuz >= 1, "kontrol grubu: vetosuz aday üretilmeliydi"
    # aynı yerde YAYINLANMIŞ bir duba varsa aday elenmeli
    vetolu, _ = _ns(kare, yayinlanan=[(256 / 512, 320 / 512, 0.12, 0.12)])
    assert vetolu._p3_opencv_adaylari(kare) == []
    assert vetolu._tani["p3_veto"] >= 1


def test_veto_listesi_HEDEF_ADAYLARINI_icermez(p3_acik):
    """🔑 Veto girdisi `/perception/buoys`'a GERÇEKTEN yayınlananlar.

    Hedef adayına ayrılan tespitler veto listesine girerse (kırmızı hedef
    mono'da tam bu yola düşüyor) OpenCV kendi bulduğu gerçek hedefi eler ve
    P3'ün gözü kapanır. Bu testi kıran değişiklik, sahada 'hedef hiç
    görünmüyor' olarak çıkar."""
    kaynak = open(os.path.join(
        os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
        "girdap_ida_algi", "duba_gecis_navigator.py"), encoding="utf-8").read()
    assert "self._yayinlanan_kutular = yayinlanan" in kaynak
    i = kaynak.index("yayinlanan.append(")
    # `yayinlanan.append` çağrısı, iki `continue`dan (hedef adayı ayırma)
    # SONRA gelmeli — yani Detection2D bloğunun içinde.
    assert kaynak.index("det = Detection2D()") < i, \
        "yayınlanan kutu listesi, hedef adayları ayrılmadan ÖNCE dolduruluyor"


# --------------------------------------------------------- ÇİFT YAYIN ----
def test_ayni_cisim_iki_kez_yayinlanmaz(p3_acik):
    """YOLO yolundan gelen adayla örtüşen OpenCV adayı tekrar basılmaz."""
    kare = _kare((60, 200, 60), merkez=(256, 320))
    yolo_aday = dgn.Duba(cls=0, x=0.0, z=8.0, conf=0.8,
                         cx=256 / 512, cy=320 / 512, w=0.10, h=0.10)
    ns, kutu = _ns(kare, hedef_adaylari=[yolo_aday])
    ns.hedef_adimi(0.0)
    assert len(kutu["targets"].detections) == 1, \
        "aynı cisim hem YOLO hem OpenCV yolundan iki kez yayınlandı"


# ------------------------------------------------------- SÖZLEŞMELER ----
def test_renk_kodu_tablosu_algi_ile_AYNI():
    """🔴 16.08'de arkadaşın gönderici/alıcısında bu tablo TERSTİ (1=siyah)
    ve İHA 'siyah' derken İDA kırmızıya saldıracaktı. İki tablo tek satırda
    donduruluyor."""
    for renk, kod in p3.KOD.items():
        assert gm.HEDEF_RENK_KODU[renk] == kod, f"{renk}: {kod} ↔ tablo ayrışmış"
    assert gm.HEDEF_RENK_KODU[None] == 0, "0 = renk çözülemedi olmalı"


def test_kopya_kanonik_kaynakla_AYNI():
    """Kopya ↔ kanonik `girdap-ida-p3/p3_hedef/hedef_bul.py` ayrışmamalı.

    Kanonik repo bu makinede yoksa test SKIP eder — sessizce yeşil VERMEZ.
    """
    import hashlib
    import re
    kopya_yolu = os.path.join(
        os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
        "girdap_ida_algi", "p3_hedef_bul.py")
    kopya = open(kopya_yolu, encoding="utf-8").read()
    m = re.search(r"kaynak sha256\s*:\s*([0-9a-f]{64})", kopya)
    assert m, "kopyada kanonik sha256 kaydı yok"
    kanonik = os.path.expanduser("~/girdap-ida-p3/p3_hedef/hedef_bul.py")
    if not os.path.exists(kanonik):
        pytest.skip(f"kanonik kaynak bu makinede yok: {kanonik}")
    gercek = hashlib.sha256(open(kanonik, "rb").read()).hexdigest()
    assert gercek == m.group(1), (
        "kanonik `hedef_bul.py` DEĞİŞMİŞ, kopya eski kalmış.\n"
        "Yapılacak: dosyayı yeniden kopyala + başlıktaki sha256'yı güncelle.\n"
        f"  kopyadaki: {m.group(1)[:16]}…\n  kanonikte : {gercek[:16]}…")


# =========================================================== SU HATTI ====
# 🔑 Yüzen duba SU ÜZERİNDEDİR: bbox alt kenarından (su değme noktası)
# hesaplanan menzil ile bbox genişliğinden hesaplanan menzil uyuşmalı.
# Kıyı bitkisinin su yansıması bu sınavda çöküyor.
# ÖLÇÜM (240 gerçek kare, iki popülasyon da GERÇEK): K=3'te yanlış adayların
# %90'ı elenir (yeşil 44/49 · siyah 2/2), 599 gerçek duba kutusunda kayıp %0.
# Ufuk (0,5015) ve eğim (0,2228) 327 gerçek duba kutusundan uydurma ile çıktı.

U = dgn.SU_HATTI_UFUK_CY
E = dgn.SU_HATTI_EGIM
PAY = dgn.P3_PITCH_PAYI_CY
K = dgn.P3_CELISKI_KATI


def _cy_alt(mesafe_m, ufuk=None):
    """Verilen mesafedeki YÜZEN cismin su değme noktasının normalize cy'si."""
    return (U if ufuk is None else ufuk) + E / mesafe_m


def test_su_hatti_YANSIMAYI_eler():
    """Teknenin ~0,5 m yanındaki suda duran leke '8 m'deki hedef' diyemez."""
    assert gm.su_hatti_celiskili(0.98, 8.0, U, E, PAY, K)


def test_su_hatti_GERCEK_hedefi_ELEMEZ():
    """Gerçekten 10 m'de yüzen hedef: iki menzil uyuşur, elenmez."""
    assert not gm.su_hatti_celiskili(_cy_alt(10.0), 10.0, U, E, PAY, K)


def test_yakin_hedef_de_ELENMEZ():
    """3 m'deki hedef: su hattı da genişlik de 3 m diyor."""
    assert not gm.su_hatti_celiskili(_cy_alt(3.0), 3.0, U, E, PAY, K)


def test_PITCH_PAYI_dalgada_hedefi_KURTARIR():
    """🔴 MUTASYON KAPISI: pay olmadan, ufku 0,06 kaydıran bir dalga/trim
    10 m'deki GERÇEK hedefi eletiyor. Pay bunu kurtarıyor. 09.08'de ufuk
    süzgeci tam bu belirsizlik yüzünden reddedilmişti."""
    cy = _cy_alt(10.0, ufuk=U + 0.06)          # dalga: ufuk kaydı
    assert gm.su_hatti_celiskili(cy, 10.0, U, E, 0.0, K), \
        "kontrol grubu: pay olmadan bu hedef elenmeliydi"
    assert not gm.su_hatti_celiskili(cy, 10.0, U, E, PAY, K), \
        "pitch payı dalgadaki gerçek hedefi kurtarmıyor"


def test_ufka_yakin_cisimde_CELISKI_IDDIA_EDILMEZ():
    """Uzak hedef ufka yakındır; orada geometri bilgi vermez ⇒ eleme YOK."""
    assert gm.su_hatti_menzili(U + 0.01, U, E, PAY) is None


def test_su_hatti_kapisi_TEK_YONLU():
    """Su hattına göre daha UZAK görünen aday elenmez (dalga/kırpık bbox)."""
    assert not gm.su_hatti_celiskili(_cy_alt(4.0), 1.0, U, E, PAY, K)


def test_yansima_node_yolunda_da_elenir_ve_sayilir(p3_acik):
    """Uçtan uca: karenin altındaki geniş yeşil leke yayınlanmamalı."""
    import cv2
    im = _kare()
    cv2.ellipse(im, (256, 470), (44, 30), 0, 0, 360, (60, 200, 60), -1)
    ns, kutu = _ns(im)
    ns.hedef_adimi(0.0)
    assert ns._tani["p3_su_hatti"] >= 1, "su hattı kapısı hiç tetiklenmedi"
    assert not kutu["targets"].detections, "yansıma yayınlandı"
