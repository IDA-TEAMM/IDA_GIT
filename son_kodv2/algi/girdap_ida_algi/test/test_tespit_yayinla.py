"""`tespit_yayinla()` — KAPSAMIN MERKEZİ: `/perception/buoys` sözleşmesi.

🔴 NEDEN: bu, karar tarafının GERÇEKTEN tükettiği tek çıktımız (10.08 ölçümü:
`/perception/buoys` abone **3**; `gate_count`/`gate_target`/`buoys_3d` abone **0**).
Ve buradaki bir sayı hatası daha önce **P1+P2'yi sessizce sıfırlamıştı** (E-1:
bbox 640×480 uzayında yayınlanıyordu, füzyon 1280×720 varsayıyordu ⇒ karenin
sağ %75'indeki her duba `bearing_tolerance_rad`'ı aşıyordu, hiçbir hata basılmadan).
Testi YOKTU — 10.08 derin taramasında bulundu.

Sahte `self` ile ROS yayıncısı olmadan koşar; **gerçek fonksiyon** çağrılır.
"""
import os
import sys
import types

import pytest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

pytest.importorskip("depthai", reason="depthai kurulu değil")
pytest.importorskip("rclpy", reason="rclpy kurulu değil")
pytest.importorskip("vision_msgs", reason="vision_msgs kurulu değil")

from girdap_ida_algi import duba_gecis_navigator as dgn  # noqa: E402
from girdap_ida_algi import gecit_mantik as gm  # noqa: E402


def _duba(cls, x, z, cx, cy, w=None, h=0.07, conf=0.9):
    """w verilmezse 0,30 m'lik duba için stereo ile TUTARLI genişlik üretilir —
    yoksa büyük cisim süzgeci (11.08) testleri kazara eler."""
    if w is None:
        w = gm.DUBA_CAP_M * gm.odak_px(1.0) / z
    return dgn.Duba(cls=cls, x=x, z=z, conf=conf, cx=cx, cy=cy, w=w, h=h)


@pytest.fixture
def p3_acik(monkeypatch):
    """P3 ANA ŞALTERİNİ aç — mono hedef yolu (OpenCV/HSV) yalnız o zaman koşar.

    🔴 16.08 akşamı: şalter (`P3_HEDEF_YAYINI`) artık `_mono_hedef_mi`'yi de
    kapatıyor. Eyüp: *"P3'ü şimdilik kapatalım, OpenCV'ye geçmesin."* Yani
    P1/P2 ölçüm koşusunda kare başına `cv2.cvtColor`/HSV analizi HİÇ koşmaz.
    Aşağıdaki testler o yolun **açıkken doğru çalıştığını** sınar; kapalıyken
    hiç koşmadığını `test_p3_kapali.py` + `test_mono_hedef_yolu_SALTERE_BAGLI`
    sınar. İkisi birlikte şalterin iki yönünü de donduruyor.
    """
    monkeypatch.setattr(dgn, "P3_HEDEF_YAYINI", True)


def _yayinla(dubalar, kenar_cls=0, engel_cls=1, lb_pay=0.0, kare=None):
    """`tespit_yayinla`'yı sahte yayıncılarla koştur, yayınlanan mesajları döndür.

    `kare`: mono yolunun renk kapısı için sahte görüntü (None = kare yok ⇒
    kapı kapalı kalır, kör kabul YOK). `_mono_hedef_mi` GERÇEK metottur —
    sahteye bağlanır ki yeni yol da gerçekten sınansın.
    """
    kutu = {}

    class _Pub:
        def __init__(self, ad):
            self.ad = ad

        def publish(self, msg):
            kutu[self.ad] = msg

    ns = types.SimpleNamespace(
        dubalar=dubalar, _lb_pay=lb_pay,
        sinif_esleme={kenar_cls: "0", engel_cls: "1"},
        buoys_pub=_Pub("2d"), buoys3d_pub=_Pub("3d"),
        _f_norm=gm.odak_px(1.0),
        _son_kare=kare,
        _tani={"buyuk_cisim": 0, "mono_hedef": 0},
        get_logger=lambda: types.SimpleNamespace(
            warn=lambda *a, **k: None, info=lambda *a, **k: None,
            error=lambda *a, **k: None),
        get_clock=lambda: types.SimpleNamespace(
            now=lambda: types.SimpleNamespace(
                to_msg=lambda: dgn.Detection2DArray().header.stamp)),
    )
    ns._mono_hedef_mi = types.MethodType(
        dgn.DubaNavigator._mono_hedef_mi, ns)      # gerçek metot, sahte self
    dgn.DubaNavigator.tespit_yayinla(ns)
    _yayinla.son_ns = ns          # sahte self'i sonraki denetim için sakla
    return kutu["2d"], kutu["3d"]


# --------------------------------------------------------------------------
# 🔴 E-1 REGRESYONU: bbox 1280×720 uzayında yayınlanmalı (fusion böyle bekliyor)
def test_bbox_1280x720_uzayinda_yayinlanir():
    # w verilmiyor → 0,30 m dubayla tutarlı genişlik (büyük cisim süzgeci elemesin)
    arr, _ = _yayinla([_duba(0, 0.0, 5.0, cx=0.5, cy=0.5)])
    d = arr.detections[0]
    assert d.bbox.center.position.x == pytest.approx(dgn.BBOX_W / 2, abs=1.0)
    assert d.bbox.center.position.y == pytest.approx(dgn.BBOX_H / 2, abs=1.0)
    assert (dgn.BBOX_W, dgn.BBOX_H) == (1280, 720), \
        "fusion camera_image_width/height_px ile AYNI olmalı (E-1)"


def test_sag_kenardaki_duba_sag_kenarda_yayinlanir():
    """E-1'in tam kırdığı yer: sağ kenar 640'ta kalırsa bearing 17° sapıyordu."""
    arr, _ = _yayinla([_duba(0, 3.0, 5.0, cx=0.95, cy=0.5, w=0.05, h=0.05)])
    x = arr.detections[0].bbox.center.position.x
    assert x > dgn.BBOX_W * 0.9, f"sağ kenar {x:.0f} px — 1280 uzayında olmalı"


# 🔴 SINIF SÖZLEŞMESİ: class_id STRING "0"/"1" (fusion int(hyp.class_id) yapıyor)
def test_class_id_string_ve_dogru_eslesir():
    arr, _ = _yayinla([_duba(0, -1.0, 5.0, 0.3, 0.5, 0.05, 0.07),
                       _duba(1, +1.0, 5.0, 0.7, 0.5, 0.05, 0.07)])
    idler = [d.results[0].hypothesis.class_id for d in arr.detections]
    assert idler == ["0", "1"]
    assert all(isinstance(i, str) for i in idler)


def test_sinif_indeksleri_ters_cozulduyse_esleme_yine_dogru():
    """Model sınıf sırası ters gelirse (`kenar_cls=1`) yayınlanan id yine
    kenar='0' olmalı — yoksa turuncu↔sarı sessizce takas olur (Ç2)."""
    arr, _ = _yayinla([_duba(1, 0.0, 5.0, 0.5, 0.5, 0.05, 0.07)],
                      kenar_cls=1, engel_cls=0)
    assert arr.detections[0].results[0].hypothesis.class_id == "0"


# 3B çıkış: kamera ofseti uygulanmış, y işareti ters (sağ + → sol +)
def test_3d_poz_kamera_ofseti_ve_isaret():
    _, arr3d = _yayinla([_duba(0, 2.0, 5.0, 0.5, 0.5, 0.05, 0.07)])
    p = arr3d.poses[0]
    assert p.position.x == pytest.approx(5.0 + dgn.KAMERA_OFSET_ILERI)
    assert p.position.y == pytest.approx(-2.0)
    assert p.orientation.z == pytest.approx(dgn.DUBA_CAP / 2.0), "z = yarıçap (obstacle_map)"


# BOŞ LİSTE de yayınlanmalı — fusion'ın zaman senkronu buna bağlı
def test_bos_liste_de_yayinlanir():
    arr, arr3d = _yayinla([])
    assert arr.detections == [] and arr3d.poses == []


def test_frame_id_sozlesmesi():
    arr, arr3d = _yayinla([_duba(0, 0.0, 5.0, 0.5, 0.5, 0.05, 0.07)])
    assert arr.header.frame_id == dgn.KAMERA_FRAME
    assert arr3d.header.frame_id == dgn.BASE_FRAME


def test_score_conf_ile_ayni():
    arr, _ = _yayinla([_duba(0, 0.0, 5.0, 0.5, 0.5, 0.05, 0.07, conf=0.73)])
    assert arr.detections[0].results[0].hypothesis.score == pytest.approx(0.73)


# ==========================================================================
# `_blob_denetle()` — yanlış model AÇILIŞTA yakalanmalı (10.08 derin taraması)
# Blob `/home/girdap/models/` altına ELLE kopyalanıyor ⇒ yanlış/eski blob
# konması gerçek bir dağıtım hatası; sonucu SESSİZ çöp tespit (sınıf karışması
# ⇒ yanlış kapı ⇒ Ç2). Blob başlığı bilgiyi taşıyor, kontrol bedava.
_BLOB = os.path.join(os.path.dirname(os.path.dirname(os.path.dirname(
    os.path.abspath(__file__)))), "models", "yolo11n_duba_rvc2.blob")


@pytest.mark.skipif(not os.path.exists(_BLOB), reason="blob repoda yok")
def test_dogru_blob_temiz_gecer():
    assert dgn._blob_denetle(_BLOB, ["kenar_dubasi", "engel_dubasi"]) == []


@pytest.mark.skipif(not os.path.exists(_BLOB), reason="blob repoda yok")
def test_sinif_sayisi_uyusmazsa_yakalanir():
    sorun = dgn._blob_denetle(_BLOB, ["a", "b", "c"])
    assert sorun and "DECODE KAYAR" in sorun[0]


def test_olmayan_blob_cokmez_sorun_dondurur():
    """Tanı kodu node'u ÖLDÜRMEMELİ — istisna değil, liste döner (09.08 dersi)."""
    sorun = dgn._blob_denetle("/olmayan/model.blob", ["a", "b"])
    assert sorun and "okunamadı" in sorun[0]


@pytest.mark.skipif(not os.path.exists(_BLOB), reason="blob repoda yok")
def test_giris_boyutu_uyusmazsa_yakalanir(monkeypatch):
    """Elimizde yanlış boyutlu blob yok ⇒ NN_GIRIS'i geçici değiştirip sınarız.
    (10.08 mutasyon testi bu boşluğu yakaladı: kontrol kapatılınca kimse
    fark etmiyordu.)"""
    monkeypatch.setattr(dgn, "NN_GIRIS", 640)
    sorun = dgn._blob_denetle(_BLOB, ["kenar_dubasi", "engel_dubasi"])
    assert sorun and "NN_GIRIS" in sorun[0], sorun


# ══ BÜYÜK CİSİM SÜZGECİ (11.08) — P3 hedefi kenar dubası sanılmasın ═══════
def _tutarli_w(cap, z):
    """`cap` çapındaki cismin `z` metrede kapladığı normalize genişlik."""
    return cap * gm.odak_px(1.0) / z


def test_p3_hedefi_yayindan_suzulur():
    """🔴 Ø0,64 m hedef `/perception/buoys`'a girerse EdgeBuoyMemory KALICI
    kenar kaydı açar (unutma yok) → iki hedef arasında hayalet kapı."""
    arr, arr3d = _yayinla([_duba(0, 0.0, 10.0, 0.5, 0.5, w=_tutarli_w(0.64, 10.0))])
    assert arr.detections == [] and arr3d.poses == []


def test_gercek_duba_suzulmez():
    """Ø0,30 m duba 2 kat payla geçer — P1/P2 etkilenmez."""
    arr, _ = _yayinla([_duba(0, 0.0, 10.0, 0.5, 0.5, w=_tutarli_w(0.30, 10.0))])
    assert len(arr.detections) == 1


@pytest.mark.parametrize("z", [3.0, 6.0, 12.0, 20.0])
def test_suzgec_mesafeden_bagimsiz(z):
    """Oran ölçütü olduğu için her mesafede aynı davranmalı."""
    kucuk, _ = _yayinla([_duba(0, 0.0, z, 0.5, 0.5, w=_tutarli_w(0.30, z))])
    buyuk, _ = _yayinla([_duba(0, 0.0, z, 0.5, 0.5, w=_tutarli_w(0.64, z))])
    assert len(kucuk.detections) == 1 and buyuk.detections == []


def test_mono_menzilde_suzgec_UYGULANMAZ():
    """Menzil bbox'tan geldiyse çapraz kontrol anlamsız (aynı sayıyı kendisiyle
    karşılaştırır) — kör eleme yapmayız (08.08 kuralı)."""
    d = _duba(0, 0.0, 10.0, 0.5, 0.5, w=_tutarli_w(0.64, 10.0))
    d.kaynak = "mono"
    arr, _ = _yayinla([d])
    assert len(arr.detections) == 1


def test_suzulen_tespit_SAYACA_islenir():
    """Sahada SSH yok — sessiz eleme görünmez arıza olur (06.08 dersi)."""
    kutu = {}

    class _Pub:
        def __init__(self, ad): self.ad = ad
        def publish(self, msg): kutu[self.ad] = msg

    tani = {"buyuk_cisim": 0}
    ns = types.SimpleNamespace(
        dubalar=[_duba(0, 0.0, 10.0, 0.5, 0.5, w=_tutarli_w(0.64, 10.0))],
        _lb_pay=0.0, sinif_esleme={0: "0", 1: "1"},
        buoys_pub=_Pub("2d"), buoys3d_pub=_Pub("3d"),
        _f_norm=gm.odak_px(1.0), _tani=tani,
        get_logger=lambda: types.SimpleNamespace(
            warn=lambda *a, **k: None, info=lambda *a, **k: None,
            error=lambda *a, **k: None),
        get_clock=lambda: types.SimpleNamespace(
            now=lambda: types.SimpleNamespace(
                to_msg=lambda: dgn.Detection2DArray().header.stamp)))
    dgn.DubaNavigator.tespit_yayinla(ns)
    assert tani["buyuk_cisim"] == 1


# ══ SIGTERM: kapanışta Dosya-1 kapanmalı (11.08) ══════════════════════════
def test_sigterm_isleyicisi_kuruluyor():
    """🔴 Python'un SIGTERM varsayılanı süreci ANINDA öldürür, `finally`
    ÇALIŞMAZ; rclpy de SIGTERM'e dokunmaz. systemd stop/reboot/güç kesme
    SIGTERM yollar ⇒ mp4'ün moov atomu yazılmaz ⇒ Dosya-1 OYNATILAMAZ =
    **5 ceza puanı** (md 5.5.4.3.5). ÖLÇÜLDÜ ve düzeltildi."""
    import signal
    onceki = signal.getsignal(signal.SIGTERM)
    try:
        dgn._sigterm_kur()
        h = signal.getsignal(signal.SIGTERM)
        assert h not in (signal.SIG_DFL, signal.SIG_IGN, None), \
            "SIGTERM varsayılanda kalırsa finally çalışmaz, mp4 bozulur"
        assert callable(h)
    finally:
        signal.signal(signal.SIGTERM, onceki)


# ── STEREO YOKKEN P3 HEDEFİ (16.08.2026) ──────────────────────────────────
# Ölçüm: 6.042 gerçek kenar/engel kutusunda 0,45 kapsama eşiğiyle yeşil
# %0,000 · siyah %0,000 yanlış pozitif; kırmızı 0,65'te bile %0,53.
def _kare_renkli(hsv, boyut=(240, 320)):
    """Tamamı tek HSV rengi olan sahte kare (bbox nereye düşerse düşsün dolu)."""
    cv2 = pytest.importorskip("cv2")
    np = pytest.importorskip("numpy")
    im = np.zeros((boyut[0], boyut[1], 3), np.uint8)
    im[:, :] = hsv
    return cv2.cvtColor(im, cv2.COLOR_HSV2BGR)


_YESIL = (73, 200, 150)      # h 62-85 · s>=80 · v>=50
_SIYAH = (0, 20, 30)         # s<=70 · v<=60
_KIRMIZI = (3, 220, 200)     # h 0-7 · s>=130 · v>=90
_TURUNCU = (12, 200, 200)    # bizim kenar dubamız — hiçbir hedef eşiğine girmemeli


def _mono(z=14.0):
    d = _duba(0, 0.0, z, 0.5, 0.5, w=_tutarli_w(0.64, z))
    d.kaynak = "mono"
    return d


@pytest.mark.parametrize("hsv,ad", [(_YESIL, "yesil"), (_SIYAH, "siyah")])
def test_mono_hedef_rengi_SUZULUR(hsv, ad, p3_acik):
    """Stereo yokken yeşil/siyah = P3 hedefi ⇒ /perception/buoys'a GİRMEZ.

    Girerse EdgeBuoyMemory'de KALICI kenar kaydı açar (unutma yok) ⇒ hayalet
    kapı + MPPI hedeften kaçar.
    """
    arr, _ = _yayinla([_mono()], kare=_kare_renkli(hsv))
    assert arr.detections == [], f"{ad} hedef yayınlandı — hayalet engel olur"


@pytest.mark.parametrize("hsv,ad", [(_KIRMIZI, "kirmizi"), (_TURUNCU, "turuncu")])
def test_mono_KIRMIZI_bu_yoldan_gecmez(hsv, ad):
    """Kırmızı hiçbir eşikte temizlenmiyor (RAL 3026 ↔ RAL 2003 komşu) ⇒
    kırmızı hedef stereo İSTER; turuncu zaten bizim kenar dubamız."""
    arr, _ = _yayinla([_mono()], kare=_kare_renkli(hsv))
    assert len(arr.detections) == 1, f"{ad} süzüldü — gerçek duba kaybolur"


def test_mono_hedef_yolu_SALTERE_BAGLI():
    """🔴 P3 KAPALIYKEN mono/OpenCV yolu HİÇ koşmaz (16.08 akşamı, Eyüp).

    Şalterin ikinci yönü. Yukarıdaki testler `p3_acik` fixture'ıyla yolun
    AÇIKKEN doğru çalıştığını sınıyor; bu test KAPALIYKEN hiç çalışmadığını
    sınıyor — fixture'sız, yani üretimdeki varsayılanla.

    Neden önemli: `_mono_hedef_mi` bbox kırpıp `cv2.cvtColor` + HSV kapsama
    hesabı yapıyor. Eskiden şalter yalnız `/perception/targets` YAYININI
    kapatıyordu, bu analiz her mono tespitte yine koşuyordu ⇒ "P3 kapalı"
    denen durumda P3 kodu sıcak yolda kalıyordu (CPU + log gürültüsü).

    ⚖️ Kapalıyken tespit `/perception/buoys`'a NORMAL duba olarak girer —
    bilinçli takas: P1/P2 ölçüm koşusunda suda P3 hedef dubası YOK.
    """
    arr, _ = _yayinla([_mono()], kare=_kare_renkli(_YESIL))
    assert len(arr.detections) == 1, (
        "P3 kapalıyken mono tespit yine süzüldü — şalter mono yolunu "
        "kapatmıyor, OpenCV analizi sıcak yolda"
    )
    assert _yayinla.son_ns._tani["mono_hedef"] == 0, "sayaç işledi = yol koştu"
    assert _yayinla.son_ns._hedef_adaylari == [], "kapalıyken aday üretildi"


def test_mono_hedef_SAYACA_islenir(p3_acik):
    """Sahada SSH yok; sessiz süzme görünmez arıza olur (06.08 dersi)."""
    _yayinla([_mono()], kare=_kare_renkli(_YESIL))
    assert _yayinla.son_ns._tani["mono_hedef"] == 1


def test_mono_hedef_adayina_eklenir(p3_acik):
    """Süzülen tespit ATILMIYOR — /perception/targets'a gidecek listede."""
    _yayinla([_mono()], kare=_kare_renkli(_YESIL))
    assert len(_yayinla.son_ns._hedef_adaylari) == 1


def test_mono_hedefin_MENZILI_064_ile_yeniden_kurulur(p3_acik):
    """Mono yedek Ø0,30 varsayar; hedef kabul edildiyse menzil Ø0,64 ile
    yeniden kurulmalı — yoksa (a) yayınlanan çap hep 0,30 çıkar ve tüketicinin
    `cap_makul_mu` bandı (0,40-1,00) hedefi ELER, (b) menzil 2,13 kat KISA olur.
    """
    z_mono = 5.0
    d = _duba(0, 0.0, z_mono, 0.5, 0.5, w=_tutarli_w(0.30, z_mono))
    d.kaynak = "mono"
    _yayinla([d], kare=_kare_renkli(_YESIL))
    aday = _yayinla.son_ns._hedef_adaylari[0]
    beklenen = gm.mesafe_genislikten(d.w, gm.HEDEF_CAP_M, gm.odak_px(1.0))
    assert abs(aday.z - beklenen) < 1e-6, "menzil hedef capiyla kurulmadi"
    assert aday.z > z_mono * 2.0, "menzil hala 0,30 varsayimindan geliyor"


def test_mono_hedefin_capi_kabul_bandina_dusuyor(p3_acik):
    """Tüketici çapı `w·z/f` ile hesaplıyor; 0,40-1,00 bandına düşmeli."""
    z_mono = 8.0
    d = _duba(0, 0.0, z_mono, 0.5, 0.5, w=_tutarli_w(0.30, z_mono))
    d.kaynak = "mono"
    _yayinla([d], kare=_kare_renkli(_SIYAH))
    aday = _yayinla.son_ns._hedef_adaylari[0]
    cap = aday.w * aday.z / gm.odak_px(1.0)
    assert 0.40 <= cap <= 1.00, f"cap {cap:.2f} bandin disinda -> hedef_sec eler"


def test_stereo_hedefin_menziline_DOKUNULMAZ():
    """Stereo varsa ölçüm gerçektir; hipotezle ezilmemeli."""
    z_stereo = 9.0
    d = _duba(0, 0.0, z_stereo, 0.5, 0.5, w=_tutarli_w(0.64, z_stereo))
    _yayinla([d])                      # kaynak stereo (varsayilan)
    aday = _yayinla.son_ns._hedef_adaylari[0]
    assert aday.z == z_stereo
