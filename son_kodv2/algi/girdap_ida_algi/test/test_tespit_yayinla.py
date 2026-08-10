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


def _duba(cls, x, z, cx, cy, w, h, conf=0.9):
    return dgn.Duba(cls=cls, x=x, z=z, conf=conf, cx=cx, cy=cy, w=w, h=h)


def _yayinla(dubalar, kenar_cls=0, engel_cls=1, lb_pay=0.0):
    """`tespit_yayinla`'yı sahte yayıncılarla koştur, yayınlanan mesajları döndür."""
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
        get_clock=lambda: types.SimpleNamespace(
            now=lambda: types.SimpleNamespace(
                to_msg=lambda: dgn.Detection2DArray().header.stamp)),
    )
    dgn.DubaNavigator.tespit_yayinla(ns)
    return kutu["2d"], kutu["3d"]


# --------------------------------------------------------------------------
# 🔴 E-1 REGRESYONU: bbox 1280×720 uzayında yayınlanmalı (fusion böyle bekliyor)
def test_bbox_1280x720_uzayinda_yayinlanir():
    arr, _ = _yayinla([_duba(0, 0.0, 5.0, cx=0.5, cy=0.5, w=0.1, h=0.1)])
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
