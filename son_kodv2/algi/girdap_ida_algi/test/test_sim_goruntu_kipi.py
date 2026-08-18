# -*- coding: utf-8 -*-
"""SİM GÖRÜNTÜ KİPİ — kamera kod yolu gölde gerçekten koşuyor mu?

🔴 DOĞURAN VAKA (18.08.2026): sanal gölde `/oak/rgb/image_raw` **yayınlanıyor
ama abonesi yoktu**; sim kaynağı tespitleri `/perception/obstacle_map`'ten
türetiyordu. Yani kameranın kendi kod yolu — görüntü taşıma, QoS uyumu, bgr8
çözme, CLAHE, doygunluk germesi, kontur, **mono pinhole menzil yedeği** —
gölde HİÇ koşmuyordu.

Üstelik o karede zaten hiç duba yoktu: `sanal_gol._algi()` içinde
`Detection3D` bloğu bir girinti kusuru yüzünden arıza `if`'inin içinde
kalmıştı ⇒ `/gercek/classified_obstacles` 120 mesajda **0 tespit** ⇒ sahte
kamera renk bulamayıp boş su karesi basıyordu. (Karar tarafında düzeltildi.)

Ölçülen (düzeltmelerden sonra, 512×512 kare):
    classified 0 → 1240 tespit · görüntü kipi kenar=3 · buoys 5,12 Hz
"""
from __future__ import annotations

import importlib
import os

import numpy as np
import pytest

cv2 = pytest.importorskip("cv2")

#: `sahte_ham_sensor._SINIF_BGR` ile AYNI renkler — göl bunları basıyor.
_TURUNCU = (0, 140, 255)
_SARI = (0, 220, 255)
_SU = (110, 70, 20)


def _nav(kip: str):
    """Modülü istenen kiple YENİDEN yükle (bayraklar modül düzeyinde okunur)."""
    os.environ["GIRDAP_SIM_KAYNAK"] = kip
    from girdap_ida_algi import duba_gecis_navigator as nav
    return importlib.reload(nav)


def _kare(daireler, W=512, H=512):
    k = np.zeros((H, W, 3), np.uint8)
    k[:, :] = _SU
    for (u, v, r, renk) in daireler:
        cv2.circle(k, (u, v), r, renk, -1)
    rng = np.random.default_rng(7)
    return np.clip(k.astype(np.int16) + rng.normal(0, 6, k.shape),
                   0, 255).astype(np.uint8)


def test_kip_bayraklari_AYRISIYOR():
    """1 = geometrik · 2 = görüntü. Karışırlarsa yanlış yol koşar."""
    n1 = _nav("1")
    assert n1.SIM_KAYNAK and not n1.SIM_GORUNTU
    n2 = _nav("2")
    assert n2.SIM_KAYNAK and n2.SIM_GORUNTU
    n0 = _nav("0")
    assert not n0.SIM_KAYNAK and not n0.SIM_GORUNTU


def test_gercek_renkleri_DOGRU_sinifa_esler():
    nav = _nav("2")
    d = nav._sim_kareden_tespitler(
        _kare([(150, 250, 12, _TURUNCU), (380, 260, 11, _SARI)]))
    siniflar = sorted(t.label for t in d)
    assert siniflar == [nav.KENAR_CLASS, nav.ENGEL_CLASS], f"bulunan: {siniflar}"


def test_bbox_NORMALIZE_ve_konum_dogru():
    nav = _nav("2")
    d = nav._sim_kareden_tespitler(_kare([(128, 256, 14, _TURUNCU)]))
    assert len(d) == 1
    t = d[0]
    assert 0.0 <= t.xmin < t.xmax <= 1.0
    assert 0.0 <= t.ymin < t.ymax <= 1.0
    assert (t.xmin + t.xmax) / 2 == pytest.approx(128 / 512, abs=0.02)


def test_stereo_YOK_mono_yedegi_ZORLANIR():
    """🔑 Bu kipin asıl kazancı: 08.08'de suyun stereo'yu öldürmesi üzerine
    eklenen pinhole yedeği ancak stereo GEÇERSİZ iken koşar."""
    nav = _nav("2")
    d = nav._sim_kareden_tespitler(_kare([(200, 250, 12, _TURUNCU)]))
    assert d and all(t.spatialCoordinates.z == 0.0 for t in d), (
        "stereo dolu geliyor → mono yedek dalı gölde HİÇ sınanmaz")


def test_BOS_su_karesinde_uydurma_tespit_YOK():
    """Yanlış pozitif üretirse göl sahte kapı kurar ve ölçüm anlamsızlaşır."""
    nav = _nav("2")
    assert nav._sim_kareden_tespitler(_kare([])) == []


def test_uretim_ON_ISLEMESI_kullaniliyor():
    """CLAHE + doygunluk germesi (F-P.21) atlanırsa loş sahnede hiç tespit
    olmaz; bu kipin amacı tam o yolu koşturmak."""
    import inspect

    nav = _nav("2")
    src = inspect.getsource(nav._sim_kareden_tespitler)
    assert "_clahe" in src and "_doygunluk_germe" in src, (
        "üretim ön işlemesi baypas edilmiş")


def test_dusuk_isikta_da_bulur_GERME_calisiyor():
    """F-P.21: gerçek donanımda S≈29-83 ölçülmüştü, sabit eşik 120 hiç
    tetiklenmiyordu. Germe olmadan bu test kırmızı yanar."""
    nav = _nav("2")
    kare = _kare([(200, 250, 14, _TURUNCU)])
    hsv = cv2.cvtColor(kare, cv2.COLOR_BGR2HSV)
    hsv[:, :, 1] = (hsv[:, :, 1].astype(np.float32) * 0.28).astype(np.uint8)
    sonuk = cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)
    assert nav._sim_kareden_tespitler(sonuk), "loş sahnede tespit YOK"


def test_encoding_KORUNUYOR_rgb8_de_calisir():
    """Yanlış encoding sessizce ters renk verirse sınıflar karışır."""
    nav = _nav("2")
    bgr = _kare([(200, 250, 13, _TURUNCU)])
    d_bgr = nav._sim_kareden_tespitler(bgr)
    d_rgb = nav._sim_kareden_tespitler(bgr[:, :, ::-1].copy())
    assert len(d_bgr) == 1 and (d_bgr[0].xmax - d_bgr[0].xmin) < 0.1
    # 🔴 Ters kanalda SU turuncuya döner (110,70,20 → 20,70,110, HSV tonu 17)
    # ve kare boyu tek bir "kenar dubası" üretilirdi (genişlik 1,000).
    # `_SIM_MAX_GENISLIK` fizik kapısı onu eler.
    assert not d_rgb, (
        "kanal sırası ters iken tüm su 'duba' sayılıyor — boyut kapısı yok")
