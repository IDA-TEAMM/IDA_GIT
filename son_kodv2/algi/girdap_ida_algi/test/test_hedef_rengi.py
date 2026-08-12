"""Parkur-3 hedef rengi — SAF katman (ROS/kamera gerekmez).

Şartname s.18: hedef renkleri RAL 3026 (kırmızı) · RAL 6037 (yeşil) ·
RAL 9005 (siyah). Yanlış renk = yanlış hedefe angajman = TS3 (s.25:
1 yanlış temas 100→50, 2 yanlış 100→**5**).

🔑 Buraya yalnız `buyuk_cisim_mi`den geçmiş tespitler gelir — yani "bu cisim
0,64 m'lik bir hedef" sorusu ZATEN cevaplanmıştır; burada sorulan tek şey
HANGİ RENK olduğu.
"""
import os
import sys

import numpy as np
import pytest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from girdap_ida_algi import gecit_mantik as gm  # noqa: E402

cv2 = pytest.importorskip("cv2", reason="opencv yok")

RAL = {"kirmizi": (2, 235, 215), "yesil": (72, 205, 150), "siyah": (0, 20, 38)}


def _yama_bgr(h, s, v, w=24, y=35, golge=True, parlama=True, gurultu=8.0, seed=0):
    """Silindir hedefin bbox'ı: kenarlara doğru koyulaşan gövde + parlama."""
    import math
    rng = np.random.default_rng(seed)
    im = np.zeros((y, w, 3), np.uint8)
    for c in range(w):
        k = (0.55 + 0.45 * math.sin(math.pi * (c + 0.5) / w)) if golge else 1.0
        im[:, c] = cv2.cvtColor(
            np.uint8([[[h, s, max(8, int(v * k))]]]), cv2.COLOR_HSV2BGR)[0, 0]
    if parlama:
        im[:, int(w * 0.62):max(1, int(w * 0.72))] = cv2.cvtColor(
            np.uint8([[[h, max(0, s - 90), min(255, v + 55)]]]),
            cv2.COLOR_HSV2BGR)[0, 0]
    im = np.clip(im.astype(int) + rng.normal(0, gurultu, im.shape), 0, 255)
    return im.astype(np.uint8)


def _coz(bgr):
    """Üretimde çağrılan yol: BGR kırpım → (gürültü bastırma + HSV) → renk."""
    return gm.hedef_rengi_bgr(bgr)[0]


@pytest.mark.parametrize("ad", list(RAL))
def test_uc_RAL_rengi_de_cozuluyor(ad):
    assert _coz(_yama_bgr(*RAL[ad])) == ad


@pytest.mark.parametrize("ad", list(RAL))
@pytest.mark.parametrize("w,y", [(48, 71), (24, 35), (16, 24), (10, 14)])
def test_UZAK_hedefte_de_cozuluyor(ad, w, y):
    """10×14 px = 25 m. Ölçüldü (13.08): bu boyutta bile %99-100."""
    assert _coz(_yama_bgr(*RAL[ad], w=w, y=y)) == ad


def test_SU_hedef_sanilmaz():
    """Gerçek su/gökyüzü tonları hiçbir hedef rengine uymamalı."""
    for h, s, v in [(100, 90, 140), (105, 60, 180), (95, 120, 90), (0, 8, 200)]:
        assert _coz(_yama_bgr(h, s, v, golge=False, parlama=False)) is None


def test_TURUNCU_ve_SARI_dubalarimiz_hedef_sanilmaz():
    """🔴 Kendi kenar (RAL 2003 turuncu) ve engel (RAL 1026 sarı) dubalarımız
    hedef rengi ÜRETMEMELİ — üretirse P3'te yanlış hedefe angajman olur."""
    assert _coz(_yama_bgr(11, 230, 220)) is None      # turuncu kenar dubası
    assert _coz(_yama_bgr(28, 230, 230)) is None      # sarı engel dubası


def test_GOLGELI_SU_siyah_sanilmaz():
    """🔴 13.08 İHA dersi: gölge parlaklığı düşürür ama RENGİ KORUR
    (gölgeli zemin S≈110-125 ↔ RAL 9005 S≈23-32). Doygunluk tavanı olmasa
    'karanlık olan her şey siyah' olurdu."""
    assert _coz(_yama_bgr(100, 120, 40, golge=False, parlama=False)) is None
    assert _coz(_yama_bgr(42, 121, 38, golge=False, parlama=False)) is None


def test_renk_cozulemezse_None_doner_HATA_DEGIL():
    """Kırpık/boş bbox → None; çağıran yine de konumu yayınlar."""
    assert gm.hedef_rengi_bgr(np.zeros((0, 0, 3), np.uint8))[0] is None
    assert gm.hedef_rengi_bgr(np.zeros((0, 5, 3), np.uint8))[0] is None
    # 2×2 (blur'un çalışamayacağı kadar küçük) ÇÖKMEMELİ — sonuç ne olursa olsun
    gm.hedef_rengi_bgr(np.full((2, 2, 3), 90, np.uint8))


def test_GURULTU_BASTIRMA_siyahi_kurtariyor():
    """🔴 13.08 ÖLÇÜMÜ: S=(max−min)/max düşük parlaklıkta kararsız. RAL 9005
    (gerçek S≈20) σ=8 gürültüde **S≈89** okunuyor ⇒ doygunluk tavanı siyahı
    **eliyordu** (kapsama %23 ↔ eşik %25 = sınırın hemen altı, sessiz ve
    rastgele). 3×3 Gauss sonrası kapsama %71-82."""
    bgr = _yama_bgr(*RAL["siyah"], w=10, y=14)
    assert gm.hedef_rengi_bgr(bgr)[0] == "siyah"
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)          # blur'suz ham yol
    assert gm.hedef_rengi(hsv[:, :, 0], hsv[:, :, 1], hsv[:, :, 2])[0] is None


def test_renk_kodu_sozlesmesi_KARAR_TARAFIYLA_AYNI():
    """🔴 Aynı tablo ÜÇ repoda: algı · karar (`renk_kodu.py KOD_RENK`) ·
    İHA (`cikis.py RENK_KODU`). Ayrışırsa yanlış hedefe angajman olur ve
    **hiç belirti vermez**."""
    assert gm.HEDEF_RENK_KODU == {None: 0, "kirmizi": 1, "yesil": 2, "siyah": 3}
