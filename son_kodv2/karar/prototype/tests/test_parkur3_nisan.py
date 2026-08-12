"""FAZ 3 — Parkur-3 nişanı (`_parkur3_nisani`), ROS'suz.

🔑 **Bu dosyanın asıl işi P1/P2'yi korumak.** Nişan yolu dört kapının
arkasında; herhangi biri kapalıysa fonksiyon `None` döner ve çağıran
**bugünkü davranışına** (kapı takibi / ham görev noktası) düşer.

TS3 (şartname s.25): yanlış hedefe temas **100→50**, iki yanlış **100→5**.
"""
from __future__ import annotations

import pytest

from prototype.mission.hedef_secim import Hedef, nisan_hedefi


def _nisan(durum="PARKUR3", renk=1, hedefler=None, yas=0.0, poz=(0.0, 0.0)):
    """Nişan kapıları — node tarafı bunun ince sarmalayıcısıdır."""
    if hedefler is None:
        hedefler = [Hedef(10, 0, 1, 0.64)]
    return nisan_hedefi(durum, renk, hedefler, poz, yas)


# ─────────────── P1/P2 KORUMASI (en kritik) ───────────────
@pytest.mark.parametrize("durum", ["PARKUR1", "PARKUR2", "BEKLEMEDE",
                                   "TAMAMLANDI", "KILL", ""])
def test_P3_DISINDA_nisan_DEGISMEZ(durum):
    """🔴 Hedef görünür olsa bile P1/P2'de nişana dokunulmaz."""
    assert _nisan(durum=durum) is None


def test_RENK_YUKLU_DEGILSE_nisan_DEGISMEZ():
    """Hakem rengi vermemişse hedefe kendi kendimize karar vermeyiz."""
    assert _nisan(renk=0) is None


# ─────────────── P3 nişanı ───────────────
def test_PARKUR3te_hedefe_nisan_alinir():
    h = _nisan(hedefler=[Hedef(12.0, -3.0, 1, 0.64)])
    assert (h.x, h.y) == (12.0, -3.0)


def test_EN_YAKIN_hedef_secilir():
    h = _nisan(hedefler=[Hedef(30, 0, 1, 0.64), Hedef(8, 0, 1, 0.64)])
    assert h.x == 8


def test_YANLIS_RENK_nisan_DEGISTIRMEZ():
    assert _nisan(renk=3, hedefler=[Hedef(8, 0, 1, 0.64)]) is None


def test_RENGI_COZULEMEYEN_hedefe_nisan_ALINMAZ():
    """Kod 0 = renk çözülemedi; konumu bilinen ama rengi bilinmeyen cisme
    nişan almak yanlış hedefe temas riskidir."""
    assert _nisan(hedefler=[Hedef(8, 0, 0, 0.64)]) is None


# ─────────────── tazelik ───────────────
def test_BAYAT_tespit_kullanilmaz():
    """🔴 Algı sussa (node öldü, kamera koptu) bayat konuma nişan almak,
    olmayan bir şeye sürmektir."""
    assert _nisan(yas=5.0) is None
    assert _nisan(yas=0.5) is not None           # 1,5 sn içinde → geçerli


def test_hedef_listesi_BOSSA_nisan_DEGISMEZ():
    assert _nisan(hedefler=[]) is None


def test_POZ_YOKSA_nisan_DEGISMEZ():
    """Poz bilinmeden gövde→dünya dönüşümü anlamsız."""
    assert _nisan(poz=None) is None


def test_CAP_bandi_disi_hedef_secilmez():
    """0,30 m'lik duba yanlışlıkla targets'a düşerse nişan çekmemeli."""
    assert _nisan(hedefler=[Hedef(8, 0, 1, 0.30)]) is None
