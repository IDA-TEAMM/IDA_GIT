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


# ═══════════════ HEDEF KİLİDİ — saldırı ortasında nişan düşmesin ═══════════
from prototype.mission.hedef_secim import HedefKilidi  # noqa: E402


def test_TEK_KARELIK_tespite_KILITLENMEZ():
    """🔴 Yanlış hedefe kilitlenmek TS3 demek (100→50→5). Onay şart."""
    k = HedefKilidi(onay_kare=3)
    assert k.guncelle(Hedef(10, 0, 1, 0.64)) is None      # 1. kare
    assert k.guncelle(Hedef(10, 0, 1, 0.64)) is None      # 2. kare
    assert k.guncelle(Hedef(10, 0, 1, 0.64)) is not None  # 3. kare → KİLİT


def test_ARDISIKLIK_bozulursa_sayac_SIFIRLANIR():
    k = HedefKilidi(onay_kare=3)
    k.guncelle(Hedef(10, 0, 1, 0.64))
    k.guncelle(Hedef(10, 0, 1, 0.64))
    assert k.guncelle(None) is None                       # tespit kesildi
    assert k.onay_sayaci == 0
    assert k.guncelle(Hedef(10, 0, 1, 0.64)) is None      # baştan sayıyor


def test_KILITLIYKEN_tespit_kesilse_de_NISAN_KORUNUR():
    """🔴🔴 BU DOSYANIN ASIL SEBEBİ.

    Ölçüldü (13.08): hedef 0,7 m'ye kadar kadrajda kalıyor ama **0,3 m altında
    stereo ölüyor** ⇒ son yarım metrede tespit **kesinlikle** kesilir. Kilit
    olmasa `nisan_hedefi` None döner, çağıran **ham görev noktasına** (Parkur-2'nin
    sonu = arkamız) düşer ve tekne **tam temas anında hedeften dönerdi**.
    """
    k = HedefKilidi(onay_kare=2)
    k.guncelle(Hedef(8, 0, 1, 0.64))
    kilit = k.guncelle(Hedef(8, 0, 1, 0.64))
    assert kilit is not None
    for _ in range(50):                                   # 25 sn tespit YOK
        h = k.guncelle(None)
        assert h is not None and (h.x, h.y) == (8, 0), "nişan düştü!"


def test_YAKIN_tespit_konumu_TAZELER():
    """Yaklaşırken hedefin ölçülen konumu güncellenmeli (odom sürüklenmesine karşı)."""
    k = HedefKilidi(onay_kare=1, tazeleme_m=3.0)
    k.guncelle(Hedef(10, 0, 1, 0.64))
    h = k.guncelle(Hedef(9.2, 0.3, 1, 0.64))              # 0,85 m ötede = aynı hedef
    assert (h.x, h.y) == (9.2, 0.3)


def test_UZAKTAKI_baska_hedefe_ATLAMAZ():
    """🔴 Hedefler yan yana duruyor (Şekil 3). Saldırı ortasında hedef
    değiştirmek İKİ hedefe birden temas riskidir (TS3=2 ⇒ 100→**5**)."""
    k = HedefKilidi(onay_kare=1, tazeleme_m=3.0)
    k.guncelle(Hedef(10, 0, 1, 0.64))
    h = k.guncelle(Hedef(10, 8, 1, 0.64))                 # 8 m ötede = BAŞKA hedef
    assert (h.x, h.y) == (10, 0), "başka hedefe atladı"


def test_sifirla_kilidi_BIRAKIR():
    k = HedefKilidi(onay_kare=1)
    k.guncelle(Hedef(10, 0, 1, 0.64))
    assert k.kilitli is not None
    k.sifirla()
    assert k.kilitli is None and k.guncelle(None) is None
