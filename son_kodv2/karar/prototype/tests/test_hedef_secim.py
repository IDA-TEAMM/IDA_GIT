"""Parkur-3 hedef seçimi — ROS gerekmez.

TS3 (şartname s.25): yanlış hedefe temas **100→50**, iki yanlış **100→5**.
Kurallar bu yüzden "emin değilsek dokunma" tarafına eğimli.
"""
from __future__ import annotations

from prototype.mission.hedef_secim import Hedef, cap_makul_mu, hedef_sec

ARAC = (0.0, 0.0)


def test_istenen_renkteki_EN_YAKIN_secilir():
    h = [Hedef(10, 0, 1, 0.64), Hedef(4, 0, 1, 0.64), Hedef(2, 0, 2, 0.64)]
    assert hedef_sec(h, 1, ARAC).x == 4


def test_ISTENEN_RENK_YOKSA_secim_YOK():
    """🔴 Hakem rengi vermemişse hedefe kendi kendimize karar vermeyiz."""
    h = [Hedef(4, 0, 1, 0.64)]
    assert hedef_sec(h, None, ARAC) is None
    assert hedef_sec(h, 0, ARAC) is None


def test_RENGI_COZULEMEYEN_hedef_ESLESMEZ():
    """Konumu bilinen ama rengi bilinmeyen cisme nişan almak TS3 riskidir.
    (Algı yine de yayınlar — varlık bilgisi başka amaçlarla değerli.)"""
    assert hedef_sec([Hedef(4, 0, 0, 0.64)], 1, ARAC) is None


def test_YANLIS_RENK_secilmez():
    assert hedef_sec([Hedef(2, 0, 2, 0.64), Hedef(9, 0, 3, 0.64)], 1, ARAC) is None


def test_cap_bandi_disi_elenir():
    """0,30 m'lik kenar dubası yanlışlıkla targets'a düşerse seçilmemeli."""
    assert hedef_sec([Hedef(4, 0, 1, 0.30)], 1, ARAC) is None
    assert hedef_sec([Hedef(4, 0, 1, 2.50)], 1, ARAC) is None
    assert hedef_sec([Hedef(4, 0, 1, 0.64)], 1, ARAC) is not None


def test_cap_OLCULEMEDIYSE_kor_eleme_YOK():
    """Ölçemediğimiz şeyde iddia etmiyoruz (algıdaki `buyuk_cisim_mi` ile aynı kural)."""
    assert cap_makul_mu(0.0) is True
    assert hedef_sec([Hedef(4, 0, 1, 0.0)], 1, ARAC) is not None


def test_bos_liste():
    assert hedef_sec([], 1, ARAC) is None


def test_arac_konumuna_gore_en_yakin():
    """'En yakın' aracın konumuna göre — origin'e göre değil."""
    h = [Hedef(0, 0, 1, 0.64), Hedef(20, 0, 1, 0.64)]
    assert hedef_sec(h, 1, (19.0, 0.0)).x == 20


def test_kod_0_KORUMASI_esitlikten_geliyor():
    """🔎 13.08 mutasyon turu: `renk_kodu != 0` diye AYRI bir kontrol
    eklemiştim; kaldırınca hiçbir test düşmedi ⇒ **ulaşılamaz kod**.
    Koruma iki yerden geliyor ve ikisi de burada donduruluyor:
      · `istenen_renk_kodu` 0/None ise fonksiyon hiç seçim yapmaz
      · değilse eşitlik (`renk_kodu == istenen`) 0'ı zaten tutmaz
    """
    kod0 = [Hedef(4, 0, 0, 0.64)]
    assert hedef_sec(kod0, 0, ARAC) is None          # istenen 0 → seçim yok
    assert hedef_sec(kod0, None, ARAC) is None
    for istenen in (1, 2, 3):                        # eşitlik 0'ı tutmuyor
        assert hedef_sec(kod0, istenen, ARAC) is None
