"""P3 çıkış ölçütleri — ROS gerekmez."""
from __future__ import annotations

from prototype.mission.p3_cikis import P3CikisIzleyici


def test_P3e_girilmeden_hicbir_sey_tetiklemez() -> None:
    iz = P3CikisIzleyici()
    for t in range(0, 300, 1):
        assert iz.guncelle(float(t), 0.0) == (False, False)


def test_temas_SUREKLI_durgunluk_ister() -> None:
    """Sayaç P3'e GİRİŞTEN değil, teknenin fiilen DURDUĞU andan başlar —
    yoksa P3'e yavaş girildiğinde temas etmeden 'temas' denirdi."""
    iz = P3CikisIzleyici(durma_suresi_s=3.0)
    iz.p3ye_girildi(0.0)
    assert iz.guncelle(1.0, 0.0)[0] is False        # durgunluk t=1'de BAŞLADI
    assert iz.guncelle(3.9, 0.0)[0] is False        # 2,9 sn durgun
    assert iz.guncelle(4.0, 0.0)[0] is True         # 3,0 sn doldu


def test_DALGA_temas_sanilmaz() -> None:
    """🔑 Dalgada hız SALINIR, sıfırda takılı kalmaz. Tek hızlı örnek sayacı
    sıfırlar — biriktirme değil KESİNTİSİZ durgunluk aranır."""
    iz = P3CikisIzleyici(durma_suresi_s=3.0)
    iz.p3ye_girildi(0.0)
    t = 0.0
    for _ in range(20):                              # 20 sn boyunca salınım
        t += 1.0
        assert iz.guncelle(t, 0.0)[0] is False       # durgun
        t += 1.0
        assert iz.guncelle(t, 0.5)[0] is False       # hareket → sayaç sıfır


def test_yaklasma_hizi_temas_sanilmaz() -> None:
    """Ölçülen temas hızı 0,134-0,154 m/s; eşik onun ALTINDA olmalı."""
    iz = P3CikisIzleyici()
    iz.p3ye_girildi(0.0)
    for t in range(1, 30):
        assert iz.guncelle(float(t), 0.134)[0] is False


def test_sure_asimi() -> None:
    iz = P3CikisIzleyici(azami_sure_s=120.0)
    iz.p3ye_girildi(10.0)
    assert iz.guncelle(129.0, 1.0)[1] is False
    assert iz.guncelle(130.0, 1.0)[1] is True


def test_sifirla_sayaclari_temizler() -> None:
    iz = P3CikisIzleyici(durma_suresi_s=1.0)
    iz.p3ye_girildi(0.0)
    iz.guncelle(5.0, 0.0)
    iz.sifirla()
    assert iz.p3te_mi is False
    assert iz.guncelle(100.0, 0.0) == (False, False)


def test_negatif_hiz_de_hareket_sayilir() -> None:
    """Geri giderken 'ilerlemiyor' denmemeli — mutlak değer alınıyor."""
    iz = P3CikisIzleyici(durma_suresi_s=1.0)
    iz.p3ye_girildi(0.0)
    for t in range(1, 10):
        assert iz.guncelle(float(t), -0.5)[0] is False


def test_BAYAT_ODOM_sahte_temas_uretmez() -> None:
    """🔴 13.08 av turu: odom susarsa hız son değerinde DONAR. Tekne o sırada
    duruyorsa "ilerleme yok" sahte tetiklenir ve görev, temas olmadığı hâlde
    TAMAMLANDI'ya düşer. Ölçemediğimiz şeyde çelişki iddia etmiyoruz.
    """
    iz = P3CikisIzleyici(durma_suresi_s=2.0)
    iz.p3ye_girildi(0.0)
    for t in range(1, 30):
        ilerleme_yok, _ = iz.guncelle(float(t), 0.0, hiz_gecerli=False)
        assert ilerleme_yok is False, "bayat odomla temas iddia edildi"


def test_bayat_odom_SURE_ASIMINI_engellemez() -> None:
    """Süre saatten gelir — odom ölse de tekne sonsuza kadar sürüklenmemeli."""
    iz = P3CikisIzleyici(azami_sure_s=10.0)
    iz.p3ye_girildi(0.0)
    assert iz.guncelle(11.0, 0.0, hiz_gecerli=False)[1] is True


def test_odom_tazelenince_sayac_BASTAN_baslar() -> None:
    iz = P3CikisIzleyici(durma_suresi_s=2.0)
    iz.p3ye_girildi(0.0)
    iz.guncelle(1.0, 0.0, hiz_gecerli=True)          # durgunluk başladı
    iz.guncelle(2.0, 0.0, hiz_gecerli=False)         # odom öldü → sıfırla
    assert iz.guncelle(3.0, 0.0, hiz_gecerli=True)[0] is False   # yeniden say
    assert iz.guncelle(5.0, 0.0, hiz_gecerli=True)[0] is True
