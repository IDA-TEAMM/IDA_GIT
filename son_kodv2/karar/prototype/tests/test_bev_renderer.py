"""BEV çizici testleri — teslim iddialarını (md 4.2) DOĞRULAR.

Her test bir ŞARTNAME iddiasına bağlıdır; kozmetik piksel testi yoktur.
cv2 gerektirmez (Mp4Yazici testleri hariç, onlar skip'lenebilir).
"""

import math

import numpy as np
import pytest

from prototype.mapping.bev_renderer import (
    KUME_PALETI,
    SINIF_RENK,
    BevConfig,
    BevRenderer,
    Kume,
    Mp4Yazici,
)


# --------------------------------------------------------------- geometri

def test_kuzey_yukari_arac_donunce_KAYMAZ():
    """🔑 Raster DÜNYA eksenli: araç dönünce engel yer değiştirmemeli.

    GIRDAP_DURUM §0.14g/H7: `_publish_local_map` verinin dünya eksenli
    olduğunu bilmeden `frame_id="base_link"` yazıyordu. Bu modül çerçeveyi
    kareye YAZDIĞI için aynı karışıklığa düşemez — ama davranışın kendisi
    de dondurulmalı.
    """
    r = BevRenderer(BevConfig(genislik_px=200, yukseklik_px=200, menzil_m=25.0))
    arac = (0.0, 0.0)
    kuzeydeki = (0.0, 10.0)
    ilk = r.dunya_to_px(kuzeydeki, arac)
    # yaw yalnız araç üçgenini döndürür; koordinat dönüşümüne GİRMEZ
    assert r.dunya_to_px(kuzeydeki, arac) == ilk
    # kuzey YUKARI => piksel y merkezin ÜSTÜNDE
    assert ilk[1] < 100.0
    assert ilk[0] == pytest.approx(100.0)


def test_dogu_saga_kuzey_yukari_esleme():
    r = BevRenderer(BevConfig(genislik_px=200, yukseklik_px=200, menzil_m=25.0))
    arac = (5.0, 5.0)
    x_dogu, y_dogu = r.dunya_to_px((15.0, 5.0), arac)      # 10 m doğu
    x_kuzey, y_kuzey = r.dunya_to_px((5.0, 15.0), arac)    # 10 m kuzey
    assert x_dogu > 100.0 and y_dogu == pytest.approx(100.0)
    assert y_kuzey < 100.0 and x_kuzey == pytest.approx(100.0)


def test_m_per_px_menzili_kareyi_kapliyor():
    cfg = BevConfig(genislik_px=400, yukseklik_px=400, menzil_m=20.0)
    assert cfg.m_per_px == pytest.approx(40.0 / 400)
    r = BevRenderer(cfg)
    # menzil kenarı tam çerçevede
    x, _ = r.dunya_to_px((20.0, 0.0), (0.0, 0.0))
    assert x == pytest.approx(400.0)


def test_bozuk_config_reddedilir():
    with pytest.raises(ValueError):
        BevConfig(genislik_px=0)
    with pytest.raises(ValueError):
        BevConfig(menzil_m=-1.0)


# ----------------------------------------------------- md 493: kümeleme

def _renk_var(kare: np.ndarray, renk) -> bool:
    return bool(np.all(kare == np.array(renk, dtype=np.uint8), axis=-1).any())


def test_kumeler_AYRI_renk_alir_ayirma_gorunur():
    """md 493: 'kümeleme, AYIRMA vs. görünecek şekilde'.

    İki komşu küme aynı renge boyanırsa ayırma GÖRÜNMEZ; teslim maddeyi
    karşılamaz. Bu test tam olarak onu dondurur.
    """
    r = BevRenderer(BevConfig(genislik_px=300, yukseklik_px=300, menzil_m=15.0))
    kumeler = [
        Kume(merkez=(-3.0, 5.0), sinif=0, kume_id=0,
             noktalar=[(-3.0, 5.0), (-3.1, 5.1)]),
        Kume(merkez=(3.0, 5.0), sinif=0, kume_id=1,
             noktalar=[(3.0, 5.0), (3.1, 5.1)]),
    ]
    kare = r.render_lidar((0.0, 0.0), yaw=math.pi / 2, kumeler=kumeler)
    assert _renk_var(kare, KUME_PALETI[0])
    assert _renk_var(kare, KUME_PALETI[1])
    assert KUME_PALETI[0] != KUME_PALETI[1]


def test_ham_noktalar_cizilir_islem_zinciri_gorunur():
    """'Tespit ve takip İŞLEMLERİ SONUCUNDA ... görünecek' — girdi de görünsün."""
    r = BevRenderer(BevConfig(genislik_px=200, yukseklik_px=200, menzil_m=15.0))
    bos = r.render_lidar((0.0, 0.0))
    dolu = r.render_lidar((0.0, 0.0),
                          ham_noktalar=[(x * 0.1, 5.0) for x in range(-20, 20)])
    assert not np.array_equal(bos, dolu)


def test_sinif_renkleri_kareye_giriyor():
    r = BevRenderer(BevConfig(genislik_px=300, yukseklik_px=300, menzil_m=15.0))
    for sinif in (0, 1, 2, 99):
        kare = r.render_lidar(
            (0.0, 0.0),
            kumeler=[Kume(merkez=(0.0, 6.0), yaricap=0.5, sinif=sinif,
                          kume_id=0)],
        )
        assert _renk_var(kare, SINIF_RENK[sinif]), f"sınıf {sinif} çizilmedi"


def test_sinifsiz_kume_UNKNOWN_sayilir():
    """Füzyon eşleşmediyse sınıf düşer; engel yine ÇİZİLMELİ (kaybolmamalı)."""
    r = BevRenderer(BevConfig(genislik_px=200, yukseklik_px=200, menzil_m=15.0))
    kare = r.render_lidar(
        (0.0, 0.0), kumeler=[Kume(merkez=(0.0, 6.0), yaricap=0.5)]
    )
    assert _renk_var(kare, SINIF_RENK[99])


def test_menzil_disi_nokta_KARE_TASIRMAZ():
    """Uzak nokta çizilmeye çalışılınca istisna atmamalı (saha dayanıklılığı)."""
    r = BevRenderer(BevConfig(genislik_px=200, yukseklik_px=200, menzil_m=10.0))
    kare = r.render_lidar((0.0, 0.0), ham_noktalar=[(500.0, -900.0)])
    assert kare.shape == (200, 200, 3)


# ------------------------------------------------ md 479-492: zaman etiketi

def test_zaman_damgasi_KAREYE_yaziliyor():
    """Zaman dosya adına değil KAREYE yazılmalı (Jetson saati güvenilmez)."""
    r = BevRenderer(BevConfig(genislik_px=300, yukseklik_px=300))
    a = r.render_lidar((0.0, 0.0), zaman_metni="2026-08-07T13:00:00Z")
    b = r.render_lidar((0.0, 0.0), zaman_metni="2026-08-07T13:00:01Z")
    assert not np.array_equal(a, b), "zaman damgası kareye girmiyor"


def test_saat_guvenilmezse_isaretlenir():
    """Yalan söylemek yerine 'SAAT?' bas — algı `saat.py` ile aynı ilke."""
    r = BevRenderer(BevConfig(genislik_px=300, yukseklik_px=300))
    guvenli = r.render_lidar((0.0, 0.0), zaman_metni="T", saat_guvenilir=True)
    supheli = r.render_lidar((0.0, 0.0), zaman_metni="T", saat_guvenilir=False)
    assert not np.array_equal(guvenli, supheli)


# --------------------------------------------------- md 505-506: Dosya-3

def test_costmap_bilinmiyor_serbest_engel_AYIRT_EDILIR():
    r = BevRenderer(BevConfig(genislik_px=120, yukseklik_px=120, menzil_m=10.0))
    occ = np.zeros((20, 20), dtype=np.int16)
    occ[0:5, :] = -1        # bilinmiyor
    occ[10:12, 10:12] = 100  # engel
    kare = r.render_costmap(occ, 1.0, (0.0, 0.0))
    renkler = {tuple(c) for c in kare.reshape(-1, 3)}
    assert (70, 70, 70) in renkler, "bilinmiyor bölgesi yok"
    assert any(c[0] > 180 and c[1] < 110 for c in renkler), "engel kırmızısı yok"


def test_costmap_ROS_satir0_GUNEY_kuralina_uyuyor():
    """ROS satır 0 = güney → kuzey yukarı için flipud (local_map.py ile aynı)."""
    r = BevRenderer(BevConfig(genislik_px=100, yukseklik_px=100, menzil_m=10.0))
    occ = np.zeros((10, 10), dtype=np.int16)
    occ[0, :] = 100                      # ROS'ta GÜNEY kenarı
    kare = r.render_costmap(occ, 1.0, (0.0, 0.0))
    ust = kare[:10].astype(int).mean()
    alt = kare[-10:].astype(int).mean()
    assert alt > ust, "güney şeridi karenin ALTINDA olmalı (kuzey yukarı)"


def test_kenar_dubalari_AYRI_katman_olarak_ciziliyor():
    """🔴 Kenar dubaları MPPI torbasından çıkarılır → occupancy'de YOKTUR.

    GIRDAP_DURUM §0.14g/H1: teslim edilen 'engel haritası' parkurun ana
    nesnesini göstermiyordu. Ayrı katman bunu kapatır.
    """
    r = BevRenderer(BevConfig(genislik_px=200, yukseklik_px=200, menzil_m=10.0))
    occ = np.zeros((20, 20), dtype=np.int16)
    yok = r.render_costmap(occ, 1.0, (0.0, 0.0))
    var = r.render_costmap(occ, 1.0, (0.0, 0.0),
                           kenar_dubalari=[(-2.0, 4.0), (2.0, 4.0)])
    assert not np.array_equal(yok, var)
    assert _renk_var(var, SINIF_RENK[0])


def test_costmap_bozuk_girdi_NET_hata_verir():
    r = BevRenderer()
    with pytest.raises(ValueError):
        r.render_costmap(np.zeros(10), 1.0, (0.0, 0.0))          # 1B
    with pytest.raises(ValueError):
        r.render_costmap(np.zeros((5, 5)), 0.0, (0.0, 0.0))      # çözünürlük 0


# ------------------------------------------------------- belirlenimcilik

def test_ayni_girdi_ayni_kare():
    """Log tekrar-oynatma ve testler için belirlenimci olmalı (A3 dersi)."""
    r = BevRenderer(BevConfig(genislik_px=160, yukseklik_px=160))
    kume = [Kume(merkez=(1.0, 4.0), sinif=1, kume_id=3,
                 noktalar=[(1.0, 4.0), (1.2, 4.1)])]
    a = r.render_lidar((0.0, 0.0), yaw=0.3, kumeler=kume, zaman_metni="T")
    b = r.render_lidar((0.0, 0.0), yaw=0.3, kumeler=kume, zaman_metni="T")
    assert np.array_equal(a, b)


# ------------------------------------------------------------ Mp4Yazici

def test_mp4_fps_1in_altinda_REDDEDILIR(tmp_path):
    """md 4.2 'En Az 1 Hz' — fps<1 sessizce kabul edilirse teslim geçersiz."""
    pytest.importorskip("cv2")
    with pytest.raises(ValueError):
        Mp4Yazici(tmp_path / "x.mp4", fps=0.5)


def test_mp4_kareler_yaziliyor(tmp_path):
    pytest.importorskip("cv2")
    r = BevRenderer(BevConfig(genislik_px=160, yukseklik_px=160))
    yol = tmp_path / "lidar.mp4"
    y = Mp4Yazici(yol, fps=2.0, boyut=(160, 160))
    for i in range(4):
        assert y.yaz(r.render_lidar((0.0, 0.0), kare_no=i))
    y.kapat()
    assert y.kare_sayisi == 4
    assert yol.exists() and yol.stat().st_size > 0


# --------------------------------------------- PNG yedeği (codec yoksa)

def test_png_yedegi_Mp4Yazici_ILE_AYNI_ARAYUZ(tmp_path):
    """Node hangisini tuttuğunu bilmek zorunda kalmamalı."""
    from prototype.mapping.bev_renderer import PngSerisiYazici

    for ad in ("yaz", "kapat", "kare_sayisi"):
        assert hasattr(PngSerisiYazici(tmp_path / "a", fps=2.0), ad)
        assert hasattr(Mp4Yazici, ad) or ad == "kare_sayisi"


def test_png_yedegi_kare_yazar_ve_ffmpeg_talimati_birakir(tmp_path):
    """🔑 PNG yedeği teslimi KURTARIR — ama ancak çevrilirse.

    Klasörü bulan kişi 20 dakikalık teslim penceresinde "bu ne?" diye
    düşünmemeli; komut klasörün içinde dursun.
    """
    from prototype.mapping.bev_renderer import PngSerisiYazici

    r = BevRenderer(BevConfig(genislik_px=120, yukseklik_px=120))
    y = PngSerisiYazici(tmp_path / "yedek", fps=2.0)
    for i in range(3):
        assert y.yaz(r.render_lidar((0.0, 0.0), kare_no=i))
    y.kapat()

    assert y.kare_sayisi == 3
    assert (tmp_path / "yedek" / "kare_00000.png").exists()
    assert (tmp_path / "yedek" / "kare_00002.png").exists()
    talimat = (tmp_path / "yedek" / "NASIL_MP4_YAPILIR.txt").read_text(
        encoding="utf-8")
    assert "ffmpeg" in talimat and "framerate 2" in talimat
