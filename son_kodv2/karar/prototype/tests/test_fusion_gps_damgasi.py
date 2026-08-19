"""§1.58 (19.08 gece) — GPS GİRİŞ damgası = ÖLÇÜM ANI nöbetçileri.

SORUN: `FusionPipeline.on_gps` hiçbir zaman damga taşımıyordu — GPS fix'i
her zaman "şimdi"nin (callback'in ateşlendiği an, `_last_imu_t`) key'ine
bağlanıyordu. GPS'in KENDİ ölçüm anı (`header.stamp`) ile callback'in
işlendiği an arasında (seri port/ROS/QoS gecikmesi) fark varsa, düzeltme
aracın O ANDA OLMADIĞI bir noktaya iğneleniyordu — §1.57'nin (poz ÇIKIŞ
damgası, 19.08 gece erken saatlerde düzeltildi) GİRİŞ tarafındaki aynı hata
sınıfı, henüz düzeltilmemişti.

ÇÖZÜM: `on_gps(..., t=...)` verilirse ve bu keyframe periyodu içinde GPS'ten
SONRA gelmiş IMU örnekleri varsa, prior GPS'in KENDİ ölçüm anındaki birikmiş
deltaya "geri sarılır" (`_gecmiste_delta`) — GPS'ten SONRAKİ IMU örnekleri
bir SONRAKİ segmente devredilir, kaybolmaz.
"""
from __future__ import annotations

import math

from prototype.fusion.pipeline import FusionPipeline, FusionPipelineConfig


def _duz_cizgi_besle(fp: FusionPipeline, t0: float, n: int, dt: float,
                      vx: float = 2.0) -> float:
    """n adım düz çizgi (dönüşsüz) IMU besle — her adım vx*dt kadar ileri.
    Dönüş sıfır olduğu için Pose2 kompozisyonu düz toplamla ÇAKIŞIR, bu da
    beklenen değerleri elle hesaplamayı basitleştirir (testin kendi amacı)."""
    t = t0
    for _ in range(n):
        t += dt
        fp.on_velocity(vx, 0.0)
        fp.on_imu(t, 0.0)
    return t


def _kurulu_pipeline(keyframe_rate_hz: float = 1.0, t0: float = 1000.0) -> FusionPipeline:
    """Kurulu pipeline + BİR taban IMU çağrısı (`_last_imu_t`'yi kurar).

    `on_imu`'nun İLK çağrısı yalnız `_last_imu_t`'yi başlatır, HİÇBİR delta
    BİRİKTİRMEZ (dt hesaplanamaz — referans yok). Bu taban çağrı burada
    YAPILIR ki testteki her `_duz_cizgi_besle` çağrısı n adımın TAMAMINI
    gerçek birikim olarak sayabilsin — aksi hâlde "n adım besledim" ile
    "n-1 gerçek birikim oldu" karışır.
    """
    fp = FusionPipeline(FusionPipelineConfig(keyframe_rate_hz=keyframe_rate_hz))
    fp.set_origin(40.0, 30.0)
    fp.on_velocity(0.0, 0.0)
    fp.on_imu(t0, 0.0)             # taban — birikim yok, yalnız _last_imu_t
    return fp


def test_t_VERILMEZSE_eski_davranis_birebir():
    """`t=None` (varsayılan) → GPS her zamanki gibi 'şimdi'ye bağlanır,
    geri sarma DEVREYE GİRMEZ. Geriye tam uyumluluk."""
    fp = _kurulu_pipeline()
    son_t = _duz_cizgi_besle(fp, 1000.0, 4, dt=0.1)   # 1000.4, dx toplam 0.8
    fp.on_gps(40.0001, 30.0, sigma_xy=0.01)           # t verilmiyor
    assert fp._son_anahtar_t == son_t
    assert math.isclose(fp._t_since_flush, 0.0, abs_tol=1e-9)


def test_gecikmeli_GPS_kendi_anina_geri_sariliyor():
    """GPS, en son IMU örneğinden ÖNCEKİ bir ana ait (t=1000.2, son IMU
    1000.4) — prior o ANA bağlanmalı, kalan iki adım (1000.3, 1000.4) bir
    SONRAKİ segmente devredilmeli, kaybolmamalı."""
    fp = _kurulu_pipeline()
    _duz_cizgi_besle(fp, 1000.0, 4, dt=0.1, vx=2.0)   # adımlar: .1,.2,.3,.4
    # snapshot'lar (dx): 1000.1→0.2 · 1000.2→0.4 · 1000.3→0.6 · 1000.4→0.8
    onceki_sayac = fp._sm.latest_key
    fp.on_gps(40.0, 30.0, sigma_xy=0.01, t=1000.2)

    # Yeni bir key AÇILMIŞ olmalı (backdated flush gerçekleşti).
    assert fp._sm.latest_key == onceki_sayac + 1
    # Damga GPS'in KENDİ anı — flush anı (1000.4) DEĞİL.
    assert math.isclose(fp._son_anahtar_t, 1000.2, abs_tol=1e-9)
    # Kalan (1000.3, 1000.4 adımları) sonraki segmente TAŞINMIŞ olmalı:
    # dx_kalan = 0.8 - 0.4 = 0.4 m, süre_kalan = 1000.4 - 1000.2 = 0.2 s.
    kalan_dx = float(fp._acc_delta.translation()[0])
    assert math.isclose(kalan_dx, 0.4, abs_tol=1e-6), kalan_dx
    assert math.isclose(fp._t_since_flush, 0.2, abs_tol=1e-9)


def test_gecikmeli_GPS_sonraki_IMU_adimlarini_KAYBETMIYOR():
    """Geri sarmadan SONRA gelen IMU adımları normal şekilde birikmeye devam
    etmeli — 'kalan' segment kopmadan sürer."""
    fp = _kurulu_pipeline()
    _duz_cizgi_besle(fp, 1000.0, 4, dt=0.1, vx=2.0)   # ...1000.4, dx=0.8
    fp.on_gps(40.0, 30.0, sigma_xy=0.01, t=1000.2)     # kalan dx=0.4, süre=0.2

    # 2 adım daha (1000.5, 1000.6) — kalan üstüne binmeli.
    _duz_cizgi_besle(fp, 1000.4, 2, dt=0.1, vx=2.0)
    # toplam kalan dx = 0.4 (önceki) + 0.2 + 0.2 = 0.8
    toplam_dx = float(fp._acc_delta.translation()[0])
    assert math.isclose(toplam_dx, 0.8, abs_tol=1e-6), toplam_dx
    assert math.isclose(fp._t_since_flush, 0.4, abs_tol=1e-9)


def test_GPS_zaten_guncelse_geri_sarma_TETIKLENMEZ():
    """t == en son IMU örneğinin anı (gecikme yok) → geri sarmaya GEREK yok,
    eski (basit) yol kullanılır — gereksiz karmaşıklık eklenmez."""
    fp = _kurulu_pipeline()
    son_t = _duz_cizgi_besle(fp, 1000.0, 4, dt=0.1, vx=2.0)
    fp.on_gps(40.0, 30.0, sigma_xy=0.01, t=son_t)      # tam "şimdi"
    assert fp._son_anahtar_t == son_t
    assert math.isclose(fp._t_since_flush, 0.0, abs_tol=1e-9)


def test_GPS_gecmisten_daha_eskiyse_sessizce_eski_yola_duser():
    """GPS'in ölçüm anı `_acc_delta_gecmisi`de HİÇ kayıt yoksa (bu segmentten
    önceki bir key'e ait, düzeltilemez) — hata FIRLATILMAZ, sessizce eski
    (basit) davranışa düşülür. Güvenli varsayılan: DÜZELTME YOKSA da GPS
    hâlâ smoother'a ULAŞIR (hiç eklenmemekten iyidir)."""
    fp = _kurulu_pipeline()
    son_t = _duz_cizgi_besle(fp, 1000.0, 4, dt=0.1, vx=2.0)
    fp.on_gps(40.0, 30.0, sigma_xy=0.01, t=1.0)        # akla mantığa sığmayan eski
    assert fp._son_anahtar_t == son_t                   # eski yola düştü
    assert math.isclose(fp._t_since_flush, 0.0, abs_tol=1e-9)


def test_gecmis_her_flushta_temizleniyor():
    """`_acc_delta_gecmisi` sınırsız büyümez — her flush'ta (normal ya da
    geri-sarılmış) temizlenir, bir sonraki segmentin kendi kaydını tutar."""
    fp = _kurulu_pipeline(keyframe_rate_hz=1.0)
    _duz_cizgi_besle(fp, 1000.0, 4, dt=0.1, vx=2.0)
    assert len(fp._acc_delta_gecmisi) == 4
    fp.on_gps(40.0, 30.0, sigma_xy=0.01, t=1000.2)
    # geri sarmadan SONRAKİ örnekler (1000.3, 1000.4) kalmalı, ÖNCEKİLER
    # (1000.1, 1000.2) düşmeli.
    assert len(fp._acc_delta_gecmisi) == 2
