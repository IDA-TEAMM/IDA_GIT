"""
Girdap İDA — iSAM2 sağlamlaştırma testleri: robust GPS + keyframe throttle.

Kapsam:
    1) Outlier GPS (10 m sapma) — robust AÇIK vs KAPALI çözüm sapması
    2) add_gps(sigma_xy=...) doğru gürültü modelini üretiyor mu
    3) gps_robust_enabled=False → eski (saf Gauss) davranış birebir korunur
    4) keyframe_rate_hz throttle'ı key kadansını gerçekten sınırlıyor mu
    5) throttle bilgi kaybettirmiyor (birikmiş delta = tek BetweenFactor)

Çalıştır: pytest prototype/tests/test_isam2_robust.py -v
"""

from __future__ import annotations

import math

import numpy as np
import pytest

pytest.importorskip("gtsam", reason="gtsam yok — iSAM2 füzyon testleri atlanır")

import gtsam                                                    # noqa: E402

from prototype.fusion.isam2_smoother import (                   # noqa: E402
    ISAM2Smoother,
    ISAM2SmootherConfig,
)
from prototype.fusion.pipeline import (                         # noqa: E402
    FusionPipeline,
    FusionPipelineConfig,
)


# --------------------------------------------------------------------------- #
# 1) Outlier GPS reddi
# --------------------------------------------------------------------------- #

# Düz çizgi senaryosu: 1 m/s ile +x boyunca, her key 1 m ilerler.
_N_KEYS = 40
_STEP_M = 1.0
_OUTLIER_KEY = 20          # ortadaki keye kötü fix bindirilir
_OUTLIER_OFFSET_M = 10.0   # şartname senaryosu: 10 m sapma


def _run_straight_line(*, robust: bool, outlier: bool) -> np.ndarray:
    """Sabit hızlı düz çizgi + her keyde GPS. Dönüş: (N, 3) smooth yörünge.

    Ölçümler gürültüsüz — böylece ölçülen sapma YALNIZ outlier'ın etkisidir
    (gürültü realizasyonu karşılaştırmayı kirletmez).
    """
    cfg = ISAM2SmootherConfig(
        gps_sigma_xy=0.10,
        odom_sigma_xy=0.10,
        odom_sigma_psi=0.02,
        gps_robust_enabled=robust,
    )
    sm = ISAM2Smoother(cfg)
    sm.initialize(gtsam.Pose2(0.0, 0.0, 0.0))

    for k in range(1, _N_KEYS + 1):
        sm.add_odometry(gtsam.Pose2(_STEP_M, 0.0, 0.0))
        gx, gy = float(k) * _STEP_M, 0.0
        if outlier and k == _OUTLIER_KEY:
            gy += _OUTLIER_OFFSET_M          # tek kötü fix (multipath/RTK kaybı)
        sm.add_gps(sm.latest_key, gx, gy)
        sm.update()

    return sm.all_xy_psi()


def _deviation_vs_truth(traj: np.ndarray) -> tuple[float, float]:
    """(maksimum, RMS) yanal sapma [m] — gerçek yörünge y=0 düz çizgisi."""
    lateral = np.abs(traj[:, 1])
    return float(lateral.max()), float(np.sqrt((lateral ** 2).mean()))


def test_outlier_gps_robust_sapmayi_belirgin_azaltir() -> None:
    """10 m'lik tek kötü GPS fix'i: Huber AÇIK çözümü çok daha az bozmalı.

    Saf Gauss modelinde outlier kare-hata ile cezalandırılır → komşu keyleri
    de kendine çeker (smoother'ın tüm penceresi bozulur). Huber, hatayı
    k·σ ötesinde doğrusal cezalandırdığı için ağırlığı söner.
    """
    plain = _run_straight_line(robust=False, outlier=True)
    robust = _run_straight_line(robust=True, outlier=True)

    plain_max, plain_rms = _deviation_vs_truth(plain)
    robust_max, robust_rms = _deviation_vs_truth(robust)

    print(
        f"\n[outlier {_OUTLIER_OFFSET_M:.0f} m @ key {_OUTLIER_KEY}]"
        f"\n  robust KAPALI : max={plain_max:.3f} m  rms={plain_rms:.3f} m"
        f"\n  robust AÇIK   : max={robust_max:.3f} m  rms={robust_rms:.3f} m"
        f"\n  iyileşme      : max {plain_max / max(robust_max, 1e-9):.1f}x, "
        f"rms {plain_rms / max(robust_rms, 1e-9):.1f}x"
    )

    assert robust_max < plain_max, "robust kernel sapmayı azaltmadı"
    # Kalite kapısı: outlier etkisi en az 3 kat bastırılmalı, yoksa Huber
    # eşiği (k) ölçüm sigma'sına göre yanlış ayarlanmış demektir.
    assert plain_max / robust_max >= 3.0, (
        f"outlier bastırma yetersiz: kapalı={plain_max:.3f} m, "
        f"açık={robust_max:.3f} m"
    )
    # Mutlak kapı: 10 m'lik outlier çözümü yarım metreden fazla kaydırmamalı.
    assert robust_max < 0.5, f"robust çözüm hâlâ {robust_max:.3f} m sapıyor"


def test_outlier_yokken_robust_temiz_cozumu_bozmaz() -> None:
    """Huber yalnız outlier'da devreye girmeli — temiz veride bedeli ~0."""
    plain = _run_straight_line(robust=False, outlier=False)
    robust = _run_straight_line(robust=True, outlier=False)

    _, plain_rms = _deviation_vs_truth(plain)
    _, robust_rms = _deviation_vs_truth(robust)
    print(
        f"\n[outlier YOK] robust KAPALI rms={plain_rms:.4f} m | "
        f"AÇIK rms={robust_rms:.4f} m"
    )
    assert robust_rms == pytest.approx(plain_rms, abs=1e-3)


def test_robust_outlierdan_sonra_toparlanir() -> None:
    """Outlier'dan SONRAKİ keyler temiz kalmalı (kalıcı bias bırakmamalı)."""
    robust = _run_straight_line(robust=True, outlier=True)
    son_kesim = np.abs(robust[_OUTLIER_KEY + 5:, 1])
    assert son_kesim.max() < 0.10, (
        f"outlier sonrası kalıcı sapma: {son_kesim.max():.3f} m"
    )


# --------------------------------------------------------------------------- #
# 2) sigma_xy override → gürültü modeli
# --------------------------------------------------------------------------- #


def _last_factor_noise(sm: ISAM2Smoother):            # noqa: ANN202
    """Henüz flush edilmemiş son faktörün gürültü modeli."""
    graph = sm._graph
    return graph.at(graph.size() - 1).noiseModel()


def _sigmas(model) -> np.ndarray:                    # noqa: ANN001
    """Diyagonal modelin sigma vektörü.

    NOT: gtsam Python binding'i Robust sarmalayıcısının TABAN modeline
    erişim vermez (yalnız Create/print/serialize). Bu yüzden sigma'nın
    faktöre doğru geçtiği robust KAPALI yolda birebir doğrulanır; robust
    AÇIK yol ayrıca (a) sarmalayıcı tipiyle ve (b) davranışsal olarak
    (test_pipeline_sigma_override_gpsi_zayiflatir) test edilir.
    """
    assert isinstance(model, gtsam.noiseModel.Diagonal), type(model)
    return np.asarray(model.sigmas())


@pytest.mark.parametrize("sigma_xy", [0.05, 0.50, 2.50])
def test_add_gps_sigma_override_dogru_modeli_uretir(sigma_xy: float) -> None:
    """Fix kalitesinden gelen sigma, faktörün (x, y) kanallarına yazılmalı."""
    sm = ISAM2Smoother(
        ISAM2SmootherConfig(gps_sigma_xy=0.99, gps_robust_enabled=False)
    )
    sm.initialize(gtsam.Pose2(0.0, 0.0, 0.0))
    sm.add_gps(0, 1.0, 2.0, sigma_xy=sigma_xy)

    sigmas = _sigmas(_last_factor_noise(sm))
    assert sigmas[0] == pytest.approx(sigma_xy)
    assert sigmas[1] == pytest.approx(sigma_xy)
    # heading kanalı ölçülmüyor → uninformative kalmalı
    assert sigmas[2] > 1e3


def test_add_gps_sigma_verilmezse_config_varsayilani() -> None:
    sm = ISAM2Smoother(
        ISAM2SmootherConfig(gps_sigma_xy=0.30, gps_robust_enabled=False)
    )
    sm.initialize(gtsam.Pose2(0.0, 0.0, 0.0))
    sm.add_gps(0, 1.0, 2.0)
    assert _sigmas(_last_factor_noise(sm))[0] == pytest.approx(0.30)


def test_add_gps_gecersiz_sigma_reddedilir() -> None:
    """σ<=0 sonsuz güven demek — sessizce kabul edilmemeli."""
    sm = ISAM2Smoother(ISAM2SmootherConfig())
    sm.initialize(gtsam.Pose2(0.0, 0.0, 0.0))
    with pytest.raises(ValueError):
        sm.add_gps(0, 1.0, 2.0, sigma_xy=0.0)
    with pytest.raises(ValueError):
        sm.add_gps(0, 1.0, 2.0, sigma_xy=-1.0)


def test_sigma_override_onbellegi_ayni_modeli_dondurur() -> None:
    """Sıcak yol: aynı sigma her fix'te yeni model alloke etmemeli."""
    sm = ISAM2Smoother(ISAM2SmootherConfig())
    sm.initialize(gtsam.Pose2(0.0, 0.0, 0.0))
    assert sm._gps_noise_for(0.5) is sm._gps_noise_for(0.5)


# --------------------------------------------------------------------------- #
# 3) Geri uyumluluk: robust kapalı = eski davranış
# --------------------------------------------------------------------------- #


def test_robust_kapali_saf_diagonal_model_uretir() -> None:
    """gps_robust_enabled=False → Robust sarmalayıcı HİÇ olmamalı."""
    sm = ISAM2Smoother(ISAM2SmootherConfig(gps_robust_enabled=False))
    sm.initialize(gtsam.Pose2(0.0, 0.0, 0.0))
    sm.add_gps(0, 1.0, 2.0)
    assert not isinstance(_last_factor_noise(sm), gtsam.noiseModel.Robust)


def test_robust_acik_huber_kerneli_sarar() -> None:
    sm = ISAM2Smoother(ISAM2SmootherConfig(gps_robust_enabled=True))
    sm.initialize(gtsam.Pose2(0.0, 0.0, 0.0))
    sm.add_gps(0, 1.0, 2.0)
    assert isinstance(_last_factor_noise(sm), gtsam.noiseModel.Robust)


def test_robust_yalnizca_gpsi_sarar_odometriyi_degil() -> None:
    """Odometri outlier üretmez (IMU sürekli) — kernel oraya bulaşmamalı."""
    sm = ISAM2Smoother(ISAM2SmootherConfig(gps_robust_enabled=True))
    sm.initialize(gtsam.Pose2(0.0, 0.0, 0.0))
    sm.add_odometry(gtsam.Pose2(1.0, 0.0, 0.0))
    assert not isinstance(_last_factor_noise(sm), gtsam.noiseModel.Robust)


def test_huber_k_config_degeri_kullanilir() -> None:
    """gps_huber_k varsayılanı literatür değeri 1.345 olmalı."""
    assert ISAM2SmootherConfig().gps_huber_k == pytest.approx(1.345)
    assert ISAM2SmootherConfig().gps_robust_enabled is True


def test_gecersiz_huber_k_erken_patlar() -> None:
    """k<=0 → Huber ağırlığı her ölçümde 0: GPS sessizce TAMAMEN yok sayılır.

    Bu, aracı yalnız IMU ölü-hesabına bırakır. yaml yazım hatası saha
    testinde değil, burada yakalanmalı.
    """
    for k in (0.0, -1.0):
        with pytest.raises(ValueError):
            ISAM2Smoother(ISAM2SmootherConfig(gps_huber_k=k))
    # robust kapalıyken k anlamsız → engellemeye gerek yok
    ISAM2Smoother(ISAM2SmootherConfig(gps_robust_enabled=False, gps_huber_k=0.0))


# --------------------------------------------------------------------------- #
# 4) Keyframe throttle
# --------------------------------------------------------------------------- #


def _keyframe_rate(
    *, keyframe_rate_hz: float, imu_hz: float, duration_s: float,
    gps_hz: float = 1.0,
) -> tuple[int, float]:
    """Senaryoyu koştur → (key sayısı, ölçülen key/s)."""
    fp = FusionPipeline(
        FusionPipelineConfig(keyframe_rate_hz=keyframe_rate_hz)
    )
    fp.on_gps(36.85, 28.27)                       # origin
    dt = 1.0 / imu_hz
    gps_stride = max(1, int(round(imu_hz / gps_hz))) if gps_hz > 0 else 0
    n = int(duration_s * imu_hz)
    for k in range(n):
        t = (k + 1) * dt
        fp.on_velocity(1.0, 0.0)
        fp.on_imu(t, 0.0)
        if gps_stride and (k + 1) % gps_stride == 0:
            lat, lon = fp.enu_to_latlon(t, 0.0)
            fp.on_gps(lat, lon)
    keys = fp._sm.latest_key + 1
    return keys, (keys - 1) / duration_s


def test_keyframe_throttle_20dk_gorevde_graf_buyumesini_yariya_indirir() -> None:
    """CLAUDE.md tuzağı: 20 dk görevde graf şişer. Throttle bunu sınırlamalı."""
    _, hizli = _keyframe_rate(keyframe_rate_hz=0.0, imu_hz=50.0, duration_s=60.0)
    _, kisik = _keyframe_rate(keyframe_rate_hz=5.0, imu_hz=50.0, duration_s=60.0)
    print(
        f"\n[keyframe] throttle KAPALI={hizli:.2f} Hz "
        f"(20 dk → {hizli * 1200:.0f} key) | "
        f"AÇIK(5 Hz)={kisik:.2f} Hz (20 dk → {kisik * 1200:.0f} key)"
    )
    assert kisik < hizli
    # GPS zorlamalı flush'ı (~1 Hz) hesaba kat: tavan 5 + 1 Hz.
    assert kisik <= 6.5, f"throttle tutmadı: {kisik:.2f} Hz"
    assert kisik * 1200 < 8000, "20 dk projeksiyonu hâlâ çok yüksek"


def test_throttle_kapaliyken_eski_kadans_korunur() -> None:
    """keyframe_rate_hz<=0 → geri uyumlu: kadans odom_period_s'te kalır."""
    cfg = FusionPipelineConfig(odom_period_s=0.1, keyframe_rate_hz=0.0)
    assert cfg.keyframe_period_s == pytest.approx(0.1)


def test_odom_period_alt_sinir_kalir() -> None:
    """Throttle periyodu KISALTAMAZ — odom_period_s alt sınırdır."""
    cfg = FusionPipelineConfig(odom_period_s=0.2, keyframe_rate_hz=20.0)
    assert cfg.keyframe_period_s == pytest.approx(0.2)


def test_throttle_ara_imu_adimlarini_kaybetmez() -> None:
    """Birikmiş delta = tek BetweenFactor: yol uzunluğu korunmalı.

    Ara adımlar atılsaydı (yalnız son hız örneği × periyot) 5 Hz'de tahmin
    edilen mesafe belirgin sapardı. Sabit hız + sabit dönüşle 20 s koştur,
    kat edilen yayın uzunluğunu gerçekle karşılaştır.
    """
    imu_hz, duration, speed, omega = 50.0, 20.0, 1.0, 0.1
    dt = 1.0 / imu_hz
    n = int(duration * imu_hz)
    sonuc = {}
    for rate in (0.0, 5.0):
        fp = FusionPipeline(FusionPipelineConfig(keyframe_rate_hz=rate))
        for k in range(n):
            fp.on_velocity(speed, 0.0)
            fp.on_imu((k + 1) * dt, omega)
        # Bekleyen kuyruğu da graf'a yaz — aksi halde son (≤1 keyframe) IMU
        # verisi henüz key'e dönüşmemiş olur ve karşılaştırma haksızlaşır.
        fp._flush(force=True)
        sonuc[rate] = (*fp.current_pose(), fp._sm.latest_key + 1)

    # Gerçek: sabit hız + sabit yaw rate → yarıçapı speed/omega olan yay.
    # İlk IMU mesajı yalnız zaman referansı kurar (delta üretmez) → entegre
    # edilen süre bir adım kısadır.
    integre_s = (n - 1) * dt
    r = speed / omega
    psi_t = omega * integre_s
    truth = (r * math.sin(psi_t), r * (1.0 - math.cos(psi_t)))

    for rate, (x, y, psi, keys) in sonuc.items():
        hata = math.hypot(x - truth[0], y - truth[1])
        print(
            f"\n[throttle {rate or 'KAPALI'}] key={keys} poz=({x:.3f}, "
            f"{y:.3f}, ψ={math.degrees(psi):.2f}°) yay hatası={hata:.4f} m"
        )
        assert hata < 0.10, f"throttle={rate} ile integrasyon hatası {hata:.3f} m"
        assert psi == pytest.approx(psi_t, abs=1e-6)

    # Asıl iddia: throttle YARIYA yakın key ile AYNI yörüngeyi üretir.
    (x0, y0, _, k0), (x5, y5, _, k5) = sonuc[0.0], sonuc[5.0]
    assert math.hypot(x5 - x0, y5 - y0) < 0.02, "throttle yörüngeyi kaydırdı"
    assert k5 < k0 * 0.6, f"throttle key sayısını düşürmedi ({k5} vs {k0})"


def test_throttle_odom_sigmasini_dt_ile_olcekler() -> None:
    """Uzun keyframe aralığı aynı σ ile yazılırsa zincir sahte güven kazanır.

    √Δt ölçeklemesi olmadan 5 Hz kadans, 10 Hz'e göre aynı sürede √2 kat
    daha güvenli görünür ve GPS'i haksız bastırırdı.
    """
    fp = FusionPipeline(FusionPipelineConfig(
        odom_period_s=0.1, keyframe_rate_hz=5.0, odom_sigma_xy=0.05,
    ))
    dt = 0.02
    for k in range(20):                       # 0.4 s → iki keyframe
        fp.on_velocity(1.0, 0.0)
        fp.on_imu((k + 1) * dt, 0.0)

    # Ölçek önbellek anahtarı olarak 4 haneye yuvarlanır (sınırlı önbellek);
    # sigma üzerindeki bağıl etki ~1e-5 → tolerans buna göre.
    sigmas = _sigmas(fp._sm._odom_noise_for(math.sqrt(2.0)))
    assert sigmas[0] == pytest.approx(0.05 * math.sqrt(2.0), rel=1e-4)
    # ölçek 1.0 → önbellekteki config modeli (yeni alokasyon yok)
    assert fp._sm._odom_noise_for(1.0) is fp._sm._odom_noise


# --------------------------------------------------------------------------- #
# 5) Boru hattı üzerinden uçtan uca outlier
# --------------------------------------------------------------------------- #


def test_pipeline_outlier_gps_fix_robustla_bastirilir() -> None:
    """FusionPipeline seviyesinde (lat/lon → ENU) outlier bastırma."""
    sapmalar = {}
    for robust in (False, True):
        fp = FusionPipeline(FusionPipelineConfig(
            gps_sigma_xy=0.10, keyframe_rate_hz=5.0,
            gps_robust_enabled=robust,
        ))
        fp.on_gps(36.85, 28.27)                        # origin
        dt = 0.02
        for k in range(1500):                          # 30 s @ 50 Hz
            t = (k + 1) * dt
            fp.on_velocity(1.0, 0.0)
            fp.on_imu(t, 0.0)
            if (k + 1) % 50 == 0:                      # 1 Hz GPS
                gy = 10.0 if k == 749 else 0.0         # 15. s'de tek outlier
                lat, lon = fp.enu_to_latlon(t, gy)
                fp.on_gps(lat, lon)
        sapmalar[robust] = float(np.abs(fp.all_xy_psi()[:, 1]).max())

    print(
        f"\n[pipeline outlier] robust KAPALI max|y|={sapmalar[False]:.3f} m | "
        f"AÇIK max|y|={sapmalar[True]:.3f} m"
    )
    assert sapmalar[True] < sapmalar[False] / 3.0


def test_pipeline_sigma_override_gpsi_zayiflatir() -> None:
    """Aynı ölçüm, kötü fix sigma'sıyla verilince çözümü daha az çekmeli."""
    cekimler = {}
    for sigma in (0.05, 2.50):                     # RTK vs tek nokta
        fp = FusionPipeline(FusionPipelineConfig(keyframe_rate_hz=5.0))
        fp.on_gps(36.85, 28.27)
        dt = 0.02
        for k in range(50):                        # 1 s hareketsiz
            fp.on_velocity(0.0, 0.0)
            fp.on_imu((k + 1) * dt, 0.0)
        lat, lon = fp.enu_to_latlon(0.0, 5.0)      # 5 m yanal ölçüm
        fp.on_gps(lat, lon, sigma_xy=sigma)
        cekimler[sigma] = fp.current_pose()[1]

    print(
        f"\n[sigma override] σ=0.05 → y={cekimler[0.05]:.3f} m | "
        f"σ=2.50 → y={cekimler[2.50]:.3f} m"
    )
    assert cekimler[0.05] > cekimler[2.50] * 2.0
