"""
Girdap İDA — iSAM2 heading (pusula) sağlamlaştırma testleri (F-F.2, 18.08.2026).

SAHA OLAYI (17.08 akşam gölü, bkz. isam2_smoother.py `add_heading`
docstring'i): saf Gauss heading prior'unda tek kötü AHRS okuması (manyetik
girişim) whitened hatayı onlarca sigma'ya çıkardı, kare-hata cezası iSAM2
çözümünü saniyeler içinde diverge ettirdi (x/y katlanarak büyüdü, günlükte
1e23 → 1e73). `/girdap/fusion/odom` o koşumun geri kalanında (8+ saat) bir
daha HİÇ yayınlanmadı — F-F.1 (§0.98a) makullük kapısı yalan söylemeyi
engelledi ama pozu da tamamen sessize aldı.

Bu dosya `test_isam2_robust.py`'nin (GPS) aynasıdır — GPS'in Huber
korumasının heading'de de var olduğunu ve gerçekten işe yaradığını kilitler.

Çalıştır: pytest prototype/tests/test_isam2_heading_robust.py -v
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


# --------------------------------------------------------------------------- #
# 1) Outlier heading reddi — düz çizgi + tek kötü pusula okuması
# --------------------------------------------------------------------------- #

# Düz çizgi senaryosu: her key'de +x'e 1 m ilerle (body-frame odometri),
# gerçek psi hep 0. GPS YOK — saf heading etkisini ölçmek için (GPS'in
# kendi Huber koruması ayrı testte kanıtlı, burada karışmasın).
_N_KEYS = 40
_STEP_M = 1.0
_OUTLIER_KEY = 20                 # ortadaki keye kötü pusula bindirilir
_OUTLIER_PSI_ERR = math.radians(90.0)   # manyetik girişimde görülen büyüklükte


def _run_straight_line(*, robust: bool, outlier: bool) -> np.ndarray:
    """Sabit hızlı düz çizgi + her keyde heading. Dönüş: (N, 3) smooth yörünge."""
    cfg = ISAM2SmootherConfig(
        odom_sigma_xy=0.10,
        odom_sigma_psi=0.02,
        heading_sigma_psi=0.05,
        heading_robust_enabled=robust,
    )
    sm = ISAM2Smoother(cfg)
    sm.initialize(gtsam.Pose2(0.0, 0.0, 0.0))

    for k in range(1, _N_KEYS + 1):
        sm.add_odometry(gtsam.Pose2(_STEP_M, 0.0, 0.0))
        psi = 0.0
        if outlier and k == _OUTLIER_KEY:
            psi += _OUTLIER_PSI_ERR      # tek kötü AHRS okuması
        sm.add_heading(sm.latest_key, psi)
        sm.update()

    return sm.all_xy_psi()


def _deviation_vs_truth(traj: np.ndarray) -> tuple[float, float]:
    """(maksimum, RMS) yanal sapma [m] — gerçek yörünge y=0 düz çizgisi."""
    lateral = np.abs(traj[:, 1])
    return float(lateral.max()), float(np.sqrt((lateral ** 2).mean()))


def test_outlier_heading_robust_sapmayi_belirgin_azaltir() -> None:
    """90°'lik tek kötü pusula okuması: Huber AÇIK çözümü çok daha az bozmalı.

    Saf Gauss modelinde outlier kare-hata ile cezalandırılır → yalnız o
    key'in psi'sini değil, sonraki TÜM odometri entegrasyonunu (yanlış
    açıyla ileri gitme) kendine çeker — tam olarak 17.08'de gözlenen
    katlanarak büyüyen sapma deseni.
    """
    plain = _run_straight_line(robust=False, outlier=True)
    robust = _run_straight_line(robust=True, outlier=True)

    plain_max, plain_rms = _deviation_vs_truth(plain)
    robust_max, robust_rms = _deviation_vs_truth(robust)

    print(
        f"\n[outlier heading {math.degrees(_OUTLIER_PSI_ERR):.0f}° @ key {_OUTLIER_KEY}]"
        f"\n  robust KAPALI : max={plain_max:.3f} m  rms={plain_rms:.3f} m"
        f"\n  robust AÇIK   : max={robust_max:.3f} m  rms={robust_rms:.3f} m"
        f"\n  iyileşme      : max {plain_max / max(robust_max, 1e-9):.1f}x, "
        f"rms {plain_rms / max(robust_rms, 1e-9):.1f}x"
    )

    assert robust_max < plain_max, "robust kernel sapmayı azaltmadı"
    assert plain_max / robust_max >= 3.0, (
        f"outlier bastırma yetersiz: kapalı={plain_max:.3f} m, "
        f"açık={robust_max:.3f} m"
    )


def test_outlier_yokken_heading_robust_temiz_cozumu_bozmaz() -> None:
    """Huber yalnız outlier'da devreye girmeli — temiz veride bedeli ~0."""
    plain = _run_straight_line(robust=False, outlier=False)
    robust = _run_straight_line(robust=True, outlier=False)

    _, plain_rms = _deviation_vs_truth(plain)
    _, robust_rms = _deviation_vs_truth(robust)
    assert robust_rms == pytest.approx(plain_rms, abs=1e-3)


def test_robust_heading_outlierdan_sonra_toparlanir() -> None:
    """Outlier'dan SONRAKİ keyler temiz kalmalı (kalıcı bias bırakmamalı)."""
    robust = _run_straight_line(robust=True, outlier=True)
    son_kesim = np.abs(robust[_OUTLIER_KEY + 5:, 1])
    assert son_kesim.max() < 0.10, (
        f"outlier sonrası kalıcı sapma: {son_kesim.max():.3f} m"
    )


# --------------------------------------------------------------------------- #
# 2) Geri uyumluluk + model şekli
# --------------------------------------------------------------------------- #


def _last_factor_noise(sm: ISAM2Smoother):            # noqa: ANN202
    graph = sm._graph
    return graph.at(graph.size() - 1).noiseModel()


def test_robust_kapali_heading_saf_diagonal_model_uretir() -> None:
    """heading_robust_enabled=False → Robust sarmalayıcı HİÇ olmamalı."""
    sm = ISAM2Smoother(ISAM2SmootherConfig(heading_robust_enabled=False))
    sm.initialize(gtsam.Pose2(0.0, 0.0, 0.0))
    sm.add_heading(0, 0.1)
    assert not isinstance(_last_factor_noise(sm), gtsam.noiseModel.Robust)


def test_robust_acik_heading_huber_kerneli_sarar() -> None:
    sm = ISAM2Smoother(ISAM2SmootherConfig(heading_robust_enabled=True))
    sm.initialize(gtsam.Pose2(0.0, 0.0, 0.0))
    sm.add_heading(0, 0.1)
    assert isinstance(_last_factor_noise(sm), gtsam.noiseModel.Robust)


def test_robust_heading_yalnizca_pusulayi_sarar_odometriyi_degil() -> None:
    """Odometri outlier üretmez (IMU sürekli) — kernel oraya bulaşmamalı."""
    sm = ISAM2Smoother(ISAM2SmootherConfig(heading_robust_enabled=True))
    sm.initialize(gtsam.Pose2(0.0, 0.0, 0.0))
    sm.add_odometry(gtsam.Pose2(1.0, 0.0, 0.0))
    assert not isinstance(_last_factor_noise(sm), gtsam.noiseModel.Robust)


def test_heading_huber_k_config_degeri_kullanilir() -> None:
    """heading_huber_k varsayılanı GPS'inkiyle aynı literatür değeri 1.345."""
    assert ISAM2SmootherConfig().heading_huber_k == pytest.approx(1.345)
    assert ISAM2SmootherConfig().heading_robust_enabled is True


def test_gecersiz_heading_huber_k_erken_patlar() -> None:
    """k<=0 → Huber ağırlığı her ölçümde 0: heading sessizce TAMAMEN yok sayılır.

    Bu, aracı yalnız jiroskop ölü-hesabına bırakır. yaml yazım hatası saha
    testinde değil, burada yakalanmalı.
    """
    for k in (0.0, -1.0):
        with pytest.raises(ValueError):
            ISAM2Smoother(ISAM2SmootherConfig(heading_huber_k=k))
    # robust kapalıyken k anlamsız → engellemeye gerek yok
    ISAM2Smoother(
        ISAM2SmootherConfig(heading_robust_enabled=False, heading_huber_k=0.0)
    )


def test_heading_noise_onbellegi_ayni_modeli_dondurur() -> None:
    """Sıcak yol: aynı sigma her heading ölçümünde yeni model alloke etmemeli."""
    sm = ISAM2Smoother(ISAM2SmootherConfig())
    sm.initialize(gtsam.Pose2(0.0, 0.0, 0.0))
    assert sm._heading_noise_for(0.05) is sm._heading_noise_for(0.05)
    assert sm._heading_noise_for(None) is sm._heading_noise
