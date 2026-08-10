"""
Girdap İDA — iSAM2 heading (mutlak yön) düzeltmesi testleri.

11.08.2026 bulgusu: smoother'ın psi çıktısı yalnız jiroskopu entegre
ediyordu — hiçbir mutlak referansı yoktu (GPS prior'u yalnız x,y'yi
düzeltir, heading kanalı bilerek serbest bırakılmıştır, bkz.
isam2_smoother.py _HEADING_FREE_SIGMA). 20 dk'lık bir görevde jiroskop
bias'ı sınırsız birikebilir. Bu dosya FC AHRS'inin (/mavros/imu/data
orientation, pusula+jiroskop+ivmeölçer füzyonu) periyodik heading prior'u
olarak eklenmesini (add_heading) sınar.

Kapsam:
    1) add_heading() çekirdek metodu — smoother düzeyi (yön düzeltir,
       x,y'yi bozmaz, geçersiz girdide hata verir)
    2) Jiroskop bias senaryosu: düzeltme KAPALIYKEN kayma birikir
    3) Aynı senaryo: düzeltme AÇIKKEN kayma bastırılır
    4) Geriye uyumluluk: psi=None ile eski çağrı biçimi davranışı DEĞİŞTİRMEZ

Çalıştır: pytest prototype/tests/test_isam2_heading.py -v
"""

from __future__ import annotations

import numpy as np
import pytest

pytest.importorskip("gtsam", reason="gtsam yok — iSAM2 füzyon testleri atlanır")

import gtsam  # noqa: E402

from prototype.fusion.isam2_smoother import (  # noqa: E402
    ISAM2Smoother,
    ISAM2SmootherConfig,
)
from prototype.fusion.pipeline import (  # noqa: E402
    FusionPipeline,
    FusionPipelineConfig,
)


# --------------------------------------------------------------------------- #
# 1) Çekirdek add_heading() — smoother düzeyi
# --------------------------------------------------------------------------- #


def test_add_heading_dogru_yona_cekiyor() -> None:
    """Sapmış bir tahmine heading prior'u eklenince psi gerçeğe yaklaşmalı."""
    sm = ISAM2Smoother(ISAM2SmootherConfig(odom_sigma_psi=0.5))  # gevşek odom
    sm.initialize(gtsam.Pose2(0.0, 0.0, 0.0))
    # Yanlış yönde (0.5 rad) bir odometri adımı — gerçekte 0.0 olmalıydı.
    sm.add_odometry(gtsam.Pose2(1.0, 0.0, 0.5))
    sm.update()
    assert sm.current_pose().theta() == pytest.approx(0.5, abs=0.05)

    # Doğru yönü (0.0 rad) heading prior'u olarak ver, sıkı sigma ile.
    sm.add_heading(sm.latest_key, 0.0, sigma_psi=0.01)
    sm.update()
    assert abs(sm.current_pose().theta()) < 0.05, (
        f"heading prior'u yönü düzeltmedi: {sm.current_pose().theta():.3f} rad"
    )


def test_add_heading_xy_yi_bozmuyor() -> None:
    """Heading prior'unun x,y kanalı serbest (huge sigma) — konumu kaydırmamalı."""
    sm = ISAM2Smoother()
    sm.initialize(gtsam.Pose2(0.0, 0.0, 0.0))
    sm.add_odometry(gtsam.Pose2(5.0, 3.0, 0.1))
    sm.update()
    x0, y0 = sm.current_pose().x(), sm.current_pose().y()

    sm.add_heading(sm.latest_key, 0.1, sigma_psi=0.01)
    sm.update()
    x1, y1 = sm.current_pose().x(), sm.current_pose().y()
    assert abs(x1 - x0) < 1e-6 and abs(y1 - y0) < 1e-6, (
        "heading prior'u x,y'yi kaydırdı — x,y kanalı serbest bırakılmalıydı"
    )


def test_add_heading_gecersiz_key_hata_verir() -> None:
    sm = ISAM2Smoother()
    sm.initialize(gtsam.Pose2(0.0, 0.0, 0.0))
    with pytest.raises(ValueError):
        sm.add_heading(5, 0.0)  # henüz var olmayan key


def test_add_heading_negatif_sigma_hata_verir() -> None:
    sm = ISAM2Smoother()
    sm.initialize(gtsam.Pose2(0.0, 0.0, 0.0))
    with pytest.raises(ValueError):
        sm.add_heading(0, 0.0, sigma_psi=-0.1)


# --------------------------------------------------------------------------- #
# 2) Jiroskop bias senaryosu — pipeline düzeyi
# --------------------------------------------------------------------------- #


def _run_bias_scenario(
    *,
    heading_correction: bool,
    duration_s: float = 60.0,
    dt: float = 0.02,
    gyro_bias: float = 0.01,
) -> np.ndarray:
    """Sabit gyro bias'ı + (opsiyonel) doğru mutlak yön örnekleriyle koştur.

    Gerçek yön SIFIRDA sabit (tekne düz gidiyor); ölçülen gyro'da kalibre
    edilmemiş sabit bir bias var (gerçekçi — jiroskop bias'ı sıcaklıkla/
    zamanla kayar). heading_correction=True ise FC AHRS'i (gerçek psi=0,
    küçük gürültüyle) her IMU adımında veriliyor (yalnız keyframe kadansında
    kullanılır — bkz. pipeline.py on_imu docstring'i).
    """
    rng = np.random.default_rng(0)
    cfg = FusionPipelineConfig(
        heading_correction_enabled=heading_correction,
        heading_sigma_psi=0.05,
    )
    fp = FusionPipeline(cfg)
    fp.on_gps(36.85, 28.27)  # origin

    n = int(duration_s / dt)
    for k in range(n):
        t = (k + 1) * dt
        fp.on_velocity(1.0, 0.0)  # düz +x, 1 m/s
        psi_ahrs = None
        if heading_correction:
            psi_ahrs = float(rng.normal(0.0, 0.02))  # gerçek=0 + küçük gürültü
        fp.on_imu(t, gyro_bias, psi=psi_ahrs)  # gerçek omega=0, ölçülen=bias

    return fp.all_xy_psi()


def test_heading_duzeltmesi_kapaliyken_gyro_bias_sinirsiz_kayar() -> None:
    """Regresyon: eski davranış — düzeltme yokken bias birikir (~0.01×60=0.6 rad)."""
    traj = _run_bias_scenario(heading_correction=False, duration_s=60.0)
    final_psi = abs(traj[-1, 2])
    assert final_psi > 0.3, (
        f"beklenen davranış: düzeltmesiz kayma birikir, ama psi={final_psi:.3f} rad"
    )


def test_heading_duzeltmesi_acikken_gyro_bias_bastirilir() -> None:
    """FC AHRS örnekleri gyro bias'ının psi'ye etkisini bastırmalı."""
    traj = _run_bias_scenario(heading_correction=True, duration_s=60.0)
    final_psi = abs(traj[-1, 2])
    assert final_psi < 0.1, (
        f"heading düzeltmesi AÇIKKEN bile psi {final_psi:.3f} rad'a kaydı"
    )


def test_heading_orneği_gelmezse_eski_davranis_korunur() -> None:
    """psi hiç verilmezse (None) — heading_correction_enabled=True olsa BİLE
    eski (yalnız-gyro) davranıştan FARKLI OLMAMALI (geriye uyumluluk)."""
    without_flag = _run_bias_scenario(heading_correction=False, duration_s=10.0)

    cfg = FusionPipelineConfig(heading_correction_enabled=True)
    fp = FusionPipeline(cfg)
    fp.on_gps(36.85, 28.27)
    n = int(10.0 / 0.02)
    for k in range(n):
        t = (k + 1) * 0.02
        fp.on_velocity(1.0, 0.0)
        fp.on_imu(t, 0.01)  # psi verilmiyor (varsayılan None)

    assert fp.current_pose()[2] == pytest.approx(without_flag[-1, 2], abs=1e-9)
