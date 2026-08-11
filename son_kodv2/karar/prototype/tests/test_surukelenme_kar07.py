"""KAR-07: sabit dururken biriken "yol" — poz gürültüsünün integrali.

Kaptanın bulgusu (`hatalar/karar.md` KAR-07): `session_19700101_020215`'te araç
33 × 27 m'lik bir kutunun içinde kalırken füzyon **4.765,8 m yol** biriktirdi.
Bu gerçek hareket değil, poz gürültüsünün integrali. Türev alan her şey bozulur:
hız tahmini, heading, kapı geçiş doğrulaması.

Kaptanın önerisi #2 aynen uygulandı: *"sabit dururken biriken yol < 5 m/dakika
olmalı. Ölçmesi ucuz, çok şey yakalar."*

🔴 ÖLÇÜM SIRASINDA ÇIKAN BEKLENMEDİK SONUÇ — sigma büyüdükçe sürüklenme AZALIYOR:

    gps_sigma  yol (m/dk)   (araç SABİT, 60 s, 3 tohum ortalaması)
         0.05        3.84
         0.50        2.71
         2.50        1.11
         3.80        0.88

Sezgiye ters değil aslında: büyük sigma = GPS'e az güven = daha çok yumuşatma.
Yani **en kötü hâl RTK sigmasıdır** (0.05) — filtre her gürültülü fix'i kovalar.
Bu, "RTK aldık, artık sorun yok" sezgisinin tersini söylüyor.

🔴 ASIL TEHLİKE — bildirilen sigma ile GERÇEK gürültü ayrışırsa:

    bildirilen  gerçek   yol (m/dk)
          0.05    0.05         3.84
          0.05    0.50        21.19
          0.05    2.50      104.30      ← 94×
          2.50    2.50         1.11

Bu, KAR-06'nın (sıfır kovaryans = sonsuz güven) genel hâli ve
`gps_sigma_by_status` tablosunun neden kritik olduğunun sayısal kanıtı:
tek-nokta çözümünü RTK sigmasıyla bildirmek tahmini yok ediyor.
"""

from __future__ import annotations

import numpy as np
import pytest

gtsam = pytest.importorskip("gtsam", reason="gtsam yok — çekirdek füzyon testi")

from prototype.fusion.pipeline import (                    # noqa: E402
    FusionPipeline,
    FusionPipelineConfig,
)

LAT0, LON0 = 36.85, 28.27          # Marmaris civarı; kesin değer önemsiz
DT = 0.02                          # 50 Hz IMU
SURE_S = 60.0                      # tam 1 dakika → eşik doğrudan m/dk
GPS_STRIDE = 50                    # 1 Hz GPS
ESIK_M_DK = 5.0                    # kaptanın ölçütü


def _sabit_dururken_biriken_yol(
    bildirilen_sigma: float, gercek_gurultu: float, tohum: int = 0
) -> float:
    """Araç HİÇ hareket etmezken füzyonun biriktirdiği yol (m/dakika).

    `on_velocity(0,0)` + `on_imu(wz=0)` → gerçek hareket sıfır. GPS'e
    `gercek_gurultu` σ'lı gürültü enjekte edilir, füzyona ise
    `bildirilen_sigma` söylenir. İkisini ayrı tutmak bilinçli: sahadaki
    asıl arıza, bildirilen belirsizliğin gerçeğinden küçük olmasıdır.
    """
    rng = np.random.default_rng(tohum)
    fp = FusionPipeline(FusionPipelineConfig(gps_sigma_xy=bildirilen_sigma))
    fp.on_gps(LAT0, LON0)                      # origin
    for k in range(int(SURE_S / DT)):
        fp.on_velocity(0.0, 0.0)               # DURUYOR
        fp.on_imu((k + 1) * DT, 0.0)
        if (k + 1) % GPS_STRIDE == 0:
            lat, lon = fp.enu_to_latlon(
                rng.normal(0.0, gercek_gurultu),
                rng.normal(0.0, gercek_gurultu),
            )
            fp.on_gps(lat, lon)
    xy = fp.all_xy_psi()[:, :2]
    return float(np.sum(np.linalg.norm(np.diff(xy, axis=0), axis=1)))


def test_KAR07_RTK_sigmasinda_surukelenme_esigin_altinda() -> None:
    """EN KÖTÜ dürüst hâl: RTK sigması (0.05) — filtre her fix'i kovalar.

    Ölçülen taban 3,84 m/dk; eşik 5,0. Pay dar (%25) ve bu KASITLI: gevşek
    bir eşik regresyonu yakalamaz. Bu test kırmızıya dönerse füzyonda
    gürültü bastırma bozulmuş demektir.
    """
    yol = _sabit_dururken_biriken_yol(0.05, 0.05)
    assert yol < ESIK_M_DK, (
        f"sabit dururken {yol:.2f} m/dk birikti (esik {ESIK_M_DK}) — "
        "poz gurultusu bastirilamıyor (KAR-07)"
    )


def test_KAR07_tek_nokta_sigmasinda_da_esigin_altinda() -> None:
    """Dürüst tek-nokta çözümü (2.5) — daha çok yumuşatma, daha az sürüklenme."""
    yol = _sabit_dururken_biriken_yol(2.5, 2.5)
    assert yol < ESIK_M_DK, f"{yol:.2f} m/dk (esik {ESIK_M_DK})"


def test_KAR07_metrik_GERCEKTEN_hassas() -> None:
    """🔑 Metodolojik olarak en önemli test: metrik arızayı YAKALIYOR mu?

    Kırmızıya dönemeyen bir regresyon testi hiçbir şey doğrulamaz. Burada
    kasıtlı olarak bozuk bir yapılandırma kuruluyor — tek-nokta gürültüsü
    (2.5 m) RTK sigmasıyla (0.05) bildiriliyor, yani KAR-06'nın genel hâli —
    ve metriğin eşiği KATBEKAT aştığı gösteriliyor.

    Ölçülen: 104,30 m/dk (dürüst hâlin 94 katı). Bu test yeşil kalırken
    yukarıdakiler de yeşilse, eşik gerçekten ayırt ediyor demektir.
    """
    yol = _sabit_dururken_biriken_yol(0.05, 2.5)
    assert yol > 10 * ESIK_M_DK, (
        f"bozuk yapilandirmada yalnizca {yol:.2f} m/dk birikti — metrik "
        "arizayi yakalamiyor, esik anlamsiz"
    )
