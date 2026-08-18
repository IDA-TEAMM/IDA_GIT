"""F-S.16 KORİDOR TERİMİ — parkur dışına çıkma kuvveti (GIRDAP_DURUM §1.51).

🔴 NEDEN VAR: `w_boundary` duvarının kutusu `pipeline._etkin_sinir()` ile
"tekne/hedef ± 30 m" kuruluyor (F-S.17) → 12 m'lik kenar duba koridorunda
**hiç ateşlenemiyor**. Sanal gölde ölçüldü (§1.50): dört koşumun DÖRDÜ de
koridordan çıktı (dalgasız koşumda bile), koşum başına ortalama 9 puan
(şartname s.24-25: her çıkış 6 puan, toplam 54 puan).

Bu dosya terimi **ve emniyet özelliklerini** donduruyor: kapalıyken eski
davranış birebir · içeride bedel sıfır · dar/tek kapıda susar · kapı
geçirgenliğini bozmaz.
"""
from __future__ import annotations

import numpy as np
import pytest

from prototype.dynamics.catamaran import CatamaranDynamics
from prototype.planning.mppi import (
    _GOVDE_YARI_GENISLIK_M,
    MPPIConfig,
    MPPIController,
)
from prototype.planning.rrt_star import Bounds

SINIR = Bounds(-50.0, 50.0, -50.0, 50.0)
# 12 m açıklıklı üç kapı, kurs +y boyunca (sanal gölün geometrisi)
OMURGA = [((0.0, 6.0), 6.0), ((0.0, 10.0), 6.0), ((0.0, 14.0), 6.0)]


def _kontrolcu(w_koridor: float, omurga=OMURGA) -> MPPIController:
    cfg = MPPIConfig(K=32, T=10, backend="numpy", w_koridor=w_koridor)
    m = MPPIController(CatamaranDynamics(), SINIR, [], cfg)
    m.set_reference([(0.0, 6.0), (0.0, 14.0)])
    m.set_koridor(omurga)
    return m


def _izler(yanal_disari: float):
    """İki yörünge: (0) koridor ortası, (1) `yanal_disari` m yanda."""
    traj = np.zeros((2, 11, 6))
    traj[1, :, 0] = yanal_disari
    traj[:, :, 1] = np.linspace(6.0, 14.0, 11)
    return traj, np.zeros((2, 10, 2))


def test_kapaliyken_hicbir_bedel_EKLEMEZ():
    """`w_koridor=0` → eski davranış BİREBİR (geri uyumluluk kapısı)."""
    traj, U = _izler(9.0)
    kapali = _kontrolcu(0.0)._trajectory_cost(traj, U)
    # aynı config, koridor HİÇ kurulmamış
    cfg = MPPIConfig(K=32, T=10, backend="numpy")
    ham = MPPIController(CatamaranDynamics(), SINIR, [], cfg)
    ham.set_reference([(0.0, 6.0), (0.0, 14.0)])
    assert np.allclose(kapali, ham._trajectory_cost(traj, U))


def test_koridor_ICINDE_bedel_sifir():
    """İçeride tek yönlü kısıt susar — yoksa terim gizli bir takip cezası olur."""
    traj, U = _izler(2.0)                     # 2 m yanal, 5,6 m sınırın içinde
    acik = _kontrolcu(50.0)._trajectory_cost(traj, U)
    kapali = _kontrolcu(0.0)._trajectory_cost(traj, U)
    assert np.allclose(acik, kapali)


def test_koridor_DISINDA_taşmanın_karesi_kadar_ceza():
    """Ceza tam `w · Σ(taşma²)` — büyüklük mertebesi kayarsa yakalanır."""
    w, yanal = 50.0, 9.0
    traj, U = _izler(yanal)
    fark = (_kontrolcu(w)._trajectory_cost(traj, U)
            - _kontrolcu(0.0)._trajectory_cost(traj, U))
    tasma = yanal - (6.0 - _GOVDE_YARI_GENISLIK_M)
    beklenen = w * (tasma ** 2) * 11          # T+1 adım
    assert fark[0] == pytest.approx(0.0, abs=1e-6)
    assert fark[1] == pytest.approx(beklenen, rel=1e-6)


def test_yari_genislikten_GOVDE_dusuluyor():
    """Koridorda kalması gereken şey nokta değil GÖVDE (ölçülmüş 0,785 m)."""
    m = _kontrolcu(50.0)
    assert m._kor_h.tolist() == [6.0 - _GOVDE_YARI_GENISLIK_M] * 2


def test_TEK_kapida_terim_SUSAR():
    """Tek noktadan koridor tanımlanamaz — yönü yoktur."""
    assert _kontrolcu(50.0, [((0.0, 6.0), 6.0)])._kor_h is None


def test_GOVDEDEN_DAR_kapi_segmenti_ATILIR():
    """Negatif yarıçaplı boru her yeri 'dışarı' ilan eder — segment atılmalı."""
    dar = [((0.0, 0.0), 0.30), ((0.0, 4.0), 0.30)]
    assert _kontrolcu(50.0, dar)._kor_h is None


def test_ZIGZAG_kapida_boru_BIRLESIMI_iceride_sayar():
    """±5 m zigzag: bir borunun içinde olmak koridorun içinde olmaktır.

    Gerçek parkur zigzaglı (§0.17b). Segmentler tek tek dar olsa da birleşim
    doğru davranmalı; aksi hâlde kapı ağzında sahte ceza doğar ve terim kapı
    geçirgenliğini bozar (ölçülmüş tuzak: `obstacle_margin` 1,5 → geçit kapanır).
    """
    zigzag = [((0.0, 6.0), 6.0), ((5.0, 10.0), 6.0), ((0.0, 14.0), 6.0)]
    m = _kontrolcu(50.0, zigzag)
    traj = np.zeros((1, 11, 6))
    traj[0, :, 0] = 5.0                       # ikinci kapının TAM ORTASI
    traj[0, :, 1] = np.linspace(6.0, 14.0, 11)
    U = np.zeros((1, 10, 2))
    kapali = _kontrolcu(0.0, zigzag)._trajectory_cost(traj, U)
    assert np.allclose(m._trajectory_cost(traj, U), kapali), \
        "kapı ortasından geçen yörünge cezalanıyor — kapı geçirgenliği bozuldu"


def test_pipeline_koridoru_MPPIye_GECIRIR():
    """Boru hattı sözleşmesi: `set_koridor` yeniden kurulumda da yaşamalı."""
    from prototype.planning.pipeline import (
        PlanningPipeline,
        PlanningPipelineConfig,
    )
    pipe = PlanningPipeline(
        SINIR, PlanningPipelineConfig(mppi_K=32, mppi_T=10),
        dynamics=CatamaranDynamics(),
    )
    pipe.set_mission_state("PARKUR1")
    pipe.set_state((0.0, 0.0, np.pi / 2, 0.0, 0.0, 0.0))
    pipe.set_reference_direct(0.0, 14.0)
    pipe.set_koridor(OMURGA)
    assert pipe._mppi is not None
    assert pipe._mppi._kor_h is not None, "koridor MPPI'ye geçmedi"
    # Ağırlık: yeni sayı icat edilmedi → parkur profilinin w_obstacle'ı
    assert pipe._mppi.cfg.w_koridor == pipe._mppi.cfg.w_obstacle
