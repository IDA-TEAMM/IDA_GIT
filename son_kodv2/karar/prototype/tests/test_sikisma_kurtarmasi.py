"""
Girdap İDA — F-P.11 SIKIŞMA KURTARMASI testleri (rclpy bağımsız).

🔴 NEDEN VAR (18.08.2026, GIRDAP_DURUM §1.24): `kapi_orani.py --zor` ile
ölçülen "370 s donma" arızasının kök nedeni bulundu — tekne, HEDEFTE OLMAYAN
ama zaten "kenar" sınıflanmış bir kapı direğinin huni payı çemberinin
(`gate_post_margin_m`=1.4 m, kasıtlı olarak `mppi_obstacle_margin`=1.0 m'den
BÜYÜK) İÇİNDEN geçen bir RRT* yolunu MPPI izleyemiyor: global planlayıcı bu
geometriyi KENDİ hard payıyla (0.5 m) güvenli sayıp yoldan geçiriyor, ama
MPPI'nin daha geniş yumuşak payı aynı segmenti sürekli ceza bölgesi yapıyor —
iki katman aynı geometride ANLAŞMIYOR ve statik bir denge (ne ilerleme ne
geri çekilme) oluşuyor. Gerçek koşumda 900 s'nin TAMAMI aynı pozisyonda
donuyor.

⚠ İLK DENEME (σ_u/λ'yı geçici yükseltmek — "keşif eksikliği" hipotezi)
ÖLÇÜLDÜ VE ETKİSİZ ÇIKTI: aynı sahnede 40 s'de 0.05 m'den fazla ilerleme
olmadı. Kök neden keşif değil, katmanlar-arası PAY ÇELİŞKİSİ olduğu için asıl
düzeltme `PlanningPipeline._mppi_icin_engeller()`'da — engel paylarını
RRT*'ın KENDİ `safety_margin`'ine geçici indirmek. Bu dosyanın önceki
sürümündeki σ/λ testleri bu yüzden değiştirildi; git geçmişi ölçümün izidir.

🔴 **VARSAYILAN KAPALI — bu ikinci deneme de ÇARPMA riski taşıyor (18.08
gece, tam parkur 12-koşum A/B, `kapi_orani.py --zor --kurtarma-ac`).**
İzole senaryoda (aşağıdaki testler) çalışıyor, ama gerçek parkurda RRT*'ın
"hard" payına inmek MPPI'nin gerçek izleme hatasını hesaba katmıyor:
  · 4 bilinen 370 s tıkanmadan yalnız 1'i kurtuldu — ama gövde payı
    **−0.21 m (ÇARPMA)** ile.
  · 2 başka koşumda payı 0.94→0.04 m ve 0.55→0.05 m'ye düşürdü (temasa
    neredeyse sıfır).
  · Toplam ÇARPMA (12 zor koşum): kapalı 0/12 → açık 2/12.
  · Diğer 3 tıkanma hiç değişmedi.
"Bazen tıkanma"yı "bazen çarpma"yla değiştirmek kabul edilemez bir takas —
370 s donma sorunu bu yüzden **HÂLÂ AÇIK**. Mekanizma + testler burada
KANITLANMAMIŞ altyapı olarak duruyor (`gate_follower.py`daki
`_arada_duba_kontrolu` ile aynı durum) — `PlanningPipelineConfig.
stuck_recovery_enabled` varsayılanı `False`; testler mekanizmayı bilerek
AÇIK doğruluyor (izole senaryoda çalıştığını kanıtlamak için), üretim
tarafı ona hiç dokunmuyor.

Kapsam:
    1) Mekanizma birim testi: durgunluk → kurtarma tetiklenir/söner (sahte
       saat, gerçek MPPI koşturulmadan — hızlı, deterministik).
    2) Gerçek senaryo regresyon testi: aynı geometri (izole edilmiş TEK
       direk + uzak hedef) kurtarma KAPALIYKEN donuyor, AÇIKKEN kurtuluyor
       (ama bu izolasyon tam parkurdaki çarpma riskini YAKALAMIYOR — bkz.
       yukarıdaki not).
"""

from __future__ import annotations

import math

import numpy as np
import pytest

from prototype.dynamics.catamaran import CatamaranDynamics
from prototype.planning.pipeline import PlanningPipeline, PlanningPipelineConfig
from prototype.planning.rrt_star import Bounds, CircleObstacle


class _SahteSaat:
    """Elle sürülen tek yönlü saat — durgunluk süresini test kontrol eder."""

    def __init__(self) -> None:
        self.t = 1000.0

    def __call__(self) -> float:
        return self.t

    def ilerlet(self, s: float) -> None:
        self.t += s


def _fast_cfg(**kwargs) -> PlanningPipelineConfig:
    return PlanningPipelineConfig(mppi_K=200, mppi_T=30, **kwargs)


@pytest.fixture
def bounds() -> Bounds:
    return Bounds(-20.0, 60.0, -20.0, 20.0)


# --------------------------------------------------------------------------- #
# 1) Mekanizma birim testi
# --------------------------------------------------------------------------- #


def test_durgunluk_ufuk_kadar_surunce_kurtarma_tetiklenir(bounds: Bounds) -> None:
    saat = _SahteSaat()
    dyn = CatamaranDynamics()
    pipe = PlanningPipeline(bounds, _fast_cfg(stuck_recovery_enabled=True), dynamics=dyn, saat=saat)
    pipe.set_obstacles([CircleObstacle(15.0, 0.0, 0.15, margin=1.4)])
    pipe.set_waypoints([(30.0, 0.0)])
    pipe.set_mission_state("PARKUR1")
    pipe.set_state(np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0]))
    assert pipe._mppi is not None

    ufuk_s = pipe.cfg.mppi_T * pipe.cfg.mppi_dt

    # Ufuk DOLMADAN önce: hız hâlâ sıfır ama kurtarma henüz tetiklenmemeli.
    pipe.compute_control()
    saat.ilerlet(ufuk_s * 0.5)
    pipe.compute_control()
    assert pipe._kurtarma_aktif is False
    assert pipe._mppi_icin_engeller()[0].margin == 1.4

    # Ufuk kadar (ve biraz fazlası) durgun kalınca tetiklenmeli; engelin payı
    # RRT*'ın kendi hard payına inmeli.
    saat.ilerlet(ufuk_s * 0.6)
    pipe.compute_control()
    assert pipe._kurtarma_aktif is True
    assert pipe._kurtarma_sayaci == 1
    kirpik = pipe._mppi_icin_engeller()[0].margin
    assert kirpik == pipe._rrt_cfg.safety_margin
    assert kirpik < 1.4


def test_hiz_normale_donunce_kurtarma_kendiliginden_soner(bounds: Bounds) -> None:
    saat = _SahteSaat()
    dyn = CatamaranDynamics()
    pipe = PlanningPipeline(bounds, _fast_cfg(stuck_recovery_enabled=True), dynamics=dyn, saat=saat)
    pipe.set_obstacles([CircleObstacle(15.0, 0.0, 0.15, margin=1.4)])
    pipe.set_waypoints([(30.0, 0.0)])
    pipe.set_mission_state("PARKUR1")
    pipe.set_state(np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0]))

    ufuk_s = pipe.cfg.mppi_T * pipe.cfg.mppi_dt
    pipe.compute_control()
    saat.ilerlet(ufuk_s * 1.1)
    pipe.compute_control()
    assert pipe._kurtarma_aktif is True

    # Hız eşiğin ÜSTÜNE çıkınca hemen DEĞİL — TEK bir yüksek-hız karesi
    # histerezisi (bilerek) kapatmamalı, tek bir "tekme" gerçek kurtuluş
    # değildir.
    esik = pipe._erisilebilir_hiz() * 0.10
    pipe.set_state(np.array([5.0, 0.0, 0.0, esik * 2.0, 0.0, 0.0]))
    pipe.compute_control()
    assert pipe._kurtarma_aktif is True

    # Ufuk kadar KESİNTİSİZ yüksek hız sürünce kendiliğinden kapanmalı ve
    # engel payı eski (huni) değerine dönmeli.
    saat.ilerlet(ufuk_s * 1.1)
    pipe.compute_control()
    assert pipe._kurtarma_aktif is False
    assert pipe._mppi_icin_engeller()[0].margin == 1.4


def test_stuck_recovery_kapaliyken_davranis_degismez(bounds: Bounds) -> None:
    """`stuck_recovery_enabled=False` → eski davranış BİREBİR (acil kapatma)."""
    saat = _SahteSaat()
    dyn = CatamaranDynamics()
    pipe = PlanningPipeline(
        bounds, _fast_cfg(stuck_recovery_enabled=False), dynamics=dyn, saat=saat
    )
    pipe.set_obstacles([CircleObstacle(15.0, 0.0, 0.15, margin=1.4)])
    pipe.set_waypoints([(30.0, 0.0)])
    pipe.set_mission_state("PARKUR1")
    pipe.set_state(np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0]))

    ufuk_s = pipe.cfg.mppi_T * pipe.cfg.mppi_dt
    for _ in range(5):
        pipe.compute_control()
        saat.ilerlet(ufuk_s)

    assert pipe._kurtarma_aktif is False
    assert pipe._kurtarma_sayaci == 0
    assert pipe._mppi_icin_engeller()[0].margin == 1.4


# --------------------------------------------------------------------------- #
# 2) Gerçek senaryonun izole edilmiş yeniden üretimi
# --------------------------------------------------------------------------- #


def _tek_direk_sahnesi(bounds: Bounds, *, stuck_recovery_enabled: bool):
    """18.08 bulgusunun asgari izolasyonu: tekne, HEDEFTE OLMAYAN ama huni
    payı (1.4 m) uygulanan TEK bir direğe ~1.5 m'den geçmeye çalışıyor,
    hedef direğin ÖTESİNDE. Gerçek `kapi_orani.py` sahnesinden farklı olarak
    kapı seçimi/algı katmanı yok — doğrudan `CircleObstacle(margin=1.4)`
    (gerçek `_huni_payi`/`HUNI_TAVANI` ile aynı sayı) veriliyor, yani MPPI'nin
    gördüğü engel geometrisi BİREBİR aynı, yalnız çevresi sadeleştirildi.
    """
    dyn = CatamaranDynamics()
    pipe = PlanningPipeline(
        bounds,
        PlanningPipelineConfig(
            mppi_K=1000, mppi_T=30,
            stuck_recovery_enabled=stuck_recovery_enabled,
        ),
        dynamics=dyn,
    )
    pipe.set_mission_state("PARKUR1")
    # Direk (6,1) — gerçek kosum() sahnesindeki kapı-0 sol direği; margin
    # 1.4 m gerçek `HUNI_TAVANI`/`gate_post_margin_m` sabiti.
    pipe.set_obstacles([CircleObstacle(6.0, 1.0, 0.15, margin=1.4)])
    hedef = (20.9, -3.0)          # gerçek koşumda kilitlenen kapının nişanı
    pipe.set_waypoints([hedef])
    state = np.array([4.8, 1.88, math.radians(-25.0), 0.0, 0.0, 0.0])
    pipe.set_state(state)
    return pipe, dyn, state


def test_tek_direk_tuzaginda_kurtarma_KAPALIYKEN_donuyor(bounds: Bounds) -> None:
    """Regresyon: kurtarma olmadan tekne gerçekten (ölçülenle tutarlı
    şekilde) bu geometride sıkışıyor — testin KENDİSİ de yanlış pozitif
    vermesin diye bu kontrol burada."""
    pipe, dyn, state = _tek_direk_sahnesi(bounds, stuck_recovery_enabled=False)
    dt = 0.1
    for _ in range(400):                       # 40 s
        pipe.set_state(state)
        u = pipe.compute_control()
        if u is None or not np.all(np.isfinite(u)):
            break
        for _ in range(2):
            state = dyn.step_rk4(state, u, dt / 2)
    # 40 s'de x hâlâ direğin (6.0) çok gerisinde kalmalı (donmuş).
    assert state[0] < 5.5, (
        f"test kurulumu: kurtarma kapalıyken bile ilerledi (x={state[0]:.2f}) "
        "— sahne artık gerçek arızayı izole etmiyor, gerekçesi güncellenmeli"
    )


def test_tek_direk_tuzaginda_kurtarma_ACIKKEN_kurtuluyor(bounds: Bounds) -> None:
    pipe, dyn, state = _tek_direk_sahnesi(bounds, stuck_recovery_enabled=True)
    dt = 0.1
    for _ in range(400):                       # 40 s
        pipe.set_state(state)
        u = pipe.compute_control()
        if u is None or not np.all(np.isfinite(u)):
            break
        for _ in range(2):
            state = dyn.step_rk4(state, u, dt / 2)
    assert state[0] > 15.0, (
        f"kurtarma açıkken de sıkışık kaldı (x={state[0]:.2f}) — mekanizma "
        "bu geometride işe yaramıyor"
    )
    assert pipe._kurtarma_sayaci >= 1
