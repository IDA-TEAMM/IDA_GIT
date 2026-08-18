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

⚠ İLK MARJ SEÇİMİ (RRT*'ın hard payı, 0.5 m) ÇARPMA ÇIKARDI (18.08 gece,
tam parkur 12-koşum A/B): 4 bilinen 370 s tıkanmadan 1'i −0.21 m payla
(ÇARPMA) kurtuldu, ÇARPMA 0/12→2/12. O yüzden GEÇİCİ olarak kapatılmıştı —
git geçmişi bu turun izidir.

✅ **İKİNCİ TUR — KÖK NEDENİN GERÇEĞİ BULUNDU, ARTIK VARSAYILAN AÇIK.**
RRT* bu engel sınıfının (huni paylı kenar duba) `margin`'ini hiç GÖRMÜYORDU
(`rrt_star.py` düz `safety_margin` kullanıyordu) — "güvenli" dediği ara
noktalar bile MPPI'nin huni payının içinde kalıyordu. Düzeltme iki ayaklı:
  1) `rrt_star.py`: RRT* artık engelin KENDİ payını görüyor (yalnız
     BÜYÜTÜR, asla küçültmez — çarpma riski yok).
  2) Kurtarma payı RRT*'ın hard payına DEĞİL, MPPI'nin KENDİ
     kanıtlanmış-güvenli global payına (1.0 m) iner.
Tam parkur A/B'sinde (12+12 koşum, üretim K/T) ölçüldü:
  · ZOR:    %50.0 → **%84.4**, en uzun donma 370 s → 135 s, 4 bilinen
            tıkanmanın 4'ü de çözüldü.
  · NORMAL: %79.2 → **%91.7**, en uzun donma 95 s → 19 s.
  · ÇARPMA: 0/12 → 0/12 (İKİSİNDE DE) — kazanç çarpmasız.
`PlanningPipelineConfig.stuck_recovery_enabled` varsayılanı artık `True`.

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
    # MPPI'nin KENDİ kanıtlanmış-güvenli global payına inmeli (RRT*'ın hard
    # payına DEĞİL — bkz. `_mppi_icin_engeller()`'ın 18.08 gece notu: RRT*
    # payına inmek tam parkurda ÇARPMA ölçtürmüştü).
    saat.ilerlet(ufuk_s * 0.6)
    pipe.compute_control()
    assert pipe._kurtarma_aktif is True
    assert pipe._kurtarma_sayaci == 1
    kirpik = pipe._mppi_icin_engeller()[0].margin
    assert kirpik == pipe._base_mppi_cfg.obstacle_margin
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


def test_tek_direk_tuzaginda_kurtarma_ACIKKEN_kurtuluyor_CARPMADAN(
    bounds: Bounds,
) -> None:
    """18.08 gece İKİNCİ tur: pay RRT*'ın hard payına (0.5 m) DEĞİL,
    MPPI'nin kendi kanıtlanmış-güvenli global payına (1.0 m) iner — bu
    yüzden kaçış daha YAVAŞ (~60 s, ilk denemenin ~24 s'sinden fazla) ama
    ÇARPMASIZ. Gövde payı boyunca hiç negatif olmamalı (`en_kucuk_pay`
    kontrolü) — bu, ilk denemenin (0.5 m) −0.21 m'lik ÇARPMASINI yakalayan
    tam da bu testin eksik bıraktığı kontroldü.
    """
    pipe, dyn, state = _tek_direk_sahnesi(bounds, stuck_recovery_enabled=True)
    dt = 0.1
    en_kucuk_pay = 99.0
    HULL_YARI_GENISLIK_M = 0.39      # ölçülmüş gövde eni / 2 (09.08)
    for _ in range(900):                       # 90 s
        pipe.set_state(state)
        u = pipe.compute_control()
        if u is None or not np.all(np.isfinite(u)):
            break
        for _ in range(2):
            state = dyn.step_rk4(state, u, dt / 2)
        pay = math.hypot(state[0] - 6.0, state[1] - 1.0) - 0.15 - HULL_YARI_GENISLIK_M
        en_kucuk_pay = min(en_kucuk_pay, pay)
    assert state[0] > 15.0, (
        f"kurtarma açıkken de sıkışık kaldı (x={state[0]:.2f}) — mekanizma "
        "bu geometride işe yaramıyor"
    )
    assert en_kucuk_pay > 0.0, (
        f"kurtardı AMA çarparak kurtardı (en küçük gövde payı {en_kucuk_pay:.3f} m) "
        "— tam da 18.08 gecenin ilk denemesindeki hata"
    )
    assert pipe._kurtarma_sayaci >= 1
