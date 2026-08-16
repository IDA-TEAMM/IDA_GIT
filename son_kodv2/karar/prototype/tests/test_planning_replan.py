"""
Girdap İDA — RRT* replan çağrı sözleşmesi testleri (F10.1 / F10.2).

test_planning_pipeline.py gtsam gerektirir (FusionPipeline importu); bu dosya
yalnız PlanningPipeline + rrt_star kullanır → gtsam'sız ortamda da koşar.

Kapsam:
    F10.1 — start/goal engel payı içindeyken replan istisna fırlatmamalı
            (önceki davranış: ValueError → rclpy callback → planning_node
            görev ortasında ölür).
    F10.2 — start/goal statik bounds dışında (negatif çeyrek) olsa da plan
            üretilmeli (bounds start+goal zarfıyla genişletilir).

Çalıştır: pytest prototype/tests/test_planning_replan.py -v
"""

from __future__ import annotations

import math

import numpy as np

from prototype.planning.pipeline import (
    PlanningPipeline,
    PlanningPipelineConfig,
)
from prototype.planning.rrt_star import Bounds, CircleObstacle


def _fast_cfg() -> PlanningPipelineConfig:
    return PlanningPipelineConfig(mppi_K=200, mppi_T=30)


def _bounds() -> Bounds:
    return Bounds(0.0, 50.0, 0.0, 50.0)


def test_replan_start_inside_obstacle_margin_does_not_raise() -> None:
    """F10.1: araç engel payının (r+safety_margin) içindeyken tetiklenen
    replan istisna FIRLATMAMALI; eski referans yol korunmalı."""
    pipe = PlanningPipeline(_bounds(), _fast_cfg())
    pipe.set_state(np.array([5.0, 5.0, 0.0, 0.0, 0.0, 0.0]))
    pipe.set_waypoints([(40.0, 40.0)])
    old_path = pipe.global_path
    assert old_path is not None
    # Engel tam aracın üstünde → start payın içinde → plan() reddeder
    pipe.set_obstacles([CircleObstacle(5.0, 5.0, 1.0)])   # istisna YOK
    assert pipe.global_path == old_path, "eski referans yol korunmalı"


def test_goal_inside_obstacle_margin_does_not_raise() -> None:
    """F10.1 (goal tarafı): hedef bir engelin payı içindeyse set_waypoints
    çökmemeli (görev callback'i de aynı ölüm zincirini tetikliyordu).

    🔴 **A1 (09.08) — BEKLENTİ DEĞİŞTİ.** Bu test eskiden `global_path is None`
    diye bitiyordu, yani "plan üretilemedi" hâlini DONDURUYORDU. O hâl sahada
    **aracın hiç kıpırdamaması** demekti: referans yoksa MPPI kurulmaz,
    `compute_control` None döner, node sıfır thrust basar. Kapalı döngüde
    ölçüldü: hedef dubanın 0,65 m halkası içindeyken **2001/2001 adım sıfır
    thrust**. Artık düz çizgi referansına düşülüyor (bkz. `_rrt_basarisiz`).
    Testin ASIL amacı — istisna sızmaması — aynen korunuyor.
    """
    pipe = PlanningPipeline(_bounds(), _fast_cfg())
    pipe.set_state(np.array([5.0, 5.0, 0.0, 0.0, 0.0, 0.0]))
    pipe.set_obstacles([CircleObstacle(40.0, 40.0, 1.0)])
    pipe.set_waypoints([(40.0, 40.0)])                    # istisna YOK
    # RRT* reddetti ama araç yine de hedefe sürülebilir olmalı:
    assert pipe.global_path == [(5.0, 5.0), (40.0, 40.0)], "düz çizgiye düşmeli"
    assert pipe.compute_control_hazir, "kontrolcü kurulmalı (sıfır thrust olmasın)"


def test_rrt_basarisizken_ESKI_hedefin_yorungesinde_KALINMAZ() -> None:
    """A1'in ikinci ayağı: "eski referansı koru" kuralı hedef DEĞİŞİNCE
    araca eski waypoint'e gitmeyi sürdürtüyordu.

    Ölçüldü (09.08): kaçık GN'li 3 kapılı sahnede araç 1. noktaya varıyor,
    görev 2. noktaya ilerliyor, yeni hedefe plan kurulamıyor ve araç 1.
    noktanın yörüngesinde kalıyordu → **1/3 GN**. Bayatlık ölçütü A3'ün
    `set_waypoints`'te kullandığı ölçütle aynı (kayma > goal_tolerance).
    """
    pipe = PlanningPipeline(_bounds(), _fast_cfg())
    pipe.set_state(np.array([5.0, 5.0, 0.0, 0.0, 0.0, 0.0]))
    pipe.set_waypoints([(20.0, 20.0)])                   # temiz plan kurulur
    assert pipe.global_path is not None
    # Şimdi YENİ hedef, ve o hedef bir engelin payı içinde → RRT* reddedecek
    pipe.set_obstacles([CircleObstacle(40.0, 40.0, 1.0)])
    pipe.set_waypoints([(40.0, 40.0)])
    varis = pipe.global_path[-1]
    assert math.hypot(varis[0] - 40.0, varis[1] - 40.0) < 1e-6, (
        f"yörünge hâlâ eski hedefe gidiyor ({varis}) — araç 2. noktaya geçemez"
    )


def test_replan_outside_static_bounds_succeeds() -> None:
    """F10.2: start/goal statik bounds'un ([0,200]²) DIŞINDA (negatif çeyrek)
    olsa da plan üretilmeli — bounds start+goal zarfıyla genişletilir."""
    pipe = PlanningPipeline(Bounds(0.0, 200.0, 0.0, 200.0), _fast_cfg())
    pipe.set_state(np.array([-20.0, -10.0, 0.0, 0.0, 0.0, 0.0]))
    pipe.set_waypoints([(-40.0, -30.0)])                  # istisna YOK
    path = pipe.global_path
    assert path is not None, "negatif çeyrekte plan üretilmeliydi"
    assert math.hypot(path[-1][0] + 40.0, path[-1][1] + 30.0) < 2.0


# ═══════════════════════════════════════════════════════════════════════════
# F-P.9 (13.08.2026) — REPLAN FRENİ
#
# 🔴 NEDEN: `planning_node` tek thread'de koşar; RRT* aynı thread'de. Jetson'da
# ölçüldü: `plan()` 100 engelle ortanca 510 ms / en kötü 1491 ms, kontrol
# bütçesi 100 ms. Bloklama İKİ yoldan vuruyor — (a) `cmd_vel` susuyor
# (ArduPilot GUIDED 3 s'de aracı durdurur, öncesinde SON komutu sürdürür =
# kör sürme), (b) düğümün KENDİ abonelikleri işlenmiyor → `_last_odom_t`
# yaşlanıyor → kendi bekçisi "poz bayat" deyip thrust'ı sıfırlıyor (sahada
# ölçüldü: "poz 2,4 s bayat" derken füzyon 50 Hz yayındaydı).
#
# 🔎 ÖNCE İÇERİK TEMELLİ FREN DENENDİ VE ÖLÇÜMLE ELENDİ (canlı LiDAR, 60 kare,
# tekne sabit): küme merkezleri kare→kare ortanca 5,2 cm ama p90 30 cm oynuyor
# ve engel sayısı 78-120 arası gidip geliyor → imza 1 m'lik ızgarada bile
# karelerin **%100'ünde** değişiyor; eşleştirmeli fark ölçütü de tol=1,0 m'de
# hâlâ %53. Sebep: kapalı alanda kümeleme duvarı her karede farklı bölüyor.
# 👉 İçerik temelli hiçbir eşik güvenilir değil; tek sağlam kaldıraç ZAMAN.
# ═══════════════════════════════════════════════════════════════════════════


class _SahteSaat:
    """Testin sürdüğü tek yönlü saat (gerçek zamana bağlı test yazmamak için).

    ⚠ Her okumada `adim` kadar ilerler. Bu şart: fren, son planın ÖLÇÜLEN
    süresinden türer; saat okuma başına hiç ilerlemezse `plan()` sıfır saniye
    sürmüş görünür ve fren (DOĞRU biçimde) devreye girmez. `adim=0.15` sahada
    ölçülen su-koşulu plan maliyetini (10-30 duba: ortanca 287-466 ms) taklit
    eder; testler bu değere değil, ondan TÜRETİLEN aralığa bakar.
    """

    def __init__(self, adim: float = 0.15) -> None:
        self.t = 1000.0
        self.adim = adim

    def __call__(self) -> float:
        simdi = self.t
        self.t += self.adim
        return simdi

    def ilerlet(self, s: float) -> None:
        self.t += s


def _fren_pipe(saat: _SahteSaat) -> PlanningPipeline:
    """Rotanın üstünde engel olan, planı kurulmuş bir boru hattı."""
    pipe = PlanningPipeline(_bounds(), _fast_cfg(), saat=saat)
    pipe.set_state(np.array([5.0, 5.0, 0.0, 0.0, 0.0, 0.0]))
    pipe.set_waypoints([(45.0, 45.0)])            # ilk plan — fren UYGULANMAZ
    return pipe


def test_FP9_engel_kaynakli_replan_kor_payi_dolmadan_KOSMAZ() -> None:
    """Ölçülmüş kural: aralık = min(katsayı × son_plan_süresi, tavan)."""
    saat = _SahteSaat()
    pipe = _fren_pipe(saat)
    kosan = pipe.replan_sayaclari[0]
    sure = pipe.son_plan_suresi_s
    assert sure is not None, "plan süresi ölçülmedi — fren girdisi yok"

    saat.ilerlet(0.01)                             # aralık daha dolmadı
    pipe.set_obstacles([CircleObstacle(25.0, 25.0, 1.0)])
    assert pipe.replan_sayaclari[0] == kosan, "fren tutmadı, RRT* erken koştu"
    assert pipe.replan_ertelendi == 1, "erteleme sayacı işlemedi"


def test_FP9_aralik_dolunca_replan_YENIDEN_KOSAR() -> None:
    """Fren geciktirir, ENGELLEMEZ — tavan sonrası rota tazelenmeli."""
    saat = _SahteSaat()
    pipe = _fren_pipe(saat)
    kosan = pipe.replan_sayaclari[0]
    saat.ilerlet(0.01)
    pipe.set_obstacles([CircleObstacle(25.0, 25.0, 1.0)])      # ertelenir
    saat.ilerlet(PlanningPipelineConfig().replan_max_interval_s + 0.1)
    pipe.set_obstacles([CircleObstacle(24.0, 24.0, 1.0)])      # artık koşmalı
    assert pipe.replan_sayaclari[0] > kosan


def test_FP9_fren_MPPI_ENGELLERINI_GECIKTIRMEZ() -> None:
    """🔴 GÜVENLİK: ertelenen yalnız GLOBAL rotadır. Kaçınma katmanı (MPPI)
    engelleri O TURDA almalı — yoksa fren, kör sürmeyi düzeltmek yerine
    körlüğü kaçınma katmanına taşımış olurdu."""
    saat = _SahteSaat()
    pipe = _fren_pipe(saat)
    saat.ilerlet(0.01)
    yeni = [CircleObstacle(25.0, 25.0, 1.0), CircleObstacle(15.0, 15.0, 1.5)]
    pipe.set_obstacles(yeni)
    assert pipe.replan_ertelendi == 1, "önkoşul: bu tur ertelenmiş olmalı"
    assert pipe._obstacles == yeni, "engeller ertelenirken MPPI'ye verilmedi"


def test_FP9_YENI_HEDEF_asla_ertelenmez() -> None:
    """Fren yalnız engel kaynaklı replan'a; hedef değişimi anında planlanır."""
    saat = _SahteSaat()
    pipe = _fren_pipe(saat)
    kosan = pipe.replan_sayaclari[0]
    saat.ilerlet(0.01)                             # aralık dolmadı
    pipe.set_waypoints([(40.0, 10.0)])             # ama hedef DEĞİŞTİ
    assert pipe.replan_sayaclari[0] > kosan, "yeni hedef frene takıldı"


def test_FP9_ILK_PLAN_frene_takilmaz() -> None:
    """Ölçüm yokken fren uygulanmaz — soğuk başlangıçta rota gecikmemeli."""
    saat = _SahteSaat()
    pipe = PlanningPipeline(_bounds(), _fast_cfg(), saat=saat)
    pipe.set_state(np.array([5.0, 5.0, 0.0, 0.0, 0.0, 0.0]))
    assert pipe.son_plan_suresi_s is None
    pipe.set_waypoints([(45.0, 45.0)])
    assert pipe.replan_sayaclari[0] >= 1, "ilk plan koşmadı"
    assert pipe.replan_ertelendi == 0


def test_FP9_katsayi_SIFIR_eski_davranisi_birebir_getirir() -> None:
    """Geri dönüş yolu: katsayı 0 → fren tamamen kapalı."""
    saat = _SahteSaat()
    cfg = PlanningPipelineConfig(
        mppi_K=200, mppi_T=30, replan_bosluk_katsayisi=0.0
    )
    pipe = PlanningPipeline(_bounds(), cfg, saat=saat)
    pipe.set_state(np.array([5.0, 5.0, 0.0, 0.0, 0.0, 0.0]))
    pipe.set_waypoints([(45.0, 45.0)])
    kosan = pipe.replan_sayaclari[0]
    saat.ilerlet(0.001)
    pipe.set_obstacles([CircleObstacle(25.0, 25.0, 1.0)])
    assert pipe.replan_sayaclari[0] > kosan, "fren kapalıyken replan koşmadı"
    assert pipe.replan_ertelendi == 0


def test_FP9_kor_payi_ve_tavan_TURETILMIS_degerlerde_donuyor() -> None:
    """Sayılar ölçümden geliyor; sessizce değişirse bu test kırmızı olur.

    katsayı 3.0 → kör oran = 1/(1+3) = %25 (T_plan/(T_plan+aralık)).
    tavan 1.9 s = replan_proximity 2,0 m ÷ ölçülmüş seyir hızı 1,05 m/s;
    ayrıca ArduPilot'ın 3 s'lik GUIDED zaman aşımının altında kalır.
    """
    cfg = PlanningPipelineConfig()
    assert cfg.replan_bosluk_katsayisi == 3.0
    assert cfg.replan_max_interval_s == 1.9
    assert cfg.replan_max_interval_s < 3.0, "ArduPilot GUIDED zaman aşımı"
