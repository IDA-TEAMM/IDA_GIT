"""
Girdap İDA — F-P.10: RRT* AYRI SÜREÇTE (asenkron planlama) testleri.

🔴 NEDEN (13.08.2026, §0.66/§0.68/§0.69 + ampirik ölçüm):
`planning_node` tek thread'de koşuyor; RRT* aynı thread'de. Bu Jetson'da
ölçüldü (10 Hz döngü, CUDA'lı ebeveyn):

    senkron plan  → döngünün en kötü gecikmesi **370,7 ms**  (>100 ms: 1/80 tur)
    asenkron plan → **1,1 ms** (taban çizgisi 1,8 ms — yani ölçülemez fark)

⚠ Ayrı THREAD çözmez: Python GIL, işlemci bağımlı işte paralellik vermez ve
`rrt_star`'ın ana döngüsü saf Python. ⚠ `fork` kullanılamaz: düğümde cupy/CUDA
bağlamı açık; `spawn` şart.

Testlerin çoğu SAHTE işçiyle koşar — süreç doğurmak yavaş ve belirlenimsizdir;
sözleşme (ne zaman gönderilir, sonuç nasıl kurulur, arıza hâlinde ne olur)
sahteyle bire bir sınanır. Bir test gerçek süreci uçtan uca doğrular.
"""

from __future__ import annotations

import numpy as np

from prototype.planning.pipeline import PlanningPipeline, PlanningPipelineConfig
from prototype.planning.rrt_star import Bounds, CircleObstacle


def _bounds() -> Bounds:
    return Bounds(0.0, 50.0, 0.0, 50.0)


def _cfg(**kw) -> PlanningPipelineConfig:
    taban = dict(mppi_K=200, mppi_T=30, plan_isci_enabled=True)
    taban.update(kw)
    return PlanningPipelineConfig(**taban)


class _SahteIsci:
    """`PlanIscisi` sözleşmesinin testte sürülebilen kopyası."""

    def __init__(self, kullanilabilir: bool = True) -> None:
        self.kullanilabilir = kullanilabilir
        self.mesgul = False
        self.istekler: list = []
        self._sonuc = None

    def gonder(self, sinir, engeller, cfg, start, goal, simdi=None) -> bool:
        if self.mesgul or not self.kullanilabilir:
            return False
        self.istekler.append((start, goal, list(engeller)))
        self.mesgul = True
        return True

    def sonuc_al(self, simdi=None):
        if self._sonuc is None:
            return None
        sonuc, self._sonuc = self._sonuc, None
        self.mesgul = False
        return sonuc

    def hazirla(self, yol, hata=None) -> None:
        """Bir sonraki `sonuc_al` bunu döndürsün."""
        self._sonuc = (yol, hata)

    def kapat(self) -> None:
        self.mesgul = False


def _pipe(isci: _SahteIsci, **kw) -> PlanningPipeline:
    pipe = PlanningPipeline(_bounds(), _cfg(**kw), isci=isci)
    pipe.set_state(np.array([5.0, 5.0, 0.0, 0.0, 0.0, 0.0]))
    return pipe


def test_FP10_ILK_plan_SENKRON_kalir() -> None:
    """🔴 Görev başında araç duruyor ve cmd_vel akışı YOK → bloklamak zararsız.
    Referanssız kalmak ise zararlı (A1: _ref_path None → MPPI kurulmaz →
    araç hiç kıpırdamaz). Bu yüzden ilk plan işçiye GİTMEZ."""
    isci = _SahteIsci()
    pipe = _pipe(isci)
    pipe.set_waypoints([(45.0, 45.0)])
    assert isci.istekler == [], "ilk plan işçiye gönderildi (araç referanssız kalır)"
    assert pipe.global_path is not None, "ilk plan senkron kurulmadı"


def test_FP10_HAREKETTEYKEN_replan_isciye_gider_ve_BLOKLAMAZ() -> None:
    """Plan kurulduktan sonraki engel kaynaklı replan asenkron olmalı ve
    çağrı ANINDA dönmeli — mevcut referans korunarak."""
    isci = _SahteIsci()
    pipe = _pipe(isci, replan_bosluk_katsayisi=0.0)      # fren karışmasın
    pipe.set_waypoints([(45.0, 45.0)])
    onceki_yol = pipe.global_path
    pipe.set_obstacles([CircleObstacle(25.0, 25.0, 1.0)])
    assert len(isci.istekler) == 1, "replan işçiye gitmedi"
    assert pipe.global_path == onceki_yol, "yol gelmeden referans değişti"


def test_FP10_sonuc_gelince_referans_KURULUR() -> None:
    isci = _SahteIsci()
    pipe = _pipe(isci, replan_bosluk_katsayisi=0.0)
    pipe.set_waypoints([(45.0, 45.0)])
    pipe.set_obstacles([CircleObstacle(25.0, 25.0, 1.0)])
    yeni_yol = [(5.0, 5.0), (20.0, 20.0), (45.0, 45.0)]
    isci.hazirla(yeni_yol)
    assert pipe.plan_sonucunu_isle() is True
    assert pipe.global_path == yeni_yol
    assert pipe.plan_sonucunu_isle() is False, "aynı sonuç iki kez kuruldu"


def test_FP10_ISCI_MESGULKEN_yeni_istek_GONDERILMEZ() -> None:
    """Kuyruk şişmesin: bayat istek zaten değersizdir (sahne değişmiştir)."""
    isci = _SahteIsci()
    pipe = _pipe(isci, replan_bosluk_katsayisi=0.0)
    pipe.set_waypoints([(45.0, 45.0)])
    pipe.set_obstacles([CircleObstacle(25.0, 25.0, 1.0)])
    pipe.set_obstacles([CircleObstacle(24.0, 24.0, 1.0)])
    pipe.set_obstacles([CircleObstacle(23.0, 23.0, 1.0)])
    assert len(isci.istekler) == 1, "işçi meşgulken yeni istek gönderildi"


def test_FP10_ISCI_MESGULKEN_SENKRONA_DUSULMEZ() -> None:
    """🔴 En sinsi gerileme: meşgulken senkron koşmak, tam da kaçınılan
    bloklamayı geri getirirdi. Sayaç artmamalı (RRT* koşmamalı)."""
    isci = _SahteIsci()
    pipe = _pipe(isci, replan_bosluk_katsayisi=0.0)
    pipe.set_waypoints([(45.0, 45.0)])
    pipe.set_obstacles([CircleObstacle(25.0, 25.0, 1.0)])
    kosan = pipe.replan_sayaclari[0]
    pipe.set_obstacles([CircleObstacle(24.0, 24.0, 1.0)])
    assert pipe.replan_sayaclari[0] == kosan, "meşgul işçiye rağmen RRT* koştu"


def test_FP10_ISCI_YOKSA_SENKRON_kola_dusulur() -> None:
    """Arıza toleransı: spawn yoksa/izin yoksa planlama YAVAŞLAR ama DURMAZ."""
    isci = _SahteIsci(kullanilabilir=False)
    pipe = _pipe(isci, replan_bosluk_katsayisi=0.0)
    pipe.set_waypoints([(45.0, 45.0)])
    kosan = pipe.replan_sayaclari[0]
    pipe.set_obstacles([CircleObstacle(25.0, 25.0, 1.0)])
    assert pipe.replan_sayaclari[0] > kosan, "işçi yokken senkron plan koşmadı"
    assert pipe.global_path is not None


def test_FP10_isci_HATA_dondurunce_mevcut_referans_KORUNUR() -> None:
    """İşçi 'çözüm yok'/zaman aşımı derse eski yol yaşar (F10.1 sözleşmesi)."""
    isci = _SahteIsci()
    pipe = _pipe(isci, replan_bosluk_katsayisi=0.0)
    pipe.set_waypoints([(45.0, 45.0)])
    onceki = pipe.global_path
    pipe.set_obstacles([CircleObstacle(25.0, 25.0, 1.0)])
    isci.hazirla(None, "zaman aşımı")
    pipe.plan_sonucunu_isle()
    assert pipe.global_path == onceki, "hata sonrası referans kayboldu"


def test_FP10_VARSAYILAN_KAPALI_prototip_belirlenimli_kalir() -> None:
    """Çevrimdışı kullanım (viz/senaryo/testler) planı AYNI turda ister."""
    assert PlanningPipelineConfig().plan_isci_enabled is False


def test_FP10_GERCEK_SUREC_uctan_uca() -> None:
    """Gerçek `spawn` işçisi: istek → ayrı süreçte RRT* → yol kurulur."""
    import time

    from prototype.planning.plan_isci import PlanIscisi

    isci = PlanIscisi(zaman_asimi_s=30.0)
    pipe = PlanningPipeline(_bounds(), _cfg(replan_bosluk_katsayisi=0.0), isci=isci)
    try:
        pipe.set_state(np.array([5.0, 5.0, 0.0, 0.0, 0.0, 0.0]))
        pipe.set_waypoints([(45.0, 45.0)])
        onceki = pipe.global_path
        pipe.set_obstacles([CircleObstacle(25.0, 25.0, 1.0)])
        assert isci.gonderilen == 1, "gerçek işçiye istek gitmedi"
        t0 = time.monotonic()
        while time.monotonic() - t0 < 40.0:
            if pipe.plan_sonucunu_isle():
                break
            time.sleep(0.02)
        assert isci.tamamlanan == 1, "ayrı süreçten sonuç gelmedi"
        assert pipe.global_path is not None
        assert pipe.global_path != onceki or len(pipe.global_path) >= 2
    finally:
        pipe.kapat()
