"""AYAK İZİ TEMİZLEME nöbetçileri — "tekne oradaysa orada engel yoktur".

Kaynak arıza: GIRDAP_DURUM §1.17d. 17:06 göl koşumunda `RRT*
ValueError('start veya goal engel/sınır içinde')` **1 347 kez** basıldı;
hayalet duba bulutu teknenin kendi konumunu engel içinde gösteriyordu ve
küresel yol **28 dakika** boyunca hiç tazelenmedi.

Dış karşılık: Nav2 engel katmanının `footprint_clearing_enabled` parametresi
(öntanımlı `true`) ayak izinin altındaki hücreleri `FREE_SPACE` yapar.

Bu dosya üç şeyi DONDURUR:
  ① gövdenin altındaki engel düşer / kenarındaki yalnız kırpılır,
  ② kırpma sonrası başlangıç noktası RRT*'a göre DAİMA serbesttir,
  ③ ②'nin dayandığı eşitsizlik (`gövde çevrel yarıçapı > safety_margin`).

Mutasyon turu (üçü de kırmızı olmalı):
  · `_planlama_engelleri`'ni `return self._obstacles` yap        → ① ve ② kırılır
  · kırpmayı silip engeli olduğu gibi bırak (yalnız düşürme)     → ① kırılır
  · `GOVDE_CEVREL_YARICAP_M`'yi 0,4'e düşür                      → ③ kırılır
"""
from __future__ import annotations

import math

import numpy as np
import pytest

from prototype.planning.pipeline import (
    GOVDE_BOYU_M,
    GOVDE_CEVREL_YARICAP_M,
    GOVDE_ENI_M,
    PlanningPipeline,
    PlanningPipelineConfig,
)
from prototype.planning.rrt_star import Bounds, CircleObstacle, RRTStar, RRTStarConfig


def _boru(x: float = 0.0, y: float = 0.0) -> PlanningPipeline:
    p = PlanningPipeline(PlanningPipelineConfig())
    durum = np.zeros(6)
    durum[0], durum[1] = x, y
    p.set_state(durum)
    return p


def test_govde_altindaki_hayalet_DUSER() -> None:
    """Merkez teknenin üstünde: 30 cm'lik bir dubanın içinde yüzüyor olamayız."""
    p = _boru()
    p.set_obstacles([CircleObstacle(0.05, 0.0, 0.15)])
    assert p._planlama_engelleri() == []


def test_govde_kenarindaki_GERCEK_ENGEL_KALIR_yalnizca_kirpilir() -> None:
    """Büyük kıyı kümesi silinmez — Nav2 de yalnız ayak izi altını boşaltır."""
    p = _boru()
    kume = CircleObstacle(4.0, 0.0, 3.6)          # tekne kümenin içinde kalıyor
    p.set_obstacles([kume])
    sonuc = p._planlama_engelleri()
    assert len(sonuc) == 1, "gerçek engel SİLİNMEMELİ, yalnız kırpılmalı"
    assert sonuc[0].r == pytest.approx(4.0 - GOVDE_CEVREL_YARICAP_M)
    assert sonuc[0].cx == kume.cx and sonuc[0].cy == kume.cy


def test_uzaktaki_engele_DOKUNULMAZ() -> None:
    p = _boru()
    uzak = CircleObstacle(20.0, 0.0, 1.0)
    p.set_obstacles([uzak])
    assert p._planlama_engelleri() == [uzak]


@pytest.mark.parametrize(
    "engel",
    [
        CircleObstacle(0.0, 0.0, 0.15),            # tam üstünde
        CircleObstacle(0.30, 0.10, 0.15),          # gövdenin içinde
        CircleObstacle(0.70, 0.0, 0.15),           # gövdeye değiyor
        CircleObstacle(4.0, 0.0, 3.6),             # büyük küme, tekne içinde
    ],
)
def test_kirpma_sonrasi_BASLANGIC_DAIMA_SERBEST(engel: CircleObstacle) -> None:
    """§1.17d'nin 1 347 reddi yapısal olarak imkânsız olmalı."""
    p = _boru()
    p.set_obstacles([engel])
    temiz = p._planlama_engelleri()
    rrt = RRTStar(Bounds(-50.0, 50.0, -50.0, 50.0), temiz, RRTStarConfig())
    assert rrt._point_free(0.0, 0.0), (
        "kırpma sonrası başlangıç hâlâ dolu görünüyor — RRT* yine reddeder"
    )


def test_govde_cevrel_yaricapi_SAFETY_MARGINI_ASMALI() -> None:
    """②'nin dayanağı bu eşitsizlik; `safety_margin` büyütülürse CI kırmızı.

    Gövde köşegeninin yarısı, planlayıcının sert payından büyük olduğu sürece
    kırpılmış engel başlangıcı asla kapatamaz. Eşitsizlik bozulursa garanti
    sessizce kaybolur — bu test o sessizliği engeller.
    """
    assert GOVDE_CEVREL_YARICAP_M > RRTStarConfig().safety_margin


def test_govde_olculeri_GATE_FOLLOWER_ILE_AYNI() -> None:
    """Aynı ölçülmüş sayı iki modülde yaşıyor; birlikte taşınmak zorunda."""
    from prototype.mission.gate_follower import GateFollowerConfig

    cfg = GateFollowerConfig()
    assert GOVDE_BOYU_M == cfg.hull_length_m
    assert GOVDE_ENI_M == cfg.hull_width_m
    assert GOVDE_CEVREL_YARICAP_M == pytest.approx(
        0.5 * math.hypot(cfg.hull_length_m, cfg.hull_width_m)
    )
