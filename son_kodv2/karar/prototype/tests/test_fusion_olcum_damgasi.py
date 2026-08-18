"""§1.57 — POZ DAMGASI = ÖLÇÜM ANI nöbetçileri.

SORUN (ölçüldü §1.56f, n=37 366): `fusion_node` hem `pose` hem `odom`'u
`get_clock().now()` ile damgalıyordu = YAYIN ANI. Besleyen mavros pozunun
damgasıyla farkı ortanca **63 ms** (%95 95,9 · %99 100,6 · maks 371).

BEDELİ: `planning_node._poz_damgada` bu damgayla poz arar ve LiDAR taramasını
o pozla dünyaya çevirir. Kayma, tekne dönerken taramayı yay boyunca kaydırır;
hafızanın eşleşme bandı ~0,60 m aşılınca aynı duba İKİNCİ kayıt açar.
18.08 bant ölçümü (4 256 603 tespit): 11 320'si bandın dışına düştü.

ROS sözleşmesi zaten bunu ister: damga ölçümün ALINDIĞI anı gösterir.
"""
from __future__ import annotations

import math

import pytest

from prototype.fusion.bypass import PosePassthrough
from prototype.fusion.pipeline import FusionPipeline, FusionPipelineConfig


def _besle(fp: FusionPipeline, t0: float, n: int, dt: float = 0.02) -> float:
    t = t0
    for _ in range(n):
        t += dt
        fp.on_velocity(0.5, 0.0)
        fp.on_imu(t, 0.05)
    return t


# ── iSAM2 kolu ────────────────────────────────────────────────────────
def test_ISAM2_kolu_olcum_zamanini_TASIYOR():
    fp = FusionPipeline(FusionPipelineConfig(keyframe_rate_hz=5.0))
    assert fp.son_olcum_zamani is None          # hiç ölçüm yokken uydurmaz
    son_t = _besle(fp, 1000.0, 30)
    assert fp.son_olcum_zamani is not None
    # Ölçüm zamanı GERÇEK bir girdi anı olmalı — yayın anı değil.
    assert 1000.0 < fp.son_olcum_zamani <= son_t


def test_olcum_zamani_ANAHTAR_KOVASI_kadar_geride():
    """Damga son anahtarın anını gösterir; kova (1/keyframe_rate) kadar
    geride olması DOĞRU davranıştır — poz gerçekten o ana aittir.
    Uydurma bir "şimdi" basmak, olmayan bir tazelik iddia etmek olurdu."""
    cfg = FusionPipelineConfig(keyframe_rate_hz=5.0)
    fp = FusionPipeline(cfg)
    son_t = _besle(fp, 1000.0, 30)
    gerilik = son_t - fp.son_olcum_zamani
    assert 0.0 <= gerilik <= cfg.keyframe_period_s + 1e-6, gerilik


def test_olcum_zamani_GERI_GITMIYOR():
    """Tüketiciler artan damga bekler; geri giden damga poz tamponunu
    temizletir (`planning_node._poz_tamponuna_yaz`)."""
    fp = FusionPipeline(FusionPipelineConfig(keyframe_rate_hz=5.0))
    t = 1000.0
    onceki = None
    for _ in range(10):
        t = _besle(fp, t, 15)
        simdi = fp.son_olcum_zamani
        if onceki is not None:
            assert simdi >= onceki, f"{simdi} < {onceki}"
        onceki = simdi


# ── video (bypass) kolu — AYNI sözleşme ───────────────────────────────
def test_BYPASS_kolu_ayni_sozlesmeyi_sunuyor():
    """İki kol da `son_olcum_zamani` sunmalı, yoksa `fusion_node` hangi
    kolda olduğunu bilmek zorunda kalır (ve biri sessizce yayın anına düşer)."""
    pp = PosePassthrough()
    assert hasattr(pp, "son_olcum_zamani")
    assert pp.son_olcum_zamani is None
    pp.update(1.0, 2.0, 0.3, t=1234.5)
    assert pp.son_olcum_zamani == pytest.approx(1234.5)


def test_bypass_ESKI_cagri_bicimi_bozulmadi():
    """`t` vermeyen eski çağrı çalışmaya devam etmeli; damga taşınmaz ve
    `fusion_node` yayın anına düşer — BİLİNEN ve loglanan hâl."""
    pp = PosePassthrough()
    pp.update(1.0, 2.0, 0.3)
    assert pp.has_pose
    assert pp.son_olcum_zamani is None


def test_iki_kol_AYNI_arayuzu_sunuyor():
    """Sözleşme testi: iki poz kaynağı da aynı adı aynı anlamda taşımalı."""
    fp = FusionPipeline(FusionPipelineConfig())
    pp = PosePassthrough()
    for kaynak in (fp, pp):
        assert hasattr(kaynak, "son_olcum_zamani"), type(kaynak).__name__
        assert hasattr(kaynak, "current_pose"), type(kaynak).__name__


# ── kaynakta bilgi ATILMIYOR ──────────────────────────────────────────
def test_fusion_node_YAYIN_ANINI_dogrudan_basmiyor():
    """🔴 REGRESYON KAPISI. `header.stamp = now` deseni geri gelirse ölçülen
    63 ms de geri gelir. Damga `_olcum_damgasi()` üzerinden geçmeli."""
    import pathlib

    kok = pathlib.Path(__file__).resolve().parents[1].parent
    yol = (kok / "ros2_ws/src/girdap_decision/girdap_decision/fusion_node.py")
    kaynak = yol.read_text(encoding="utf-8")
    # ⚠ ALT-DİZGİ ARAMA YETMEZ: `_olcum_damgasi_KALDIRILDI` de "_olcum_damgasi"
    # içerir ve mutasyon kaçar (kendi mutasyon koşumumda yakalandı). Tanımı
    # `ast` ile, çağrıyı tam desenle ara.
    import ast as _ast

    agac = _ast.parse(kaynak)
    tanimlar = {
        d.name
        for n in _ast.walk(agac)
        if isinstance(n, _ast.ClassDef)
        for d in n.body
        if isinstance(d, (_ast.FunctionDef, _ast.AsyncFunctionDef))
    }
    assert "_olcum_damgasi" in tanimlar, f"yardımcı TANIMI yok; bulunanlar: {sorted(tanimlar)[:5]}…"
    assert "self._olcum_damgasi(" in kaynak, "yardımcı tanımlı ama ÇAĞRILMIYOR"
    # Poz/odom yayınında doğrudan `now` basılmamalı.
    assert "ps.header.stamp = now" not in kaynak
    assert "od.header.stamp = now" not in kaynak
    assert "ps.header.stamp = stamp" in kaynak
    assert "od.header.stamp = stamp" in kaynak
