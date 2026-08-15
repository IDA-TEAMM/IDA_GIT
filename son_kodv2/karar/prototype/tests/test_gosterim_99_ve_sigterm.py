"""15.08.2026 kaptan kararlarının nöbetçileri.

  1) Dosya-3 haritasında CLASS_UNKNOWN=99 ÇİZİLMEZ — ama KONTROL torbasında
     kalır (güvenlik). İkisini birden bağlar: biri bozulursa kırmızı.
  2) LiDAR mp4'ünde sınıflanmamış kümeye "?" halkası/etiketi çizilmez, ama
     küme rengi (şartname md 493 "ayırma") DURUR.
  3) SIGTERM düzgün kapanışa çevrilir — mp4'ün moov atomu yazılsın diye.
"""
from __future__ import annotations

import os
import signal
import subprocess
import sys

import numpy as np
import pytest

from prototype.dynamics.catamaran import CatamaranDynamics
from prototype.mapping.bev_renderer import (
    SINIF_BILINMEYEN,
    BevConfig,
    BevRenderer,
    Kume,
)
from prototype.perception.fusion import CLASS_UNKNOWN
from prototype.planning.pipeline import PlanningPipeline, PlanningPipelineConfig
from prototype.planning.rrt_star import Bounds, CircleObstacle


# --------------------------------------------------------------------------
# 1) Yerel harita — 99 çizilmez, kontrol torbası dokunulmaz
# --------------------------------------------------------------------------

def _boru() -> PlanningPipeline:
    p = PlanningPipeline(
        Bounds(-50.0, 50.0, -50.0, 50.0),
        PlanningPipelineConfig(),
        dynamics=CatamaranDynamics(),
    )
    p.set_state(np.zeros(6))
    return p


def test_gosterim_engelleri_haritayi_suzer_kontrolu_SUZMEZ() -> None:
    """99'lar haritadan düşer, MPPI torbasında kalır — ikisi AYRI."""
    p = _boru()
    hepsi = [CircleObstacle(5.0, 0.0, 0.5), CircleObstacle(-5.0, 0.0, 0.5)]
    p.set_obstacles(hepsi)

    dolu_hepsi = int((p.local_cost_grid().data == 100).sum())
    assert dolu_hepsi > 0, "iki engelle harita boş olamaz"

    # Yalnız BİRİ sınıflanmış → haritada yalnız o görünmeli
    p.set_gosterim_engelleri([hepsi[0]])
    dolu_suzulmus = int((p.local_cost_grid().data == 100).sum())

    assert 0 < dolu_suzulmus < dolu_hepsi, (
        "gösterim süzgeci haritayı küçültmeli "
        f"(hepsi={dolu_hepsi}, süzülmüş={dolu_suzulmus})"
    )
    # 🔑 KONTROL YOLU DEĞİŞMEDİ — güvenlik kuralı
    assert p._obstacles == hepsi, "gösterim süzgeci kontrol torbasına dokunmamalı"


def test_gosterim_engelleri_None_eski_davranis() -> None:
    """Süzgeç verilmemişse harita kontrol listesinden çizilir (geriye uyum)."""
    p = _boru()
    hepsi = [CircleObstacle(5.0, 0.0, 0.5), CircleObstacle(-5.0, 0.0, 0.5)]
    p.set_obstacles(hepsi)
    referans = p.local_cost_grid().data.copy()

    p.set_gosterim_engelleri([hepsi[0]])
    p.set_gosterim_engelleri(None)              # geri al
    assert np.array_equal(p.local_cost_grid().data, referans)


def test_bos_gosterim_listesi_haritayi_bosaltir() -> None:
    """Hiç sınıflanmış nesne yoksa harita boş — 'None' ile karıştırılmamalı."""
    p = _boru()
    p.set_obstacles([CircleObstacle(5.0, 0.0, 0.5)])
    p.set_gosterim_engelleri([])
    assert int((p.local_cost_grid().data > 0).sum()) == 0


# --------------------------------------------------------------------------
# 2) LiDAR mp4 çizimi — "?" yok, küme rengi var
# --------------------------------------------------------------------------

def _kume(sinif, merkez=(3.0, 0.0)) -> Kume:
    noktalar = [(merkez[0] + dx, merkez[1] + dy)
                for dx in (-0.1, 0.0, 0.1) for dy in (-0.1, 0.0, 0.1)]
    return Kume(merkez=merkez, yaricap=0.15, noktalar=noktalar,
                sinif=sinif, kume_id=3)


def test_siniflanmamis_kume_halka_cizmez_ama_kume_rengi_kalir() -> None:
    r = BevRenderer(BevConfig())                       # gizle_siniflanmamis=True
    kare = r.render_lidar((0.0, 0.0), 0.0, (), [_kume(SINIF_BILINMEYEN)],
                           zaman_metni="00:00:00")
    kare_bos = r.render_lidar((0.0, 0.0), 0.0, (), (), zaman_metni="00:00:00")

    # Küme rengi çizilmiş olmalı → kare boş kareden FARKLI (md 493 "ayırma")
    assert not np.array_equal(kare, kare_bos), (
        "sınıflanmamış küme tamamen kaybolmamalı — küme rengi kalmalı"
    )
    # Gri sınıf halkası (150,150,150) çizilmemiş olmalı
    gri = np.all(kare == np.array([150, 150, 150], dtype=np.uint8), axis=-1)
    assert not gri.any(), "sınıflanmamış kümeye gri halka/etiket çizilmemeli"


def test_siniflanmis_kume_halkasini_KORUR() -> None:
    """Süzgeç yalnız 99'u vurmalı — turuncu KENAR hâlâ halkalı ve etiketli."""
    r = BevRenderer(BevConfig())
    kare = r.render_lidar((0.0, 0.0), 0.0, (), [_kume(0)], zaman_metni="00:00:00")
    turuncu = np.all(kare == np.array([255, 140, 0], dtype=np.uint8), axis=-1)
    assert turuncu.any(), "sınıflanmış kümenin halkası/etiketi çizilmeli"


def test_gizleme_kapatilabilir() -> None:
    """A/B: bayrak False ise eski davranış birebir geri gelir."""
    r = BevRenderer(BevConfig(gizle_siniflanmamis=False))
    kare = r.render_lidar((0.0, 0.0), 0.0, (), [_kume(SINIF_BILINMEYEN)],
                           zaman_metni="00:00:00")
    gri = np.all(kare == np.array([150, 150, 150], dtype=np.uint8), axis=-1)
    assert gri.any(), "bayrak kapalıyken gri halka geri gelmeli"


def test_bev_bilinmeyen_sinif_fusion_ile_ayni() -> None:
    """Çizicideki kopya sabit, füzyonun sözleşmesinden AYRILMAMALI."""
    assert SINIF_BILINMEYEN == CLASS_UNKNOWN


# --------------------------------------------------------------------------
# 3) SIGTERM → düzgün kapanış (mp4 moov atomu)
# --------------------------------------------------------------------------

_SIGTERM_BETIK = r"""
import signal, sys, time
sys.path.insert(0, %r)
from girdap_decision.sigterm_kapanis import sigterm_kapanisi_kur

kapandi = False
sigterm_kapanisi_kur()
try:
    while True:
        time.sleep(0.02)
except KeyboardInterrupt:
    kapandi = True
finally:
    # Gerçek node'da burada `destroy_node()` → `Mp4Yazici.kapat()` çalışır.
    print("DUZGUN_KAPANIS" if kapandi else "YANLIS", flush=True)
"""


def test_sigterm_duzgun_kapanisa_cevrilir() -> None:
    """SIGTERM `finally`'yi ÇALIŞTIRMALI — yoksa mp4 moov atomsuz kalır."""
    paket = os.path.join(
        os.path.dirname(os.path.dirname(os.path.dirname(
            os.path.abspath(__file__)))),
        "ros2_ws", "src", "girdap_decision",
    )
    if not os.path.isdir(paket):
        pytest.skip("girdap_decision paketi bulunamadı")

    p = subprocess.Popen(
        [sys.executable, "-c", _SIGTERM_BETIK % paket],
        stdout=subprocess.PIPE, text=True,
    )
    try:
        # işleyicinin kurulmasına zaman tanı
        for _ in range(200):
            if p.poll() is not None:
                break
            import time as _t
            _t.sleep(0.01)
        p.send_signal(signal.SIGTERM)
        cikti, _ = p.communicate(timeout=10)
    finally:
        if p.poll() is None:
            p.kill()

    assert "DUZGUN_KAPANIS" in cikti, (
        "SIGTERM `finally`'yi çalıştırmadı — teslim mp4'ü oynatılamaz olur "
        f"(çıktı: {cikti!r})"
    )
