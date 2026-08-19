"""🔴 DAĞITIMDAN ERİŞİLEMEYEN ŞALTER OLMAZ (19.08.2026).

Bu depoda **dört kez** aynı kusur çıktı: ölçülmüş, işe yarayan bir kol ayar
sınıfında tanımlı ama hiçbir ROS parametresine / yaml'a bağlı değil ⇒ sahada
**yeniden derlemeden denenemiyor**, yani fiilen yok:

  · `edge_unutma_katsayisi` — `04bddb7` (ilk vaka, A/B tablosu vardı)
  · `geri_hiz_yasak`        — §1.60b (canlıda `ros2 param get` → "Parameter not set")
  · `reanchor_sigma_xy`     — §1.68e-② (çapa 0,05 m sabit, GPS σ≈0,30 m iken)
  · `stuck_recovery_enabled`— kendi belgesi "A/B / ACİL KAPATMA için" diyor,
                              ama kapatmanın YOLU YOKTU

Kusur sinsi çünkü **hiçbir hata vermez**: yaml'a yazarsın, düğüm sessizce
varsayılanla koşar. Sahada SSH yok, tek görünürlük kanalı journal.

Bu nöbetçi, dağıtım için kritik şalterlerin **dört yerde birden** bağlı
olduğunu dondurur: `declare_parameter` · launch beyaz listesi · `hardware.yaml`
· `params.yaml`. Listeye yeni bir şalter eklemek bilinçli bir karardır —
gerekçesi yanına yazılır.
"""
from __future__ import annotations

from pathlib import Path

import pytest

KOK = Path(__file__).resolve().parents[2]
SRC = KOK / "ros2_ws" / "src" / "girdap_decision"

#: (şalter, neden dağıtımdan erişilebilir OLMAK ZORUNDA)
SALTERLER = [
    ("stuck_recovery_enabled",
     "F-P.11 sıkışma kurtarması — belgesi 'A/B ve ACİL KAPATMA için' diyor"),
    ("mppi_geri_hiz_yasak",
     "geri sürüşü HIZ uzayında eler, fren serbest; `mppi_ileri_kisit`in "
     "fren kaybı sorunu bunda yok — denenebilmeli"),
    ("mppi_ileri_kisit",
     "kaptan kararı KAPALI; kararı taşıyan şalter erişilebilir kalmalı"),
    ("edge_unutma_katsayisi",
     "04bddb7'nin ilk vakası — A/B adayı (2,0 → 1,0)"),
    ("gecis_zorunlu",
     "geçiş ölçütü; fly-by buna bağlı"),
    ("bilinmeyen_engelleri_tut",
     "kaptan kararı kodda yazılı, TEKNOFEST öncesi gözden geçirilecek"),
]

#: Füzyon şalterleri `hardware.launch.py` üzerinden GEÇMEZ — `fusion_node`
#: doğrudan `params.yaml`/`hardware.yaml` okur. O yüzden onlar için launch
#: koşulu aranmaz; `declare_parameter` + iki yaml yeter.
FUZYON_SALTERLERI = [
    ("reanchor_sigma_xy",
     "§1.68e-②: bağlı değilken çapa 0,05 m SABİT iddia ediyordu; ölçülen "
     "GPS σ ≈ 0,25-0,34 m (bantların hiçbirinde RTK yok)"),
]


def _metin(yol: Path) -> str:
    return yol.read_text(encoding="utf-8")


@pytest.mark.parametrize("salter,neden", SALTERLER)
def test_SALTER_dort_yerde_de_BAGLI(salter: str, neden: str) -> None:
    eksik = []
    dugumler = list((SRC / "girdap_decision").glob("*.py"))
    if not any(f'"{salter}"' in _metin(f) for f in dugumler):
        eksik.append("declare_parameter (düğüm)")
    if salter not in _metin(SRC / "launch" / "hardware.launch.py"):
        eksik.append("launch beyaz listesi")
    for y in ("hardware.yaml", "params.yaml"):
        if salter not in _metin(SRC / "config" / y):
            eksik.append(y)
    assert not eksik, (
        f"`{salter}` şu yerlerde BAĞLI DEĞİL: {eksik}. Neden bağlı olmalı: "
        f"{neden}. Bağlanmayan şalter sahada yeniden derlemeden denenemez ve "
        f"yaml'a yazılan değer SESSİZCE yok sayılır (04bddb7 sınıfı)."
    )


@pytest.mark.parametrize("salter,neden", FUZYON_SALTERLERI)
def test_FUZYON_SALTERI_dugum_ve_iki_yamlda_BAGLI(salter: str, neden: str) -> None:
    """Füzyon tarafı launch'tan geçmez; `declare_parameter` + iki yaml aranır."""
    eksik = []
    dugumler = list((SRC / "girdap_decision").glob("*.py"))
    if not any(f'"{salter}"' in _metin(f) for f in dugumler):
        eksik.append("declare_parameter (düğüm)")
    for y in ("hardware.yaml", "params.yaml"):
        if salter not in _metin(SRC / "config" / y):
            eksik.append(y)
    assert not eksik, (
        f"`{salter}` şu yerlerde BAĞLI DEĞİL: {eksik}. Neden bağlı olmalı: {neden}"
    )
