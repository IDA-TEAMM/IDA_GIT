"""
Girdap İDA — B2 HUNİ: kapı direklerinin engel payı (ROS'SUZ).

🔴 **Çözdüğü arıza (§0.2b B2 → §0.17g/2).** Kenar dubaları çarpışma
korumasından TAMAMEN çıkarılıyordu (`continue  # kapı dubası ENGEL DEĞİL`).
Gerekçe doğruydu — küresel 1,0 m'lik ceza halkası dar bir geçidin içini
kaplar, 1,5 m'de araç geçitten hiç geçmiyor — ama sonucu şuydu: **dubalardan
iten hiçbir kuvvet kalmadı**, yalnız orta noktanın çekimi vardı. Gerçek
parkurda ölçülen gövde payı **−0,23 m**, yani temas (Ç1: P1'de 16 puan).

Çözüm ne payı sıfırlamak ne de küresel payı dayatmak: payı **o an ölçülen
açıklıktan** türetmek. Formülün girdileri ya ölçülmüş tekne boyutu, ya
şartname sabiti (duba çapı 30 cm), ya da o an ölçülen geometri → §0.0d'nin
"ayarlanabilir eşik yok" kuralı bozulmuyor.

Bu dosya `planning_node`'a bağlanmadan (rclpy'siz) formülün kendisini ve
MPPI'nin per-engel payı gerçekten kullandığını dondurur.
"""

from __future__ import annotations

import math

from prototype.planning.mppi import MPPIConfig, MPPIController
from prototype.planning.rrt_star import Bounds, CircleObstacle

HULL_W = 0.78
BUOY_R = 0.15


def huni_payi(i: int, kenarlar: list, tavan: float, hull_w: float = HULL_W) -> float:
    """`planning_node._huni_payi`'nin ROS'suz ikizi (formül birebir aynı).

    ⚠ İkiz olduğu için ayrışabilir; `test_node_formulu_ayni_kaldi` ikisini
    kaynak düzeyinde bağlar.
    """
    if len(kenarlar) < 2:
        return tavan
    dx, dy = kenarlar[i]
    en_yakin = min(
        math.hypot(dx - kx, dy - ky)
        for j, (kx, ky) in enumerate(kenarlar) if j != i
    )
    return max(0.0, min(tavan, (en_yakin - hull_w - 2.0 * BUOY_R) / 2.0))


# ------------------------------------------------------------------ formül

def test_genis_kapida_pay_TAVANA_dayanir() -> None:
    """Gerçek P1: 12 m kapı → pay tavana dayanır.

    (12 − 0,78 − 0,30)/2 = 5,46 m ⇒ `gate_post_margin_m` kazanır.
    """
    kenarlar = [(6.0, 1.0), (6.0, -11.0)]        # 12 m açıklık
    assert huni_payi(0, kenarlar, 1.4) == 1.4


def test_dar_kapida_pay_KUCULUR_gecit_kapanmaz() -> None:
    """B2 tablosu: W=1,65 m ⇒ yan başına 0,285 m. Tavan (1,4 m) dayatılsaydı
    iki halka geçidin içinde ÜST ÜSTE BİNER ve araç hiç giremezdi."""
    kenarlar = [(0.0, 0.0), (1.65, 0.0)]
    m = huni_payi(0, kenarlar, 1.4)
    assert math.isclose(m, (1.65 - HULL_W - 0.30) / 2.0, abs_tol=1e-9)
    assert math.isclose(m, 0.285, abs_tol=1e-3)
    # Geçilebilirlik: iki halka arasında gövde sığmalı
    serbest = 1.65 - 2 * BUOY_R - 2 * m
    assert serbest >= HULL_W - 1e-9


def test_gecilemez_kapida_pay_SIFIR() -> None:
    """W < hull + 2r ⇒ zaten geçilemez; pay negatife düşmez, 0'da durur."""
    kenarlar = [(0.0, 0.0), (0.9, 0.0)]          # 0,9 < 0,78+0,30 = 1,08
    assert huni_payi(0, kenarlar, 1.4) == 0.0


def test_pay_KAPININ_kendi_genisligiyle_sinirli_degil() -> None:
    """Koridoru daraltan komşu kapının direği olabilir — ölçüt "geçmem gereken
    EN DAR boşluk". Gerçek P1'de partner 12 m, komşu kapının direği 6,4 m."""
    partner = (6.0, -11.0)
    komsu = (10.0, 6.0)                          # bir sonraki kapının direği
    duba = (6.0, 1.0)
    kenarlar = [duba, partner, komsu]
    d_partner = math.dist(duba, partner)
    d_komsu = math.dist(duba, komsu)
    assert d_komsu < d_partner                   # kurgunun kendisi
    beklenen = min(1.4, (d_komsu - HULL_W - 0.30) / 2.0)
    assert math.isclose(huni_payi(0, kenarlar, 1.4), beklenen, abs_tol=1e-9)


def test_tek_duba_varken_tavan() -> None:
    """Kapı çifti kurulmamış (tek direk görünüyor) → tavan."""
    assert huni_payi(0, [(1.0, 2.0)], 1.4) == 1.4


def test_ayni_noktadaki_iki_tespit_payi_TAVANA_kacirmaz() -> None:
    """İndeksle dışlama nöbetçisi: koordinatla dışlansaydı iki özdeş tespit
    birbirini eler ve pay yanlışlıkla tavana çıkardı."""
    kenarlar = [(5.0, 0.0), (5.0, 0.0)]
    assert huni_payi(0, kenarlar, 1.4) == 0.0    # mesafe 0 → geçilemez


# ------------------------------------------------------- MPPI entegrasyonu

def _ctrl(obstacles, margin: float) -> MPPIController:
    from prototype.dynamics.catamaran import CatamaranDynamics

    return MPPIController(
        CatamaranDynamics(),
        Bounds(-50.0, 50.0, -50.0, 50.0),
        obstacles,
        MPPIConfig(obstacle_margin=margin, K=8, T=5),
    )


def test_mppi_engelin_KENDI_payini_kullanir() -> None:
    """`CircleObstacle.margin` küresel payın YERİNE geçmeli."""
    ctrl = _ctrl([CircleObstacle(0.0, 0.0, 0.15, margin=0.3)], margin=1.4)
    # MPPI dizileri float32 (`_dtype`) → tolerans ona göre, 1e-9 tutmaz.
    r_eff = float(ctrl._as_numpy(ctrl._obs_r)[0])
    assert math.isclose(r_eff, 0.15 + 0.3, abs_tol=1e-6)


def test_mppi_margin_YOKSA_kuresel_payi_kullanir() -> None:
    """Geriye tam uyum: margin=None olan engeller eski davranışta kalır."""
    ctrl = _ctrl([CircleObstacle(0.0, 0.0, 0.15)], margin=1.4)
    r_eff = float(ctrl._as_numpy(ctrl._obs_r)[0])
    assert math.isclose(r_eff, 0.15 + 1.4, abs_tol=1e-6)


def test_mppi_karisik_torbada_ikisi_birden() -> None:
    """Aynı taramada hem huni direği hem normal engel bulunur."""
    ctrl = _ctrl(
        [CircleObstacle(0.0, 0.0, 0.15, margin=0.3), CircleObstacle(9.0, 0.0, 0.4)],
        margin=1.4,
    )
    r = [float(v) for v in ctrl._as_numpy(ctrl._obs_r)]
    assert math.isclose(r[0], 0.45, abs_tol=1e-6)
    assert math.isclose(r[1], 1.80, abs_tol=1e-6)


def test_sifir_pay_dubayi_yine_de_ENGEL_birakir() -> None:
    """Pay 0 olsa bile dubanın kendi yarıçapı iter — B2'nin asıl kazancı bu:
    eskiden duba torbadan tamamen çıkıyordu, yani ceza yarıçapı YOKTU."""
    ctrl = _ctrl([CircleObstacle(0.0, 0.0, 0.15, margin=0.0)], margin=1.4)
    assert math.isclose(float(ctrl._as_numpy(ctrl._obs_r)[0]), 0.15, abs_tol=1e-6)


# ------------------------------------------------------------ ikiz nöbetçi

def test_node_formulu_ayni_kaldi() -> None:
    """`planning_node._huni_payi` bu dosyadaki ikizle aynı formülü kullanmalı.

    Node rclpy gerektirdiği için burada import EDİLEMEZ; kaynak metni okunup
    formülün üç bileşeni aranıyor. Node tarafı değişirse bu test kırmızıya
    döner ve ikizin de güncellenmesi gerektiği anlaşılır.
    """
    from pathlib import Path

    kaynak = (
        Path(__file__).resolve().parents[2]
        / "ros2_ws" / "src" / "girdap_decision" / "girdap_decision"
        / "planning_node.py"
    ).read_text(encoding="utf-8")
    govde = kaynak.split("def _huni_payi")[1].split("def _log_edge_memory")[0]
    assert "hull_width_m - 2.0 * BUOY_RADIUS_M" in govde
    assert "serbest / 2.0" in govde
    assert "max(0.0, min(" in govde
    assert "j != i" in govde, "indeksle dışlama kaldırılmış"
