"""
Girdap İDA — Duba kapısı takibi (gate_follower) çekirdek testleri.

select_gate (durumsuz seçim): kapı yok → None, geçerli kapı → orta nokta,
genişlik/derinlik/menzil elemeleri, en yakın kapının seçimi, arkadaki duba eleme,
left/right etiketleme. GateFollower (durumlu): fallback, kilitlenme, geçince
serbest bırakma, oklüzyonda koruma, drift güncelleme, reset.

Çalıştır: pytest prototype/tests/test_gate_follower.py -v
"""

from __future__ import annotations

import math

import pytest

from prototype.mission.gate_follower import (
    Gate,
    GateDiagnostics,
    GateFollower,
    GateFollowerConfig,
    select_gate,
)

_CFG = GateFollowerConfig()


# ------------------------------------------------------------ yardımcı

def _approx(a: tuple[float, float], b: tuple[float, float], tol: float = 1e-6) -> bool:
    return math.hypot(a[0] - b[0], a[1] - b[1]) <= tol


# ------------------------------------------------------------ select_gate: temel

def test_bos_liste_none() -> None:
    assert select_gate((0.0, 0.0), (0.0, 20.0), [], _CFG) is None


def test_tek_duba_none() -> None:
    # Kapı için en az 2 kenar dubası gerekir.
    assert select_gate((0.0, 0.0), (0.0, 20.0), [(-2.0, 10.0)], _CFG) is None


def test_arac_hedefin_ustunde_none() -> None:
    # Kurs yönü tanımsız (araç == GN) → seçim yapılamaz.
    assert select_gate((5.0, 5.0), (5.0, 5.0), [(-2.0, 10.0), (2.0, 10.0)], _CFG) is None


def test_temiz_kapi_orta_nokta() -> None:
    # Araç orijinde, GN kuzeyde (+y). Kapı: (-2,10) ve (2,10) → orta (0,10).
    gate = select_gate((0.0, 0.0), (0.0, 20.0), [(-2.0, 10.0), (2.0, 10.0)], _CFG)
    assert gate is not None
    assert _approx(gate.midpoint, (0.0, 10.0))
    assert gate.width == 4.0


def test_gn_kapinin_yaninda_yine_orta_noktadan_gecer() -> None:
    """Şartname md 5.5.2.2: GN tam kapı ortasında OLMAYABİLİR. GN yana kaymış
    olsa bile hedef, algılanan kapının ORTASI olmalı (ham GN değil)."""
    # Gerçek kapı ortası (0,10); ama hakem GN'yi (5,20) vermiş (5 m yanda).
    gate = select_gate((0.0, 0.0), (5.0, 20.0), [(-2.0, 10.0), (2.0, 10.0)], _CFG)
    assert gate is not None
    # Kapı hâlâ (0,10) civarı — GN'nin yan kayması hedefi kapıdan koparmadı.
    assert gate.midpoint[0] < 2.0        # ham GN'nin x=5'ine kaymadı
    assert 8.0 < gate.midpoint[1] < 12.0


# ------------------------------------------------------------ select_gate: elemeler

def test_cok_genis_cift_elenir() -> None:
    # Ayrım 30 m > gate_width_max(20) → kapı sayılmaz.
    # ⚠ 2026-08-03: bant 1.5-8 → 1.0-20 m'ye GENİŞLETİLDİ. Gerekçe: şartname
    # kapı genişliğinin yarışma alanına göre değişeceğini ve dubaların deniz
    # şartlarıyla yer değiştirebileceğini söylüyor → genişlik önceden
    # bilinemez. Dar bant gerçek kapıyı SESSİZCE reddederdi (bkz.
    # GateFollowerConfig docstring'i). Bu test artık yeni üst sınırı bekliyor.
    assert select_gate(
        (0.0, 0.0), (0.0, 40.0), [(-15.0, 10.0), (15.0, 10.0)], _CFG
    ) is None


def test_genislik_reddi_olculen_mesafeyi_raporlar() -> None:
    """Sessiz ret kapanı: banda girmeyen çiftin ÖLÇÜLEN mesafesi teşhise yazılır.

    Genişlik önceden bilinemediği için sahada bandın yanlış olması olasıdır;
    operatörün bunu düzeltebilmesi ancak gerçek mesafeyi görmesiyle mümkün.
    """
    diag = GateDiagnostics()
    assert select_gate(
        (0.0, 0.0), (0.0, 40.0), [(-15.0, 10.0), (15.0, 10.0)], _CFG, diag
    ) is None
    assert diag.n_edge_buoys == 2
    assert diag.n_pairs_checked == 1
    assert len(diag.reddedilen_genislik) == 1
    assert diag.reddedilen_genislik[0] == pytest.approx(30.0, abs=1e-6)
    assert diag.secilen_genislik is None


def test_secilen_kapinin_genisligi_raporlanir() -> None:
    """Kapı bulunduğunda ölçülen genişlik teşhise yazılır (saha teyidi)."""
    diag = GateDiagnostics()
    gate = select_gate(
        (0.0, 0.0), (0.0, 20.0), [(-2.0, 10.0), (2.0, 10.0)], _CFG, diag
    )
    assert gate is not None
    assert diag.secilen_genislik == pytest.approx(4.0, abs=1e-6)
    assert diag.reddedilen_genislik == []


def test_cok_dar_cift_elenir() -> None:
    # Ayrım 0.5 m < gate_width_min(1.0) → tek dubanın gürültüsü, kapı değil.
    assert select_gate(
        (0.0, 0.0), (0.0, 20.0), [(-0.25, 10.0), (0.25, 10.0)], _CFG
    ) is None


def test_farkli_derinlik_cift_elenir() -> None:
    # Biri 5 m'de biri 15 m'de (fark 10 > pair_depth_tol=3) → yan yana değil.
    assert select_gate(
        (0.0, 0.0), (0.0, 30.0), [(-2.0, 5.0), (2.0, 15.0)], _CFG
    ) is None


def test_menzil_disi_duba_elenir() -> None:
    # İkisi de max_lookahead(25) ötesinde → dikkate alınmaz.
    assert select_gate(
        (0.0, 0.0), (0.0, 40.0), [(-2.0, 30.0), (2.0, 30.0)], _CFG
    ) is None


def test_arkadaki_dubalar_elenir() -> None:
    # Dubalar aracın ARKASINDA (-y, GN +y iken) → ileri projeksiyon negatif.
    assert select_gate(
        (0.0, 0.0), (0.0, 20.0), [(-2.0, -10.0), (2.0, -10.0)], _CFG
    ) is None


# ------------------------------------------------------------ select_gate: seçim

def test_en_yakin_kapi_secilir() -> None:
    # İki kapı: yakın (y=8) ve uzak (y=18). En yakın seçilmeli.
    buoys = [(-2.0, 8.0), (2.0, 8.0), (-2.0, 18.0), (2.0, 18.0)]
    gate = select_gate((0.0, 0.0), (0.0, 30.0), buoys, _CFG)
    assert gate is not None
    assert _approx(gate.midpoint, (0.0, 8.0))


def test_left_right_etiketleme() -> None:
    # GN kuzeyde: sol = -x tarafı (+lateral), sağ = +x tarafı.
    gate = select_gate((0.0, 0.0), (0.0, 20.0), [(-2.0, 10.0), (2.0, 10.0)], _CFG)
    assert gate is not None
    assert gate.left[0] < 0.0            # sol duba -x'te
    assert gate.right[0] > 0.0           # sağ duba +x'te


def test_dogu_yonunde_kurs() -> None:
    # GN doğuda (+x). Kapı dubaları y ekseninde ayrık, x=10'da.
    gate = select_gate((0.0, 0.0), (20.0, 0.0), [(10.0, -2.0), (10.0, 2.0)], _CFG)
    assert gate is not None
    assert _approx(gate.midpoint, (10.0, 0.0))


# ------------------------------------------------------------ GateFollower: fallback

def test_follower_kapi_yoksa_ham_gn() -> None:
    gf = GateFollower(_CFG)
    res = gf.update((0.0, 0.0), (0.0, 20.0), [])
    assert res.used_fallback is True
    assert _approx(res.target, (0.0, 20.0))
    assert res.gate is None


def test_follower_kapi_varsa_orta_nokta() -> None:
    gf = GateFollower(_CFG)
    res = gf.update((0.0, 0.0), (0.0, 20.0), [(-2.0, 10.0), (2.0, 10.0)])
    assert res.used_fallback is False
    assert _approx(res.target, (0.0, 10.0))
    assert gf.committed_gate is not None


# ------------------------------------------------------------ GateFollower: histerezis

def test_follower_gecince_serbest_birakir() -> None:
    gf = GateFollower(_CFG)
    buoys = [(-2.0, 10.0), (2.0, 10.0)]
    gf.update((0.0, 0.0), (0.0, 20.0), buoys)          # kapıya kilitlen
    assert gf.committed_gate is not None
    # Araç kapının ötesine geçti (y=10.5 > 10) → serbest bırakılmalı.
    gf.update((0.0, 10.5), (0.0, 20.0), [])            # kapı artık arkada + görünmüyor
    assert gf.committed_gate is None


def test_follower_okluzyonda_kapiyi_korur() -> None:
    # Kapıya kilitlendikten sonra bir tick duba GÖRÜNMEZSE (dalga/oklüzyon)
    # ve araç henüz geçmediyse hedef zıplamamalı — kilitli kapı korunur.
    gf = GateFollower(_CFG)
    buoys = [(-2.0, 10.0), (2.0, 10.0)]
    gf.update((0.0, 0.0), (0.0, 20.0), buoys)
    res = gf.update((0.0, 3.0), (0.0, 20.0), [])       # duba görünmüyor, henüz geçmedi
    assert res.used_fallback is False
    assert _approx(res.target, (0.0, 10.0))            # ham GN'ye düşmedi
    assert gf.committed_gate is not None


def test_follower_taze_algiyla_drift_gunceller() -> None:
    # Kapı biraz kaydıysa (dalga) ve match_radius içindeyse kilitli kapı
    # taze algıyla güncellenir (aynı kapı, drift).
    gf = GateFollower(_CFG)
    gf.update((0.0, 0.0), (0.0, 20.0), [(-2.0, 10.0), (2.0, 10.0)])
    res = gf.update((0.0, 1.0), (0.0, 20.0), [(-1.5, 10.5), (2.5, 10.5)])
    assert res.used_fallback is False
    # Orta nokta ~ (0.5, 10.5) — güncellendi, eski (0,10)'da kalmadı.
    assert res.target[1] > 10.0


def test_follower_reset_kilidi_temizler() -> None:
    gf = GateFollower(_CFG)
    gf.update((0.0, 0.0), (0.0, 20.0), [(-2.0, 10.0), (2.0, 10.0)])
    assert gf.committed_gate is not None
    gf.reset()
    assert gf.committed_gate is None


# ------------------------------------------------------------ Gate veri sınıfı

def test_gate_width_hesabi() -> None:
    g = Gate(left=(-3.0, 5.0), right=(3.0, 5.0), midpoint=(0.0, 5.0))
    assert g.width == 6.0
