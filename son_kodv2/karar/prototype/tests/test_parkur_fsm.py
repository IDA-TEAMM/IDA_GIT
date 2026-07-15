"""
Girdap İDA — Parkur geçiş logic'i çekirdek testleri (Sprint 4).

Waypoint-index tabanlı parkur geçişi: son-index hesabı, PARKUR_1→2→3 ilerleme,
impact→COMPLETED, tek yönlülük (geri dönüş yok), tek-parkur (video) dayanıklılığı.

Çalıştır: pytest prototype/tests/test_parkur_fsm.py -v
"""

from __future__ import annotations

import textwrap

import pytest

from prototype.mission.parkur_fsm import (
    ParkurState,
    ParkurTransitionLogic,
    build_waypoint_infos,
    load_parkur_labels,
)

# Standart yarışma dağılımı: 2 wp parkur-1, 2 wp parkur-2, 1 wp parkur-3.
_COMP_LABELS = [1, 1, 2, 2, 3]


# ---------------------------------------------------------------- son-index

def test_build_waypoint_infos_marks_last_of_each_parkur() -> None:
    infos = build_waypoint_infos(_COMP_LABELS)
    last = {i.index for i in infos if i.is_last_of_parkur}
    assert last == {1, 3, 4}                      # parkur 1→idx1, 2→idx3, 3→idx4


def test_build_waypoint_infos_uneven_distribution() -> None:
    # parkur-1: 3 wp, parkur-2: 1 wp, parkur-3: 2 wp
    infos = build_waypoint_infos([1, 1, 1, 2, 3, 3])
    last = {i.parkur: i.index for i in infos if i.is_last_of_parkur}
    assert last == {1: 2, 2: 3, 3: 5}


def test_fp9_contiguous_olmayan_etiketler_reddedilir() -> None:
    """F-P.9 (robustness taraması, 2026-07-15): [1,1,2,1,3] gibi bir veri
    girişi hatası (parkur-1 parkur-2'den SONRA tekrar görünüyor) öncesinde
    SESSİZCE yanlış last_index hesaplardı (last_index[1] gerçek son parkur-1
    yerine 2. tekrarın index'i olurdu). Artık ValueError fırlatır."""
    with pytest.raises(ValueError, match="contiguous değil"):
        build_waypoint_infos([1, 1, 2, 1, 3])


def test_fp9_contiguous_etiketler_kabul_edilir() -> None:
    """Normal (monoton) diziler regresyon olmadan çalışmaya devam etmeli."""
    build_waypoint_infos([1, 1, 2, 2, 3])          # exception atmamalı
    build_waypoint_infos([1, 1, 1, 2, 3, 3])       # uneven de normal


def test_last_index_of_parkur_property() -> None:
    logic = ParkurTransitionLogic(_COMP_LABELS)
    assert logic.last_index_of_parkur == {1: 1, 2: 3, 3: 4}


# ---------------------------------------------------------------- parser

def test_load_parkur_labels_reads_parkur_field(tmp_path) -> None:  # noqa: ANN001
    path = tmp_path / "m.yaml"
    path.write_text(
        textwrap.dedent(
            """
            waypoints:
              - {lat: 0.0, lon: 0.0, parkur: 1}
              - {lat: 0.0, lon: 0.0, parkur: 2}
              - {lat: 0.0, lon: 0.0, parkur: 3}
            """
        ),
        encoding="utf-8",
    )
    assert load_parkur_labels(str(path)) == [1, 2, 3]


def test_load_parkur_labels_defaults_missing_to_1(tmp_path) -> None:  # noqa: ANN001
    # Video görevi: parkur alanı yok → hepsi 1 (tek parkur).
    path = tmp_path / "video.yaml"
    path.write_text(
        textwrap.dedent(
            """
            waypoints:
              - {lat: 0.0, lon: 0.0, name: "P1"}
              - {lat: 0.0, lon: 0.0, name: "P2"}
            """
        ),
        encoding="utf-8",
    )
    assert load_parkur_labels(str(path)) == [1, 1]


# ---------------------------------------------------------------- geçişler

def test_parkur1_last_wp_advances_to_parkur2() -> None:
    logic = ParkurTransitionLogic(_COMP_LABELS)
    assert logic.state is ParkurState.PARKUR_1
    logic.current_waypoint_reached(0)             # parkur-1 ilk wp — geçiş yok
    assert logic.state is ParkurState.PARKUR_1
    logic.current_waypoint_reached(1)             # parkur-1 SON wp → PARKUR_2
    assert logic.state is ParkurState.PARKUR_2
    assert logic.current_parkur == 2


def test_parkur2_last_wp_advances_to_parkur3() -> None:
    logic = ParkurTransitionLogic(_COMP_LABELS)
    logic.current_waypoint_reached(1)             # → PARKUR_2
    logic.current_waypoint_reached(2)             # parkur-2 ilk wp — geçiş yok
    assert logic.state is ParkurState.PARKUR_2
    logic.current_waypoint_reached(3)             # parkur-2 SON wp → PARKUR_3
    assert logic.state is ParkurState.PARKUR_3


def test_non_last_waypoint_does_not_transition() -> None:
    logic = ParkurTransitionLogic([1, 1, 1, 2, 2, 3])
    for idx in (0, 1):                            # parkur-1 son (idx 2) değil
        logic.current_waypoint_reached(idx)
        assert logic.state is ParkurState.PARKUR_1


# ---------------------------------------------------------------- impact

def test_impact_completes_only_in_parkur3() -> None:
    logic = ParkurTransitionLogic(_COMP_LABELS)
    logic.current_waypoint_reached(1)             # → PARKUR_2
    logic.current_waypoint_reached(3)             # → PARKUR_3
    assert logic.confirm_impact() is ParkurState.COMPLETED
    assert logic.is_complete


def test_impact_before_parkur3_is_noop() -> None:
    logic = ParkurTransitionLogic(_COMP_LABELS)   # PARKUR_1
    logic.confirm_impact()
    assert logic.state is ParkurState.PARKUR_1    # erken impact durumu bozmaz
    assert logic.impact_confirmed is True         # bayrak set ama geçiş yok


# ---------------------------------------------------------------- tek yönlülük

def test_no_backward_transition() -> None:
    logic = ParkurTransitionLogic(_COMP_LABELS)
    logic.current_waypoint_reached(1)             # → PARKUR_2
    # Parkur-1 son wp'sine TEKRAR varış (teorik) → geri dönüş OLMAMALI
    logic.current_waypoint_reached(1)
    assert logic.state is ParkurState.PARKUR_2


def test_full_sequence_history() -> None:
    logic = ParkurTransitionLogic(_COMP_LABELS)
    logic.current_waypoint_reached(1)
    logic.current_waypoint_reached(3)
    logic.confirm_impact()
    chain = [(o.value, n.value) for o, n, _ in logic.history]
    assert chain == [
        ("PARKUR_1", "PARKUR_2"),
        ("PARKUR_2", "PARKUR_3"),
        ("PARKUR_3", "COMPLETED"),
    ]


# ---------------------------------------------------------------- video (tek parkur)

def test_single_parkur_video_stays_parkur1() -> None:
    # Video görevi: 5 waypoint hepsi parkur=1 → parkur katmanı PARKUR_1'de kalır
    logic = ParkurTransitionLogic([1, 1, 1, 1, 1])
    for idx in range(5):
        logic.current_waypoint_reached(idx)
    assert logic.state is ParkurState.PARKUR_1    # bozulmadan, geçiş yok
    assert logic.history == []


def test_empty_mission_is_inert() -> None:
    logic = ParkurTransitionLogic([])             # mission_file yok senaryosu
    logic.current_waypoint_reached(0)
    logic.confirm_impact()
    assert logic.state is ParkurState.PARKUR_1
