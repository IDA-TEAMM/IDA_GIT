"""
Girdap İDA — MissionManager (video waypoint görev) testleri.

3 waypoint'lik sahte görevle durum makinesini doğrular:
    IDLE → ACTIVE → DWELL → ACTIVE → ... → COMPLETE
    - arrival_radius doğru tetikleniyor mu
    - dwell_time'a saygı gösteriliyor mu

Çalıştır: pytest prototype/tests/test_mission_manager.py -v
"""

from __future__ import annotations

import math

from prototype.mission.mission_manager import (
    FcMissionItem,
    MissionManager,
    MissionManagerConfig,
    MissionPhase,
    Waypoint,
    fc_items_to_waypoints,
    fc_items_to_waypoints_with_seqs,
    latlon_to_enu,
)

_R = 6378137.0
_CFG = MissionManagerConfig(arrival_radius_m=2.0, dwell_time_s=2.0)


def _mission() -> MissionManager:
    """Kuzeye dizili 3 waypoint (~111 m aralık)."""
    return MissionManager(
        [
            Waypoint(0.0010, 0.0, "P1"),
            Waypoint(0.0020, 0.0, "P2"),
            Waypoint(0.0030, 0.0, "P3"),
        ],
        _CFG,
    )


def _lat_offset_m(meters: float) -> float:
    """Kuzey metre → derece enlem farkı."""
    return math.degrees(meters / _R)


# --------------------------------------------------------------------------- #
# ENU dönüşümü
# --------------------------------------------------------------------------- #


def test_latlon_to_enu_north_east() -> None:
    east, north = latlon_to_enu(0.0, 0.0, _lat_offset_m(100.0), 0.0)
    assert abs(north - 100.0) < 0.5          # ~100 m kuzey
    assert abs(east) < 1e-6                   # doğu ~0


# --------------------------------------------------------------------------- #
# Durum makinesi
# --------------------------------------------------------------------------- #


def test_idle_until_start() -> None:
    m = _mission()
    assert m.phase is MissionPhase.IDLE
    assert m.update(0.0, 0.0, 0.0) is None    # IDLE → hedef yok
    m.start()
    assert m.phase is MissionPhase.ACTIVE
    assert m.current_index == 0


def test_active_returns_offset_to_target() -> None:
    m = _mission()
    m.start()
    off = m.update(0.0, 0.0, 0.0)             # P1 ~111 m kuzeyde
    assert off is not None
    east, north = off
    assert north > 100.0 and abs(east) < 1.0
    assert m.phase is MissionPhase.ACTIVE


def test_arrival_triggers_dwell() -> None:
    m = _mission()
    m.start()
    m.update(0.0010, 0.0, 0.0)                # tam P1'de → varış
    assert m.phase is MissionPhase.DWELL


def test_dwell_respects_time_then_advances() -> None:
    m = _mission()
    m.start()
    m.update(0.0010, 0.0, 0.0)                # t=0 varış → DWELL
    assert m.phase is MissionPhase.DWELL
    m.update(0.0010, 0.0, 1.0)                # t=1 < dwell(2) → hâlâ DWELL
    assert m.phase is MissionPhase.DWELL
    assert m.current_index == 0
    m.update(0.0010, 0.0, 2.0)                # t=2 ≥ dwell → index++ ACTIVE
    assert m.phase is MissionPhase.ACTIVE
    assert m.current_index == 1


def test_arrival_radius_boundary() -> None:
    m = _mission()
    m.start()
    # 3 m güneyde (arrival_radius 2 m dışı) → ACTIVE kalmalı
    m.update(0.0010 - _lat_offset_m(3.0), 0.0, 0.0)
    assert m.phase is MissionPhase.ACTIVE
    # 1.5 m güneyde (içi) → DWELL
    m.update(0.0010 - _lat_offset_m(1.5), 0.0, 1.0)
    assert m.phase is MissionPhase.DWELL


def test_full_sequence_to_complete() -> None:
    m = _mission()
    m.start()
    t = 0.0
    for wp_lat in (0.0010, 0.0020, 0.0030):
        m.update(wp_lat, 0.0, t)             # varış → DWELL
        assert m.phase is MissionPhase.DWELL
        t += 2.0
        m.update(wp_lat, 0.0, t)             # dwell bitti → ilerle / tamamla
    assert m.phase is MissionPhase.COMPLETE
    assert m.is_complete
    assert m.update(0.0030, 0.0, t + 1.0) is None
    assert m.current_waypoint is None


# --------------------------------------------------------------------------- #
# FC (MAVLink) görev listesi → Waypoint dönüşümü (T0-f)
# --------------------------------------------------------------------------- #


def _item(seq: int, lat: float, lon: float, command: int = 16) -> FcMissionItem:
    return FcMissionItem(seq=seq, command=command, lat=lat, lon=lon)


def test_fc_skips_home_seq0_by_default() -> None:
    items = [
        _item(0, 40.0, 29.0),        # home → atlanmalı
        _item(1, 40.001, 29.0),
        _item(2, 40.001, 29.001),
    ]
    wps = fc_items_to_waypoints(items)
    assert len(wps) == 2
    assert (wps[0].lat, wps[0].lon) == (40.001, 29.0)
    assert (wps[1].lat, wps[1].lon) == (40.001, 29.001)


def test_fc_keeps_seq0_when_skip_disabled() -> None:
    items = [_item(0, 40.0, 29.0), _item(1, 40.001, 29.0)]
    wps = fc_items_to_waypoints(items, skip_home_seq0=False)
    assert len(wps) == 2
    assert (wps[0].lat, wps[0].lon) == (40.0, 29.0)


def test_fc_filters_non_nav_commands() -> None:
    items = [
        _item(0, 40.0, 29.0),                 # home
        _item(1, 40.001, 29.0),               # NAV_WAYPOINT → tut
        _item(2, 40.002, 29.0, command=177),  # DO_JUMP → at (konumlu olsa bile)
        _item(3, 40.003, 29.0, command=82),   # NAV_SPLINE_WAYPOINT → tut
    ]
    wps = fc_items_to_waypoints(items)
    assert [(w.lat) for w in wps] == [40.001, 40.003]


def test_fc_skips_zero_coordinates() -> None:
    items = [
        _item(0, 40.0, 29.0),        # home
        _item(1, 0.0, 0.0),          # tanımsız (0,0) → at
        _item(2, 40.001, 29.0),      # tut
    ]
    wps = fc_items_to_waypoints(items)
    assert len(wps) == 1
    assert (wps[0].lat, wps[0].lon) == (40.001, 29.0)


def test_fc_labels_all_parkur_1() -> None:
    items = [_item(0, 40.0, 29.0), _item(1, 40.001, 29.0), _item(2, 40.002, 29.0)]
    wps = fc_items_to_waypoints(items)
    assert all(w.parkur == 1 for w in wps)


def test_fc_empty_and_home_only() -> None:
    assert fc_items_to_waypoints([]) == []
    # Yalnız home varsa (görev yüklenmemiş) → boş; node bunu güncellemez.
    assert fc_items_to_waypoints([_item(0, 40.0, 29.0)]) == []


def test_fc_dikdortgen_video_senaryosu() -> None:
    """Şartname md 3.3.1(2)+(3) video senaryosu ucu uca (çekirdek).

    QGC'den yüklenen görev: home (seq 0) + DİKDÖRTGEN oluşturan 4 nokta.
    Araç 4 köşeyi sırayla gezer; 4. (SON) noktada dwell dolunca görev
    TAMAMLANMIŞ olmalı (md 3.3.1(3) — sonrası manuel dönüş, görev noktası
    olarak başlangıca dönüş EKLENMEZ).
    """
    lat0, lon0 = 40.8000000, 29.3000000          # gerçekçi göl koordinatı
    dlat, dlon = 0.0002, 0.0003                  # ~22 m × ~25 m dikdörtgen
    corners = [
        (lat0 + dlat, lon0),                     # P1
        (lat0 + dlat, lon0 + dlon),              # P2
        (lat0, lon0 + dlon),                     # P3
        (lat0, lon0),                            # P4 (başlangıç köşesi ≠ dönüş)
    ]
    items = [_item(0, lat0, lon0)] + [
        _item(i + 1, la, lo) for i, (la, lo) in enumerate(corners)
    ]
    wps = fc_items_to_waypoints(items)
    assert len(wps) == 4                         # home atlandı, 4 köşe kaldı

    m = MissionManager(wps, _CFG)
    m.start()
    t = 0.0
    for i, (la, lo) in enumerate(corners):
        m.update(la, lo, t)                      # köşeye varış → DWELL
        assert m.phase is MissionPhase.DWELL
        assert m.current_index == i
        t += _CFG.dwell_time_s
        m.update(la, lo, t)                      # dwell doldu → ilerle/tamamla
    assert m.is_complete                         # 4. noktada görev tamam
    assert m.update(lat0, lon0, t + 1.0) is None # sonrası hedef üretilmez


def test_fc_waypoints_drive_mission_manager() -> None:
    """FC'den üretilen waypoint'ler MissionManager'ı normal sürer."""
    items = [
        _item(0, 0.0005, 0.0),       # home → atlanır
        _item(1, 0.0010, 0.0),
        _item(2, 0.0020, 0.0),
    ]
    m = MissionManager(fc_items_to_waypoints(items), _CFG)
    assert m.waypoint_count == 2
    m.start()
    assert m.phase is MissionPhase.ACTIVE
    off = m.update(0.0, 0.0, 0.0)                # ilk hedefe ENU ofseti
    assert off is not None and off[1] > 100.0    # ~111 m kuzey


# --------------------------------------------------------------------------- #
# F-M.1 — hedef-mesafe makullük kontrolü (masa OOM olayı, 2026-07-12)
# --------------------------------------------------------------------------- #


def test_farthest_waypoint_m_yakin_ve_bos() -> None:
    """~11 m kuzeydeki tek wp doğru ölçülür; boş liste 0 döner."""
    from prototype.mission.mission_manager import farthest_waypoint_m

    wps = [Waypoint(lat=41.0001, lon=29.0, name="k", parkur=1)]
    d = farthest_waypoint_m(41.0, 29.0, wps)
    assert 10.0 < d < 13.0
    assert farthest_waypoint_m(41.0, 29.0, []) == 0.0


def test_farthest_waypoint_m_null_island_istanbul() -> None:
    """Masa senaryosu: (0,0) sahte konumdan 40°K/29°D hedef binlerce km çıkar."""
    from prototype.mission.mission_manager import farthest_waypoint_m

    wps = [
        Waypoint(lat=40.0, lon=29.0, name="a", parkur=1),
        Waypoint(lat=40.001, lon=29.0, name="b", parkur=1),
    ]
    d = farthest_waypoint_m(0.0, 0.0, wps)
    assert d > 4_000_000.0                     # > 4000 km — makullükten fersah uzak


# ----- F-V.8: FC görev ilerlemesiyle senkron (MISSION_ITEM_REACHED) -----
#
# AUTO'da görevi FC uçurur; bizim varış tespitimiz (arrival_radius_m) FC'nin
# WP_RADIUS'undan farklıysa ya da rover köşeyi yarıçapımıza girmeden dönerse
# index'imiz TAKILIR: yon_setpoint görev sonuna kadar geriyi gösterir,
# COMPLETE hiç gelmez, FSM PARKUR1'de kalır → manuel dönüşte setpoint yazılır.
# Çözüm: FC'nin kendi MISSION_ITEM_REACHED'i (mavros /mavros/mission/reached)
# bizim index'i İLERİ senkronlar. GUIDED yarışmasında FC görev koşmaz →
# sinyal hiç gelmez → davranış değişmez.


def test_fv8_seq_eslemesi_home_ve_do_itemlarini_atlar() -> None:
    """FC wp_seq → bizim index eşlemesi; home(0) + DO item'ları listede yok."""
    items = [
        FcMissionItem(seq=0, command=16, lat=41.0, lon=29.0),     # home → atla
        FcMissionItem(seq=1, command=16, lat=41.001, lon=29.0),
        FcMissionItem(seq=2, command=178, lat=0.0, lon=0.0),      # DO_CHANGE_SPEED
        FcMissionItem(seq=3, command=16, lat=41.001, lon=29.001),
    ]
    wps, seqs = fc_items_to_waypoints_with_seqs(items, skip_home_seq0=True)
    assert len(wps) == 2
    assert seqs == [1, 3]                     # FC seq'leri; DO item eşlenmez
    # Eski API aynı kalmalı (geriye uyum).
    assert [w.lat for w in fc_items_to_waypoints(items)] == [w.lat for w in wps]


def _iki_wp_mgr() -> MissionManager:
    wps = [
        Waypoint(lat=41.001, lon=29.0, name="wp0", parkur=1),
        Waypoint(lat=41.001, lon=29.001, name="wp1", parkur=1),
    ]
    return MissionManager(wps, MissionManagerConfig(dwell_time_s=0.0))


def test_fv8_dis_varis_indexi_ilerletir() -> None:
    m = _iki_wp_mgr()
    m.start()
    assert m.notify_external_reached(0) is True
    assert m.current_index == 1
    assert m.phase is MissionPhase.ACTIVE


def test_fv8_son_waypoint_complete_yapar() -> None:
    m = _iki_wp_mgr()
    m.start()
    assert m.notify_external_reached(1) is True     # index atlayarak da olur
    assert m.is_complete is True


def test_fv8_gecmis_ve_gecersiz_index_yok_sayilir() -> None:
    m = _iki_wp_mgr()
    m.start()
    m.notify_external_reached(0)
    assert m.notify_external_reached(0) is False    # geride kalan tekrar
    assert m.current_index == 1
    assert m.notify_external_reached(99) is False   # aralık dışı


def test_fv8_baslamamis_gorevde_yok_sayilir() -> None:
    """IDLE'da gelen reached (örn. bayat latched mesaj) görevi İLERLETMEMELİ."""
    m = _iki_wp_mgr()
    assert m.notify_external_reached(1) is False
    assert m.phase is MissionPhase.IDLE
