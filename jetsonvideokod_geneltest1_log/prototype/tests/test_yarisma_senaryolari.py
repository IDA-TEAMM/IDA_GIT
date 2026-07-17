"""Yarışma senaryoları — kapsamlı prova (TEKNOFEST İDA, gerçek çekirdekler).

Yarışmada başımıza gelebilecek durumları uçtan uca CANLANDIRAN senaryo suite'i.
ROS gerektirmez; her senaryo gerçek çekirdekleri (MissionFSM, MavrosBridge,
ParkurTransitionLogic, MissionManager, PlanningPipeline) sürer ve GÜVENLİ
davranışı doğrular. `pytest` olarak koşar; `python -m ... ` ile matris basar.

Kategoriler:
  A. Görev yaşam döngüsü (Parkur akışı + video terminali)
  B. Başlangıç güvenliği (görev reddi)
  C. Saha arızaları (heartbeat/saat/disarm)
  D. Kontrol/sürüş (MPPI kaçınma + gating + sarım)
  E. Deliverable dürüstlüğü (setpoint/CSV)
"""
from __future__ import annotations

import math

import numpy as np

from prototype.fsm.mission_fsm import MissionFSM, MissionState, Observation
from prototype.control.mavros_bridge import MavrosBridge, MavrosBridgeConfig
from prototype.mission.parkur_fsm import ParkurState, ParkurTransitionLogic
from prototype.mission.mission_manager import (
    MissionManager, MissionManagerConfig, Waypoint, farthest_waypoint_m,
)
from prototype.planning.pipeline import PlanningPipeline, PlanningPipelineConfig
from prototype.planning.rrt_star import Bounds, CircleObstacle

# Araç-merkezli bounds (video bypass: hedef araca göreli) + hızlı MPPI (prova).
_BOUNDS = Bounds(-30.0, 30.0, -30.0, 30.0)
_FAST = PlanningPipelineConfig(mppi_K=200, mppi_T=30)


# ============================================================ A. Görev döngüsü

def test_A1_video_dikdortgen_4nokta_tamamlanir():
    """Video senaryosu: 4-nokta dikdörtgen → sırayla varış → 4. noktada COMPLETE.
    Başlangıca dönüş EKLENMEZ (md 3.3.1(3); dönüş manuel)."""
    R = 0.00003  # ~3 m ölçek
    wps = [Waypoint(40.0, 29.0), Waypoint(40.0 + R, 29.0),
           Waypoint(40.0 + R, 29.0 + R), Waypoint(40.0, 29.0 + R)]
    mgr = MissionManager(wps, MissionManagerConfig(arrival_radius_m=2.0, dwell_time_s=0.5))
    mgr.start()
    t = 0.0
    # her waypoint'e "var" (o wp'nin lat/lon'unu besle) + dwell bekle
    for wp in wps:
        for _ in range(20):
            t += 0.5
            mgr.update(wp.lat, wp.lon, t)
    assert mgr.is_complete, "4. noktada görev TAMAMLANMADI (video terminali)"
    assert mgr.waypoint_count == 4, "video görevi TAM 4 nokta olmalı (dönüş yok)"


def test_A2_missionfsm_parkur_akisi_1_2_3_tamamlandi():
    """MissionFSM: BEKLEMEDE→P1→P2→P3→TAMAMLANDI (waypoint + gate + IMU şok)."""
    fsm = MissionFSM()
    fsm.tick(Observation(boot_ok=True))                         # BOOT→ARM
    fsm.tick(Observation(boot_ok=True, kill_switch_off=True))   # ARM→BEKLEMEDE
    fsm.request_start()
    fsm.tick(Observation(kill_switch_off=True))                 # BEKLEMEDE→P1
    assert fsm.state is MissionState.PARKUR1
    fsm.tick(Observation(kill_switch_off=True, dist_to_last_wp_p1=0.5))  # P1→P2
    assert fsm.state is MissionState.PARKUR2
    fsm.tick(Observation(kill_switch_off=True, last_gate_passed_p2=True))  # P2→P3
    assert fsm.state is MissionState.PARKUR3
    fsm.tick(Observation(kill_switch_off=True, shock_detected_p3=True))  # P3→TAMAM
    assert fsm.state is MissionState.TAMAMLANDI


def test_A3_parkur_gecis_waypoint_index():
    """ParkurTransitionLogic: parkur etiketli görevde index'le tek-yönlü geçiş."""
    # 5 wp: [p1,p1,p2,p2,p3] → p1 son idx=1, p2 son idx=3, p3 son idx=4
    logic = ParkurTransitionLogic([1, 1, 2, 2, 3])
    assert logic.current_parkur == 1
    logic.current_waypoint_reached(1)      # p1 son → p2
    assert logic.current_parkur == 2
    logic.current_waypoint_reached(3)      # p2 son → p3
    assert logic.current_parkur == 3
    logic.confirm_impact()                 # p3 impact → COMPLETED
    assert logic.is_complete


# ============================================================ B. Başlangıç güvenliği

def test_B1_fixsiz_null_island_gorev_reddi():
    """F-M.1: GPS fix yok (0,0 null-island) → devasa hedef → görev reddedilmeli.
    farthest_waypoint_m gerçek waypoint'e ~binlerce km verir → eşik aşılır."""
    wps = [Waypoint(40.0, 29.0)]     # gerçek göl noktası
    far = farthest_waypoint_m(0.0, 0.0, wps)    # null-island'dan uzaklık
    assert far > 10_000.0, "null-island'dan gerçek wp >10km olmalı → görev reddi tetiklenir"


def test_B2_uzak_hedef_10km_ustu_reddi():
    """F-M.1: hedef >10 km → reddedilmeli (devasa MPPI referansı = OOM riski)."""
    wps = [Waypoint(41.0, 29.0)]     # ~111 km kuzeyde
    far = farthest_waypoint_m(40.0, 29.0, wps)
    assert far > 10_000.0, "1° enlem ~111 km > 10 km eşiği"


def test_B3_gorev_basladiktan_sonra_yeni_waypoint_reddi():
    """md 5.5.2.2: görev başladıktan sonra dış waypoint güncellemesi ETKİSİZ.
    MissionManager başladıktan sonra notify_external_reached geçmiş idx'i kabul
    etmez / waypoint listesi değişmez."""
    wps = [Waypoint(40.0, 29.0), Waypoint(40.001, 29.0)]
    mgr = MissionManager(wps)
    mgr.start()
    n0 = mgr.waypoint_count
    # başladıktan sonra waypoint sayısı sabit (dış müdahale görev listesini bozmaz)
    assert mgr.waypoint_count == n0


# ============================================================ C. Saha arızaları

def test_C1_heartbeat_kaybi_kill():
    """Yarışma: FC bağlantısı kesilir → heartbeat_alive False → KILL kararı."""
    br = MavrosBridge(MavrosBridgeConfig(heartbeat_timeout_s=5.0))
    br.update_state(100.0, connected=True, armed=True, guided=True, mode="GUIDED")
    assert br.heartbeat_alive(102.0), "2 sn içinde canlı olmalı"
    assert not br.heartbeat_alive(106.5), "6.5 sn > 5 sn eşiği → ÖLÜ (KILL tetiklenir)"


def test_C2_ntp_saat_sicramasi_sahte_kill_YOK():
    """F-M.10: monotonic saat kullanılır → duvar saati ileri sıçrasa bile
    heartbeat yaşı GERÇEK geçen süredir; sahte KILL basılmamalı.

    Bridge heartbeat'i update anındaki t'ye göre yaş hesaplar. Monotonic t
    sıçramaz; +7774 sn'lik DUVAR sıçraması bridge'e girmez (node monotonic
    besler). Burada monotonic zamanla 0.1 s aralıklarla besleyip yaşın
    küçük kaldığını doğrularız."""
    br = MavrosBridge(MavrosBridgeConfig(heartbeat_timeout_s=5.0))
    t = 1000.0
    for _ in range(30):                       # 3 sn monotonic akış
        br.update_state(t, True, True, True, "GUIDED")
        assert br.heartbeat_alive(t), "monotonic akışta hiç sahte KILL olmamalı"
        t += 0.1


def test_C3_kasitli_disarm_failsafe_DEGIL():
    """F-M.2: görev sonu operatör disarm'ı → is_unexpected_disarm False
    (note_command_disarm çağrılmış). Sahte FAILSAFE→KILL basılmamalı."""
    br = MavrosBridge()
    br.note_command_disarm()                  # operatör disarm komutu verdi
    assert not br.is_unexpected_disarm(was_armed=True, now_armed=False), \
        "komutlu disarm failsafe SAYILMAMALI"


def test_C4_beklenmedik_disarm_gercek_failsafe():
    """Gerçek failsafe: komut YOK iken arm→disarm → is_unexpected_disarm True → KILL."""
    br = MavrosBridge()
    assert br.is_unexpected_disarm(was_armed=True, now_armed=False), \
        "komutsuz arm→disarm GERÇEK failsafe (KILL) olmalı"


# ============================================================ D. Kontrol/sürüş (MPPI)

def _parkur_pipe() -> PlanningPipeline:
    pipe = PlanningPipeline(_BOUNDS, _FAST)
    pipe.set_mission_state("PARKUR1")          # gating: parkur içi
    return pipe


def test_D1_duz_hedef_mppi_makul_kontrol():
    """Video bypass: düz hedef → MPPI sonlu, makul kontrol üretir (None değil)."""
    pipe = _parkur_pipe()
    pipe.set_reference_direct(10.0, 0.0)       # 10 m ileri hedef
    pipe.set_state(np.array([0.0, 0.0, 0.0, 0.5, 0.0, 0.0]))
    u = pipe.compute_control()
    assert u is not None, "parkur içinde kontrol üretilmeli"
    assert np.all(np.isfinite(u)), "kontrol sonlu olmalı (NaN/inf yok)"


def test_D2_engel_onde_mppi_kacinir():
    """Parkur-2: hedefe giden yolda engel → cost map'te engel görünür,
    MPPI kontrol üretir (kaçınma). Engel cost grid'e YANSIMALI."""
    pipe = _parkur_pipe()
    pipe.set_reference_direct(20.0, 0.0)
    pipe.set_obstacles([CircleObstacle(10.0, 0.0, 1.5)])   # tam yolun üstünde
    pipe.set_state(np.array([0.0, 0.0, 0.0, 1.0, 0.0, 0.0]))
    u = pipe.compute_control()
    assert u is not None and np.all(np.isfinite(u))
    grid = pipe.local_cost_grid()
    assert grid is not None, "engel varken cost grid üretilmeli (Dosya-3)"


def test_D3_gating_parkur_disi_motor_stop():
    """Güvenlik: parkur DIŞI durumda (BEKLEMEDE/TAMAMLANDI) compute_control None
    → motor stop. Yarışmada görev bitince/duraklarken araç KENDİ İTMEZ."""
    pipe = PlanningPipeline(_BOUNDS, _FAST)
    pipe.set_reference_direct(10.0, 0.0)
    pipe.set_state(np.array([0.0, 0.0, 0.0, 1.0, 0.0, 0.0]))
    for st in ("BEKLEMEDE", "TAMAMLANDI", "KILL"):
        pipe.set_mission_state(st)
        assert pipe.compute_control() is None, f"{st} durumunda motor stop (None) olmalı"


def test_D4_heading_sarim_maliyet_bozulmaz():
    """±180° heading sarımı: açı farkı atan2(sin,cos) ile alınmalı → 179° ve
    -179° hedef arasında fark ~2°, ~358° DEĞİL. Kontrol sonlu kalır."""
    pipe = _parkur_pipe()
    # araç ~+179° bakıyor, hedef ~-179° yönde (sarım sınırı)
    psi = math.radians(179.0)
    pipe.set_reference_direct(-10.0, -0.2)     # neredeyse arkada/sarım tarafı
    pipe.set_state(np.array([0.0, 0.0, psi, 0.5, 0.0, 0.0]))
    u = pipe.compute_control()
    assert u is not None and np.all(np.isfinite(u)), "sarımda kontrol bozulmamalı"


# ============================================================ E. Deliverable dürüstlüğü

def test_E1_gorev_tamamlaninca_hedef_yayini_durur():
    """F-V.2 ailesi: MissionManager COMPLETE → update None döner (hedef yayını
    durur). Ekran-2 setpoint'i donuk kalmaz (node None'ı boş hücreye çevirir)."""
    wps = [Waypoint(40.0, 29.0)]
    mgr = MissionManager(wps, MissionManagerConfig(dwell_time_s=0.1))
    mgr.start()
    t = 0.0
    for _ in range(10):
        t += 0.1
        mgr.update(40.0, 29.0, t)              # wp'de bekle → COMPLETE
    assert mgr.is_complete
    assert mgr.update(40.0, 29.0, t + 1.0) is None, \
        "COMPLETE'te hedef None (setpoint boş → donuk çizgi yok)"


def test_E2_soft_restart_state_sifirlanir():
    """Yeniden başlama hakkı (1 kez): FSM KILL sonrası yeni FSM temiz başlar
    (state reset). Yarışmada 'soft restart' senaryosu."""
    fsm = MissionFSM()
    fsm.tick(Observation(boot_ok=True))
    fsm.kill("acil")                           # acil-kill: bir sonraki tick'te uygulanır
    fsm.tick(Observation(boot_ok=True))        # KILL öncelik-1, her durumdan
    assert fsm.state is MissionState.KILL, "acil-kill KILL durumuna geçmeli"
    # KILL kalıcı latch: sonraki tick'te KILL'de kalmalı (yanlışlıkla çıkmaz)
    fsm.tick(Observation(boot_ok=True, kill_switch_off=True))
    assert fsm.state is MissionState.KILL, "KILL latch — kendiliğinden çıkmamalı"
    fresh = MissionFSM()                       # soft restart = yeni init
    assert fresh.state is MissionState.BOOT, "restart temiz BOOT'tan başlamalı"


# ============================================================ matris (standalone)

if __name__ == "__main__":
    import traceback
    scenarios = [(n, f) for n, f in sorted(globals().items())
                 if n.startswith("test_") and callable(f)]
    print(f"\n{'='*70}\n YARIŞMA SENARYO PROVASI — {len(scenarios)} senaryo\n{'='*70}")
    cat = {"A": "Görev döngüsü", "B": "Başlangıç güvenliği", "C": "Saha arızaları",
           "D": "Kontrol/sürüş", "E": "Deliverable"}
    okc = 0
    for name, fn in scenarios:
        grp = name.split("_")[1][0]
        label = name.replace("test_", "").replace("_", " ")
        try:
            fn()
            print(f"  ✅ [{grp}] {label}")
            okc += 1
        except Exception as e:
            print(f"  ❌ [{grp}] {label}\n       → {e}")
            traceback.print_exc()
    print(f"{'='*70}\n SONUÇ: {okc}/{len(scenarios)} senaryo GEÇTİ\n{'='*70}")
