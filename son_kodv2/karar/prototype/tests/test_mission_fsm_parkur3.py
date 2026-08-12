"""Parkur-3 giriş/çıkış — FAZ 1 (2026-08-13). ROS gerekmez.

Şartname: P3 = kamikaze angajman, **145 puan** (toplam 300'ün %48'i, s.25).
Tetik = *"son görev noktasına varmak"*; P1→P2 geçişiyle SİMETRİK ve Şekil 3'ün
P3'e ayrı görev noktası verip vermemesinden ETKİLENMEZ (şartname o konuda
sessiz).

🔑 Tüm P3 davranışı `p3_bekleniyor` (hedef rengi yüklü) kapısının arkasında:
kapalıyken FSM bugünküyle **bit birebir** aynı.
"""
from __future__ import annotations

from prototype.fsm.mission_fsm import MissionFSM, MissionState, Observation


def _p1e_getir(fsm: MissionFSM) -> None:
    """BOOT→ARM→BEKLEMEDE→PARKUR1. Başlatma TEK ATIŞ `request_start()` ile
    (YKİ komutu); `kill_switch_off` tek başına yetmez."""
    fsm.tick(Observation(boot_ok=True))                       # BOOT → ARM
    fsm.tick(Observation(boot_ok=True, kill_switch_off=True))  # ARM → BEKLEMEDE
    fsm.request_start()
    fsm.tick(Observation(boot_ok=True, kill_switch_off=True))  # → PARKUR1
    assert fsm.state is MissionState.PARKUR1


def _p2ye_getir(fsm: MissionFSM) -> None:
    _p1e_getir(fsm)
    fsm.tick(Observation(kill_switch_off=True, dist_to_last_wp_p1=0.5))
    assert fsm.state is MissionState.PARKUR2


# ─────────────────────────── GİRİŞ (M1) ───────────────────────────
def test_renk_YOKSA_eski_davranis_BIT_BIREBIR() -> None:
    """Kapı kapalı: waypoint'ler bitince TAMAMLANDI (bugünkü davranış)."""
    fsm = MissionFSM()
    _p2ye_getir(fsm)
    fsm.tick(Observation(kill_switch_off=True, mission_complete=True))
    assert fsm.state is MissionState.TAMAMLANDI


def test_renk_VARSA_son_waypointte_PARKUR3e_gecer() -> None:
    fsm = MissionFSM()
    _p2ye_getir(fsm)
    fsm.tick(Observation(kill_switch_off=True, mission_complete=True,
                         p3_bekleniyor=True))
    assert fsm.state is MissionState.PARKUR3


def test_PARKUR1den_de_gecebilir() -> None:
    """Görev tek parkurluysa (P2'ye hiç girilmediyse) de P3 açılmalı."""
    fsm = MissionFSM()
    _p1e_getir(fsm)
    fsm.tick(Observation(kill_switch_off=True, mission_complete=True,
                         p3_bekleniyor=True))
    assert fsm.state is MissionState.PARKUR3


# ────────────── 🔴 TASARIMDA YAKALANAN TUZAK ──────────────
def test_PARKUR3_TEK_TICK_yasamaz() -> None:
    """🔴🔴 `mission_complete` LATCH'lidir (bir kez True olunca sıfırlanmaz).

    Eski kural PARKUR3'ü de kapsıyordu ⇒ P3'e giren tekne bir SONRAKİ tick'te
    aynı kuralla TAMAMLANDI'ya düşerdi: **kamikaze tek tick yaşar, hedefe hiç
    gidilmez, 145 puan sessizce gider.** Tasarım sırasında yakalandı.
    """
    fsm = MissionFSM()
    _p2ye_getir(fsm)
    obs = Observation(kill_switch_off=True, mission_complete=True,
                      p3_bekleniyor=True)
    fsm.tick(obs)
    assert fsm.state is MissionState.PARKUR3
    for _ in range(20):                       # latch'li sinyal akmaya devam
        fsm.tick(obs)
        assert fsm.state is MissionState.PARKUR3, "P3 erken sonlandı"


# ─────────────────────────── ÇIKIŞ (M2) ───────────────────────────
def _p3e_getir(fsm: MissionFSM) -> Observation:
    _p2ye_getir(fsm)
    obs = Observation(kill_switch_off=True, mission_complete=True,
                      p3_bekleniyor=True)
    fsm.tick(obs)
    assert fsm.state is MissionState.PARKUR3
    return obs


def test_cikis_SOK() -> None:
    fsm = MissionFSM(); obs = _p3e_getir(fsm)
    fsm.tick(Observation(**{**obs.__dict__, "shock_detected_p3": True}))
    assert fsm.state is MissionState.TAMAMLANDI


def test_cikis_ILERLEME_YOK() -> None:
    """🔴 Asıl tetik bu: şok eşiği 3,0 g ama temas 0,03-0,14 g üretiyor
    (IMU durağanken zaten 1,0 g okur) ⇒ şok ASLA gelmez."""
    fsm = MissionFSM(); obs = _p3e_getir(fsm)
    fsm.tick(Observation(**{**obs.__dict__, "p3_ilerleme_yok": True}))
    assert fsm.state is MissionState.TAMAMLANDI


def test_cikis_SURE_ASIMI() -> None:
    """Hedef hiç bulunamazsa tekne sonsuza kadar sürüklenmesin."""
    fsm = MissionFSM(); obs = _p3e_getir(fsm)
    fsm.tick(Observation(**{**obs.__dict__, "p3_sure_doldu": True}))
    assert fsm.state is MissionState.TAMAMLANDI


def test_P3te_hicbir_cikis_yoksa_KALIR() -> None:
    fsm = MissionFSM(); obs = _p3e_getir(fsm)
    for _ in range(50):
        fsm.tick(obs)
    assert fsm.state is MissionState.PARKUR3


def test_KILL_P3ten_de_calisir() -> None:
    """Güvenlik yolu P3'te de açık kalmalı."""
    fsm = MissionFSM(); obs = _p3e_getir(fsm)
    fsm.tick(Observation(**{**obs.__dict__, "kill_switch_active": True}))
    assert fsm.state is MissionState.KILL
