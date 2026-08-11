"""Soft-restart çekirdek anlamları — 12'lik madde #11, şartname md 5.5.3.1.

*"Yeniden başlama hakkı 1 kez kullanılabilir, toplanan puanlar SIFIRLANIR,
süre DURMAZ."*

Belgenin kapatma ölçütü beş etki sayıyor: FSM→BEKLEMEDE · `gate.reset()` ·
mission index 0 · MPPI warm-start temizle · CSV/PNG oturumu yeni dizine.
Buradaki testler ROS'suz olan dördünü donduruyor (oturum dizinleri node
seviyesinde, `test_yeniden_baslama_node.py`'de).

En kritik test `test_PUANLAR_sifirlaniyor`: geçiş sayacı temizlenmezse ikinci
turda aynı geçitler "zaten geçildi" sayılır ve **hiçbiri puan getirmez**.
"""

from __future__ import annotations

import pytest

from prototype.fsm.mission_fsm import MissionFSM, MissionState, Observation
from prototype.mission.edge_memory import EdgeBuoyMemory
from prototype.mission.mission_manager import (
    MissionManager,
    MissionManagerConfig,
    MissionPhase,
    Waypoint,
)
from prototype.mission.parkur_fsm import ParkurState, ParkurTransitionLogic


# ------------------------------------------------------------------ FSM


def _hazir_obs() -> Observation:
    o = Observation()
    o.boot_ok = True                  # BOOT→ARM
    o.kill_switch_off = True          # ARM→BEKLEMEDE
    return o


def _fsm_beklemede() -> MissionFSM:
    f = MissionFSM()
    o = _hazir_obs()
    f.tick(o)                         # BOOT→ARM
    f.tick(o)                         # ARM→BEKLEMEDE
    assert f.state is MissionState.BEKLEMEDE, f.state
    return f


def test_fsm_KILL_den_BEKLEMEDE_ye_donuyor() -> None:
    """Asıl kullanım: acil durdurma sonrası ikinci tur."""
    f = _fsm_beklemede()
    f.kill("test")
    f.tick(_hazir_obs())
    assert f.state is MissionState.KILL

    f.yeniden_basla()
    assert f.state is MissionState.BEKLEMEDE


def test_yeniden_baslama_KILL_gerekcesini_temizliyor() -> None:
    """🔴 Temizlenmezse bir sonraki tick ANINDA KILL'e geri döner — yeniden
    başlama hiç işe yaramaz ve sebebi görünmez.
    """
    f = _fsm_beklemede()
    f.kill("test")
    f.yeniden_basla()
    f.tick(_hazir_obs())
    assert f.state is MissionState.BEKLEMEDE, "KILL gerekcesi yasiyor"


def test_yeniden_baslama_BASLAT_komutunu_temizliyor() -> None:
    """Sıfırlama sonrası görev KENDİLİĞİNDEN başlamamalı — operatör yeniden
    `/girdap/mission/start` çağırmalı (iki ayrı komut ilkesi).
    """
    f = _fsm_beklemede()
    f.request_start()
    f.yeniden_basla()
    f.tick(_hazir_obs())
    assert f.state is MissionState.BEKLEMEDE, "gorev kendiliginden basladi"


def test_yeniden_baslama_gecmise_yaziliyor() -> None:
    """Geçiş geçmişi kanıt kaydı — yeniden başlamanın kendisi de görünmeli."""
    f = _fsm_beklemede()
    f.kill("test")
    f.yeniden_basla("YKI yeniden baslama")
    gerekceler = [g for _, _, g in f.history]
    assert any("yeniden baslama" in g for g in gerekceler), gerekceler


def test_fiziksel_kill_switch_YAZILIMI_EZIYOR() -> None:
    """Donanım her zaman kazanır: kill switch hâlâ basılıysa yeniden başlama
    tekneyi hareket edebilir hâle GETİRMEMELİ.
    """
    f = _fsm_beklemede()
    f.kill("test")
    f.yeniden_basla()
    o = Observation()
    o.boot_ok = True
    o.kill_switch_off = False       # kill switch HALA basili
    f.tick(o)
    assert f.state is not MissionState.PARKUR1


# -------------------------------------------------------- görev index'i


def _mgr() -> MissionManager:
    wps = [Waypoint(lat=0.0, lon=0.0), Waypoint(lat=0.001, lon=0.0)]
    return MissionManager(wps, MissionManagerConfig())


def test_gorev_index_sifira_donuyor() -> None:
    m = _mgr()
    m.start()
    assert m._phase is MissionPhase.ACTIVE
    m._idx = 1                                   # ilerlemiş gibi yap
    m.reset()
    assert m.current_index == 0
    assert m._phase is MissionPhase.IDLE


def test_reset_sonrasi_start_YENIDEN_etkili() -> None:
    """`start()` yalnız IDLE'da etkili → IDLE'a dönmek ikinci turu mümkün kılar."""
    m = _mgr()
    m.start()
    m.reset()
    m.start()
    assert m._phase is MissionPhase.ACTIVE
    assert m.current_index == 0


# ------------------------------------------------------- parkur katmanı


def test_parkur_katmani_PARKUR_1_e_donuyor() -> None:
    p = ParkurTransitionLogic([1, 1, 2, 2, 3])
    p.current_waypoint_reached(1)                # P1 bitti → PARKUR_2
    assert p.state is ParkurState.PARKUR_2
    p.reset()
    assert p.state is ParkurState.PARKUR_1


def test_carpma_onayi_temizleniyor() -> None:
    """🔴 Temizlenmezse ikinci turda Parkur-3'e girildiği ANDA "çarpma zaten
    oldu" sayılıp görev TAMAMLANDI'ya düşer.
    """
    p = ParkurTransitionLogic([1, 2, 3])
    p.confirm_impact()
    p.reset()
    assert p._impact_confirmed is False


def test_parkur_gecmisi_KORUNUYOR() -> None:
    """Geçmiş kanıt/teşhis kaydı — ilk turun nereye kadar gittiği lazım."""
    p = ParkurTransitionLogic([1, 1, 2, 2, 3])
    p.current_waypoint_reached(1)
    n = len(p.history)
    p.reset()
    assert len(p.history) >= n


# ---------------------------------------------------- kenar duba hafızası


def test_kenar_hafizasi_temizleniyor() -> None:
    """Araç başa döndü → eski kayıtlar yanlış yerde; hayalet kapı üretirler."""
    m = EdgeBuoyMemory()
    # `Tespit` bir tuple takma adi: (x, y, r, sinif)
    m.siniflandir([(5.0, 2.0, 0.15, 0)], edge_class_id=0)
    assert m.boyut == 1, "kenar hatirlanmadi — test kurulumu yanlis"
    m.temizle()
    assert m.boyut == 0


def test_teshis_sayaclari_KORUNUYOR() -> None:
    """Koşum boyu kümülatif ölçüm — sıfırlanırsa hafızanın katkısı kayıt dışı."""
    m = EdgeBuoyMemory()
    m._hatirlanarak_kurtarilan = 7
    m._celiskiyle_silinen = 3
    m.temizle()
    assert m.hatirlanarak_kurtarilan == 7
    assert m.celiskiyle_silinen == 3


# ------------------------------------------------- PUAN SIFIRLAMASI (md 5.5.3.1)


def test_PUANLAR_sifirlaniyor() -> None:
    """🔴 MADDE #11'İN EN KRİTİK TESTİ.

    Şartname: *"Yeniden başlama hakkını kullanan takımın topladığı puanlar
    SIFIRLANACAKTIR."* İkinci turda AYNI geçitlerden yeniden geçilir. Geçiş
    hafızası temizlenmezse hepsi "zaten geçildi" sayılır → ikinci tur
    **hiç puan getirmez** ve bu sessizce olur.
    """
    from prototype.mission.gate_follower import GateFollower, GateFollowerConfig

    g = GateFollower(GateFollowerConfig(0.785, 1.04))
    g._passed_midpoints.extend([(10.0, 0.0), (20.0, 0.0)])
    g._gecilen_kapilar.append((10.0, 0.0, 1.0))

    g.reset()                                   # parkur geçişi: sayaç KORUNUR
    assert len(g._passed_midpoints) == 2, (
        "reset() puan kanitina dokunmamali (parkur gecisinde cagriliyor)"
    )

    g.reset_passed_gates()                      # yeniden başlama: sayaç GİDER
    assert len(g._passed_midpoints) == 0
    assert len(g._gecilen_kapilar) == 0, (
        "K1 listesi temizlenmezse ikinci turda hicbir kapi aday olamaz"
    )
