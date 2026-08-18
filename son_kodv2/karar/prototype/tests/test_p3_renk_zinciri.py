"""Parkur-3 RENK ZİNCİRİ — kopuk halkaların testi (rclpy GEREKMEZ).

16.08'de P3 dalı main'e birleştirilince zincir "tamam" göründü ama **üç
halkası kopuktu** ve üçü de SESSİZDİ (hata basılmaz, tekne son waypoint'te
temiz durur, Parkur-3 = 0 → 145 puan, toplamın %48'i):

  1. `prototype/mission/renk_kodu.py` **YOKTU** — `planning_node._on_hedef_rengi`
     onu import ediyordu ama modül yalnız `girdap-ida-p3`'teydi ⇒ renk mesajı
     gelince ImportError. Callback `@_guard`'lı olduğu için node ölmüyor,
     renk **hiç uygulanmıyordu**.
  2. `/girdap/mission/hedef_rengi` topic'inin **YAYINCISI YOKTU** —
     `kamikaze_param.py`'nin docstring'i "kapı yayınlıyor" diyordu ama
     `create_publisher` hiç çağrılmıyordu.
  3. `p3_bekleniyor` hiçbir yerde **True yapılmıyordu** — FSM geçiş kuralı
     `mission_complete + p3_bekleniyor → PARKUR3`, ikinci şart asla sağlanmaz.

Buradaki testler üçünü de bağlar.
"""
from __future__ import annotations

import os

import pytest

from prototype.fsm.mission_fsm import MissionFSM, MissionState, Observation
from prototype.mission.renk_kodu import (
    ETIKET,
    IMZA,
    KOD_RENK,
    RENK_KOD,
    _anahtarla,
    dogrula,
    kod_dogru_mu,
)

# .../karar/prototype/tests/bu_dosya.py → .../karar
_KOK = os.path.dirname(os.path.dirname(
    os.path.dirname(os.path.abspath(__file__))))


# ── 1) sözleşme: karar tarafı ↔ p3 deposu AYNI olmalı ──────────────────────
def test_renk_tablosu_kanonik():
    """🔴 Tablo dört yerde elle kopyalanmıştı ve BİRİ TERSTİ (1=siyah).

    İHA "1" gönderip siyah demek isterken İDA kırmızıya saldırırdı — sessizce.
    Şartname s.25: 1 yanlış temas 100→50, 2 yanlış temas 100→**5**.
    """
    assert KOD_RENK == {0: None, 1: "kirmizi", 2: "yesil", 3: "siyah"}
    assert RENK_KOD == {"kirmizi": 1, "yesil": 2, "siyah": 3}
    assert IMZA == "p3renk-v1:0=yok,1=kirmizi,2=yesil,3=siyah"


def test_p3_deposuyla_AYRISMAMIS():
    """Kanonik kaynak `girdap-ida-p3/p3_hedef/renk_kodu.py`; burası kopya.

    Kopya sessizce ayrışırsa iki repo farklı renk konuşur. Dosya varsa
    karşılaştır; yoksa (başka makine) testi atla — ama sözleşme yine de
    yukarıdaki testle bağlı.
    """
    yol = os.path.expanduser("~/girdap-ida-p3/p3_hedef/renk_kodu.py")
    if not os.path.exists(yol):
        pytest.skip("girdap-ida-p3 bu makinede yok")
    ns: dict = {}
    exec(compile(open(yol, encoding="utf-8").read(), yol, "exec"), ns)
    assert ns["KOD_RENK"] == KOD_RENK, "İKİ REPO FARKLI RENK KONUSUYOR"
    assert ns["IMZA"] == IMZA
    assert ns["ETIKET"] == ETIKET


def test_dogrula_ayrisik_tabloyu_PATLATIR():
    """Sessiz yanlış eşleme, gürültülü çökmeden çok daha pahalı."""
    dogrula(KOD_RENK)                                   # doğru tablo: sessiz
    with pytest.raises(ValueError, match="AYRIŞMIŞ"):
        dogrula({0: None, 1: "siyah", 2: "kirmizi", 3: "yesil"})   # ters tablo


def test_kod_etiket_caprazlamasi():
    """Sayı tek başına sürüm ayrışmasını yakalayamaz; etiketle çaprazlanır."""
    assert kod_dogru_mu(1, "KIRMIZI-RAL3026")
    assert not kod_dogru_mu(1, "SIYAH-RAL9005"), "celiskili cift KABUL EDILDI"
    assert not kod_dogru_mu(2, "")


# ── operatör girdisi normalizasyonu ────────────────────────────────────────
@pytest.mark.parametrize("girdi", ["kirmizi", "KIRMIZI", " Kirmizi ", "kırmızı",
                                   "KIRMIZI ", "Kırmızı"])
def test_operator_yazimi_TOLERE_edilir(girdi):
    """🔴 Operatör `ros2 param set` ile Türkçe karakterle yazabilir.

    'kırmızı' ASCII'ye indirgenmezse tabloda BULUNAMAZ ⇒ renk sessizce
    atanmamış sayılır ⇒ P3 hiç açılmaz. Tam da kaçınmaya çalıştığımız arıza.
    """
    assert RENK_KOD.get(_anahtarla(girdi)) == 1


def test_bilinmeyen_renk_KOD_URETMEZ():
    """Yanlış renge saldırmaktansa hiç saldırmamak yeğdir."""
    for kotu in ("mavi", "turuncu", "", "   ", "1", "kirmizimsi"):
        assert RENK_KOD.get(_anahtarla(kotu)) is None


# ── 3) FSM kapısı: p3_bekleniyor ───────────────────────────────────────────
def _p2ye_getir() -> MissionFSM:
    """FSM'i PARKUR2'ye taşı (geçiş yolu testin konusu değil)."""
    fsm = MissionFSM()
    for durum in (MissionState.ARM, MissionState.BEKLEMEDE,
                  MissionState.PARKUR1, MissionState.PARKUR2):
        fsm._state = durum          # noqa: SLF001 — sabitleme, geçiş testi ayrı
    return fsm


def test_renk_YOKSA_P3E_GECILMEZ():
    """🔑 Bu DOĞRU davranış, eksiklik değil.

    İHA rengi bulamadıysa tekne son waypoint'te temiz durur ve P1+P2 puanı
    KORUNUR. Rastgele bir hedefe saldırmak 100→50 puan.
    """
    fsm = _p2ye_getir()
    fsm.tick(Observation(p2_waypoints_done=True, p3_bekleniyor=False))
    assert fsm.state is not MissionState.PARKUR3


def test_renk_VARSA_P3E_GECILIR():
    """Kopuk halka #3: `p3_bekleniyor` üretimde hiç True yapılmıyordu ⇒
    bu geçiş SAHADA ASLA gerçekleşmezdi."""
    fsm = _p2ye_getir()
    fsm.tick(Observation(p2_waypoints_done=True, p3_bekleniyor=True))
    assert fsm.state is MissionState.PARKUR3


def test_gorev_bitmeden_renk_TEK_BASINA_yetmez():
    """Renk kalkıştan önce yükleniyor (md s.22). Tek başına P3'ü açarsa
    tekne Parkur-2 bitmeden kamikaze moduna geçerdi.

    🔑 TETİK 14.08'de `p2_waypoints_done`'a taşındı (Yahya); renk kapısı ona
    EK bir şart — ikisi BİRLİKTE gerekiyor."""
    fsm = _p2ye_getir()
    fsm.tick(Observation(p2_waypoints_done=False, p3_bekleniyor=True))
    assert fsm.state is not MissionState.PARKUR3


# ── 2) yayıncı gerçekten var mı (kaynak sözleşmesi) ────────────────────────
def _kaynak(ad: str) -> str:
    yol = os.path.join(_KOK, "ros2_ws", "src", "girdap_decision",
                       "girdap_decision", ad)
    return open(yol, encoding="utf-8").read()


def test_renk_yayincisi_VAR_ve_LATCHED():
    """Kopuk halka #2. LATCH şart: renk kalkıştan ÖNCE bir kez yayınlanır;
    latch'siz topic'te geç açılan/yeniden başlayan abone onu SONSUZA KADAR
    kaçırır ve P3 sessizce ölür."""
    s = _kaynak("kamikaze_param.py")
    assert "/girdap/mission/hedef_rengi" in s
    assert "create_publisher" in s, "renk yayincisi YOK — P3 hic acilmaz"
    assert "latched_qos()" in s, "renk topic'i LATCH'siz — gec abone kacirir"


def test_fsm_node_rengi_DINLIYOR_ve_latched():
    """Kopuk halka #3'ün ROS ucu."""
    s = _kaynak("fsm_node.py")
    assert "/girdap/mission/hedef_rengi" in s, "fsm_node rengi dinlemiyor"
    assert "p3_bekleniyor" in s, "fsm_node p3_bekleniyor'u ayarlamiyor"
    assert "latched_qos()" in s


def test_param_degisince_YENIDEN_yayinlanir():
    """Operatör kalkıştan önce rengi değiştirirse yeni değer duyurulmalı;
    yoksa yığın eski rengi avlar."""
    s = _kaynak("kamikaze_param.py")
    govde = s.split("def _on_param_set", 1)[1].split("\n    def ", 1)[0]
    assert "_renk_yayinla()" in govde, "param degisince yeniden yayinlanmiyor"
