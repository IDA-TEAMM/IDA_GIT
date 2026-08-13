"""
Girdap İDA — SAAT SIÇRAMASI nöbetçisi (§0.61, 13.08.2026).

🔴 NEYİ KORUYOR — canlı arıza, Jetson, 13.08 05:52:11:

    systemd-timesyncd: sistem saati +1497,6 s ADIMLANDI
    mavros_bridge:     FAILSAFE — heartbeat kaybı (1497.6s) → KILL
    fsm_node:          *** KILL — motorlar durduruluyor ***

Hat kopmamıştı. `/mavros/state` `connected: true`, uçuş kontrolcüsünde tek bir
failsafe mesajı yok, füzyon 76 ms sonra "poz kaynağı geri geldi" dedi. Aynı
saniyede heartbeat 1497,6 · GPS 1496,7 · poz 1496,8 · engel 1496,7 s bayatladı
— **hepsi aynı miktarda**. Bağımsız kaynakların aynı anda aynı kadar bayatlaması
veri kaybının değil SAAT ADIMININ imzasıdır. KILL mandallı olduğu için araç
yığın yeniden başlatılana kadar durdu. Aynı imza 11.08 16:05:17'de de var
(1195,6 s, yine ilk NTP adımı); kayıttaki diğer tüm KILL'ler 5,1-5,5 s = gerçek.

Sebep: her düğüm "ne kadar zaman geçti"yi `get_clock()` ile, yani DUVAR
SAATİYLE ölçüyordu. Jetson'un gerçek zaman saati tutmadığı için (§0.53e) saat
her açılışta yanlış başlıyor ve sonradan tek adımda düzeltiliyor.

⚠ NEDEN NÖBETÇİ GEREKLİ: kusur SESSİZ ve KAÇINILMAZ. Kod her yerde derde deva
görünüyor (`now - son_mesaj > esik`), hata yalnız saat adımlandığında ortaya
çıkıyor — ve o adım sahada GARANTİ (girdap-saat GPS fix'i beklerken açılış
saatiyle koşuluyor). Tek bir düğümde `time.monotonic` yerine `get_clock()`
geri dönerse hata da sessizce geri gelir.

Bu dosya ROS GEREKTİRMEZ: yapısal denetim düğüm dosyalarını düz metin okur,
davranış testleri sahte düğüm nesnesiyle koşar. Uçtan uca senaryo (sıçrama →
KILL yok) `test_mavros_bridge_node.py` içinde, gerçek düğümle.
"""

from __future__ import annotations

import time
from pathlib import Path

import pytest

# 🔴 13.08 — ROS PAKETİ KAPISI. Bu dosya `girdap_decision`'ı (ROS paketi)
# import ediyor ama kapısı yoktu; ROS'suz makinede (geliştirme laptopu,
# Fedora + py3.14) 5 test `ModuleNotFoundError` ile KIRMIZI dönüyordu.
# Gerçek bir arıza değil, ortam eksikliği — ama her koşumda 5 sahte kırmızı
# görmek, gerçek kırmızıyı fark etmemize engel olur. Repodaki diğer düğüm
# testlerinin tamamı (test_fsm_node, test_planning_node, …) bu kapıyı
# kullanıyor; bu dosya atlanmış.
pytest.importorskip(
    "girdap_decision",
    reason="girdap_decision yok (ROS'suz makine) — Jetson'da/ROS ortamında koş",
)

_PKG = (
    Path(__file__).resolve().parents[2]
    / "ros2_ws" / "src" / "girdap_decision" / "girdap_decision"
)
_SAAT_MODULU = "saat_kaynagi.py"

# "Ne kadar zaman geçti" ölçümünün duvar saatinden okunduğu kalıp. Yalnız
# saat_kaynagi.py'nin sim-zamanı kolunda bulunabilir.
_YASAK_KALIP = "get_clock().now().nanoseconds"


def _dugum_dosyalari() -> list[Path]:
    return sorted(
        p for p in _PKG.glob("*.py")
        if p.name not in {"__init__.py", _SAAT_MODULU}
    )


# ----------------------------------------------------------------- yapısal


def test_paket_dizini_bulundu() -> None:
    """Yol yanlışsa aşağıdaki nöbetçi sessizce 'geçer' — önce onu doğrula."""
    assert _PKG.is_dir(), f"paket dizini yok: {_PKG}"
    assert (_PKG / _SAAT_MODULU).exists(), "saat_kaynagi.py YOK — düzeltme geri alınmış"
    assert len(_dugum_dosyalari()) >= 10, "düğüm dosyaları bulunamadı (yol kaymış)"


def test_hicbir_dugum_gecen_sureyi_duvar_saatinden_okumaz() -> None:
    """§0.61: bayatlık/zaman aşımı ölçümü `get_clock()` ile YAPILMAZ.

    Damgalama (`get_clock().now().to_msg()`) serbesttir ve bilinçlidir — mesaj
    damgası mutlak an olmak ZORUNDA. Yasak olan `.nanoseconds` ile saniyeye
    çevirip fark almak; sıçrayan tek şey odur.
    """
    suclular = [
        p.name for p in _dugum_dosyalari()
        if _YASAK_KALIP in p.read_text(encoding="utf-8")
    ]
    assert not suclular, (
        f"{suclular} geçen süreyi DUVAR SAATİNDEN ölçüyor — saat adımlanınca "
        "sahte bayatlık/FAILSAFE üretir (§0.61). "
        "`saat_kaynagi.bayatlik_saati(self)` kullan."
    )


def test_damgalar_duvar_saatinde_KALIR() -> None:
    """Ters yön nöbetçisi: damgalar yanlışlıkla tek yönlü saate taşınmasın.

    Tek yönlü saatin başlangıcı keyfidir (makine açılışı). Mesaj damgasına ya da
    CSV `zaman` sütununa yazılırsa teslim dosyaları anlamsız olur (md 4.2).
    """
    planning = (_PKG / "planning_node.py").read_text(encoding="utf-8")
    assert "get_clock().now().to_msg()" in planning, (
        "planning_node damgaları duvar saatinden almıyor — Dosya-2/3 zaman "
        "sütunu anlamsızlaşır"
    )


# ------------------------------------------------------------- davranışsal


class _SahteParametre:
    def __init__(self, value) -> None:  # noqa: ANN001
        self.value = value


class _SahteDugum:
    """`bayatlik_saati`nin düğümden ihtiyaç duyduğu tek şey: use_sim_time."""

    def __init__(self, sim: bool, saat_degeri: float = 0.0) -> None:
        self._sim = sim
        self.saat_degeri = saat_degeri

    def get_parameter(self, ad: str):  # noqa: ANN201
        if ad == "use_sim_time":
            return _SahteParametre(self._sim)
        raise KeyError(ad)

    def get_clock(self):  # noqa: ANN201
        deger = self.saat_degeri

        class _Saat:
            def now(self):  # noqa: ANN201
                class _T:
                    nanoseconds = int(deger * 1e9)
                return _T()

        return _Saat()


def test_donanimda_tek_yonlu_saat_secilir() -> None:
    from girdap_decision.saat_kaynagi import bayatlik_saati

    assert bayatlik_saati(_SahteDugum(sim=False)) is time.monotonic


def test_sim_zamaninda_ros_saati_KALIR() -> None:
    """`ros2 bag play --clock` / Gazebo: zamanı `/clock` sürer.

    Orada duvar saati adımı diye bir sorun yok; buna karşılık tek yönlü saat
    sim durdurulduğunda akmaya devam eder ve bekçiler yalancı ateşler.
    """
    from girdap_decision.saat_kaynagi import bayatlik_saati

    dugum = _SahteDugum(sim=True, saat_degeri=42.0)
    saat = bayatlik_saati(dugum)
    assert saat is not time.monotonic
    assert saat() == pytest.approx(42.0)
    dugum.saat_degeri = 43.5
    assert saat() == pytest.approx(43.5)


def test_parametre_okunamazsa_donanim_varsayilir() -> None:
    """Parametre henüz yoksa güvenli taraf: sıçramaya bağışık saat."""
    from girdap_decision.saat_kaynagi import bayatlik_saati

    class _Parametresiz:
        def get_parameter(self, ad):  # noqa: ANN001, ANN201
            raise RuntimeError("parametre yok")

    assert bayatlik_saati(_Parametresiz()) is time.monotonic


def test_sicrama_bekcisi_adimi_yakalar(monkeypatch) -> None:  # noqa: ANN001
    """Sahadaki olayın birebir sayısı: duvar saati +1497,6 s adımlanıyor."""
    from girdap_decision import saat_kaynagi

    duvar = [1000.0]
    tek_yonlu = [500.0]
    monkeypatch.setattr(saat_kaynagi.time, "time", lambda: duvar[0])
    monkeypatch.setattr(saat_kaynagi.time, "monotonic", lambda: tek_yonlu[0])

    bekci = saat_kaynagi.SaatSicramaBekcisi(esik_s=2.0)

    # Normal akış: iki saat birlikte ilerliyor → sıçrama yok.
    duvar[0] += 1.0
    tek_yonlu[0] += 1.0
    assert bekci.kontrol() is None

    # NTP adımı: duvar saati 1497,6 s atlıyor, tek yönlü saat atlamıyor.
    duvar[0] += 1497.6 + 0.5
    tek_yonlu[0] += 0.5
    sapma = bekci.kontrol()
    assert sapma is not None
    assert sapma == pytest.approx(1497.6, abs=0.01)

    # Adım bir kez raporlanır; sonraki tur yeniden sessiz.
    duvar[0] += 1.0
    tek_yonlu[0] += 1.0
    assert bekci.kontrol() is None


def test_sicrama_bekcisi_geri_adimi_da_yakalar(monkeypatch) -> None:  # noqa: ANN001
    """Saat GERİ de alınabilir (girdap-saat GPS'ten kurunca) — işaretli döner."""
    from girdap_decision import saat_kaynagi

    duvar = [1000.0]
    tek_yonlu = [500.0]
    monkeypatch.setattr(saat_kaynagi.time, "time", lambda: duvar[0])
    monkeypatch.setattr(saat_kaynagi.time, "monotonic", lambda: tek_yonlu[0])

    bekci = saat_kaynagi.SaatSicramaBekcisi(esik_s=2.0)
    duvar[0] -= 60.0
    tek_yonlu[0] += 0.5
    sapma = bekci.kontrol()
    assert sapma is not None and sapma < 0
    assert sapma == pytest.approx(-60.5, abs=0.01)
