# -*- coding: utf-8 -*-
"""DAMGA = ÇEKİM ANI (2026-08-18) — `/perception/*` başlıklarının sözleşmesi.

🔴 NEDEN VAR: damga YAYIN anını taşıyordu. Bu kamerayla, dağıtım boru hattıyla
ölçüldü: çekim → yayın **202,4 ms** (cihaz+XLink 169,1 + 15 Hz döngümüz 33,3);
tüketicinin `sync_slop_s` değeri **100 ms**, yani hata slop'un **2,02 katı**.

Asıl bedel 18.08'de büyüdü: karar tarafı tespiti artık **damgadaki** poza göre
dünyaya çeviriyor (`planning_node._poz_damgada`). Geç damga ona 202 ms
sonrasının pozunu verdirir; hesaplandı — 30°/s dönüşte 8 m'deki duba **0,85 m**
kayar ve `edge_memory`nin **0,60 m** eşleşme bandını aşar ⇒ aynı duba YENİ
kayıt = **hayalet duba**. Üstelik SESSİZ: uyarı yalnız damga poz tamponunun
dışında kalırsa yanıyor, 202 ms tamponun (10 sn) tam içinde.
"""
import os
import sys
import types

import pytest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

pytest.importorskip("depthai", reason="depthai kurulu değil")
pytest.importorskip("rclpy", reason="rclpy kurulu değil")
pytest.importorskip("vision_msgs", reason="vision_msgs kurulu değil")

from girdap_ida_algi import duba_gecis_navigator as dgn  # noqa: E402

SIMDI_NS = 1_800_000_000 * 10**9


def _saniye(stamp) -> float:
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def _ns(yas=None):
    """`_damga`yı taşıyan en küçük sahte nesne — GERÇEK metot bağlanır."""
    n = types.SimpleNamespace(_tani={"damga_yedek": 0})
    n.get_clock = lambda: types.SimpleNamespace(
        now=lambda: types.SimpleNamespace(
            nanoseconds=SIMDI_NS,
            to_msg=lambda: _stamp(SIMDI_NS)))
    n._damga = types.MethodType(dgn.DubaNavigator._damga, n)
    n._yas = yas
    return n


def _stamp(ns_toplam):
    from builtin_interfaces.msg import Time as _T
    t = _T()
    t.sec = int(ns_toplam // 10**9)
    t.nanosec = int(ns_toplam % 10**9)
    return t


class _SahteMesaj:
    """depthai mesajı gibi `getTimestamp()` taşıyan sahte."""

    def __init__(self, damga_s):
        self._d = damga_s

    def getTimestamp(self):                       # noqa: N802 (depthai adı)
        import datetime
        return datetime.timedelta(seconds=self._d)


# ─────────────────────────────── ÇEKİRDEK: yaş çıkarılıyor mu ──────────────
def test_yas_damgadan_CIKARILIR():
    """Damga = şimdi − yaş. Bu testin kırılması = 202 ms'lik hatanın dönüşü."""
    simdi = SIMDI_NS / 1e9
    d = _ns()._damga(0.2024)
    assert _saniye(d) == pytest.approx(simdi - 0.2024, abs=1e-6)


def test_yas_SIFIRSA_damga_degismez():
    assert _saniye(_ns()._damga(0.0)) == pytest.approx(SIMDI_NS / 1e9, abs=1e-9)


# ─────────────────────────────── GERİ DÜŞÜŞ ────────────────────────────────
def test_yas_YOKSA_eski_davranis():
    """`getTimestamp` yoksa uydurma yapılmaz — yayın anı basılır, sayaç ARTMAZ.

    Sayaç artmamalı: "yaş hiç ölçülemedi" ile "yaş saçma geldi" farklı
    arızalar; ikisini aynı sayaçta toplamak sahada yanlış yere baktırır.
    """
    n = _ns()
    assert _saniye(n._damga(None)) == pytest.approx(SIMDI_NS / 1e9, abs=1e-9)
    assert n._tani["damga_yedek"] == 0


@pytest.mark.parametrize("yas", [-0.5, dgn.DAMGA_MAKUL_TAVAN_S + 0.1, 1e9])
def test_yas_SACMAYSA_eski_davranis_ve_SAYAC(yas):
    """Farklı zaman tabanı / sıfır damga → uydurma geçmişe damga BASILMAZ."""
    n = _ns()
    assert _saniye(n._damga(yas)) == pytest.approx(SIMDI_NS / 1e9, abs=1e-9)
    assert n._tani["damga_yedek"] == 1, "sessiz kalmamalı — sahada tek kanal bu"


def test_tavan_SINIRINDA_kabul():
    """Sınır dahil: tavan bir ret eşiği, keyfi bir tolerans değil."""
    n = _ns()
    d = n._damga(dgn.DAMGA_MAKUL_TAVAN_S)
    assert _saniye(d) == pytest.approx(SIMDI_NS / 1e9 - dgn.DAMGA_MAKUL_TAVAN_S,
                                       abs=1e-6)
    assert n._tani["damga_yedek"] == 0


# ─────────────────────────────── YAŞ ÖLÇÜMÜ ────────────────────────────────
def test_mesaj_yasi_monotonic_tabanindan_hesaplanir(monkeypatch):
    """`getTimestamp()` host steady_clock tabanlı ⇒ `time.monotonic()` ile ölç.

    ROS damgası DUVAR saatidir; doğrudan yazmak iki tabanı karıştırırdı.
    """
    monkeypatch.setattr(dgn.time, "monotonic", lambda: 100.0)
    assert dgn.DubaNavigator._mesaj_yasi(_SahteMesaj(99.8)) == pytest.approx(0.2)


def test_mesaj_yasi_damga_TASIMAYAN_mesajda_None():
    """Sürüm/mesaj tipi damga taşımıyorsa çökmek yok — None, sonra eski davranış."""
    assert dgn.DubaNavigator._mesaj_yasi(object()) is None


# ─────────────────────────── REGRESYON: kare kaybı ─────────────────────────
def test_yas_hatasi_KAREYI_DUSURMEZ():
    """🪤 İlk yazımda yaş hesabı, kareyi saklayan try'ın İÇİNDEYDİ ⇒ oradaki
    bir hata kareyi tamamen düşürüyordu = Dosya-1'de boşluk (md 4.2, ≥1 Hz,
    eksik dosya 5 ceza puanı). Damga, kaydın önüne ASLA geçmemeli."""
    class _Kuyruk:
        def __init__(self):
            self.k = [types.SimpleNamespace(getCvFrame=lambda: "KARE")]

        def tryGet(self):
            return self.k.pop() if self.k else None

    n = types.SimpleNamespace(rgb_q=_Kuyruk(), _son_kare=None, _kare_no=0,
                              _kare_yasi_s=None)
    n.get_logger = lambda: types.SimpleNamespace(
        warn=lambda *a, **k: None)
    n._mesaj_yasi = staticmethod(lambda m: (_ for _ in ()).throw(RuntimeError))
    n._kare_tazele = types.MethodType(dgn.DubaNavigator._kare_tazele, n)
    with pytest.raises(RuntimeError):
        n._kare_tazele()
    assert n._son_kare == "KARE", "kare yaş hatasına RAĞMEN saklanmalı"
    assert n._kare_no == 1, "kare sayacı yaş hatasından ETKİLENMEMELİ"


# ─────────────────────────── SÖZLEŞME: hangi yaş nereye ────────────────────
def test_buoys_TESPIT_yasini_targets_KARE_yasini_kullanir():
    """İki kaynak ayrı: tespit mesajı ↔ RGB passthrough. Karıştırılırsa
    damga yine kayar (ölçümde ikisi 202,4 ↔ 202,8 ms — yakın ama aynı değil)."""
    kaynak = open(dgn.__file__, encoding="utf-8").read()
    assert 'stamp = self._damga(getattr(self, "_tespit_yasi_s", None))' in kaynak
    assert ('arr.header.stamp = self._damga('
            'getattr(self, "_kare_yasi_s", None))') in kaynak
