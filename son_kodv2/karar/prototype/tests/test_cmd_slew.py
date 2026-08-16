"""F-F.18 — `cmd_vel` eğim sınırlayıcı çekirdek testleri (ROS'SUZ).

Ölçüm bağlamı: GIRDAP_DURUM §0.99u. Testler hem davranışı hem de **güvenlik
sözleşmesini** dondurur (bkz. `test_planning_node.py::...bekci...`).
"""
from __future__ import annotations

import pytest

from prototype.control.cmd_slew import EgimSinirlayici, EgimSinirlayiciConfig


def _sinirlayici(ivme: float = 0.8, acisal: float = 0.0, azami_dt: float = 0.5):
    return EgimSinirlayici(
        EgimSinirlayiciConfig(
            azami_ivme_mps2=ivme,
            azami_acisal_ivme_rps2=acisal,
            azami_dt_s=azami_dt,
        )
    )


def test_ilk_cagri_hedefi_oldugu_gibi_gecirir() -> None:
    """Tohumlama: sıfırdan rampa BAŞLATMAZ — araç hareket hâlinde olabilir."""
    s = _sinirlayici()
    assert s.uygula(1.0, 0.3, 0.0) == (1.0, 0.3)


def test_ivme_tavani_uygulanir() -> None:
    s = _sinirlayici(ivme=0.8)
    s.uygula(0.0, 0.0, 0.0)
    # 0,1 s'te en fazla 0,08 m/s değişebilir
    u, _ = s.uygula(1.0, 0.0, 0.1)
    assert u == pytest.approx(0.08)
    u, _ = s.uygula(1.0, 0.0, 0.2)
    assert u == pytest.approx(0.16)


def test_olculen_en_kotu_sicrama_kirpilir() -> None:
    """14.08 ölçümü: 10 Hz'te 0,982 m/s'lik sıçrama görüldü. 0,8 m/s² sınırı
    bunu 0,08'e indirmeli — teknenin fiilen yapabildiği mertebeye."""
    s = _sinirlayici(ivme=0.8)
    s.uygula(0.05, 0.0, 0.0)
    u, _ = s.uygula(0.05 + 0.982, 0.0, 0.1)
    assert u == pytest.approx(0.13)


def test_hedefe_varinca_asmaz() -> None:
    """Küçük fark tavanın altındaysa hedef BİREBİR verilir (rampa uydurmaz)."""
    s = _sinirlayici(ivme=0.8)
    s.uygula(0.0, 0.0, 0.0)
    u, _ = s.uygula(0.01, 0.0, 1.0)
    assert u == pytest.approx(0.01)


def test_iki_yon_de_sinirlanir() -> None:
    """Azalma da sınırlanır — ama bekçi duruşları çekirdeğe HİÇ uğramaz
    (sözleşme çağıranda, bkz. modül docstring'i)."""
    s = _sinirlayici(ivme=0.8)
    s.uygula(1.0, 0.0, 0.0)
    u, _ = s.uygula(0.0, 0.0, 0.1)
    assert u == pytest.approx(0.92)


def test_sinir_sifirsa_kapali() -> None:
    """0 → eski davranış birebir (A/B ölçümü için)."""
    s = _sinirlayici(ivme=0.0)
    s.uygula(0.0, 0.0, 0.0)
    u, _ = s.uygula(5.0, 0.0, 0.001)
    assert u == pytest.approx(5.0)


def test_acisal_eksen_varsayilan_kapali_ve_bagimsiz() -> None:
    s = _sinirlayici(ivme=0.8, acisal=0.0)
    s.uygula(0.0, 0.0, 0.0)
    u, r = s.uygula(1.0, 2.0, 0.1)
    assert u == pytest.approx(0.08)     # doğrusal sınırlı
    assert r == pytest.approx(2.0)      # açısal serbest

    s2 = _sinirlayici(ivme=0.8, acisal=0.5)
    s2.uygula(0.0, 0.0, 0.0)
    _, r2 = s2.uygula(0.0, 2.0, 0.1)
    assert r2 == pytest.approx(0.05)


def test_uzun_bosluk_dt_kirpilir() -> None:
    """Yığın 10 s donarsa tek adımda 8 m/s'lik değişime izin verilmemeli."""
    s = _sinirlayici(ivme=0.8, azami_dt=0.5)
    s.uygula(0.0, 0.0, 0.0)
    u, _ = s.uygula(10.0, 0.0, 10.0)
    assert u == pytest.approx(0.4)      # 0,8 × 0,5 s


def test_geri_giden_saat_degistirmez() -> None:
    s = _sinirlayici()
    s.uygula(0.5, 0.0, 5.0)
    assert s.uygula(1.0, 0.0, 4.0) == (0.5, 0.0)


def test_sifirla_tohumlamayi_geri_getirir() -> None:
    """`sifirla()` sonrası ilk çağrı yine pass-through olmalı — aksi hâlde
    duruştan sonra araç hâlâ hareketliyken komut gereksiz kısılır."""
    s = _sinirlayici(ivme=0.8)
    s.uygula(0.0, 0.0, 0.0)
    s.sifirla()
    assert s.uygula(1.0, 0.0, 0.01) == (1.0, 0.0)
