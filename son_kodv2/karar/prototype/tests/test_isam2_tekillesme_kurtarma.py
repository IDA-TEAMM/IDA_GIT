"""iSAM2 tekilleşme (IndeterminantLinearSystem) kurtarma testleri — F-F.1.

SAHA OLAYI (§0.42d, 11.08.2026, Jetson): `calculateEstimate()` gerçek GPS
akışında `Indeterminant linear system ... (Symbol: x1569)` fırlattı; istisna
`rclpy.spin`'e kadar çıkıp **fusion_node'u öldürdü** ve node geri gelmedi →
poz yayını kesildi → planning_node F-P.1 ile MPPI'yi durdurdu → araç sessizce
sürmez oldu. Bu testler o davranışın geri gelmesini engeller.

⚠ 12.08 NOTU — bu dosya Jetson'da COMMIT EDİLMEMİŞ ve UYGULAMASIZ duruyordu:
test 11.08 16:25'te yazılmış, `_is_indeterminant` ve kurtarma mantığı hiç
yazılmamıştı (git geçmişinin hiçbir yerinde yok, `.pyc` önbelleğinde bile izi
yok — bir kez bile import edilmemiş). Yani saha olayına karşı koruma
yazılacak diye başlanmış, yarım kalmış ve kimse fark etmemiş. Uygulama
12.08'de yazıldı, test olduğu gibi korundu ve repoya alındı.

Tekilleşmeyi gerçek GTSAM'da deterministik üretmek zor olduğundan `_isam`
sahte bir nesneyle değiştirilip istisna doğrudan enjekte ediliyor — test
edilen şey GTSAM'ın kendisi değil, **kurtarma sözleşmesi**.
"""

from __future__ import annotations

import pytest

gtsam = pytest.importorskip("gtsam")

from prototype.fusion.isam2_smoother import (  # noqa: E402
    ISAM2Smoother,
    _is_indeterminant,
)


_TEKILLESME_MESAJI = (
    "Indeterminant linear system detected while working near variable "
    "8646911284551353889 (Symbol: x1569)."
)


class _TekillesenISAM:
    """update() her çağrıda tekilleşme fırlatır; okuma yolu gerçek nesneye gider."""

    def __init__(self, gercek) -> None:
        self._gercek = gercek

    def update(self, *args):
        raise RuntimeError(_TEKILLESME_MESAJI)

    def calculateEstimate(self):
        return self._gercek.calculateEstimate()

    # 17.08: sicak yol tam Values yerine TEK ANAHTAR soruyor (O(N) kaldirildi).
    # Sahte, gercek nesnenin ayni cagrisina devrediyor — okuma yolu degismedi.
    def calculateEstimatePose2(self, key):
        return self._gercek.calculateEstimatePose2(key)


def _dolu_smoother(n: int = 5) -> ISAM2Smoother:
    sm = ISAM2Smoother()
    sm.initialize(gtsam.Pose2(0.0, 0.0, 0.0))
    sm.update()
    for i in range(n):
        sm.add_odometry(gtsam.Pose2(1.0, 0.0, 0.0))
        sm.add_gps(sm.latest_key, float(i + 1), 0.0)
        sm.update()
    return sm


def test_tekillesme_node_u_OLDURMEZ_poz_korunur():
    """Kurtarma sonrası update() istisna atmaz ve son iyi poz korunur."""
    sm = _dolu_smoother()
    iyi = sm.current_pose()

    sm._isam = _TekillesenISAM(sm._isam)
    sm.add_odometry(gtsam.Pose2(1.0, 0.0, 0.0))
    sm.update()  # REGRESYON: eskiden RuntimeError ile süreci öldürüyordu

    assert sm.recovery_count == 1
    kurtarilan = sm.current_pose()
    assert kurtarilan.x() == pytest.approx(iyi.x())
    assert kurtarilan.y() == pytest.approx(iyi.y())
    assert kurtarilan.theta() == pytest.approx(iyi.theta())


def test_kurtarma_sonrasi_odometri_zinciri_devam_eder():
    """Çapadan sonra normal akış sürer — `_latest_key` geri sarılmış olmalı.

    Geri sarılmazsa add_odometry var olmayan bir prev_key'e compose etmeye
    çalışır ve node ikinci kez ölür (bu test ilk yazımda tam bunu yakaladı).
    """
    sm = _dolu_smoother()
    sm._isam = _TekillesenISAM(sm._isam)
    sm.add_odometry(gtsam.Pose2(1.0, 0.0, 0.0))
    sm.update()

    # Kurtarma `_isam`'i ZATEN gerçek/boş bir ISAM2 ile değiştirdi (sahte
    # nesne düştü) — smoother doğrudan kullanılabilir durumda olmalı.
    assert not isinstance(sm._isam, _TekillesenISAM)
    capa = sm.current_pose()

    for i in range(3):
        sm.add_odometry(gtsam.Pose2(1.0, 0.0, 0.0))
        sm.add_gps(sm.latest_key, capa.x() + i + 1.0, 0.0)
        sm.update()

    assert sm.recovery_count == 1
    assert sm.current_pose().x() == pytest.approx(capa.x() + 3.0, abs=0.5)


def test_alakasiz_RuntimeError_YUTULMAZ():
    """Yalnız tekilleşme kurtarılır; başka hata sessizce yutulursa arıza gizlenir."""

    class _BaskaHata:
        def __init__(self, gercek) -> None:
            self._gercek = gercek

        def update(self, *args):
            raise RuntimeError("tamamen baska bir hata")

        def calculateEstimatePose2(self, key):
            return self._gercek.calculateEstimatePose2(key)

    sm = ISAM2Smoother()
    sm.initialize(gtsam.Pose2(0.0, 0.0, 0.0))
    sm.update()
    sm._isam = _BaskaHata(sm._isam)
    sm.add_odometry(gtsam.Pose2(1.0, 0.0, 0.0))

    with pytest.raises(RuntimeError, match="tamamen baska"):
        sm.update()
    assert sm.recovery_count == 0


def test_ilk_update_te_tekillesme_kurtarilamaz_istisna_yukselir():
    """Çözülmüş tahmin yokken sessizce sahte poz üretme — istisna yükselmeli."""
    sm = ISAM2Smoother()
    sm._isam = _TekillesenISAM(sm._isam)
    with pytest.raises(RuntimeError, match="Indeterminant"):
        sm.initialize(gtsam.Pose2(0.0, 0.0, 0.0))
    assert sm.recovery_count == 0


def test_is_indeterminant_mesaj_eslesmesi():
    """GTSAM istisnayı ayrı sınıf olarak vermiyor → tek ayırt edici mesaj metni."""
    assert _is_indeterminant(RuntimeError(_TEKILLESME_MESAJI))
    assert _is_indeterminant(RuntimeError("IndeterminantLinearSystemException"))
    assert not _is_indeterminant(RuntimeError("Attempting to at the key x6"))
    assert not _is_indeterminant(ValueError("gps_huber_k pozitif olmali"))
