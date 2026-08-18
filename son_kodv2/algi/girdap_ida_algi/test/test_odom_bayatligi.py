"""ODOM BAYATLIĞI + sessiz-ret alarmı — 2026-08-13 düzeltmelerinin mührü.

🔴 NEDEN BU TESTLER VAR (ölçümle bulundu, tahmin değil):

**1) Bayat poz, poz olmamasından DAHA KÖTÜYDÜ.**
`son_odom` yalnız mesaj GELDİĞİNDE güncelleniyor. Karar tarafı `fusion_node`
girdi akışı kesilince odom yayınını BİLEREK kesiyor (F8.2,
`fusion_node.py:560-568`, varsayılan `pose_timeout_s = 1.0` s; `hardware.yaml:83`
bunu açıkça uyarıyor). Yayın kesilince değer DONAR. Eski kod donuk pozu geçerli
sayıyordu ⇒ uzun pencereli geçit çizgisi kuruluyor, `gecitten_gecti` donuk pozla
HER ZAMAN False dönüyor, geçiş "doğrulanamadı" diye eleniyordu.

Aynı fiziksel geçişte ölçülen tutarsızlık:
    odom HİÇ gelmemiş -> geçit SAYILDI  (1)
    odom BAYAT/DONUK  -> geçit SAYILMADI (0)

Bedeli doğrudan puan: Parkur-1 `(G1/KD1)×10`, Parkur-2 `(G2/KD2)×40`.
"Doğrulanamadı" ile "geçmedi" aynı şey değildir — bayat pozda doğrulayacak
ÖLÇÜT yoktur, o hâlde odomsuz yolun zaten yaptığı şeye (zaman tahmini) düşülür.

**2) Sessiz-ret alarmı bir kez yanınca sönmüyordu.**
Tanı sayaçları kümülatif (bilerek) ama alarm doğrudan "toplam > 0"a bakıyordu ⇒
tek bir geçmiş red, sonraki HER turda alarmı yakıyordu. Bu, kendi yazılı
dersimizin ihlali: *"her zaman yanan alarm alarm değildir"* (09.08, mono_menzil).
Sahada SSH yok — journal tek görünürlük kanalımız; sürekli yanan uyarı GERÇEK
arızayı gizler.

⚠️ REGRESYON TARAFI PAZARLIKSIZ: odom SAĞLIKLIYKEN davranış bit birebir eski
olmalı — takılan araç hâlâ "geçti" sayılmamalı. Aşağıda ayrıca sınanır.
"""
import math
import os
import sys
import time
import types

import pytest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

pytest.importorskip("depthai", reason="depthai kurulu değil")
pytest.importorskip("rclpy", reason="rclpy kurulu değil")
pytest.importorskip("vision_msgs", reason="vision_msgs kurulu değil")

from girdap_ida_algi import duba_gecis_navigator as dgn  # noqa: E402


class _Logger:
    def __init__(self):
        self.warns = []

    def warn(self, m, **k):
        self.warns.append(m)

    def info(self, m, **k):
        pass

    def error(self, m, **k):
        pass


def _ns_gecis(son_odom, yas_sn=0.0, ilerleyen=False, gecerken_kesilir=False):
    """GECIS fazına girip zaman aşımına kadar koşan sahte node.

    İKİ AYRI BAYATLIK HÂLİ VAR, karıştırma:
      · `yas_sn` — geçişe girilirken odom ZATEN bayat ⇒ geçit çizgisi hiç
        kurulmaz, kod eski "odomsuz" yoluna (kısa pencere + zaman) düşer.
      · `gecerken_kesilir` — geçişe girilirken odom SAĞLIKLI (çizgi kurulur),
        tam geçerken akış kesilir ⇒ 2026-08-13'te eklenen dal burasıdır.
        Sahada beklenen hâl bu: yaklaşırken poz var, geçerken kesiliyor.
    """
    lg = _Logger()
    ns = types.SimpleNamespace(
        durum="YAKLASMA", gecit_cizgi=None, pass_bitis_t=0.0,
        gecit_sayisi=0, gecilen_gecitler=[], gorev_tamam=False,
        gecit_yari_gen=None, son_yari_gen=1.5,
        son_odom=son_odom, son_odom_t=time.monotonic() - yas_sn,
        _kayit_bozuk=True, _son_kayit_t=0.0, son_tespit_t=-math.inf,
        dubalar=[], son_gecit=None, son_taraf=1.0,
        arama_baslangic=time.monotonic(), _son_log=-math.inf,
        olculen_fps=8.0, kenar_cls=0, engel_cls=1, _tani_onceki={},
        _tani={k: 0 for k in ("dar", "dizili", "arada_duba", "menzil_celiski",
                              "buyuk_cisim", "mono_menzil", "menzil_yok")},
        tespitleri_oku=lambda: None, gecit_bul=lambda: None,
        durum_log=lambda: None,
        # 16.08: `dongu()` artık kareyi TEK YERDEN tazeliyor (P3 birleşmesi).
        # Bu test odom bayatlığını ölçüyor, kare akışını değil — sahte node'a
        # işlemsiz karşılık yeter. (Kare kuyruğunun kendi testi ayrı:
        # test_kare_kuyrugu.py)
        _kare_tazele=lambda: None,
        hedef_adimi=lambda simdi: None,
        gate_count_pub=types.SimpleNamespace(publish=lambda m: None),
        get_logger=lambda: lg,
    )
    ns.duruma_gec = lambda y: setattr(ns, "durum", y)
    ns.arac_poz_yaw = lambda timeout_s=0.0: dgn.DubaNavigator.arac_poz_yaw(ns)
    ns.cizgi_hedef_frame = lambda g: dgn.DubaNavigator.cizgi_hedef_frame(ns, g)

    dgn.DubaNavigator.gecis_baslat(ns, 2.0, (2.0, 0.0, 1.0, 0.0), 1.5)
    if gecerken_kesilir:                # çizgi kuruldu, sonra akış kesildi
        ns.son_odom_t = time.monotonic() - (dgn.ODOM_BAYAT_SN + 1.0)
    if ilerleyen:                       # araç geçidi GERÇEKTEN aştı, poz taze
        ns.son_odom = (2.0 + dgn.PASS_EK_YOL + 0.5, 0.0, 0.0)
        ns.son_odom_t = time.monotonic()
    ns.pass_bitis_t = time.monotonic() - 0.1     # pencere doldu
    dgn.DubaNavigator.dongu(ns)
    return ns, lg


@pytest.fixture(autouse=True)
def _algi_yayin_modu():
    eski = dgn.MOD
    dgn.MOD = "algi_yayin"
    yield
    dgn.MOD = eski


# ───────────────────────── bayatlık kapısı ─────────────────────────
def test_taze_odom_poz_olarak_kabul_edilir():
    ns = types.SimpleNamespace(son_odom=(1.0, 2.0, 0.3),
                               son_odom_t=time.monotonic())
    assert dgn.DubaNavigator.arac_poz_yaw(ns) == (1.0, 2.0, 0.3)


def test_bayat_odom_POZ_YOK_sayilir():
    """Eşiği aşan poz None döner — 'donuk pozu geçerli sayma' kuralı."""
    ns = types.SimpleNamespace(
        son_odom=(1.0, 2.0, 0.3),
        son_odom_t=time.monotonic() - (dgn.ODOM_BAYAT_SN + 0.5))
    assert dgn.DubaNavigator.arac_poz_yaw(ns) is None


def test_esik_karar_tarafinin_timeoutundan_BUYUK():
    """Karar tarafı pose_timeout_s=1.0 ile yayını keser; eşiğimiz payla üstünde
    olmalı, yoksa normal jitter'da poz 'bayat' sanılır."""
    assert dgn.ODOM_BAYAT_SN > 1.0


# ───────────────────────── geçit sayımı ─────────────────────────
def test_GECERKEN_odom_kesilirse_gecit_SAYILIR():
    """🔴 Asıl regresyon: eskiden 0 sayılıyordu (doğrudan puan kaybı).

    Sahadaki hâl: geçide yaklaşırken poz var (çizgi kurulur), tam geçerken
    `fusion_node` yayını keser (pose_timeout_s=1.0) ⇒ poz DONAR ⇒ eski kod
    `gecitten_gecti`yi donuk pozla sınayıp hep False alıyor, geçidi eliyordu.
    """
    ns, lg = _ns_gecis((0.0, 0.0, 0.0), gecerken_kesilir=True)
    assert ns.gecit_sayisi == 1
    assert any("BAYAT" in w for w in lg.warns), "sessizce sayılmamalı, uyarmalı"


def test_bastan_bayat_odomda_gecit_SAYILIR():
    """Geçişe girilirken odom zaten bayat ⇒ çizgi kurulmaz, odomsuz yol."""
    ns, _ = _ns_gecis((10.0, 5.0, 0.0), yas_sn=3.0)
    assert ns.gecit_sayisi == 1
    assert ns.gecit_cizgi is None, "bayat pozdan geçit çizgisi KURULMAMALI"


def test_odom_hic_yokken_gecit_SAYILIR():
    """Değişmedi — bayat hâliyle tutarlı olması gereken referans dal."""
    ns, _ = _ns_gecis(None)
    assert ns.gecit_sayisi == 1


def test_REGRESYON_saglikli_odom_gercek_gecis_sayilir():
    ns, _ = _ns_gecis((0.0, 0.0, 0.0), ilerleyen=True)
    assert ns.gecit_sayisi == 1


def test_REGRESYON_saglikli_odom_GECMEDIYSE_sayilmaz():
    """⚠️ PAZARLIKSIZ: takılan araca 'geçti' dedirtmek şartname G tanımını
    ihlal eder. Düzeltme bu dala DOKUNMAMALI."""
    ns, lg = _ns_gecis((0.0, 0.0, 0.0), ilerleyen=False)
    assert ns.gecit_sayisi == 0
    assert any("DOĞRULAMADI" in w for w in lg.warns)
    assert ns.durum == "ARAMA"


def test_gecerken_kesilen_dalda_tekrar_sayma_korumasi_calisir():
    """Bu dalda `gecit_cizgi` BİLEREK sıfırlanmıyor ⇒ orta nokta kaydedilir ⇒
    `yeni_gecit_mi` aynı geçidi ikinci kez saymaz (şartname G tanımı:
    'FARKLI karşılıklı kenar dubaları arasından geçiş sayısı')."""
    ns, _ = _ns_gecis((0.0, 0.0, 0.0), gecerken_kesilir=True)
    assert len(ns.gecilen_gecitler) == 1, "orta nokta kaydedilmeli"


# ───────────────────────── sessiz-ret alarmı ─────────────────────────
def _ns_log(tani, tani_onceki):
    lg = _Logger()
    ns = types.SimpleNamespace(
        _son_log=-math.inf, durum="ARAMA", gecit_sayisi=0, olculen_fps=8.0,
        gorev_tamam=False, kenar_cls=0, engel_cls=1,
        dubalar=[dgn.Duba(cls=0, x=-1.0, z=5.0, conf=0.9, cx=0.3, cy=0.5,
                          w=0.05, h=0.07),
                 dgn.Duba(cls=0, x=1.0, z=5.0, conf=0.9, cx=0.7, cy=0.5,
                          w=0.05, h=0.07)],
        _tani=dict(tani), _tani_onceki=dict(tani_onceki),
        get_logger=lambda: lg,
    )
    dgn.DubaNavigator.durum_log(ns)
    return ns, lg


_SIFIR = {k: 0 for k in ("dar", "dizili", "arada_duba", "menzil_celiski",
                         "buyuk_cisim", "mono_menzil", "menzil_yok")}


def test_alarm_YENI_redde_yanar():
    tani = dict(_SIFIR, dar=1)
    _, lg = _ns_log(tani, _SIFIR)
    assert any("kapı kurulamıyor" in w for w in lg.warns)


def test_alarm_GECMIS_redde_YANMAZ():
    """🔴 Asıl regresyon: eskiden bir kez yanınca bir daha sönmüyordu."""
    tani = dict(_SIFIR, dar=1)
    _, lg = _ns_log(tani, tani)          # sayaç aynı: yeni red YOK
    assert not any("kapı kurulamıyor" in w for w in lg.warns)


def test_alarm_mesaji_TOPLAMI_hala_gosterir():
    """Tetik artışa bağlandı ama tanı için toplam sahada lazım."""
    tani = dict(_SIFIR, dar=7)
    _, lg = _ns_log(tani, dict(_SIFIR, dar=6))
    assert any("dar=7" in w for w in lg.warns)


def test_alarm_mono_menzili_TETIKLEYICI_saymaz():
    """09.08 kuralı korunuyor: mono_menzil red sebebi değil, kurtarma."""
    tani = dict(_SIFIR, mono_menzil=5)
    _, lg = _ns_log(tani, _SIFIR)
    assert not any("kapı kurulamıyor" in w for w in lg.warns)
