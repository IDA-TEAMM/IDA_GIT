"""F-F.22/F-F.23/F-F.24 — İLERİ TERCİHİ ve pivot kapısının kör noktaları.

ÖLÇÜM BAĞLAMI (17.08.2026 göl bandı `session_20260817_193312`, 12,5 dk,
GUIDED+ARMED 227 s):

    komut dağılımı   GERİ %23,1 · SIFIR %46,5 · İLERİ %30,4
    geri komut anı   hedef ARKADA %83,4 · kapı da ARKADA %88,6
                     kapı↔hedef zıt yarımkürede yalnız %7,3
    o anlarda pivot  KAPALI %91   ← F-F.20 devrede olsaydı komut çıkmazdı
    1. pencerede     hedefe 0,71 m yaklaştı, 177° döndü, sonra ARKASINA tam gaz

Bu süit üç kapıyı birden dondurur:
  * F-F.22 `ileri_kisit` / `w_ileri` — MPPI'nin geri gitme seçeneği
  * F-F.23 pivot yedek referansı — plan boşken kapı sessizce kapanmasın
  * F-F.24 `yakin_esik_m` — yakın alanda kerteriz singülaritesi

⚠ ÜÇÜ DE VARSAYILAN KAPALI. Buradaki ilk test tam bunu donduruyor: biri
varsayılan açılırsa (mutasyon) süit KIRMIZI döner.
"""
from __future__ import annotations

import math

import numpy as np
import pytest

from prototype.control.pivot_kapisi import PivotKapisi, PivotKapisiConfig
from prototype.dynamics.catamaran import CatamaranDynamics
from prototype.planning.mppi import MPPIConfig, MPPIController
from prototype.planning.rrt_star import Bounds

_SINIR = Bounds(-500.0, 500.0, -500.0, 500.0)


# ---------------------------------------------------------------- varsayılanlar

def test_VARSAYILANLAR_KAPALI_eski_davranis_birebir() -> None:
    """🔒 MUTASYON KAPISI: üç yeni şalter de varsayılan KAPALI olmalı.

    Açık varsayılan = ölçülmemiş bir davranışın sahaya sessizce girmesi.
    §0.8a: yeni yetenek önce A/B ile ölçülür, sonra varsayılan olur.
    """
    cfg = MPPIConfig()
    assert cfg.ileri_kisit is False
    assert cfg.w_ileri == 0.0
    assert PivotKapisiConfig().yakin_esik_m == 0.50


# ------------------------------------------------- F-F.22 sert kısıt: kip ayrımı

def _kontrolcu(**kw) -> MPPIController:
    return MPPIController(CatamaranDynamics(), _SINIR, [],
                          MPPIConfig(K=64, T=10, **kw))


@pytest.mark.parametrize(
    "girdi, beklenen, aciklama",
    [
        ((+1.0, +1.0), (+1.0, +1.0), "saf ileri — dokunulmaz"),
        ((-1.0, +1.0), (-1.0, +1.0), "SAF PİVOT (ortak=0) — dokunulmaz"),
        ((+1.0, -1.0), (+1.0, -1.0), "saf pivot, ters yön — dokunulmaz"),
        ((-1.0, -1.0), (0.0, 0.0), "saf GERİ — tamamen kesilir"),
        ((-1.0, +0.2), (-0.6, +0.6), "geri+dönüş → ortak sıfırlanır, fark kalır"),
        ((+0.2, -1.0), (+0.6, -0.6), "aynısı ters yön"),
    ],
)
def test_ileri_kisit_ortak_kipi_keser_fark_kipine_DOKUNMAZ(
    girdi, beklenen, aciklama
) -> None:
    """Kısıt yalnız (T_l+T_r)/2 < 0 hâlini kaldırır; dönme momenti korunur.

    🔑 En kritik satır SAF PİVOT: F-F.20'nin ürettiği [−a, +a] itkisinin
    ortak kipi tam sıfırdır. Kısıt onu bozsaydı iki kapı birbirini yerdi —
    pivot dönemez, tekne 14.08'in "ileri-geri saldırma" arızasına dönerdi.
    """
    c = _kontrolcu(ileri_kisit=True)
    U = np.array([[list(girdi)]], dtype=float)          # (1, 1, 2)
    cikti = np.asarray(c._as_numpy(c._ileri_kisitla(c.xp.asarray(U))))
    assert cikti[0, 0] == pytest.approx(beklenen, abs=1e-9), aciklama


def test_ileri_kisit_tavani_asmaz() -> None:
    """Ortak kip sıfıra çekilirken fark tavanı aşarsa KIRPILIR.

    `_batch_derivatives` girdiyi zaten ±max_thrust'a doyuruyor; kısıtın
    çıktısı o doyumla tutarlı olmak zorunda, yoksa maliyet hesabı ile
    fiilen uygulanan itki ayrışır.
    """
    c = _kontrolcu(ileri_kisit=True)
    max_T = c.p.max_thrust
    U = np.array([[[-3.0 * max_T, +max_T]]], dtype=float)
    cikti = np.asarray(c._as_numpy(c._ileri_kisitla(c.xp.asarray(U))))
    assert np.all(np.abs(cikti) <= max_T + 1e-12)


# ------------------------------------------- F-F.22 kapalı kol: bit-birebir aynı

def test_kisit_KAPALIYKEN_cikti_BIT_BIREBIR_ayni() -> None:
    """Şalter kapalıyken tek bir bit değişmemeli (geriye uyum sözleşmesi)."""
    hedef = [(float(i), 0.0) for i in range(40)]
    ciktilar = []
    for kisit in (False, False):
        c = MPPIController(CatamaranDynamics(), _SINIR, [],
                           MPPIConfig(K=128, T=10, seed=7, ileri_kisit=kisit))
        c.set_reference(hedef)
        st = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        ciktilar.append([c.step(st).copy() for _ in range(5)])
    for a, b in zip(*ciktilar):
        assert np.array_equal(a, b)


# -------------------------------------- F-F.22 açık kol: geri komut YOK (ölçüt)

def _hedef_arkada_kosumu(*, ileri_kisit: bool, adim: int = 40) -> list:
    """Hedefi teknenin ARKASINA koyup net ileri itkiyi toplar.

    17.08 bandındaki geri komutların %83,4'ü tam bu geometride (hedef
    |kerteriz| > 90°) üretildi — senaryo ölçümden alınmıştır, uydurma değil.
    """
    dyn = CatamaranDynamics()
    c = MPPIController(dyn, _SINIR, [],
                       MPPIConfig(K=256, T=20, seed=3, ileri_kisit=ileri_kisit))
    # Tekne +x'e bakıyor (ψ=0); referans −x yönünde, yani TAM ARKADA.
    c.set_reference([(-float(i) * 0.5, 0.0) for i in range(1, 60)])
    st = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    ortak = []
    for _ in range(adim):
        u = c.step(st)
        ortak.append(0.5 * (u[0] + u[1]))
        st = dyn.step_rk4(st, u, 0.05)
    return ortak


def test_hedef_ARKADAYKEN_kisit_ACIK_iken_GERI_KOMUT_YOK() -> None:
    """Ölçütün kendisi: net ileri itki hiçbir adımda negatif olamaz."""
    ortak = _hedef_arkada_kosumu(ileri_kisit=True)
    assert min(ortak) >= -1e-9, f"kısıt açıkken geri komut çıktı: {min(ortak)}"


def test_hedef_ARKADAYKEN_kisit_KAPALI_iken_GERI_KOMUT_CIKAR() -> None:
    """🔒 AYIRT EDİCİ TEST — testin gerçekten bir şey ölçtüğünün kanıtı.

    Bu test kırmızıya dönmüyorsa yukarıdaki yeşil test **vakumdur**: senaryo
    geri komut üretmiyor demektir ve kısıtın çalıştığını göstermez.
    (§0.31: "bir fonksiyonun doğru çalışması, ÇAĞRILDIĞI anlamına gelmez".)
    """
    ortak = _hedef_arkada_kosumu(ileri_kisit=False)
    assert min(ortak) < 0.0, (
        "kısıt KAPALIYKEN de geri komut çıkmadı — senaryo ayırt edici değil, "
        "yukarıdaki yeşil test hiçbir şey kanıtlamıyor"
    )


# ---------------------------------------- F-F.22 yumuşak terim (PreferForward)

def test_w_ileri_geri_giden_yorungeyi_CEZALANDIRIR() -> None:
    """Nav2 `PreferForwardCritic` karşılığı: `Σ_t max(−u_t, 0) · dt · w`."""
    c = _kontrolcu(w_ileri=100.0)
    K, T = 2, 10
    traj = np.zeros((K, T + 1, 6))
    traj[0, :, 3] = +0.5          # ileri giden yörünge
    traj[1, :, 3] = -0.5          # geri giden yörünge
    U = np.zeros((K, T, 2))
    maliyet = np.asarray(c._as_numpy(
        c._trajectory_cost(c.xp.asarray(traj), c.xp.asarray(U))))
    beklenen_fark = 100.0 * 0.5 * (T + 1) * c.cfg.dt
    assert maliyet[1] - maliyet[0] == pytest.approx(beklenen_fark, rel=1e-6)


def test_w_ileri_SIFIRKEN_maliyete_HIC_dokunmaz() -> None:
    c = _kontrolcu(w_ileri=0.0)
    traj = np.zeros((2, 11, 6))
    traj[1, :, 3] = -0.5
    U = np.zeros((2, 10, 2))
    maliyet = np.asarray(c._as_numpy(
        c._trajectory_cost(c.xp.asarray(traj), c.xp.asarray(U))))
    assert maliyet[0] == pytest.approx(maliyet[1])


# ------------------------------------------------- F-F.24 yakın alan körlüğü

def test_yakin_esik_kerteriz_singularitesini_kapatir() -> None:
    """Referans noktası eşiğin İÇİNDEYSE kapı açılmaz (hata `None`).

    17.08 izinde tekne hedefe 0,71 m'deyken kapı hâlâ açıktı ve kerteriz
    35° → 178° süpürdü. LOS literatürünün "circle of acceptance" değeri iki
    gemi boyu = 2 × 0,785 = 1,57 m; eski sabit 0,50 m onun üçte biriydi.
    """
    yakin = [(0.0, 1.0)]                    # araçtan 1,0 m — 0,50 dışı, 1,57 içi
    eski = PivotKapisi(PivotKapisiConfig(yakin_esik_m=0.50))
    aktif_eski, hata_eski = eski.guncelle(0.0, 0.0, 0.0, yakin)
    assert hata_eski is not None, "0,50 eşikte eski davranış korunmalı"
    assert aktif_eski is True                # 90° hata → kapı açılır

    olculen = PivotKapisi(PivotKapisiConfig(yakin_esik_m=1.57))
    aktif_yeni, hata_yeni = olculen.guncelle(0.0, 0.0, 0.0, yakin)
    assert hata_yeni is None, "1,57 eşikte yakın nokta kerterizi ÖLÇÜLMEMELİ"
    assert aktif_yeni is False


def test_yakin_esik_UZAK_noktayi_ETKILEMEZ() -> None:
    """Eşik yalnız yakın alanı kapatır; normal çalışma bandı bozulmaz."""
    uzak = [(0.0, 5.0)]
    kapi = PivotKapisi(PivotKapisiConfig(yakin_esik_m=1.57))
    aktif, hata = kapi.guncelle(0.0, 0.0, 0.0, uzak)
    assert hata == pytest.approx(math.pi / 2)
    assert aktif is True


# ------------------------------------------ F-F.23 yedek referans çekirdeği

def test_TEK_NOKTALI_yedek_referans_kapiyi_ACAR() -> None:
    """Yedek referans tek noktadır — kapı bunu kabul etmek ZORUNDA.

    `planning_node` plan boşken `[(hedef_x, hedef_y)]` veriyor. `_ufuk_noktasi`
    ufuk mesafesinden uzak nokta bulamazsa SON noktaya düşer; tek noktalı
    listede o nokta zaten hedefin kendisidir.
    """
    kapi = PivotKapisi(PivotKapisiConfig(ufuk_m=3.0))
    aktif, hata = kapi.guncelle(0.0, 0.0, 0.0, [(-10.0, 0.0)])
    assert hata is not None and abs(math.degrees(hata)) == pytest.approx(180.0)
    assert aktif is True


def test_referans_YOKKEN_kapi_KAPALI_kalir() -> None:
    """🔴 F-F.23'ün ölçtüğü kör nokta — düzeltme ÖNCESİ davranış.

    Bu davranış `pivot_kapisi` seviyesinde DOĞRUdur (neye döneceğini bilmeden
    dönmek kör sürmenin dönen hâlidir); yanlış olan, `planning_node`'un plan
    boşken elinde hedef VARKEN bile boş referans geçmesiydi. Bu test o
    sözleşmeyi dondurur: çekirdek asla kendi başına hedef uydurmaz.
    """
    kapi = PivotKapisi()
    for bos in (None, [], ()):
        aktif, hata = kapi.guncelle(0.0, 0.0, 0.0, bos)
        assert (aktif, hata) == (False, None)


# ------------------------------------ F-F.25 kapı SESSİZ KALAMAZ (sebep raporu)

def test_FF25_kapali_kapinin_SEBEBI_ayirt_edilir() -> None:
    """🔑 Üç ayrı "kapalı" hâli AYRI raporlanmalı — çareleri farklı.

    17.08 göl bandı: yön hatası ortanca 130° olan geri komutların %91'inde
    kapı kapalıydı; aynı bantta nöbetçi 43 kez `RRT-RED global plan
    uretilemedi` bastı. "Referans yoktu" kuvvetli adaydı ama kapı üç sebebi
    de aynı `(False, None)` ile döndürdüğü için **kanıtlanamadı**.
    """
    kapi = PivotKapisi(PivotKapisiConfig(yakin_esik_m=1.57))

    aktif, hata = kapi.guncelle(0.0, 0.0, 0.0, [])
    assert (aktif, hata) == (False, None)
    assert kapi.son_sebep == PivotKapisi.SEBEP_REFERANS_YOK   # RRT-RED adayı
    assert kapi.son_hata_derece is None

    aktif, hata = kapi.guncelle(0.0, 0.0, 0.0, [(0.0, 1.0)])  # 1,0 m — eşik içi
    assert (aktif, hata) == (False, None)
    assert kapi.son_sebep == PivotKapisi.SEBEP_COK_YAKIN      # F-F.24 vakası

    aktif, hata = kapi.guncelle(0.0, 0.0, 0.0, [(10.0, 0.0)])  # tam önde
    assert aktif is False and hata is not None
    assert kapi.son_sebep == PivotKapisi.SEBEP_HATA_KUCUK
    assert kapi.son_hata_derece == pytest.approx(0.0, abs=1e-9)

    aktif, _ = kapi.guncelle(0.0, 0.0, 0.0, [(-10.0, 0.0)])   # tam arkada
    assert aktif is True
    assert kapi.son_sebep == PivotKapisi.SEBEP_AKTIF
    assert kapi.son_hata_derece == pytest.approx(180.0)


def test_FF25_kapi_devre_disiyken_ayri_sebep() -> None:
    """`tetik_derece <= 0` (A/B için kapı tamamen kapalı) ayrı raporlanır —
    "referans yok" ile karıştırılırsa yanlış arıza aranır."""
    kapi = PivotKapisi(PivotKapisiConfig(tetik_derece=0.0))
    assert kapi.guncelle(0.0, 0.0, 0.0, [(-10.0, 0.0)]) == (False, None)
    assert kapi.son_sebep == PivotKapisi.SEBEP_KAPALI


def test_FF25_HATA_KUCUK_alarma_DONUSMEZ() -> None:
    """🔒 §7: "bir alarm her zaman yanıyorsa alarm değildir".

    `HATA-KUCUK` normal seyrin ta kendisi (araç hedefe bakıyor). Bunu da
    inhibit_reason'a yazmak, gerçek iki sessiz arızayı (`REFERANS-YOK`,
    `COK-YAKIN`) gürültüye gömerdi. Bu test o ayrımı dondurur.
    """
    raporlanan = (PivotKapisi.SEBEP_REFERANS_YOK, PivotKapisi.SEBEP_COK_YAKIN)
    assert PivotKapisi.SEBEP_HATA_KUCUK not in raporlanan
    assert PivotKapisi.SEBEP_AKTIF not in raporlanan


# ------------------------------------- F-F.26 RRT başarısızlık sebebi görünür

def test_FF26_rrt_sebebi_disari_aciliyor() -> None:
    """🔑 RRT* neden plan üretemedi — sayaç değil SEBEP.

    17.08 göl bandı: `RRT-RED global plan uretilemedi` **43 kez** ateşledi,
    sebep hiçbir kayıtta yoktu. Üç sebebin çaresi ayrı:
      goal engel içinde  → suçlu engel torbası (aynı bantta %98,6 UNKNOWN)
      start engel içinde → suçlu poz ya da torba
      çözüm bulamadı     → suçlu iterasyon bütçesi
    """
    from prototype.planning.pipeline import PlanningPipeline
    from prototype.planning.rrt_star import Bounds as B

    boru = PlanningPipeline(bounds=B(-100.0, 100.0, -100.0, 100.0))
    assert boru.son_rrt_sebep is None, "başlangıçta sebep OLMAMALI"

    # Hedefi devasa bir engelin tam içine koy → RRT* reddetmeli.
    from prototype.planning.rrt_star import CircleObstacle
    boru.set_obstacles([CircleObstacle(20.0, 0.0, 8.0)])
    boru.set_state([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    boru.set_waypoints([(20.0, 0.0)])

    assert boru.son_rrt_sebep is not None, (
        "RRT* hedefi engel içindeyken reddetmeli ve SEBEBİ yazmalı"
    )
    assert "goal" in boru.son_rrt_sebep or "çözüm" in boru.son_rrt_sebep
    assert boru.duz_cizgiye_dusuldu >= 1


# ------------------------------------------- F-F.27 hedef kurtarma (goal recovery)

def test_FF27_VARSAYILAN_KAPALI_eski_davranis() -> None:
    """🔒 MUTASYON KAPISI: kurtarma varsayılan kapalı, hedef engel içindeyse
    eski davranış (ValueError) birebir sürer."""
    from prototype.planning.rrt_star import (
        Bounds as B, CircleObstacle, RRTStar, RRTStarConfig,
    )
    assert RRTStarConfig().hedef_kurtarma_m == 0.0
    r = RRTStar(B(-50.0, 50.0, -50.0, 50.0), [CircleObstacle(20.0, 0.0, 5.0)],
                RRTStarConfig(seed=1))
    with pytest.raises(ValueError, match="goal"):
        r.plan((0.0, 0.0), (20.0, 0.0))


def test_FF27_hedef_engel_icindeyken_KURTARILIR() -> None:
    """🔑 17.08'de 43 kez ateşleyen yolun kapanması.

    Hedef engelin tam merkezinde; kurtarma açıkken plan ÜRETİLMELİ ve hedef
    engelin dışına, ona EN YAKIN noktaya taşınmış olmalı.
    """
    from prototype.planning.rrt_star import (
        Bounds as B, CircleObstacle, RRTStar, RRTStarConfig,
    )
    engel = CircleObstacle(20.0, 0.0, 5.0)
    cfg = RRTStarConfig(seed=1, hedef_kurtarma_m=8.0, safety_margin=0.5)
    r = RRTStar(B(-50.0, 50.0, -50.0, 50.0), [engel], cfg)
    yol = r.plan((0.0, 0.0), (20.0, 0.0))

    assert r.hedef_kurtarildi is not None, "hedef taşındı olarak İŞARETLENMELİ"
    eski, yeni, mesafe = r.hedef_kurtarildi
    assert eski == (20.0, 0.0)
    # Engel yarıçapı 5,0 + emniyet payı 0,5 ⇒ serbest bölge ≥ 5,5 m
    d = math.hypot(yeni[0] - engel.cx, yeni[1] - engel.cy)
    assert d >= engel.r + cfg.safety_margin - 1e-6, "yeni hedef HÂLÂ engel içinde"
    assert mesafe <= cfg.hedef_kurtarma_m, "azami taşıma mesafesi aşıldı"
    assert yol is not None and len(yol) >= 2


def test_FF27_azami_mesafe_ASILAMAZ() -> None:
    """Kurtarma bütçesi yetmiyorsa eski davranış (ValueError) sürer —
    hedefi görevin dışına taşımak, görevi sessizce değiştirmek olurdu."""
    from prototype.planning.rrt_star import (
        Bounds as B, CircleObstacle, RRTStar, RRTStarConfig,
    )
    r = RRTStar(B(-50.0, 50.0, -50.0, 50.0), [CircleObstacle(20.0, 0.0, 10.0)],
                RRTStarConfig(seed=1, hedef_kurtarma_m=2.0))
    with pytest.raises(ValueError, match="goal"):
        r.plan((0.0, 0.0), (20.0, 0.0))


def test_FF27_BASLANGIC_engel_icindeyse_KURTARILMAZ() -> None:
    """🔑 Başlangıç kurtarılmaz: o, aracın gerçekte nerede olduğu sorusudur.

    Poz engel içinde görünüyorsa ya poz yanlıştır ya torba — ikisini de
    hedefi oynatarak gizlemek, arızayı görünmez yapar (§7: "bir bekçinin
    gösterdiği yer, arızanın olduğu yer değildir" — ama bekçiyi susturmak
    da çözüm değildir).
    """
    from prototype.planning.rrt_star import (
        Bounds as B, CircleObstacle, RRTStar, RRTStarConfig,
    )
    r = RRTStar(B(-50.0, 50.0, -50.0, 50.0), [CircleObstacle(0.0, 0.0, 5.0)],
                RRTStarConfig(seed=1, hedef_kurtarma_m=8.0))
    with pytest.raises(ValueError, match="start"):
        r.plan((0.0, 0.0), (30.0, 0.0))


# ------------------------------------------------- F-F.28 kısmi plan (best-effort)

def _tikali_sahne():
    """Hedefin önü TAMAMEN kapalı — ölçülen 95 engelli göl yoğunluğunun özü."""
    from prototype.planning.rrt_star import Bounds as B, CircleObstacle
    duvar = [CircleObstacle(x, 20.0, 3.0) for x in range(-30, 31, 4)]
    return B(-40.0, 40.0, -10.0, 60.0), duvar


def test_FF28_VARSAYILAN_KAPALI_eski_davranis() -> None:
    """🔒 MUTASYON KAPISI: kısmi plan varsayılan kapalı → None döner."""
    from prototype.planning.rrt_star import RRTStar, RRTStarConfig
    assert RRTStarConfig().kismi_plan_min_ilerleme_m == 0.0
    sinir, engeller = _tikali_sahne()
    r = RRTStar(sinir, engeller, RRTStarConfig(seed=2, max_iter=800))
    assert r.plan((0.0, 0.0), (0.0, 40.0)) is None


def test_FF28_tikali_uzayda_KISMI_PLAN_uretilir() -> None:
    """🔑 17.08'de 43 kez düz çizgiye düşülen yolun çaresi.

    Hedefe varan yol YOK — ama ağacın hedefe en çok yaklaşan düğümüne kadar
    olan yol engelsizdir. Onu döndürmek, tıkalı alana düz çizgiyle dalmaktan
    her koşulda iyidir.
    """
    from prototype.planning.rrt_star import RRTStar, RRTStarConfig
    sinir, engeller = _tikali_sahne()
    cfg = RRTStarConfig(seed=2, max_iter=800, kismi_plan_min_ilerleme_m=3.0)
    r = RRTStar(sinir, engeller, cfg)
    yol = r.plan((0.0, 0.0), (0.0, 40.0))

    assert yol is not None, "tıkalı uzayda KISMİ plan üretilmeliydi"
    assert r.kismi_plan is not None
    ilerleme, kalan = r.kismi_plan
    assert ilerleme >= 3.0

    # 🔑 ASIL ŞART: kısmi yolun HER SEGMENTİ engelsiz olmalı — yoksa düz
    # çizgiden farkı kalmaz ve "iyileştirme" adı altında çarpma üretiriz.
    for (x1, y1), (x2, y2) in zip(yol, yol[1:]):
        assert r._segment_free(x1, y1, x2, y2), (
            f"kısmi planın ({x1:.1f},{y1:.1f})→({x2:.1f},{y2:.1f}) "
            "segmenti ENGEL İÇİNDEN geçiyor"
        )


def test_FF28_ilerleme_yetersizse_URETILMEZ() -> None:
    """Ağaç hedefe doğru anlamlı ilerlemediyse kısmi plan verilmez —
    yerinde sayan bir 'plan' operatörü yanıltır."""
    from prototype.planning.rrt_star import RRTStar, RRTStarConfig
    sinir, engeller = _tikali_sahne()
    cfg = RRTStarConfig(seed=2, max_iter=800,
                        kismi_plan_min_ilerleme_m=1000.0)   # imkânsız eşik
    r = RRTStar(sinir, engeller, cfg)
    assert r.plan((0.0, 0.0), (0.0, 40.0)) is None


# ---------------------------- F-P.30 uzamış cismi daire ZİNCİRİYLE temsil et

def _kiyi_sahnesi():
    """30 m'lik kıyı doğrusu + iki gerçek duba (r=0,15) — göl geometrisi."""
    r = np.random.default_rng(0)
    kiyi = np.stack([np.linspace(-15, 15, 120),
                     np.full(120, 18.0) + r.normal(0, 0.05, 120),
                     np.full(120, 0.4)], axis=1)

    def duba(cx, cy):
        a = np.linspace(0, 2 * math.pi, 20)
        return np.stack([cx + 0.15 * np.cos(a), cy + 0.15 * np.sin(a),
                         np.full(20, 0.3)], axis=1)

    return np.vstack([kiyi, duba(-3.0, 8.0), duba(3.0, 8.0)])


def _engeller(esik: float):
    from prototype.perception.lidar_obstacles import (
        LidarObstacleConfig, cluster_points, cluster_to_obstacle,
    )
    cfg = LidarObstacleConfig(split_max_yaricap_m=esik, cluster_tolerance=0.5,
                              min_cluster_size=3, split_cell_m=1.0)
    return [cluster_to_obstacle(c) for c in cluster_points(_kiyi_sahnesi(), cfg)]


def test_FP30_VARSAYILAN_KAPALI() -> None:
    """🔒 MUTASYON KAPISI: yayılım bölmesi varsayılan kapalı."""
    from prototype.perception.lidar_obstacles import LidarObstacleConfig
    assert LidarObstacleConfig().split_max_yaricap_m == 0.0
    assert max(e.radius for e in _engeller(0.0)) > 10.0, (
        "eski davranışta kıyı TEK DEV daire olmalı (ölçülen maks 17,2 m)"
    )


def test_FP30_kapsanan_alan_CARPICI_bicimde_kuculur() -> None:
    """Uzamış cisim daire zincirine bölününce boş su serbest kalır."""
    eski = sum(math.pi * e.radius ** 2 for e in _engeller(0.0))
    yeni = sum(math.pi * e.radius ** 2 for e in _engeller(2.0))
    assert yeni < eski / 10.0, (
        f"alan yeterince küçülmedi: {eski:.0f} → {yeni:.0f} m²"
    )


def test_FP30_HICBIR_OLCULEN_NOKTA_KAYBOLMAZ() -> None:
    """🔑 GÜVENLİK ŞARTI — bölme kapsamı DARALTAMAZ.

    Bu test olmadan F-P.30 bir "optimizasyon" değil, sessiz bir çarpma
    üreticisi olurdu: engel sayısını azaltmak kolay, kapsamı korumak zor.
    Ölçülen her nokta hâlâ BİR dairenin içinde kalmak zorunda.
    """
    noktalar = _kiyi_sahnesi()
    for esik in (0.0, 2.0, 1.0):
        engeller = _engeller(esik)
        for p in noktalar:
            assert any(
                math.hypot(p[0] - e.center_x, p[1] - e.center_y)
                <= e.radius + 1e-9 for e in engeller
            ), f"eşik {esik}: ({p[0]:.2f},{p[1]:.2f}) noktası KAPSANMIYOR"


def test_FP30_gercek_duba_yaricapini_BOZMAZ() -> None:
    """Duba (r=0,15) zaten küçük — bölme onu parçalamamalı."""
    engeller = _engeller(2.0)
    dubalar = [e for e in engeller if abs(e.center_y - 8.0) < 1.0]
    assert dubalar, "dubalar kayboldu"
    assert all(e.radius <= 0.45 for e in dubalar)


def test_FP30_saha_yuzeyine_BAGLI() -> None:
    """🔒 F-P.30 sahada ayarlanabilir olmalı — yoksa ölçülmüş bir düzeltme
    yeniden derlemeden denenemez (`edge_unutma_katsayisi` bu tuzağa düştü:
    parametre node'da açılmış ama launch/yaml'a HİÇ bağlanmamıştı).

    Dört yer birden bağlanır: çekirdek config · node · launch · hardware.yaml.
    """
    import ast
    from pathlib import Path
    kok = Path(__file__).resolve().parents[2]
    pkg = kok / "ros2_ws" / "src" / "girdap_decision"

    node = (pkg / "girdap_decision" / "perception_lidar_node.py").read_text()
    assert 'declare_parameter("split_max_yaricap_m"' in node
    assert "split_max_yaricap_m=float(" in node, "okunuyor ama CONFIG'e GEÇİRİLMİYOR"

    launch = (pkg / "launch" / "hardware.launch.py").read_text()
    assert '"split_max_yaricap_m"' in launch

    hw = (pkg / "config" / "hardware.yaml").read_text()
    assert "split_max_yaricap_m:" in hw

    from prototype.perception.lidar_obstacles import LidarObstacleConfig
    assert LidarObstacleConfig().split_max_yaricap_m == 0.0, "varsayılan KAPALI"
    del ast
