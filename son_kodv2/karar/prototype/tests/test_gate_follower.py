"""
Girdap İDA — Duba kapısı takibi (gate_follower) çekirdek testleri.

select_gate (durumsuz seçim): kapı yok → None, geçerli kapı → orta nokta,
genişlik/derinlik/menzil elemeleri, en yakın kapının seçimi, arkadaki duba eleme,
left/right etiketleme. GateFollower (durumlu): fallback, kilitlenme, geçince
serbest bırakma, oklüzyonda koruma, drift güncelleme, reset.

Çalıştır: pytest prototype/tests/test_gate_follower.py -v
"""

from __future__ import annotations

import math

import pytest

from prototype.mission.gate_follower import (
    BUOY_RADIUS_M,
    ONAY_TICK,
    Gate,
    GateDiagnostics,
    GateFollower,
    GateFollowerConfig,
    select_gate,
)

_CFG = GateFollowerConfig()


# ------------------------------------------------------------ yardımcı

def _approx(a: tuple[float, float], b: tuple[float, float], tol: float = 1e-6) -> bool:
    return math.hypot(a[0] - b[0], a[1] - b[1]) <= tol


def _kilitlen(gf, arac, gn, dubalar, engeller=()):
    """Kapıya KİLİTLENENE kadar `update()` çağır; son sonucu döndür.

    B5 (2026-08-06): kilitlenme, aynı kapı `ONAY_TICK` kez üst üste görülene
    kadar ertelenir. Konusu onay penceresi OLMAYAN testler o pencereyi bu
    yardımcıyla geçer — böylece onay sayısı tek yerde yaşar ve testler
    kilitlenme sonrası davranışı ölçmeye devam eder.
    """
    res = None
    for _ in range(ONAY_TICK):
        res = gf.update(arac, gn, dubalar, engeller)
    assert gf.committed_gate is not None, "onay penceresi geçilemedi"
    return res


# ------------------------------------------------------------ select_gate: temel

def test_bos_liste_none() -> None:
    assert select_gate((0.0, 0.0), (0.0, 20.0), [], _CFG) is None


def test_tek_duba_none() -> None:
    # Kapı için en az 2 kenar dubası gerekir.
    assert select_gate((0.0, 0.0), (0.0, 20.0), [(-2.0, 10.0)], _CFG) is None


def test_arac_hedefin_ustunde_none() -> None:
    # Kurs yönü tanımsız (araç == GN) → seçim yapılamaz.
    assert select_gate((5.0, 5.0), (5.0, 5.0), [(-2.0, 10.0), (2.0, 10.0)], _CFG) is None


def test_temiz_kapi_orta_nokta() -> None:
    # Araç orijinde, GN kuzeyde (+y). Kapı: (-2,10) ve (2,10) → orta (0,10).
    gate = select_gate((0.0, 0.0), (0.0, 20.0), [(-2.0, 10.0), (2.0, 10.0)], _CFG)
    assert gate is not None
    assert _approx(gate.midpoint, (0.0, 10.0))
    assert gate.width == 4.0


def test_gn_kapinin_yaninda_yine_orta_noktadan_gecer() -> None:
    """Şartname md 5.5.2.2: GN tam kapı ortasında OLMAYABİLİR. GN yana kaymış
    olsa bile hedef, algılanan kapının ORTASI olmalı (ham GN değil)."""
    # Gerçek kapı ortası (0,10); ama hakem GN'yi (5,20) vermiş (5 m yanda).
    gate = select_gate((0.0, 0.0), (5.0, 20.0), [(-2.0, 10.0), (2.0, 10.0)], _CFG)
    assert gate is not None
    # Kapı hâlâ (0,10) civarı — GN'nin yan kayması hedefi kapıdan koparmadı.
    assert gate.midpoint[0] < 2.0        # ham GN'nin x=5'ine kaymadı
    assert 8.0 < gate.midpoint[1] < 12.0


# ------------------------------------------------------------ select_gate: elemeler

def test_genis_kapi_ARTIK_ELENMEZ() -> None:
    """🔑 2026-08-03: genişlik ÜST SINIRI kaldırıldı — tahmine dayalıydı.

    Kapı ne kadar geniş olursa olsun geçilebilir; şartname genişliğin yarışma
    alanına göre değişeceğini söylüyor, dolayısıyla bir üst sınır uydurmak
    gerçek kapıyı SESSİZCE reddetme riski demekti. 30 m'lik bir kapı artık
    geçerli (yan yana oldukları sürece)."""
    gate = select_gate(
        (0.0, 0.0), (0.0, 40.0), [(-15.0, 10.0), (15.0, 10.0)], _CFG
    )
    assert gate is not None
    assert gate.width == pytest.approx(30.0, abs=1e-6)


def test_govdeden_dar_acilik_elenir() -> None:
    """Tek kalan genişlik testi FİZİK: tekne sığmıyorsa kapı değildir.

    Bu bir eşik AYARI değil — gövde genişliği ölçülmüş bir tekne boyutudur.
    (Bu kadar yakın iki tespit pratikte tek dubanın ikiye bölünmesidir.)"""
    diag = GateDiagnostics()
    assert select_gate(
        (0.0, 0.0), (0.0, 20.0), [(-0.2, 10.0), (0.2, 10.0)], _CFG, diag
    ) is None
    assert diag.reddedilen_genislik[0] == pytest.approx(0.4, abs=1e-6)
    assert 0.4 < _CFG.hull_width_m          # gerçekten gövdeden dar


def test_kursa_dik_olmayan_cift_elenir() -> None:
    """Ardışık kapıların dubaları eşleşmemeli — |Δileri| < |Δyanal| testi.

    Ölçek-bağımsız: kapı genişliğini bilmeyi GEREKTİRMEZ, dolayısıyla
    tahmine dayalı bir tolerans içermez."""
    # Biri 5 m'de biri 15 m'de (Δileri=10), yanal ayrım 4 → 10 >= 4 → elenir.
    assert select_gate(
        (0.0, 0.0), (0.0, 30.0), [(-2.0, 5.0), (2.0, 15.0)], _CFG
    ) is None


def test_tek_dubasi_gorunmeyen_kapi_komsusuyla_eslesmez() -> None:
    """Asıl koruma senaryosu: A kapısının sağ dubası görünmüyor.

    En yakın iki duba A-sol + B-sol olur (ikisi de AYNI tarafta). Bunlar
    kurs boyunca dizili olduğu için |Δileri| >= |Δyanal| → elenir; yoksa
    orta nokta yana kayar ve tekne A-sol dubasının üstüne sürerdi."""
    # A-sol (-2, 10), B-sol (-2, 20) — A-sağ görünmüyor.
    assert select_gate(
        (0.0, 0.0), (0.0, 40.0), [(-2.0, 10.0), (-2.0, 20.0)], _CFG
    ) is None


def test_secilen_kapinin_genisligi_raporlanir() -> None:
    """Kapı bulunduğunda ölçülen genişlik teşhise yazılır (saha teyidi)."""
    diag = GateDiagnostics()
    gate = select_gate(
        (0.0, 0.0), (0.0, 20.0), [(-2.0, 10.0), (2.0, 10.0)], _CFG, diag
    )
    assert gate is not None
    assert diag.secilen_genislik == pytest.approx(4.0, abs=1e-6)
    assert diag.reddedilen_genislik == []


def test_arkadaki_dubalar_elenir() -> None:
    # Dubalar aracın ARKASINDA (-y, GN +y iken) → ileri projeksiyon negatif.
    assert select_gate(
        (0.0, 0.0), (0.0, 20.0), [(-2.0, -10.0), (2.0, -10.0)], _CFG
    ) is None


# ------------------------------------------------------------ select_gate: seçim

def test_en_yakin_kapi_secilir() -> None:
    # İki kapı: yakın (y=8) ve uzak (y=18). En yakın seçilmeli.
    buoys = [(-2.0, 8.0), (2.0, 8.0), (-2.0, 18.0), (2.0, 18.0)]
    gate = select_gate((0.0, 0.0), (0.0, 30.0), buoys, _CFG)
    assert gate is not None
    assert _approx(gate.midpoint, (0.0, 8.0))


def test_left_right_etiketleme() -> None:
    # GN kuzeyde: sol = -x tarafı (+lateral), sağ = +x tarafı.
    gate = select_gate((0.0, 0.0), (0.0, 20.0), [(-2.0, 10.0), (2.0, 10.0)], _CFG)
    assert gate is not None
    assert gate.left[0] < 0.0            # sol duba -x'te
    assert gate.right[0] > 0.0           # sağ duba +x'te


def test_dogu_yonunde_kurs() -> None:
    # GN doğuda (+x). Kapı dubaları y ekseninde ayrık, x=10'da.
    gate = select_gate((0.0, 0.0), (20.0, 0.0), [(10.0, -2.0), (10.0, 2.0)], _CFG)
    assert gate is not None
    assert _approx(gate.midpoint, (10.0, 0.0))


# ------------------------------------------------------------ GateFollower: fallback

def test_follower_kapi_yoksa_ham_gn() -> None:
    gf = GateFollower(_CFG)
    res = gf.update((0.0, 0.0), (0.0, 20.0), [])
    assert res.used_fallback is True
    assert _approx(res.target, (0.0, 20.0))
    assert res.gate is None


def test_follower_kapi_varsa_orta_nokta() -> None:
    gf = GateFollower(_CFG)
    res = _kilitlen(gf, (0.0, 0.0), (0.0, 20.0), [(-2.0, 10.0), (2.0, 10.0)])
    assert res.used_fallback is False
    assert _approx(res.target, (0.0, 10.0))
    assert gf.committed_gate is not None


# ------------------------------------------------------------ GateFollower: histerezis

def test_follower_gecince_serbest_birakir() -> None:
    gf = GateFollower(_CFG)
    buoys = [(-2.0, 10.0), (2.0, 10.0)]
    _kilitlen(gf, (0.0, 0.0), (0.0, 20.0), buoys)      # kapıya kilitlen
    assert gf.committed_gate is not None
    # Araç kapının ötesine geçti (y=10.5 > 10) → serbest bırakılmalı.
    gf.update((0.0, 10.5), (0.0, 20.0), [])            # kapı artık arkada + görünmüyor
    assert gf.committed_gate is None


def test_follower_okluzyonda_kapiyi_korur() -> None:
    # Kapıya kilitlendikten sonra bir tick duba GÖRÜNMEZSE (dalga/oklüzyon)
    # ve araç henüz geçmediyse hedef zıplamamalı — kilitli kapı korunur.
    gf = GateFollower(_CFG)
    buoys = [(-2.0, 10.0), (2.0, 10.0)]
    _kilitlen(gf, (0.0, 0.0), (0.0, 20.0), buoys)
    res = gf.update((0.0, 3.0), (0.0, 20.0), [])       # duba görünmüyor, henüz geçmedi
    assert res.used_fallback is False
    assert _approx(res.target, (0.0, 10.0))            # ham GN'ye düşmedi
    assert gf.committed_gate is not None


def test_follower_taze_algiyla_drift_gunceller() -> None:
    # Kapı biraz kaydıysa (dalga) ve match_radius içindeyse kilitli kapı
    # taze algıyla güncellenir (aynı kapı, drift).
    gf = GateFollower(_CFG)
    gf.update((0.0, 0.0), (0.0, 20.0), [(-2.0, 10.0), (2.0, 10.0)])
    res = gf.update((0.0, 1.0), (0.0, 20.0), [(-1.5, 10.5), (2.5, 10.5)])
    assert res.used_fallback is False
    # Orta nokta ~ (0.5, 10.5) — güncellendi, eski (0,10)'da kalmadı.
    assert res.target[1] > 10.0


def test_follower_reset_kilidi_temizler() -> None:
    gf = GateFollower(_CFG)
    _kilitlen(gf, (0.0, 0.0), (0.0, 20.0), [(-2.0, 10.0), (2.0, 10.0)])
    assert gf.committed_gate is not None
    gf.reset()
    assert gf.committed_gate is None


# ------------------------------------------------------------ Gate veri sınıfı

def test_gate_width_hesabi() -> None:
    g = Gate(left=(-3.0, 5.0), right=(3.0, 5.0), midpoint=(0.0, 5.0))
    assert g.width == 6.0


def test_aim_verilmezse_hedef_orta_noktadir() -> None:
    """Elle kurulan Gate'te `aim` yok → sürüş hedefi geometrik orta (uyumluluk)."""
    g = Gate(left=(-3.0, 5.0), right=(3.0, 5.0), midpoint=(0.0, 5.0))
    assert g.drive_target == (0.0, 5.0)
    assert g.aim_shift == pytest.approx(0.0, abs=1e-9)


# ------------------------------------------- geçilebilirlik: duba ÇAPI (B1)

def test_gecilebilirlik_duba_capini_hesaba_katar() -> None:
    """🔴 Dubalar MERKEZDEN algılanır; serbest açıklık `sep − 2r`.

    Yalnız gövde genişliğiyle karşılaştırmak süzgeci duba çapı kadar (30 cm)
    GEÇ kapatır: `[0.785 ; 1.085)` aralığındaki çiftler geçilemez oldukları
    hâlde kapı sayılır, araç sığmayacağı bir ortaya nişan alır ve İKİ dubaya
    birden çarpar. Üstelik tam bu aralık sahte çiftlerin (tek dubanın iki
    kümeye bölünmesi, su yansıması) düştüğü yerdir.

    ⚠ Eşik SABİT YAZILMAZ — gövde genişliği ölçülen bir sayıdır (09.08'de
    0.78 → 0.785 güncellendi). Test bir DEĞERİ değil İLİŞKİYİ donduruyor:
    "geçilebilir en dar merkez-mesafesi = gövde genişliği + duba ÇAPI".
    """
    assert _CFG.min_passable_width == pytest.approx(
        _CFG.hull_width_m + 2.0 * BUOY_RADIUS_M, abs=1e-9
    )
    # Duba çapı GERÇEKTEN hesaba giriyor (yalnız gövde genişliği değil).
    assert _CFG.min_passable_width - _CFG.hull_width_m == pytest.approx(
        0.30, abs=1e-9
    )
    diag = GateDiagnostics()
    # sep = 0.90 → gövdeden (0.785) GENİŞ ama duba yüzeyleri arası yalnız 0.60.
    assert select_gate(
        (0.0, 0.0), (0.0, 20.0), [(-0.45, 10.0), (0.45, 10.0)], _CFG, diag
    ) is None
    assert diag.reddedilen_genislik[0] == pytest.approx(0.9, abs=1e-6)
    assert 0.9 > _CFG.hull_width_m          # eski testi GEÇERDİ — asıl bulgu bu

    # sep = 1.20 → serbest açıklık 0.90 > 0.785, gövde sığar.
    gate = select_gate((0.0, 0.0), (0.0, 20.0), [(-0.6, 10.0), (0.6, 10.0)], _CFG)
    assert gate is not None


# ------------------------------------------- nişan noktası (engele göre orta)

def test_engel_yokken_nisan_TAM_ORTADA() -> None:
    """Geriye uyumluluk kilidi: engel yoksa davranış birebir eski hâli.

    Bozulursa kapısız/temiz senaryodaki tüm saha gözlemleri geçersizleşir.
    """
    gate = select_gate((0.0, 0.0), (0.0, 20.0), [(-2.0, 10.0), (2.0, 10.0)], _CFG)
    assert gate is not None
    assert _approx(gate.drive_target, gate.midpoint)
    assert _approx(gate.drive_target, (0.0, 10.0))


def test_engel_nisani_karsi_tarafa_iter() -> None:
    """Geçidin ağzındaki sarı duba nişanı boş tarafa kaydırır.

    Kenar dubaları MPPI'nin engel torbasında OLMADIĞI için geçitte iten tek
    kuvvet budur; kör orta noktada araç engelin üstüne sürerdi.
    """
    engel = [(-1.0, 10.0, 0.3)]           # solda, kapının içinde
    gate = select_gate(
        (0.0, 0.0), (0.0, 20.0), [(-2.0, 10.0), (2.0, 10.0)], _CFG,
        None, engel,
    )
    assert gate is not None
    assert _approx(gate.midpoint, (0.0, 10.0))       # kimlik değişmedi
    assert gate.drive_target[0] > 0.3                 # nişan SAĞA kaydı
    assert gate.drive_target[1] == pytest.approx(10.0, abs=1e-6)   # kiriş üstünde
    assert gate.aim_shift > 0.3


def test_nisan_govde_payli_bantta_KALIR() -> None:
    """Nişan hiçbir zaman dubaya `r + gövde/2`'den yakın olamaz.

    Devasa bir engel tek tarafa bastırsa bile araç direğe sürmemeli: bant
    fiziksel, engel maliyeti onu ezemez.
    """
    engel = [(-0.5, 10.0, 2.0)]           # kapının solunu tamamen kaplıyor
    gate = select_gate(
        (0.0, 0.0), (0.0, 20.0), [(-2.0, 10.0), (2.0, 10.0)], _CFG,
        None, engel,
    )
    assert gate is not None
    sinir = 2.0 - (0.15 + _CFG.half_beam)             # = 1.46 m
    assert -sinir - 1e-9 <= gate.drive_target[0] <= sinir + 1e-9


def test_en_dar_kapida_nisan_ortada_kalir() -> None:
    """Tam `min_passable_width` genişliğinde bant tek noktaya iner.

    Serbest yer sıfır → engel ne derse desin nişan ortadadır (kaymak
    gövdeyi dubaya sokardı).
    """
    w = _CFG.min_passable_width
    buoys = [(-w / 2.0, 10.0), (w / 2.0, 10.0)]
    gate = select_gate(
        (0.0, 0.0), (0.0, 20.0), buoys, _CFG, None, [(-0.4, 10.0, 0.2)]
    )
    assert gate is not None
    assert _approx(gate.drive_target, gate.midpoint, tol=1e-6)


def test_koridordaki_ucuncu_duba_nisani_iter() -> None:
    """Kapının içine sarkan üçüncü kenar dubası da nişanı iter.

    Seçilen çift yine (-2, 2) kapısıdır (orta noktası daha önde); üçüncü duba
    engel torbasında olmasa bile açıklık hesabına girer.
    """
    buoys = [(-2.0, 10.0), (2.0, 10.0), (-0.8, 10.1)]
    gate = select_gate((0.0, 0.0), (0.0, 20.0), buoys, _CFG)
    assert gate is not None
    assert _approx(gate.midpoint, (0.0, 10.0))       # doğru çift seçildi
    assert gate.drive_target[0] > 0.3                 # nişan sağa kaydı


def test_teshis_nisan_kaymasini_raporlar() -> None:
    """Saha teşhisi: kayma 0 → geçit temiz, kayma büyük → içinde bir şey var."""
    temiz = GateDiagnostics()
    select_gate((0.0, 0.0), (0.0, 20.0), [(-2.0, 10.0), (2.0, 10.0)], _CFG, temiz)
    assert temiz.aim_kaymasi_m == pytest.approx(0.0, abs=1e-9)

    engelli = GateDiagnostics()
    select_gate(
        (0.0, 0.0), (0.0, 20.0), [(-2.0, 10.0), (2.0, 10.0)], _CFG,
        engelli, [(-1.0, 10.0, 0.3)],
    )
    assert engelli.aim_kaymasi_m > 0.3


def test_follower_engelleri_gecirir_ve_kimlik_bozulmaz() -> None:
    """Durumlu katman: hedef nişan, kilit/eşleşme hâlâ geometrik ortadan.

    `midpoint` engelle birlikte kaysaydı "aynı kapı mı" eşleşmesi ve "geçildi
    mi" testi her tick'te kendi kendine kırılırdı.
    """
    gf = GateFollower(_CFG)
    buoys = [(-2.0, 10.0), (2.0, 10.0)]
    res = _kilitlen(gf, (0.0, 0.0), (0.0, 20.0), buoys, [(-1.0, 10.0, 0.3)])
    assert res.used_fallback is False
    assert res.gate is not None
    assert res.target[0] > 0.3                        # sürülen nokta kaydı
    assert _approx(res.gate.midpoint, (0.0, 10.0))    # kimlik sabit
    # Engel listesi verilmezse eski davranış birebir geri gelir.
    gf2 = GateFollower(_CFG)
    res2 = _kilitlen(gf2, (0.0, 0.0), (0.0, 20.0), buoys)
    assert _approx(res2.target, (0.0, 10.0))


# ------------------------------------- kapının KENDİ normali (B4/B6) + sayaç

def test_kapinin_kendi_normali_hesaplaniyor() -> None:
    """Normal kirişe DİK ve gidiş yönüyle aynı yarı düzlemde olmalı."""
    gate = select_gate((0.0, 0.0), (0.0, 20.0), [(-2.0, 10.0), (2.0, 10.0)], _CFG)
    assert gate is not None
    assert gate.normal is not None
    nx, ny = gate.normal
    assert math.hypot(nx, ny) == pytest.approx(1.0, abs=1e-9)
    assert _approx((nx, ny), (0.0, 1.0))                 # kurs +y → normal +y
    # Kirişe diklik: kiriş +x boyunca → normal·kiriş = 0
    assert nx == pytest.approx(0.0, abs=1e-9)


def test_isaretli_mesafe_duzlemin_iki_yanini_ayirir() -> None:
    gate = select_gate((0.0, 0.0), (0.0, 20.0), [(-2.0, 10.0), (2.0, 10.0)], _CFG)
    assert gate is not None
    assert gate.signed_distance((0.0, 3.0)) == pytest.approx(-7.0, abs=1e-6)
    assert gate.signed_distance((0.0, 12.0)) == pytest.approx(+2.0, abs=1e-6)


def test_GN_YANA_KACIKKEN_kilit_COZULUR() -> None:
    """🔴 B4/B6'nın asıl senaryosu — eski kurs-ekseni testinin kırıldığı yer.

    Şartname md 5.5.2.2: *"görev noktası doğrudan iki kenar dubasının arasında
    bir nokta OLMAYABİLİR."* GN kapının 20 m SAĞINDA olsun; araç kapıdan
    geçtikten sonra araç→GN ekseni hâlâ kapıyı ÖNDE gösterir → eski test
    kilidi çözmez, araç geçtiği kapıya GERİ DÖNERDİ.
    """
    gn = (8.0, 9.0)                        # GN kapının 8 m sağında, 1 m berisinde
    buoys = [(-2.0, 10.0), (2.0, 10.0)]
    gf = GateFollower(_CFG)
    _kilitlen(gf, (0.0, 0.0), gn, buoys)
    assert gf.committed_gate is not None

    arac = (0.0, 11.0)                     # kapı düzleminin 1 m ÖTESİNDE
    # Eski ölçüt bu geometride hâlâ "geçilmedi" der (araç→GN ekseni geriye
    # döndüğü için kapı ÖNDE görünür) — kilit çözülmez, araç geri dönerdi:
    fx, fy = (gn[0] - arac[0]), (gn[1] - arac[1])
    n = math.hypot(fx, fy)
    eski_fwd = (0.0 - arac[0]) * fx / n + (10.0 - arac[1]) * fy / n
    assert eski_fwd > 0.0, "eski testin kırıldığı durum kurulamadı"

    res = gf.update(arac, gn, [])          # kapı görünmüyor, geçtik
    assert gf.committed_gate is None       # kilit ÇÖZÜLDÜ
    assert res.used_fallback is True       # ham GN'ye dönüldü
    assert gf.passed_gate_count == 1       # ve geçiş SAYILDI


def test_yandan_DOLASILAN_kapi_sayilmaz_ama_kilit_birakilir() -> None:
    """Düzlemi aşmak yetmez: direklerin ARASINDAN geçilmiş olmalı (G1/G2)."""
    buoys = [(-2.0, 10.0), (2.0, 10.0)]
    gf = GateFollower(_CFG)
    _kilitlen(gf, (0.0, 0.0), (0.0, 20.0), buoys)
    # Kapının 9 m sağından dolaşarak düzlemi geç (yanal 9 > yarı genişlik 2).
    gf.update((9.0, 11.0), (0.0, 20.0), [])
    assert gf.committed_gate is None            # kilit bırakıldı (arkada kaldı)
    assert gf.passed_gate_count == 0            # ama SAYILMADI


def test_ayni_kapidan_tekrar_gecis_SAYILMAZ() -> None:
    """Manevra/geri dönüş aynı kapıyı ikinci kez saydırmamalı.

    ⚠ K1 (2026-08-06) bu senaryoyu bir adım ÖNCEDEN kapatıyor: geçilen kapı
    artık yeniden aday bile olmuyor, dolayısıyla "geri dön + tekrar kilitlen"
    yolu hiç açılmıyor. Sayaç güvencesi (aynı kapı = 1) aynen korunuyor;
    burada ikisi birden donduruluyor.
    """
    buoys = [(-2.0, 10.0), (2.0, 10.0)]
    gf = GateFollower(_CFG)
    _kilitlen(gf, (0.0, 0.0), (0.0, 20.0), buoys)
    gf.update((0.0, 11.0), (0.0, 20.0), buoys)      # geçti → 1
    assert gf.passed_gate_count == 1
    for _ in range(ONAY_TICK + 1):                  # geri döndü, kapı görünüyor
        gf.update((0.0, 5.0), (0.0, 20.0), buoys)
    assert gf.committed_gate is None                # K1: yeniden KİLİTLENMEZ
    gf.update((0.0, 11.0), (0.0, 20.0), buoys)      # düzlemi tekrar geçse bile
    assert gf.passed_gate_count == 1                # hâlâ 1 — aynı kapı


def test_ardisik_IKI_kapi_ayri_sayilir() -> None:
    """Parkur-2 şartı: en az 2 FARKLI duba ikilisi (md 5.5.2.4)."""
    k1 = [(-2.0, 10.0), (2.0, 10.0)]
    k2 = [(-2.0, 25.0), (2.0, 25.0)]
    gf = GateFollower(_CFG)
    _kilitlen(gf, (0.0, 0.0), (0.0, 40.0), k1)
    gf.update((0.0, 11.0), (0.0, 40.0), k1)         # 1. kapı geçildi
    _kilitlen(gf, (0.0, 20.0), (0.0, 40.0), k2)     # 2. kapıya kilitlen
    gf.update((0.0, 26.0), (0.0, 40.0), k2)         # 2. kapı geçildi
    assert gf.passed_gate_count == 2


def test_yeniden_baslama_sayaci_sifirlar_reset_ETMEZ() -> None:
    """reset() kilidi temizler, sayacı DEĞİL (puan kanıtı parkur geçişinde durur)."""
    buoys = [(-2.0, 10.0), (2.0, 10.0)]
    gf = GateFollower(_CFG)
    _kilitlen(gf, (0.0, 0.0), (0.0, 20.0), buoys)
    gf.update((0.0, 11.0), (0.0, 20.0), buoys)
    assert gf.passed_gate_count == 1
    gf.reset()
    assert gf.passed_gate_count == 1                # parkur geçişi sayacı bozmaz
    gf.reset_passed_gates()                          # md 5.5.3.1 yeniden başlama
    assert gf.passed_gate_count == 0


# --------------------------------- B5: kilitlenmeden önce onay (2026-08-06)

def test_B5_tek_karelik_hayalet_kapi_KILITLENMEZ() -> None:
    """🔴 B5'in asıl gerekçesi: bir karelik yanlış tespit KALICI hedef olmasın.

    Onay kapısı yokken hayalet kapı ANINDA kilitleniyordu; sonraki tick'te algı
    onu görmese bile **oklüzyon koruması** (kendisi doğru bir davranış) kilitli
    kapıyı saklıyordu → araç, hiç var olmayan bir kapının düzlemini fiilen
    geçene kadar oraya sürüyordu. İki doğru davranışın kesişimindeki arıza.
    """
    gf = GateFollower(_CFG)
    hayalet = [(-2.0, 10.0), (2.0, 10.0)]
    res = gf.update((0.0, 0.0), (0.0, 20.0), hayalet)   # tek kare göründü
    assert gf.committed_gate is None                    # kilitlenmedi
    assert res.used_fallback is True
    assert _approx(res.target, (0.0, 20.0))             # hedef hâlâ ham GN
    res = gf.update((0.0, 1.0), (0.0, 20.0), [])        # hayalet kayboldu
    assert gf.committed_gate is None                    # hiç kilitlenmedi
    assert res.used_fallback is True


def test_B5_kapi_ONAY_TICK_kez_gorulunce_kilitlenir() -> None:
    """Kalıcı kapı tam `ONAY_TICK`'inci tick'te kilitlenir — ne erken ne geç."""
    gf = GateFollower(_CFG)
    buoys = [(-2.0, 10.0), (2.0, 10.0)]
    for tick in range(1, ONAY_TICK):
        gf.update((0.0, 0.0), (0.0, 20.0), buoys)
        assert gf.committed_gate is None, f"{tick}. tick'te ERKEN kilitlendi"
    res = gf.update((0.0, 0.0), (0.0, 20.0), buoys)     # ONAY_TICK'inci tick
    assert gf.committed_gate is not None
    assert res.used_fallback is False
    assert _approx(res.target, (0.0, 10.0))


def test_B5_aday_degisirse_sayac_SIFIRLANIR() -> None:
    """Dönüşümlü iki farklı kapı: hiçbiri ÜST ÜSTE onaylanmaz → kilit yok.

    Onay 'toplam kaç kez görüldü' değil 'kaç kez ÜST ÜSTE aynı kapı görüldü'
    olmalı; yoksa iki ayrı titrek tespit birbirini onaylardı.
    """
    gf = GateFollower(_CFG)
    a = [(-2.0, 10.0), (2.0, 10.0)]
    b = [(-2.0, 30.0), (2.0, 30.0)]        # 20 m ötede: yarı genişlikten uzak
    for _ in range(3 * ONAY_TICK):
        gf.update((0.0, 0.0), (0.0, 40.0), a)
        gf.update((0.0, 0.0), (0.0, 40.0), b)
    assert gf.committed_gate is None


def test_B5_teshis_onay_sayacini_raporlar() -> None:
    """Node 'kapıyı görüyor ama kilitlenmiyor' hâlini bu alandan ayırt eder."""
    gf = GateFollower(_CFG)
    gf.update((0.0, 0.0), (0.0, 20.0), [(-2.0, 10.0), (2.0, 10.0)])
    assert gf.last_diagnostics.aday_onay_sayaci == 1


def test_B5_reset_aday_penceresini_de_temizler() -> None:
    """Parkur geçişinde yarım kalmış aday yeni parkura TAŞINMAMALI."""
    gf = GateFollower(_CFG)
    buoys = [(-2.0, 10.0), (2.0, 10.0)]
    gf.update((0.0, 0.0), (0.0, 20.0), buoys)          # aday sayacı 1
    gf.reset()
    gf.update((0.0, 0.0), (0.0, 20.0), buoys)          # yeniden 1'den başlar
    assert gf.committed_gate is None
    assert gf.last_diagnostics.aday_onay_sayaci == 1


# ------------------------- seçim ekseni: çiftin kendi çerçevesi (2026-08-06)

def test_GN_YANA_KACIKKEN_gercek_kapi_REDDEDILMEZ() -> None:
    """🔴 Seçim ekseni kalıntısı — kurs ekseninde ölçülen 45° sınırının kırdığı yer.

    Şartname md 5.5.2.2: *"görev noktası doğrudan iki kenar dubasının arasında
    bir nokta OLMAYABİLİR."* GN yana kaçınca kurs ekseni döner ve gerçek kapı
    "kursa dik değil" diye **sessizce** reddedilirdi (hata basılmaz, araç ham
    GN'ye gider, geçiş puanı kaybedilir). Yeni ölçüm çerçevesi çiftin kendi
    bakış hattıdır → GN'nin nerede olduğu sonucu değiştirmez.
    """
    arac, gn = (0.0, 0.0), (12.0, 10.0)      # GN, kapı yönünden ~50° sapmış
    buoys = [(-2.0, 10.0), (2.0, 10.0)]      # gerçek kapı, orta (0,10)

    # Önce ESKİ ölçütün bu geometride elediğini göster (kurs ekseninde):
    n = math.hypot(gn[0], gn[1])
    fx, fy = gn[0] / n, gn[1] / n
    lx, ly = -fy, fx
    dx, dy = buoys[0][0] - buoys[1][0], buoys[0][1] - buoys[1][1]
    assert abs(dx * fx + dy * fy) >= abs(dx * lx + dy * ly), (
        "eski kurs-ekseni ölçütünün kırıldığı durum kurulamadı"
    )

    gate = select_gate(arac, gn, buoys, _CFG)
    assert gate is not None                              # artık REDDEDİLMİYOR
    assert _approx(gate.midpoint, (0.0, 10.0))


def test_GN_YANDAYKEN_de_ardisik_kapi_dubalari_HALA_ELENIR() -> None:
    """Yeni çerçeve korumayı ZAYIFLATMADI: aynı taraftaki iki duba eşleşmemeli.

    A-sol + B-sol tuzağı (tek dubası görünmeyen kapının komşusuyla eşleşmesi)
    GN yana kaçıkken de elenmeli — yoksa orta nokta yana kayar ve tekne A-sol
    dubasının üstüne sürer.
    """
    diag = GateDiagnostics()
    assert select_gate(
        (0.0, 0.0), (12.0, 10.0), [(-2.0, 10.0), (-2.0, 20.0)], _CFG, diag
    ) is None
    assert diag.reddedilen_derinlik == 1                 # bakış hattı BOYUNCA


# --------------------------------- K1: geçilen kapı bir daha aday olamaz (2026-08-06)
#
# Ölçülen arıza (GIRDAP_DURUM §0.9b): araç kapıya EĞİK yaklaşınca öndeki gerçek
# kapı (b) testine takılıyor, geriye tek geçerli aday olarak ARKADAKİ kapı
# kalıyor ("önde mi" süzgeci kurs eksenini kullanıyor ve o eksen GN'ye doğru
# ~90° dönmüş oluyor) → araç geri dönüyor → sonsuz salınım. Aşağıdaki geometri
# kapalı-döngü koşumundan BİREBİR alındı (t=55 s anı).

_K1_ARAC = (40.81, 0.76)          # kapı-2'nin yanında, ona eğik bakıyor
_K1_GN = (40.0, 5.0)              # ham görev noktası (kapı ortasında DEĞİL)
_K1_KAPI1 = [(20.0, 2.0), (20.0, -2.0)]       # GEÇİLMİŞ kapı (arkada)
_K1_KAPI2 = [(40.0, 5.25), (40.0, 2.75)]      # asıl hedef (yanımızda)


def test_K1_ESKI_TUZAK_gecilmis_liste_YOKKEN_arkadaki_kapi_secilir() -> None:
    """Düzeltmenin gerekçesini donduran test: liste verilmezse tuzak geri gelir.

    Bu, düzeltilen davranışın BELGESİDİR — `gecilmis` boşken seçim, araç
    kapı-2'nin yanındayken 20 m GERİDEKİ kapı-1'i veriyor.
    """
    diag = GateDiagnostics()
    gate = select_gate(_K1_ARAC, _K1_GN, _K1_KAPI1 + _K1_KAPI2, _CFG, diag)
    assert gate is not None
    assert _approx(gate.midpoint, (20.0, 0.0)), "tuzak kurulamadı"
    assert diag.reddedilen_derinlik >= 1, "kapı-2 diklik testinde elenmeliydi"


def test_K1_gecilmis_kapi_YENIDEN_SECILMEZ() -> None:
    """Aynı geometri + 'kapı-1 zaten geçildi' bilgisi → geri dönüş YOK."""
    diag = GateDiagnostics()
    gate = select_gate(
        _K1_ARAC, _K1_GN, _K1_KAPI1 + _K1_KAPI2, _CFG, diag,
        gecilmis=[(20.0, 0.0, 2.0)],
    )
    assert gate is None                       # ham GN'ye düşülür (kendini düzeltir)
    assert diag.reddedilen_gecilmis == 1


def test_K1_ILERIDEKI_kapi_ETKILENMEZ() -> None:
    """Eleme yalnız ARKADAKİNİ vurmalı; öndeki kapı hâlâ seçilebilir olmalı."""
    ileri = [(0.0, 30.0), (4.0, 30.0)]
    gate = select_gate(
        (2.0, 0.0), (2.0, 40.0), ileri, _CFG, gecilmis=[(2.0, 10.0, 2.0)]
    )
    assert gate is not None
    assert _approx(gate.midpoint, (2.0, 30.0))


def test_K1_follower_gecince_kapiyi_ARKAYA_yazar_ve_yeniden_kilitlenmez() -> None:
    """Uçtan uca: kilitlen → geç → aynı dubalar hâlâ görünüyor → tekrar KİLİTLENME."""
    buoys = [(-2.0, 10.0), (2.0, 10.0)]
    gf = GateFollower(_CFG)
    _kilitlen(gf, (0.0, 0.0), (0.0, 20.0), buoys)
    gf.update((0.0, 11.0), (0.0, 20.0), buoys)          # geçti
    assert gf.passed_gate_count == 1
    assert len(gf.gecilen_kapilar) == 1

    # Araç geri savruldu ve kapı hâlâ görünüyor: ESKİDEN buraya kilitlenip
    # geri sürüyordu. Artık ham GN'ye düşer.
    for _ in range(ONAY_TICK + 1):
        res = gf.update((0.0, 5.0), (0.0, 20.0), buoys)
    assert gf.committed_gate is None
    assert res.used_fallback is True
    assert _approx(res.target, (0.0, 20.0))


def test_K1_kapinin_YANINDAN_gecmek_arkada_sayilir_UZAGINDAN_gecmek_SAYILMAZ() -> None:
    """Kapı düzlemi SONSUZDUR — "arkada" kararı yanal mesafeye bağlı olmalı.

    Bu koşul olmadan, kapıya hâlâ 3 m yandan yaklaşan araç düzlemi teğet
    geçer geçmez kapıyı "arkada" yazıp KALICI kaybediyordu (ölçümde kapı-2
    hiç geçilemedi). Ölçü kapının kendi genişliği: yarı genişlik "aradan
    geçtim" (puan), bir tam genişlik "o kapının yanındaydım" (geometri).
    """
    buoys = [(-2.0, 10.0), (2.0, 10.0)]            # genişlik 4.0 m

    # (a) 3 m yandan geçiş: aradan geçilmedi (puan yok) ama kapının YANINDA
    #     → arkada sayılır, yeniden hedef olmaz.
    gf = GateFollower(_CFG)
    _kilitlen(gf, (0.0, 0.0), (0.0, 20.0), buoys)
    gf.update((3.0, 11.0), (0.0, 20.0), [])
    assert gf.passed_gate_count == 0
    assert len(gf.gecilen_kapilar) == 1

    # (b) 9 m uzaktan geçiş: kapıyla ilgimiz yok → arkada SAYILMAZ, açı
    #     düzelirse yeniden denenebilir (kaçırılan kapı kalıcı kaybolmasın).
    gf2 = GateFollower(_CFG)
    _kilitlen(gf2, (0.0, 0.0), (0.0, 20.0), buoys)
    gf2.update((9.0, 11.0), (0.0, 20.0), [])
    assert gf2.passed_gate_count == 0
    assert gf2.gecilen_kapilar == []


def test_K1_reset_ARKADAKILERI_KORUR_yeniden_baslama_TEMIZLER() -> None:
    """Parkur geçişinde kapılar hâlâ arkadadır; yeniden başlamada değildir."""
    buoys = [(-2.0, 10.0), (2.0, 10.0)]
    gf = GateFollower(_CFG)
    _kilitlen(gf, (0.0, 0.0), (0.0, 20.0), buoys)
    gf.update((0.0, 11.0), (0.0, 20.0), buoys)
    assert len(gf.gecilen_kapilar) == 1

    gf.reset()                                          # parkur geçişi
    assert len(gf.gecilen_kapilar) == 1, "parkur geçişi arkadakileri unutamaz"

    gf.reset_passed_gates()                             # md 5.5.3.1 yeniden başlama
    assert gf.gecilen_kapilar == []
    # Temizlendiğine göre aynı kapıya yeniden kilitlenebilmeli (2. tur).
    _kilitlen(gf, (0.0, 0.0), (0.0, 20.0), buoys)
    assert gf.committed_gate is not None


def test_K1_ardisik_parkurda_hedef_GERIYE_gitmez() -> None:
    """Salınımın kendisini donduran regresyon: hedef bir daha geriye düşmesin.

    Üç kapılı kurs; araç her kapıdan sonra bir sonrakine EĞİK yaklaşıyor
    (ölçümde salınımı tetikleyen durum). Hedefin x'i asla azalmamalı.
    """
    k1 = [(20.0, 2.0), (20.0, -2.0)]
    k2 = [(40.0, 5.25), (40.0, 2.75)]
    k3 = [(60.0, -1.0), (60.0, -5.0)]
    hepsi = k1 + k2 + k3
    gf = GateFollower(_CFG)

    izlence = [                    # (araç, ham GN) — kursun kabaca gidişi
        ((0.0, 0.0), (20.0, 1.3)), ((10.0, 0.5), (20.0, 1.3)),
        ((19.0, 0.4), (20.0, 1.3)), ((21.0, -0.6), (40.0, 4.2)),
        ((30.0, -3.2), (40.0, 4.2)), ((38.1, -1.8), (40.0, 4.2)),
        ((40.81, 0.76), (40.0, 4.2)),          # ← salınımın tetiklendiği an
        # Açı düzeliyor: kapı-2 yeniden geçerli oluyor, kilitleniyor, geçiliyor
        ((36.0, 2.0), (40.0, 4.2)), ((38.5, 3.5), (40.0, 4.2)),
        ((39.5, 3.9), (40.0, 4.2)), ((40.5, 4.0), (60.0, -3.1)),
        ((50.0, -1.0), (60.0, -3.1)), ((59.0, -2.9), (60.0, -3.1)),
    ]
    en_ileri_hedef_x = -math.inf
    for arac, gn in izlence:
        res = gf.update(arac, gn, hepsi)
        # Hedef ya kapı nişanıdır ya ham GN — ikisi de geriye gitmemeli.
        assert res.target[0] >= en_ileri_hedef_x - 1.0, (
            f"hedef GERİYE düştü: {res.target} (araç {arac})"
        )
        # Kapı-1 ARKADA kaldıktan SONRA ona yeniden kilitlenilmemeli
        # (salınımın kendisi). Öncesinde kilitlenmesi zaten doğru davranış.
        kapi1_arkada = any(
            _approx((g[0], g[1]), (20.0, 0.0), tol=0.5)
            for g in gf.gecilen_kapilar
        )
        if kapi1_arkada and res.gate is not None:
            assert not _approx(res.gate.midpoint, (20.0, 0.0), tol=0.5), (
                f"geçilmiş kapı-1'e yeniden kilitlendi (araç {arac})"
            )
        en_ileri_hedef_x = max(en_ileri_hedef_x, res.target[0])
    # ①'in kendi kendini düzelttiğinin kanıtı: eğik yaklaşmada elenen kapı-2,
    # açı düzelince kilitlenip GEÇİLİYOR (yani eleme kalıcı puan kaybı değil).
    assert gf.passed_gate_count >= 2, "kurs boyunca en az 2 kapı geçilmeliydi"


# ═══════════════════════════════════════════════════════════════════════════
# F-K.1 (13.08.2026) — KAPI VARIŞ NOKTASI DEĞİL, GEÇİLECEK EŞİKTİR
#
# 🔴 SANAL GÖLDE ÖLÇÜLDÜ (kapalı döngü, gerçek düğümler, MP'den görev):
# tekne 25 m sürüp 1. kapıya vardı ve **(0.02, 24.95)'te kilitlendi**;
# `current_target` (−0.02, 25.04)'te takılı, MPPI thrust (−0.13, +0.05) N =
# fiilen sıfır. Görev yöneticisi bir sonraki noktaya GEÇMİŞ olmasına rağmen
# araç durdu.
#
# 🔑 ZİNCİR: nişan kapı düzleminin ÜSTÜNDE → MPPI referansı orada BİTER →
# `mppi._terminal_goal` referans sonuna KIRPAR (kod okundu: `ref[min(n-1,
# anchor+adim)]`) → terminal gradyanı 2·w·d sıfıra iner → araç kapı ortasında
# frenler → düzlem geçilmez → `signed_distance > 0` olmaz → kilit çözülmez →
# hedef bir daha ilerlemez. Kendi kendini kilitleyen döngü.
#
# 📏 UZATMA = ÖLÇÜLMÜŞ GÖVDE BOYU (1,04 m), ayarlanabilir eşik DEĞİL. Gerekçe
# yarışma tanımının kendisi: geçiş süresi *pruva* dubaları geçince başlar,
# ***kıç* geçince* biter — yani geçmiş sayılmak için tüm tekne düzlemin
# ötesine çıkmalı. (Aynı ilke pure-pursuit ailesinde de vardır: nişan hep
# aracın ÖNÜNDEDİR, araçta biten bir terminal nokta değil.)
# ═══════════════════════════════════════════════════════════════════════════


def test_FK1_surus_hedefi_kapi_DUZLEMININ_OTESINDE() -> None:
    g = Gate(left=(-2.0, 25.0), right=(2.0, 25.0), midpoint=(0.0, 25.0),
             normal=(0.0, 1.0))
    hedef = g.gecis_hedefi(1.04)
    assert hedef == pytest.approx((0.0, 26.04), abs=1e-6)
    # Düzlemin ötesinde: aracı oraya süren MPPI kapıyı GEÇMEK ZORUNDA kalır.
    assert g.signed_distance(hedef) > 0.0


def test_FK1_uzatma_NISANI_bozmaz_kimlik_korunur() -> None:
    """`midpoint`/`drive_target` KİMLİKTİR (eşleşme + geçiş testi buna bakar);
    uzatma yalnız KONTROL hedefidir. İkisi karışırsa kapı kendi kendini
    kaybeder (drift eşiği orta noktaya göre ölçülüyor)."""
    g = Gate(left=(-2.0, 25.0), right=(2.0, 25.0), midpoint=(0.0, 25.0),
             normal=(0.0, 1.0))
    _ = g.gecis_hedefi(1.04)
    assert g.midpoint == (0.0, 25.0)
    assert g.drive_target == (0.0, 25.0)


def test_FK1_normal_YOKSA_uzatma_YAPILMAZ() -> None:
    """Eski kurs-ekseni kolu: yön bilinmiyorsa körlemesine uzatmak, hedefi
    yanlış tarafa (geriye) taşıyabilirdi."""
    g = Gate(left=(-2.0, 25.0), right=(2.0, 25.0), midpoint=(0.0, 25.0))
    assert g.gecis_hedefi(1.04) == g.drive_target


def test_FK1_KAPI_YOKKEN_surus_hedefi_ham_GN_ile_AYNI() -> None:
    """Geriye tam uyumluluk: kapısız davranış birebir korunmalı."""
    takip = GateFollower()
    sonuc = takip.update((0.0, 0.0), (20.0, 3.0), [], [])
    assert sonuc.used_fallback is True
    assert sonuc.surus_hedefi == sonuc.target == (20.0, 3.0)


def test_FK1_kilitli_kapida_surus_hedefi_OTEDE_target_NISANDA() -> None:
    """Uçtan uca: `update()` iki noktayı da doğru döndürmeli."""
    takip = GateFollower()
    dubalar = [(10.0, +2.0), (10.0, -2.0)]
    sonuc = None
    for i in range(ONAY_TICK + 1):
        sonuc = takip.update((0.0, 0.0), (20.0, 0.0), dubalar, [], gozlem_no=i)
    assert sonuc is not None and sonuc.gate is not None
    assert sonuc.target == pytest.approx((10.0, 0.0), abs=1e-6)
    # Sürüş hedefi kapının ötesinde ve düzlemi geçmiş konumda.
    assert sonuc.surus_hedefi[0] > 10.0
    assert sonuc.gate.signed_distance(sonuc.surus_hedefi) > 0.0
