"""
Girdap İDA — MPPI saha tuning yüzeyinin config-drift kapıları (ROS'SUZ).

Aynı varsayılan üç yerde yaşıyor:
    prototype/planning/mppi.py      MPPIConfig            (TEK GERÇEK KAYNAK)
    launch/hardware.launch.py       _MPPI_DEFAULTS        (launch/CLI kopyası)
    config/params.yaml              planning_node bloğu   (saha tuning yüzeyi)

Launch dosyası `prototype`'ı import ETMEZ (--show-args ROS'suz/numpy'siz
makinede de çalışmalı) → kopya kaçınılmaz. Sessiz drift ise kabul edilemez:
kod varsayılanı değişip launch'ta kalırsa `ros2 launch` ile `ros2 run` FARKLI
davranır ve bu sahada fark edilmez. Bu testler kopyaları bağlar.

launch/launch_ros GEREKTİRMEZ (dosya `ast` ile okunur) — CI'ın ROS'suz
çekirdek job'ında da koşar (F16.2 kapılama dersi: gerçekten koşabilen test).
"""

from __future__ import annotations

import ast
from pathlib import Path

import pytest
import yaml

from prototype.planning.mppi import MPPIConfig

_PKG_DIR = (
    Path(__file__).resolve().parents[2] / "ros2_ws" / "src" / "girdap_decision"
)
_LAUNCH_FILE = _PKG_DIR / "launch" / "hardware.launch.py"
_PARAMS_FILE = _PKG_DIR / "config" / "params.yaml"
_HARDWARE_FILE = _PKG_DIR / "config" / "hardware.yaml"

# launch anahtarı → MPPIConfig alanı (λ hariç: nöbetçi değerli, ayrı test)
_ESLEME = {
    "mppi_sigma_u": "sigma_u",
    "mppi_obstacle_margin": "obstacle_margin",
    "mppi_terminal_mode": "terminal_mode",
    "mppi_terminal_lookahead_m": "terminal_lookahead_m",
    "mppi_ref_window_size": "ref_window_size",
    "mppi_ref_window_enabled": "ref_window_enabled",
}


# Launch sabitleri `(değer, tip)` demetleri tutuyor; `float`/`int`/... ast'te
# literal DEĞİL (Name) → literal_eval tek başına yetmez, tip adlarını çözeriz.
_TIPLER = {"float": float, "int": int, "str": str, "bool": bool}


def _coz(dugum: ast.AST):
    if isinstance(dugum, ast.Name) and dugum.id in _TIPLER:
        return _TIPLER[dugum.id]
    if isinstance(dugum, ast.Tuple):
        return tuple(_coz(e) for e in dugum.elts)
    if isinstance(dugum, ast.Dict):
        return {_coz(k): _coz(v) for k, v in zip(dugum.keys, dugum.values)}
    return ast.literal_eval(dugum)


def _sabit(ad: str) -> dict:
    """Launch dosyasındaki modül düzeyi `<ad> = {...}` sabitini ast ile oku."""
    agac = ast.parse(_LAUNCH_FILE.read_text(encoding="utf-8"))
    for dugum in agac.body:
        hedefler = (
            dugum.targets if isinstance(dugum, ast.Assign)
            else [dugum.target] if isinstance(dugum, ast.AnnAssign)
            else []
        )
        for hedef in hedefler:
            if isinstance(hedef, ast.Name) and hedef.id == ad:
                return _coz(dugum.value)
    raise AssertionError(f"{ad} launch dosyasında bulunamadı")


def _yaml(path: Path) -> dict:
    return yaml.safe_load(path.read_text(encoding="utf-8")) or {}


def test_launch_varsayilanlari_mppiconfig_ile_ayni() -> None:
    """`_MPPI_DEFAULTS` ↔ `MPPIConfig`: değer VE tip birebir."""
    kod = MPPIConfig()
    launch = _sabit("_MPPI_DEFAULTS")
    for launch_key, cfg_key in _ESLEME.items():
        deger, tip = launch[launch_key]
        beklenen = getattr(kod, cfg_key)
        assert deger == beklenen, (
            f"{launch_key}: launch={deger!r} ≠ MPPIConfig.{cfg_key}={beklenen!r}"
            " — kod varsayılanı değiştiyse launch'ı da güncelle"
        )
        assert isinstance(beklenen, tip), f"{launch_key} tipi {tip} olmalı"


def test_lambda_nobetcisi_profili_kazandirir() -> None:
    """λ launch/yaml varsayılanı 0.0 = "parkur profili kazansın" nöbetçisi.
    Buraya gerçek bir λ yazılırsa üç parkurun profil değeri sessizce ezilir."""
    from prototype.planning.pipeline import _PARKUR_PROFILES

    assert _sabit("_MPPI_DEFAULTS")["mppi_lambda"][0] == 0.0
    assert _yaml(_PARAMS_FILE)["planning_node"]["ros__parameters"][
        "mppi_lambda"
    ] == 0.0
    assert _yaml(_HARDWARE_FILE)["planning"]["mppi_lambda"] == 0.0
    # Nöbetçi anlamlı olsun diye profillerin λ'sı pozitif olmalı
    assert all(p.lambda_ > 0.0 for p in _PARKUR_PROFILES.values())


@pytest.mark.parametrize("dosya", ["params", "hardware"])
def test_yaml_anahtarlari_launch_ile_ayni(dosya: str) -> None:
    """yaml'daki mppi_* anahtar KÜMESİ launch/node ile birebir olmalı.

    ROS bilinmeyen yaml anahtarını SESSİZCE atar → `mppi_lambdaa: 50` yazımı
    hiçbir uyarı üretmeden yok sayılırdı (sahada "değiştirdim ama değişmedi").
    """
    if dosya == "params":
        blok = _yaml(_PARAMS_FILE)["planning_node"]["ros__parameters"]
    else:
        blok = _yaml(_HARDWARE_FILE)["planning"]
    anahtarlar = {
        k for k in blok
        if k.startswith("mppi_") and k not in ("mppi_K", "mppi_T")
    }
    assert anahtarlar == set(_sabit("_MPPI_DEFAULTS"))


@pytest.mark.parametrize("dosya", ["params", "hardware"])
def test_yaml_degerleri_kod_varsayilaniyla_ayni(dosya: str) -> None:
    """Sevk edilen yaml'lar kod varsayılanını YANSITMALI — yaml'ı okuyan
    operatör aracın gerçekte hangi ayarla uçtuğunu görebilsin."""
    kod = MPPIConfig()
    if dosya == "params":
        blok = _yaml(_PARAMS_FILE)["planning_node"]["ros__parameters"]
    else:
        blok = _yaml(_HARDWARE_FILE)["planning"]
    for yaml_key, cfg_key in _ESLEME.items():
        assert blok[yaml_key] == getattr(kod, cfg_key), (
            f"{dosya}.yaml {yaml_key}={blok[yaml_key]!r} ≠ "
            f"MPPIConfig.{cfg_key}={getattr(kod, cfg_key)!r}"
        )


def test_launch_argumanlari_aciklamali() -> None:
    """`--show-args` çıktısında her mppi_* argümanı açıklamalı görünsün —
    operatör sınırları (geçit tavanı, λ dejenerasyonu) orada okuyabilmeli."""
    tanimlar = _sabit("_MPPI_DEFAULTS")
    aciklamalar = _sabit("_MPPI_ARG_DESC")
    assert set(tanimlar) == set(aciklamalar)
    assert all(len(v) > 20 for v in aciklamalar.values())


def test_terminal_mode_yaml_degeri_gecerli() -> None:
    """yaml'daki terminal_mode MPPIConfig doğrulamasından geçmeli (geçersiz
    değer node'da WARN + varsayılana düşüş üretir, sessiz kalmaz)."""
    for blok in (
        _yaml(_PARAMS_FILE)["planning_node"]["ros__parameters"],
        _yaml(_HARDWARE_FILE)["planning"],
    ):
        MPPIConfig(terminal_mode=blok["mppi_terminal_mode"])   # ValueError = geçersiz


# --------------------------------------------------------------------------- #
# Kapı takibi (gate following, 2026-08-03) — aynı drift kapısı
# --------------------------------------------------------------------------- #

# launch anahtarı → GateFollowerConfig alanı. Üç anahtar (enabled / class_id /
# use_classified) node'a özgüdür, çekirdek config'te karşılığı YOKTUR.
_GATE_ESLEME = {
    "hull_width_m": "hull_width_m",
    "hull_length_m": "hull_length_m",
}
_GATE_NODE_ONLY = {
    "gate_following_enabled",
    "edge_buoy_class_id",
    "use_classified_obstacles",
}


def test_gate_launch_varsayilanlari_kodla_ayni() -> None:
    """`_GATE_DEFAULTS` ↔ `GateFollowerConfig`: değer VE tip birebir."""
    from prototype.mission.gate_follower import GateFollowerConfig

    kod = GateFollowerConfig()
    launch = _sabit("_GATE_DEFAULTS")
    for launch_key, cfg_key in _GATE_ESLEME.items():
        deger, tip = launch[launch_key]
        beklenen = getattr(kod, cfg_key)
        assert deger == beklenen, (
            f"{launch_key}: launch={deger!r} ≠ "
            f"GateFollowerConfig.{cfg_key}={beklenen!r} — çekirdek varsayılanı "
            "değiştiyse launch'ı da güncelle"
        )
        assert isinstance(beklenen, tip), f"{launch_key} tipi {tip} olmalı"


def test_gate_takibi_varsayilan_ACIK() -> None:
    """Kapı takibi varsayılan AÇIK olmalı — md 5.5.2.2'ye göre Parkur-1/2
    puanı kapıdan geçmekten gelir; sessizce kapalı kalması puan kaybettirir.
    (Kapı görünmüyorken çekirdek zaten ham GN'ye düşer, yani açık olmak
    kapısız senaryoda davranışı DEĞİŞTİRMEZ.)"""
    assert _sabit("_GATE_DEFAULTS")["gate_following_enabled"][0] is True
    assert _yaml(_PARAMS_FILE)["planning_node"]["ros__parameters"][
        "gate_following_enabled"
    ] is True
    assert _yaml(_HARDWARE_FILE)["planning"]["gate_following_enabled"] is True


def test_huni_tavani_UC_yerde_de_ayni() -> None:
    """B2 huni tavanı (`gate_post_margin_m`) launch ↔ params ↔ hardware birebir.

    Çekirdek `GateFollowerConfig`'te KARŞILIĞI YOK ve olmamalı: bu bir kapı
    SEÇİM eşiği değil, kaçınma şiddeti (`obstacle_margin` ailesinden) —
    `test_kapi_seciminde_ayarlanabilir_esik_KALMADI` o ayrımı koruyor.

    ⚠ Ayrışırsa arıza sessiz: yaml'ı okuyan operatör 1.8 görür, node 1.0 ile
    koşar ve gövde payı temasa döner (ölçüm: 1.0 → −0,019 m).
    """
    launch_deger, launch_tip = _sabit("_GATE_DEFAULTS")["gate_post_margin_m"]
    params = _yaml(_PARAMS_FILE)["planning_node"]["ros__parameters"]
    hardware = _yaml(_HARDWARE_FILE)["planning"]

    assert launch_tip is float
    assert params["gate_post_margin_m"] == launch_deger
    assert hardware["gate_post_margin_m"] == launch_deger
    # İKİ TARAFLI ölçülmüş bant — ikisi de gerçek parkurda ölçüldü:
    #   alt: 1.0'da gövde payı −0,006 m = TEMAS (Ç1, P1'de 16 puan)
    #   üst: 1.8'de SON güzergah noktasına varılamıyor (3/4) — o nokta kapı
    #        direğine 2,0 m uzakta ve halka 2,0 m'lik varış yarıçapını yiyor
    # Kırmızıya dönerse çözüm testi gevşetmek değil, ikisini de yeniden ÖLÇMEK.
    assert 1.0 < launch_deger < 1.8, (
        f"gate_post_margin_m={launch_deger} ölçülen bandın ({1.0}, {1.8}) "
        "dışında — altı temas, üstü son noktaya varamama demek"
    )


def test_huni_tavani_KURESEL_paydan_bagimsiz() -> None:
    """Huni tavanı `mppi_obstacle_margin`'e EŞİTLENMEMELİ.

    09.08'de tam bu denendi (huni tavanı = küresel pay) ve küresel payı
    büyütmek zorunda kaldık; sonuç: model gelmeyen kol kırıldı (ham güzergah
    noktasına sürüş 3,3/4 → 1/4 nokta). İkisinin ayrı kalması, kapı takipli
    kolu model gelmeyen kolu bozmadan ayarlayabilmenin tek yolu.
    """
    gate = _sabit("_GATE_DEFAULTS")["gate_post_margin_m"][0]
    mppi = _sabit("_MPPI_DEFAULTS")["mppi_obstacle_margin"][0]
    assert gate != mppi, (
        "huni tavanı küresel engel payıyla aynı değere getirilmiş — biri "
        "diğerini takip etmeye başlarsa 09.08'deki gerileme geri gelir"
    )
    assert mppi == 1.0, (
        f"mppi_obstacle_margin={mppi}; BÜYÜTÜLMEMELİ (1.2'de model gelmeyen "
        "kol 2/4 noktaya düşüyor) — gövde payı için gate_post_margin_m'i kullan"
    )


@pytest.mark.parametrize("dosya", ["params", "hardware"])
def test_gate_yaml_anahtarlari_launch_ile_ayni(dosya: str) -> None:
    """yaml'daki kapı anahtar KÜMESİ launch/node ile birebir (yazım hatası
    kapanı — ROS bilinmeyen anahtarı sessizce atar)."""
    if dosya == "params":
        blok = _yaml(_PARAMS_FILE)["planning_node"]["ros__parameters"]
    else:
        blok = _yaml(_HARDWARE_FILE)["planning"]
    anahtarlar = {
        k for k in blok if k.startswith("gate_") or k.startswith("hull_")
        or k in _GATE_NODE_ONLY
    }
    assert anahtarlar == set(_sabit("_GATE_DEFAULTS"))


@pytest.mark.parametrize("dosya", ["params", "hardware"])
def test_gate_yaml_degerleri_kod_varsayilaniyla_ayni(dosya: str) -> None:
    """Sevk edilen yaml'lar çekirdek varsayılanını YANSITMALI."""
    from prototype.mission.gate_follower import GateFollowerConfig

    kod = GateFollowerConfig()
    if dosya == "params":
        blok = _yaml(_PARAMS_FILE)["planning_node"]["ros__parameters"]
    else:
        blok = _yaml(_HARDWARE_FILE)["planning"]
    for yaml_key, cfg_key in _GATE_ESLEME.items():
        assert blok[yaml_key] == getattr(kod, cfg_key), (
            f"{dosya}.yaml {yaml_key}={blok[yaml_key]!r} ≠ "
            f"GateFollowerConfig.{cfg_key}={getattr(kod, cfg_key)!r}"
        )


def test_kapi_seciminde_ayarlanabilir_esik_KALMADI() -> None:
    """🔑 Tasarım kuralı (2026-08-03): kapı seçiminde tahmine dayalı sayı yok.

    Kapı geometrisi önceden bilinemez (şartname: mesafeler yarışma alanına göre
    değişir + dubalar deniz şartlarıyla yer değiştirir), dolayısıyla "sahada
    ölçüp gir" diye bir eşik OLAMAZ. Geriye yalnız ÖLÇÜLMÜŞ tekne boyutları
    kalmalı; genişlik bandı / menzil / derinlik toleransı / bırakma mesafesi /
    eşleşme yarıçapı hepsi geometriden türetilir.

    Bu test o kuralı DONDURUR: biri geri eklenirse CI kırmızı olur.
    """
    from prototype.mission.gate_follower import GateFollowerConfig

    yasak = {
        "gate_width_min", "gate_width_max", "max_lookahead",
        "pair_depth_tol", "release_distance", "match_radius",
    }
    alanlar = set(GateFollowerConfig.__dataclass_fields__)
    sizan = yasak & alanlar
    assert not sizan, (
        f"GateFollowerConfig'e tahmine dayalı eşik geri gelmiş: {sorted(sizan)}. "
        "Kapı geometrisi önceden bilinemez — türetilmiş geometri kullan."
    )
    assert alanlar == {"hull_width_m", "hull_length_m"}, (
        f"Beklenen alanlar yalnız ölçülmüş tekne boyutları; bulunan: {sorted(alanlar)}"
    )
    # Launch/yaml tarafında da hiçbir eşik kalmamalı.
    launch = set(_sabit("_GATE_DEFAULTS"))
    assert not (yasak & {k.replace("gate_", "") for k in launch})


# --------------------------------------------------------------------------- #
# 2026-08-06 — MPPI sabitleri AKTÜATÖRE bağlandı (GIRDAP_DURUM §0.8).
# Bu iki test bir SAYIYI değil bir İLİŞKİYİ donduruyor: dinamik yeniden
# tanılanırsa (max_thrust/Xu değişirse) MPPI ayarları da onunla birlikte
# taşınmak zorunda. Eski σ=5.0 / λ=10 tam olarak bu bağın kopmasıydı —
# dinamik 30 N'dan 1.455 N'a inerken ayarlar eski teknede kaldı ve
# hiçbir test bunu yakalamadı.
# --------------------------------------------------------------------------- #


def _dinamik():
    from prototype.dynamics.catamaran import CatamaranParams
    return CatamaranParams.from_yaml()


# ÖLÇÜLEN ÇALIŞMA BANDI (2026-08-06, GIRDAP_DURUM §0.8g/0.8i):
#   ÇALIŞAN   σ/T ∈ [0.15, 0.50]  (temiz + bozucu %30'da %100)
#   ÇÖKEN     σ/T ≤ 0.10 (keşif yetersiz) · σ/T ≥ 0.69 (doygunluk)
#   SEVK      σ/T = 0.25 → çöküş bandının geometrik ortası √(0.10·0.69)=0.263
# Test sınırları çalışan bandın İÇİ: kenarına dayanmış bir değeri de reddeder,
# çünkü sahada model hatası bizi kenara doğru itebilir (yaw doğrulanmadı,
# itki 2-4× yanlış olabilir).
_ORAN_ALT, _ORAN_UST = 0.20, 0.45


def test_mppi_sigma_u_olculen_calisma_araliginda() -> None:
    """σ_u, ölçülen aralığın İÇİNDE olmalı — tek nokta değil, bant.

    ALT sınır (0.25·T): altında MPPI kaçış manevrasını yeterince
    örnekleyemiyor → dar koridor/slalom sahnelerinde takılıyor.
    ÜST sınır (0.41·T doğrulandı; 0.69'da kesin çöküyor): örnekler ±T_max'a
    kırpılıyor, ETKİN gürültü σ'dan kopuyor (σ/T=0.33'te %82 → 3.44'te %33)
    ve tek yönlü kırpma TEPE HIZI yiyor (ölçüm: %89 → %63'e düşüyor).
    """
    p = _dinamik()
    oran = MPPIConfig().sigma_u / p.max_thrust
    assert _ORAN_ALT <= oran <= _ORAN_UST, (
        f"σ_u/T = {oran:.3f} ölçülen [{_ORAN_ALT}, {_ORAN_UST}] aralığının "
        f"dışında (σ_u={MPPIConfig().sigma_u} N, T={p.max_thrust} N). "
        "İtki yeniden tanılandıysa σ_u'yu ORANI koruyacak şekilde taşı."
    )


def test_lambda_sigmaya_bagli_argmin_rejiminde_kalir() -> None:
    """🔑 λ ile σ BAĞLI — λ tek başına seçilemez.

    λ, rollout maliyetlerinin yayılımının biriminde bir sayı. Yayılım σ ile
    büyüyor; MPPI'nin içi ölçüldü: **S_p10 − S_min ≈ 131·(σ/T)**. Softmax'ın
    bu teknede işe yarayan rejimi 'argmin' (ESS≈1) — 1.455 N'lik aktüatörde
    ağırlıklı ortalama kontrolü nominale çekiyor ve tekne sürüklenmeyi
    yenemiyor (ölçüm §0.7c). Argmin rejiminde kalmak için λ ≲ yayılım/10:
        λ_maks ≈ 13 · (σ/T)
    Bu yasa 405 koşumluk BAĞIMSIZ ızgarada %80, taze taramada birebir
    tutuyor (σ/T=0.15 → λ=3 düşüyor · 0.25 → λ=3 geçiyor).
    Emniyet payı: yürürlükteki λ, tavanın en fazla YARISI olsun.
    """
    from prototype.planning.pipeline import _PARKUR_PROFILES

    p = _dinamik()
    oran = MPPIConfig().sigma_u / p.max_thrust
    lambda_tavan = 13.1 * oran
    for parkur in ("PARKUR1", "PARKUR2"):
        lam = _PARKUR_PROFILES[parkur].lambda_
        assert lam <= lambda_tavan / 2.0, (
            f"{parkur} λ={lam}, σ/T={oran:.3f} için tavan {lambda_tavan:.1f} "
            f"(emniyetli {lambda_tavan/2:.1f}) — softmax ortalamaya kayıyor"
        )


def test_parkur_lambdalari_olculen_degerlerde_donduruldu() -> None:
    """λ profilleri 06.08 kapalı-döngü ölçümünün sonucunda.

    P1/P2 = 1.0 · P3 = 10.0. Değiştiren, ÖLÇÜMLE değiştirsin:
      · λ=10 (P1/P2) → bozucu altında 6/9 sahnede hedefe varılamadı
      · λ=50 (P3)    → temas hızı 0.60 → 0.14 m/s (IMU şok eşiği riski)
    """
    from prototype.planning.pipeline import _PARKUR_PROFILES

    assert _PARKUR_PROFILES["PARKUR1"].lambda_ == 1.0
    assert _PARKUR_PROFILES["PARKUR2"].lambda_ == 1.0
    assert _PARKUR_PROFILES["PARKUR3"].lambda_ == 10.0
    # P3'ün λ'sı P1/P2'den BÜYÜK olmalı: kamikaze çekicisi maliyet ölçeğini
    # büyütür, λ maliyet ölçeğiyle birlikte seçilir.
    assert (
        _PARKUR_PROFILES["PARKUR3"].lambda_
        > _PARKUR_PROFILES["PARKUR2"].lambda_
    )


# --------------------------------------------------------------------------- #
# ÖLÇÜM TABLOLARI (2026-08-06, kapalı döngü, GIRDAP_DURUM §0.8g).
# Aşağıdaki testler σ seçimini FARKLI AÇILARDAN bağlar: yalnız "hedefe varıyor
# mu" değil, teknenin hızını ne kadar yiyor · gürültünün ne kadarı gerçekleşiyor
# · yarışma overlay'i bunları sessizce ezebiliyor mu.
# --------------------------------------------------------------------------- #

#: σ/T → tepe hızın modelin terminal hızına oranı (80-100 m düz hat, p99).
_HIZ_OLCUMU = {0.10: 0.944, 0.15: 0.944, 0.20: 0.931, 0.25: 0.919,
               0.33: 0.891, 0.41: 0.871, 0.50: 0.845, 0.69: 0.794, 1.03: 0.749}
#: σ/T → ETKİN gürültünün komut edilene oranı (kırpma sonrası, aynı sahne).
_ETKIN_OLCUMU = {0.03: 1.00, 0.10: 0.94, 0.25: 0.85, 0.33: 0.82, 0.50: 0.78,
                 0.69: 0.74, 1.03: 0.66, 1.72: 0.52, 3.44: 0.33}


def _ara_deger(tablo: dict, x: float) -> float:
    """Ölçüm noktaları arasında doğrusal ara değer (dışında en yakın uç)."""
    xs = sorted(tablo)
    if x <= xs[0]:
        return tablo[xs[0]]
    for a, b in zip(xs[:-1], xs[1:]):
        if a <= x <= b:
            return tablo[a] + (x - a) / (b - a) * (tablo[b] - tablo[a])
    return tablo[xs[-1]]


def test_sigma_teknenin_tepe_hizini_en_fazla_yuzde_15_yer() -> None:
    """σ, tepe hızın en az %85'ini bırakmalı — 20 dk sınırı doğrudan mesafe.

    Mekanizma tek yönlü kırpma: seyirde nominal itki zaten doyuma yakın
    (ölçüldü: ~%68) → POZİTİF gürültü kırpılır, negatif kırpılmaz, ortalama
    itki düşer. Kapalı form: kayıp ≈ 0.4·(σ/T); ölçümle uyumlu (%6/%6.7 ·
    %13/%11 · %20/%16). Eski σ=5.0 N ayarı tepe hızın **%37'sini** yiyordu.
    """
    p = _dinamik()
    oran = MPPIConfig().sigma_u / p.max_thrust
    kalan = _ara_deger(_HIZ_OLCUMU, oran)
    assert kalan >= 0.85, (
        f"σ/T={oran:.3f} tepe hızın yalnız %{100*kalan:.0f}'ini bırakıyor "
        "(alt sınır %85) — gürültü aktüatörü doyuruyor (GIRDAP_DURUM §0.8g)"
    )


def test_komut_edilen_gurultunun_cogu_GERCEKLESIYOR() -> None:
    """Kırpma sonrası ETKİN gürültü, komut edilenin ≥%75'i olmalı.

    σ ≥ max_thrust olduğunda örneklerin çoğu ±T_max'a yapışır: MPPI 'kontrol
    keşfi' yapmayı bırakıp fiilen rastgele tam gaz üretir ve komut edilen
    σ'nın anlamı kalmaz (ölçüm: σ/T=3.44'te örneklerin %78'i kırpılıyor,
    etkin/komut oranı 0.33). Bu test o rejime girilmesini engeller.
    """
    p = _dinamik()
    oran = MPPIConfig().sigma_u / p.max_thrust
    etkin = _ara_deger(_ETKIN_OLCUMU, oran)
    assert etkin >= 0.75, (
        f"σ/T={oran:.3f} → komut edilen gürültünün yalnız %{100*etkin:.0f}'i "
        "gerçekleşiyor; gerisi doygunluğa gidiyor"
    )


def test_p3_lambdasi_temas_hizini_koruyan_bantta() -> None:
    """PARKUR3 λ ∈ [3, 30] — P3'ün bitişi TEMAS HIZINA bağlı.

    Ölçüm (5 tohum, hedef duba Ø0.64 m): λ≤1 → araç hedefe sürünüp **0.00
    m/s** ile yaslanıyor (0/5 içinden geçiş); λ=10 → 5/5 temas, 0.390 m/s,
    5/5 İÇİNDEN GEÇİŞ; λ=50 → temas var ama 0.034 m/s.
    Görev sonu IMU şokuyla algılandığı için (fsm_node shock_threshold_g)
    yavaş temas = **algılanmayan görev sonu** = 145 puan riski.
    ⚠ Bu test λ'yı bağlar; şok EŞİĞİNİN kendisi ayrı bir açık madde
    (0.39 m/s ölçülen temas, eşik 1.81 m/s'lik hayali tekneye göre konmuştu).
    """
    from prototype.planning.pipeline import _PARKUR_PROFILES

    lam = _PARKUR_PROFILES["PARKUR3"].lambda_
    assert 3.0 <= lam <= 30.0, (
        f"PARKUR3 λ={lam} — ölçülen temas-hızı bandının ({3.0}-{30.0}) dışında"
    )


def test_yarisma_overlayi_mppi_sabitlerini_sessizce_ezemez() -> None:
    """🔴 yarisma.yaml `planning:` yazarsa ölçülen bandın DIŞINA çıkamaz.

    Overlay yalnız FARKLARI içerir ve hardware.yaml'ın üstüne biner; bugün
    `planning:` bloğu YOK. Ama biri yarışma sabahı oraya `mppi_sigma_u: 5.0`
    yazarsa hiçbir şey uyarmaz — drift testi yalnız params/hardware/launch
    üçlüsünü bağlıyor. Bu test o dördüncü yolu kapatır.
    """
    yol = _PKG_DIR / "config" / "yarisma.yaml"
    if not yol.exists():
        pytest.skip("yarisma.yaml yok")
    import yaml
    with open(yol, "r", encoding="utf-8") as f:
        overlay = yaml.safe_load(f) or {}
    planning = overlay.get("planning") or {}
    p = _dinamik()

    if "mppi_sigma_u" in planning:
        oran = float(planning["mppi_sigma_u"]) / p.max_thrust
        assert _ORAN_ALT <= oran <= _ORAN_UST, (
            f"yarisma.yaml σ_u={planning['mppi_sigma_u']} → σ/T={oran:.3f}, "
            f"ölçülen [{_ORAN_ALT}, {_ORAN_UST}] bandının dışında"
        )
    if "mppi_lambda" in planning:
        lam = float(planning["mppi_lambda"])
        if lam > 0.0:                     # 0 = nöbetçi, profil kazanır
            oran = float(planning.get("mppi_sigma_u", MPPIConfig().sigma_u))
            oran = oran / p.max_thrust
            assert lam <= 13.1 * oran / 2.0, (
                f"yarisma.yaml λ={lam}, σ/T={oran:.3f} için emniyetli tavan "
                f"{13.1*oran/2:.1f} — softmax ortalamaya kayar"
            )


def test_hicbir_URETIM_modulu_sigma_u_yu_ELLE_yazmiyor() -> None:
    """🔴 5. YOL: demo/viz/script'lerde elle yazılmış σ_u.

    Bu test gerçek bir sızıntıdan doğdu: σ_u dört "resmî" yerde (MPPIConfig,
    launch, iki yaml) 0.364'e çekildikten SONRA bile `mppi.py` demo bloğunda
    ve `deniz_durumu_karsilastirma.py`'de **`sigma_u=5.0`** duruyordu — yani
    Deniz Durumu karşılaştırma grafikleri hâlâ 30 N/motor'luk HAYALİ tekneyi
    gösteriyordu ve ona bakan yanlış sonuç çıkarırdı. Drift testi bu yolu
    görmüyordu (yalnız config üçlüsünü bağlıyor).

    Kural: üretim/demo/görselleştirme kodu σ_u'yu ELLE YAZMAZ — varsayılanı
    kullanır ya da ROS parametresinden alır. (Testler hariç: onlar bilerek
    uç değerler enjekte eder.)
    """
    import re

    kok = Path(__file__).resolve().parents[1]          # prototype/
    hedefler = list((kok / "planning").rglob("*.py"))
    hedefler += list((kok / "viz").rglob("*.py"))
    hedefler += list((kok.parent / "scripts").rglob("*.py"))
    desen = re.compile(r"sigma_u\s*=\s*([0-9]+\.?[0-9]*)")

    ihlal = []
    for yol in hedefler:
        metin = yol.read_text(encoding="utf-8")
        for satir_no, satir in enumerate(metin.splitlines(), 1):
            if satir.lstrip().startswith("#"):
                continue
            m = desen.search(satir)
            if not m:
                continue
            # MPPIConfig alan TANIMI (varsayılanın kendisi) serbest.
            if "sigma_u: float" in satir:
                continue
            if float(m.group(1)) != MPPIConfig().sigma_u:
                ihlal.append(f"{yol.name}:{satir_no} → {satir.strip()}")
    assert not ihlal, (
        "σ_u elle yazılmış (ölçülen varsayılandan kopuk):\n  " + "\n  ".join(ihlal)
    )
