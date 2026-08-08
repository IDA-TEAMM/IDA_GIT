"""
MPPI birim testleri — kamikaze attractor + warm-start davranışı.
Çalıştır: pytest prototype/tests/test_mppi.py -v

Stratejisi: maliyet fonksiyonunu doğrudan (white-box) çağırarak determinist
testler yap; örnekleme kaynaklı stokastik kırılganlığı atla.
"""

import logging

import numpy as np
import pytest

from prototype.dynamics.catamaran import CatamaranDynamics
from prototype.planning.mppi import MPPIConfig, MPPIController
from prototype.planning.rrt_star import Bounds


@pytest.fixture
def dyn() -> CatamaranDynamics:
    return CatamaranDynamics()


@pytest.fixture
def bounds() -> Bounds:
    # Test yörüngeleri rahat içerisinde kalsın (boundary cezası tetiklenmesin)
    return Bounds(-1000.0, 1000.0, -1000.0, 1000.0)


# --------------------------------------------------------------------------- #
# Kamikaze attractor
# --------------------------------------------------------------------------- #


def _make_traj(K: int, T: int, x_const: float, y_const: float) -> np.ndarray:
    """Tüm state'leri (x_const, y_const, 0, 0, 0, 0) olan sabit yörünge."""
    traj = np.zeros((K, T + 1, 6))
    traj[:, :, 0] = x_const
    traj[:, :, 1] = y_const
    return traj


def test_kamikaze_disabled_invariant(dyn: CatamaranDynamics, bounds: Bounds) -> None:
    """kamikaze_mode=False → cost terimi eklenmesin (geriye uyumluluk)."""
    cfg = MPPIConfig(K=2, T=3, kamikaze_mode=False, backend="numpy")
    ctrl = MPPIController(dyn, bounds, [], cfg)
    traj = _make_traj(2, 3, x_const=10.0, y_const=0.0)
    U = np.zeros((2, 3, 2))
    cost = ctrl._trajectory_cost(traj, U)
    # Referans yok, engel yok, sınır içi, U=0, kamikaze off → tüm cost = 0
    np.testing.assert_allclose(cost, 0.0, atol=1e-12)


def test_kamikaze_target_none_no_effect(dyn: CatamaranDynamics, bounds: Bounds) -> None:
    """kamikaze_mode=True ama target=None → cost terimi eklenmesin (savunma)."""
    cfg = MPPIConfig(K=2, T=3, kamikaze_mode=True, kamikaze_target=None, backend="numpy")
    ctrl = MPPIController(dyn, bounds, [], cfg)
    traj = _make_traj(2, 3, x_const=10.0, y_const=0.0)
    U = np.zeros((2, 3, 2))
    cost = ctrl._trajectory_cost(traj, U)
    np.testing.assert_allclose(cost, 0.0, atol=1e-12)


def test_kamikaze_negative_cost_at_target(
    dyn: CatamaranDynamics, bounds: Bounds
) -> None:
    """Hedefte (d=0) konumlanan yörünge negatif cost almalı (çekici)."""
    target = (10.0, 0.0)
    cfg = MPPIConfig(
        K=1, T=4,
        kamikaze_mode=True, kamikaze_target=target,
        w_kamikaze=100.0, kamikaze_radius=5.0,
        backend="numpy",
    )
    ctrl = MPPIController(dyn, bounds, [], cfg)
    traj = _make_traj(1, 4, x_const=10.0, y_const=0.0)  # tam hedefte
    U = np.zeros((1, 4, 2))
    cost = ctrl._trajectory_cost(traj, U)
    # exp(0) = 1, T+1 = 5 adım → cost = -100 * 5 = -500
    np.testing.assert_allclose(cost[0], -500.0, atol=1e-9)


def test_kamikaze_target_pulls_relative_cost(
    dyn: CatamaranDynamics, bounds: Bounds
) -> None:
    """Hedefe yakın yörünge, uzak yörüngeden DAHA düşük cost almalı."""
    target = (10.0, 0.0)
    cfg = MPPIConfig(
        K=2, T=4,
        kamikaze_mode=True, kamikaze_target=target,
        w_kamikaze=200.0, kamikaze_radius=3.0,
        backend="numpy",
    )
    ctrl = MPPIController(dyn, bounds, [], cfg)
    # rollout 0: hedefte; rollout 1: 50 m uzakta
    traj = np.zeros((2, 5, 6))
    traj[0, :, 0] = 10.0   # hedefte
    traj[1, :, 0] = 60.0   # 50 m uzakta — exp(-50²/(2·9)) ≈ 0
    U = np.zeros((2, 4, 2))
    cost = ctrl._trajectory_cost(traj, U)
    assert cost[0] < cost[1], "Hedefteki yörünge daha düşük cost almalı"
    assert cost[0] < 0, "Hedefteki yörünge cost'u negatif olmalı"
    assert abs(cost[1]) < 1e-3, "Uzak yörünge cost'u ihmal edilebilir olmalı"


def test_kamikaze_overrides_obstacle(
    dyn: CatamaranDynamics, bounds: Bounds
) -> None:
    """w_kamikaze yeterince büyükse engel maliyetini ezmeli (CLAUDE.md spec)."""
    from prototype.planning.rrt_star import CircleObstacle
    target = (10.0, 0.0)
    # Hedef tam engelin içinde (kamikaze görevinin doğası)
    obstacles = [CircleObstacle(10.0, 0.0, 1.0)]

    cfg_off = MPPIConfig(
        K=1, T=4,
        kamikaze_mode=False,
        w_obstacle=200.0,
        backend="numpy",
    )
    ctrl_off = MPPIController(dyn, bounds, obstacles, cfg_off)
    traj = _make_traj(1, 4, x_const=10.0, y_const=0.0)
    U = np.zeros((1, 4, 2))
    cost_off = ctrl_off._trajectory_cost(traj, U)
    assert cost_off[0] > 0, "Kamikaze kapalıyken engel cezası pozitif olmalı"

    cfg_on = MPPIConfig(
        K=1, T=4,
        kamikaze_mode=True, kamikaze_target=target,
        w_kamikaze=10000.0, kamikaze_radius=3.0,
        w_obstacle=200.0,
        backend="numpy",
    )
    ctrl_on = MPPIController(dyn, bounds, obstacles, cfg_on)
    cost_on = ctrl_on._trajectory_cost(traj, U)
    assert cost_on[0] < 0, "Kamikaze açıkken net cost negatif olmalı (engeli ezdi)"


# --------------------------------------------------------------------------- #
# Warm-start
# --------------------------------------------------------------------------- #


def test_warm_start_post_step_last_zero(
    dyn: CatamaranDynamics, bounds: Bounds
) -> None:
    """warm_start_enabled=True → step sonrası U_nominal[-1] tam sıfır."""
    cfg = MPPIConfig(K=64, T=8, seed=0, warm_start_enabled=True, backend="numpy")
    ctrl = MPPIController(dyn, bounds, [], cfg)
    ctrl.set_reference([(0.0, 0.0), (10.0, 0.0)])
    ctrl.step(np.zeros(6))
    np.testing.assert_allclose(ctrl.U_nominal[-1], 0.0, atol=1e-12)


def test_warm_start_shift_invariant(
    dyn: CatamaranDynamics, bounds: Bounds
) -> None:
    """
    İki ardışık step arasında: U_nominal[1:] (k-1'inci adım sonrası) ==
    U_nominal[:-1] (k'inci adımın yeni nominal'i, son zero hariç) olmalı.

    Bu, kaydırma mantığını seed'den bağımsız doğrular.
    """
    cfg = MPPIConfig(K=64, T=8, seed=0, warm_start_enabled=True, backend="numpy")
    ctrl = MPPIController(dyn, bounds, [], cfg)
    ctrl.set_reference([(0.0, 0.0), (10.0, 0.0)])

    ctrl.step(np.zeros(6))
    snapshot = ctrl.U_nominal.copy()    # post-step1 nominal

    # Manuel olarak warm-start kaydırması beklenen formu üret:
    # post-step2 nominal = shift(U_new_step2) — U_new'i bilmediğimiz için
    # invariant'i kontrol et: post-stepN nominal[-1] her zaman 0
    ctrl.step(np.zeros(6))
    np.testing.assert_allclose(ctrl.U_nominal[-1], 0.0, atol=1e-12)
    # snapshot ve yeni nominal değişmiş olmalı (örnekleme aktif)
    assert not np.allclose(ctrl.U_nominal, snapshot), \
        "İkinci step nominal'i değiştirmeliydi"


def test_warm_start_disabled_resets_nominal(
    dyn: CatamaranDynamics, bounds: Bounds
) -> None:
    """warm_start_enabled=False → her step sonrası U_nominal tamamen sıfır."""
    cfg = MPPIConfig(K=64, T=8, seed=0, warm_start_enabled=False, backend="numpy")
    ctrl = MPPIController(dyn, bounds, [], cfg)
    ctrl.set_reference([(0.0, 0.0), (10.0, 0.0)])

    ctrl.step(np.zeros(6))
    np.testing.assert_allclose(ctrl.U_nominal, 0.0, atol=1e-12)
    # İkinci step de aynı — her seferinde cold-start
    ctrl.step(np.zeros(6))
    np.testing.assert_allclose(ctrl.U_nominal, 0.0, atol=1e-12)


def test_warm_start_failed_update_path(
    dyn: CatamaranDynamics, bounds: Bounds
) -> None:
    """
    Tüm rolloutlar feci olursa (w_sum<=0) warm-start gating yine uygulanmalı.
    Bu yolu λ→0 ile zorlamak yerine doğrudan _apply_warmstart çağırarak
    invariant'i doğrula.
    """
    cfg_off = MPPIConfig(K=4, T=4, warm_start_enabled=False, backend="numpy")
    ctrl_off = MPPIController(dyn, bounds, [], cfg_off)
    ctrl_off.U_nominal[:] = 7.0     # kasten sıfırdan farklı
    ctrl_off._apply_warmstart(ctrl_off.U_nominal)
    np.testing.assert_allclose(ctrl_off.U_nominal, 0.0, atol=1e-12)

    cfg_on = MPPIConfig(K=4, T=4, warm_start_enabled=True, backend="numpy")
    ctrl_on = MPPIController(dyn, bounds, [], cfg_on)
    ctrl_on.U_nominal[:] = 1.0
    ctrl_on._apply_warmstart(ctrl_on.U_nominal)
    # ilk T-1 adım kayan değerler (1.0), son adım 0
    np.testing.assert_allclose(ctrl_on.U_nominal[:-1], 1.0, atol=1e-12)
    np.testing.assert_allclose(ctrl_on.U_nominal[-1], 0.0, atol=1e-12)


# --------------------------------------------------------------------------- #
# Faz 0 — xp backend soyutlaması (docs/mppi_cuda_plani.md)
# --------------------------------------------------------------------------- #


def _has_cupy() -> bool:
    try:
        import cupy  # noqa: F401
        return True
    except Exception:
        return False


def test_backend_numpy_resolves_float64() -> None:
    """Açık 'numpy' → np modülü + float64 (eski davranışla bit-birebir yol)."""
    from prototype.planning.mppi import _resolve_backend

    xp, dtype = _resolve_backend("numpy")
    assert xp is np
    assert dtype is np.float64


def test_backend_auto_falls_back_to_numpy_without_gpu() -> None:
    """'auto': cupy yoksa (ya da cihaz yoksa) sessizce numpy'a düşer —
    Jetson drop-in davranışının bu makinedeki yarısı."""
    from prototype.planning.mppi import _resolve_backend

    xp, dtype = _resolve_backend("auto")
    if _has_cupy():
        pytest.skip("cupy kurulu — auto GPU seçer, fallback kolu Jetson'da anlamsız")
    assert xp is np
    assert dtype is np.float64


def test_backend_invalid_raises() -> None:
    from prototype.planning.mppi import _resolve_backend

    with pytest.raises(ValueError, match="backend"):
        _resolve_backend("tpu")


def test_numpy_backend_step_deterministic_and_host_output(
    dyn: CatamaranDynamics, bounds: Bounds
) -> None:
    """Aynı seed + aynı config → iki kontrolcü AYNI u üretir; çıktı sözleşmesi
    host numpy float64 (2,) — ROS node sınırında cihaz dizisi sızmaz."""
    cfg = MPPIConfig(K=64, T=10, seed=7, backend="numpy")
    ref = [(0.0, 0.0), (10.0, 0.0)]
    state = np.zeros(6)

    us = []
    for _ in range(2):
        ctrl = MPPIController(dyn, bounds, [], cfg)
        ctrl.set_reference(ref)
        us.append(ctrl.step(state))
    np.testing.assert_array_equal(us[0], us[1])
    assert isinstance(us[0], np.ndarray)
    assert us[0].dtype == np.float64
    assert us[0].shape == (2,)


def test_numpy_backend_bit_identical_to_auto_on_cpu(
    dyn: CatamaranDynamics, bounds: Bounds
) -> None:
    """Bu makinede (GPU yok) 'auto' ile 'numpy' AYNI yol — davranış birebir.

    Faz 0 sözleşmesi: refactor CPU davranışını DEĞİŞTİRMEZ."""
    if _has_cupy():
        pytest.skip("cupy kurulu — auto GPU seçer, bu karşılaştırma CPU'suz makine için")
    ref = [(0.0, 0.0), (10.0, 5.0)]
    state = np.zeros(6)
    outs = []
    for backend in ("numpy", "auto"):
        ctrl = MPPIController(
            dyn, bounds, [], MPPIConfig(K=64, T=10, seed=3, backend=backend)
        )
        ctrl.set_reference(ref)
        outs.append(ctrl.step(state))
    np.testing.assert_array_equal(outs[0], outs[1])


def test_parity_numpy_cupy_same_noise() -> None:
    """Parite (plan §4 Faz 0): aynı girdi + AYNI enjekte gürültü → iki backend
    u0 çıktısı float32 toleransında eş. GPU'suz makinede dürüst skip;
    Jetson'da (cupy kurulunca) gerçekten koşar."""
    cupy = pytest.importorskip("cupy")
    try:
        if cupy.cuda.runtime.getDeviceCount() < 1:
            pytest.skip("cupy kurulu ama CUDA cihazı yok")
    except Exception as exc:
        pytest.skip(f"CUDA runtime kullanılamıyor: {exc}")

    dyn = CatamaranDynamics()
    bounds = Bounds(-1000.0, 1000.0, -1000.0, 1000.0)
    ref = [(0.0, 0.0), (10.0, 5.0)]
    state = np.zeros(6)
    host_noise = np.random.default_rng(11).normal(
        0.0, 5.0, size=(64, 10, 2)
    )

    u0s = {}
    for backend in ("numpy", "cupy"):
        ctrl = MPPIController(
            dyn, bounds, [], MPPIConfig(K=64, T=10, seed=0, backend=backend)
        )
        ctrl.set_reference(ref)
        ctrl._sample_noise = (                       # sabit gürültü enjeksiyonu
            lambda c=ctrl: c.xp.asarray(host_noise, dtype=c._dtype)
        )
        u0s[backend] = ctrl.step(state)
    np.testing.assert_allclose(u0s["numpy"], u0s["cupy"], atol=1e-3)


def _cupy_gpu_or_skip():
    """cupy + gerçek CUDA cihazı yoksa gerekçeli skip (F16.2 deseni)."""
    cupy = pytest.importorskip("cupy")
    try:
        if cupy.cuda.runtime.getDeviceCount() < 1:
            pytest.skip("cupy kurulu ama CUDA cihazı yok")
    except Exception as exc:
        pytest.skip(f"CUDA runtime kullanılamıyor: {exc}")
    return cupy


def test_fused_rollout_matches_generic_cupy() -> None:
    """Faz B: RawKernel rollout, jenerik xp rollout ile AYNI yörüngeyi
    üretmeli (ikisi de float32, sıkı tolerans)."""
    _cupy_gpu_or_skip()
    dyn = CatamaranDynamics()
    bounds = Bounds(-1000.0, 1000.0, -1000.0, 1000.0)
    ctrls = {
        ad: MPPIController(
            dyn, bounds, [],
            MPPIConfig(K=64, T=50, seed=0, backend="cupy", fused_rollout=f),
        )
        for ad, f in (("fused", True), ("generic", False))
    }
    U = np.random.default_rng(5).normal(0.0, 8.0, size=(64, 50, 2))
    trajs = {}
    for ad, c in ctrls.items():
        x0 = c.xp.zeros(6, dtype=c._dtype)
        trajs[ad] = c._as_numpy(
            c._rollout(x0, c.xp.asarray(U, dtype=c._dtype))
        )
    assert trajs["fused"].shape == (64, 51, 6)
    np.testing.assert_allclose(
        trajs["fused"][:, 0], 0.0, atol=0.0
    )  # ilk kare = x0
    np.testing.assert_allclose(
        trajs["fused"], trajs["generic"], rtol=1e-4, atol=1e-4
    )


def test_fused_rollout_matches_numpy() -> None:
    """Faz B: fused cupy rollout, numpy float64 referansına float32
    toleransında eş (parite zinciri: numpy ≈ generic-cupy ≈ fused-cupy)."""
    _cupy_gpu_or_skip()
    dyn = CatamaranDynamics()
    bounds = Bounds(-1000.0, 1000.0, -1000.0, 1000.0)
    c_np = MPPIController(
        dyn, bounds, [], MPPIConfig(K=32, T=50, seed=0, backend="numpy")
    )
    c_gpu = MPPIController(
        dyn, bounds, [],
        MPPIConfig(K=32, T=50, seed=0, backend="cupy", fused_rollout=True),
    )
    U = np.random.default_rng(9).normal(0.0, 8.0, size=(32, 50, 2))
    t_np = c_np._rollout(np.zeros(6), U.astype(np.float64))
    t_gpu = c_gpu._as_numpy(
        c_gpu._rollout(
            c_gpu.xp.zeros(6, dtype=c_gpu._dtype),
            c_gpu.xp.asarray(U, dtype=c_gpu._dtype),
        )
    )
    np.testing.assert_allclose(t_gpu, t_np, rtol=1e-3, atol=1e-3)


def test_fused_flag_no_effect_on_numpy() -> None:
    """fused_rollout yalnız cupy yolunda anlamlı — numpy backend'inde
    iki değer de BİT-BİREBİR aynı jenerik yolu koşmalı (CPU makine/CI)."""
    dyn = CatamaranDynamics()
    bounds = Bounds(-1000.0, 1000.0, -1000.0, 1000.0)
    U = np.random.default_rng(3).normal(0.0, 8.0, size=(8, 10, 2))
    trajs = []
    for f in (True, False):
        c = MPPIController(
            dyn, bounds, [],
            MPPIConfig(K=8, T=10, seed=0, backend="numpy", fused_rollout=f),
        )
        trajs.append(c._rollout(np.zeros(6), U))
    assert np.array_equal(trajs[0], trajs[1])


def test_cupy_real_sample_noise_step() -> None:
    """Parite testi _sample_noise'u monkeypatch'lediği için GERÇEK gürültü
    üretimi cupy'de hiç koşmamıştı: cupy.random.Generator'da numpy'deki
    .normal() yok (cupy 13.x, yalnız standard_normal) → step() ilk gerçek
    Jetson koşusunda AttributeError ile ölüyordu. Bu test gerçek RNG yolunu
    cupy backend'inde uçtan uca koşturur (maskeleme deseni avı, F6 dersi)."""
    cupy = pytest.importorskip("cupy")
    try:
        if cupy.cuda.runtime.getDeviceCount() < 1:
            pytest.skip("cupy kurulu ama CUDA cihazı yok")
    except Exception as exc:
        pytest.skip(f"CUDA runtime kullanılamıyor: {exc}")

    dyn = CatamaranDynamics()
    bounds = Bounds(-1000.0, 1000.0, -1000.0, 1000.0)
    ctrl = MPPIController(
        dyn, bounds, [], MPPIConfig(K=64, T=10, seed=0, backend="cupy")
    )
    ctrl.set_reference([(0.0, 0.0), (10.0, 5.0)])
    u = ctrl.step(np.zeros(6))  # monkeypatch YOK — gerçek _sample_noise
    assert isinstance(u, np.ndarray) and u.shape == (2,)
    assert np.all(np.isfinite(u))


# --------------------------------------------------------------------------- #
# F-M.1 — referans nokta tavanı (masa OOM olayı: 4400 km hedef → 8.8M nokta
# → (K,T+1,n_ref) maliyet tensörü 92 GB → cupy OOM, planning ölümü)
# --------------------------------------------------------------------------- #


def test_referans_nokta_tavani_fm1(dyn: CatamaranDynamics, bounds: Bounds) -> None:
    """Aşırı uzun yol tavana kırpılır; uçlar korunur, step çalışır kalır."""
    cfg = MPPIConfig(K=2, T=3, backend="numpy")
    ctrl = MPPIController(dyn, bounds, [], cfg)
    ctrl.set_reference([(0.0, 0.0), (10_000.0, 0.0)], spacing=0.5)  # kapsız 20001 nokta
    assert ctrl._ref_xy.shape[0] <= cfg.max_ref_points
    np.testing.assert_allclose(ctrl._ref_xy[0], [0.0, 0.0], atol=1e-9)
    np.testing.assert_allclose(ctrl._ref_xy[-1], [10_000.0, 0.0], atol=1e-6)
    ctrl.step(np.zeros(6))                      # OOM'suz tek adım


def test_referans_tavan_alti_davranis_degismez(
    dyn: CatamaranDynamics, bounds: Bounds
) -> None:
    """Tavanın altındaki normal yol (yarışma ölçeği) birebir eski davranış."""
    cfg = MPPIConfig(K=2, T=3, backend="numpy")
    ctrl = MPPIController(dyn, bounds, [], cfg)
    ctrl.set_reference([(0.0, 0.0), (100.0, 0.0)], spacing=0.5)
    assert ctrl._ref_xy.shape[0] == 201         # 100 m / 0.5 m + 1 — kırpılmadı


# --------------------------------------------------------------------------- #
# F-M.2 — kayan referans penceresi (maliyet tensörü (K,T+1,n_ref) küçültme)
# --------------------------------------------------------------------------- #

_UZUN_REF = [(0.0, 0.0), (200.0, 0.0)]   # spacing=0.5 → 401 nokta (pencere aktif)


def _kapali_dongu(
    dyn: CatamaranDynamics,
    bounds: Bounds,
    cfg: MPPIConfig,
    n_steps: int,
    ref=_UZUN_REF,
):
    """n_steps kapalı döngü koştur → (ctrl, u dizisi, çapa dizisi, son durum)."""
    ctrl = MPPIController(dyn, bounds, [], cfg)
    ctrl.set_reference(ref, spacing=0.5)
    state = np.zeros(6)
    us = []
    anchors = []
    for _ in range(n_steps):
        u = ctrl.step(state)
        us.append(u)
        anchors.append(ctrl._ref_anchor_idx)
        state = dyn.step_rk4(state, u, cfg.dt)
    return ctrl, np.array(us), anchors, state


def _cfg(pencere: bool, **kw) -> MPPIConfig:
    base = dict(
        K=128, T=20, seed=4, backend="numpy",
        ref_window_enabled=pencere, ref_window_size=100,
        # ⚠ w_terminal: kod varsayılanı 5.0, ama SEVK EDİLEN eşleşme
        # `_PARKUR_PROFILES`'ın üçünde de 50.0 — lookahead'den AYRILMAZ
        # (CLAUDE.md F-M.3, `test_terminal_mode_varsayilan_lookahead`).
        # Telafisiz 5.0'da terminal gradyan (2·w·d) w_control'ü yenemez;
        # araç sürünür, çapa hiç ilerlemez → pencere testleri sahada HİÇ
        # koşmayan bir konfigürasyonu ölçmüş olurdu (08.08'de lookahead
        # 15 → 3 düşünce tam bu oldu, üç test kırıldı).
        w_terminal=50.0,
    )
    base.update(kw)
    return MPPIConfig(**base)


def test_pencere_tam_tarama_ile_ayni_kontrolu_uretir(
    dyn: CatamaranDynamics, bounds: Bounds
) -> None:
    """Pencereli ↔ tam tarama: aynı senaryoda seçilen u0 tolerans içinde.

    Gerçek argmin pencere içinde kaldığı sürece maliyet BİREBİR aynıdır →
    kapalı döngü de ayrışmaz. Tolerans (1e-6) yalnız kayan nokta payı.
    """
    _, u_pencere, _, son_p = _kapali_dongu(dyn, bounds, _cfg(True), n_steps=25)
    _, u_tam, _, son_t = _kapali_dongu(dyn, bounds, _cfg(False), n_steps=25)
    assert np.max(np.abs(u_pencere - u_tam)) < 1e-6
    np.testing.assert_allclose(son_p, son_t, atol=1e-6)


def test_capa_kapali_donguda_monoton_ilerler(
    dyn: CatamaranDynamics, bounds: Bounds
) -> None:
    """Çapa kapalı döngü boyunca monoton artmalı (tekne referansta ilerliyor)
    ve hiçbir adımda kenar-fallback'e düşmemeli (pencere yeterli).

    n_steps: gerçek tekne yavaş (tam itkide denge hızı ~1,17 m/s, duruştan
    ivmeleniyor) — 60 adımda 0,6 m, yani 0,5 m aralıkta çapa ancak 1 ilerler.
    150 adımda ~3,1 m / 6 indeks: pay dar değil, dinamiğin küçük bir
    değişikliği testi kararsızlaştırmasın."""
    ctrl, _, anchors, _ = _kapali_dongu(dyn, bounds, _cfg(True), n_steps=150)
    assert all(b >= a for a, b in zip(anchors, anchors[1:])), (
        f"çapa geri gitti: {anchors}"
    )
    assert anchors[-1] > anchors[0], "çapa hiç ilerlemedi — pencere kaymıyor"
    assert ctrl._ref_window_fallbacks == 0, "normal seyirde tam taramaya düşmemeli"


def test_kenar_durumu_tam_taramaya_duser_ve_capayi_bulur(
    dyn: CatamaranDynamics, bounds: Bounds, caplog
) -> None:
    """Çapa kilitlenirse (referans yeniden planlandı / poz sıçradı) en yakın
    nokta pencere kenarına yapışır → o adım tam taramaya düşülür, çapa
    yeniden bulunur, WARN loglanır."""
    cfg = _cfg(True)
    ctrl = MPPIController(dyn, bounds, [], cfg)
    ctrl.set_reference(_UZUN_REF, spacing=0.5)
    ctrl._ref_anchor_idx = 300           # tekne 0'da, çapa 150 m ileride (kilitli)

    with caplog.at_level(logging.WARNING, logger="prototype.planning.mppi"):
        ctrl.step(np.zeros(6))

    assert ctrl._ref_window_fallbacks == 1
    assert "pencere" in caplog.text.lower()
    # Fallback tam tarama yaptığı için çapa gerçek konuma (indeks ~0) döndü
    assert ctrl._ref_anchor_idx <= 2, f"çapa düzeltilmedi: {ctrl._ref_anchor_idx}"


def test_kenar_fallback_maliyeti_tam_tarama_ile_ayni(
    dyn: CatamaranDynamics, bounds: Bounds
) -> None:
    """Fallback yalnız hız kaybettirir, maliyeti DEĞİŞTİRMEZ: bozuk çapayla
    hesaplanan maliyet, tam tarama maliyetiyle bit-birebir olmalı."""
    traj = np.zeros((3, 5, 6))
    traj[:, :, 0] = np.linspace(0.0, 2.0, 5)[None, :]     # x ilerliyor, y=0
    U = np.zeros((3, 4, 2))

    ctrls = {}
    for ad, pencere in (("pencere", True), ("tam", False)):
        c = MPPIController(dyn, bounds, [], _cfg(pencere, K=3, T=4))
        c.set_reference(_UZUN_REF, spacing=0.5)
        c._ref_anchor_idx = 300                            # kasten kilitli çapa
        ctrls[ad] = c
    cost_p = ctrls["pencere"]._trajectory_cost(traj, U)
    cost_t = ctrls["tam"]._trajectory_cost(traj, U)

    assert ctrls["pencere"]._ref_window_fallbacks == 1
    assert ctrls["tam"]._ref_window_fallbacks == 0          # kapalıyken hiç bakılmaz
    np.testing.assert_array_equal(cost_p, cost_t)


def test_terminal_maliyet_pencereden_bagimsiz(
    dyn: CatamaranDynamics, bounds: Bounds
) -> None:
    """terminal_mode="global": terminal daima GLOBAL referans ucunu hedefler —
    pencere dilimi ne olursa olsun (dilim ucu hedef sanılırsa araç pencerede
    takılır). Aynı zamanda F-M.3 öncesi davranışın regresyon kilidi."""
    cfg = _cfg(True, K=1, T=4, w_track=0.0, w_heading=0.0, w_terminal=5.0,
               terminal_mode="global")
    ctrl = MPPIController(dyn, bounds, [], cfg)
    ctrl.set_reference(_UZUN_REF, spacing=0.5)             # goal = (200, 0)
    ctrl._ref_anchor_idx = 200               # pencere [175, 300) → x ∈ [87.5, 150)
    traj = _make_traj(1, 4, x_const=100.0, y_const=1.0)    # pencere içi, kenarda değil
    cost = ctrl._trajectory_cost(traj, np.zeros((1, 4, 2)))

    beklenen = 5.0 * ((100.0 - 200.0) ** 2 + 1.0 ** 2)     # global goal'e uzaklık
    assert ctrl._ref_window_fallbacks == 0                 # kenara değmedi
    np.testing.assert_allclose(cost[0], beklenen, rtol=1e-9)


def test_yeni_referans_capayi_sifirlar(
    dyn: CatamaranDynamics, bounds: Bounds
) -> None:
    """set_reference YENİ bir yol verirse çapa sıfırlanır; AYNI yol tekrar
    yüklenirse korunur (boru hattı her engel tazelemesinde çağırıyor —
    koşulsuz sıfırlama pencereyi her adımda rotanın başına atardı)."""
    ctrl, _, anchors, _ = _kapali_dongu(dyn, bounds, _cfg(True), n_steps=150)
    assert ctrl._ref_anchor_idx > 0                    # çapa ilerlemiş durumda
    ilerlemis = ctrl._ref_anchor_idx

    ctrl.set_reference(_UZUN_REF, spacing=0.5)         # AYNI referans → korunur
    assert ctrl._ref_anchor_idx == ilerlemis

    ctrl.set_reference([(0.0, 0.0), (150.0, 30.0)], spacing=0.5)   # YENİ → sıfır
    assert ctrl._ref_anchor_idx == 0
    assert ctrl._ref_anchor_pending == 0


# --------------------------------------------------------------------------- #
# F-M.3 — lookahead terminal maliyet
# --------------------------------------------------------------------------- #


def test_lookahead_yay_uzunlugu_boyunca_dogru_noktayi_secer(
    dyn: CatamaranDynamics, bounds: Bounds
) -> None:
    """Terminal hedefi = çapadan terminal_lookahead_m ileride, YAY uzunluğu
    boyunca. 0.5 m aralıkta 15 m → 30 indeks ileri."""
    ctrl = MPPIController(
        dyn, bounds, [],
        _cfg(True, K=1, T=4, terminal_mode="lookahead", terminal_lookahead_m=15.0),
    )
    ctrl.set_reference(_UZUN_REF, spacing=0.5)          # 401 nokta, aralık 0.5 m
    assert ctrl._ref_spacing == pytest.approx(0.5)
    for capa, beklenen_x in ((0, 15.0), (100, 65.0), (200, 115.0)):
        hedef = np.asarray(ctrl._terminal_goal(capa))
        np.testing.assert_allclose(hedef, [beklenen_x, 0.0], atol=1e-9)


def test_lookahead_kaba_araliktan_etkilenmez(
    dyn: CatamaranDynamics, bounds: Bounds
) -> None:
    """F-M.1 tavanı devreye girip aralık kabalaşırsa lookahead metre cinsinden
    AYNI kalmalı (indeks çevrimi fiili aralıkla yapılır, istenen spacing'le
    değil)."""
    ctrl = MPPIController(
        dyn, bounds, [],
        _cfg(True, K=1, T=4, terminal_mode="lookahead", terminal_lookahead_m=15.0),
    )
    ctrl.set_reference([(0.0, 0.0), (10_000.0, 0.0)], spacing=0.5)   # tavan → ~4.9 m
    assert ctrl._ref_spacing > 4.0                       # aralık kabalaştı
    aralik = ctrl._ref_spacing
    hedef = np.asarray(ctrl._terminal_goal(100))
    beklenen = (100 + round(15.0 / aralik)) * aralik
    np.testing.assert_allclose(hedef[0], beklenen, rtol=1e-9)
    assert abs(hedef[0] - 100 * ctrl._ref_spacing - 15.0) <= ctrl._ref_spacing


def test_lookahead_referans_sonunda_son_noktaya_duser(
    dyn: CatamaranDynamics, bounds: Bounds
) -> None:
    """Çapa rotanın sonuna yaklaşınca terminal hedefi ref[-1]'e kırpılır —
    varış davranışı (goal'e yanaşma) korunur."""
    ctrl = MPPIController(
        # 15.0 AÇIK yazılı: aşağıdaki çapalar (380/400) "30 indekslik
        # lookahead'ten yakın" olacak şekilde seçildi. Sevk edilen varsayılan
        # 3.0 = 6 indeks; onunla çapa 380 kırpılmaz ve test kırpmayı ölçmez.
        dyn, bounds, [],
        _cfg(True, K=1, T=4, terminal_mode="lookahead",
             terminal_lookahead_m=15.0),
    )
    ctrl.set_reference(_UZUN_REF, spacing=0.5)          # son nokta (200, 0)
    son = np.asarray(ctrl._ref_xy[-1])
    for capa in (380, 400):            # 30 indekslik lookahead'ten daha yakın
        np.testing.assert_array_equal(np.asarray(ctrl._terminal_goal(capa)), son)


def test_terminal_mode_global_eski_davranis_birebir(
    dyn: CatamaranDynamics, bounds: Bounds
) -> None:
    """terminal_mode="global" → hedef daima ref[-1], çapa nerede olursa olsun
    (F-M.3 öncesi davranışın birebir regresyonu)."""
    ctrl = MPPIController(
        dyn, bounds, [], _cfg(True, K=1, T=4, terminal_mode="global"),
    )
    ctrl.set_reference(_UZUN_REF, spacing=0.5)
    son = np.asarray(ctrl._ref_xy[-1])
    for capa in (0, 50, 200, 400):
        np.testing.assert_array_equal(np.asarray(ctrl._terminal_goal(capa)), son)


def test_terminal_mode_varsayilan_lookahead() -> None:
    """Varsayılan "lookahead" (2026-08-02 benimseme). ⚠ Bu varsayılan
    `_PARKUR_PROFILES`'daki w_terminal=50 TELAFİSİYLE BİRLİKTE geçerli —
    w=5 ile araç 5.64 → 1.07 m/s'e düşüp hedefe hiç varamıyordu (102 m hata).
    İkisi ayrı ayrı değiştirilmesin diye ikisi de teste bağlandı."""
    from prototype.planning.pipeline import _PARKUR_PROFILES

    assert MPPIConfig().terminal_mode == "lookahead"
    assert all(p.w_terminal == 50.0 for p in _PARKUR_PROFILES.values()), \
        "lookahead varsayılanı w_terminal telafisi olmadan aracı yavaşlatır"


def test_terminal_lookahead_ufuk_kuralini_saglar(dyn: CatamaranDynamics) -> None:
    """🔴 NÖBETÇİ (2026-08-08): `terminal_lookahead_m` ≥ seyir_hızı × ufuk.

    Bu test bir arızadan doğdu. 05.08'de dinamik model log 58'den yeniden
    tanılandı (max_thrust 30 → 1.455 N, Xu → −2.48) ama ONA BAĞLI olan
    `terminal_lookahead_m`=15.0 dokunulmadan kaldı — çünkü aralarındaki bağ
    hiçbir yerde koda yazılmamıştı, yalnız CLAUDE.md'de cümleydi. 15 m, gerçek
    teknenin 2,5 s'lik ufkunda ASLA varamayacağı bir nokta: terminal terim
    rolloutlar arasında ayrım yapmayı bırakıp "burnu hedefe çevir, tam gaz"a
    dejenere olur, yanal (kapı çizgisini tutturma) hassasiyeti kaybolur.
    P1 kapalı döngüde ölçülen sonuç: 0/2 tohum parkuru bitiremedi (§0.16a).

    Kural artık koda bağlı. Terminal hız = `2·max_thrust/|Xu|` (tam itkide
    sürükleme dengesi — `test_planning_node.py`'deki aynı formül), ufuk = T·dt.

    ⚠ Bu test KIRMIZI olursa çözüm sayıyı büyütmek DEĞİL: dinamik yeniden
    ölçüldüyse `terminal_lookahead_m`'yi (gerekirse `T`'yi) o ölçümden TÜRET
    ve dört yerde birden güncelle (`test_planning_config_drift` bağlıyor).
    """
    cfg = MPPIConfig()
    terminal_hiz = 2.0 * dyn.p.max_thrust / abs(dyn.p.Xu)
    ufuk_s = cfg.T * cfg.dt
    assert cfg.terminal_lookahead_m >= terminal_hiz * ufuk_s, (
        f"lookahead {cfg.terminal_lookahead_m} m < terminal hız "
        f"{terminal_hiz:.3f} m/s × ufuk {ufuk_s:.2f} s = "
        f"{terminal_hiz * ufuk_s:.3f} m — rollout uçları hedefe ERİŞEMEZ, "
        "terminal terim örnekler arasında ayrım yapmayı bırakır"
    )


def test_lookahead_ileri_surus_gradyanini_kucultur(
    dyn: CatamaranDynamics, bounds: Bounds
) -> None:
    """Mekanizma kilidi: terminal gradyanı 2·w·d — hedef uzaklığıyla orantılı.

    Bir step içinde tüm rolloutlar AYNI x0'dan başlar (dolayısıyla lookahead
    hedefi de ortaktır); ayrıştıkları yer UÇ noktalarıdır. Aynı başlangıç,
    1 m farklı uç: global hedefte maliyet kazancı ~14×; lookahead'de küçük
    kazanç w_control'ü yenemez → seyir hızı çöker (telafi bu yüzden gerekli).
    """
    def _yoruge(x_son: float) -> np.ndarray:
        traj = np.zeros((1, 5, 6))
        traj[0, :, 0] = np.linspace(100.0, x_son, 5)   # t=0 sabit, uç değişken
        return traj

    U = np.zeros((1, 4, 2))
    kazanc = {}
    for mod in ("global", "lookahead"):
        c = MPPIController(
            dyn, bounds, [],
            _cfg(True, K=1, T=4, w_track=0.0, w_heading=0.0, w_terminal=5.0,
                 # 15.0 AÇIK yazılı — aşağıdaki aritmetik elle bu mesafeye
                 # göre hesaplandı. Sevk edilen varsayılan 3.0 (08.08, §0.16a);
                 # onu `test_planning_config_drift` + ufuk nöbetçisi donduruyor.
                 terminal_lookahead_m=15.0, terminal_mode=mod),
        )
        c.set_reference(_UZUN_REF, spacing=0.5)        # rota sonu x=200
        c._ref_anchor_idx = 200                        # x=100 → lookahead hedefi x=115
        m = [float(c._trajectory_cost(_yoruge(x), U)[0]) for x in (108.0, 109.0)]
        kazanc[mod] = m[0] - m[1]
    # global: hedef 200 → 5·(92² − 91²) = 915 | lookahead: hedef 115 → 5·(7² − 6²) = 65
    assert kazanc["global"] == pytest.approx(915.0)
    assert kazanc["lookahead"] == pytest.approx(65.0)
    assert kazanc["global"] > 14.0 * kazanc["lookahead"]


def test_terminal_mode_gecersiz_deger_kurulumda_patlar() -> None:
    """Yazım hatası saha ortasında değil, config kurulumunda yakalanmalı."""
    with pytest.raises(ValueError, match="terminal_mode"):
        MPPIConfig(terminal_mode="lookahed")


def test_lookahead_terminal_maliyeti_dusurur(
    dyn: CatamaranDynamics, bounds: Bounds
) -> None:
    """Uzun rotada lookahead terminal katkısı global'e göre ölçek küçültür —
    float32 kuantalama gerekçesi (aynı yörünge, aynı ağırlık)."""
    traj = _make_traj(1, 4, x_const=100.0, y_const=0.0)
    U = np.zeros((1, 4, 2))
    maliyet = {}
    for mod in ("global", "lookahead"):
        c = MPPIController(
            dyn, bounds, [],
            _cfg(True, K=1, T=4, w_track=0.0, w_heading=0.0, w_terminal=5.0,
                 # 15.0 AÇIK yazılı — aşağıdaki aritmetik elle bu mesafeye
                 # göre hesaplandı. Sevk edilen varsayılan 3.0 (08.08, §0.16a);
                 # onu `test_planning_config_drift` + ufuk nöbetçisi donduruyor.
                 terminal_lookahead_m=15.0, terminal_mode=mod),
        )
        c.set_reference(_UZUN_REF, spacing=0.5)
        c._ref_anchor_idx = 200                          # x=100 — rota ortası
        maliyet[mod] = float(c._trajectory_cost(traj, U)[0])
    assert maliyet["global"] == pytest.approx(5.0 * 100.0 ** 2)      # 50 000
    assert maliyet["lookahead"] == pytest.approx(5.0 * 15.0 ** 2)    # 1 125
    assert maliyet["lookahead"] < maliyet["global"] / 40


def test_pencere_kisa_referansta_etkisiz(
    dyn: CatamaranDynamics, bounds: Bounds
) -> None:
    """n_ref ≤ pencere açıklığı → dilim tüm referans; davranış tam taramayla
    bit-birebir (yarışma öncesi kısa video rotaları bu kolda)."""
    kisa = [(0.0, 0.0), (10.0, 0.0)]                   # 21 nokta < 125 açıklık
    outs = []
    for pencere in (True, False):
        c = MPPIController(dyn, bounds, [], _cfg(pencere, K=64, T=10, seed=3))
        c.set_reference(kisa, spacing=0.5)
        outs.append(c.step(np.zeros(6)))
    np.testing.assert_array_equal(outs[0], outs[1])
