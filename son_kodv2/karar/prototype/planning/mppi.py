"""
Girdap İDA — MPPI Lokal Planlayıcı (NumPy CPU, Layer 0 prototip)

Amaç:
    Anlık engel kaçınma + diferansiyel tork çıktısı; dalga bozucularına
    dayanıklı yumuşak kontrol. RRT* global yörüngesini referans olarak alır,
    her döngüde diferansiyel thruster komutu (T_l, T_r) üretir.

Algoritma (Williams 2017, IEEE CSM):
    1) U_nominal etrafında K adet gürültülü kontrol dizisi V = U + ε örnekle.
    2) Her V_k için katamaran 3-DOF modelini RK4 ile T adım rollout et.
       (Vektörize: 1000 araç paralel — for-loop sadece T üzerinde.)
    3) Her rollout için maliyet S_k = Σ q(x_t, u_t) hesapla.
    4) Softmax ağırlıkla: w_k ∝ exp(-(S_k - S_min) / λ).
    5) U_new = U_nominal + Σ w_k · ε_k. İlk adım u_0 sahaya gönderilir.
    6) Warm-start: U_nominal'i bir adım kaydır, sonuna sıfır ekle.

Tasarım notları:
    - Vektörize batch dinamiği MPPIController içinde duplikleniyor; sebebi
      CatamaranDynamics.derivatives'in skaler API'sini değiştirmemek
      (Layer 1/2'ye geçişte ROS 2 node aynı sınıfı kullanacak). Parametreler
      tek kaynaktan gelir: dynamics.p (configs/dynamics.yaml).
    - Heading hatası atan2(sin, cos) ile sarılır — π sıçraması maliyeti bozmaz.
    - Engel maliyeti emniyet çemberi içinde quadratic barrier
      (max(0, r_safe - d))²; çember dışında sıfır.
    - Referans takip maliyeti KAYAN PENCERE üzerinde (F-M.2): en yakın nokta
      araması tüm referansta değil, önceki adımın çapası (_ref_anchor_idx)
      etrafındaki [anchor-w/4, anchor+w) diliminde yapılır — (K, T+1, n_ref)
      tensörü 16× küçülür. Çapa pencere kenarına dayanırsa o adım tam
      taramaya düşer (geri manevra / yeniden planlama güvenliği).
    - K=1000 demoda yavaş olabilir; CUDA portu Jetson testinde son adım
      (CLAUDE.md). CPU prototip ~100 ms / iter ölçüldüğünde gerçek zamanlı
      kısıt 50 Hz GPU'da karşılanır.
    - xp backend deseni (docs/mppi_cuda_plani.md Faz 0): çekirdek matematik
      TEK kopya — tüm dizi işlemleri self.xp üzerinden (numpy ya da cupy).
      NumPy yolu float64, davranış eski kodla bit-birebir; CuPy yolu float32
      (Orin fp64 ~1:32). Host↔device sınırı yalnız step() giriş/çıkışı ve
      set_reference/set_obstacles yüklemeleri; viz snapshot'ları cihazda
      kalır, property erişiminde host'a çevrilir.

Çalıştırma (demo + KTR figürü):
    python -m prototype.planning.mppi
"""

from __future__ import annotations

import logging
import math
import time
from dataclasses import dataclass
from pathlib import Path
from typing import List, Optional, Sequence, Tuple

import numpy as np

# NOT: matplotlib yalnızca KTR görselleştirmesi (_draw_demo) içindir; runtime
# ROS node'u çekmesin diye modül seviyesinde import EDİLMEZ, fonksiyon içinde
# tembel import edilir. (Sistem matplotlib'i NumPy 1.x ABI'sine bağlı.)
from prototype.dynamics.catamaran import CatamaranDynamics
from prototype.planning.rrt_star import (
    Bounds,
    CircleObstacle,
    RRTStar,
    RRTStarConfig,
)

_REPO_ROOT = Path(__file__).resolve().parents[2]


def _wrap_angle(a: np.ndarray, xp=np) -> np.ndarray:
    """Açıyı (-π, π] aralığına sar — atan2(sin, cos) tekniği."""
    return xp.arctan2(xp.sin(a), xp.cos(a))


def _resolve_backend(backend: str):
    """'numpy' | 'cupy' | 'auto' → (xp modülü, hesap dtype'ı) — plan Faz 0/A.

    NumPy yolu float64 (eski davranışla bit-birebir); CuPy yolu float32 —
    Orin Ampere iGPU'da fp64 oranı ~1:32, float32 hassasiyeti problem
    ölçeğine (thrust ~30 N, konum <200 m) fazlasıyla yeter (plan §2).
    'auto': cupy + CUDA cihazı varsa GPU, yoksa sessizce numpy (Jetson'da
    `pip install cupy-cuda12x` sonrası drop-in hızlanma).
    """
    if backend == "numpy":
        return np, np.float64
    if backend == "cupy":
        import cupy                     # açık istek — yoksa ImportError doğru

        return cupy, np.float32
    if backend == "auto":
        try:
            import cupy

            if cupy.cuda.runtime.getDeviceCount() > 0:
                return cupy, np.float32
        except Exception:
            pass
        return np, np.float64
    raise ValueError(f"MPPIConfig.backend geçersiz: {backend!r}")


# Faz B — rollout'un tamamı tek CUDA kernel'i (K thread, her thread kendi
# yörüngesini register'larda T adım entegre eder). Fizik _batch_derivatives
# ile İŞLEM SIRASINA KADAR aynı tutuldu (parite testleri sıkı toleransta);
# dalga bozucusu batch yolundaki gibi BİLEREK yok (MPPI planlama modeli).
_ROLLOUT_KERNEL_SRC = r"""
extern "C" {

__device__ __forceinline__ void turev(
    const float psi, const float u, const float v, const float r,
    const float Fx, const float Mz,
    const float Xu, const float Yv, const float Nr,
    const float mass, const float Iz,
    float* d)
{
    float sp, cp;
    sincosf(psi, &sp, &cp);
    d[0] = u * cp - v * sp;
    d[1] = u * sp + v * cp;
    d[2] = r;
    d[3] = (Fx + Xu * u) / mass;
    d[4] = (Yv * v) / mass;
    d[5] = (Mz + Nr * r) / Iz;
}

__global__ void rollout_rk4(
    const float* __restrict__ x0,    /* (6,)          */
    const float* __restrict__ U,     /* (K, T, 2) C   */
    float* __restrict__ traj,        /* (K, T+1, 6) C */
    const int K, const int T,
    const float dt,
    const float Xu, const float Yv, const float Nr,
    const float mass, const float Iz,
    const float half_B, const float maxT)
{
    const int k = blockDim.x * blockIdx.x + threadIdx.x;
    if (k >= K) return;

    float s[6];
    #pragma unroll
    for (int i = 0; i < 6; ++i) s[i] = x0[i];

    float* out = traj + (size_t)k * (T + 1) * 6;
    #pragma unroll
    for (int i = 0; i < 6; ++i) out[i] = s[i];

    const float* Uk = U + (size_t)k * T * 2;
    for (int t = 0; t < T; ++t) {
        float Tl = Uk[2 * t];
        float Tr = Uk[2 * t + 1];
        Tl = fminf(fmaxf(Tl, -maxT), maxT);
        Tr = fminf(fmaxf(Tr, -maxT), maxT);
        const float Fx = Tl + Tr;
        const float Mz = (Tr - Tl) * half_B;

        float k1[6], k2[6], k3[6], k4[6], tmp[6];
        turev(s[2], s[3], s[4], s[5], Fx, Mz, Xu, Yv, Nr, mass, Iz, k1);
        #pragma unroll
        for (int i = 0; i < 6; ++i) tmp[i] = s[i] + 0.5f * dt * k1[i];
        turev(tmp[2], tmp[3], tmp[4], tmp[5], Fx, Mz, Xu, Yv, Nr, mass, Iz, k2);
        #pragma unroll
        for (int i = 0; i < 6; ++i) tmp[i] = s[i] + 0.5f * dt * k2[i];
        turev(tmp[2], tmp[3], tmp[4], tmp[5], Fx, Mz, Xu, Yv, Nr, mass, Iz, k3);
        #pragma unroll
        for (int i = 0; i < 6; ++i) tmp[i] = s[i] + dt * k3[i];
        turev(tmp[2], tmp[3], tmp[4], tmp[5], Fx, Mz, Xu, Yv, Nr, mass, Iz, k4);

        const float c = dt / 6.0f;
        #pragma unroll
        for (int i = 0; i < 6; ++i)
            s[i] += c * (k1[i] + 2.0f * k2[i] + 2.0f * k3[i] + k4[i]);

        float* o = out + (size_t)(t + 1) * 6;
        #pragma unroll
        for (int i = 0; i < 6; ++i) o[i] = s[i];
    }
}

}  /* extern "C" */
"""


# --------------------------------------------------------------------------- #
# Konfigürasyon
# --------------------------------------------------------------------------- #


@dataclass
class MPPIConfig:
    """MPPI hiperparametreleri — saha kalibrasyonunda tune edilir."""

    # Örnekleme & horizon
    K: int = 1000                    # rollout sayısı
    T: int = 50                      # horizon adımı (2.5 s @ dt=0.05)
    dt: float = 0.05                 # entegrasyon adımı (s)
    lambda_: float = 1.0             # softmax sıcaklığı
    # σ_u 5.0 → 0.364 N (2026-08-06). Eski 5.0, itkisi 30 N/motor sanılan
    # HAYALİ tekneye aitti; ölçülen aktüatör ±1.455 N (log 58) olduğu için
    # örneklerin %78'i doygunluğa kırpılıyor, gürültü fiilen "rastgele tam
    # gaz" oluyordu (tepe hızın %37'si kayıp).
    #
    # DEĞER TEK NOKTA DEĞİL, ÖLÇÜLEN BANDIN İÇİNDEN SEÇİLDİ (§0.8g/0.8i):
    #   çalışma bandı  σ/T ∈ [0.15, 0.50]   (kesin çöküş: ≤0.10 ve ≥0.69)
    #   seçilen        σ/T = 0.25  →  σ_u = 0.25 · 1.455 = 0.364 N
    # 3 finalist (σ/T = 0.25 / 0.33 / 0.41) 4 koşulda 40'ar koşumla
    # yarıştırıldı (temiz · bozucu %30 · bozucu %50 · plant yaw 1.9× çevik):
    #   güvenilirlik 40/40 ÜÇÜNDE DE eşit → ayırt etmiyor
    #   gövde payı p10  0.123 / 0.130 / 0.103 m → pratikte eşit (0.41 en kötü)
    #   TEPE HIZ        1.078 / 1.046 / 1.022 m/s  → 0.25 kazanıyor
    #   |Δu₀| (ESC)     0.620 / 0.781 / 0.925      → 0.25 kazanıyor (%21-33)
    #   kenar payı      0.25, [0.10, 0.69] çöküş bandının GEOMETRİK ORTASI
    #                   (√(0.10·0.69) = 0.263) → model hatasına en dayanıklı
    # ⚠ İtki yeniden tanılanırsa σ_u'yu ORANI koruyarak taşı (0.25·T_yeni);
    #   λ tavanı da aynı oranla ölçeklendiği için λ=1.0 geçerli kalır.
    sigma_u: float = 0.364           # N, kontrol gürültüsü σ (her thruster)

    # Maliyet ağırlıkları (CLAUDE.md MPPI bölümü ile uyumlu)
    w_track: float = 5.0             # yörünge sapması (m²)
    w_heading: float = 1.0           # yaw hata (rad²)
    w_obstacle: float = 200.0        # engel ihlali (m²)
    w_control: float = 0.01          # kontrol efor (N²)
    w_boundary: float = 1000.0       # sınır dışı adım sayısı
    w_terminal: float = 5.0          # terminal goal yakınlığı (m²)

    # Engel emniyet payı — quadratic barrier'ın başladığı yarıçap (r + margin).
    # 0.5 → 1.0 (2026-08-02, F9.2 kapatma): MPPI aracı NOKTA sayar, tekne
    # genişliği (0.75 m → yarı genişlik 0.375 m) maliyete girmez; tek pay bu.
    # Ölçüm (200 m rota, rotanın üstünde 2 m yarıçaplı engel): margin 0.5 →
    # engel yüzeyine min açıklık +0.08 m → gövde payı **−0.29 m = ÇARPMA**;
    # margin 1.0 → açıklık +0.54 m, gövde payı **+0.17 m**. Yaklaşık kural:
    # açıklık ≈ margin − 0.45 m. Bedel yalnız iz sapmasında ~0.1 m (RMS
    # 0.67 → 0.76), hız ve varış performansı değişmiyor.
    # ⚠ ÜST SINIR — Parkur-2 geçidi: net açıklık ~1.35 m (kod_denetimi F10.1),
    # yani merkez hattından duba YÜZEYİNE 0.675 m. Bunun üstündeki her margin
    # geçidin İÇİNİ sürekli ceza bölgesi yapar; çok büyürse (≥1.5) MPPI
    # geçitten geçmek yerine ETRAFINDAN dolaşmayı ucuz bulur (görev ihlali).
    # 1.0 m bu sınırın altında kalır — geçit ölçümü CLAUDE.md'de.
    # ⚠ Bu SOFT ceza; RRT* safety_margin'i HARD kısıt (bkz. rrt_star.py) —
    # ikisi eşit OLMAK ZORUNDA DEĞİL, sıralı olmalı: RRT* ≤ MPPI.
    #
    # 🔴 **1.0'DA KALDI — büyütmek DENENDİ ve GERİ ALINDI (2026-08-09).**
    # Gövde payını pozitife çıkarmak için 1.0 → 1.4 denendi: kapı takibi olan
    # kolda işe yaradı (−0,019 → +0,273 m) ama **model gelmeyen kolu kırdı**
    # (ham güzergah noktasına sürüş: 3,3/4 → 1/4 nokta; 1.2'de bile 2/4).
    # Sebep: hakemin noktaları dubaya ~2,2 m yakın olabiliyor (md 5.5.2.2) ve
    # büyüyen ceza halkası aracı varış yarıçapına (2,0 m) sokmuyor.
    # `.pt` gelmezse koşulacak kol tam da o kol → küresel pay BÜYÜTÜLEMEZ.
    #
    # Gövde payı bunun yerine **B2 huni** ile kazanıldı: kapı direkleri kendi
    # payını `planning_node._huni_payi`'den alır (`gate_post_margin_m`), o da
    # model YOKKEN hiç devreye girmez → iki kol birbirini bozmuyor.
    obstacle_margin: float = 1.0     # m

    # Parkur-3 kamikaze modu — hedef noktasına Gaussian çekici (negatif maliyet,
    # engel maliyetini ezer). Kapalı: tamamen geriye uyumlu.
    kamikaze_mode: bool = False
    kamikaze_target: Optional[Tuple[float, float]] = None
    w_kamikaze: float = 50.0         # Gaussian zirve büyüklüğü (≥0)
    kamikaze_radius: float = 5.0     # m, Gaussian σ (etki yarıçapı)

    # Warm-start: True → her step'te U_nominal[:-1]=U_new[1:], U_nominal[-1]=0.
    # False → her step'te U_nominal sıfırlanır (cold-start; debug/test için).
    warm_start_enabled: bool = True

    # F-M.1: referans nokta tavanı — set_reference yoğunlaştırması bunu
    # aşamaz (aşınca spacing otomatik kabalaşır, uçlar korunur). Masa olayı
    # (2026-07-12): 4400 km hedef → 8.8M nokta → (K,T+1,n_ref) maliyet
    # tensörü 92 GB → cupy OOM, planning ölümü. 2048 = tam yarışma rotası
    # (≤1 km, 0.5 m aralık) + pay; normal kullanımda tavana değilmez.
    max_ref_points: int = 2048

    # F-M.2: kayan referans penceresi. Tavandaki rotada bile maliyet tensörü
    # (K, T+1, n_ref) = 1000·51·2048 ≈ 104 M eleman (float32 418 MB; ara
    # dx/dy/kare geçicileriyle ~1.7 GB) — Orin Nano'nun 8 GB PAYLAŞILAN
    # belleğinde hem hız hem kapasite darboğazı. Tekne referans üzerinde
    # MONOTON ilerlediği için tüm referansa bakmak gereksiz: bir önceki
    # adımın en yakın indeksi (_ref_anchor_idx) etrafındaki dilim yeter.
    # Pencere NOKTA cinsindendir; metre karşılığı set_reference(spacing) ile
    # ölçeklenir (0.5 m aralıkta 100 nokta = 50 m ileri, 25 nokta = 12.5 m
    # geri). Horizon'da kat edilen yol ≤ ~10 m (2.5 s, maks ~7.5 m/s) —
    # ileri pay 5× güvenli. Kenar durumunda otomatik tam taramaya düşer.
    ref_window_size: int = 100       # ileri pencere derinliği (nokta)
    ref_window_enabled: bool = True  # False → eski tam tarama (regresyon testi)

    # 🔴 F-F.21 (14.08.2026, GIRDAP_DURUM §1.04) — TAM TARAMADAN SONRA cupy
    # BELLEK HAVUZUNU SERBEST BIRAK. Jetson'da bu bir performans ayarı değil,
    # **SİSTEM ÇÖKMESİNİ ÖNLEYEN** kapıdır:
    #
    #   • cupy'nin varsayılan havuzu, kullanıcı diziyi serbest bıraksa bile
    #     blokları ÖNBELLEĞE ALIR ve işletim sistemine GERİ VERMEZ (cupy belgesi:
    #     "a memory pool preserves any allocations even if they are freed").
    #   • Orin Nano'da GPU belleği AYRI DEĞİL — **sistem RAM'inin kendisi**.
    #     Yani havuzda tutulan her bayt, MAVROS'un/kaydedicinin/algının
    #     kullanamadığı bayttır.
    #   • Tam tarama tensörü ölçüldü: n_ref=2048'de d2 tek başına float32 389 MiB
    #     ve geçicilerle tepe ~3-4×. **TEK BİR kenar-fallback havuzun tepe
    #     değerini kalıcı olarak yükseltmeye yeter.**
    #
    # 14.08 ölçümü (13:15 koşumu, 108 dk): RAM 14:00→14:05 arası **+2 GB**
    # sıçradı, 50 dakika o seviyede kaldı, 14:53'te bir anda serbest kaldı.
    # Aynı anda `lfb` 2 MB'a düştü ve **tüm yığın ~10 saniye dondu** (MAVROS
    # IMU dahil → makinenin kendisi dondu, 1 m/s'de ~10 m kör seyir).
    #
    # ⚠ Neden `set_limit` DEĞİL: sert sınır aşılınca cupy OOM atar → MPPI ölür →
    # `_safe_stop`. Yani "yavaş" yerine "görev biter". `free_all_blocks()` ise
    # yalnız KULLANILMAYAN blokları verir, asla OOM üretmez.
    # ⚠ Neden her adımda değil: serbest bırakılan blok bir sonraki tahsiste
    # sürücüden yeniden alınır (yavaş). Fallback ZATEN nadir ve tam da büyük
    # tahsisin olduğu yer — doğru kanca orası.
    bellek_havuzu_temizle: bool = True   # False → eski davranış (A/B ölçümü)

    # F-M.3: terminal maliyet hedefi.
    #   "global"    = VARSAYILAN, eski davranış — daima ref[-1] (rotanın SONU).
    #   "lookahead" = çapadan `terminal_lookahead_m` ileride, yay uzunluğu
    #                 boyunca referans noktası; referans bitince ref[-1]'e
    #                 düşer (varış davranışı korunur).
    #
    # ⚠⚠ MOD DEĞİŞTİRİRKEN w_terminal'i BİRLİKTE AYARLA. Terminal terim
    # w·d² olduğundan ileri sürüş gradyanı 2·w·d — yani HEDEF UZAKLIĞIYLA
    # ORANTILI. Uzak hedefte (140 m) gradyan w_control'ü ezer ve fiilen
    # SEYİR HIZI KONTROLCÜSÜ görevi görür (maliyette ayrı hız terimi YOK).
    # 15 m'lik hedefte aynı w ile gradyan ~13× düşer ve w_control kazanır:
    # ölçüm (200 m rota, PARKUR2 profili) — global w=5 → 5.64 m/s, hedefe
    # 1.42 m; lookahead w=5 → **1.07 m/s, hedefe hiç varamadı (102 m)**.
    # Telafi ~10×: lookahead + w_terminal=50 → 5.31 m/s, hedefe 1.49 m.
    #
    # Lookahead'in ölçülen kazancı (telafi edilmiş w ile, DÖNÜŞLÜ rota):
    # iz sapması RMS 1.66 → 0.52 m, maks 4.61 → 1.89 m (global terminal
    # köşe keser; lookahead rotayı takip eder). Düz rotada fark küçük.
    # Ayrıca sayısal: w·d² uzun rotada dev sabit ofset üretir (300 m'de
    # ~1.1e5, 1 km'de ~3.6e6); Jetson float32 yolunda bu büyüklüğün ULP'si
    # ince terimleri kuantalar (ölçüm: 858 m'de ULP 0.25, 3.9 km'de 8.0 —
    # track std'si 8.6 iken). Lookahead terminali ~1e2-1e3'e indirir.
    #
    # terminal_lookahead_m ≥ seyir_hızı × horizon (T·dt) olmalı; altında hedef
    # rollout uçlarının gerisinde kalır ve fren gibi davranır (ölçüm: 10 m →
    # 3.85 m/s'de sınırlandı). 40 m'de ise engel açıklığı bozulur (+0.19 →
    # −0.12 m): uzadıkça global davranışa yaklaşır.
    # BENİMSENDİ (2026-08-02): varsayılan "lookahead" + profillerde
    # w_terminal=50 (pipeline._PARKUR_PROFILES). İkisi BİRLİKTE değişir;
    # w_terminal telafisi olmadan lookahead aracı 1.07 m/s'e düşürür.
    terminal_mode: str = "lookahead"
    # 🔴 15.0 → 3.0 (2026-08-08, kaptan onayı). 15 m yukarıdaki ölçümlerin
    # yapıldığı HAYALİ tekneye aitti (itki 30 N sanılıyordu, seyir ~5,3 m/s:
    # 5,3 × 2,5 s = 13,25 → 15 kuralı sağlıyordu). 05.08'de dinamik log 58'den
    # tanılandı (max_thrust 1,455 N, Xu −2,48) ve gerçek seyir **1,05 m/s**
    # çıktı → aynı kural artık **1,05 × 2,5 = 2,62 m** diyor; 15 m ufkun
    # 5,8 KATI, yani rollout uçlarının ERİŞEMEYECEĞİ bir nokta. O zaman
    # terminal terim örnekler arasında ayrım yapmayı bırakır (hepsi hedeften
    # ~13 m uzakta biter) ve fiilen "burnu hedefe çevir, tam gaz"a dejenere
    # olur — yanal hassasiyet, yani kapı çizgisini tutturma yeteneği kaybolur.
    #
    # ÖLÇÜM (P1 kapalı-döngü, planning_node zincirinin aynası, 6 kapı + 6 GN,
    # docs/parkur1_kontrol_listesi.md §4):
    #   la=15 (eski) → 0/2 tohum parkuru BİTİREMEDİ (3/6 ve 1/6 GN, 400 s)
    #   la=3         → 3/3 tohum bitti (101/140/252 s), ort 4,0/6 kapı
    #   la=5         → 3/3 bitti ama ort 3,0/6 kapı (puan üreten kapı daha az)
    # Kamera 360°/25 m yapılıp algı MÜKEMMELLEŞTİRİLDİĞİNDE de la=15 bir
    # tohumda varamadı → sorun algıda değil, ufuk/lookahead ilişkisinde.
    # ⚠ `T` BİLEREK 50 kaldı: hesap maliyeti (Orin Nano) değişmesin.
    # ⚠ 3,0 uydurulmuş bir sayı DEĞİL: kuralın kendi alt sınırı (2,62 m)
    #   yukarı yuvarlanmış hâli. Tekne hızı ölçümle değişirse bu da değişir.
    terminal_lookahead_m: float = 3.0

    # Hesap backend'i (docs/mppi_cuda_plani.md): "numpy" = CPU float64
    # (eski davranış birebir), "cupy" = GPU float32, "auto" = cupy+CUDA
    # varsa GPU yoksa numpy. Jetson D3'te ölçüm: bench_mppi.py --backend.
    backend: str = "auto"

    # Faz B (docs/mppi_cuda_plani.md): cupy yolunda rollout tek CUDA
    # kernel'i (RawKernel) — T ardışık RK4 adımının kernel-launch-overhead'i
    # kalkar (Faz A ölçümü: rollout 266 ms'nin ~tamamı launch maliyetiydi).
    # numpy yolunda etkisiz. False = jenerik xp rollout (A/B ölçümü için).
    fused_rollout: bool = True

    seed: int = 0

    def __post_init__(self) -> None:
        """Yazım hatası saha ortasında değil, kurulumda patlasın (F-P.18 ruhu)."""
        if self.terminal_mode not in ("lookahead", "global"):
            raise ValueError(
                f"MPPIConfig.terminal_mode geçersiz: {self.terminal_mode!r} "
                "(geçerli: 'lookahead', 'global')"
            )


# --------------------------------------------------------------------------- #
# Kontrolör
# --------------------------------------------------------------------------- #


class MPPIController:
    """
    Vektörize MPPI — NumPy CPU.

    Tipik kullanım:
        ctrl = MPPIController(dynamics, bounds, obstacles, MPPIConfig())
        ctrl.set_reference(rrt_star_path)
        u = ctrl.step(state)        # (2,) — (T_l, T_r)
    """

    def __init__(
        self,
        dynamics: CatamaranDynamics,
        bounds: Bounds,
        obstacles: Sequence[CircleObstacle],
        cfg: Optional[MPPIConfig] = None,
    ) -> None:
        self.dyn = dynamics
        self.p = dynamics.p
        self.bounds = bounds
        self.cfg = cfg or MPPIConfig()
        self.xp, self._dtype = _resolve_backend(self.cfg.backend)
        self._rng = self.xp.random.default_rng(self.cfg.seed)

        # Engel dizilerini önceden hazırla (her step'te yeniden alokasyon yok)
        self._load_obstacles(obstacles)

        # Warm-start: nominal kontrol dizisi (T, 2)
        self.U_nominal = self.xp.zeros((self.cfg.T, 2), dtype=self._dtype)

        # Yoğunlaştırılmış RRT* referansı
        self._ref_xy: Optional[np.ndarray] = None      # (n_ref, 2)
        self._ref_psi: Optional[np.ndarray] = None      # (n_ref,)
        # Referansın host kopyası — YALNIZ set_reference'ta "referans gerçekten
        # değişti mi?" karşılaştırması için (cihazda karşılaştırmak senkron
        # maliyeti getirir; 2048×2 float64 = 32 KB, ihmal edilebilir).
        self._ref_xy_host: Optional[np.ndarray] = None
        # Referansın FİİLİ nokta aralığı (m) — set_reference'ta yeniden
        # örnekleme uniform olduğu için tek skaler yeter; F-M.3 lookahead'i
        # metreyi indekse bununla çevirir (tavan uygulanınca aralık kabalaşır,
        # istenen `spacing` değil bu değer doğrudur).
        self._ref_spacing: float = 0.0

        # F-M.2 kayan pencere durumu: çapa = teknenin son step'te en yakın
        # olduğu GLOBAL referans indeksi. _trajectory_cost bir sonraki adayı
        # _pending'e yazar, step() commit eder (maliyet fonksiyonu idempotent
        # kalsın — art arda iki çağrı aynı pencereyi kullanır).
        self._ref_anchor_idx: int = 0
        self._ref_anchor_pending: int = 0
        self._ref_window_fallbacks: int = 0   # kenar durumu sayacı (test + log)
        self._bellek_temizleme_sayaci: int = 0  # F-F.21 — kaç kez havuz boşaltıldı

        # Görselleştirme için son rollout snapshot'ı
        self._last_traj: Optional[np.ndarray] = None    # (K, T+1, 6)
        self._last_weights: Optional[np.ndarray] = None # (K,)

    # ----- referans -----

    def set_reference(
        self,
        path_xy: Sequence[Tuple[float, float]],
        spacing: float = 0.5,
    ) -> None:
        """
        RRT* waypoint zincirini yoğun referansa yeniden örnekle.
        spacing : ardışık ref noktaları arası m. 0.5 m kapsama → noktasal
                  arama maliyeti makul, takip kalitesi yeterli.
        """
        pts = np.asarray(path_xy, dtype=float)
        if pts.shape[0] < 2:
            raise ValueError("Referans yörünge en az 2 nokta içermeli")

        # Kümülatif yay uzunluğu üzerinden uniform örnekle
        seg_len = np.linalg.norm(np.diff(pts, axis=0), axis=1)
        s = np.concatenate([[0.0], np.cumsum(seg_len)])
        n_new = max(2, int(s[-1] / spacing) + 1)
        if n_new > self.cfg.max_ref_points:
            # F-M.1: OOM koruması — ValueError DEĞİL (F10.1 dersi: planlama
            # yolunda fırlatılan hata node'u öldürür); kabalaştır ve uyar.
            logging.getLogger(__name__).warning(
                "referans %d noktaya bölünecekti (yol %.0f m) — tavan %d "
                "uygulandı (F-M.1 OOM koruması); hedef koordinatları kontrol et",
                n_new, s[-1], self.cfg.max_ref_points,
            )
            n_new = self.cfg.max_ref_points
        s_new = np.linspace(0.0, s[-1], n_new)
        ref = np.column_stack(
            [np.interp(s_new, s, pts[:, 0]),
             np.interp(s_new, s, pts[:, 1])]
        )

        # Tangent açısı; son nokta önceki tangenti devralır
        tan = np.diff(ref, axis=0)
        psi = np.arctan2(tan[:, 1], tan[:, 0])
        psi = np.concatenate([psi, [psi[-1]]])

        # F-M.2: kayan pencere çapası YENİ bir referansta sıfırlanır — ama
        # yalnız referans GERÇEKTEN değiştiyse. Boru hattı
        # (PlanningPipeline._rebuild_mppi) her engel tazelemesinde AYNI yolla
        # set_reference çağırır (5-10 Hz); koşulsuz sıfırlama pencereyi sürekli
        # rotanın başına atar → her adımda kenar-fallback + WARN → kazanç
        # tamamen kaybolur. (U_nominal'in F11.1 dersinin referans karşılığı.)
        if self._ref_xy_host is None or not np.array_equal(ref, self._ref_xy_host):
            self._ref_anchor_idx = 0
            self._ref_anchor_pending = 0
        self._ref_xy_host = ref
        self._ref_spacing = float(s[-1]) / (n_new - 1) if n_new > 1 else 0.0

        # Sınır dönüşümü: referans cihaza BİR kez kopyalanır (seyrek çağrı)
        self._ref_xy = self.xp.asarray(ref, dtype=self._dtype)
        self._ref_psi = self.xp.asarray(psi, dtype=self._dtype)

    def reset_warm_start(self) -> None:
        """Başarısız iterasyon sonrası nominal kontrolü sıfırla."""
        self.U_nominal[:] = 0.0

    def carry_state_from(self, other: "MPPIController") -> None:
        """Eski kontrolcünün SICAK durumunu devral — parkur geçişi (F11.1+F-M.2).

        Ağırlık profili değişince (`_active_mppi_cfg`) boru hattı YENİ bir
        kontrolcü kurmak zorunda; taze nesne hem warm-start dizisini hem kayan
        pencere çapasını sıfırdan başlatır → tek adımlık zikzak + tek adımlık
        tam tarama (~790 ms ölçüldü). İkisi de burada taşınır.

        `set_reference` çağrıldıktan SONRA çağrılmalı: çapa yalnız referans
        birebir aynıysa taşınır — yol gerçekten değiştiyse eski indeks
        anlamsızdır, 0'da kalır (kenar-fallback ilk adımda düzeltir).
        """
        if other.U_nominal.shape == self.U_nominal.shape:
            self.U_nominal[:] = other.U_nominal
        ayni_referans = (
            self._ref_xy_host is not None
            and other._ref_xy_host is not None
            and np.array_equal(self._ref_xy_host, other._ref_xy_host)
        )
        if ayni_referans:
            self._ref_anchor_idx = other._ref_anchor_idx
            self._ref_anchor_pending = other._ref_anchor_pending

    def set_obstacles(self, obstacles: Sequence[CircleObstacle]) -> None:
        """Engel listesini yerinde güncelle — warm-start (U_nominal) KORUNUR.

        Kontrolcüyü yeniden yaratmak yerine bunu çağırmak, önceki adımın nominal
        kontrol dizisini yaşatır (aksi halde her engel/referans tazelemesinde
        soğuk başlangıç → zikzak). __init__ ile aynı ön-hesaplama.
        """
        self._load_obstacles(obstacles)

    def _bellek_havuzunu_serbest_birak(self) -> None:
        """F-F.21: cupy havuzundaki KULLANILMAYAN blokları işletim sistemine ver.

        numpy yolunda hiçbir şey yapmaz (havuz yok). cupy import edilemezse ya
        da sürücü hata verirse sessizce geçer — bu bir **iyileştirme**dir,
        kontrol döngüsünü asla düşürmemeli.
        """
        if not self.cfg.bellek_havuzu_temizle:
            return
        if getattr(self.xp, "__name__", "") != "cupy":
            return
        try:
            self.xp.get_default_memory_pool().free_all_blocks()
            self.xp.get_default_pinned_memory_pool().free_all_blocks()
            self._bellek_temizleme_sayaci += 1
        except Exception:                    # sürücü/sürüm farkı — kritik değil
            pass

    def bellek_havuzu_bayt(self) -> tuple[int, int]:
        """(kullanılan, havuzun tuttuğu toplam) bayt — teşhis için.

        numpy yolunda (0, 0). Bu sayı bantta/logda görünmezse F-F.21'in
        tekrarlayıp tekrarlamadığı **sonradan anlaşılamaz** (14.08'de tam bu
        yüzden 2 GB'lık sıçramanın sebebi bant üzerinden bulunamadı).
        """
        if getattr(self.xp, "__name__", "") != "cupy":
            return (0, 0)
        try:
            mp = self.xp.get_default_memory_pool()
            return (int(mp.used_bytes()), int(mp.total_bytes()))
        except Exception:
            return (0, 0)

    @property
    def backend_adi(self) -> str:
        """Fiilen ÇÖZÜLEN hesap yolu, ör. `"cupy/float32"` — teşhis için.

        🔴 **Neden gerekli (2026-08-10).** `MPPIConfig.backend` varsayılanı
        `"auto"` ve `_resolve_backend` cupy'yi bulamazsa **hiçbir uyarı
        basmadan** numpy/float64'e iniyor. İki yol arasındaki fark ölçüldü
        (GIRDAP_DURUM §0.26c, RTX 4060 Laptop, K=1000/T=50):

            N=100 engel → cupy 3,7 ms · numpy 144 ms  (10 Hz bütçesi 100 ms)

        Yani sessiz düşüş, kontrol döngüsünü bütçe dışına atar. Jetson'da cupy
        kurulumu kırılgan (cupy 14 numpy≥2 ister, gtsam numpy 1.x ister — §0.8e)
        ve orada ekransız koştuğumuz için belirti VERMEZ. Açılışta loglanmalı:
        log yoksa sahada hangi tablonun geçerli olduğunu kimse bilemez.
        """
        return f"{getattr(self.xp, '__name__', '?')}/{np.dtype(self._dtype).name}"

    def _load_obstacles(self, obstacles: Sequence[CircleObstacle]) -> None:
        """Engel dizilerini backend'e yükle (host→device kopyası seyrek)."""
        if obstacles:
            self._obs_xy = self.xp.asarray(
                [[o.cx, o.cy] for o in obstacles], dtype=self._dtype
            )
            # B2 HUNİ: engelin kendi `margin`'i varsa küresel payın YERİNE geçer
            # (kapı direkleri — payları o an ölçülen açıklıktan türer, yoksa
            # küresel 1,0 m dar bir kapıyı kapatır). None → eski davranış.
            self._obs_r = self.xp.asarray(
                [
                    o.r + (
                        self.cfg.obstacle_margin
                        if getattr(o, "margin", None) is None
                        else o.margin
                    )
                    for o in obstacles
                ],
                dtype=self._dtype,
            )
        else:
            self._obs_xy = self.xp.zeros((0, 2), dtype=self._dtype)
            self._obs_r = self.xp.zeros(0, dtype=self._dtype)

    def _as_numpy(self, arr):
        """Sınır dönüşümü: xp dizisi → host numpy (numpy yolunda no-op)."""
        if self.xp is np:
            return arr
        return self.xp.asnumpy(arr)

    # ----- batch dinamik (vektörize 3-DOF + RK4) -----

    def _batch_derivatives(
        self, st: np.ndarray, ct: np.ndarray
    ) -> np.ndarray:
        """st: (K, 6), ct: (K, 2) → (K, 6) durum türevi."""
        xp = self.xp
        max_T = self.p.max_thrust
        # Doygunluk — katamaran modeliyle birebir aynı
        ct = xp.clip(ct, -max_T, max_T)

        psi = st[:, 2]
        u = st[:, 3]
        v = st[:, 4]
        r = st[:, 5]
        T_l = ct[:, 0]
        T_r = ct[:, 1]

        Fx = T_l + T_r
        Mz = (T_r - T_l) * self.p.thruster_spacing / 2.0

        d = xp.empty_like(st)
        cos_p = xp.cos(psi)
        sin_p = xp.sin(psi)
        d[:, 0] = u * cos_p - v * sin_p
        d[:, 1] = u * sin_p + v * cos_p
        d[:, 2] = r
        d[:, 3] = (Fx + self.p.Xu * u) / self.p.mass
        d[:, 4] = (self.p.Yv * v) / self.p.mass
        d[:, 5] = (Mz + self.p.Nr * r) / self.p.inertia_z
        return d

    def _batch_rk4(self, st: np.ndarray, ct: np.ndarray) -> np.ndarray:
        dt = self.cfg.dt
        k1 = self._batch_derivatives(st, ct)
        k2 = self._batch_derivatives(st + 0.5 * dt * k1, ct)
        k3 = self._batch_derivatives(st + 0.5 * dt * k2, ct)
        k4 = self._batch_derivatives(st + dt * k3, ct)
        return st + (dt / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4)

    # Sınıf düzeyi derleme önbelleği: None=denenmedi, False=derlenemedi
    # (bir kez uyar, jenerik yola düş), aksi hâlde RawKernel nesnesi.
    _fused_kernel = None

    def _rollout(self, x0: np.ndarray, U: np.ndarray) -> np.ndarray:
        """U: (K, T, 2) → trajectories (K, T+1, 6)."""
        if (
            self.cfg.fused_rollout
            and self.xp is not np
            and self._dtype == np.float32
        ):
            traj = self._rollout_fused(x0, U)
            if traj is not None:
                return traj
        K, T, _ = U.shape
        traj = self.xp.empty((K, T + 1, 6), dtype=self._dtype)
        traj[:, 0] = x0[None, :]
        st = traj[:, 0].copy()
        for t in range(T):
            st = self._batch_rk4(st, U[:, t])
            traj[:, t + 1] = st
        return traj

    def _rollout_fused(self, x0, U):
        """Faz B: tüm rollout tek CUDA kernel launch'ı (K thread × T adım).

        Derleme başarısızsa None döner (çağıran jenerik yola düşer) — saha
        kodu kernel yüzünden ölmez, yalnız yavaşlar ve bir kez WARN loglanır.
        """
        xp = self.xp
        kern = MPPIController._fused_kernel
        if kern is False:
            return None
        if kern is None:
            try:
                kern = xp.RawKernel(_ROLLOUT_KERNEL_SRC, "rollout_rk4")
                kern.compile()
                MPPIController._fused_kernel = kern
            except Exception as exc:  # NVRTC/driver sorunu — jenerik yola düş
                MPPIController._fused_kernel = False
                logging.getLogger(__name__).warning(
                    "fused rollout derlenemedi (%s) — jenerik xp rollout "
                    "kullanılacak (yavaş ama doğru)", exc,
                )
                return None
        K, T, _ = U.shape
        traj = xp.empty((K, T + 1, 6), dtype=self._dtype)
        x0d = xp.ascontiguousarray(x0, dtype=self._dtype)
        Ud = xp.ascontiguousarray(U, dtype=self._dtype)
        p = self.p
        block = 128
        grid = (K + block - 1) // block
        kern(
            (grid,), (block,),
            (
                x0d, Ud, traj,
                np.int32(K), np.int32(T),
                np.float32(self.cfg.dt),
                np.float32(p.Xu), np.float32(p.Yv), np.float32(p.Nr),
                np.float32(p.mass), np.float32(p.inertia_z),
                np.float32(p.thruster_spacing * 0.5),
                np.float32(p.max_thrust),
            ),
        )
        return traj

    # ----- referans penceresi (F-M.2) -----

    def _ref_window(self, n_ref: int) -> Tuple[int, int]:
        """Çapa etrafındaki [lo, hi) referans dilimi.

        Asimetrik: geriye `size//4`, ileriye `size` — tekne referans üzerinde
        ilerlediği için ileri pay geniş, geri pay yalnız salınım/akıntı
        sapmasını karşılar. Dilim daima referans sınırlarına kırpılır; kısa
        referansta (n_ref ≤ span) doğal olarak tam taramaya eşitlenir.
        """
        w = max(1, int(self.cfg.ref_window_size))
        anchor = min(max(self._ref_anchor_idx, 0), n_ref - 1)
        return max(0, anchor - w // 4), min(n_ref, anchor + w)

    def _nearest_ref(
        self, xs: np.ndarray, ys: np.ndarray, lo: int, hi: int
    ) -> Tuple[np.ndarray, np.ndarray]:
        """[lo, hi) dilimindeki en yakın referans → (idx_local, d2_min).

        idx_local dilime GÖRE indekstir (global için `+ lo`). Baskın maliyet
        tensörü (K, T+1, hi-lo) burada doğar ve fonksiyon kapsamında ölür —
        çağıran yalnız iki (K, T+1) diziyi tutar.
        """
        xp = self.xp
        ref = self._ref_xy[lo:hi]                         # cihazda view, kopya yok
        dx = xs[:, :, None] - ref[None, None, :, 0]
        dy = ys[:, :, None] - ref[None, None, :, 1]
        d2 = dx * dx + dy * dy                            # (K, T+1, hi-lo)
        idx_local = xp.argmin(d2, axis=-1)                # (K, T+1)
        d2_min = xp.take_along_axis(d2, idx_local[..., None], axis=-1)[..., 0]
        return idx_local, d2_min

    def _terminal_goal(self, anchor: int) -> np.ndarray:
        """Terminal maliyetin hedef noktası (F-M.3) — (2,) cihaz dizisi.

        "global": rotanın sonu (ref[-1]) — eski davranış.
        "lookahead": çapadan `terminal_lookahead_m` ileride, YAY UZUNLUĞU
        boyunca. Yeniden örnekleme uniform olduğu için yay → indeks çevrimi
        tek bölme; referans biterse doğal olarak ref[-1]'e kırpılır (varış
        davranışı aynen korunur).
        """
        ref = self._ref_xy
        if self.cfg.terminal_mode != "lookahead" or self._ref_spacing <= 0.0:
            return ref[-1]
        adim = int(round(self.cfg.terminal_lookahead_m / self._ref_spacing))
        return ref[min(ref.shape[0] - 1, max(0, anchor) + adim)]

    def _window_edge_hit(
        self, idx_local: np.ndarray, lo: int, hi: int, n_ref: int
    ) -> bool:
        """En yakın nokta pencerenin YAPAY kenarına yapıştı mı?

        Yapıştıysa gerçek en yakın nokta pencere dışında kalmış olabilir
        (tekne geri gitti, referans yeniden planlandı, poz ışınlandı) → çapa
        güvenilmez. Referansın GERÇEK uçları (lo==0 / hi==n_ref) kenar
        sayılmaz: orada tam tarama da aynı sonucu verirdi; aksi hâlde rota
        başında ve hedefe varışta her adım boşuna fallback + WARN olurdu.
        """
        if lo > 0 and bool((idx_local == 0).any()):
            return True
        return hi < n_ref and bool((idx_local == hi - lo - 1).any())

    # ----- maliyet -----

    def _trajectory_cost(
        self, traj: np.ndarray, U: np.ndarray
    ) -> np.ndarray:
        """traj: (K, T+1, 6), U: (K, T, 2) → cost (K,)."""
        cfg = self.cfg
        xp = self.xp
        xs = traj[:, :, 0]
        ys = traj[:, :, 1]
        psis = traj[:, :, 2]
        K = traj.shape[0]
        cost = xp.zeros(K, dtype=self._dtype)

        # 1) Yörünge sapması + heading hata + terminal goal
        if self._ref_xy is not None and self._ref_psi is not None:
            ref = self._ref_xy
            psi_ref = self._ref_psi
            n_ref = ref.shape[0]

            # F-M.2: en yakın nokta araması tam referansta değil, çapa
            # etrafındaki dilimde. Kısıtlı küme üzerinde min alındığı için
            # d2_min yalnız BÜYÜYEBİLİR — yani pencere dışına savrulan
            # rollout'un takip maliyeti olduğundan yüksek çıkar; referansa
            # yakın (kararı belirleyen) rolloutlarda sonuç birebir aynıdır.
            lo, hi = (
                self._ref_window(n_ref) if cfg.ref_window_enabled else (0, n_ref)
            )
            idx_local, d2_min = self._nearest_ref(xs, ys, lo, hi)

            if self._window_edge_hit(idx_local, lo, hi, n_ref):
                # Çapa güvenilmez → bu adımda tam tarama, çapayı yeniden bul.
                self._ref_window_fallbacks += 1
                n = self._ref_window_fallbacks
                if n == 1 or n % 50 == 0:                # 20 Hz'te log seli olmasın
                    logging.getLogger(__name__).warning(
                        "MPPI referans penceresi kenarında eşleşme (çapa=%d, "
                        "pencere=[%d,%d), n_ref=%d) → bu adımda TAM tarama, "
                        "çapa yeniden bulundu (toplam %d kez). Sürekli "
                        "tekrarlıyorsa ref_window_size küçük veya referans "
                        "sık yeniden planlanıyor.",
                        self._ref_anchor_idx, lo, hi, n_ref, n,
                    )
                lo, hi = 0, n_ref
                idx_local, d2_min = self._nearest_ref(xs, ys, lo, hi)
                # F-F.21: tam tarama tensörü (yüzlerce MB) HEMEN geri verilsin;
                # yoksa cupy havuzu onu Jetson'ın paylaşılan RAM'inde tutar.
                self._bellek_havuzunu_serbest_birak()

            idx_min = idx_local + lo                     # (K, T+1) GLOBAL indeks
            # Sonraki adımın çapası = teknenin ŞU ANKİ (t=0) en yakın indeksi.
            # step() içinde tüm rolloutlar aynı x0'dan başlar → sütun sabittir;
            # medyan white-box çağrılarda da tek stabil değer verir.
            anchor_now = int(self._as_numpy(xp.median(idx_min[:, 0])))
            self._ref_anchor_pending = anchor_now
            cost += cfg.w_track * d2_min.sum(axis=1)

            psi_nearest = psi_ref[idx_min]               # (K, T+1)
            yaw_err = _wrap_angle(psis - psi_nearest, xp)
            cost += cfg.w_heading * (yaw_err ** 2).sum(axis=1)

            # Terminal (F-M.3): rolloutun son noktası hedefe yakın olsun.
            # ⚠ Hedef ASLA kayan pencerenin ucu DEĞİL — ya global rota sonu
            # (terminal_mode="global") ya da çapadan sabit yay uzaklığındaki
            # nokta ("lookahead"). Dilimin ucu alınırsa araç pencere sınırını
            # hedef sanar.
            goal = self._terminal_goal(anchor_now)
            dxg = xs[:, -1] - goal[0]
            dyg = ys[:, -1] - goal[1]
            cost += cfg.w_terminal * (dxg * dxg + dyg * dyg)

        # 2) Engel maliyeti — emniyet çemberi içinde quadratic barrier
        if self._obs_xy.shape[0] > 0:
            ox = self._obs_xy[:, 0]
            oy = self._obs_xy[:, 1]
            r = self._obs_r
            dxo = xs[:, :, None] - ox[None, None, :]
            dyo = ys[:, :, None] - oy[None, None, :]
            d_obs = xp.sqrt(dxo * dxo + dyo * dyo)
            penalty = xp.maximum(0.0, r[None, None, :] - d_obs) ** 2
            cost += cfg.w_obstacle * penalty.sum(axis=(1, 2))

        # 3) Sınır ihlali (binary, ağır)
        b = self.bounds
        out = (
            (xs < b.x_min) | (xs > b.x_max)
            | (ys < b.y_min) | (ys > b.y_max)
        )
        cost += cfg.w_boundary * out.sum(axis=1)

        # 4) Kontrol efor
        cost += cfg.w_control * (U ** 2).sum(axis=(1, 2))

        # 5) Parkur-3 kamikaze çekici — Gaussian, negatif maliyet katkısı.
        # Hedefe yakın yörünge → büyük negatif terim → engel/sınır maliyetlerini
        # ezerek aracı hedefe yöneltir. exp(-d²/2σ²) ∈ [0, 1] olduğundan
        # toplam katkı sınırlı: −w_kamikaze · (T+1).
        if cfg.kamikaze_mode and cfg.kamikaze_target is not None:
            tx, ty = cfg.kamikaze_target
            d2 = (xs - tx) ** 2 + (ys - ty) ** 2
            sigma2 = max(1e-6, cfg.kamikaze_radius ** 2)
            attractor = xp.exp(-d2 / (2.0 * sigma2))
            cost -= cfg.w_kamikaze * attractor.sum(axis=1)

        return cost

    # ----- adım -----

    def _sample_noise(self) -> np.ndarray:
        """(K, T, 2) kontrol gürültüsü — parite testleri sabit gürültü
        enjekte etmek için bunu monkeypatch'ler (plan Faz 0)."""
        # cupy.random.Generator'da .normal() yok (cupy 13.x); standard_normal
        # iki backend'de de var ve numpy'de normal(0, σ) ile bit-birebir.
        eps = self._rng.standard_normal(
            size=(self.cfg.K, self.cfg.T, 2)
        ) * self.cfg.sigma_u
        return self.xp.asarray(eps, dtype=self._dtype)

    def step(self, state: np.ndarray) -> np.ndarray:
        """Tek MPPI iterasyonu. state: (6,) host. Dönüş: (2,) host float64
        [T_l, T_r] (N) — cihaz dizisi çağırana sızmaz (sınır sözleşmesi)."""
        cfg = self.cfg
        xp = self.xp
        max_T = self.p.max_thrust
        state_xp = xp.asarray(state, dtype=self._dtype)

        # Gürültü → aday kontrol dizileri (kırpma sonrası etkin gürültüyü kullan)
        eps = self._sample_noise()
        V = xp.clip(self.U_nominal[None, :, :] + eps, -max_T, max_T)
        eps_eff = V - self.U_nominal[None, :, :]

        # Rollout & maliyet
        traj = self._rollout(state_xp, V)
        S = self._trajectory_cost(traj, V)
        # F-M.2: kayan pencere çapasını bir sonraki iterasyon için ilerlet.
        # Erken dönüş yollarından ÖNCE — çapa hiçbir adımda takılı kalmasın.
        self._ref_anchor_idx = self._ref_anchor_pending

        # Softmax ağırlık (numerik stabil)
        S_min = S.min()
        w = xp.exp(-(S - S_min) / cfg.lambda_)
        w_sum = w.sum()
        if not bool(xp.isfinite(w_sum)) or float(w_sum) < 1e-12:
            # Tüm rolloutlar feci — nominal'i sürdür
            self._last_traj = traj
            self._last_weights = xp.full(cfg.K, 1.0 / cfg.K, dtype=self._dtype)
            u0 = self.U_nominal[0].copy()
            self._apply_warmstart(self.U_nominal)
            return np.asarray(self._as_numpy(u0), dtype=float)
        w /= w_sum

        # Ağırlıklı güncelleme
        delta = (w[:, None, None] * eps_eff).sum(axis=0)        # (T, 2)
        U_new = xp.clip(self.U_nominal + delta, -max_T, max_T)
        u0 = U_new[0].copy()

        # Warm-start (cfg.warm_start_enabled): U_new'i bir adım kaydır + sıfır.
        # Devre dışı: cold-start (her step taze sıfır nominal) — debug/test için.
        self._apply_warmstart(U_new)

        # Görselleştirme snapshot'ı (cihazda kalır; property'de host'a çevrilir)
        self._last_traj = traj
        self._last_weights = w
        return np.asarray(self._as_numpy(u0), dtype=float)

    def _apply_warmstart(self, U_source: np.ndarray) -> None:
        """U_source[1:] → U_nominal[:-1], son adım 0; toggle ile cold-start."""
        if self.cfg.warm_start_enabled:
            self.U_nominal[:-1] = U_source[1:]
            self.U_nominal[-1] = 0.0
        else:
            self.U_nominal[:] = 0.0

    # ----- görselleştirme yardımcıları -----

    @property
    def last_trajectories(self) -> Optional[np.ndarray]:
        """Son rollout demeti (K, T+1, 6) — host numpy (istekte çevrilir)."""
        if self._last_traj is None:
            return None
        return self._as_numpy(self._last_traj)

    @property
    def last_weights(self) -> Optional[np.ndarray]:
        """Son softmax ağırlıkları (K,) — host numpy (istekte çevrilir)."""
        if self._last_weights is None:
            return None
        return self._as_numpy(self._last_weights)


# --------------------------------------------------------------------------- #
# KTR demosu
# --------------------------------------------------------------------------- #


def _build_scenario() -> Tuple[
    Bounds, List[CircleObstacle], Tuple[float, float], Tuple[float, float]
]:
    """RRT* demosuyla aynı sahne — iki demo karşılaştırılabilir."""
    bounds = Bounds(0.0, 50.0, 0.0, 50.0)
    obstacles = [
        CircleObstacle(15.0, 20.0, 3.0),
        CircleObstacle(25.0, 25.0, 4.0),
        CircleObstacle(35.0, 15.0, 3.0),
        CircleObstacle(20.0, 35.0, 3.0),
        CircleObstacle(35.0, 35.0, 3.5),
        CircleObstacle(10.0, 40.0, 2.5),
    ]
    start = (5.0, 5.0)
    goal = (45.0, 45.0)
    return bounds, obstacles, start, goal


def _draw_demo(
    bounds: Bounds,
    obstacles: List[CircleObstacle],
    ref_path: List[Tuple[float, float]],
    executed: np.ndarray,
    snapshot_traj: np.ndarray,
    snapshot_w: np.ndarray,
    snapshot_state_xy: Tuple[float, float],
    controls: np.ndarray,
    times: np.ndarray,
    cfg: MPPIConfig,
    out_path: Path,
) -> None:
    import matplotlib.pyplot as plt          # tembel: yalnız görselleştirme
    from matplotlib.patches import Circle

    fig, axes = plt.subplots(1, 2, figsize=(13.5, 6.2))
    ax1, ax2 = axes

    # --- Panel 1: yörünge + rollout bulutu ---
    for o in obstacles:
        ax1.add_patch(Circle((o.cx, o.cy), o.r,
                             color="tab:red", alpha=0.45, zorder=2))
        ax1.add_patch(Circle((o.cx, o.cy), o.r + cfg.obstacle_margin,
                             color="tab:red", fill=False, ls=":",
                             lw=0.8, alpha=0.5, zorder=2))

    # Snapshot anındaki K rolloutun bir alt kümesini çiz (görsel kirlilik az)
    K = snapshot_traj.shape[0]
    n_show = min(80, K)
    # En düşük maliyetli (en yüksek ağırlıklı) ilk n_show rolloutu seç
    idx = np.argsort(-snapshot_w)[:n_show]
    for i in idx:
        ax1.plot(snapshot_traj[i, :, 0], snapshot_traj[i, :, 1],
                 color="tab:blue", alpha=0.07, lw=0.6, zorder=1)

    # Referans, executed, snapshot konumu
    ref_arr = np.array(ref_path)
    ax1.plot(ref_arr[:, 0], ref_arr[:, 1],
             color="tab:orange", lw=1.4, ls="--",
             label="RRT* referansı", zorder=3)
    ax1.plot(executed[:, 0], executed[:, 1],
             color="tab:purple", lw=2.2,
             label="MPPI kapalı döngü", zorder=4)
    ax1.scatter(*snapshot_state_xy, c="black", s=60, marker="o",
                zorder=5, label="snapshot")

    ax1.scatter(ref_arr[0, 0], ref_arr[0, 1],
                c="tab:green", marker="X", s=130, zorder=5, label="Start")
    ax1.scatter(ref_arr[-1, 0], ref_arr[-1, 1],
                c="black", marker="*", s=180, zorder=5, label="Goal")

    ax1.set_xlim(bounds.x_min, bounds.x_max)
    ax1.set_ylim(bounds.y_min, bounds.y_max)
    ax1.set_aspect("equal")
    ax1.set_xlabel("x (m)")
    ax1.set_ylabel("y (m)")
    ax1.set_title(f"MPPI rollout bulutu (K={cfg.K}, T={cfg.T}) "
                  f"+ kapalı döngü iz")
    ax1.grid(True, alpha=0.3)
    ax1.legend(loc="lower right", fontsize=9)

    # --- Panel 2: kontrol geçmişi ---
    ax2.plot(times, controls[:, 0], color="tab:blue", lw=1.2,
             label="T_left (N)")
    ax2.plot(times, controls[:, 1], color="tab:red", lw=1.2,
             label="T_right (N)")
    ax2.axhline(0.0, color="gray", lw=0.5)
    ax2.set_xlabel("zaman (s)")
    ax2.set_ylabel("thruster kuvveti (N)")
    ax2.set_title("MPPI kontrol çıktısı")
    ax2.grid(True, alpha=0.3)
    ax2.legend(loc="best", fontsize=9)

    fig.suptitle(
        f"MPPI Lokal Planlayıcı — λ={cfg.lambda_}, σ_u={cfg.sigma_u} N, "
        f"dt={cfg.dt} s",
        fontsize=12,
    )
    fig.tight_layout()

    out_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out_path, dpi=150, bbox_inches="tight")
    plt.close(fig)


def _demo() -> None:
    bounds, obstacles, start, goal = _build_scenario()

    # 1) RRT* ile referans yörünge
    rrt = RRTStar(
        bounds, obstacles,
        RRTStarConfig(use_informed=True, seed=0, max_iter=1500),
    )
    ref_path = rrt.plan(start, goal)
    if ref_path is None:
        raise RuntimeError("RRT* çözüm bulamadı — demo iptal")
    print(f"[demo] RRT* yol: {len(ref_path)} waypoint, "
          f"cost = {rrt.best_cost:.2f} m")

    # 2) MPPI'yi kur
    dyn = CatamaranDynamics()
    # Demoda K'yı 1000'de tutuyoruz (CLAUDE.md spec); kapalı döngü ~15-25 s
    # ⚠ σ_u/λ BİLEREK VERİLMİYOR — ölçülen varsayılanlar (σ=0.364 N,
    # λ=1.0) kullanılsın. Demoda elle 5.0 yazılıydı; o değer 30 N/motor'luk
    # hayali tekneye aitti ve demo gerçek davranışı YANLIŞ gösteriyordu.
    cfg = MPPIConfig(K=1000, T=50, dt=0.05, seed=0)
    ctrl = MPPIController(dyn, bounds, obstacles, cfg)
    ctrl.set_reference(ref_path, spacing=0.5)

    # 3) Kapalı döngü simülasyonu — goal'e yakınsa veya max süre dolarsa kes
    state = np.zeros(6)
    state[0], state[1] = start
    # Başlangıç heading'i yörünge tangentine hizala (cold-start kararsızlığı az)
    state[2] = math.atan2(goal[1] - start[1], goal[0] - start[0])

    max_steps = int(40.0 / cfg.dt)
    executed = np.zeros((max_steps + 1, 6))
    executed[0] = state
    controls = np.zeros((max_steps, 2))
    snap_idx = 80                       # ~4 s noktasında snapshot
    snap_traj = None
    snap_w = None
    snap_xy: Tuple[float, float] = start

    t_start = time.perf_counter()
    n_done = 0
    for k in range(max_steps):
        u = ctrl.step(state)
        controls[k] = u
        state = dyn.step_rk4(state, u, cfg.dt)
        executed[k + 1] = state
        n_done = k + 1

        if k == snap_idx and ctrl.last_trajectories is not None:
            snap_traj = ctrl.last_trajectories.copy()
            snap_w = ctrl.last_weights.copy()
            snap_xy = (executed[k, 0], executed[k, 1])

        # Erken sonlanma: goal'e <1.5 m
        if math.hypot(state[0] - goal[0], state[1] - goal[1]) < 1.5:
            break
    t_total = time.perf_counter() - t_start

    executed = executed[: n_done + 1]
    controls = controls[: n_done]
    times = np.arange(n_done) * cfg.dt

    # Snapshot alınamadıysa (hızlı yakınsama) son rollouttan al
    if snap_traj is None:
        snap_traj = ctrl.last_trajectories
        snap_w = ctrl.last_weights
        snap_xy = (executed[-1, 0], executed[-1, 1])
    assert snap_traj is not None and snap_w is not None  # mypy

    final_err = math.hypot(executed[-1, 0] - goal[0],
                           executed[-1, 1] - goal[1])
    print(f"[demo] {n_done} adım koştu ({n_done * cfg.dt:.1f} s sim), "
          f"goal hata = {final_err:.2f} m")
    print(f"[demo] toplam süre = {t_total:.2f} s, "
          f"ortalama {1e3 * t_total / n_done:.1f} ms / iter "
          f"(K={cfg.K}, T={cfg.T}, CPU)")

    # 4) Görsel
    out_path = _REPO_ROOT / "docs" / "KTR" / "mppi_demo.png"
    _draw_demo(
        bounds, obstacles, ref_path, executed,
        snap_traj, snap_w, snap_xy,
        controls, times, cfg, out_path,
    )
    print(f"[demo] kaydedildi: {out_path.relative_to(_REPO_ROOT)}")


if __name__ == "__main__":
    _demo()
