# MPPI → Jetson GPU (CUDA) Port Planı

> Amaç: `control_rate_hz` 10 → 20-50 Hz (şartname md 3.3.1.1 "istemsiz
> hareket" kriterine doğrudan hizmet — daha sık kontrol = daha az zikzak).
> Bugünkü durum: NumPy CPU ~100 ms/iter (K=1000, T=50) → 10 Hz tavanı.
> Hedef donanım: **Jetson Orin Nano 8GB Super** (JetPack 6 = Ubuntu 22.04,
> Python 3.10, CUDA 12.x, Ampere iGPU 1024 CUDA çekirdeği, RAM paylaşımlı).
>
> Yazım tarihi: 2026-07-11. "Doğrulama" etiketli maddeler Jetson'da tek
> komutla test edilmeden kesin sayılmaz.

## 1. Teknoloji seçimi: CuPy (drop-in NumPy)

| Aday | Jetson durumu | Karar |
|---|---|---|
| **CuPy** (`cupy-cuda12x`) | PyPI'da aarch64 (manylinux2014) wheel; CuPy 13 dokümanı CUDA-12 aarch64 wheel'lerinin **SBSA + JetPack 6** desteğini listeler. JetPack 5 (CUDA 11.4) sancılıydı; JetPack 6 bunu çözdü. **Doğrulama: Jetson'da `pip install cupy-cuda12x` + `cupy.cuda.runtime.getDeviceCount()`** | ✅ **SEÇİLDİ** |
| PyTorch | PyPI'daki aarch64 CUDA wheel'leri SBSA (sunucu ARM) içindir, Jetson iGPU DEĞİL; Jetson için NVIDIA'nın jetson-ai-lab indeksi gerekir. Kurulum ~2 GB disk + import ~1 GB RAM — 8 GB paylaşımlı RAM'de ROS+algı yanında ağır. `pytorch_mppi` kütüphanesi hazır olsa da 3-DOF model için overkill | ❌ |
| Numba (`numba-cuda`) | CUDA hedefi çekirdek Numba'dan `numba-cuda` paketine taşındı; Jetson'da çalışır ama libNVVM yol ayarları tarihsel olarak nazlı — CuPy'den fazla hareketli parça | ❌ (B planı) |
| NVIDIA Warp | aarch64 wheel var, kernel-yazım modeli füzyona uygun; ama takıma yeni dil yüzeyi | ❌ (gelecek) |
| JAX | Jetson desteği fiilen yok | ❌ |
| Ham CUDA C++ | Her zaman çalışır (JetPack nvcc gemide) ama Layer-1 C++ katmanı hiç açılmadı; bakım maliyeti en yüksek | ❌ |

**CuPy gerekçesi:** mppi.py zaten %100 vektörize NumPy (`clip`, `argmin`,
`take_along_axis`, `exp` — hepsi CuPy'de birebir var). Kod değişikliği
minimum, Layer-0 CPU test edilebilirliği bozulmaz ([[jetson-yuk-kod-sadeligi]]
ilkesi). Kurulum tek pip paketi (~100-200 MB), torch'un ~10'da biri.

## 2. Ölçülen sıcak noktalar (mppi.py, 2026-07-11 okuma)

1. **`_rollout` (:246):** T=50 **ardışık** `_batch_rk4` → 50×4 `_batch_derivatives`
   → step başına ~2000 küçük NumPy çağrısı. CPU'da asıl maliyet Python dispatch.
   GPU'da karşılığı kernel-launch overhead'idir (~2000 launch × 5-10 µs ≈
   10-20 ms) — Faz B'de füzyonla düşürülür.
2. **`_trajectory_cost` (:259):** `d2 = (K, T+1, n_ref)` tensörü + `argmin` —
   denetimin "baskın maliyet" bulgusu. n_ref ≈ yol_uzunluğu/0.5 m. GPU'nun en
   sevdiği iş; doğrudan 10-50× hızlanır.
3. **dtype:** her şey float64. ⚠ **Orin'in Ampere iGPU'sunda fp64 oranı ~1:32
   — GPU'da float64 kullanmak kazancı yer.** Port float32 olmalı (thrust ~30 N,
   konum <200 m → float32 hassasiyeti bol bol yeter).

## 3. Yapı: `xp` backend deseni (tek kod, iki backend)

```python
# MPPIConfig'e:
backend: str = "auto"        # "numpy" | "cupy" | "auto" (varsa cupy)
gpu_dtype = float32          # yalnız cupy yolunda kullanılır

# MPPIController.__init__:
self.xp, self._dtype = _resolve_backend(cfg.backend)   # numpy/float64 ya da cupy/float32
# tüm np.* çağrıları self.xp.* olur; diziler self._dtype ile yaratılır

# step() çıkışı: xp.asnumpy benzeri sınır dönüşümü — yalnız (2,) vektör
u0 = cupy.asnumpy(u0) if self.xp is cupy else u0
```

Kurallar:
- **Çekirdek matematik TEK kopya** — `if cupy:` dallı ikinci implementasyon
  YOK (bearing dersinin genel hali: kopya = sessiz ıraksama).
- Sınır geçişleri (host↔device) yalnız `step()` giriş/çıkışında: `state (6,)`
  girer, `u (2,)` çıkar — bant genişliği ihmal edilebilir. Referans/engel
  güncellemeleri (`set_reference`/`set_obstacles`) zaten seyrek; diziler
  device'a bir kez kopyalanır.
- RNG: `cupy.random.default_rng` NumPy Generator API'sinin alt kümesini verir;
  `normal(size=(K,T,2))` birebir. ⚠ İki backend AYNI seed'le AYNI sayıları
  ÜRETMEZ — determinizm testleri backend-içi kalır (aşağıda).
- `viz` snapshot'ları (`_last_traj`) GPU'da kalır; yalnız istenince
  `asnumpy` (predicted_trajectory çağrısında).

## 4. Fazlı uygulama

- **Faz 0 (bu makinede, GPU'suz):** `xp` soyutlaması + float32 geçiş anahtarı
  refactor'u, NumPy backend'iyle **216 testin tamamı yeşil** kalarak. Parite
  testleri yazılır: aynı girdi + AYNI enjekte gürültüyle iki backend maliyeti
  `atol=1e-3` (float32) içinde eş. CuPy testleri `importorskip("cupy")` ile
  dürüst skip (F16.2 deseni).
  **✅ TAMAM (2026-07-11):** xp soyutlaması + parite testleri commit
  `9257fb4`; `scripts/bench_mppi.py` D3 ölçüm aracı (ort/min/maks ms +
  ilk→son yarı sürüklenme, §5 eşik raporu) sonraki commit. x86 doğrulama:
  numpy ort ~99 ms/iter (K=1000, T=50, 20 adım) — §2'deki "~100 ms" tespiti
  bağımsız teyit; `auto` GPU'suz makinede numpy'a düşüyor.
- **Faz A (Jetson, drop-in):** `pip install cupy-cuda12x` → `backend: auto`
  → D3 ölçümü (25W "super" güç modu + `jetson_clocks` AÇIK, aksi ölçüm
  yanıltır). Beklenti: maliyet tensörü ezilir, rollout launch-overhead'e
  takılır → step ~15-30 ms ≈ 30-60 Hz bandı. 20 Hz'i geçiyorsa
  `control_rate_hz` kademeli artırılır (20 → sahada doğrula → 50).
  **✅ ÖLÇÜLDÜ (2026-07-11, Jetson MAXN_SUPER; jetson_clocks HENÜZ kilitli
  değil — sudo bekliyor, sayılar onunla bir miktar iyileşebilir):**
  cupy 13.6.0 sorunsuz import (§6 risk 1 kapandı), CUDA context RAM ~50 MB
  (kriter c ✓), parite testi Jetson'da GEÇTİ (kriter b ✓). bench_mppi
  K=1000/T=50: **numpy ort 208.9 ms (~4.8 Hz) · cupy ort 301.9 ms (~3.3 Hz)
  — kriter a KALDI, iki backend de 20 Hz altında.** Ayrıştırma (§4 buluşma
  noktası): rollout numpy 34 ms / cupy 266 ms (launch-overhead, T=50 ardışık
  adım × küçük kerneller); maliyet numpy 162 ms / cupy 9 ms (18× — beklenti
  doğrulandı). ⚠ Jetson CPU x86'nın ~2× yavaşı (99→209 ms): mevcut
  `control_rate_hz: 10` CPU'da da TUTMAZ (~4.8 Hz tavan) → Faz B'ye kadar
  control_rate kararı askıda; video provasından önce ya Faz B biter ya
  control_rate ~4 Hz'e çekilir (ekip kararı).
- **Faz B (yalnız gerekirse):** RK4'ü tek kernel'e füzyon —
  `cupy.fuse` (dekoratörle otomatik) ya da `cupy.RawKernel` (50 adımlık
  rollout tek launch). Faz A 20 Hz'i veriyorsa B açılmaz (spekülatif kod
  yasağı). **🔴 TETİKLENDİ (2026-07-11):** Faz A ölçümü 20 Hz'i vermedi;
  darboğaz kesin olarak rollout launch-overhead'i (yukarıdaki ayrıştırma).
  Hedef: rollout tek launch → step ≈ rollout(~10-20?) + maliyet(9) ms
  → 20 Hz kriteri. Maliyet tarafı zaten GPU'da kazanıyor, dokunma.
  **✅ TAMAM (2026-07-11 aynı gün, Jetson'da):** `RawKernel` yolu seçildi
  (cupy.fuse slicing'li yapıda uygulanamaz): `rollout_rk4` kernel'i — K
  thread, her thread kendi yörüngesini register'larda T adım entegre eder,
  rollout = TEK launch. Fizik `_batch_derivatives` ile işlem-sırası düzeyinde
  aynı (sincosf, aynı bölmeler); dalga bozucusu batch yolundaki gibi bilerek
  yok. `MPPIConfig.fused_rollout=True` varsayılan (yalnız cupy yolunda
  etkili); derleme başarısızsa bir kez WARN + jenerik yola sessiz düşüş
  (saha kodu kernel yüzünden ölmez). TDD: 3 yeni test (fused≡generic 1e-4,
  fused≡numpy 1e-3, numpy yolunda bit-etkisiz).
  **📊 SONUÇ (MAXN_SUPER, jetson_clocks kilitsiz): step ort 9.0 ms
  (min 8.2 / maks 14.5) → tavan ~112 Hz — 20 Hz VE 50 Hz kriterleri GEÇTİ**
  (Faz A cupy 301.9 ms'den ~33×; numpy 208.9 ms'den ~23×). 600 adım sürekli
  koşuda sürüklenme −1.0% (termal throttle yok). Suite 249/2, kapalı-döngü
  testleri fused'la geçiyor (suite süresi 313→67 sn). control_rate kararı:
  artık 10 Hz RAHAT; 20 Hz mümkün — sahada F4.2 protokolüyle doğrulanacak.
- **İlk buluşma noktası:** ilk CUDA denemesi için `cupyx.profiler.benchmark`
  ile step ayrıştırması (rollout vs cost) — hangisi domine ediyorsa optimizasyon
  oraya.

## 5. Jetson kurulum/doğrulama listesi (D3 gününe)

```bash
# 1) Güç modu + saatler (ÖLÇÜMDEN ÖNCE):
sudo nvpmodel -m 0 && sudo jetson_clocks       # 25W super mod
# 2) CuPy:
python3 -m pip install --user cupy-cuda12x
python3 -c "import cupy; print(cupy.cuda.runtime.getDeviceCount(), cupy.__version__)"
# 3) RAM maliyeti (CUDA context Jetson'da ~300-600 MB — 8 GB bütçede ölç):
python3 -c "import cupy; cupy.zeros(1); import subprocess; subprocess.run(['free','-m'])"
# 4) MPPI benchmark (scripts/bench_mppi.py — Faz 0'da eklendi; sürüklenme
#    ölçümü için --steps 600, tegrastats ile birlikte oku):
python3 scripts/bench_mppi.py --backend numpy && python3 scripts/bench_mppi.py --backend cupy
```

Kabul kriterleri: (a) cupy step ortalaması < 50 ms (20 Hz) — hedef < 20 ms
(50 Hz); (b) parite testleri Jetson'da geçer; (c) `free -m` ile toplam ek RAM
< 1 GB; (d) 60 sn sürekli koşuda step süresi sürüklenmiyor (thermal throttle
gözlemi — `tegrastats`).

## 6. Riskler / bilinmeyenler

- 🔶 **cupy-cuda12x wheel'inin JetPack 6'da sorunsuz import'u** — en kritik
  doğrulama; başarısızsa B planı `numba-cuda`, C planı kaynaktan CuPy derleme
  (saatler ama tek seferlik).
- 🔶 CUDA context RAM'i + ROS + algı birlikte 8 GB'ta — ölçülmeden control_rate
  artırılmaz.
- 🔶 float32 geçişinin CPU yoluna etkisi: Faz 0'da CPU float64 KALIR (davranış
  birebir); float32 yalnız GPU yolunda. Parite testi ikisini köprüler.
- ⚪ Kernel-launch overhead'i Orin'de x86'dan biraz yüksek olabilir — Faz B
  tetiği budur.
