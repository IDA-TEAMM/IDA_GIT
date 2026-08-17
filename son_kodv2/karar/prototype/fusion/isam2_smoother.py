"""
Girdap İDA — iSAM2 Pose2 Smoother (GTSAM 4.3a0 Python binding)

NOT (sürüm kilidi): gtsam==4.2 wheel'ı NumPy 1.x'e karşı build edilmiştir;
NumPy 2.x ile çağrıldığında ABI çatışması nedeniyle segfault verir.
4.3a0 pre-release wheel'ı NumPy 2.x uyumludur — `pip install --pre gtsam`.

Amaç:
    GPS gürültüsü ve dalga sarsıntısından arındırılmış pürüzsüz poz tahmini.
    Yarışma şartnamesi Deniz Durumu-2 dayanıklılığı ister; saf GPS dump'ı
    yerine inkremental factor-graph smoothing daha temiz çıktı verir.

Faktörler:
    PriorFactorPose2     — başlangıç anchor'u (key=0)
    BetweenFactorPose2   — ardışık keyler arası odometri/IMU adımı
    PriorFactorPose2     — RTK GPS düzeltmesi (heading sigma=∞ ile (x,y)-only)

Robust GPS (M-estimator):
    RTK "fix" kaybı, çoklu-yol (multipath) yansıması ya da tek kötü uydu
    geometrisi tek bir GPS ölçümünü metrelerce kaydırabilir. Saf Gauss
    gürültü modelinde bu outlier kare-hata ile cezalandırıldığı için TÜM
    çözümü (geçmiş keyler dahil) kendine çeker. Huber M-estimator hatanın
    k·sigma eşiğinden sonra kare değil DOĞRUSAL büyümesini sağlar → outlier'ın
    ağırlığı 1/|e| ile söner, çözüm sapması sınırlı kalır.
    gps_robust_enabled=False eski (saf Gauss) davranışı birebir korur.

Tasarım notu:
    Bu Layer 0 prototipi gerçek IMU pre-integration yapmaz; çağıran taraftan
    Pose2 delta (odom_delta) kabul eder. Saha tarafına geçişte (Layer 2) bu
    sınıfın add_odometry'si CombinedImuFactor ile değiştirilecek.
    Pose2 / Pose3 kararı: ilk prototip Pose2 (yüzey aracı, roll/pitch küçük);
    KTR'de gerekirse Pose3 portu doğrudandır.

GTSAM API: 4.2+. ISAM2 inkremental — sadece etkilenen düğümler relinearize.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Dict, List, Optional, Tuple

import numpy as np
import gtsam
from gtsam.symbol_shorthand import X

# GPS Pose2-prior'unda heading kanalını uninformative bırakan sigma (≈∞).
# Robust kernel ile birlikte de güvenli: whitened hata bu kanalda ~0 kalır,
# dolayısıyla Huber ağırlığını sulandırmaz.
_HEADING_FREE_SIGMA = 1e6

# add_gps(sigma_xy=...) / add_odometry(sigma_scale=...) ile üretilen gürültü
# modelleri önbelleği. Değerler sabit bir tablodan (fix kalitesi) veya keyframe
# periyodundan gelir → birkaç ayrık değer; sınır yalnız patolojik girdide
# sınırsız büyümeyi engeller.
_NOISE_CACHE_MAX = 16


@dataclass
class ISAM2SmootherConfig:
    """Gürültü modelleri ve iSAM2 ayarları. Saha kalibrasyonunda tune edilir."""

    # Başlangıç prior'u — başlangıç pozu çok güvenli kabul edilir
    prior_sigma_xy: float = 0.05          # m
    prior_sigma_psi: float = 0.05         # rad

    # Odometri/IMU pre-integration adımı (saha testinde ölçülecek)
    odom_sigma_xy: float = 0.10           # m
    odom_sigma_psi: float = 0.02          # rad

    # RTK GPS — tipik fix doğruluğu ~2 cm; biraz pesimistik tutuyoruz
    gps_sigma_xy: float = 0.05            # m

    # Mutlak yön düzeltmesi (FC'nin AHRS'i: pusula+jiroskop+ivmeölçer füzyonu,
    # /mavros/imu/data orientation alanı). Kalibre edilmiş bir AHRS için
    # tipik yaw doğruluğu birkaç derece — 0.05 rad ≈ 2.9°, saha testinde
    # tune edilecek (diğer sigma'larla aynı statüde: ölçülmüş sabit değil,
    # başlangıç tahmini).
    heading_sigma_psi: float = 0.05       # rad

    # GPS outlier reddi (Huber M-estimator). False → eski saf Gauss davranışı.
    gps_robust_enabled: bool = True
    # Huber eşiği, whitened hata birimi (σ katı). 1.345 literatürdeki standart
    # seçim: Gauss gürültüde en küçük kareler verimliliğinin %95'ini korur,
    # bunun ötesindeki hataları doğrusal cezalandırır.
    gps_huber_k: float = 1.345

    # iSAM2 incremental ayarları
    relinearize_threshold: float = 0.01
    relinearize_skip: int = 1


#: GTSAM tekilleşmeyi ayrı bir istisna sınıfı olarak VERMİYOR (Python
#: binding'inde düz `RuntimeError`), o yüzden tek ayırt edici şey mesaj metni.
#: İki varyant da sahada görüldü: uzun açıklamalı hâli ve sınıf adı hâli.
_TEKILLESME_IZLERI = (
    "indeterminant linear system",
    "indeterminantlinearsystemexception",
)


def _is_indeterminant(exc: BaseException) -> bool:
    """İstisna GTSAM tekilleşmesi mi — kurtarılabilir tek hata sınıfı.

    Dar tutulması KASITLI: burası genişletilirse gerçek arızalar da
    "kurtarılır" ve araç bozuk tahminle sürmeye devam eder. Metin eşleşmesi
    kırılgan görünür ama alternatifi yok; GTSAM ayrı sınıf sunmuyor.
    """
    if not isinstance(exc, RuntimeError):
        return False
    mesaj = str(exc).lower()
    return any(iz in mesaj for iz in _TEKILLESME_IZLERI)


class ISAM2Smoother:
    """
    GTSAM iSAM2 sarmalayıcısı — Pose2 inkremental smoother.

    Tipik kullanım:
        sm = ISAM2Smoother()
        sm.initialize(gtsam.Pose2(0, 0, 0))
        for k in range(N):
            sm.add_odometry(gtsam.Pose2(dx, dy, dpsi))
            if got_gps:
                sm.add_gps(sm.latest_key, gx, gy)
            sm.update()
        traj = sm.all_xy_psi()        # (M, 3) — smooth yörünge
    """

    def __init__(self, cfg: Optional[ISAM2SmootherConfig] = None) -> None:
        self.cfg = cfg or ISAM2SmootherConfig()
        self._recovery_count = 0

        # k<=0'da Huber ağırlığı (k/|e|) her ölçüm için 0 olur → GPS TAMAMEN
        # yok sayılır ve araç yalnız IMU ölü-hesabıyla seyreder. yaml'daki bir
        # yazım hatası sessizce buraya düşmesin.
        if self.cfg.gps_robust_enabled and not self.cfg.gps_huber_k > 0.0:
            raise ValueError(
                f"gps_huber_k pozitif olmalı, geldi: {self.cfg.gps_huber_k}"
            )

        self._isam = gtsam.ISAM2(self._isam_params())

        # Pending faktörler & başlangıç değerleri (her update'te boşaltılır)
        self._graph = gtsam.NonlinearFactorGraph()
        self._initial = gtsam.Values()

        # 🔴 17.08.2026 — TAM `calculateEstimate()` SICAK YOLDAN KALDIRILDI.
        # GTSAM dokümanı: "If only a single variable is needed, it is faster
        # to call calculateEstimate(const KEY&)". Ölçüldü (bu Jetson):
        #     N=250  → tam 0,11 ms · tek-anahtar 0,098 ms
        #     N=6000 → tam 3,87 ms · tek-anahtar 0,008 ms   ⇒ tek-anahtar DÜZ
        # Tam sorgu N ile 36× büyüyor, tek-anahtar N'den BAĞIMSIZ.
        #
        # ⚠️ Ama `_latest_estimate` yalnız hız için tutulmuyordu: tekilleşme
        # kurtarması onu "son ÇÖZÜLMÜŞ hâl" ÇAPASI olarak kullanıyor. Graf
        # tekilleştiğinde ISAM2'ye canlı sorgu atmak yeniden patlar — 11.08'de
        # fusion_node'u öldüren zincir buydu. O yüzden tam anlık görüntü
        # yerine, çözülmüş son (anahtar, poz) İKİLİSİ saklanıyor: kurtarma
        # için yeterli, ve geriye doğru tarama gerektirmediği için daha kesin.
        self._son_iyi: Optional[Tuple[int, gtsam.Pose2]] = None
        self._latest_key: int = -1

        # Önceden hesaplanmış gürültü modelleri (her faktör için yeniden
        # üretmek gereksiz alokasyon)
        self._prior_noise = gtsam.noiseModel.Diagonal.Sigmas(
            np.array(
                [
                    self.cfg.prior_sigma_xy,
                    self.cfg.prior_sigma_xy,
                    self.cfg.prior_sigma_psi,
                ]
            )
        )
        self._odom_noise = gtsam.noiseModel.Diagonal.Sigmas(
            np.array(
                [
                    self.cfg.odom_sigma_xy,
                    self.cfg.odom_sigma_xy,
                    self.cfg.odom_sigma_psi,
                ]
            )
        )
        # GPS Pose2-prior olarak modellenir; heading kanalı uninformative
        # (sigma=1e6 ≈ ∞) bırakılır ki sadece (x,y) ölçümü etkili olsun.
        # gps_robust_enabled ise bu taban model Huber kernel'i ile sarılır.
        self._gps_noise = self._make_gps_noise(self.cfg.gps_sigma_xy)

        # Fix kalitesine göre override edilen (add_gps sigma_xy) ve keyframe
        # periyoduna göre ölçeklenen (add_odometry sigma_scale) modeller —
        # sıcak yolda her faktörde yeniden inşa etmemek için önbelleklenir.
        self._gps_noise_cache: Dict[float, Any] = {}
        self._odom_noise_cache: Dict[float, Any] = {}

    # ----- gürültü modeli fabrikaları -----

    def _isam_params(self) -> Any:
        """ISAM2Params — hem kuruluşta hem tekilleşme kurtarmasında kullanılır.

        Ayrı fonksiyon olmasının sebebi: kurtarma yeni bir ISAM2 kurar ve
        parametreler AYNI olmak zorunda. Kopyalanmış iki blok, birinde yapılan
        ayarın diğerine geçmemesiyle sonuçlanırdı.
        """
        params = gtsam.ISAM2Params()
        params.setRelinearizeThreshold(self.cfg.relinearize_threshold)
        # NOT: GTSAM 4.x Python binding'inde setter yok; attribute olarak atanır
        params.relinearizeSkip = self.cfg.relinearize_skip
        return params

    @property
    def recovery_count(self) -> int:
        """Kaç kez tekilleşmeden kurtarıldı (0 = hiç yaşanmadı).

        Saha teşhisi: sıfırdan büyükse graf düzenli olarak çözülemez hâle
        geliyor demektir; sebebi genelde yetersiz kısıt (uzun süre GPS yok)
        ya da çelişkili ölçümdür. Kurtarma aracı KURTARIR ama kök nedeni
        gizlemesin diye sayaç dışarı açık.
        """
        return self._recovery_count

    def _make_gps_noise(self, sigma_xy: float) -> Any:
        """(x, y) sigma'sından GPS prior gürültü modeli üret.

        gps_robust_enabled=True → Huber M-estimator ile sarılmış model;
        False → taban diyagonal model (geri uyumlu, eski davranış).
        """
        if not sigma_xy > 0.0:
            raise ValueError(f"gps sigma_xy pozitif olmalı, geldi: {sigma_xy}")
        base = gtsam.noiseModel.Diagonal.Sigmas(
            np.array([sigma_xy, sigma_xy, _HEADING_FREE_SIGMA])
        )
        if not self.cfg.gps_robust_enabled:
            return base
        return gtsam.noiseModel.Robust.Create(
            gtsam.noiseModel.mEstimator.Huber.Create(self.cfg.gps_huber_k),
            base,
        )

    def _gps_noise_for(self, sigma_xy: Optional[float]) -> Any:
        """sigma_xy=None → config varsayılanı; aksi halde önbellekli override."""
        if sigma_xy is None:
            return self._gps_noise
        key = round(float(sigma_xy), 6)
        model = self._gps_noise_cache.get(key)
        if model is None:
            if len(self._gps_noise_cache) >= _NOISE_CACHE_MAX:
                self._gps_noise_cache.clear()
            model = self._make_gps_noise(key)
            self._gps_noise_cache[key] = model
        return model

    def _odom_noise_for(self, sigma_scale: float) -> Any:
        """Keyframe periyoduna göre ölçeklenmiş odometri gürültü modeli.

        Odometri hatası rastgele yürüyüş: σ ∝ √Δt. Keyframe throttle'ı adım
        süresini uzattığında ölçek verilmezse zincir aynı sürede DAHA GÜVENLİ
        görünür (aynı σ, daha az faktör) ve GPS'i haksız yere bastırır.

        Ölçek 4 haneye yuvarlanır (önbellek anahtarı sınırlı kalsın); σ'ya
        etkisi ~1e-5 bağıl — gürültü modeli için tamamen ihmal edilebilir.
        """
        if not sigma_scale > 0.0:
            raise ValueError(f"sigma_scale pozitif olmalı, geldi: {sigma_scale}")
        key = round(float(sigma_scale), 4)
        if key == 1.0:
            return self._odom_noise
        model = self._odom_noise_cache.get(key)
        if model is None:
            if len(self._odom_noise_cache) >= _NOISE_CACHE_MAX:
                self._odom_noise_cache.clear()
            model = gtsam.noiseModel.Diagonal.Sigmas(
                np.array(
                    [
                        self.cfg.odom_sigma_xy * key,
                        self.cfg.odom_sigma_xy * key,
                        self.cfg.odom_sigma_psi * key,
                    ]
                )
            )
            self._odom_noise_cache[key] = model
        return model

    # ----- public properties -----

    @property
    def latest_key(self) -> int:
        """En son eklenen Pose2 anahtarının indeksi (X(i) içindeki i)."""
        return self._latest_key

    @property
    def is_initialized(self) -> bool:
        return self._latest_key >= 0

    # ----- graph mutators -----

    def initialize(self, pose0: gtsam.Pose2) -> None:
        """Faktör grafiğine X(0) anchor'unu ekle ve ilk update'i çalıştır."""
        if self.is_initialized:
            raise RuntimeError("Smoother zaten initialize edilmiş")

        key0 = X(0)
        self._graph.add(gtsam.PriorFactorPose2(key0, pose0, self._prior_noise))
        self._initial.insert(key0, pose0)
        self._latest_key = 0
        self._flush()

    def add_odometry(self, delta: gtsam.Pose2, sigma_scale: float = 1.0) -> int:
        """
        Yeni Pose2 anahtarı oluştur ve önceki anahtara BetweenFactor bağla.
        delta: önceki frame'de ifade edilen relative pose (IMU pre-int çıktısı).
        sigma_scale: odometri sigma'sı bu katsayıyla çarpılır — nominalden uzun
            (ya da kısa) bir keyframe aralığı biriktirildiğinde √Δt ölçeklemesi
            için. 1.0 → config sigma'ları (varsayılan, eski davranış).
        Yeni anahtarın indeksini döndürür (sm.latest_key ile aynı).
        """
        if not self.is_initialized:
            raise RuntimeError("initialize() önce çağrılmalı")

        prev_key = X(self._latest_key)
        self._latest_key += 1
        new_key = X(self._latest_key)

        self._graph.add(
            gtsam.BetweenFactorPose2(
                prev_key, new_key, delta, self._odom_noise_for(sigma_scale)
            )
        )

        # İlk tahmin: önceki poz ⊕ delta. Önceki poz initial'da pending olabilir
        # (peş peşe add_odometry çağrıldıysa) veya çoktan ISAM2'de olabilir.
        if self._initial.exists(prev_key):
            prev_pose = self._initial.atPose2(prev_key)
        else:
            assert self._son_iyi is not None  # initialize sonrası garantili
            prev_pose = self._isam.calculateEstimatePose2(prev_key)

        self._initial.insert(new_key, prev_pose.compose(delta))
        return self._latest_key

    def add_gps(
        self,
        key_index: int,
        x: float,
        y: float,
        sigma_xy: Optional[float] = None,
    ) -> None:
        """
        Belirli bir keye RTK GPS düzeltmesi ekle (Pose2 prior; heading serbest).
        key_index: hedef anahtarın indeksi (genelde latest_key).
        sigma_xy: bu ÖLÇÜME özel (x, y) sigma'sı — fix kalitesi (RTK/SBAS/tek
            nokta) ölçümden ölçüme değiştiği için çağıran taraf override eder.
            None → config gps_sigma_xy.
        """
        if key_index < 0 or key_index > self._latest_key:
            raise ValueError(f"Geçersiz key_index={key_index}")
        # heading gerçekten ölçülmediği için 0.0 — sigma=1e6 kanalı serbest bırakır
        gps_pose = gtsam.Pose2(x, y, 0.0)
        self._graph.add(
            gtsam.PriorFactorPose2(
                X(key_index), gps_pose, self._gps_noise_for(sigma_xy)
            )
        )

    def add_heading(
        self,
        key_index: int,
        psi: float,
        sigma_psi: Optional[float] = None,
    ) -> None:
        """
        Mutlak yön (heading) düzeltmesi ekle — FC'nin AHRS'i (pusula+jiroskop+
        ivmeölçer füzyonu, /mavros/imu/data orientation) kaynaklı.

        GPS prior'unun aynasıdır: orada (x,y) ölçülür heading serbest
        bırakılır (_HEADING_FREE_SIGMA); burada TERSİ — yalnız psi ölçülür,
        (x,y) kanalı _HEADING_FREE_SIGMA ile serbest bırakılır (whitened
        katkısı ~0, x,y için gerçek değer önemsiz).

        Neden gerekli: add_odometry yalnız jiroskop yaw rate'ini entegre
        eder — hiçbir mutlak referansı yoktur, sınırsız kayabilir (20 dk'lık
        görevde jiroskop bias'ı birikir). GPS prior'u yalnızca (x,y)'yi
        düzeltir, heading kanalı bilerek serbesttir. Bu metod olmadan
        smoother'ın psi çıktısı zamanla FC'nin gerçek AHRS'inden ayrışabilir.
        """
        if key_index < 0 or key_index > self._latest_key:
            raise ValueError(f"Geçersiz key_index={key_index}")
        sp = sigma_psi if sigma_psi is not None else self.cfg.heading_sigma_psi
        if not sp > 0.0:
            raise ValueError(f"heading sigma_psi pozitif olmalı, geldi: {sp}")
        # x,y placeholder (0,0) — sigma=_HEADING_FREE_SIGMA olduğu için
        # whitened katkıları ~0, gerçek değerleri fark etmez.
        heading_pose = gtsam.Pose2(0.0, 0.0, psi)
        noise = gtsam.noiseModel.Diagonal.Sigmas(
            np.array([_HEADING_FREE_SIGMA, _HEADING_FREE_SIGMA, sp])
        )
        self._graph.add(gtsam.PriorFactorPose2(X(key_index), heading_pose, noise))

    # ----- optimizer -----

    def update(self, n_extra_iters: int = 0) -> None:
        """Pending faktörleri ISAM2'ye gönder ve tahmini yenile."""
        self._flush(n_extra_iters)

    def _flush(self, n_extra_iters: int = 0) -> None:
        """Pending faktörleri ISAM2'ye gönder; tekilleşmede KURTAR, ölme.

        🔴 SAHA OLAYI (11.08.2026, Jetson): `calculateEstimate()` gerçek GPS
        akışında `Indeterminant linear system ... (Symbol: x1569)` fırlattı.
        İstisna `rclpy.spin`'e kadar çıktı, **fusion_node öldü ve geri
        gelmedi** → poz yayını kesildi → planning_node F-P.1 ile MPPI'yi
        durdurdu → araç sessizce sürmez oldu. Zincirin tamamı tek bir
        yakalanmamış istisnadan.

        Tekilleşme, grafın o anda çözülemez olması demektir (yetersiz kısıt,
        çelişkili ölçüm). Kalıcı bir bozulma DEĞİL — son iyi pozu çapa alıp
        yeni bir ISAM2 kurmak akışı sürdürür.

        ⚠ Yalnız tekilleşme kurtarılır. Başka bir `RuntimeError` yutulursa
        gerçek arıza gizlenir ve bu, öldüren istisnadan daha kötüdür — araç
        bozuk bir tahminle sürmeye DEVAM eder.
        """
        try:
            self._isam.update(self._graph, self._initial)
            for _ in range(n_extra_iters):
                self._isam.update()
            # GTSAM Python: graph.resize(0) yerine yeni instance — taşınabilir
            self._graph = gtsam.NonlinearFactorGraph()
            self._initial = gtsam.Values()
            self._son_iyi = (
                self._latest_key,
                self._isam.calculateEstimatePose2(X(self._latest_key)),
            )
        except RuntimeError as exc:
            if not _is_indeterminant(exc):
                raise
            if self._son_iyi is None:
                # Çözülmüş tek bir tahmin bile yok → çapa yok. Buradan
                # "kurtarmak" uydurma bir poz üretmek olurdu; sessiz yanlış
                # veri, gürültülü çökmeden daha tehlikelidir.
                raise
            self._tekillesmeden_kurtar()

    def _tekillesmeden_kurtar(self) -> None:
        """Son ÇÖZÜLMÜŞ pozu çapa alıp grafı yeniden kur.

        `_latest_key` geri sarılmak ZORUNDA: `add_odometry` anahtarı çözüm
        gerçekleşmeden önce ilerletmiş olabilir. Geri sarılmazsa bir sonraki
        `add_odometry` var olmayan bir `prev_key`'e compose etmeye çalışır ve
        node İKİNCİ kez ölür — yani kurtarma, kurtardığı arızayı bir adım
        öteler.
        """
        if self._son_iyi is None:
            raise RuntimeError(
                "tekillesme kurtarilamadi: cozulmus hicbir anahtar yok"
            )
        # Eskiden burada `_latest_estimate` içinde GERİYE DOĞRU taranıyordu
        # ("hangi anahtar çözülmüş?"). Artık çözülmüş son anahtar ZATEN
        # kayıtlı — tarama gereksiz ve saklanan ikili daha kesin: tam olarak
        # başarıyla çözülen anahtarı verir, "Values'ta var" olanı değil.
        son, capa = self._son_iyi
        self._isam = gtsam.ISAM2(self._isam_params())
        self._graph = gtsam.NonlinearFactorGraph()
        self._initial = gtsam.Values()
        self._latest_key = son                      # GERİ SAR

        self._graph.add(gtsam.PriorFactorPose2(X(son), capa, self._prior_noise))
        self._initial.insert(X(son), capa)
        self._isam.update(self._graph, self._initial)
        self._graph = gtsam.NonlinearFactorGraph()
        self._initial = gtsam.Values()
        self._son_iyi = (son, self._isam.calculateEstimatePose2(X(son)))
        self._recovery_count += 1

    # ----- queries -----

    def current_pose(self) -> gtsam.Pose2:
        """En son anahtarın smooth tahminini döndür — TEK ANAHTAR sorgusu.

        Maliyeti N'den bağımsız (ölçüldü: 6000 key'de 0,008 ms).
        """
        if self._son_iyi is None:
            raise RuntimeError("Henüz update edilmedi")
        return self._isam.calculateEstimatePose2(X(self._latest_key))

    def pose_at(self, key_index: int) -> gtsam.Pose2:
        if self._son_iyi is None:
            raise RuntimeError("Henüz update edilmedi")
        return self._isam.calculateEstimatePose2(X(key_index))

    def all_poses(self) -> List[gtsam.Pose2]:
        """Tüm geçmiş Pose2 tahminlerini sırayla döndür.

        ⚠️ SOĞUK YOL: burada tam `calculateEstimate()` BİLEREK kullanılıyor —
        zaten bütün pozlar isteniyor. Sıcak yolda (`current_pose`) çağrılmaz;
        çağrılırsa 17.08'de kaldırılan O(N) maliyeti geri gelir.
        """
        if self._son_iyi is None:
            return []
        est = self._isam.calculateEstimate()
        return [est.atPose2(X(i)) for i in range(self._latest_key + 1)]

    def all_xy_psi(self) -> np.ndarray:
        """Smooth yörüngeyi (N, 3) numpy array olarak döndür: [x, y, psi]."""
        poses = self.all_poses()
        if not poses:
            return np.zeros((0, 3))
        return np.array([[p.x(), p.y(), p.theta()] for p in poses])
