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

    # 🔴 F-F.2 (18.08.2026) — heading outlier reddi (Huber M-estimator).
    # SAHA OLAYI (17.08 akşam gölü): heading_sigma_psi=0.05 rad (≈2.9°) ile
    # saf Gauss prior olarak modellenen pusula ölçümü, tek bir hatalı AHRS
    # okumasında (manyetik girişim — motor/gövde metali, kalibrasyon
    # tamamlanmamış pusula) whitened hatası onlarca sigma'ya çıkıp KARE
    # hatayla cezalandırıldı → iSAM2 çözümü saniyeler içinde diverge etti
    # (x/y katlanarak büyüdü, 1e23 → 1e73). F-F.1 (§0.98a) makullük kapısı
    # bunu yayınlamadan YAKALADI (araç yanlış pozla sürmedi) ama sonucu
    # sessizlikti — planning_node MPPI'yi tüm gece durdurdu, poz bir daha
    # HİÇ gelmedi (görev bandı 783 mesaj/8+ saat). GPS'in Huber koruması
    # (yukarı) zaten var, heading'in aynı korumadan yoksun olması tutarsızdı.
    # gps_robust_enabled'ın birebir aynası: False → eski saf Gauss davranışı.
    heading_robust_enabled: bool = True
    heading_huber_k: float = 1.345

    # iSAM2 incremental ayarları
    relinearize_threshold: float = 0.01
    relinearize_skip: int = 1

    # 🌱 §1.56g (18.08.2026) — DÖNEMSEL YENİDEN ÇIPALAMA (graf sınırlama).
    #
    # SORUN: bu sınıf grafı HİÇ budamıyor — ne fixed-lag, ne marjinalizasyon,
    # ne pencere. 10 Hz girdiyle saatte ~36 000 faktör birikir ve `update()`
    # doğrusaldan kötü büyür. Çevrimdışı ölçüldü (sentetik 70 dk @10 Hz, bu
    # Jetson): 3,40 ms → 72,00 ms = **21,2×**, düzleşme YOK.
    #
    # ÇÖZÜM: periyodik olarak son ÇÖZÜLMÜŞ pozu çapa alıp grafı sıfırla.
    # Ölçülen A/B (aynı yer gerçeği, aynı ölçümler, 70 dk):
    #     yöntem                     ilk 10 dk   son 10 dk   sürüklenme   hata
    #     sınırsız (mevcut)            3,40 ms     72,00 ms     21,2×     0,100 m
    #     fixed-lag 30 s               0,74 ms      0,89 ms      1,2×     0,100 m
    #     çıpala 30 s (kovaryanslı)    0,38 ms      0,34 ms      0,90×    0,0998 m
    #     çıpala 30 s (SABİT σ)  ←     0,31 ms      0,31 ms      1,00×    0,0998 m
    # Doğruluk dördünde de AYNI ⇒ hesap bedava kazanılıyor.
    #
    # NEDEN BEDAVA: grafta döngü kapanışı YOK — yalnız odometri + mutlak GPS
    # prior'u + heading prior'u. Mutlak konum varken 30 sn önceki poz, şu anki
    # poz hakkında bilgi taşımıyor; atılan şey bilgi değil, maliyet.
    #
    # NEDEN MARJİNALİZASYON DEĞİL: `marginalizeLeaves` GTSAM issue #1101'in üç
    # belgelenmiş hatasını taşıyor (null pointer · `firstBlock() =` yerine `+=`
    # olmalı, tekil matris üretiyor · anahtar sıralaması varsayımı).
    # `IncrementalFixedLagSmoother` tam da onu çağırıyor. Yeniden çıpalama
    # `marginalizeLeaves`'i HİÇ çağırmaz ⇒ o hatalara sıfır maruziyet.
    #
    # DİKİŞ SIÇRAMASI YOK (ölçüldü): sıfırlamadan sonraki 5 adımın hatası genel
    # ortalamadan DAHA DÜŞÜK (−0,0059 m) — sıfırlama birikmiş doğrusallaştırma
    # bayatlığını atıyor.
    #
    # ⚠️ SINIR — ÇÖZÜLMEDİ, DONDURULDU: sabit σ, çapanın "kendimi bu kadar iyi
    # biliyorum" iddiasıdır. RTK kaybında GPS sigması 2,5 m'ye çıkar; o anda
    # 0,05 m'lik bir çapa aşırı güvenlidir ve o anki hatayı KİLİTLER. Çevrimdışı
    # A/B bunu sınamadı (sabit 0,05 m GPS ile koştu). `reanchor_sigma_xy` bu
    # yüzden ayrı ayarlanabilir bırakıldı; GPS kalitesine bağlamak ölçülmemiş
    # bir iştir ve ölçülmeden yapılmaz.
    #
    # ⚠️ GEÇMİŞ KESİLİR: çıpadan önceki anahtarlar graftan düşer. `all_poses()`
    # / `all_xy_psi()` yalnız çıpadan itibarenini döndürür (`anchor_key`).
    # Canlı yol etkilenmez — `fusion_node` yalnız `current_pose()` çağırır
    # (denetlendi, 18.08); geçmişi kullanan tek üretim kodu `synthetic_demo`.
    #
    # 0 = KAPALI = eski davranış BİREBİR (varsayılan). Dağıtımda açılır.
    reanchor_period_keys: int = 0
    # Çapa prior'unun sigmaları. None → prior_sigma_xy / prior_sigma_psi
    # (ölçülen kazanan tam olarak budur: [0,05 · 0,05 · 0,05]).
    reanchor_sigma_xy: Optional[float] = None
    reanchor_sigma_psi: Optional[float] = None

    # 🔁 KUYRUK TUTMA (kaptan sorusu, 18.08): *"hepsini sıfırlamasak, %80'ini
    # silsek — iSAM2 geçmiş veriyle optimize oluyor"*. Doğru sezgi; ölçüm ise
    # doğruluğun üç yöntemde de AYNI çıktığını söylüyor (yukarıdaki tablo),
    # çünkü grafta döngü kapanışı yok. Yine de kayan pencere ayrı bir
    # seçenek olarak duruyor ve varsayılanı ÖLÇÜM seçsin diye buraya kondu.
    #
    # 0  → saf çıpalama (graf tek poza iner)
    # K  → son K poz KORUNUR; çapa kuyruğun BAŞINA konur, aradaki bağlar
    #      tahminlerden yeniden kurulur (`p[i-1].between(p[i])`).
    # İkisi TEK kod yolu — 0, `k0 == son` özel durumuna kendiliğinden düşer.
    #
    # ⚠️ Kuyruk, tahminleri ölçüm gibi yeniden kullanır (hafif bilgi
    # tekrarı) — çıpalamanın kendisiyle aynı sınıf yaklaşım. `marginalizeLeaves`
    # ÇAĞRILMAZ, dolayısıyla GTSAM #1101'e maruziyet burada da SIFIR.
    reanchor_keep_keys: int = 0


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
        # heading_robust_enabled aynası: k<=0 → heading TAMAMEN yok sayılır,
        # yaml yazım hatası sessizce buraya düşmesin (bkz. GPS gerekçesi).
        if self.cfg.heading_robust_enabled and not self.cfg.heading_huber_k > 0.0:
            raise ValueError(
                f"heading_huber_k pozitif olmalı, geldi: {self.cfg.heading_huber_k}"
            )

        # 🌱 §1.56g — yeniden çıpalama kapıları. Sessizce yanlış ayara
        # düşmek, kapalı kalmaktan kötüdür: negatif periyot "kapalı" DEĞİL,
        # yazım hatasıdır ve öyle bildirilir.
        if self.cfg.reanchor_period_keys < 0:
            raise ValueError(
                "reanchor_period_keys negatif olamaz (0 = kapalı), geldi: "
                f"{self.cfg.reanchor_period_keys}"
            )
        if self.cfg.reanchor_keep_keys < 0:
            raise ValueError(
                "reanchor_keep_keys negatif olamaz, geldi: "
                f"{self.cfg.reanchor_keep_keys}"
            )
        # Kuyruk periyottan uzunsa çıpalama HİÇBİR ŞEY atmaz — graf yine
        # sınırsız büyür ve ayar "açık" göründüğü için bu sessizce olur.
        # Tam da §1.56g'nin önlemek istediği durum.
        if (
            self.cfg.reanchor_period_keys > 0
            and self.cfg.reanchor_keep_keys >= self.cfg.reanchor_period_keys
        ):
            raise ValueError(
                "reanchor_keep_keys periyottan KÜÇÜK olmalı, yoksa graf "
                "budanmaz (çıpalama etkisiz kalır): "
                f"keep={self.cfg.reanchor_keep_keys} >= "
                f"period={self.cfg.reanchor_period_keys}"
            )

        self._isam = gtsam.ISAM2(self._isam_params())

        # Çıpalama sayacı ve grafın en eski anahtarı. `_anchor_key` yalnız
        # istatistik değil: `all_poses()` bunun ALTINDAKİ anahtarları
        # sorgulamamalı — o anahtarlar artık grafta YOK.
        self._reanchor_count = 0
        self._anchor_key: int = 0
        self._keys_since_anchor: int = 0

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
        # Heading prior'u — GPS'in aynası (F-F.2): (x,y) kanalı uninformative,
        # yalnız psi ölçülür. heading_robust_enabled ise Huber ile sarılır.
        self._heading_noise = self._make_heading_noise(self.cfg.heading_sigma_psi)
        # Çapa prior'u — SABİT σ (ölçülen kazanan). None → prior sigmaları,
        # ki ölçülen yapılandırma tam olarak budur ([0,05 · 0,05 · 0,05]).
        _c_xy = (
            self.cfg.prior_sigma_xy
            if self.cfg.reanchor_sigma_xy is None
            else float(self.cfg.reanchor_sigma_xy)
        )
        _c_psi = (
            self.cfg.prior_sigma_psi
            if self.cfg.reanchor_sigma_psi is None
            else float(self.cfg.reanchor_sigma_psi)
        )
        if not (_c_xy > 0.0 and _c_psi > 0.0):
            raise ValueError(
                f"reanchor sigmaları pozitif olmalı, geldi: xy={_c_xy} psi={_c_psi}"
            )
        self._cipa_noise = gtsam.noiseModel.Diagonal.Sigmas(
            np.array([_c_xy, _c_xy, _c_psi])
        )

        # Fix kalitesine göre override edilen (add_gps sigma_xy) ve keyframe
        # periyoduna göre ölçeklenen (add_odometry sigma_scale) modeller —
        # sıcak yolda her faktörde yeniden inşa etmemek için önbelleklenir.
        self._gps_noise_cache: Dict[float, Any] = {}
        self._odom_noise_cache: Dict[float, Any] = {}
        self._heading_noise_cache: Dict[float, Any] = {}

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

    @property
    def reanchor_count(self) -> int:
        """Kaç kez planlı olarak yeniden çıpalandı (§1.56g).

        `recovery_count` ile KARIŞTIRILMAMALI: o arıza sayar, bu bakım sayar.
        Beklenen değer koşum süresi / çıpalama periyodu; sapması ayarın
        gerçekten uygulanmadığının işaretidir.
        """
        return self._reanchor_count

    @property
    def anchor_key(self) -> int:
        """Graftaki EN ESKİ anahtar. Bunun altındaki geçmiş atılmıştır."""
        return self._anchor_key

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

    def _make_heading_noise(self, sigma_psi: float) -> Any:
        """sigma_psi'den heading prior gürültü modeli üret — `_make_gps_noise`
        aynası: (x,y) kanalı uninformative, yalnız psi ölçülür.

        heading_robust_enabled=True → Huber M-estimator ile sarılmış model;
        False → taban diyagonal model (geri uyumlu, eski davranış).
        """
        if not sigma_psi > 0.0:
            raise ValueError(f"heading sigma_psi pozitif olmalı, geldi: {sigma_psi}")
        base = gtsam.noiseModel.Diagonal.Sigmas(
            np.array([_HEADING_FREE_SIGMA, _HEADING_FREE_SIGMA, sigma_psi])
        )
        if not self.cfg.heading_robust_enabled:
            return base
        return gtsam.noiseModel.Robust.Create(
            gtsam.noiseModel.mEstimator.Huber.Create(self.cfg.heading_huber_k),
            base,
        )

    def _heading_noise_for(self, sigma_psi: Optional[float]) -> Any:
        """sigma_psi=None → config varsayılanı; aksi halde önbellekli override."""
        if sigma_psi is None:
            return self._heading_noise
        key = round(float(sigma_psi), 6)
        model = self._heading_noise_cache.get(key)
        if model is None:
            if len(self._heading_noise_cache) >= _NOISE_CACHE_MAX:
                self._heading_noise_cache.clear()
            model = self._make_heading_noise(key)
            self._heading_noise_cache[key] = model
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
        # Çıpalama periyodu ANAHTAR sayar, saniye değil: bu sınıfın saati yok
        # ve keyframe throttle'ı kadansı değiştirebiliyor. Saniyeye çeviren
        # taraf `FusionPipeline` (keyframe_period_s'i o biliyor).
        self._keys_since_anchor += 1
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

        F-F.2 (18.08.2026): heading_robust_enabled=True (varsayılan) ise bu
        prior GPS'in Huber kernel'iyle aynı korumadan geçer. SAHA OLAYI
        (17.08 akşam): saf Gauss modelinde tek kötü AHRS okuması (manyetik
        girişim) whitened hatası onlarca sigma'ya çıkardı, kare-hata cezası
        iSAM2'yi saniyeler içinde diverge ettirdi (bkz. ISAM2SmootherConfig
        docstring'i). Huber, GPS'te olduğu gibi outlier'ın ağırlığını
        1/|e| ile söndürür — tek kötü okuma artık tüm çözümü kendine çekmez.
        """
        if key_index < 0 or key_index > self._latest_key:
            raise ValueError(f"Geçersiz key_index={key_index}")
        # x,y placeholder (0,0) — sigma=_HEADING_FREE_SIGMA olduğu için
        # whitened katkıları ~0, gerçek değerleri fark etmez.
        heading_pose = gtsam.Pose2(0.0, 0.0, psi)
        self._graph.add(
            gtsam.PriorFactorPose2(
                X(key_index), heading_pose, self._heading_noise_for(sigma_psi)
            )
        )

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
            # Çıpalama YALNIZ başarılı çözümden sonra. Tekilleşmiş graf zaten
            # `_tekillesmeden_kurtar` ile sıfırlanıyor; oraya ikinci bir
            # sıfırlama bindirmek çapayı çözülmemiş bir poza kurardı.
            if (
                self.cfg.reanchor_period_keys > 0
                and self._keys_since_anchor >= self.cfg.reanchor_period_keys
            ):
                self._yeniden_cipala()
        except RuntimeError as exc:
            if not _is_indeterminant(exc):
                raise
            if self._son_iyi is None:
                # Çözülmüş tek bir tahmin bile yok → çapa yok. Buradan
                # "kurtarmak" uydurma bir poz üretmek olurdu; sessiz yanlış
                # veri, gürültülü çökmeden daha tehlikelidir.
                raise
            self._tekillesmeden_kurtar()

    def _yeniden_cipala(self) -> None:
        """§1.56g — grafı sınırla: son çözülmüş pozu çapa alıp yeniden kur.

        `_tekillesmeden_kurtar`'ın AKRABASI ama aynısı DEĞİL, iki fark kasıtlı:
          · burada graf SAĞLIKLI — `_latest_key` geri sarılMAZ (kurtarmada
            sarılmak zorunda, çünkü orada çözüm hiç gerçekleşmemiş olabilir);
          · `recovery_count` ARTMAZ. O sayaç bir ARIZA göstergesidir; planlı
            bakımı oraya yazmak sahada "graf sürekli tekilleşiyor" yanılgısı
            üretirdi. Ayrı sayaç: `reanchor_count`.

        KUYRUK (`reanchor_keep_keys`): son K poz korunur. Çapa kuyruğun BAŞINA
        konur; aradaki bağlar tahminlerden yeniden kurulur
        (`p[i-1].between(p[i])`), böylece faktör defteri tutmaya gerek kalmaz.
        K=0'da `k0 == son` olur ve döngü hiç dönmez — saf çıpalama, özel durum
        yazmadan.

        ⚠ Kuyruktaki GPS/heading prior'ları TAŞINMAZ. Taşınmasına gerek yok:
        korunan pozlar zaten o ölçümleri soğurmuş ÇÖZÜMLERdir. Ama bu, tahmini
        ölçüm gibi yeniden kullanmak demektir (hafif bilgi tekrarı) — çıpanın
        kendisiyle aynı sınıf yaklaşım, ve bu yüzden kuyruk uzunluğu
        periyottan küçük tutulmak zorunda (`__init__` kapısı).
        """
        if self._son_iyi is None:                       # savunma; _flush garanti eder
            return
        son, _ = self._son_iyi

        keep = self.cfg.reanchor_keep_keys
        # Kuyruk grafın kendi başlangıcından geriye taşamaz.
        k0 = max(self._anchor_key, son - keep) if keep > 0 else son

        # Tek anahtar sorguları — maliyet N'den bağımsız (17.08 ölçümü).
        pozlar = {i: self._isam.calculateEstimatePose2(X(i)) for i in range(k0, son + 1)}

        self._isam = gtsam.ISAM2(self._isam_params())
        graph = gtsam.NonlinearFactorGraph()
        initial = gtsam.Values()

        graph.add(gtsam.PriorFactorPose2(X(k0), pozlar[k0], self._cipa_noise))
        initial.insert(X(k0), pozlar[k0])
        for i in range(k0 + 1, son + 1):
            graph.add(
                gtsam.BetweenFactorPose2(
                    X(i - 1), X(i), pozlar[i - 1].between(pozlar[i]), self._odom_noise
                )
            )
            initial.insert(X(i), pozlar[i])

        self._isam.update(graph, initial)
        self._graph = gtsam.NonlinearFactorGraph()
        self._initial = gtsam.Values()
        self._son_iyi = (son, self._isam.calculateEstimatePose2(X(son)))
        self._anchor_key = k0
        self._keys_since_anchor = 0
        self._reanchor_count += 1

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
        # Kurtarma da grafı tek poza indirir: `all_poses()` bunun altını
        # sorgularsa GTSAM patlar. Bu kusur çıpalamadan ÖNCE de vardı ama
        # kurtarma nadir olduğu için hiç tetiklenmemişti (§1.56g denetimi).
        self._anchor_key = son
        self._keys_since_anchor = 0
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
        # Çıpalama/kurtarma sonrası çapanın ALTINDAKİ anahtarlar graftan
        # düşmüştür. GTSAM'in kendi hatası bunu "Requested variable ... is
        # not in this VectorValues" diye bildirir — sahada okunması zor.
        # Açık mesaj, sessiz yanlış poz döndürmekten de kötü hatadan da iyidir.
        if key_index < self._anchor_key:
            raise ValueError(
                f"X({key_index}) çapanın altında (anchor_key="
                f"{self._anchor_key}); yeniden çıpalama o geçmişi attı"
            )
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
        # 🔴 `range(0, ...)` DEĞİL: çıpalama (ve tekilleşme kurtarması) grafın
        # başını `_anchor_key`e taşır; altındaki anahtarlar artık YOK ve
        # `est.atPose2` onlarda patlar. Kurtarma nadir olduğu için bu kusur
        # bugüne kadar tetiklenmedi; çıpalama onu RUTİN hâle getirirdi.
        return [est.atPose2(X(i)) for i in range(self._anchor_key, self._latest_key + 1)]

    def all_xy_psi(self) -> np.ndarray:
        """Smooth yörüngeyi (N, 3) numpy array olarak döndür: [x, y, psi]."""
        poses = self.all_poses()
        if not poses:
            return np.zeros((0, 3))
        return np.array([[p.x(), p.y(), p.theta()] for p in poses])
