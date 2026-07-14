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
from typing import List, Optional

import numpy as np
import gtsam
from gtsam.symbol_shorthand import X


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

    # iSAM2 incremental ayarları
    relinearize_threshold: float = 0.01
    relinearize_skip: int = 1


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

        params = gtsam.ISAM2Params()
        params.setRelinearizeThreshold(self.cfg.relinearize_threshold)
        # NOT: GTSAM 4.x Python binding'inde setter yok; attribute olarak atanır
        params.relinearizeSkip = self.cfg.relinearize_skip
        self._isam = gtsam.ISAM2(params)

        # Pending faktörler & başlangıç değerleri (her update'te boşaltılır)
        self._graph = gtsam.NonlinearFactorGraph()
        self._initial = gtsam.Values()

        # Son ISAM2 tahmini — add_odometry'nin compose'u için referans
        self._latest_estimate: Optional[gtsam.Values] = None
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
        self._gps_noise = gtsam.noiseModel.Diagonal.Sigmas(
            np.array([self.cfg.gps_sigma_xy, self.cfg.gps_sigma_xy, 1e6])
        )

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

    def add_odometry(self, delta: gtsam.Pose2) -> int:
        """
        Yeni Pose2 anahtarı oluştur ve önceki anahtara BetweenFactor bağla.
        delta: önceki frame'de ifade edilen relative pose (IMU pre-int çıktısı).
        Yeni anahtarın indeksini döndürür (sm.latest_key ile aynı).
        """
        if not self.is_initialized:
            raise RuntimeError("initialize() önce çağrılmalı")

        prev_key = X(self._latest_key)
        self._latest_key += 1
        new_key = X(self._latest_key)

        self._graph.add(
            gtsam.BetweenFactorPose2(prev_key, new_key, delta, self._odom_noise)
        )

        # İlk tahmin: önceki poz ⊕ delta. Önceki poz initial'da pending olabilir
        # (peş peşe add_odometry çağrıldıysa) veya çoktan ISAM2'de olabilir.
        if self._initial.exists(prev_key):
            prev_pose = self._initial.atPose2(prev_key)
        else:
            assert self._latest_estimate is not None  # initialize sonrası garantili
            prev_pose = self._latest_estimate.atPose2(prev_key)

        self._initial.insert(new_key, prev_pose.compose(delta))
        return self._latest_key

    def add_gps(self, key_index: int, x: float, y: float) -> None:
        """
        Belirli bir keye RTK GPS düzeltmesi ekle (Pose2 prior; heading serbest).
        key_index: hedef anahtarın indeksi (genelde latest_key).
        """
        if key_index < 0 or key_index > self._latest_key:
            raise ValueError(f"Geçersiz key_index={key_index}")
        # heading gerçekten ölçülmediği için 0.0 — sigma=1e6 kanalı serbest bırakır
        gps_pose = gtsam.Pose2(x, y, 0.0)
        self._graph.add(
            gtsam.PriorFactorPose2(X(key_index), gps_pose, self._gps_noise)
        )

    # ----- optimizer -----

    def update(self, n_extra_iters: int = 0) -> None:
        """Pending faktörleri ISAM2'ye gönder ve tahmini yenile."""
        self._flush(n_extra_iters)

    def _flush(self, n_extra_iters: int = 0) -> None:
        self._isam.update(self._graph, self._initial)
        for _ in range(n_extra_iters):
            self._isam.update()
        # GTSAM Python: graph.resize(0) yerine yeni instance — daha taşınabilir
        self._graph = gtsam.NonlinearFactorGraph()
        self._initial = gtsam.Values()
        self._latest_estimate = self._isam.calculateEstimate()

    # ----- queries -----

    def current_pose(self) -> gtsam.Pose2:
        """En son anahtarın smooth tahminini döndür."""
        if self._latest_estimate is None:
            raise RuntimeError("Henüz update edilmedi")
        return self._latest_estimate.atPose2(X(self._latest_key))

    def pose_at(self, key_index: int) -> gtsam.Pose2:
        if self._latest_estimate is None:
            raise RuntimeError("Henüz update edilmedi")
        return self._latest_estimate.atPose2(X(key_index))

    def all_poses(self) -> List[gtsam.Pose2]:
        """Tüm geçmiş Pose2 tahminlerini sırayla döndür."""
        if self._latest_estimate is None:
            return []
        return [
            self._latest_estimate.atPose2(X(i))
            for i in range(self._latest_key + 1)
        ]

    def all_xy_psi(self) -> np.ndarray:
        """Smooth yörüngeyi (N, 3) numpy array olarak döndür: [x, y, psi]."""
        poses = self.all_poses()
        if not poses:
            return np.zeros((0, 3))
        return np.array([[p.x(), p.y(), p.theta()] for p in poses])
