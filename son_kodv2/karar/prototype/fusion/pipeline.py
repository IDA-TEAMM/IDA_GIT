"""
Girdap İDA — iSAM2 Sensör Füzyonu Boru Hattı (ROS-bağımsız)

Layer 2 fusion_node ve Layer 0 birim testleri ortak bu sınıfı kullanır.
ROS 2 mesaj tipleri yerine düz skaler değer alır; böylece pytest rclpy
olmadan koşar (gtsam içeren .venv yeterli).

Akış:
    1) on_velocity(vx, vy)         — /mavros/local_position/velocity_body
    2) on_imu(t, omega_z)          — /mavros/imu/data, gyro yaw rate
       Her IMU çağrısı odom_period_s'i geçtiyse Pose2 delta üretir
       (vx·dt, vy·dt, ωz·dt) ve add_odometry'ye gönderir.
    3) on_gps(lat, lon)            — /mavros/global_position/global
       İlk fix origin olarak alınır; sonraki fix'ler ENU'ya
       eşit-dikdörtgensel projeksiyonla çevrilip add_gps prior'u olur.

Tasarım kararları:
    - IMU pre-integration ham accel'den değil, mavros'un EKF-temelli
      velocity_body çıktısından yapılır. Gerçek sahada bias-düzeltilmiş
      hız bu topic'te zaten mevcut; ham accel integrasyonunun drift'ini
      bypass etmenin temiz yolu bu.
    - Yaw rate IMU gyrosundan (omega_z); mavros velocity_body bazen yaw
      hızını içermez, bu yüzden ayrı kanal.
    - GPS prior kabul edilmeden önce bekleyen IMU integrasyonu flush
      edilir (latest_key güncel olsun).
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Optional, Tuple

import gtsam
import numpy as np

from prototype.fusion.isam2_smoother import ISAM2Smoother, ISAM2SmootherConfig

# WGS-84 yarı-büyük yarıçapı (ENU projeksiyonu için yeterli yaklaşım)
_EARTH_R = 6378137.0


@dataclass
class FusionPipelineConfig:
    """Boru hattı ayarları. ROS 2 parametre arayüzünden aynı isimle gelir."""

    odom_period_s: float = 0.1            # IMU step → smoother flush periyodu (10 Hz)
    gps_sigma_xy: float = 0.30            # m (mock için RTK olmayan değer; saha 0.05)
    odom_sigma_xy: float = 0.05           # m, vel·dt ölçek gürültüsü
    odom_sigma_psi: float = 0.01          # rad


class FusionPipeline:
    """
    iSAM2 inkremental smoother sarmalayıcısı + IMU/GPS pre-integration.

    Tipik kullanım:
        fp = FusionPipeline()
        fp.on_velocity(vx, vy)
        fp.on_imu(t, omega_z)        # her IMU mesajında
        fp.on_gps(lat, lon)          # GPS geldiğinde
        x, y, psi = fp.current_pose()
    """

    def __init__(self, cfg: Optional[FusionPipelineConfig] = None) -> None:
        self.cfg = cfg or FusionPipelineConfig()
        self._sm = ISAM2Smoother(
            ISAM2SmootherConfig(
                gps_sigma_xy=self.cfg.gps_sigma_xy,
                odom_sigma_xy=self.cfg.odom_sigma_xy,
                odom_sigma_psi=self.cfg.odom_sigma_psi,
            )
        )
        self._sm.initialize(gtsam.Pose2(0.0, 0.0, 0.0))

        # Pre-integration akümülatörleri
        self._vx_body: float = 0.0
        self._vy_body: float = 0.0
        self._wz: float = 0.0
        self._last_imu_t: Optional[float] = None
        self._t_since_flush: float = 0.0

        # GPS origin (ilk fix)
        self._lat0: Optional[float] = None
        self._lon0: Optional[float] = None
        self._cos_lat0: float = 1.0

    # ----- callback API (ROS 2 mesaj alanlarıyla 1:1 eşleşir) -----

    def on_velocity(self, vx_body: float, vy_body: float) -> None:
        """Body-frame hız akümülatörünü güncelle (TwistStamped.linear)."""
        self._vx_body = vx_body
        self._vy_body = vy_body

    def on_imu(self, t: float, omega_z: float) -> bool:
        """
        IMU mesajı: yaw rate'i güncelle, dt biriktir, periyot dolduğunda
        smoother'a Pose2 delta gönder.
        Dönüş: True ise smoother'a yeni delta yazıldı.
        """
        self._wz = omega_z
        if self._last_imu_t is None:
            self._last_imu_t = t
            return False

        dt = t - self._last_imu_t
        self._last_imu_t = t
        # Saçma dt'leri at (zaman geri sıçraması veya uzun gap)
        if dt <= 0.0 or dt > 0.5:
            return False

        self._t_since_flush += dt
        if self._t_since_flush < self.cfg.odom_period_s:
            return False

        return self._flush()

    def on_gps(self, lat: float, lon: float) -> None:
        """GPS fix: ENU'ya çevir, latest_key'e prior ekle."""
        if self._lat0 is None:
            # İlk fix → origin. Smoother başlangıçtan beri (0,0)'da; origin
            # buraya pinlenir. add_gps eklemeden update yapma; X(0) zaten
            # PriorFactor anchor'una sahip.
            self._lat0 = lat
            self._lon0 = lon
            self._cos_lat0 = math.cos(math.radians(lat))
            return

        # Bekleyen IMU integrasyonu varsa önce flush et — latest_key güncel olsun
        if self._t_since_flush > 1e-6:
            self._flush(force=True)

        x, y = self._latlon_to_enu(lat, lon)
        self._sm.add_gps(self._sm.latest_key, x, y)
        self._sm.update()

    # ----- iç yardımcılar -----

    def _flush(self, force: bool = False) -> bool:
        """Birikmiş hız×dt + yaw rate×dt → Pose2 delta → smoother."""
        period = self._t_since_flush
        if period <= 0.0:
            return False
        if not force and period < self.cfg.odom_period_s:
            return False

        delta = gtsam.Pose2(
            self._vx_body * period,
            self._vy_body * period,
            self._wz * period,
        )
        self._sm.add_odometry(delta)
        self._sm.update()
        self._t_since_flush = 0.0
        return True

    def _latlon_to_enu(self, lat: float, lon: float) -> Tuple[float, float]:
        """Eşit-dikdörtgensel projeksiyon. Yarışma alanı <1 km için yeterli."""
        assert self._lat0 is not None and self._lon0 is not None
        x = math.radians(lon - self._lon0) * self._cos_lat0 * _EARTH_R
        y = math.radians(lat - self._lat0) * _EARTH_R
        return x, y

    def enu_to_latlon(self, x: float, y: float) -> Tuple[float, float]:
        """Mock sensör tarafının kullanması için ters projeksiyon."""
        assert self._lat0 is not None and self._lon0 is not None
        lat = self._lat0 + math.degrees(y / _EARTH_R)
        lon = self._lon0 + math.degrees(x / (_EARTH_R * self._cos_lat0))
        return lat, lon

    # ----- sorgu -----

    def current_pose(self) -> Tuple[float, float, float]:
        """En son smooth tahmini (x, y, psi) olarak döndür."""
        p = self._sm.current_pose()
        return p.x(), p.y(), p.theta()

    def all_xy_psi(self) -> np.ndarray:
        """Tüm geçmiş pozları (N, 3) [x, y, psi] olarak döndür."""
        return self._sm.all_xy_psi()

    @property
    def is_origin_set(self) -> bool:
        return self._lat0 is not None
