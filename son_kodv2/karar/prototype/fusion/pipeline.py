"""
Girdap İDA — iSAM2 Sensör Füzyonu Boru Hattı (ROS-bağımsız)

Layer 2 fusion_node ve Layer 0 birim testleri ortak bu sınıfı kullanır.
ROS 2 mesaj tipleri yerine düz skaler değer alır; böylece pytest rclpy
olmadan koşar (gtsam içeren .venv yeterli).

Akış:
    1) on_velocity(vx, vy)         — /mavros/local_position/velocity_body
    2) on_imu(t, omega_z)          — /mavros/imu/data, gyro yaw rate
       Her IMU çağrısında ara adım (vx·dt, vy·dt, ωz·dt) BİRİKTİRİLİR;
       keyframe periyodu dolduğunda birikmiş delta TEK BetweenFactor
       olarak add_odometry'ye gönderilir.
    3) on_gps(lat, lon, sigma_xy)  — /mavros/global_position/global
       İlk fix origin olarak alınır; sonraki fix'ler ENU'ya
       eşit-dikdörtgensel projeksiyonla çevrilip add_gps prior'u olur.
       sigma_xy fix kalitesinden gelir (bkz. fusion.gps_quality).

Tasarım kararları:
    - IMU pre-integration ham accel'den değil, mavros'un EKF-temelli
      velocity_body çıktısından yapılır. Gerçek sahada bias-düzeltilmiş
      hız bu topic'te zaten mevcut; ham accel integrasyonunun drift'ini
      bypass etmenin temiz yolu bu.
    - Yaw rate IMU gyrosundan (omega_z); mavros velocity_body bazen yaw
      hızını içermez, bu yüzden ayrı kanal.
    - GPS prior kabul edilmeden önce bekleyen IMU integrasyonu flush
      edilir (latest_key güncel olsun).

Keyframe throttle (graf büyümesi):
    Eski davranışta her flush odom_period_s (0.1 s) = 10 Hz key üretiyordu;
    20 dk'lık yarışma görevinde ~12 000 key → ISAM2 grafiği ve her
    calculateEstimate() çağrısı bununla doğru orantılı büyür. keyframe_rate_hz
    (varsayılan 5 Hz) key kadansını sınırlar; ara IMU adımları Pose2
    kompozisyonuyla biriktirilip tek faktöre indirgendiği için BİLGİ KAYBI
    YOKTUR (eski kod zaten yalnız son hız örneğini kullanıyordu — bu yönüyle
    biriktirme daha da doğru). Odometri sigma'sı √Δt ile ölçeklenir ki
    throttle filtreyi sessizce yeniden ayarlamasın.
"""

from __future__ import annotations

import logging
import math
from dataclasses import dataclass
from typing import Optional, Tuple

import gtsam
import numpy as np

from prototype.fusion.isam2_smoother import ISAM2Smoother, ISAM2SmootherConfig

_log = logging.getLogger(__name__)

# WGS-84 yarı-büyük yarıçapı (ENU projeksiyonu için yeterli yaklaşım)
_EARTH_R = 6378137.0


# GPS prior'unu mevcut key'e bagladigimizda olusan konum hatasi, olcumun
# kendi sigma'sinin bu kesrini asiyorsa YENI KEY acilir. 0,2 = "hatanin
# katkisi olcumun gurultusunun besde birinden kucukse ihmal edilebilir".
# Sabit bir mesafe DEGIL, olcumun belirsizligine gore olceklenen bir oran.
GPS_KEY_BAYATLIK_PAYI = 0.2


@dataclass
class FusionPipelineConfig:
    """Boru hattı ayarları. ROS 2 parametre arayüzünden aynı isimle gelir."""

    odom_period_s: float = 0.1            # IMU delta biriktirme adımı (alt sınır)
    gps_sigma_xy: float = 0.30            # m (mock için RTK olmayan değer; saha 0.05)
    odom_sigma_xy: float = 0.05           # m, vel·dt ölçek gürültüsü
    odom_sigma_psi: float = 0.01          # rad

    # Key (keyframe) üretim hızı tavanı — graf büyümesini sınırlar.
    # <= 0 → throttle KAPALI, kadans odom_period_s'te kalır (eski davranış).
    keyframe_rate_hz: float = 5.0

    # GPS outlier reddi (ISAM2Smoother'a geçer)
    gps_robust_enabled: bool = True
    gps_huber_k: float = 1.345

    # Mutlak yön düzeltmesi (ISAM2Smoother.add_heading'e geçer). True →
    # her keyframe'de en son FC AHRS örneği (varsa) heading prior'u olarak
    # eklenir — jiroskop-yalnız entegrasyonun sınırsız kaymasını (drift)
    # önler. False → eski davranış (yalnız jiroskop, hiç mutlak referans yok).
    heading_correction_enabled: bool = True
    heading_sigma_psi: float = 0.05       # rad, bkz. ISAM2SmootherConfig

    @property
    def keyframe_period_s(self) -> float:
        """Etkin key periyodu: throttle ile odom_period_s'in büyüğü.

        odom_period_s bir ALT SINIR olarak kalır — throttle'ı kapatmak
        (keyframe_rate_hz<=0) eski kadansı birebir geri verir.
        """
        if self.keyframe_rate_hz <= 0.0:
            return self.odom_period_s
        return max(self.odom_period_s, 1.0 / self.keyframe_rate_hz)


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
                gps_robust_enabled=self.cfg.gps_robust_enabled,
                gps_huber_k=self.cfg.gps_huber_k,
                heading_sigma_psi=self.cfg.heading_sigma_psi,
            )
        )
        self._sm.initialize(gtsam.Pose2(0.0, 0.0, 0.0))

        # Pre-integration akümülatörleri
        self._vx_body: float = 0.0
        self._vy_body: float = 0.0
        self._last_imu_t: Optional[float] = None
        self._t_since_flush: float = 0.0
        # En son görülen mutlak yön örneği (FC AHRS) — her keyframe flush'ında
        # heading prior'u olarak eklenir. None = hiç örnek gelmedi (heading
        # düzeltmesi o key'de atlanır, sistem eski jiroskop-yalnız davranışına
        # sessizce düşer — sert hata değil, kademeli bozulma).
        self._last_psi_sample: Optional[float] = None
        # Keyframe'ler arası birikmiş göreli poz (body frame). Ara IMU
        # adımları Pose2 kompozisyonuyla eklenir → dönüş sırasında bile
        # doğru; skaler toplam olsaydı yaw değişimi ihmal edilirdi.
        self._acc_delta: gtsam.Pose2 = gtsam.Pose2()

        # GPS origin (ilk fix)
        self._lat0: Optional[float] = None
        self._lon0: Optional[float] = None
        self._cos_lat0: float = 1.0
        #: NaN/Inf reddi bir kez WARN basar (sel olmasın), sayaç her zaman artar.
        self._n_non_finite_rejected: int = 0
        self._non_finite_warned: bool = False

    # ----- callback API (ROS 2 mesaj alanlarıyla 1:1 eşleşir) -----

    def _reddet_non_finite(self, kaynak: str, *degerler: float) -> bool:
        """🔴 14.08 — SONLU DEĞER KAPISI (bkz. `fusion_node._on_gps` KAR-06 ile
        aynı aile). Jetson günlüğünde 13.08 22:46'da `fusion_node` **SIGSEGV**
        ile öldü (exit code -11) — Python istisnası DEĞİL, native çökme, hiçbir
        traceback bırakmadı. `_flush()`'un yakaladığı `IndeterminantLinear
        SystemException` (bkz. o metodun docstring'i, 11.08 olayı) bu değil;
        bu istisna FIRLATMADAN önce sistemi öldüren bir sınıf.

        Kesin kök neden bir çekirdek dökümü olmadan KANITLANAMAZ — ama GTSAM'ın
        Eigen tabanlı Cholesky çözücüsü NaN/Inf girdisinde C++ tarafında
        segfault ÜRETEBİLİR (yönetilen bir istisna değil). Bu üç giriş noktası
        (GPS lat/lon, IMU açısal hız + heading, body hızı) DIŞ DONANIMDAN
        gelir — sistem sınırı. `_on_gps`'teki sıfır-kovaryans ve yenilik
        kapıları da AYNI gerekçeyle var (kötü niyetli/bozuk veri); bu kapı
        onların YANINDA, NaN/Inf boşluğunu kapatıyor.

        Neden burada, ROS düğümünde DEĞİL: bu sınıf `sanal_gol.py`'nin sahte
        beslemesini de besliyor — kapı ROS-bağımsız katmanda olursa HER iki
        yol da korunur.
        """
        if all(math.isfinite(v) for v in degerler):
            return False
        self._n_non_finite_rejected += 1
        if not self._non_finite_warned:
            self._non_finite_warned = True
            _log.error(
                "%s: NaN/Inf DEĞER REDDEDİLDİ (%s) — geçerli bir sensör bunu "
                "üretmez. Muhtemel sebep: bozuk sürücü/hat. Kabul edilseydi "
                "GTSAM'a native çökme (SIGSEGV) riskiyle giderdi.",
                kaynak, degerler,
            )
        return True

    def on_velocity(self, vx_body: float, vy_body: float) -> None:
        """Body-frame hız akümülatörünü güncelle (TwistStamped.linear)."""
        if self._reddet_non_finite("on_velocity", vx_body, vy_body):
            return
        self._vx_body = vx_body
        self._vy_body = vy_body

    def on_imu(
        self, t: float, omega_z: float, psi: Optional[float] = None
    ) -> bool:
        """
        IMU mesajı: yaw rate'i güncelle, ara adımı biriktir, keyframe periyodu
        dolduğunda birikmiş deltayı TEK BetweenFactor olarak smoother'a gönder.

        psi: FC AHRS'inin o anki MUTLAK yön tahmini (rad, varsa —
            /mavros/imu/data orientation'dan). heading_correction_enabled
            açıksa bir sonraki flush'ta heading prior'u olarak eklenir
            (en son örnek kullanılır — IMU rate'inde her mesajda GRAF
            BÜYÜMEZ, yalnız keyframe kadansında). None → o flush'ta heading
            düzeltmesi atlanır (eski davranış).
        Dönüş: True ise smoother'a yeni key yazıldı.
        """
        degerler = (t, omega_z) if psi is None else (t, omega_z, psi)
        if self._reddet_non_finite("on_imu", *degerler):
            return False
        if psi is not None:
            self._last_psi_sample = psi
        if self._last_imu_t is None:
            self._last_imu_t = t
            return False

        dt = t - self._last_imu_t
        self._last_imu_t = t
        # Saçma dt'leri at (zaman geri sıçraması veya uzun gap)
        if dt <= 0.0 or dt > 0.5:
            return False

        # Ara adımı birikmiş deltaya BAĞLA (compose): her alt adım kendi
        # body frame'inde ifade edildiği için düz toplam değil kompozisyon.
        self._acc_delta = self._acc_delta.compose(
            gtsam.Pose2(self._vx_body * dt, self._vy_body * dt, omega_z * dt)
        )
        self._t_since_flush += dt
        if self._t_since_flush < self.cfg.keyframe_period_s:
            return False

        return self._flush()

    def on_gps(
        self, lat: float, lon: float, sigma_xy: Optional[float] = None
    ) -> None:
        """GPS fix: ENU'ya çevir, latest_key'e prior ekle.

        sigma_xy: fix kalitesinden türetilen ölçüm sigma'sı (m). None →
        config gps_sigma_xy. Reddedilmiş fix'ler buraya HİÇ gelmemeli
        (bkz. fusion.gps_quality.sigma_for_status).
        """
        degerler = (lat, lon) if sigma_xy is None else (lat, lon, sigma_xy)
        if self._reddet_non_finite("on_gps", *degerler):
            return
        if self._lat0 is None:
            # İlk fix → origin. Smoother başlangıçtan beri (0,0)'da; origin
            # buraya pinlenir. add_gps eklemeden update yapma; X(0) zaten
            # PriorFactor anchor'una sahip.
            self._lat0 = lat
            self._lon0 = lon
            self._cos_lat0 = math.cos(math.radians(lat))
            return

        # 🔴 17.08.2026 — KEYFRAME THROTTLE'INI GPS EZİYORDU.
        #
        # Eskiden burada koşulsuz `_flush(force=True)` vardı ve `force=True`
        # `keyframe_period_s` kapısını ATLAR. Tasarım GPS'i **1 Hz** varsaymış
        # (CLAUDE.md: "IMU 50 Hz + GPS 1 Hz" → 11.416 key'i 6.000'e indirdi).
        # Cihazda MAVROS GPS'i **9,9 Hz** veriyor (ölçüldü: gps=50/5 sn) ⇒ her
        # fix yeni key açıyordu ve throttle'ın kazancı sahada TAMAMEN geri
        # veriliyordu — hata basılmadan.
        #
        # ÖLÇÜLDÜ (gerçek boru hattı, 8 dk görev, IMU 10 Hz):
        #     GPS  1,0 Hz →  2.016 key ·  36,6 s hesap  (%7,6 çekirdek)
        #     GPS  9,9 Hz →  4.799 key · 112,9 s hesap  (%23,5 çekirdek)
        #   throttle'ın vaat ettiği: 2.400 key ⇒ gerçekte 2× fazla, 3,1× pahalı
        # Cihazda karşılığı: fusion_node %16 → %40 (11 dk), 1,5 saatte %85-96.
        #
        # ÇÖZÜM — eşik SABİT YAZILMIYOR, ölçümün KENDİ belirsizliğinden
        # türetiliyor (§"koruma, koruduğu değerden TÜRETİLİR"):
        # Prior'u mevcut key'e bağlamak, key'in bayatlığı kadar konum hatası
        # katar. O hata ölçümün kendi σ'sının yanında ihmal edilebilirse yeni
        # key AÇMAYA DEĞMEZ; değilse açılır. Böylece:
        #   * tek-nokta fix (σ=2,5 m) → eşik 50 cm; 0,6 m/s'de 0,83 s ⇒
        #     throttle yönetir, key sayısı tasarımdaki gibi kalır
        #   * RTK (σ=5 cm)           → eşik 1 cm; neredeyse her fix'te flush ⇒
        #     ESKİ DAVRANIŞ birebir korunur, hassasiyet kaybı YOK
        # Yani düzeltme fix kalitesine göre kendini ayarlıyor; RTK gelirse
        # hiçbir şey değişmiyor.
        kaymis = self._acc_delta.translation()
        mesafe = float(math.hypot(float(kaymis[0]), float(kaymis[1])))
        sigma = self.cfg.gps_sigma_xy if sigma_xy is None else float(sigma_xy)
        if mesafe > GPS_KEY_BAYATLIK_PAYI * max(sigma, 1e-9):
            self._flush(force=True)      # bayatlık ölçümün yanında ANLAMLI
        else:
            self._flush()                # vakti geldiyse açar, gelmediyse AÇMAZ

        x, y = self._latlon_to_enu(lat, lon)
        self._sm.add_gps(self._sm.latest_key, x, y, sigma_xy=sigma_xy)
        self._sm.update()

    # ----- iç yardımcılar -----

    def _flush(self, force: bool = False) -> bool:
        """Birikmiş ara IMU adımlarını tek BetweenFactor olarak smoother'a yaz."""
        period = self._t_since_flush
        if period <= 0.0:
            return False
        if not force and period < self.cfg.keyframe_period_s:
            return False

        # Rastgele yürüyüş: σ ∝ √Δt. Ölçek nominal odom_period_s'e göre alınır,
        # böylece throttle kapalıyken (period == odom_period_s) katsayı 1.0 ve
        # davranış eski koda birebir eşit kalır.
        nominal = self.cfg.odom_period_s
        sigma_scale = math.sqrt(period / nominal) if nominal > 0.0 else 1.0

        self._sm.add_odometry(self._acc_delta, sigma_scale=sigma_scale)
        # Aynı key'e, aynı update() içinde: FC AHRS'inin en son mutlak yön
        # örneği varsa heading prior'u ekle. add_odometry'den SONRA çünkü
        # add_heading key_index'in (self._sm.latest_key) zaten var olmasını
        # ister — add_odometry az önce onu oluşturdu.
        if self.cfg.heading_correction_enabled and self._last_psi_sample is not None:
            self._sm.add_heading(self._sm.latest_key, self._last_psi_sample)
        self._sm.update()
        self._acc_delta = gtsam.Pose2()
        self._t_since_flush = 0.0
        return True

    def set_origin(self, lat: float, lon: float) -> None:
        """ENU orijinini AÇIKÇA çak — süreç yeniden doğduğunda çerçeve kaymasın.

        🔴 12.08 (kaptanın F-M.12 respawn değişikliğiyle etkileşim): orijin
        normalde İLK GPS FIX'inden alınıyor. `fusion_node` artık `respawn=True`
        ile açıldığı için, düğüm ölüp yeniden doğduğunda `_lat0` `None`'a döner
        ve **bir sonraki fix yeni orijin olur** → dünya çerçevesi aracın o anki
        konumuna yeniden çakılır.

        Görev hedefleri bundan etkilenmez (mission topic'leri ENU-hizalı
        ÖTELEME taşır, odom xy ile birlikte kayar) ama `planning_node`'da
        DÜNYA çerçevesinde biriken her şey bozulur: `EdgeBuoyMemory`, geçilmiş
        kapı kayıtları (md 5.5.3.1 puan sayacı), RRT* referansı, MPPI
        warm-start. O düğüm eşzamanlı yeniden başlamaz. KAR-11'de tam bu tür
        bir kayma kenar hafızasını şişirmişti.

        ⇒ Düğüm katmanı orijini kalıcılaştırıp respawn'dan sonra buradan geri
        yükler; böylece "sessiz ölüm" kapatılırken yerine "sessiz çerçeve
        kayması" konmaz.

        Yalnız orijin HENÜZ ÇAKILMAMIŞSA çağrılabilir: koşu ortasında orijin
        değiştirmek, tam da engellemeye çalıştığımız sıçramayı üretir.
        """
        if self._lat0 is not None:
            raise RuntimeError(
                "ENU orijini zaten cakili — kosu ortasinda degistirmek "
                "cerceve sicramasi uretir"
            )
        self._lat0 = lat
        self._lon0 = lon
        self._cos_lat0 = math.cos(math.radians(lat))

    def origin_latlon(self) -> Optional[Tuple[float, float]]:
        """Çakılı ENU orijini (lat, lon) — çakılmadıysa None."""
        if self._lat0 is None or self._lon0 is None:
            return None
        return (self._lat0, self._lon0)

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
