"""Füzyon: GRAF BÜYÜMESİ ve SICAK YOL MALİYETİ — 17.08.2026 düzeltmeleri.

Cihazda ölçülen belirti: `fusion_node` CPU'su çalışma süresiyle tırmanıyor —
**%16 → %40 (11 dakika)**, ~1,5 saatte **%85-96**. Yarışma açısından önemli,
çünkü tekne koşumdan saatler önce açılıyor: yarış anında N zaten büyük.

İki bağımsız kök neden bulundu ve ikisi de burada donduruluyor:

**A · Keyframe throttle'ını GPS eziyordu.** `on_gps` koşulsuz
`_flush(force=True)` çağırıyordu ve `force=True` `keyframe_period_s` kapısını
ATLAR. Tasarım GPS'i 1 Hz varsaymış (CLAUDE.md); cihazda MAVROS 9,9 Hz veriyor.
Ölçüldü (gerçek boru hattı, 8 dk, IMU 10 Hz):
    GPS 1,0 Hz → 2.016 key ·  36,6 s   |  GPS 9,9 Hz → 4.799 key · 112,9 s
    throttle'ın vaat ettiği: 2.400 key ⇒ **2× fazla, 3,1× pahalı**
Düzeltme eşiği SABİT DEĞİL, ölçümün kendi σ'sından türer ⇒ RTK'da eski
davranış birebir korunur, tek-nokta fix'te throttle geri kazanılır.

**B · `calculateEstimate()` sıcak yolda O(N).** GTSAM dokümanı tek anahtar
için `calculateEstimate(KEY)` öneriyor. Ölçüldü (bu Jetson):
    N=6000 → tam **3,87 ms** · tek-anahtar **0,008 ms** (N'den BAĞIMSIZ)
⚠️ Tuzak: `_latest_estimate` aynı zamanda **tekilleşme kurtarma çapasıydı**
(11.08'de fusion_node'u öldüren zincir). Çapa korundu — tam anlık görüntü
yerine çözülmüş son (anahtar, poz) ikilisi saklanıyor.
"""
import math

import gtsam
import pytest

from prototype.fusion.pipeline import (
    GPS_KEY_BAYATLIK_PAYI, FusionPipeline, FusionPipelineConfig,
)

IMU_HZ = 10.0
HIZ = 0.6                 # m/s — bantlardan ölçülen medyan seyir
LAT0, LON0 = 40.7, 29.5


def _kosum(*, gps_hz: float, sigma_xy: float, saniye: float = 20.0) -> int:
    """Boru hattını sür, üretilen key sayısını döndür."""
    p = FusionPipeline(FusionPipelineConfig())
    dt = 1.0 / IMU_HZ
    n = int(saniye * IMU_HZ)
    gps_her = max(1, int(round(IMU_HZ / gps_hz)))
    p.on_gps(LAT0, LON0, sigma_xy=sigma_xy)           # orijin
    for i in range(1, n + 1):
        p.on_velocity(HIZ, 0.0)
        p.on_imu(i * dt, 0.0, psi=0.0)
        if i % gps_her == 0:
            dlat = (i * HIZ * dt) / 111320.0
            p.on_gps(LAT0 + dlat, LON0, sigma_xy=sigma_xy)
    return p._sm.latest_key


# ── A: GPS artık throttle'ı ezmiyor ────────────────────────────────────

def test_HIZLI_GPS_keyframe_throttle_ini_EZMEZ():
    """🔴 Asıl regresyon testi. Kırmızıya dönerse graf 2× hızlı büyüyor."""
    cfg = FusionPipelineConfig()
    saniye = 20.0
    tavan = saniye * cfg.keyframe_rate_hz          # throttle'ın vaadi

    yavas = _kosum(gps_hz=1.0, sigma_xy=2.5, saniye=saniye)
    hizli = _kosum(gps_hz=10.0, sigma_xy=2.5, saniye=saniye)

    assert hizli <= tavan * 1.1, (
        f"GPS 10 Hz {hizli} key üretti, throttle tavanı {tavan:.0f} — "
        "force flush kapısı geri gelmiş, graf tasarımdan hızlı büyüyor")
    assert hizli <= yavas * 1.25, (
        f"GPS hızı key sayısını {hizli / max(yavas, 1):.1f}× artırdı; "
        "throttle GPS hızından BAĞIMSIZ olmalı")


def test_RTK_HASSASIYETINDE_ESKI_DAVRANIS_KORUNUR():
    """σ küçükken (RTK) key'in bayatlığı ölçümün yanında ANLAMLI olur.

    O durumda flush zorlanmalı — yoksa 0,2 s'lik bayatlık (0,6 m/s'de 12 cm)
    5 cm'lik RTK ölçümünden büyük bir hata katardı. Yani düzeltme
    hassasiyeti DÜŞÜRMÜYOR, fix kalitesine göre kendini ayarlıyor.
    """
    tek_nokta = _kosum(gps_hz=10.0, sigma_xy=2.5, saniye=20.0)
    rtk = _kosum(gps_hz=10.0, sigma_xy=0.05, saniye=20.0)
    assert rtk > tek_nokta, (
        "RTK'da da throttle uygulanıyor — hassas ölçüm bayat key'e bağlanıyor")


def test_ESIK_SABIT_DEGIL_OLCUMUN_SIGMASINDAN_TURER():
    """§'koruma, koruduğu değerden TÜRETİLİR': sahada ayarlanacak sayı YOK.

    Şartname mesafelerin alana göre değişeceğini söylüyor; sabit bir metre
    eşiği tahmin olurdu. Eşik ölçümün kendi belirsizliğine oranlı.
    """
    assert 0.0 < GPS_KEY_BAYATLIK_PAYI < 1.0
    # 0,6 m/s'de tek-nokta fix (σ=2,5) eşiği: 0,5 m ⇒ 0,83 s ⇒ throttle yönetir
    assert GPS_KEY_BAYATLIK_PAYI * 2.5 / HIZ > FusionPipelineConfig().keyframe_period_s
    # RTK (σ=0,05) eşiği: 1 cm ⇒ 0,017 s ⇒ throttle'dan kısa, flush zorlanır
    assert GPS_KEY_BAYATLIK_PAYI * 0.05 / HIZ < FusionPipelineConfig().keyframe_period_s


# ── B: sıcak yolda tam calculateEstimate() YOK ─────────────────────────

class _Sayan:
    """Gerçek ISAM2'yi sarar, hangi sorgunun kaç kez çağrıldığını sayar."""

    def __init__(self, gercek):
        self._g = gercek
        self.tam = 0
        self.tek = 0

    def update(self, *a):
        return self._g.update(*a)

    def calculateEstimate(self):
        self.tam += 1
        return self._g.calculateEstimate()

    def calculateEstimatePose2(self, key):
        self.tek += 1
        return self._g.calculateEstimatePose2(key)


def _hazir_boru():
    p = FusionPipeline(FusionPipelineConfig())
    p.on_gps(LAT0, LON0, sigma_xy=2.5)
    for i in range(1, 21):
        p.on_velocity(HIZ, 0.0)
        p.on_imu(i * 0.1, 0.0, psi=0.0)
    return p


def test_SICAK_YOLDA_TAM_calculateEstimate_CAGRILMAZ():
    """🔴 O(N) maliyetin geri gelmesini engelleyen test."""
    p = _hazir_boru()
    say = _Sayan(p._sm._isam)
    p._sm._isam = say
    for i in range(21, 61):                    # 4 saniyelik normal akış
        p.on_velocity(HIZ, 0.0)
        p.on_imu(i * 0.1, 0.0, psi=0.0)
        p.on_gps(LAT0 + i * 1e-6, LON0, sigma_xy=2.5)
        p.current_pose()
    assert say.tam == 0, (
        f"sıcak yolda tam calculateEstimate() {say.tam} kez çağrıldı — "
        "O(N) maliyet geri geldi")
    assert say.tek > 0, "tek-anahtar sorgu hiç kullanılmamış"


def test_all_poses_SOGUK_YOLDA_TAM_SORGU_KULLANIR():
    """Bütün pozlar isteniyorsa tam sorgu DOĞRU seçim — orada kısıtlamıyoruz."""
    p = _hazir_boru()
    say = _Sayan(p._sm._isam)
    p._sm._isam = say
    poses = p._sm.all_poses()
    assert len(poses) == p._sm.latest_key + 1
    assert say.tam == 1


def test_current_pose_DOGRU_DEGERI_VERIYOR():
    """Hız değişikliği sonucu bozmamalı: ileri giden tekne ileri görünmeli."""
    p = _hazir_boru()
    x, y, psi = p.current_pose()
    assert x > 0.5, f"2 sn × 0,6 m/s ilerledi, x={x:.2f} beklenmedik"
    assert abs(y) < 0.5 and abs(psi) < 0.1


def test_kurtarma_CAPASI_HALA_DURUYOR():
    """Tam anlık görüntü kaldırıldı ama kurtarma çapası korunmalı.

    Çapa kaybolursa 11.08'in zinciri geri gelir: tekilleşme → istisna →
    fusion_node ölür → poz yayını kesilir → planning_node MPPI'yi durdurur.
    """
    p = _hazir_boru()
    sm = p._sm
    assert sm._son_iyi is not None, "çözülmüş çapa saklanmıyor"
    anahtar, poz = sm._son_iyi
    assert anahtar == sm.latest_key
    assert isinstance(poz, gtsam.Pose2)
