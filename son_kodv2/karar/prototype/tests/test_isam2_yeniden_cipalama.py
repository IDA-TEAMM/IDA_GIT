"""§1.56g — DÖNEMSEL YENİDEN ÇIPALAMA nöbetçileri.

SORUN (ölçüldü, çevrimdışı, bu Jetson): `ISAM2Smoother` grafı hiç budamıyor.
Sentetik 70 dk @10 Hz koşumda `update()` 3,40 ms → 72,00 ms (**21,2×**),
düzleşme yok. Tekne saatlerce açık kalıyor.

ÇÖZÜM: periyodik olarak son ÇÖZÜLMÜŞ pozu çapa alıp grafı yeniden kur.
`marginalizeLeaves` ÇAĞRILMAZ (GTSAM #1101'in üç hatası orada; üstelik o
fonksiyon Python binding'inde hiç açık değil — denetlenemez).

Buradaki her test bir MUTASYONLA doğrulandı: kod bozulunca kırmızı olduğu
görülmeden test yazılmış sayılmaz.
"""
from __future__ import annotations

import math
import time

import gtsam
import numpy as np
import pytest

from prototype.fusion.isam2_smoother import ISAM2Smoother, ISAM2SmootherConfig
from prototype.fusion.pipeline import FusionPipelineConfig


def _cfg(**kw) -> ISAM2SmootherConfig:
    taban = dict(
        prior_sigma_xy=0.05, prior_sigma_psi=0.05,
        odom_sigma_xy=0.10, odom_sigma_psi=0.02, gps_sigma_xy=0.05,
        gps_robust_enabled=False, heading_robust_enabled=False,
    )
    taban.update(kw)
    return ISAM2SmootherConfig(**taban)


def _kosum(n: int, periyot: int = 0, keep: int = 0, tohum: int = 7):
    """Düz bir yörüngede n adım koştur; (smoother, hatalar, süreler) döndür."""
    rng = np.random.default_rng(tohum)
    sm = ISAM2Smoother(_cfg(reanchor_period_keys=periyot, reanchor_keep_keys=keep))
    sm.initialize(gtsam.Pose2(0.0, 0.0, 0.0))
    x = y = psi = 0.0
    hatalar, sureler = [], []
    for i in range(1, n + 1):
        dx, dpsi = 0.05, 0.01
        x += dx * math.cos(psi); y += dx * math.sin(psi); psi += dpsi
        t0 = time.perf_counter()
        sm.add_odometry(gtsam.Pose2(dx + rng.normal(0, 0.003), 0.0,
                                    dpsi + rng.normal(0, 0.001)))
        if i % 10 == 0:
            sm.add_gps(sm.latest_key, x + rng.normal(0, 0.05), y + rng.normal(0, 0.05))
        sm.update()
        p = sm.current_pose()
        sureler.append(time.perf_counter() - t0)
        hatalar.append(math.hypot(p.x() - x, p.y() - y))
    return sm, np.array(hatalar), np.array(sureler) * 1e3


# ── kapalıyken hiçbir şey değişmemeli ──────────────────────────────────
def test_VARSAYILAN_kapali_eski_davranis_birebir():
    """Varsayılan 0 = kapalı. Bir tek çıpalama olmamalı ve graf baştan sona
    tek parça kalmalı — yeni ayarın ESKİ koşumları sessizce değiştirmemesi
    bu projede ilk kabul şartı (`${X:-0}` deseni)."""
    sm, _, _ = _kosum(200)
    assert ISAM2SmootherConfig().reanchor_period_keys == 0
    assert sm.reanchor_count == 0
    assert sm.anchor_key == 0
    assert len(sm.all_poses()) == sm.latest_key + 1


# ── açıkken gerçekten ateşlemeli ───────────────────────────────────────
def test_periyot_dolunca_cipalama_ATESLIYOR():
    sm, _, _ = _kosum(300, periyot=50)
    assert sm.reanchor_count == 300 // 50, sm.reanchor_count
    # Çıpalama bir ARIZA değil: kurtarma sayacına yazılmamalı.
    assert sm.recovery_count == 0


def test_cipalama_GRAFI_kuculttu():
    """Asıl amaç. Çıpalamadan sonra graf tek poza inmeli (kuyruksuz)."""
    sm, _, _ = _kosum(300, periyot=50)
    assert sm.anchor_key == sm.latest_key
    assert len(sm.all_poses()) == 1


def test_update_suresi_BUYUMUYOR():
    """Hastalığın kendisi: süre koşum boyunca tırmanıyor mu?

    Eşik gevşek (2,0×) çünkü ölçüm makine yüküne duyarlı; kusurun imzası
    21,2× mertebesinde — bu eşik gürültüyü geçirir, hastalığı geçirmez.
    """
    _, _, s_acik = _kosum(1500, periyot=150)
    n = len(s_acik) // 5
    ilk, son = s_acik[:n].mean(), s_acik[-n:].mean()
    assert son / max(ilk, 1e-9) < 2.0, f"ilk {ilk:.3f} ms son {son:.3f} ms"


def test_dogruluk_BOZULMADI():
    """Hız doğruluktan çalınmamalı. Aynı yörünge, aynı tohum, tek değişken."""
    _, h_kapali, _ = _kosum(600, periyot=0)
    _, h_acik, _ = _kosum(600, periyot=100)
    # %20'den fazla kötüleşme kabul edilmez (ölçülen fark ~%0).
    assert h_acik.mean() < h_kapali.mean() * 1.2 + 1e-3, (
        f"kapali {h_kapali.mean():.4f} m · acik {h_acik.mean():.4f} m"
    )


def test_dikis_SICRAMASI_yok():
    """Sıfırlama anında poz sıçramamalı — araç yalpalamaz.

    Çevrimdışı ölçüm sıçramanın NEGATİF olduğunu söylüyor (−0,0059 m):
    sıfırlama birikmiş doğrusallaştırma bayatlığını atıyor.
    """
    sm = ISAM2Smoother(_cfg(reanchor_period_keys=50))
    sm.initialize(gtsam.Pose2(0.0, 0.0, 0.0))
    onceki = sm.current_pose()
    sicrama_max = 0.0
    for i in range(1, 201):
        sm.add_odometry(gtsam.Pose2(0.05, 0.0, 0.0))
        sm.update()
        p = sm.current_pose()
        # adım başına beklenen yer değişimi 0,05 m; fazlası sıçramadır
        sicrama_max = max(sicrama_max, abs(math.hypot(p.x()-onceki.x(), p.y()-onceki.y()) - 0.05))
        onceki = p
    assert sicrama_max < 0.01, f"dikiste {sicrama_max:.4f} m sicrama"


# ── kuyruk tutma (kaptan sorusu: "hepsini silmesek") ───────────────────
def test_KUYRUK_korunuyor():
    sm, _, _ = _kosum(300, periyot=50, keep=20)
    assert sm.anchor_key == sm.latest_key - 20
    assert len(sm.all_poses()) == 21


def test_kuyruk_SIFIR_saf_cipalama_ile_ayni_yol():
    """keep=0 özel durum DEĞİL, aynı kod yolunun ucu."""
    sm, _, _ = _kosum(200, periyot=50, keep=0)
    assert sm.anchor_key == sm.latest_key


# ── geçmiş sorgusu çapanın altına inmemeli (asıl tuzak) ────────────────
def test_all_poses_CAPANIN_ALTINA_inmiyor():
    """`range(0, latest+1)` taraması çıpalamadan sonra GTSAM'de patlar.

    Bu kusur tekilleşme kurtarmasında da vardı ama kurtarma nadir olduğu
    için hiç tetiklenmemişti; çıpalama onu RUTİN yapıyor.
    """
    sm, _, _ = _kosum(300, periyot=50, keep=10)
    pozlar = sm.all_poses()          # patlamamalı
    assert len(pozlar) == 11
    assert sm.all_xy_psi().shape == (11, 3)


def test_pose_at_capanin_altinda_ACIK_hata_veriyor():
    sm, _, _ = _kosum(300, periyot=50)
    with pytest.raises(ValueError, match="çapanın altında"):
        sm.pose_at(0)
    sm.pose_at(sm.latest_key)        # üstü çalışmalı


# ── ayar kapıları ──────────────────────────────────────────────────────
def test_kuyruk_periyottan_UZUNSA_reddediliyor():
    """keep >= period ise çıpalama hiçbir şey atmaz: ayar "açık" görünür ama
    graf yine sınırsız büyür. Sessiz etkisizlik, açık hatadan kötüdür."""
    with pytest.raises(ValueError, match="KÜÇÜK olmalı"):
        ISAM2Smoother(_cfg(reanchor_period_keys=50, reanchor_keep_keys=50))


def test_negatif_periyot_reddediliyor():
    with pytest.raises(ValueError, match="negatif olamaz"):
        ISAM2Smoother(_cfg(reanchor_period_keys=-1))
    with pytest.raises(ValueError, match="negatif olamaz"):
        ISAM2Smoother(_cfg(reanchor_keep_keys=-1))


# ── boru hattı: saniye → anahtar çevirisi ──────────────────────────────
def test_boru_hatti_saniyeyi_ANAHTARA_ceviriyor():
    """Sabit anahtar sayısı yazmak keyframe throttle'ını sessizce ezerdi:
    5 Hz'de 30 s = 150 anahtar, 10 Hz'de 300. Çeviri kadanstan gelmeli."""
    c5 = FusionPipelineConfig(keyframe_rate_hz=5.0, reanchor_period_s=30.0,
                              reanchor_keep_s=6.0)
    assert c5.reanchor_period_keys == 150
    assert c5.reanchor_keep_keys == 30

    c10 = FusionPipelineConfig(keyframe_rate_hz=10.0, reanchor_period_s=30.0)
    assert c10.reanchor_period_keys == 300

    assert FusionPipelineConfig().reanchor_period_keys == 0      # varsayılan kapalı


# ── dağıtım ayarı gerçekten AÇIK ve ETKİLİ mi ─────────────────────────
def _yaml_yukle(ad: str):
    import pathlib
    import yaml
    kok = pathlib.Path(__file__).resolve().parents[1].parent
    yol = kok / "ros2_ws/src/girdap_decision/config" / ad
    with yol.open(encoding="utf-8") as f:
        return yaml.safe_load(f)


def _fusion_bloklari():
    """Her iki dağıtım dosyasından çıpalama ayarını (periyot, kuyruk) çıkar."""
    out = {}
    p = _yaml_yukle("params.yaml")
    for dugum, govde in p.items():
        ros = (govde or {}).get("ros__parameters", {})
        if "reanchor_period_s" in ros:
            out[f"params.yaml:{dugum}"] = (
                ros["reanchor_period_s"], ros.get("reanchor_keep_s", 0.0),
                ros.get("keyframe_rate_hz", 5.0),
            )
    h = _yaml_yukle("hardware.yaml").get("fusion", {})
    if "reanchor_period_s" in h:
        out["hardware.yaml:fusion"] = (
            h["reanchor_period_s"], h.get("reanchor_keep_s", 0.0),
            h.get("keyframe_rate_hz", 5.0),
        )
    return out


def test_DAGITIM_CONFIGI_cipalamayi_ACIK_tutuyor():
    """Ayar sessizce 0'a düşerse graf yine sınırsız büyür ve bunu kimse
    fark etmez — belirti ancak SAATLER sonra çıkar (bant ölçümü: 80 dk'da
    25×, mutlak değerler hâlâ küçük olduğu için yayın hızı bozulmuyor)."""
    bloklar = _fusion_bloklari()
    assert bloklar, "dağıtım dosyalarında reanchor_period_s HİÇ yok"
    for nere, (per, keep, _) in bloklar.items():
        assert per > 0.0, f"{nere}: çıpalama KAPALI (reanchor_period_s={per})"
        assert keep < per, f"{nere}: kuyruk({keep}) >= periyot({per}) — graf budanmaz"


def test_DAGITIM_CONFIGI_anahtara_cevrilince_ETKILI_kaliyor():
    """Saniye değerleri kadansla anahtara çevrilir; yuvarlama sonrası kuyruk
    periyoda EŞİTLENİRSE smoother ayarı reddeder (ValueError). Yani yalnız
    saniyeleri denetlemek yetmez — çevrilmiş hâli de sınanmalı."""
    for nere, (per, keep, hz) in _fusion_bloklari().items():
        cfg = FusionPipelineConfig(
            keyframe_rate_hz=hz, reanchor_period_s=per, reanchor_keep_s=keep
        )
        assert cfg.reanchor_period_keys > 0, nere
        assert cfg.reanchor_keep_keys < cfg.reanchor_period_keys, (
            f"{nere}: {cfg.reanchor_keep_keys} >= {cfg.reanchor_period_keys} anahtar"
        )
