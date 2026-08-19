"""
Girdap İDA — VARIŞ ÖLÇÜTÜ: "yaklaştım" mı, "GEÇTİM" mi (§1.68).

🔑 NEDEN BU DOSYA VAR. Şartname kapı geçişini *"İDA'nın duba ikilisinin
**%100'ünü geçmiş olması**"* diye tanımlıyor ve algı tarafı bunu düzlem aşma
ile sayıyor (`duba_gecis_navigator.PASS_EK_YOL` = ARAC_BOY 1,03 + 0,5 =
**1,53 m**). `mission_manager` ise klasik *circle of acceptance* kullanıyordu:
araç kapıya **2,0 m kala** "vardım" deyip sonraki noktaya dönüyordu.
İki taraf aynı olayı farklı tanımlıyor; açık = 2,0 + 1,53 = **3,53 m** ve
ikisi hiçbir zaman buluşmuyor.

`gecis_zorunlu` bu açığı kapatır: varış ayrıca noktadan geçen ve bacak yönüne
dik düzlemin aşılmasını da ister — `(p − wp)·t̂ > 0`. ArduRover 4.3+ kendi
waypoint tamamlamasını zaten böyle yapıyor (WP_RADIUS AUTO'da etkisiz,
ArduPilot #23457); bizde koşan sürüm V4.6.3.

⚠ Varsayılan KAPALI: bu dosyanın ilk testi o sözleşmeyi DONDURUR.

Çalıştır: pytest prototype/tests/test_varis_gecis_olcutu.py -v
"""

from __future__ import annotations

import math
from pathlib import Path

from prototype.mission.mission_manager import (
    MissionManager,
    MissionManagerConfig,
    MissionPhase,
    Waypoint,
)

_R = 6378137.0


def _kuzey_m(metre: float) -> float:
    """Kuzeye `metre` kadar ötelemenin enlem karşılığı (derece)."""
    return math.degrees(metre / _R)


def _gorev(**cfg_kw) -> MissionManager:
    """Kuzeye dizili iki nokta; araç güneyden kuzeye yaklaşır.

    P1 = (0,0), P2 P1'in 100 m kuzeyinde. Bacak yönü = +kuzey.
    """
    cfg = MissionManagerConfig(arrival_radius_m=2.0, dwell_time_s=2.0, **cfg_kw)
    m = MissionManager([Waypoint(0.0, 0.0, "P1"), Waypoint(_kuzey_m(100.0), 0.0, "P2")], cfg)
    m.start()
    return m


# --------------------------------------------------------------------------- #
# 1) SÖZLEŞME: varsayılan KAPALI ve eski davranış BİREBİR
# --------------------------------------------------------------------------- #

def test_VARSAYILAN_kapali() -> None:
    """Şalter kapalı doğmalı — açık doğarsa saha davranışı sessizce değişirdi."""
    assert MissionManagerConfig().gecis_zorunlu is False


def test_KAPALIYKEN_yaricapa_girince_varildi_ESKI_DAVRANIS() -> None:
    """Kapalıyken araç düzlemin ÖNÜNDE de olsa varış sayılır (eski davranış)."""
    m = _gorev()
    m.update(_kuzey_m(-1.0), 0.0, 0.0)          # noktanın 1 m GÜNEYİ (önünde)
    assert m.phase is MissionPhase.DWELL


# --------------------------------------------------------------------------- #
# 2) AÇIKKEN: düzlem aşılmadan varış YOK
# --------------------------------------------------------------------------- #

def test_ACIKKEN_duzlem_asilmadan_VARIS_YOK() -> None:
    """Asıl kusur budur: yarıçapın içinde ama kapının ÖNÜNDE ⇒ varış olmamalı."""
    m = _gorev(gecis_zorunlu=True)
    m.update(_kuzey_m(-1.0), 0.0, 0.0)          # 1 m güneyde, yarıçap içinde
    assert m.phase is MissionPhase.ACTIVE
    assert m.gecis_bekleyen == 1


def test_ACIKKEN_duzlem_asilinca_VARIS_VAR() -> None:
    """Düzlemi aşınca (noktanın KUZEYİ) varış hemen sayılır."""
    m = _gorev(gecis_zorunlu=True)
    m.update(_kuzey_m(-1.0), 0.0, 0.0)
    assert m.phase is MissionPhase.ACTIVE
    m.update(_kuzey_m(0.5), 0.0, 1.0)           # 0,5 m KUZEY = düzlem aşıldı
    assert m.phase is MissionPhase.DWELL
    assert m.zaman_asimiyla_varilan == 0        # zaman aşımıyla değil, gerçekten


# --------------------------------------------------------------------------- #
# 3) KİLİTLENME YEDEĞİ
# --------------------------------------------------------------------------- #

def test_zaman_asimi_KILITLENMEYI_onler() -> None:
    """Düzlem hiç aşılamazsa görev sonsuza kadar takılmamalı."""
    m = _gorev(gecis_zorunlu=True, gecis_zaman_asimi_s=5.0)
    m.update(_kuzey_m(-1.0), 0.0, 0.0)
    assert m.phase is MissionPhase.ACTIVE
    m.update(_kuzey_m(-1.0), 0.0, 4.9)          # henüz dolmadı
    assert m.phase is MissionPhase.ACTIVE
    m.update(_kuzey_m(-1.0), 0.0, 5.0)          # doldu
    assert m.phase is MissionPhase.DWELL
    assert m.zaman_asimiyla_varilan == 1        # SESSİZ düşmedi, sayaç arttı


def test_zaman_asimi_SIFIRSA_yedek_YOK() -> None:
    """0 = yedek yok: araç geçemedikçe varış asla sayılmaz."""
    m = _gorev(gecis_zorunlu=True, gecis_zaman_asimi_s=0.0)
    for t in (0.0, 10.0, 100.0, 1000.0):
        m.update(_kuzey_m(-1.0), 0.0, t)
    assert m.phase is MissionPhase.ACTIVE


def test_yaricaptan_CIKINCA_zaman_asimi_SIFIRLANIR() -> None:
    """Dalga/akıntı yüzünden girip çıkan araç zaman aşımını BİRİKTİRMEMELİ."""
    m = _gorev(gecis_zorunlu=True, gecis_zaman_asimi_s=5.0)
    m.update(_kuzey_m(-1.0), 0.0, 0.0)          # içeri
    m.update(_kuzey_m(-5.0), 0.0, 3.0)          # DIŞARI (yarıçap 2 m)
    m.update(_kuzey_m(-1.0), 0.0, 4.0)          # tekrar içeri → saat yeniden
    m.update(_kuzey_m(-1.0), 0.0, 8.0)          # girişten 4 s → dolmadı
    assert m.phase is MissionPhase.ACTIVE
    m.update(_kuzey_m(-1.0), 0.0, 9.0)          # girişten 5 s → doldu
    assert m.phase is MissionPhase.DWELL


# --------------------------------------------------------------------------- #
# 4) DÜZLEMİN YÖNÜ — bacak yönünden geliyor mu
# --------------------------------------------------------------------------- #

def test_duzlem_BACAK_yonune_dik_kurulur() -> None:
    """İkinci noktada `t̂` önceki noktadan gelir; yanal sapma ölçütü bozmaz."""
    m = _gorev(gecis_zorunlu=True)
    m.update(_kuzey_m(-1.0), 0.0, 0.0)          # P1'e GÜNEYDEN yaklaş (yön kurulur)
    m.update(_kuzey_m(0.5), 0.0, 1.0)           # P1 geçildi → DWELL
    m.update(_kuzey_m(0.5), 0.0, 4.0)           # dwell doldu → P2 aktif
    assert m.current_index == 1
    # P2'nin 1 m GÜNEYİ ama 1 m DOĞUSU: yanal kayma var, düzlem aşılmadı
    m.update(_kuzey_m(99.0), math.degrees(1.0 / _R), 4.0)
    assert m.phase is MissionPhase.ACTIVE
    # P2'nin 0,5 m KUZEYİ: aşıldı
    m.update(_kuzey_m(100.5), 0.0, 5.0)
    assert m.phase is MissionPhase.DWELL


def test_ILK_noktaya_GECMIS_halde_girilirse_yon_KURULAMAZ() -> None:
    """🪤 TASARIM KENARI — donduruluyor.

    İlk waypoint'te önceki nokta yoktur; yaklaşma yönü aracın çembere GİRDİĞİ
    andaki `araç → nokta` vektöründen kurulur. Araç çembere noktayı ZATEN
    geçmiş hâlde girerse (ör. görev tam üstünde başlarsa) o yön TERS çıkar ve
    ölçüt "henüz geçmedim" der. Kilitlenmez — zaman aşımı yedeği devreye girer.

    Bu bilinçli bir seçim: yön bilgisi yokken "geçtim" demek, kapıyı hiç
    geçmeden puan saymaktan daha tehlikeli olurdu (şartname %100 geçiş ister).
    """
    m = _gorev(gecis_zorunlu=True, gecis_zaman_asimi_s=5.0)
    m.update(_kuzey_m(0.5), 0.0, 0.0)           # çembere KUZEYDEN girdi
    assert m.phase is MissionPhase.ACTIVE       # yön ters ⇒ "geçmedim"
    m.update(_kuzey_m(0.5), 0.0, 5.0)           # yedek devreye girer
    assert m.phase is MissionPhase.DWELL
    assert m.zaman_asimiyla_varilan == 1


# ─────────────────────────── FLY-BY (19.08.2026) ───────────────────────────
# 🔴 Sabit öteleme (`hedef_oteleme_m`) DENENDİ ve KALDIRILDI: 2,03 m → red2
# +1,41 m (eşik 1,53 ⇒ 12 cm eksik), 2,5 m → DAHA KÖTÜ (geçit 0/8). Mesafeyi
# mesafeyle yenmek kırılgan. Fly-by ayar gerektirmiyor: kapı durulacak değil
# GEÇİLECEK noktadır.

def _wp(lat, lon, parkur=1):
    return Waypoint(lat=lat, lon=lon, parkur=parkur)


def _ikinci_noktaya_getir(mm):
    """Rotanın ORTA noktasına kadar sür (muafiyetler uçlarda uygulanır)."""
    for i in range(400):
        mm.update(39.99995 + i * 2e-6, 31.0, i * 0.1)
        if mm.current_index == 1:
            return True
    return False


def _nisan_mesafesi(mm, la):
    e, n = mm.update(la, 31.0, 500.0)
    return math.hypot(e, n)


def test_FLYBY_PARKUR3te_UYGULANMAZ() -> None:
    """P3 (kamikaze) noktaya VARMAYI ister — içinden geçmeyi değil."""
    rota = [_wp(40.0, 31.0, 3), _wp(40.0001, 31.0, 3), _wp(40.0002, 31.0, 3)]
    mm = MissionManager(rota, MissionManagerConfig(gecis_zorunlu=True))
    mm.start()
    assert _ikinci_noktaya_getir(mm), "test kurulumu"
    assert _nisan_mesafesi(mm, 40.0001 - 9e-6) < 3.0, (
        "PARKUR 3'te fly-by uygulandı — araç kamikaze hedefinin ötesine sürülür"
    )


def test_FLYBY_PARKUR_DEGISEN_noktada_UYGULANMAZ() -> None:
    """P2 → P3 devir noktası bir GEÇİT değil.

    Orada ileri gitmek aracı **P3'ün büyük dubasının** görüş/menzil
    penceresinden (kamera 69°, LiDAR ~8 m) çıkarabilir.
    """
    rota = [_wp(40.0, 31.0, 2), _wp(40.0001, 31.0, 2), _wp(40.0002, 31.0, 3)]
    mm = MissionManager(rota, MissionManagerConfig(gecis_zorunlu=True))
    mm.start()
    assert _ikinci_noktaya_getir(mm), "test kurulumu"
    assert _nisan_mesafesi(mm, 40.0001 - 9e-6) < 3.0, (
        "parkur DEĞİŞEN noktada fly-by uygulandı — P3'ün büyük dubası kaçabilir"
    )


def test_FLYBY_SON_NOKTADA_UYGULANMAZ() -> None:
    """Görevin son noktasının ötesinde sayılacak bir şey yok."""
    rota = [_wp(40.0, 31.0), _wp(40.0001, 31.0)]
    mm = MissionManager(rota, MissionManagerConfig(gecis_zorunlu=True))
    mm.start()
    assert _ikinci_noktaya_getir(mm), "test kurulumu"
    assert _nisan_mesafesi(mm, 40.0001 - 9e-6) < 3.0, (
        "SON noktada fly-by uygulandı — fazladan yol, kazanç yok"
    )


def test_FLYBY_cember_icinde_duzlem_asilmadiysa_nisan_SONRAKI_nokta() -> None:
    """🔑 Kapı durulacak değil GEÇİLECEK noktadır — ayarlanacak sayı yok.

    Sabit ötelemenin çıkmazı ölçüldü: 2,03 m → red2 +1,41 m (eşik 1,53 ⇒
    12 cm eksik), 2,5 m → daha kötü (0/8 geçit). Belirleyici olan öteleme
    değil aracın DURDUĞU yer: çemberin içine girince nişan ayağının dibinde
    kalıyor ve araç kapı ortasında ölüyor.
    """
    rota = [_wp(40.0, 31.0), _wp(40.0001, 31.0), _wp(40.0002, 31.0)]
    mm = MissionManager(rota, MissionManagerConfig(gecis_zorunlu=True))
    mm.start()
    # 2. noktanın ~1 m güneyinde: çember (2 m) İÇİNDE, düzlem AŞILMADI.
    for i in range(400):
        la = 39.99995 + i * 2e-6
        mm.update(la, 31.0, i * 0.1)
        if mm.current_index == 1:
            break
    assert mm.current_index == 1, "test kurulumu: ikinci noktaya gelinemedi"
    la_ic = 40.0001 - 9e-6                      # ~1 m güney → çember içi
    nisan = mm.update(la_ic, 31.0, 500.0)
    hedef_mesafe = math.hypot(*nisan)
    # Nişan artık SONRAKİ nokta (≈11,1 m ileride), kendi noktası (~1 m) değil.
    assert hedef_mesafe > 5.0, (
        f"nişan hâlâ kapının kendisinde ({hedef_mesafe:.2f} m) — araç kapıda "
        "durur, düzlemi aşamaz"
    )


def test_FLYBY_gecis_zorunlu_KAPALIYKEN_eski_davranis() -> None:
    """`gecis_zorunlu` kapalıyken fly-by devreye girmez (geriye uyum)."""
    rota = [_wp(40.0, 31.0), _wp(40.0001, 31.0), _wp(40.0002, 31.0)]
    mm = MissionManager(rota, MissionManagerConfig(gecis_zorunlu=False))
    mm.start()
    for i in range(400):
        la = 39.99995 + i * 2e-6
        mm.update(la, 31.0, i * 0.1)
        if mm.current_index == 1:
            break
    la_ic = 40.0001 - 9e-6
    nisan = mm.update(la_ic, 31.0, 500.0)
    assert math.hypot(*nisan) < 3.0, (
        "gecis_zorunlu KAPALIYKEN nişan sonraki noktaya kaydı — eski davranış bozuldu"
    )


def test_FLYBY_DUZLEM_ASILINCA_nisan_geri_doner() -> None:
    """Düzlem aşıldıysa fly-by'a gerek yok; nişan kendi noktasına döner."""
    rota = [_wp(40.0, 31.0), _wp(40.0001, 31.0), _wp(40.0002, 31.0)]
    mm = MissionManager(rota, MissionManagerConfig(gecis_zorunlu=True))
    mm.start()
    for i in range(400):
        la = 39.99995 + i * 2e-6
        mm.update(la, 31.0, i * 0.1)
        if mm.current_index == 1:
            break
    la_asti = 40.0001 + 9e-6                    # düzlemin ÖTESİ, çember içi
    nisan = mm.update(la_asti, 31.0, 500.0)
    assert math.hypot(*nisan) < 3.0, (
        "düzlem aşıldığı hâlde nişan sonraki noktaya kaydı"
    )
