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
