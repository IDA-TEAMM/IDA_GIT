"""
Girdap İDA — SAAT KURULUM ZİNCİRİ nöbetçisi (§0.61g, 13.08.2026).

🔴 NEYİ KORUYOR — ölçülmüş açılış gerçeği (§0.59a, 01:34 açılışının günlüğü):

     61. sn   `girdap-saat` koştu → zaman aşımı ("GPS fix yok?")
    171. sn   ilk KABUL EDİLEN GPS
    391. sn   EKF3 GPS'i kullanıyor

Yani açılış (seri) servisi fix'ten ~2 dakika ÖNCE koşuyor: soğuk açılışta
zaman aşımına düşmesi neredeyse KESİN. Beklemeyi uzatmak çözüm DEĞİL — o
servis `Before=girdap-karar` olduğu için her saniyesi doğrudan açılış
gecikmesidir (45 s ölçüldü, açılışın en yavaş birimi).

Sahada internet YOK (md 4.1 WiFi yasak) → saat bir daha hiç kurulmaz →
md 4.2 teslim damgaları bozuk kalır (geçersiz dosya başına 5 ceza puanı).
13.08'de geliştirme makinesinde internet vardı ve saat 29 dakika sonra tek
adımda düzeldi; o adım da sahte FAILSAFE/KILL üretti (§0.61).

Çözüm iki parçalıdır ve **ikisi birden gereklidir**:
  1. `girdap-saat.service`      — SICAK açılış (FC zaten fix'li), kısa pencere.
  2. `girdap-saat-gec.service`  — SOĞUK açılış: fix geldikten SONRA, yığın
     ayaktayken, `/mavros/time_reference`'tan. Hiçbir şeyi BEKLETMEZ.

⚠ NEDEN NÖBETÇİ: ikisi de düz metin dosya; biri silinir ya da açılış penceresi
sessizce büyütülürse hata SESSİZ döner — açılış yavaşlar ya da saat hiç
kurulmaz, ikisi de ancak yarışma günü fark edilir.

ROS GEREKTİRMEZ: servis dosyaları ve script düz metin olarak okunur.
"""

from __future__ import annotations

from pathlib import Path

_SCRIPTS = Path(__file__).resolve().parents[2] / "scripts"
_ACILIS = _SCRIPTS / "girdap-saat.service"
_GEC = _SCRIPTS / "girdap-saat-gec.service"
_KURUCU = _SCRIPTS / "girdap_saat_kur.py"

#: Açılış servisi yığını BEKLETTİĞİ için penceresi kısa kalmak ZORUNDA.
_ACILIS_AZAMI_PENCERE_S = 15.0


def _oku(p: Path) -> str:
    assert p.exists(), f"{p.name} YOK — saat zinciri kırık"
    return p.read_text(encoding="utf-8")


def _exec_satiri(metin: str) -> str:
    for satir in metin.splitlines():
        if satir.startswith("ExecStart="):
            return satir
    raise AssertionError("ExecStart satiri yok")


def _zaman_asimi(exec_satiri: str) -> float:
    parcalar = exec_satiri.split()
    i = parcalar.index("--zaman-asimi")
    return float(parcalar[i + 1])


# ------------------------------------------------------- açılış (seri) yolu


def test_acilis_servisi_yigini_KISA_bekletir() -> None:
    """`Before=girdap-karar` + uzun pencere = bedava açılış gecikmesi.

    45 s ölçüldü ve soğuk fix'i YİNE yakalamıyordu (fix 171. sn). Sıcak fix
    bir-iki saniyede gelir; pencere onu yakalayacak kadar olmalı, daha fazlası
    yalnız gecikmedir.
    """
    metin = _oku(_ACILIS)
    assert "Before=girdap-karar.service" in metin, (
        "açılış servisi artık yığını beklemiyorsa dosya adları yanlış saatle "
        "üretilir (teslim düğümleri adı BAŞLANGIÇTA kuruyor)"
    )
    pencere = _zaman_asimi(_exec_satiri(metin))
    assert pencere <= _ACILIS_AZAMI_PENCERE_S, (
        f"açılış penceresi {pencere:.0f} s — yığın bu kadar bekletiliyor. "
        f"Soğuk fix zaten yakalanamıyor (§0.59a); uzun pencere yalnız "
        f"açılışı geciktirir. Üst sınır {_ACILIS_AZAMI_PENCERE_S:.0f} s."
    )


def test_acilis_servisi_seri_yolu_kullanir() -> None:
    """MAVROS'tan ÖNCE koşar → zamanı seri porttan almak ZORUNDA."""
    exec_satiri = _exec_satiri(_oku(_ACILIS))
    assert "--kaynak ros" not in exec_satiri, (
        "açılış servisi ROS yolunu kullanamaz — MAVROS henüz yok"
    )
    assert "--port" in exec_satiri


# ----------------------------------------------------------- geç (ROS) yolu


def test_gec_servisi_VAR_ve_ros_yolunu_kullanir() -> None:
    """Soğuk açılışın tek çözümü bu servis — silinirse saat hiç kurulmaz."""
    metin = _oku(_GEC)
    exec_satiri = _exec_satiri(metin)
    assert "--kaynak ros" in exec_satiri, (
        "geç servis seri portu kullanamaz: MAVROS /dev/pixhawk'ı tekeline "
        "alır, iki süreç aynı portu AÇAMAZ"
    )


def test_gec_servisi_HICBIR_SEYI_BEKLETMEZ() -> None:
    """`Before=` yazılırsa uzun penceresi doğrudan açılış gecikmesine döner."""
    metin = _oku(_GEC)
    assert "After=girdap-karar.service" in metin, (
        "geç servis yığından SONRA koşmalı — /mavros/time_reference'ı MAVROS "
        "yayınlıyor"
    )
    bekletiyor = [
        s for s in metin.splitlines()
        if s.startswith("Before=")
    ]
    assert not bekletiyor, (
        f"geç servis bir şeyi bekletiyor: {bekletiyor} — 30 dakikalık "
        "penceresi açılış gecikmesine dönüşür"
    )


def test_gec_servisi_YENIDEN_DENER() -> None:
    """Tek işi yeniden denemek: fix ne zaman gelirse gelsin yakalamalı.

    ⚠ `Type=oneshot` ile `Restart=` systemd tarafından KABUL EDİLMEZ; bu
    yüzden `Type=simple`. Biri değişip diğeri kalırsa servis sessizce tek
    atışa döner ve soğuk açılış yine kurtarılamaz.
    """
    metin = _oku(_GEC)
    assert "Type=simple" in metin, "oneshot + Restart= systemd'de geçersiz"
    assert "Restart=on-failure" in metin, "geç servis yeniden denemiyor"
    assert "StartLimitIntervalSec=0" in metin, (
        "systemd hız sınırı denemeleri kesebilir"
    )


def test_gec_servisi_yiginla_AYNI_alan_adinda() -> None:
    """Farklı `ROS_DOMAIN_ID` → konu hiç görünmez, servis sessizce beklerdi."""
    metin = _oku(_GEC)
    assert "Environment=ROS_DOMAIN_ID=42" in metin, (
        "geç servis yığınla aynı ROS alan adında olmalı (girdap-karar: 42)"
    )


def test_gec_servisi_root_kosar() -> None:
    """`clock_settime` + `adjtimex(ADJ_STATUS)` CAP_SYS_TIME ister."""
    assert "User=root" in _oku(_GEC)


# ------------------------------------------------------------ script yüzeyi


def test_kurucu_iki_kaynagi_da_taniyor() -> None:
    metin = _oku(_KURUCU)
    assert 'choices=("seri", "ros")' in metin, "--kaynak seçenekleri değişmiş"
    assert "def gps_saati_al_ros(" in metin
    assert "def gps_saati_al(" in metin, "seri yol silinmiş — açılış yolu ölür"


def test_kurucu_ARM_kapisini_VARSAYILAN_acik_tutar() -> None:
    """Koşu ortasında saat adımlamak Dosya-2'nin zaman sütununu kaydırır.

    `--armdayken-kur` OPT-IN olmalı: `store_true` + varsayılan False. Varsayılan
    True'ya dönerse teslim dosyası ortasından bölünür ve bu SESSİZ olur.
    """
    metin = _oku(_KURUCU)
    assert '"--armdayken-kur", action="store_true"' in metin, (
        "ARM kapısı varsayılan olarak KAPALI (opt-in) kalmalı"
    )
    exec_satiri = _exec_satiri(_oku(_GEC))
    assert "--armdayken-kur" not in exec_satiri, (
        "servis ARM kapısını kendiliğinden açmamalı"
    )


def test_saati_uygula_TEK_yerde() -> None:
    """Tolerans · root denetimi · clock_settime · RTC · STA_UNSYNC SIRASI.

    İki kaynak (seri, ROS) aynı uygulama yolunu paylaşmalı; ikinci bir kopya
    çıkarsa sıra sessizce ayrışır — 11.08'de tam bu sıra yüzünden bayrak geri
    gelmişti (`sta_unsync_temizle` EN SONDA olmak zorunda).
    """
    metin = _oku(_KURUCU)
    assert metin.count("def saati_uygula(") == 1
    # Yalnız GERÇEK çağrı sayılır; açıklama metinlerinde adı geçebilir.
    assert metin.count("time.clock_settime(") == 1, "saat kurma yolu ikiye ayrılmış"
    govde = metin.split("def saati_uygula(", 1)[1]
    assert govde.index("clock_settime") < govde.index("hwclock --systohc")
    assert govde.index("hwclock --systohc") < govde.rindex("sta_unsync_temizle")


# ---------------------------------------------- yer istasyonunda görünürlük
#
# Kaptan: *"bu veriyi fix oldu ya da olmadı diye pixhawkta göreyim."*
# Saatin kurulup kurulmadığı artık Mission Planner ekranında: `SAAT-YOK` kodu
# DURUYORSA kurulmadı, YOKSA kuruldu. Kod §0.58b'nin kuralına uyar (arıza
# DURUMDUR, olay değil) → `girdap-saat-gec` fix gelince saati kurunca kendi
# kendine düşer, elle temizleme gerekmez.


def test_SAAT_YOK_kodu_var_ve_telsize_SIGIYOR() -> None:
    """MAVLink STATUSTEXT 50 karakter — taşan satır kırpılır, teşhis kaybolur."""
    from prototype.telemetry.ariza_bildirici import ARIZALAR, SAAT_YOK

    assert SAAT_YOK in ARIZALAR, "SAAT-YOK kod defterinde yok — ekranda çıkmaz"
    satir = SAAT_YOK.statustext()
    assert len(satir) <= 50, f"{len(satir)} karakter: {satir!r}"


def test_SAAT_YOK_gercek_arizalari_GOLGELEMEZ() -> None:
    """Saat yokluğu görevi durdurmaz: seviye WARNING ve öncelikte EN SONDA.

    Kod defterinin sırası "operatör önce neye baksın" demek. Saat, MPPI'yi
    durduran bir arızanın önüne geçerse gerçek arıza ekranda gizlenir.
    """
    from prototype.telemetry.ariza_bildirici import (
        ARIZALAR, SEVIYE_WARNING, SAAT_YOK,
    )

    assert SAAT_YOK.seviye == SEVIYE_WARNING
    assert ARIZALAR.index(SAAT_YOK) == len(ARIZALAR) - 1, (
        "SAAT-YOK öncelik listesinde yukarı taşınmış — gerçek arızayı gölgeler"
    )


def test_SAAT_YOK_teslim_damgasiyla_AYNI_olcute_bagli() -> None:
    """İki yer aynı gerçeği söylemeli: ekrandaki kod ↔ dosya adındaki damga.

    Teslim düğümleri `saat_guveni.saat_guvenilir_mi()` (çekirdek STA_UNSYNC)
    kullanıyor. `planning_node` ayrı bir ölçüt uydurursa (örn. "tarih 2026'dan
    büyük mü") ekran "saat var" derken dosyalar "güvenilmez" damgalanır —
    tam da kapatmaya çalıştığımız sessiz çelişki.
    """
    pn = (
        Path(__file__).resolve().parents[2]
        / "ros2_ws" / "src" / "girdap_decision" / "girdap_decision"
        / "planning_node.py"
    ).read_text(encoding="utf-8")

    assert "from prototype.telemetry.saat_guveni import saat_guvenilir_mi" in pn
    assert "self._ariza.ayarla(SAAT_YOK, aktif=not saat_ok)" in pn, (
        "SAAT-YOK durumu her turda yüklemden kurulmuyor (§0.58b: arıza kodu "
        "DURUMDUR, olay değil) — saat kurulunca kendiliğinden düşmeli"
    )


def test_kurulum_scripti_HER_IKI_servisi_de_yukler() -> None:
    """Elle çok satırlı komut 13.08'de terminalde kırıldı ve YARIM kurulum
    sessizce "tamam" gibi göründü (`cp: missing destination file operand`).

    Kurulum tek satıra indirildi; script iki servisi de yüklemeli — biri
    unutulursa soğuk açılışta saat yine kurulmaz ve bu ancak sahada anlaşılır.
    """
    kur = _SCRIPTS / "saat_servisleri_kur.sh"
    metin = _oku(kur)
    for dosya in ("girdap-saat.service", "girdap-saat-gec.service",
                  "girdap_saat_kur.py"):
        assert dosya in metin, f"kurulum script'i {dosya} yüklemiyor"
    assert "systemctl enable girdap-saat-gec.service" in metin, (
        "geç servis enable edilmezse açılışta hiç koşmaz"
    )
    # Yığını yeniden başlatmak AYRI ve bilinçli bir karardır; script yalnız
    # ÖNERİR (yorum/echo). Çalıştırılabilir bir `systemctl restart` satırı
    # olmamalı — açıklama metninde adının geçmesi serbest.
    calisan = [
        s for s in metin.splitlines()
        if s.strip().startswith(("systemctl restart", "sudo systemctl restart"))
    ]
    assert not calisan, (
        f"kurulum script'i canlı yığını KENDİLİĞİNDEN yeniden başlatıyor: {calisan}"
    )
