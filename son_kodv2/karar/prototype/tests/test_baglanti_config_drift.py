"""
Girdap İDA — uçuş kontrolcüsü BAĞLANTI ayarlarının drift kapıları (ROS'SUZ).

Neden bu dosya var (2026-08-12, §0.52 + §0.56):
    Hat aylarca `57600`'de yapılandırılmıştı, uçuş kontrolcüsü ise
    `SERIAL2_BAUD`=921 (921600) konuşuyordu. Sonuç sessizdi: MAVROS
    "bağlanamadı" demiyor, hattan çöp akıyor, `mavros_router` sahte uzak
    adresler sayıyor, füzyon poz üretmiyor, görev BOOT'ta çakılı kalıyor —
    ve hiçbir test kırmızıya dönmüyordu.

    `planning.*` anahtarlarının drift kapısı vardı (`test_planning_config_
    drift.py`) ama **bağlantı ayarlarının hiç nöbetçisi yoktu**; yanlış baud
    tam bu boşlukta yaşadı. Bu dosya o boşluğu kapatır.

Bağlanan dört kopya:
    config/hardware.yaml          fcu_url                (TEK GERÇEK KAYNAK)
    launch/hardware.launch.py     _HW_DEFAULTS["fcu_url"] (yaml okunamazsa)
    scripts/girdap-saat.service   --port / --baud         (ayrı süreç, aynı hat)
    testler/fc_teshis.sh          kullanım satırı + parametre okuma yolu

`hardware.yaml` okunamadığında launch kendi varsayılanına düşer; ikisi
ayrışırsa `ros2 launch` sessizce BAŞKA bir hatta bağlanır. Saat servisi ise
aynı fiziksel porta ayrı bir süreçle bağlanır — hızı ayrışırsa uçuş
kontrolcüsünden saat alınamaz (12.08'de tam bu oldu).

launch/launch_ros/rclpy GEREKTİRMEZ (dosya `ast` ile okunur) — ROS'suz
çekirdek job'ında da koşar.
"""

from __future__ import annotations

import ast
import re
from pathlib import Path

import pytest
import yaml

_KARAR_DIR = Path(__file__).resolve().parents[2]
_PKG_DIR = _KARAR_DIR / "ros2_ws" / "src" / "girdap_decision"
_HARDWARE_YAML = _PKG_DIR / "config" / "hardware.yaml"
_LAUNCH_PY = _PKG_DIR / "launch" / "hardware.launch.py"
_SAAT_SERVICE = _KARAR_DIR / "scripts" / "girdap-saat.service"
_UDEV_RULES = _KARAR_DIR / "scripts" / "99-girdap-fc.rules"
_TESHIS_BETIGI = _KARAR_DIR.parent / "testler" / "fc_teshis.sh"

# ROS 1 API'si. MAVROS 2 (Humble) bu servisi HİÇ açmaz — 13.08.2026'da canlı
# ölçüldü: `param` eklentisi yüklüyken de yalnız `param/pull` + `param/set` var.
_OLMAYAN_PARAM_SERVISI = "/mavros/param/get"
# MAVROS 2'de parametre okumanın çalışan tek yolu (ölçüldü: "Integer value is: 921").
_CALISAN_PARAM_YOLU = "ros2 param get /mavros/param"

# Kalıcı aygıt adı: udev kuralının ürettiği symlink. Ham `ttyUSB*`/`ttyACM*`
# adları takılma sırasına göre yer değiştirir (Jetson'da iki FTDI var).
_KALICI_AYGIT = "/dev/pixhawk"

# `serial:///dev/pixhawk:921600` → ("/dev/pixhawk", 921600)
_FCU_URL_DESENI = re.compile(r"^serial://(?P<aygit>[^:]+):(?P<baud>\d+)$")


def _fcu_url_coz(url: str) -> tuple[str, int]:
    """`fcu_url`'i (aygıt, baud) ikilisine ayır."""
    eslesme = _FCU_URL_DESENI.match(url)
    assert eslesme is not None, (
        f"fcu_url beklenen biçimde değil: {url!r} "
        "(beklenen: serial:///dev/<aygıt>:<baud>)"
    )
    return eslesme.group("aygit"), int(eslesme.group("baud"))


def _yaml_fcu_url() -> str:
    veri = yaml.safe_load(_HARDWARE_YAML.read_text(encoding="utf-8"))
    assert "fcu_url" in veri, "hardware.yaml'da fcu_url anahtarı YOK"
    return str(veri["fcu_url"])


def _launch_hw_defaults() -> dict:
    """`_HW_DEFAULTS` sözlüğünü `ast` ile oku (launch import ETMEDEN)."""
    agac = ast.parse(_LAUNCH_PY.read_text(encoding="utf-8"))
    for dugum in ast.walk(agac):
        if not isinstance(dugum, ast.Assign):
            continue
        for hedef in dugum.targets:
            if isinstance(hedef, ast.Name) and hedef.id == "_HW_DEFAULTS":
                return ast.literal_eval(dugum.value)
    pytest.fail("hardware.launch.py içinde _HW_DEFAULTS bulunamadı")


def _saat_servisi_argumanlari() -> tuple[str, int]:
    """girdap-saat.service ExecStart satırından (port, baud) çıkar."""
    metin = _SAAT_SERVICE.read_text(encoding="utf-8")
    exec_satirlari = [
        s for s in metin.splitlines() if s.strip().startswith("ExecStart=")
    ]
    assert exec_satirlari, "girdap-saat.service'te ExecStart satırı YOK"
    satir = exec_satirlari[-1]

    port = re.search(r"--port\s+(\S+)", satir)
    baud = re.search(r"--baud\s+(\d+)", satir)
    assert port is not None, f"ExecStart'ta --port yok: {satir}"
    assert baud is not None, f"ExecStart'ta --baud yok: {satir}"
    return port.group(1), int(baud.group(1))


# --------------------------------------------------------- drift kapıları


def test_launch_varsayilani_hardware_yaml_ile_ayni() -> None:
    """`hardware.yaml` okunamazsa launch AYNI hatta bağlanmalı.

    Ayrışırsa `ros2 launch` sessizce başka bir aygıta/hıza düşer — 12.08'e
    kadar varsayılan `serial:///dev/ttyACM0:57600` idi: bu teknede ttyACM0
    HİÇ oluşmuyor (Pixhawk USB-C soketi arızalı, F-M.9) ve hız da yanlıştı.
    """
    assert _launch_hw_defaults()["fcu_url"] == _yaml_fcu_url()


def test_fcu_url_kalici_udev_adini_kullanir() -> None:
    """Ham `ttyUSB*`/`ttyACM*` adı YASAK — takılma sırasına göre kayar.

    Jetson'da iki FTDI dönüştürücü var; hangisinin `ttyUSB0` olacağı
    açılışa göre değişir. udev kuralı seri numarasına göre sabit symlink
    üretir; bağlantı ayarları onu kullanmak zorundadır.
    """
    for kaynak, url in (
        ("hardware.yaml", _yaml_fcu_url()),
        ("hardware.launch.py", _launch_hw_defaults()["fcu_url"]),
    ):
        aygit, _ = _fcu_url_coz(url)
        assert aygit == _KALICI_AYGIT, (
            f"{kaynak}: fcu_url ham aygıt adı kullanıyor ({aygit}) — "
            f"kalıcı udev adı {_KALICI_AYGIT} olmalı"
        )


def test_udev_kurali_kalici_adi_gercekten_uretiyor() -> None:
    """`fcu_url`'in dayandığı symlink'i üreten kural repoda DURMALI.

    Kural 10.08'de SSD arızasında kaybolmuştu ve hiçbir repoda yoktu;
    kaybolursa `/dev/pixhawk` hiç oluşmaz ve bağlantı ayarı boşa düşer.
    """
    metin = _UDEV_RULES.read_text(encoding="utf-8")
    symlink_adi = _KALICI_AYGIT.removeprefix("/dev/")
    assert f'SYMLINK+="{symlink_adi}"' in metin, (
        f"99-girdap-fc.rules {symlink_adi} symlink'ini üretmiyor"
    )


def test_saat_servisi_ayni_port_ve_hizi_kullanir() -> None:
    """Saat servisi aynı fiziksel hatta bağlanır — port ve hız AYNI olmalı.

    12.08 ölçümü: `fcu_url` 921600'e çekilmişken saat servisi 57600'de
    kalırsa uçuş kontrolcüsünden saat alınamaz (`girdap-saat` 45 saniye
    deneyip "GPS fix yok?" diye çıkar) ve Jetson'ın saati 1970'te kalır —
    bant klasör adları ve teslim dosyalarının zaman etiketleri bozulur.
    """
    aygit, baud = _fcu_url_coz(_yaml_fcu_url())
    saat_port, saat_baud = _saat_servisi_argumanlari()

    assert saat_port == aygit, (
        f"girdap-saat.service portu ({saat_port}) fcu_url'den ({aygit}) FARKLI"
    )
    assert saat_baud == baud, (
        f"girdap-saat.service hızı ({saat_baud}) fcu_url'den ({baud}) FARKLI "
        "— uçuş kontrolcüsünden saat alınamaz"
    )


def test_fcu_url_hizi_ucus_kontrolcusunun_serial2_baudu_ile_ayni() -> None:
    """Hat hızı, uçuş kontrolcüsünde ÖLÇÜLEN değerde donduruldu.

    2026-08-12'de uçuş kontrolcüsünden okundu: `SERIAL2_BAUD` = 921
    (= 921600). Bu test o ölçümü sabitler; biri hızı değiştirirse ya uçuş
    kontrolcüsünde de değiştirmiş ve bu sayıyı güncellemiş olmalı, ya da
    §0.52'nin arızasını geri getiriyordur.
    """
    _, baud = _fcu_url_coz(_yaml_fcu_url())
    assert baud == 921600, (
        f"fcu_url hızı {baud} — ölçülen SERIAL2_BAUD 921600. Uçuş "
        "kontrolcüsünde de değiştiyse bu testi ölçümle birlikte güncelle."
    )


# ------------------------------------------------- teşhis betiği nöbetçileri


def _teshis_betigi_calisan_satirlari() -> str:
    """`fc_teshis.sh`'ın YORUM OLMAYAN satırları.

    Yorumlar ayıklanır çünkü betiğin açıklama bloğu, artık kullanılmayan
    servisin adını bilerek anıyor (neden bırakıldığını anlatmak için) —
    düz metin araması o açıklamayı gerçek bir çağrı sanardı.
    """
    satirlar = _TESHIS_BETIGI.read_text(encoding="utf-8").splitlines()
    return "\n".join(s for s in satirlar if not s.lstrip().startswith("#"))


def test_teshis_betigi_olmayan_mavros_servisini_cagirmaz() -> None:
    """Teşhis betiği MAVROS 2'de var olmayan servisi çağırmamalı.

    13.08.2026'da ölçüldü: betik 45 parametrenin tamamı için
    `ros2 service call /mavros/param/get` çağırıyordu; o servis MAVROS 2'de
    yok, çağrı zaman aşımına uğrayıp BOŞ dönüyordu ve betik `SERIAL2_BAUD : `
    diye basıp geçiyordu. Yani teşhis aracının kendisi sahte-yeşildi: parametre
    okunamadığı hâlde okunmuş gibi rapor ediliyordu. §0.41'in "kalibrasyon
    geçersiz" teşhisi tam bu betiğe dayanıyordu.
    """
    calisan = _teshis_betigi_calisan_satirlari()

    assert _OLMAYAN_PARAM_SERVISI not in calisan, (
        f"fc_teshis.sh {_OLMAYAN_PARAM_SERVISI} çağırıyor — MAVROS 2'de bu "
        "servis YOK, çağrı sessizce boş döner. Doğru yol: "
        f"{_CALISAN_PARAM_YOLU} <PARAM>"
    )
    assert _CALISAN_PARAM_YOLU in calisan, (
        f"fc_teshis.sh parametreleri {_CALISAN_PARAM_YOLU} ile okumuyor — "
        "parametre dökümü bölümü kaybolmuş olabilir"
    )


def test_teshis_betigi_bos_okumayi_deger_gibi_basmaz() -> None:
    """Okunamayan parametre, değeri boş olan parametreden AYIRT EDİLMELİ.

    Sahte-yeşilin asıl mekanizması buydu: boş çıktı doğrudan loglanınca
    "SERIAL2_BAUD : " satırı, "parametre 0/tanımsız" diye okunuyordu.
    """
    calisan = _teshis_betigi_calisan_satirlari()
    assert "OKUNAMADI" in calisan, (
        "fc_teshis.sh boş parametre okumasını ayırt etmiyor — boş çıktı "
        "değer gibi basılırsa teşhis yanlış yönlendirir"
    )


def test_teshis_betigi_guncel_hat_adresini_yazar() -> None:
    """Betiğin kullanım satırı `hardware.yaml` ile AYNI hattı göstermeli.

    Kullanım satırı 12.08'e kadar `serial:///dev/ttyUSB0:57600` diyordu:
    hem ham aygıt adı (açılışa göre kayar) hem de §0.52'de çürütülen hız.
    Betiği o satıra bakarak elle koşan biri yanlış hatta bağlanır.
    """
    metin = _TESHIS_BETIGI.read_text(encoding="utf-8")
    url = _yaml_fcu_url()
    assert url in metin, (
        f"fc_teshis.sh kullanım satırı {url} yazmıyor — hardware.yaml ile "
        "ayrışmış, betiği elle koşan yanlış hatta bağlanır"
    )
