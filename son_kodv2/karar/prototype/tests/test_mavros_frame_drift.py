"""
Girdap İDA — MAVROS `setpoint_velocity` çerçevesi NÖBETÇİSİ (2026-08-07).

🔴 NEYİ KORUYOR: `planning_node` **gövde** çerçevesinde surge basıyor
(`twist.linear.x` = ileri hız, işaretli; geri için negatif; `linear.y` hiç
doldurulmuyor). MAVROS'un `setpoint_velocity` eklentisi ise varsayılan
`mav_frame: LOCAL_NED` ile bunu **dünya** çerçevesi sanıp gönderiyor: ROS
ENU → MAVLink NED dönüşümünde `linear.x` (ENU **doğu**) NED'in **vy** alanına
düşüyor, **vx sıfır kalıyor.** ArduPilot Rover işaretli hızı `vx`'ten okuduğu
için işaret kayboluyor.

**Sonucu:** GERİ komutu araçta İLERİ olarak uygulanıyordu. Gerçek donanımda
ölçüldü (07.08, nötr PWM 1487):
    linear.x = +1.053 → SERVO 1930 / 1932   (ileri)
    linear.x = -1.059 → SERVO 1940 / 1938   (yine İLERİ!)
Kaptan motorların fiziksel olarak tam ileri bastığını da doğruladı.

`config/mavros_overrides.yaml` (`mav_frame: BODY_NED`) bunu düzeltiyor;
aynı donanımda düzeltmeden sonra:
    linear.x = -1.072 → SERVO 1058 / 1060, min 1000 (TAM GERİ), fark 2 PWM

⚠ NEDEN NÖBETÇİ GEREKLİ: hata **sessiz**. Araç komut alıyor, motorlar dönüyor,
hiçbir log satırı hata vermiyor — sadece yön ters. MPPI hedef burnun
90°'sinden geniş açıdaysa geri gitmeyi seçtiği için (ölçüldü: 95°'de ileri
bileşke −1,190 N) bu, aracın hedeften UZAKLAŞARAK tam gaz gitmesi demek.
Override silinir ya da apm_config.yaml'dan ÖNCE'ye alınırsa hata geri gelir
ve kimse fark etmez.

ROS gerektirmez — launch dosyası düz metin, config düz YAML olarak okunur.
"""

from __future__ import annotations

from pathlib import Path

import pytest
import yaml

_PKG = (
    Path(__file__).resolve().parents[2]
    / "ros2_ws" / "src" / "girdap_decision"
)
_LAUNCH = _PKG / "launch" / "hardware.launch.py"
_OVERRIDE = _PKG / "config" / "mavros_overrides.yaml"


def test_override_dosyasi_BODY_NED_veriyor() -> None:
    """Override dosyası var ve `setpoint_velocity` çerçevesini BODY_NED yapıyor."""
    assert _OVERRIDE.exists(), f"{_OVERRIDE.name} YOK — geri gidiş sessizce bozulur"
    with open(_OVERRIDE, "r", encoding="utf-8") as f:
        cfg = yaml.safe_load(f)

    anahtarlar = [k for k in cfg if "setpoint_velocity" in k]
    assert anahtarlar, "override'da setpoint_velocity bloğu yok"
    blok = cfg[anahtarlar[0]]["ros__parameters"]
    assert blok["mav_frame"] == "BODY_NED", (
        f"mav_frame {blok['mav_frame']!r} — LOCAL_NED'de GERİ komutu İLERİ olur"
    )


def test_launch_override_i_apm_config_ten_SONRA_yukluyor() -> None:
    """🔴 SIRA KRİTİK: ROS'ta sonraki parametre dosyası öncekini ezer.

    Override `apm_config.yaml`'dan ÖNCE gelirse sessizce ezilir ve
    `mav_frame` LOCAL_NED'e geri döner — hata da geri gelir.
    """
    metin = _LAUNCH.read_text(encoding="utf-8")
    assert "mavros_overrides.yaml" in metin, (
        "hardware.launch.py override'ı yüklemiyor — dosya tek başına etkisiz"
    )
    i_apm = metin.index("apm_config.yaml")
    i_ovr = metin.index("mavros_overrides.yaml")
    assert i_ovr > i_apm, (
        "mavros_overrides.yaml apm_config.yaml'dan ÖNCE yükleniyor → ezilir"
    )


def test_planning_node_hala_GOVDE_surge_basiyor() -> None:
    """Nöbetçinin dayanağı: `linear.y` doldurulmuyorsa çerçeve gövde olmalı.

    Biri ileride `linear.y`'yi de doldurup dünya çerçevesine geçerse bu test
    kırılır ve override'ın gözden geçirilmesi gerektiğini hatırlatır — iki
    tarafın aynı anda değişmesi şart, yoksa yön yine bozulur.
    """
    kaynak = (_PKG / "girdap_decision" / "planning_node.py").read_text(encoding="utf-8")
    assert "twist.linear.x" in kaynak, "cmd_vel üretimi değişmiş — çerçeveyi gözden geçir"
    assert "twist.linear.y" not in kaynak, (
        "planning_node artık linear.y basıyor → dünya çerçevesine geçilmiş olabilir; "
        "mavros_overrides.yaml'daki BODY_NED kararı yeniden değerlendirilmeli"
    )
