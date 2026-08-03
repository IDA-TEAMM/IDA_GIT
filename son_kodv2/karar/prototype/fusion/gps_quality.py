"""
Girdap İDA — GPS fix kalitesi → ölçüm sigma'sı eşlemesi (ROS-bağımsız).

Sorun:
    fusion_node GPS'i tek bir sabit sigma ile (gps_sigma_xy) kabul ediyordu.
    Gerçekte H-RTK F9P'nin doğruluğu fix tipine göre iki KAT BÜYÜKLÜK
    değişir: RTK-fixed ~2 cm, SBAS ~0.5 m, tek-nokta (autonomous) ~2.5 m.
    Tek sigma seçmek ya RTK'nın hassasiyetini çöpe atar (pesimistik) ya da
    RTK kaybında smoother'ı yanlış ölçüme aşırı güvendirir (iyimser) —
    ikincisi Deniz Durumu-2'de metrelerce sapma demek.

Çözüm:
    NavSatFix.status.status alanı (sensor_msgs) fix tipini zaten taşıyor.
    Bu modül o değeri ölçüm sigma'sına çevirir; None dönerse ölçüm KABUL
    EDİLMEZ (smoother'a hiç verilmez).

ROS-bağımsızlık:
    sensor_msgs/NavSatStatus sabitleri mesaj tanımının değişmez parçasıdır
    (REP-0, ROS 1'den beri aynı) → düz int olarak yeniden tanımlanır ki
    çekirdek rclpy'siz pytest'te koşsun. fusion_node bu sabitleri gerçek
    mesaj sınıfına karşı test eder (test_fusion_node.py).
"""

from __future__ import annotations

from typing import Dict, Optional

# sensor_msgs/NavSatStatus sabitleri (mesaj tanımıyla birebir).
STATUS_NO_FIX: int = -1      # fix yok / geçersiz
STATUS_FIX: int = 0          # tek nokta (autonomous) çözüm
STATUS_SBAS_FIX: int = 1     # uydu-temelli düzeltme (WAAS/EGNOS)
STATUS_GBAS_FIX: int = 2     # yer-temelli düzeltme = RTK (Holybro H-RTK F9P)

# Fix tipine göre (x, y) ölçüm sigma'sı [m]. Saha kalibrasyonunda tune edilir;
# hardware.yaml `fusion.gps_sigma_by_status` bloğu override eder.
DEFAULT_SIGMA_BY_STATUS: Dict[int, float] = {
    STATUS_GBAS_FIX: 0.05,   # RTK fixed — H-RTK F9P datasheet ~2 cm, pesimistik
    STATUS_SBAS_FIX: 0.50,   # SBAS düzeltmeli
    STATUS_FIX: 2.50,        # tek nokta — kabul edilir ama neredeyse etkisiz
}


def sigma_for_status(
    status: int,
    table: Optional[Dict[int, float]] = None,
) -> Optional[float]:
    """
    NavSatFix.status.status → ölçüm sigma'sı [m].

    Dönüş None ise ölçüm REDDEDİLİR — çağıran taraf add_gps'i ÇAĞIRMAMALI.
    Bilinmeyen (tabloda olmayan) pozitif bir status, tablodaki EN KÖTÜMSER
    sigma ile kabul edilir: yeni/özel bir fix tipini sessizce RTK sanmaktansa
    zayıf ağırlıkla almak güvenli taraftır.
    """
    if status <= STATUS_NO_FIX:
        return None
    tbl = table if table is not None else DEFAULT_SIGMA_BY_STATUS
    if not tbl:
        return None
    sigma = tbl.get(status)
    if sigma is None:
        sigma = max(tbl.values())
    return sigma if sigma > 0.0 else None


def status_name(status: int) -> str:
    """Log satırları için okunabilir fix adı."""
    return {
        STATUS_NO_FIX: "NO_FIX",
        STATUS_FIX: "FIX",
        STATUS_SBAS_FIX: "SBAS_FIX",
        STATUS_GBAS_FIX: "GBAS_FIX(RTK)",
    }.get(status, f"BİLİNMEYEN({status})")
