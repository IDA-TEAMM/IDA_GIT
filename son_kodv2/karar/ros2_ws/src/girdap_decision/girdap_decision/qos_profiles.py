"""
Girdap İDA — Ortak QoS profilleri (Layer 2).

Tek kaynak: tüm node'lar mavros sensör topic'lerine bu profille abone/publish
olur. Böylece QoS uyumsuzluğu (RELIABLE↔BEST_EFFORT) sınıfı hatalar tek yerde
engellenir.

mavros konvansiyonu:
    - Sensör verileri (imu, global_position, velocity_body): BEST_EFFORT +
      KEEP_LAST. rclpy `qos_profile_sensor_data` varsayılan derinliği 5'tir;
      biz kısa tıkanmalarda örnek kaybını azaltmak için 10 kullanırız.
    - Durum topic'leri (/mavros/state): RELIABLE (bu profil değil, varsayılan
      QoS yeterli) — state kaçırılmamalı.
"""

from __future__ import annotations

from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)

# mavros sensör topic'leriyle uyumlu derinlik.
SENSOR_QOS_DEPTH = 10


def sensor_data_qos(depth: int = SENSOR_QOS_DEPTH) -> QoSProfile:
    """mavros sensör topic'leri için SensorData profili (BEST_EFFORT, KEEP_LAST).

    ``rclpy.qos.qos_profile_sensor_data`` ile aynı güvenilirlik/dayanıklılık
    politikası; yalnız derinlik parametrik (varsayılan 10).
    """
    return QoSProfile(
        reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.VOLATILE,
        history=HistoryPolicy.KEEP_LAST,
        depth=depth,
    )


def latched_qos(depth: int = 1) -> QoSProfile:
    """Latch'li (TRANSIENT_LOCAL) topic'ler için — geç abone son mesajı alır.

    mavros görev plugin'i ``/mavros/mission/waypoints``'i latch'li (RELIABLE +
    TRANSIENT_LOCAL) yayınlar; FC görevini kaçırmamak için abone bu dayanıklılığı
    eşlemelidir (VOLATILE ile eşleşmezse hiç mesaj gelmez).
    """
    return QoSProfile(
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
        history=HistoryPolicy.KEEP_LAST,
        depth=depth,
    )
