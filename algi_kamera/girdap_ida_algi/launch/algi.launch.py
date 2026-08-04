# -*- coding: utf-8 -*-
"""GİRDAP İDA — algı node'u launch dosyası (Jetson).

Tek node başlatır: duba_gecis_navigator (MOD dosya başındaki bayraktan).
Çökerse otomatik yeniden başlar (respawn) — OAK USB kopması, VPU takılması
gibi geçici arızalarda yarışma sırasında elle müdahale gerekmez.

Kullanım:
    ros2 launch girdap_ida_algi algi.launch.py
Systemd ile açılışta otomatik başlatma: scripts/girdap-algi.service
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        Node(
            package="girdap_ida_algi",
            executable="duba_gecis_navigator",
            name="duba_gecis_navigator",
            output="screen",
            respawn=True,          # OAK/USB geçici arızasında kendini toparla
            respawn_delay=3.0,     # s — cihazın USB'de yeniden görünme payı
        ),
    ])
