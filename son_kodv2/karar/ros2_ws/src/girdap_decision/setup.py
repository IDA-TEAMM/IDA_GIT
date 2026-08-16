"""
ament_python build için setup. Layer 2 ROS 2 Humble paketi.

Prototip kodu (prototype/) bu pakete bağımlıdır; çalıştırmadan önce repo
kökü PYTHONPATH'e eklenmeli (README.md'ye bakınız).
"""

import os
from glob import glob

from setuptools import find_packages, setup

PACKAGE_NAME = "girdap_decision"

setup(
    name=PACKAGE_NAME,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            ["resource/" + PACKAGE_NAME],
        ),
        ("share/" + PACKAGE_NAME, ["package.xml"]),
        (
            os.path.join("share", PACKAGE_NAME, "launch"),
            glob("launch/*.launch.py"),
        ),
        (
            os.path.join("share", PACKAGE_NAME, "config"),
            glob("config/*.yaml"),
        ),
        # Eğitilmiş YOLO duba lokalizatörü (best.pt) — share'e kurulur ki
        # launch get_package_share_directory ile makineden bağımsız bulsun
        # (host ~/ros2_ws ↔ container /root/ros2_ws mutlak-yol farkını yener).
        (
            os.path.join("share", PACKAGE_NAME, "models"),
            glob("models/*.pt"),
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Girdap Takımı",
    maintainer_email="unszside@gmail.com",
    description="Girdap İDA — karar/planlama paketi (iSAM2 + RRT* + MPPI + FSM)",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "fusion_node = girdap_decision.fusion_node:main",
            "planning_node = girdap_decision.planning_node:main",
            "fsm_node = girdap_decision.fsm_node:main",
            "telemetry_node = girdap_decision.telemetry_node:main",
            "mavros_bridge_node = girdap_decision.mavros_bridge_node:main",
            "local_map_node = girdap_decision.local_map_node:main",
            "lidar_kayit_node = girdap_decision.lidar_kayit_node:main",
            "mission_manager_node = girdap_decision.mission_manager_node:main",
            "perception_lidar_node = girdap_decision.perception_lidar_node:main",
            "perception_fusion_node = girdap_decision.perception_fusion_node:main",
            "mock_sensors = girdap_decision.mock_sensors:main",
            "yarisma_simulasyonu = girdap_decision.yarisma_simulasyonu:main",
        ],
    },
)
