"""
Girdap İDA — fusion_node + mock_sensors entegrasyon testi launch.

mavros olmadan, sentetik sensör verisiyle iSAM2 fusion_node'unun
uçtan uca çalıştığını doğrular.

Çalıştır:
    ros2 launch girdap_decision fusion_test.launch.py

İzlenecek topic'ler:
    /girdap/fusion/odom         — smoother çıktısı
    /girdap/groundtruth/pose    — ground truth (RMSE karşılaştırması için)
    /mavros/global_position/global — gürültülü GPS (debug)

RMSE doğrulaması (ROS-bağımsız):
    pytest prototype/tests/test_fusion_pipeline.py -v
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    pkg_share = Path(get_package_share_directory("girdap_decision"))
    default_params = pkg_share / "config" / "params.yaml"

    params_arg = DeclareLaunchArgument(
        "params_file",
        default_value=str(default_params),
        description="Tüm node parametrelerinin tek YAML dosyası",
    )
    params = LaunchConfiguration("params_file")

    return LaunchDescription(
        [
            params_arg,
            Node(
                package="girdap_decision",
                executable="mock_sensors",
                name="mock_sensors",
                output="screen",
            ),
            Node(
                package="girdap_decision",
                executable="fusion_node",
                name="fusion_node",
                parameters=[params],
                output="screen",
            ),
        ]
    )
