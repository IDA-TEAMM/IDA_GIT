"""
Girdap İDA — karar yığını launch dosyası.

Tüm 4 node'u tek komutla başlatır:
    ros2 launch girdap_decision decision.launch.py

mavros'u ayrı başlatmayı unutma:
    ros2 launch mavros apm.launch fcu_url:=/dev/ttyUSB0:921600
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
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
    # Sahada sensörleri mavros besler; entegrasyon testinde sahte sensör.
    # ÜRETİM GÜVENLİĞİ: varsayılan false — mock yalnız `use_mock:=true` ile.
    mock_arg = DeclareLaunchArgument(
        "use_mock",
        default_value="false",
        description="true → mock_sensors başlat (mavros yerine sahte sensör)",
    )

    params = LaunchConfiguration("params_file")
    use_mock = LaunchConfiguration("use_mock")

    common = {"parameters": [params], "output": "screen"}

    return LaunchDescription(
        [
            params_arg,
            mock_arg,
            Node(
                package="girdap_decision",
                executable="mock_sensors",
                name="mock_sensors",
                condition=IfCondition(use_mock),
                **common,
            ),
            Node(
                package="girdap_decision",
                executable="fusion_node",
                name="fusion_node",
                **common,
            ),
            Node(
                package="girdap_decision",
                executable="planning_node",
                name="planning_node",
                **common,
            ),
            Node(
                package="girdap_decision",
                executable="fsm_node",
                name="fsm_node",
                **common,
            ),
            Node(
                package="girdap_decision",
                executable="mavros_bridge_node",
                name="mavros_bridge",
                **common,
            ),
            Node(
                package="girdap_decision",
                executable="telemetry_node",
                name="telemetry_node",
                **common,
            ),
            Node(
                package="girdap_decision",
                executable="local_map_node",
                name="local_map_node",
                **common,
            ),
        ]
    )
