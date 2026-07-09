import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

def generate_launch_description():
    pkg_path = get_package_share_directory('Girdap')
    urdf_file = os.path.join(pkg_path, 'urdf', 'Girdap.urdf')
    
    with open(urdf_file, 'r') as f:
        robot_description = f.read()

    return LaunchDescription([
        # Robot state publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description}]
        ),
        # Spawn model into Gazebo
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            name='spawn_girdap',
            output='screen',
            arguments=[
                '-entity', 'Girdap',
                '-file', urdf_file,
                '-x', '0', '-y', '0', '-z', '0.5'
            ]
        ),
    ])
