import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    namespace = LaunchConfiguration('namespace')
    twist_mux_params = os.path.join(
        get_package_share_directory('amr_sweeper_twist_mux'),
        'config',
        'twist_mux.yaml',
    )

    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value='amr_sweeper'),
        Node(
            package='twist_mux',
            executable='twist_mux',
            namespace=namespace,
            output='screen',
            parameters=[twist_mux_params],
            remappings=[('cmd_vel_out', 'cmd_vel_wheels')],
        ),
    ])
