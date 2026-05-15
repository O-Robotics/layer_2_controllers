import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    namespace = LaunchConfiguration('namespace')
    joy_dev = LaunchConfiguration('joy_dev')
    joy_params = os.path.join(
        get_package_share_directory('amr_sweeper_joystick'),
        'config',
        'joystick.yaml',
    )

    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value='amr_sweeper'),
        DeclareLaunchArgument('joy_dev', default_value='/dev/input/js0'),
        Node(
            package='joy',
            executable='joy_node',
            namespace=namespace,
            output='screen',
            parameters=[joy_params, {'device_id': 0, 'dev': joy_dev}],
        ),
        Node(
            package='teleop_twist_joy',
            executable='teleop_node',
            name='teleop_twist_joy_wheels',
            namespace=namespace,
            output='screen',
            parameters=[joy_params],
            remappings=[('cmd_vel', 'cmd_vel_joy_wheels')],
        ),
        Node(
            package='teleop_twist_joy',
            executable='teleop_node',
            name='teleop_twist_joy_tools',
            namespace=namespace,
            output='screen',
            parameters=[joy_params],
            remappings=[('cmd_vel', 'cmd_vel_joy_tools')],
        ),
    ])
