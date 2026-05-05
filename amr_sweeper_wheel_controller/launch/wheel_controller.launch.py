from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    namespace = LaunchConfiguration('namespace')
    cmd_vel_input_topic = LaunchConfiguration('cmd_vel_input_topic')
    cmd_vel_output_topic = LaunchConfiguration('cmd_vel_output_topic')

    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value='amr_sweeper'),
        DeclareLaunchArgument('cmd_vel_input_topic', default_value='cmd_vel_wheels'),
        DeclareLaunchArgument('cmd_vel_output_topic', default_value='diff_cont/cmd_vel_unstamped'),
        Node(
            package='topic_tools',
            executable='relay',
            namespace=namespace,
            name='wheel_command_relay',
            output='screen',
            parameters=[{
                'input_topic': cmd_vel_input_topic,
                'output_topic': cmd_vel_output_topic,
                'lazy': False,
            }],
        ),
    ])
