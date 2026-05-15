from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    namespace = LaunchConfiguration('namespace')
    cmd_vel_input_topic = LaunchConfiguration('cmd_vel_input_topic')
    cmd_vel_output_topic = LaunchConfiguration('cmd_vel_output_topic')
    cmd_vel_frame_id = LaunchConfiguration('cmd_vel_frame_id')

    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value='amr_sweeper'),
        DeclareLaunchArgument('cmd_vel_input_topic', default_value='cmd_vel_wheels'),
        DeclareLaunchArgument('cmd_vel_output_topic', default_value='diff_cont/cmd_vel'),
        DeclareLaunchArgument('cmd_vel_frame_id', default_value='base_footprint'),
        Node(
            package='amr_sweeper_wheel_controller',
            executable='twist_stamper_node',
            namespace=namespace,
            name='wheel_command_stamper',
            output='screen',
            parameters=[{
                'input_topic': cmd_vel_input_topic,
                'output_topic': cmd_vel_output_topic,
                'frame_id': cmd_vel_frame_id,
            }],
        ),
    ])
