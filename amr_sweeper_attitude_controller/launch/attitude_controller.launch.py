from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    namespace = LaunchConfiguration("namespace")
    params_file = LaunchConfiguration("params_file")

    return LaunchDescription(
        [
            DeclareLaunchArgument("namespace", default_value="amr_sweeper"),
            DeclareLaunchArgument(
                "params_file",
                default_value=PathJoinSubstitution(
                    [
                        FindPackageShare("amr_sweeper_attitude_controller"),
                        "config",
                        "amr_sweeper_attitude_controller.yaml",
                    ]
                ),
            ),
            Node(
                package="amr_sweeper_attitude_controller",
                executable="attitude_controller_node",
                name="amr_sweeper_attitude_controller_node",
                namespace=namespace,
                output="screen",
                parameters=[params_file],
            ),
        ]
    )
