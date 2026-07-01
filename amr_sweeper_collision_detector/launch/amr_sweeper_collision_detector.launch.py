from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    namespace = LaunchConfiguration("namespace")
    use_sim_time = LaunchConfiguration("use_sim_time")
    params_file = LaunchConfiguration("params_file")

    return LaunchDescription(
        [
            DeclareLaunchArgument("namespace", default_value="amr_sweeper"),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument(
                "params_file",
                default_value=PathJoinSubstitution(
                    [
                        FindPackageShare("amr_sweeper_collision_detector"),
                        "config",
                        "amr_sweeper_collision_detector.yaml",
                    ]
                ),
            ),
            Node(
                package="amr_sweeper_collision_detector",
                executable="collision_detector_node",
                name="collision_detector_node",
                namespace=namespace,
                output="screen",
                parameters=[
                    params_file,
                    {"use_sim_time": use_sim_time},
                ],
            ),
        ]
    )
