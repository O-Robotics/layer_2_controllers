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
                        FindPackageShare("amr_sweeper_attitude_controller"),
                        "config",
                        "amr_sweeper_attitude_controller.yaml",
                    ]
                ),
                description=(
                    "Controller parameter file. Defaults expect the grouped "
                    "attitude_estimation.* and tool_angle_estimation.* layout."
                ),
            ),
            Node(
                package="amr_sweeper_attitude_controller",
                executable="attitude_controller_node",
                name="attitude_controller_node",
                namespace=namespace,
                output="screen",
                parameters=[
                    params_file,
                    {"use_sim_time": use_sim_time},
                ],
            ),
        ]
    )
