from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _as_bool(value: str) -> bool:
    return value.strip().lower() in {"1", "true", "yes", "on"}


def _launch_setup(context, *args, **kwargs):
    namespace = LaunchConfiguration("namespace")
    params_file = LaunchConfiguration("params_file")
    use_simulation = _as_bool(LaunchConfiguration("use_simulation").perform(context))

    parameters = [params_file]
    if use_simulation:
        parameters.append(
            {
                "direct_can_motor_stop_enabled": False,
                "odrive_direct_can_stop_enabled": False,
                "steadydrive_direct_can_stop_enabled": False,
                "button_can_monitor_enabled": False,
            }
        )

    return [
        Node(
            package="amr_sweeper_safety_controller",
            executable="safety_controller_node",
            name="safety_controller_node",
            namespace=namespace,
            output="screen",
            parameters=parameters,
        ),
    ]


def generate_launch_description():

    return LaunchDescription(
        [
            DeclareLaunchArgument("namespace", default_value="amr_sweeper"),
            DeclareLaunchArgument("use_simulation", default_value="false"),
            DeclareLaunchArgument(
                "params_file",
                default_value=PathJoinSubstitution(
                    [
                        FindPackageShare("amr_sweeper_safety_controller"),
                        "config",
                        "amr_sweeper_safety_controller.yaml",
                    ]
                ),
            ),
            OpaqueFunction(function=_launch_setup),
        ]
    )
