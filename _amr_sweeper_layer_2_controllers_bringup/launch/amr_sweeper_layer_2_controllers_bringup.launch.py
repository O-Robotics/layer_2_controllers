"""Launch the AMR Sweeper controller stack for manual and autonomous commands."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def _launch_file(package_name: str, launch_file_name: str):
    return PathJoinSubstitution([
        FindPackageShare(package_name),
        "launch",
        launch_file_name,
    ])


def generate_launch_description():
    namespace = LaunchConfiguration("namespace")
    use_joystick = LaunchConfiguration("use_joystick")
    use_twist_mux = LaunchConfiguration("use_twist_mux")
    use_wheel_controller = LaunchConfiguration("use_wheel_controller")
    use_tool_controller = LaunchConfiguration("use_tool_controller")
    use_attitude_controller = LaunchConfiguration("use_attitude_controller")
    joy_dev = LaunchConfiguration("joy_dev")

    return LaunchDescription([
        DeclareLaunchArgument("namespace", default_value="amr_sweeper"),
        DeclareLaunchArgument("use_joystick", default_value="true"),
        DeclareLaunchArgument("use_twist_mux", default_value="true"),
        DeclareLaunchArgument("use_wheel_controller", default_value="true"),
        DeclareLaunchArgument("use_tool_controller", default_value="true"),
        DeclareLaunchArgument("use_attitude_controller", default_value="true"),
        DeclareLaunchArgument("joy_dev", default_value="/dev/input/js0"),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(_launch_file("amr_sweeper_joystick", "joystick.launch.py")),
            launch_arguments={"namespace": namespace, "joy_dev": joy_dev}.items(),
            condition=IfCondition(use_joystick),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(_launch_file("amr_sweeper_twist_mux", "twist_mux.launch.py")),
            launch_arguments={"namespace": namespace}.items(),
            condition=IfCondition(use_twist_mux),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(_launch_file("amr_sweeper_wheel_controller", "wheel_controller.launch.py")),
            launch_arguments={"namespace": namespace}.items(),
            condition=IfCondition(use_wheel_controller),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(_launch_file("amr_sweeper_tool_controller", "tool_controller.launch.py")),
            launch_arguments={"namespace": namespace}.items(),
            condition=IfCondition(use_tool_controller),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(_launch_file("amr_sweeper_attitude_controller", "attitude_controller.launch.py")),
            launch_arguments={"namespace": namespace}.items(),
            condition=IfCondition(use_attitude_controller),
        ),
    ])
