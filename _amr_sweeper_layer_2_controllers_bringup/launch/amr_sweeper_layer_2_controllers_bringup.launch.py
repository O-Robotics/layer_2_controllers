"""Launch the AMR Sweeper controller stack for manual and autonomous commands."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, OpaqueFunction, RegisterEventHandler, SetEnvironmentVariable
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _launch_file(package_name: str, launch_file_name: str):
    return PathJoinSubstitution([
        FindPackageShare(package_name),
        "launch",
        launch_file_name,
    ])


def _as_bool(value: str) -> bool:
    return value.strip().lower() in {"1", "true", "yes", "on"}


def _launch_setup(context, *args, **kwargs):
    namespace = LaunchConfiguration("namespace").perform(context)
    use_amr_sweeper_drive_controller = _as_bool(
        LaunchConfiguration("use_amr_sweeper_drive_controller").perform(context))
    use_amr_sweeper_tool_controller = _as_bool(
        LaunchConfiguration("use_amr_sweeper_tool_controller").perform(context))
    use_amr_sweeper_joystick = _as_bool(LaunchConfiguration("use_amr_sweeper_joystick").perform(context))
    use_amr_sweeper_sweeping_controller = _as_bool(
        LaunchConfiguration("use_amr_sweeper_sweeping_controller").perform(context))
    use_amr_sweeper_attitude_controller = _as_bool(
        LaunchConfiguration("use_amr_sweeper_attitude_controller").perform(context))
    use_amr_sweeper_collision_detector = _as_bool(
        LaunchConfiguration("use_amr_sweeper_collision_detector").perform(context))
    use_amr_sweeper_safety_controller = _as_bool(
        LaunchConfiguration("use_amr_sweeper_safety_controller").perform(context))
    use_joy_node = LaunchConfiguration("use_joy_node").perform(context)
    joy_dev = LaunchConfiguration("joy_dev").perform(context)
    ros2_control_config_file = PathJoinSubstitution([
        FindPackageShare("amr_sweeper_description"),
        "urdf",
        "control",
        "ros2_control.yaml",
    ])

    actions = []
    controller_dependent_actions = []

    if use_amr_sweeper_joystick:
        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(_launch_file("amr_sweeper_joystick", "joystick.launch.py")),
            launch_arguments={
                "namespace": namespace,
                "joy_dev": joy_dev,
                "use_joy_node": use_joy_node,
            }.items(),
        ))

    if use_amr_sweeper_attitude_controller:
        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(_launch_file("amr_sweeper_attitude_controller", "attitude_controller.launch.py")),
            launch_arguments={"namespace": namespace}.items(),
        ))

    if use_amr_sweeper_collision_detector:
        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                _launch_file(
                    "amr_sweeper_collision_detector",
                    "amr_sweeper_collision_detector.launch.py")),
            launch_arguments={"namespace": namespace}.items(),
        ))

    if use_amr_sweeper_sweeping_controller:
        controller_dependent_actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                _launch_file("amr_sweeper_sweeping_controller", "sweeping_controller.launch.py")),
            launch_arguments={"namespace": namespace}.items(),
        ))

    if use_amr_sweeper_safety_controller:
        controller_dependent_actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(_launch_file("amr_sweeper_safety_controller", "safety_controller.launch.py")),
            launch_arguments={"namespace": namespace}.items(),
        ))

    drive_controller_spawner = None
    if use_amr_sweeper_drive_controller:
        drive_controller_spawner = Node(
            package="controller_manager",
            executable="spawner",
            arguments=[
                "drive_controller",
                "--controller-manager",
                f"/{namespace}/controller_manager",
                "--controller-manager-timeout",
                "60",
                "--param-file",
                ros2_control_config_file,
                "--controller-ros-args",
                "--remap /tf:=drive_controller_disabled_tf",
            ],
            namespace=namespace,
            output="screen",
            additional_env={"RCUTILS_COLORIZED_OUTPUT": "0"},
        )

    tool_controller_spawner = None
    if use_amr_sweeper_tool_controller:
        tool_controller_spawner = Node(
            package="controller_manager",
            executable="spawner",
            arguments=[
                "tool_controller",
                "--controller-manager",
                f"/{namespace}/controller_manager",
                "--controller-manager-timeout",
                "60",
                "--param-file",
                ros2_control_config_file,
            ],
            namespace=namespace,
            output="screen",
            additional_env={"RCUTILS_COLORIZED_OUTPUT": "0"},
        )

    controller_dependent_group = GroupAction(actions=controller_dependent_actions)

    if drive_controller_spawner and tool_controller_spawner:
        actions.extend([
            drive_controller_spawner,
            RegisterEventHandler(
                OnProcessExit(
                    target_action=drive_controller_spawner,
                    on_exit=[tool_controller_spawner],
                )),
            RegisterEventHandler(
                OnProcessExit(
                    target_action=tool_controller_spawner,
                    on_exit=[controller_dependent_group],
                )),
        ])
    elif drive_controller_spawner:
        actions.extend([
            drive_controller_spawner,
            RegisterEventHandler(
                OnProcessExit(
                    target_action=drive_controller_spawner,
                    on_exit=[controller_dependent_group],
                )),
        ])
    elif tool_controller_spawner:
        actions.extend([
            tool_controller_spawner,
            RegisterEventHandler(
                OnProcessExit(
                    target_action=tool_controller_spawner,
                    on_exit=[controller_dependent_group],
                )),
        ])
    else:
        actions.append(controller_dependent_group)

    return actions


def generate_launch_description():
    console_output_format = "[{severity}] [{time}] [{name}] : {message}"
    return LaunchDescription([
        SetEnvironmentVariable("RCUTILS_COLORIZED_OUTPUT", "1"),
        SetEnvironmentVariable("RCUTILS_CONSOLE_OUTPUT_FORMAT", console_output_format),
        DeclareLaunchArgument("namespace", default_value="amr_sweeper"),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("use_amr_sweeper_drive_controller", default_value="true"),
        DeclareLaunchArgument("use_amr_sweeper_tool_controller", default_value="true"),
        DeclareLaunchArgument("use_amr_sweeper_joystick", default_value="true"),
        DeclareLaunchArgument("use_amr_sweeper_sweeping_controller", default_value="true"),
        DeclareLaunchArgument("use_amr_sweeper_attitude_controller", default_value="true"),
        DeclareLaunchArgument("use_amr_sweeper_collision_detector", default_value="true"),
        DeclareLaunchArgument("use_amr_sweeper_safety_controller", default_value="true"),
        DeclareLaunchArgument("use_joy_node", default_value="false"),
        DeclareLaunchArgument("joy_dev", default_value="/dev/input/js0"),
        OpaqueFunction(function=_launch_setup),
    ])
