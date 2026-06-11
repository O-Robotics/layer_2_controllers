from pathlib import Path

import yaml

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _load_selected_mode(modes_file: str, selected_mode_name: str) -> dict:
    config = yaml.safe_load(Path(modes_file).read_text(encoding="utf-8")) or {}
    sweeping_modes = config.get("sweeping_modes", {})
    modes = sweeping_modes.get("modes", [])
    default_mode_name = sweeping_modes.get("default_mode", "cmd_vel_sweeping")
    requested_mode_name = selected_mode_name or default_mode_name

    for mode in modes:
        if mode.get("name") == requested_mode_name:
            return {key: value for key, value in mode.items() if key != "name"}

    available_modes = ", ".join(
        sorted(str(mode.get("name")) for mode in modes if mode.get("name")))
    raise RuntimeError(
        f"Unknown sweeping mode '{requested_mode_name}' in {modes_file}. "
        f"Available modes: {available_modes}")


def _launch_setup(context, *args, **kwargs):
    namespace = LaunchConfiguration('namespace').perform(context)
    params_file = LaunchConfiguration('params_file')
    sweeping_modes_file = LaunchConfiguration('sweeping_modes_file').perform(context)
    selected_mode_name = LaunchConfiguration('use_sweeping_mode').perform(context)
    selected_mode_parameters = _load_selected_mode(sweeping_modes_file, selected_mode_name)

    return [
        Node(
            package='amr_sweeper_sweeping_controller',
            executable='sweeping_controller_node',
            namespace=namespace,
            name='sweeping_controller_node',
            output='screen',
            parameters=[params_file, selected_mode_parameters],
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value='amr_sweeper'),
        DeclareLaunchArgument(
            'params_file',
            default_value=PathJoinSubstitution([
                FindPackageShare('amr_sweeper_sweeping_controller'),
                'config',
                'amr_sweeper_sweeping_controller.yaml',
            ]),
        ),
        DeclareLaunchArgument(
            'sweeping_modes_file',
            default_value=PathJoinSubstitution([
                FindPackageShare('amr_sweeper_sweeping_controller'),
                'config',
                'sweeping_modes.yaml',
            ]),
        ),
        DeclareLaunchArgument('use_sweeping_mode', default_value='cmd_vel_sweeping'),
        OpaqueFunction(function=_launch_setup),
    ])
