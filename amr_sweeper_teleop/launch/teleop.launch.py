import os
import yaml

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _load_use_joy_node_default(config_path: str) -> str:
    with open(config_path, "r", encoding="utf-8") as handle:
        data = yaml.safe_load(handle) or {}

    params = data.get("/**/joy_node", {}).get("ros__parameters", {})
    return "true" if params.get("use_joy_node", False) else "false"


def generate_launch_description():
    namespace = LaunchConfiguration("namespace")
    joy_dev = LaunchConfiguration("joy_dev")
    use_joy_node = LaunchConfiguration("use_joy_node")
    joy_params = os.path.join(
        get_package_share_directory("amr_sweeper_teleop"),
        "config",
        "amr_sweeper_teleop.yaml",
    )
    use_joy_node_default = _load_use_joy_node_default(joy_params)

    return LaunchDescription([
        DeclareLaunchArgument("namespace", default_value="amr_sweeper"),
        DeclareLaunchArgument("joy_dev", default_value="/dev/input/js0"),
        DeclareLaunchArgument("use_joy_node", default_value=use_joy_node_default),
        Node(
            package="joy",
            executable="joy_node",
            namespace=namespace,
            output="screen",
            parameters=[joy_params, {"device_id": 0, "dev": joy_dev}],
            condition=IfCondition(use_joy_node),
        ),
        Node(
            package="teleop_twist_joy",
            executable="teleop_node",
            name="teleop_twist_joy_wheels",
            namespace=namespace,
            output="screen",
            parameters=[joy_params],
            remappings=[("cmd_vel", "teleop/cmd_vel_drive")],
        ),
        Node(
            package="teleop_twist_joy",
            executable="teleop_node",
            name="teleop_twist_joy_tools",
            namespace=namespace,
            output="screen",
            parameters=[joy_params],
            remappings=[("cmd_vel", "teleop/cmd_vel_tools")],
        ),
    ])
