# amr_sweeper_tool_controller

```bash
ros2 control list_controllers -c /amr_sweeper/controller_manager
```

Dependencies to other AMR Sweeper packages:
- `amr_sweeper_steadydrive`
- `amr_sweeper_ros2_control`

## Purpose
This package provides the custom `ros2_control` controller that drives the SteadyDrive brush joints directly.

## Configuration Files
- `config/amr_sweeper_tool_controller.yaml`

## Controller Parameters
- `left_joint`: default `LeftBrush_joint`
- `right_joint`: default `RightBrush_joint`
- `input_topic`: default `sweeping_controller/cmd_vel_tools`
- `direct_command_topic`: default `tool_controller/commands`
- `max_tool_speed_rad_s`: default `35.0`
- `command_timeout_sec`: default `0.5`
- `direct_command_timeout_sec`: default `0.5`

## Overview
`amr_sweeper_tool_controller` replaces the old standalone tool bridge plus the generic forward controller. It subscribes to the sweeping-controller tool twist output, converts that twist into left/right brush velocities, and writes those values straight into the SteadyDrive velocity interfaces inside `ros2_control`.

## Notes
- Consumes `sweeping_controller/cmd_vel_tools`.
- Consumes `tool_controller/commands` as a direct hardware-stop override path for the safety controller.
- Tool commands are written immediately after twist-to-brush conversion.
- Direct hardware-stop commands retain precedence for the safety controller.
- Runtime controller name: `tool_controller`.
- Default controller parameters live in `config/amr_sweeper_tool_controller.yaml` and are mirrored in the shared `ros2_control.yaml`; the layer 2 bringup path loads the shared `ros2_control.yaml` for this controller.
- Requires the layer 1 `ros2_control` runtime plus the layer 1 SteadyDrive hardware plugin to be available.
