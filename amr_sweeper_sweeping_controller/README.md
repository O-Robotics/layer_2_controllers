# amr_sweeper_sweeping_controller

```bash
ros2 launch amr_sweeper_sweeping_controller sweeping_controller.launch.py
```

Dependencies to other AMR Sweeper packages:
- `amr_sweeper_teleop`
- `amr_sweeper_drive_controller`
- `amr_sweeper_tool_controller`
- `amr_sweeper_safety_controller`

## Purpose
This package arbitrates wheel and tool command sources for the AMR Sweeper and forwards the selected commands to the downstream layer 2 wheel and tool controllers.

## Main Launch File
`launch/sweeping_controller.launch.py`

## Available Launch Files
- `sweeping_controller.launch.py`

## Configuration Files
- `config/amr_sweeper_sweeping_controller.yaml`
- `config/sweeping_modes.yaml`

## Launch Arguments
- `namespace`: default `amr_sweeper`
- `params_file`: default `config/amr_sweeper_sweeping_controller.yaml`
- `sweeping_modes_file`: default `config/sweeping_modes.yaml`
- `use_sweeping_mode`: default `cmd_vel_sweeping`

## Overview
`amr_sweeper_sweeping_controller` replaces the old wheel-only `twist_mux` path with one controller-aware arbitration node. It consumes wheel joystick commands, tool joystick commands, Nav2 wheel commands, and the independent safety-stop wheel/tool override. It publishes the selected wheel command on `sweeping_controller/cmd_vel_drive`, publishes the selected tool command on `sweeping_controller/cmd_vel_tools`, and emits a small status string describing which source is currently active.

## Notes
- Wheel source priority is configured in YAML and defaults to `safety_stop > joystick > navigation`.
- Tool source priority is configured in YAML and defaults to `safety_stop > joystick > navigation`.
- The default joystick hold time is 1 second before wheel control is handed back to navigation.
- Autonomous sweeping presets live in `config/sweeping_modes.yaml`.
- `use_sweeping_mode:=cmd_vel_sweeping` keeps the gain-and-offset behavior derived from `navigation/cmd_vel`.
- `use_sweeping_mode:=constant_inward` publishes the configured constant tool `Twist` whenever a fresh autonomous `navigation/cmd_vel` command is active.
- The sweeping controller stays above `amr_sweeper_drive_controller` and `amr_sweeper_tool_controller`; it does not replace those downstream controllers.
