# amr_sweeper_sweeping_controller

```bash
ros2 launch amr_sweeper_sweeping_controller sweeping_controller.launch.py
```

Dependencies to other AMR Sweeper packages:
- `amr_sweeper_joystick`
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

## Launch Arguments
- `namespace`: default `amr_sweeper`
- `params_file`: default `config/amr_sweeper_sweeping_controller.yaml`

## Overview
`amr_sweeper_sweeping_controller` replaces the old wheel-only `twist_mux` path with one controller-aware arbitration node. It consumes wheel joystick commands, tool joystick commands, Nav2 wheel commands, and the independent safety-stop wheel/tool override. It publishes the selected wheel command on `cmd_vel_sweep_wheels`, publishes the selected tool command on `cmd_vel_sweep_tools`, and emits a small status string describing which source is currently active.

## Notes
- Wheel source priority is configured in YAML and defaults to `safety_stop > joystick > navigation`.
- Tool source priority is configured in YAML and defaults to `safety_stop > joystick > navigation`.
- The default joystick hold time is 1 second before wheel control is handed back to navigation.
- Autonomous tool motion can be generated from `cmd_vel_nav` through configurable gains and offsets so a single Nav2 motion command can drive both the wheel and tool outputs.
- The sweeping controller stays above `amr_sweeper_drive_controller` and `amr_sweeper_tool_controller`; it does not replace those downstream controllers.
