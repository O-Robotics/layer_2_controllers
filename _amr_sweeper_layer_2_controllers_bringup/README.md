# amr_sweeper_layer_2_controllers_bringup

```bash
ros2 launch amr_sweeper_layer_2_controllers_bringup amr_sweeper_layer_2_controllers_bringup.launch.py
```

Dependencies to other AMR Sweeper packages:
- `amr_sweeper_joystick`
- `amr_sweeper_sweeping_controller`
- `amr_sweeper_wheel_controller`
- `amr_sweeper_tool_controller`
- `amr_sweeper_attitude_controller`
- `amr_sweeper_safety_controller`
- `amr_sweeper_layer_1_hardware_bringup`

## Purpose
This package is the main entrypoint for the AMR Sweeper controller layer.

## Main Launch File
`launch/amr_sweeper_layer_2_controllers_bringup.launch.py`

## Available Launch Files
- `amr_sweeper_layer_2_controllers_bringup.launch.py`

## Launch Arguments
- `namespace`: default `amr_sweeper`
- `use_amr_sweeper_joystick`: default `true`
- `use_amr_sweeper_sweeping_controller`: default `true`
- `use_amr_sweeper_wheel_controller`: default `true`
- `use_amr_sweeper_tool_controller`: default `true`
- `use_amr_sweeper_attitude_controller`: default `true`
- `use_amr_sweeper_safety_controller`: default `true`
- `joy_dev`: default `/dev/input/js0`

## Overview
`amr_sweeper_layer_2_controllers_bringup` starts the layer 2 packages that shape wheel and tool commands before they reach the layer 1 hardware interfaces. It combines joystick teleoperation, sweeping-command arbitration, wheel-command forwarding, tool-command forwarding, attitude supervision, and the latched safety-stop path into one coordinated bringup.

## Notes
- Use this package when you want the whole controller layer running together.
- Layer 1 must already expose the wheel and tool controller interfaces needed by the layer 2 packages.
- The attitude controller expects the IMU topic at `/amr_sweeper/imu/data_raw` when using the default namespace.
