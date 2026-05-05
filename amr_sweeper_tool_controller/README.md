# amr_sweeper_tool_controller

```bash
ros2 launch amr_sweeper_tool_controller tool_controller.launch.py
```

Dependencies to other AMR Sweeper packages:
- `amr_sweeper_steadydrive`

## Purpose
This package converts tool-side joystick commands into the SteadyDrive controller command topic used by the AMR Sweeper.

## Main Launch File
`launch/tool_controller.launch.py`

## Available Launch Files
- `tool_controller.launch.py`

## Launch Arguments
- `namespace`: default `amr_sweeper`

## Overview
`amr_sweeper_tool_controller` is the layer 2 adapter between operator tool commands and the layer 1 SteadyDrive controller interface. It subscribes to the tool joystick command topic and publishes motor-command values that the SteadyDrive side can consume.

## Notes
- Consumes `cmd_vel_joy_brushes`.
- Publishes `controller_steadydrive/commands`.
- Requires the layer 1 SteadyDrive control path to be available.
