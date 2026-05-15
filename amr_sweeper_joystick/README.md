# amr_sweeper_joystick

```bash
ros2 launch amr_sweeper_joystick joystick.launch.py
```

Dependencies to other AMR Sweeper packages:
- None

## Purpose
This package provides joystick teleoperation inputs for both wheel motion and tool motion.

## Main Launch File
`launch/joystick.launch.py`

## Available Launch Files
- `joystick.launch.py`

## Launch Arguments
- `namespace`: default `amr_sweeper`
- `joy_dev`: default `/dev/input/js0`

## Overview
`amr_sweeper_joystick` starts the joystick device node together with two teleop_twist_joy nodes. One path publishes wheel commands, and the other publishes tool commands. It is the manual operator entrypoint into the layer 2 control chain.

## Notes
- Publishes wheel teleop commands on `cmd_vel_joy_wheels`.
- Publishes tool teleop commands on `cmd_vel_joy_tools`.
