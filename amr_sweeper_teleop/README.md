# amr_sweeper_teleop

```bash
ros2 launch amr_sweeper_teleop teleop.launch.py
```

Dependencies to other AMR Sweeper packages:
- None

## Purpose
This package provides joystick teleoperation inputs for both wheel motion and tool motion.

## Main Launch File
`launch/teleop.launch.py`

## Available Launch Files
- `teleop.launch.py`

## Launch Arguments
- `namespace`: default `amr_sweeper`
- `use_joy_node`: default `false`
- `joy_dev`: default `/dev/input/js0`

## Overview
`amr_sweeper_teleop` starts two `teleop_twist_joy` nodes and can optionally start the physical `joy_node`. One path publishes wheel commands, and the other publishes tool commands. It is the manual operator entrypoint into the layer 2 control chain.

## Notes
- The default config file is `config/amr_sweeper_teleop.yaml`.
- `use_joy_node:=false` lets another process own `/joy` while reusing the same teleop mappings.
- Publishes wheel teleop commands on `teleop/cmd_vel_drive`.
- Publishes tool teleop commands on `teleop/cmd_vel_tools`.
