# amr_sweeper_twist_mux

`ros2 launch amr_sweeper_twist_mux twist_mux.launch.py`

Dependencies to other AMR Sweeper packages:
- None

## Purpose
This package arbitrates multiple wheel-command sources for the AMR Sweeper.

## Main Launch File
`launch/twist_mux.launch.py`

## Available Launch Files
- `twist_mux.launch.py`

## Launch Arguments
- `namespace`: default `amr_sweeper`

## Overview
`amr_sweeper_twist_mux` merges wheel command inputs such as joystick commands, navigation commands, and debug commands into one wheel-command output topic. It is the central command arbitration point in layer 2 for wheel motion.

## Notes
- Publishes merged wheel commands on `cmd_vel_wheels`.
- Common upstream inputs are joystick and layer 3 navigation commands.
