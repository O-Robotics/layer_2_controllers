# amr_sweeper_safety_controller

```bash
ros2 launch amr_sweeper_safety_controller safety_controller.launch.py
```

Dependencies to other AMR Sweeper packages:
- `amr_sweeper_sweeping_controller`
- `amr_sweeper_wheel_controller`
- `amr_sweeper_tool_controller`

## Purpose
This package latches shared stop requests and forces the AMR Sweeper to a stop from layer 2.

## Main Launch File
`launch/safety_controller.launch.py`

## Available Launch Files
- `safety_controller.launch.py`

## Launch Arguments
- `namespace`: default `amr_sweeper`
- `params_file`: default `<package_share>/config/amr_sweeper_safety_controller.yaml`

## Overview
`amr_sweeper_safety_controller` subscribes to one shared typed stop topic that any package can publish to. On the first stop request it latches the event, republishes zero wheel commands into a dedicated sweeping-controller safety-stop input, republishes zero tool commands, and keeps the robot stopped until a manual reset service is called.

## Stop Message Type
- `amr_sweeper_safety_msgs/msg/SafetyStop`
- Fields: `stamp`, `sender`, `reason`

## Interfaces
- Subscribes to `safety_msgs/stop` as `amr_sweeper_safety_msgs/msg/SafetyStop`.
- Publishes `cmd_vel_safety_stop` as `geometry_msgs/msg/Twist`.
- Publishes `cmd_vel_joy_tools` as a zero `geometry_msgs/msg/Twist` while the stop is latched.
- Publishes `safety_controller/status` as `diagnostic_msgs/msg/DiagnosticArray`.
- Provides `amr_sweeper_safety_controller/reset_latched_stop` as `std_srvs/srv/Trigger`.
- Provides `amr_sweeper_safety_controller/enable_controller` as `std_srvs/srv/SetBool`.

## Notes
- The default node name is `safety_controller`, so under the default namespace it runs as `/amr_sweeper/safety_controller`.
- The public safety topics are kept intentionally small: `safety_msgs/stop` for stop events and `safety_controller/status` for diagnostics.
- `amr_sweeper_sweeping_controller` should assign the `cmd_vel_safety_stop` input the highest priority in layer 2.
- Navigation cancellation and direct hardware-stop hooks are left as explicit placeholders for the next integration step.
