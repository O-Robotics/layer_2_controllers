# amr_sweeper_drive_controller

```bash
ros2 control list_controllers -c /amr_sweeper/controller_manager
```

Dependencies to other AMR Sweeper packages:
- `amr_sweeper_odrive`
- `amr_sweeper_ros2_control`

## Purpose
This package provides the custom `ros2_control` differential-drive controller that replaces the old wheel-command stamper plus the generic diff-drive controller.

## Configuration Files
- `config/amr_sweeper_drive_controller.yaml`

## Controller Parameters
- `left_wheel_names`: default `['LeftWheel_joint']`
- `right_wheel_names`: default `['RightWheel_joint']`
- `input_topic`: default `cmd_vel_sweep_wheels`
- `direct_command_topic`: default `drive_controller/cmd_vel`
- `odom_topic`: default `drive_controller/odom`
- `wheel_separation`: default `0.490`
- `wheel_radius`: default `0.13`
- `position_feedback`: default `false`
- `publish_rate`: default `10.0`
- `command_timeout_sec`: default `0.5`
- `direct_command_timeout_sec`: default `0.5`

## Overview
`amr_sweeper_drive_controller` owns the differential-drive kinematics and wheel-odometry publishing inside the shared `ros2_control` runtime. It subscribes to the selected `geometry_msgs/Twist` wheel-command stream from the sweeping controller, keeps the direct `drive_controller/cmd_vel` safety-stop override path, writes wheel velocity commands directly to the ODrive hardware interfaces, and publishes `nav_msgs/Odometry` on `drive_controller/odom`.

## Notes
- Default input topic: `cmd_vel_sweep_wheels`.
- Default direct override topic: `drive_controller/cmd_vel`.
- Default wheel odometry topic: `drive_controller/odom`.
- Runtime controller name: `drive_controller`.
- Default controller parameters live in `config/amr_sweeper_drive_controller.yaml` and are mirrored in the shared `ros2_control.yaml`.
- Requires the layer 1 `ros2_control` runtime plus the layer 1 ODrive hardware plugin to be available.
