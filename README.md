# layer_2_controllers

```
ros2 launch amr_sweeper_layer_2_controllers_bringup amr_sweeper_layer_2_controllers_bringup.launch.py
```

Dependencies to other AMR Sweeper packages:
- `amr_sweeper_layer_2_controllers_bringup`
- `amr_sweeper_joystick`
- `amr_sweeper_sweeping_controller`
- `amr_sweeper_attitude_controller`
- `amr_sweeper_collision_detector`
- `amr_sweeper_safety_msgs`
- `amr_sweeper_safety_controller`
- `amr_sweeper_layer_1_hardware_bringup`

## Purpose
This repository is the controller layer for the AMR Sweeper. It turns human or autonomous motion commands into the specific command topics consumed by the layer 1 wheel and tool motor interfaces.

## Launch Arguments
- `namespace`: default `amr_sweeper`
- `use_sim_time`: default `false`
- `use_amr_sweeper_drive_controller`: default `true`
- `use_amr_sweeper_tool_controller`: default `true`
- `use_amr_sweeper_joystick`: default `true`
- `use_amr_sweeper_sweeping_controller`: default `true`
- `use_amr_sweeper_attitude_controller`: default `true`
- `use_amr_sweeper_collision_detector`: default `true`
- `use_amr_sweeper_safety_controller`: default `true`
- `use_joy_node`: default `false`
- `joy_dev`: default `/dev/input/js0`
- `use_sweeping_mode`: default `cmd_vel_sweeping`

## Overview
Layer 2 sits between the hardware interfaces in layer 1 and the higher-level decision-making in layer 3. It contains the joystick input path, the sweeping-controller arbitration path for wheel and tool motion, the attitude supervision path driven by the robot IMU, the collision detector that fuses IMU and motor-force proxies, and the shared latched safety-stop controller.

## Notes
- The default command launches the full layer 2 controller bringup package.
- Layer 1 owns the `ros2_control` runtime, the hardware components, and `joint_broad`.
- Layer 2 assumes the layer 1 hardware bringup has already activated `ros2_control` and exposed the raw hardware feedback needed for `drive_controller` and `tool_controller`.
- The attitude controller consumes `imu/data_raw`, which resolves to `/amr_sweeper/imu/data_raw` under the default namespace, and can publish shared safety-stop requests into the namespaced layer 2 stop path.
- The collision detector uses enabled IMU inputs immediately and keeps the currently unavailable motor-force proxy inputs disabled by default until layer 1 telemetry is exposed.
