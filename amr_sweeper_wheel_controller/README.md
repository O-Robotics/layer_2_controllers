# amr_sweeper_wheel_controller

```bash
ros2 launch amr_sweeper_wheel_controller wheel_controller.launch.py
```

Dependencies to other AMR Sweeper packages:
- `amr_sweeper_odrive`

## Purpose
This package stamps layer 2 wheel commands and forwards them into the ODrive wheel controller path used by the AMR Sweeper.

## Main Launch File
`launch/wheel_controller.launch.py`

## Available Launch Files
- `wheel_controller.launch.py`

## Launch Arguments
- `namespace`: default `amr_sweeper`
- `cmd_vel_input_topic`: default `cmd_vel_wheels`
- `cmd_vel_output_topic`: default `diff_cont/cmd_vel`
- `cmd_vel_frame_id`: default `base_footprint`

## Overview
`amr_sweeper_wheel_controller` acts as the final layer 2 bridge before wheel commands reach the layer 1 ODrive control interface. It takes the selected `geometry_msgs/Twist` wheel command stream, stamps it as `geometry_msgs/TwistStamped`, and publishes it to the `diff_cont` command topic provided by the ODrive ros2_control setup.

## Notes
- Default input topic: `cmd_vel_wheels`.
- Default output topic: `diff_cont/cmd_vel`.
- Requires the layer 1 ODrive ros2_control path to be running.
