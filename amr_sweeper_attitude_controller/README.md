# amr_sweeper_attitude_controller

```bash
ros2 launch amr_sweeper_attitude_controller attitude_controller.launch.py
```

Dependencies to other AMR Sweeper packages:
- `amr_sweeper_imu`
- `amr_sweeper_description`

## Purpose
This package estimates chassis attitude and exposes safety-stop supervision signals for the AMR Sweeper controller layer.

## Main Launch File
`launch/attitude_controller.launch.py`

## Available Launch Files
- `attitude_controller.launch.py`

## Launch Arguments
- `namespace`: default `amr_sweeper`
- `params_file`: default `<package_share>/config/amr_sweeper_attitude_controller.yaml`

## Overview
`amr_sweeper_attitude_controller` subscribes to one or more IMU inputs, uses the IMU-provided orientation for roll and pitch, publishes attitude and diagnostic topics, and can publish base-attitude joint positions for the robot model. It also evaluates configurable warning and stop thresholds so higher layers can react to unsafe chassis attitude.

## Default Parameters
- `attitude_estimation_enabled`: default `true`
- `safety_stop_enabled`: default `true`
- `publish_base_link_joint_states`: default `true`
- `publish_tool_link_tf`: default `false`
- `base_roll_joint_name`: default `base_roll_joint`
- `base_pitch_joint_name`: default `base_pitch_joint`
- `imu_topics`: default `["imu/data_raw"]`
- `imu_weights`: default `[1.0]`
- `imu_timeout_warning_sec`: default `0.3`
- `imu_timeout_error_sec`: default `1.0`
- `imu_startup_grace_sec`: default `3.0`
- `imu_timeout_stop_enabled`: default `true`
- `publish_rate_hz`: default `10.0`
- `stop.roll_warning_deg`: default `15.0`
- `stop.pitch_warning_deg`: default `15.0`
- `stop.roll_stop_deg`: default `30.0`
- `stop.pitch_stop_deg`: default `30.0`
- `stop.nominal_roll_deg`: default `0.0`
- `stop.nominal_pitch_deg`: default `5.0`
- `stop.require_manual_reset`: default `true`

## Interfaces
- Subscribes to `imu/data_raw` in the selected robot namespace by default and uses the IMU orientation quaternion.
- Publishes `attitude/roll_pitch` as `geometry_msgs/msg/Vector3Stamped` with `x=roll_rad`, `y=pitch_rad`, `z=0`.
- Publishes `attitude/status` as `diagnostic_msgs/msg/DiagnosticArray`.
- Publishes `joint_states` updates for `base_roll_joint` and `base_pitch_joint` by default so `robot_state_publisher` can resolve the `base_footprint -> base_link` attitude chain from the URDF.
- Publishes `safety_msgs/stop` as `amr_sweeper_safety_msgs/msg/SafetyStop`, with debugging detail such as exceeded roll/pitch embedded in the `reason` string.
- Provides `amr_sweeper_attitude_controller/reset_fault` as `std_srvs/srv/Trigger`.
- Provides `amr_sweeper_attitude_controller/enable_attitude_estimation` as `std_srvs/srv/SetBool`.
- Provides `amr_sweeper_attitude_controller/enable_safety_stop` as `std_srvs/srv/SetBool`.

## Notes
- Default IMU input: `imu/data_raw` in the selected robot namespace.
- Default outputs: `attitude/roll_pitch`, `attitude/status`, and `safety_msgs/stop` in the selected robot namespace.
- Default base-attitude joint output: `joint_states` for `base_roll_joint` and `base_pitch_joint`.
- Default services: `amr_sweeper_attitude_controller/reset_fault`, `amr_sweeper_attitude_controller/enable_attitude_estimation`, and `amr_sweeper_attitude_controller/enable_safety_stop` in the selected robot namespace.
- When `publish_base_link_joint_states` is true, this node drives the URDF attitude joints that connect `base_footprint` to `base_link`.
- The controller expects the IMU `header.frame_id` to be transformable into `base_link` when a non-empty frame id is present.
