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
`amr_sweeper_attitude_controller` subscribes to one or more IMU inputs, uses the IMU-provided orientation for roll and pitch, publishes attitude and diagnostic topics, and publishes the dynamic `base_footprint -> base_link` transform for the robot model. It also evaluates configurable warning and stop thresholds so higher layers can react to unsafe chassis attitude.

## Default Parameters
- `attitude_estimation_enabled`: default `true`
- `attitude_estimation.parent_frame`: default `base_footprint`
- `attitude_estimation.child_frame`: default `base_link`
- `attitude_estimation.publish_tf`: default `true`
- `attitude_estimation.hold_last_transform`: default `true`
- `attitude_estimation.initial_roll_deg`: default `0.0`
- `attitude_estimation.initial_pitch_deg`: default `4.5`
- `attitude_estimation.origin_z_m`: default `0.13`
- `attitude_estimation.imu_topic`: default `imu/data_raw`
- `attitude_estimation.imu_weights`: default `1.0`
- `attitude_estimation.imu_timeout_warning_sec`: default `0.3`
- `attitude_estimation.imu_timeout_error_sec`: default `1.0`
- `attitude_estimation.imu_startup_grace_sec`: default `6.0`
- `attitude_estimation.imu_timeout_stop_enabled`: default `true`
- `tool_angle_estimation_enabled`: default `false`
- `tool_angle_estimation.parent_frame`: default `base_link`
- `tool_angle_estimation.child_frame`: default `tool_link`
- `tool_angle_estimation.publish_tf`: default `false`
- `tool_angle_estimation.hold_last_transform`: default `true`
- `tool_angle_estimation.initial_roll_deg`: default `0.0`
- `tool_angle_estimation.initial_pitch_deg`: default `0.0`
- `tool_angle_estimation.origin_z_m`: default `0.0`
- `tool_angle_estimation.imu_topic`: default `""`
- `tool_angle_estimation.imu_weights`: default `1.0`
- `tool_angle_estimation.imu_timeout_warning_sec`: default `0.3`
- `tool_angle_estimation.imu_timeout_error_sec`: default `1.0`
- `tool_angle_estimation.imu_startup_grace_sec`: default `6.0`
- `tool_angle_estimation.imu_timeout_stop_enabled`: default `false`
- `safety_stop_enabled`: default `true`
- `stop.roll_warning_deg`: default `15.0`
- `stop.pitch_warning_deg`: default `15.0`
- `stop.roll_stop_deg`: default `30.0`
- `stop.pitch_stop_deg`: default `30.0`
- `stop.nominal_roll_deg`: default `0.0`
- `stop.nominal_pitch_deg`: default `5.0`
- `stop.require_manual_reset`: default `true`

## Interfaces
- Subscribes to `attitude_estimation.imu_topic`, default `imu/data_raw`, and uses the IMU orientation quaternion.
- Publishes `attitude_controller/roll_pitch` as `geometry_msgs/msg/Vector3Stamped` with `x=roll_rad`, `y=pitch_rad`, `z=0`.
- Publishes `attitude_controller/status` as `diagnostic_msgs/msg/DiagnosticArray`.
- Publishes `/tf` updates for the dynamic `base_footprint -> base_link` transform directly.
- Publishes `safety_msgs/stop` as `amr_sweeper_safety_msgs/msg/SafetyStop`, with debugging detail such as exceeded roll/pitch embedded in the `reason` string.
- Provides `amr_sweeper_attitude_controller/reset_fault` as `std_srvs/srv/Trigger`.
- Provides `amr_sweeper_attitude_controller/enable_attitude_estimation` as `std_srvs/srv/SetBool`.
- Provides `amr_sweeper_attitude_controller/enable_safety_stop` as `std_srvs/srv/SetBool`.

## Notes
- Default IMU input: `attitude_estimation.imu_topic`, which resolves to `imu/data_raw` in the selected robot namespace.
- Default outputs: `attitude_controller/roll_pitch`, `attitude_controller/status`, and `safety_msgs/stop` in the selected robot namespace.
- Default base-attitude output: dynamic `attitude_estimation.parent_frame -> attitude_estimation.child_frame` TF.
- Default services: `amr_sweeper_attitude_controller/reset_fault`, `amr_sweeper_attitude_controller/enable_attitude_estimation`, and `amr_sweeper_attitude_controller/enable_safety_stop` in the selected robot namespace.
- When `attitude_estimation.publish_tf` is true, this node directly publishes the `attitude_estimation.parent_frame -> attitude_estimation.child_frame` transform using the estimated roll and pitch plus the configured `attitude_estimation.origin_z_m` height offset.
- The controller expects the IMU `header.frame_id` to be transformable into `attitude_estimation.child_frame` when a non-empty frame id is present.
