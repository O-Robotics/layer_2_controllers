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
- `params_file`: default `<package_share>/config/attitude_controller.yaml`

## Overview
`amr_sweeper_attitude_controller` subscribes to one or more IMU inputs, estimates roll and pitch, publishes attitude and diagnostic topics, and optionally publishes the dynamic `base_footprint -> base_link` transform. It also evaluates configurable warning, stop, and latch thresholds so higher layers can react to unsafe chassis attitude.

## Default Parameters
- `attitude_estimation_enabled`: default `true`
- `safety_stop_enabled`: default `false`
- `publish_base_link_tf`: default `true`
- `publish_tool_link_tf`: default `false`
- `imu_topics`: default `["imu/data_raw"]`
- `imu_weights`: default `[1.0]`
- `imu_timeout_sec`: default `0.25`
- `publish_rate_hz`: default `50.0`
- `filter.type`: default `complementary`
- `stop.roll_warning_deg`: default `8.0`
- `stop.pitch_warning_deg`: default `8.0`
- `stop.roll_stop_deg`: default `15.0`
- `stop.pitch_stop_deg`: default `15.0`
- `stop.roll_latch_deg`: default `25.0`
- `stop.pitch_latch_deg`: default `25.0`
- `stop.nominal_roll_deg`: default `0.0`
- `stop.nominal_pitch_deg`: default `5.0`
- `stop.hard_decel_threshold_mps2`: default `4.0`
- `stop.shock_threshold_mps2`: default `12.0`
- `stop.min_event_duration_ms`: default `80`
- `stop.require_manual_reset`: default `true`

## Interfaces
- Subscribes to `imu/data_raw` in the selected robot namespace by default.
- Publishes `attitude/roll_pitch` as `geometry_msgs/msg/Vector3Stamped` with `x=roll_rad`, `y=pitch_rad`, `z=0`.
- Publishes `attitude/status` as `diagnostic_msgs/msg/DiagnosticArray`.
- Publishes `safety_stop` as `std_msgs/msg/Bool`.
- Publishes `safety/status` as `diagnostic_msgs/msg/DiagnosticArray`.
- Provides `amr_sweeper_attitude_controller/reset_fault` as `std_srvs/srv/Trigger`.
- Provides `amr_sweeper_attitude_controller/enable_attitude_estimation` as `std_srvs/srv/SetBool`.
- Provides `amr_sweeper_attitude_controller/enable_safety_stop` as `std_srvs/srv/SetBool`.

## Notes
- Default IMU input: `imu/data_raw` in the selected robot namespace.
- Default outputs: `attitude/roll_pitch`, `attitude/status`, `safety_stop`, and `safety/status` in the selected robot namespace.
- Default services: `amr_sweeper_attitude_controller/reset_fault`, `amr_sweeper_attitude_controller/enable_attitude_estimation`, and `amr_sweeper_attitude_controller/enable_safety_stop` in the selected robot namespace.
- When `publish_base_link_tf` is true, this node owns `base_footprint -> base_link` and should be the only publisher for that transform.
- The controller expects the IMU `header.frame_id` to be transformable into `base_link` when a non-empty frame id is present.
