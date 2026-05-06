# AMR Sweeper Attitude Controller

ROS 2 Humble C++ package for chassis roll/pitch estimation and operational stop supervision.

## First version

- Estimates roll and pitch from one or more `sensor_msgs/msg/Imu` inputs.
- Publishes `attitude/roll_pitch` in the robot namespace as radians in `geometry_msgs/msg/Vector3Stamped` with `x=roll`, `y=pitch`, `z=0`.
- Treats configurable nominal roll/pitch as the center point for safety warning/stop thresholds.
- Publishes attitude and safety diagnostics.
- Optionally publishes the dynamic `base_footprint -> base_link` attitude transform with zero translation.
- Provides enable/disable services for attitude estimation and safety stop supervision.
- Provides a reset service for latched stop faults.

## Integration

- The launch file accepts `namespace` and defaults to `amr_sweeper`.
- The default IMU input is `imu/data_raw` in that namespace.
- The default output topics are `attitude/roll_pitch`, `attitude/status`, `safety_stop`, and `safety/status` in that namespace.
- The default services are `amr_sweeper_attitude_controller/reset_fault`, `amr_sweeper_attitude_controller/enable_attitude_estimation`, and `amr_sweeper_attitude_controller/enable_safety_stop` in that namespace.

## TF ownership

When `publish_base_link_tf` is true, this node publishes `base_footprint -> base_link`.
Do not publish another transform for the same parent/child pair at the same time.

The controller does not provide a configurable Z offset. Any fixed physical offsets should live in the robot model around the chassis/link geometry, while this controller owns only the dynamic attitude transform it is configured to publish.

The IMU may be mounted as `imu_link`; the node uses TF to rotate IMU acceleration and angular velocity into `base_link` when the IMU message has a non-empty `header.frame_id`.

The estimator output is not offset: a level IMU reports approximately zero roll and pitch. The safety supervisor compares thresholds against `roll - stop.nominal_roll_deg` and `pitch - stop.nominal_pitch_deg`, so the default `5 deg` running pitch is treated as nominal for warnings and stops.

## TODO

- Add `cmd_vel_safe` gating after the desired incoming `cmd_vel` topic and integration path are chosen.
- Add a configurable "IMU health failure is fatal" safety-stop policy.
- Add future `tool_link` roll TF publication.
