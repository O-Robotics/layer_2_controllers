# amr_sweeper_safety_controller

```bash
ros2 launch amr_sweeper_safety_controller safety_controller.launch.py
```

Dependencies to other AMR Sweeper packages:
- `amr_sweeper_sweeping_controller`
- `amr_sweeper_drive_controller`
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
`amr_sweeper_safety_controller` subscribes to one shared typed stop topic that any package can publish to. On the first stop request it latches the event, republishes zero wheel commands into a dedicated sweeping-controller safety-stop input, republishes zero tool commands, also drives zero commands directly into the final wheel and tool controller inputs, and sends direct CAN stop frames to the Layer 1 ODrive and Steadydrive motor nodes. The stop stays latched until a manual reset service is called.

## Stop Message Type
- `amr_sweeper_safety_msgs/msg/SafetyStop`
- Fields: `stamp`, `sender`, `reason`

## Interfaces
- Subscribes to `safety_msgs/stop` as `amr_sweeper_safety_msgs/msg/SafetyStop`.
- Publishes `cmd_vel_safety_stop` as `geometry_msgs/msg/Twist`.
- Publishes `cmd_vel_joy_tools` as a zero `geometry_msgs/msg/Twist` while the stop is latched.
- Publishes `drive_controller/cmd_vel` as a zero `geometry_msgs/msg/TwistStamped` while the stop is latched so the absorbed drive controller sees a direct wheel-stop override.
- Publishes `tool_controller/commands` as a zero `std_msgs/msg/Float64MultiArray` while the stop is latched.
- Sends ODrive CANSimple e-stop frames (`cmd_id 0x02`) directly to the configured ODrive node IDs while the stop is latched.
- Sends Steadydrive stop (`0x81`) and motor-off (`0x80`) frames directly to the configured tool-motor CAN IDs while the stop is latched.
- Monitors the configured button-module CAN base ID and latches a stop when it receives either the button event frame (`data[0] = 0x01` on `base_id`) or a status frame with pressed bits set (`data[1] & 0x11` on `base_id + 1`).
- Watches the button status heartbeat on `base_id + 1` and latches a safety stop if that heartbeat disappears past the configured timeout.
- Publishes `safety_controller/status` as `diagnostic_msgs/msg/DiagnosticArray`.
- Calls `end_mission` on `amr_sweeper_mission_executor` when a new stop latches so the active mission is aborted and the FSM can return to `IDLING`.
- Calls the FSM supervisor `request_state` service to force the robot into `FAULT` profile `400` whenever a safety stop latches.
- Provides `amr_sweeper_safety_controller/reset_latched_stop` as `std_srvs/srv/Trigger`.
- Provides `amr_sweeper_safety_controller/enable_controller` as `std_srvs/srv/SetBool`.

## Notes
- The default node name is `safety_controller`, so under the default namespace it runs as `/amr_sweeper/safety_controller`.
- The public safety topics are kept intentionally small: `safety_msgs/stop` for stop events and `safety_controller/status` for diagnostics.
- `amr_sweeper_sweeping_controller` should assign the `cmd_vel_safety_stop` input the highest priority in layer 2.
- Recovery still happens through the existing Layer 1 `clear_safety_stop` services so the ODrive e-stop/error latch is cleared and the Steadydrive motors are re-enabled with their native bringup path.
- The current default `button_can_base_id` is `0x200`, which matches the firmware fallback when no stored `CFG_CAN_ID` is present; if your flashed button module uses another base ID, update the Layer 2 parameter to match it.
- The current button firmware documents `CFG_STATUS_MS` defaulting to `5000 ms`, so the default watchdog timeout here is `12000 ms` to allow missed frames without masking a real module disappearance.
- Direct Nav2 goal cancellation is still an open follow-up integration.
