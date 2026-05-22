# amr_sweeper_safety_msgs

```bash
ros2 interface show amr_sweeper_safety_msgs/msg/SafetyStop
```

Dependencies to other AMR Sweeper packages:
- None

## Purpose
This package defines the shared ROS interfaces used for AMR Sweeper safety-stop communication.

## Interfaces
- `SafetyStop.msg`

## Overview
`amr_sweeper_safety_msgs` is the common message package used by layer 2 controllers and any future safety-aware nodes that need to request or inspect a shared robot stop.

## SafetyStop
- `stamp`: time the stop condition was produced
- `sender`: node or subsystem that raised the stop
- `reason`: human-readable stop reason

## Notes
- Publish to `safety_msgs/stop` only when your node needs the robot to stop.
- Keep `sender` stable and easy to identify in logs and diagnostics.
- Make `reason` informative for debugging, not just for stopping.
- If your node has a numeric trigger value, include it directly in `reason`.
- Example attitude-controller reason: `roll and pitch stop, roll=178.9 deg, pitch=-176.4 deg`
