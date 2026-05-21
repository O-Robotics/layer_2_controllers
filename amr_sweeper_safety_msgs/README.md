# amr_sweeper_safety_msgs

Shared ROS interfaces for AMR Sweeper safety-stop communication.

## Messages
- `SafetyStop.msg`

## SafetyStop
- `stamp`: time the stop condition was produced
- `sender`: node or subsystem that raised the stop
- `reason`: human-readable stop reason

## How To Use
- Publish to `safety_msgs/stop` only when your node needs the robot to stop.
- Keep `sender` stable and easy to identify in logs and diagnostics.
- Make `reason` informative for debugging, not just for stopping.
- If your node has a numeric trigger value, include it directly in `reason`.
- Example attitude-controller reason: `roll and pitch stop, roll=178.9 deg, pitch=-176.4 deg`
