# amr_sweeper_safety_msgs

Shared ROS interfaces for AMR Sweeper safety-stop communication.

## Messages
- `SafetyStop.msg`

## SafetyStop
- `stamp`: time the stop condition was produced
- `sender`: node or subsystem that raised the stop
- `reason`: human-readable stop reason
- `status`: producer-defined state such as `ERROR`, `STOP`, or `LATCHED`
- `value`: optional numeric value associated with the stop condition
