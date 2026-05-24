# amr_sweeper_tool_controller

```bash
ros2 launch amr_sweeper_tool_controller tool_controller.launch.py
```

Dependencies to other AMR Sweeper packages:
- `amr_sweeper_steadydrive`

## Purpose
This package converts selected layer 2 tool commands into the SteadyDrive controller command topic used by the AMR Sweeper.

## Main Launch File
`launch/tool_controller.launch.py`

## Available Launch Files
- `tool_controller.launch.py`

## Configuration Files
- `config/amr_sweeper_tool_controller.yaml`

## Launch Arguments
- `namespace`: default `amr_sweeper`
- `params_file`: default `config/amr_sweeper_tool_controller.yaml`

## Overview
`amr_sweeper_tool_controller` is the layer 2 adapter between the sweeping-controller tool command output and the layer 1 SteadyDrive controller interface. It subscribes to the selected tool command topic and publishes motor-command values that the SteadyDrive side can consume.

## Notes
- Consumes `cmd_vel_sweep_tools`.
- Publishes `controller_steadydrive/commands`.
- Runtime node target: `amr_sweeper_tool_controller_node`.
- Default runtime parameters live in `config/amr_sweeper_tool_controller.yaml`.
- Requires the layer 1 SteadyDrive control path to be available.
