# mtt_driver launch files

## Entry points

- `mtt.launch.py`
  Canonical driver runtime. This is the launch file used by `norlab_robot` and the demos.
- `mtt_operator_teleop.launch.py`
  Operator-side joystick stack only.
- `mtt_composable_system.launch.py`
  Compatibility wrapper to `mtt.launch.py`.
- `mtt_teleop.launch.py`
  Compatibility wrapper to `mtt.launch.py` with teleop-friendly defaults.

## Parameter ownership

Package files keep standalone defaults:

- `config/mtt_driver_params.yaml`
- `config/mtt_health_monitor.yaml`
- `config/mtt_teleop_joy.yaml`
- `config/teleop_smoother.yaml`
- `config/twist_mux.yaml`

Live runtime tuning belongs in the demo parameter files under `demos/common/config/`
and the demo-specific `runtime.env`.

## Command flow

The canonical runtime launched by `mtt.launch.py` is:

- `joy_linux_node`
- `mtt_operator_input_node`
- `mtt_manual_cmd_filter_node`
- `mtt_mode_manager_node`
- `mtt_cmd_arbiter_node`
- `mtt_can_node`
- `mtt_odometry_node`
- `mtt_health_monitor_node`

Command topics:

- `cmd_vel/manual_raw`
- `cmd_vel/manual`
- `controller/cmd_vel`
- `cmd_vel`

Only `mtt_cmd_arbiter_node` publishes the final `cmd_vel`.

## Direct launch examples

```bash
ros2 launch mtt_driver mtt.launch.py
ros2 launch mtt_driver mtt.launch.py setup_real_can:=true can_interface:=can0
ros2 launch mtt_driver mtt.launch.py use_sim_time:=true can_interface:=vcan0
ros2 launch mtt_driver mtt_operator_teleop.launch.py joy_device:=/dev/input/js0
```

## CAN debug topic

Enable `publish_can_debug: true` in `config/mtt_driver_params.yaml` to publish
`mtt_can/debug_frames`.

## Health monitor

`mtt_health_monitor_node` publishes:

- `mtt_health`
- `mtt_monitor/cmd_fallback_odom`

The terminal helper is:

```bash
python3 scripts/mtt_health_monitor.py
ros2 run mtt_driver mtt_health_monitor.py
```
