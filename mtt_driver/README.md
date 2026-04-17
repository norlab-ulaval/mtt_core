# mtt_driver

`mtt_driver` contains the MTT driver stack.

Today the core runtime path is in C++:
- CAN I/O
- command encoding
- odometry
- joystick teleoperation
- command smoothing

The package also keeps a few small helper scripts for simulation and checks.

## Build

From the workspace root:

```bash
source /opt/ros/jazzy/setup.bash
colcon build --packages-select mtt_driver
source install/setup.bash
```

## Main launch files

- `mtt_composable_system.launch.py`
  main driver stack with composable nodes
- `mtt.launch.py`
  higher-level driver entry point
- `mtt_teleop.launch.py`
  teleop-focused launch path
- `mtt_operator_teleop.launch.py`
  operator-side teleop workflow

## CAN setup

Real CAN:

```bash
sudo ip link set can0 up type can bitrate 250000
```

Virtual CAN:

```bash
sudo modprobe vcan
sudo ip link add dev vcan0 type vcan
sudo ip link set vcan0 up
```

## Typical runs

Real interface:

```bash
ros2 launch mtt_driver mtt_composable_system.launch.py can_interface:=can0 can_id:=1
```

Virtual interface:

```bash
ros2 launch mtt_driver mtt_composable_system.launch.py can_interface:=vcan0 can_id:=1
python3 src/mtt_core/mtt_driver/scripts/mtt_cmd_tachometer_sim.py --can-interface vcan0
```

Teleop:

```bash
ros2 launch mtt_driver mtt_teleop.launch.py
```

## Tests

This package includes:
- C++ unit tests under `test/`
- launch and safety checks under `test/`

Run them from the workspace root with your usual `colcon test` flow.

## Notes

- The low-level contract still needs robot-side validation for command ID ownership, steering truth, and safety behavior.
- The repository-level CAN notes live under `documentations/`.
