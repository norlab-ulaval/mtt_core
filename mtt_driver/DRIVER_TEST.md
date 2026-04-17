# mtt_driver test notes

This file keeps a short checklist for low-risk local testing of `mtt_driver`.

## Build the package

From the workspace root:

```bash
source /opt/ros/jazzy/setup.bash
colcon build --packages-select mtt_driver
source install/setup.bash
```

## Use a virtual CAN interface

```bash
sudo modprobe vcan
sudo ip link add dev vcan0 type vcan
sudo ip link set vcan0 up
```

## Launch the driver stack on `vcan0`

```bash
ros2 launch mtt_driver mtt_composable_system.launch.py can_interface:=vcan0 can_id:=1
```

## Feed tachometer traffic

In another terminal:

```bash
python3 src/mtt_core/mtt_driver/scripts/mtt_cmd_tachometer_sim.py --can-interface vcan0
```

## What to check

- the launch comes up cleanly
- CAN frames are exchanged on the expected interface
- odometry and telemetry topics are published
- stopping the launch shuts the nodes down cleanly

## What this file is not

This is not a claim that CAN truth is fully closed.
Use it for local checks, not as proof of the real robot contract.
