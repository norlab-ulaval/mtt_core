# MTT-154 launch files

This package no longer uses the old `production/`, `development/`, and `tests/`
subfolders. The active launch files live directly in `mtt_driver/launch/`.

## Current launch files

- `mtt.launch.py`
- `mtt_teleop.launch.py`
- `mtt_composable_system.launch.py`

## Recommended entry points

### Teleop only

```bash
ros2 launch mtt_driver mtt_teleop.launch.py can_interface:=vcan0
```

### Full driver system

```bash
ros2 launch mtt_driver mtt_composable_system.launch.py
```

### Full driver system without teleop

```bash
ros2 launch mtt_driver mtt_composable_system.launch.py enable_teleop:=false
```

### Explicit command CAN ID

```bash
ros2 launch mtt_driver mtt_composable_system.launch.py can_interface:=can0 can_id:=1
```

`can_id:=1` corresponds to arbitration ID `0x001`.

## CAN interface setup

`mtt_composable_system.launch.py` supports three modes:

### Manual setup

Default behavior. The CAN interface is assumed to be configured on the host
before launching ROS.

```bash
sudo ip link set can0 up type can bitrate 250000
ros2 launch mtt_driver mtt_composable_system.launch.py can_interface:=can0
```

### Virtual CAN

Useful for local tests or Docker.

```bash
ros2 launch mtt_driver mtt_composable_system.launch.py setup_vcan:=true can_interface:=vcan0
```

### Real CAN from launch

Useful for host-side bringup when the operator accepts the launch file managing
the interface.

```bash
ros2 launch mtt_driver mtt_composable_system.launch.py \
  setup_real_can:=true \
  can_interface:=can0 \
  can_bitrate:=250000
```

## Current architecture

The active launch path centers around:

- `mtt_ros_wrapper`
- `twist_mux`
- `joy_linux_node`
- `mtt_teleop_joy`

The odometry manager is still present in the package but is not part of the
default live launch path at the moment.
