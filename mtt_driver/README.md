# MTT-154 Driver

ROS 2 driver for MTT-154 All-Terrain Vehicle.

**CAN Bus Specification v1.1 Compliant** - Includes tachometer data processing, proper security switch handling, and emergency stop functionality.

## Installation

```bash
cd ~/mtt_ws
colcon build --packages-select mtt_driver
source install/setup.bash
```

## Usage

```bash 
# Setup CAN interface
sudo ip link set can0 up type can bitrate 250000
```
Or setup virtual can interface:
```bash
# Setup virtual CAN interface
sudo ip link add dev vcan0 type vcan
sudo ip link set up vcan0
```
To launch only the driver and the teleoperation:
```
# Test teleop
ros2 launch mtt_driver mtt_teleop.launch.py
```
To launch the full driver system with the odometry on the mtt launch:
```bash
# Launch driver on mtt
ros2 launch mtt_driver mtt_composable_system.launch.py can_interface:=can0 can_id:=1
```
In simulation:
```bash
# Launch driver in simulation
ros2 launch mtt_driver mtt_composable_system.launch.py can_interface:=vcan0 can_id:=1

# in simulation to reproduce the tachometer
python3 src/mtt_driver/scripts/mtt_cmd_tachometer_sim.py --can-interface vcan0
```

## Components

- `mtt_ros_wrapper.py`: Main ROS 2 wrapper with tachometer data publishing
- `mtt_driver.py`: Pure Python CAN bus interface (CANBus_Specification.md v1.1 compliant)
- `mtt_teleop_joy.py`: Joystick control
```bash
ros2 run mtt_driver mtt_teleop_joy
```

## Topics

The nodes in this package use relative topic and service names by default.
The names below are shown as resolved names in the root namespace.

### Subscribed Topics

- `/cmd_vel` (geometry_msgs/TwistStamped): Standard ROS velocity commands
- `/mtt_aux_cmd` (mtt_driver/MttAuxCommand): Auxiliary commands (brake, winch, dead man's switch)
- `/joy` (sensor_msgs/Joy): Raw joystick input (teleop node only)

### Published Topics

#### Control Topics (Teleop only)
- `/cmd_vel/teleop` (geometry_msgs/TwistStamped): Velocity commands from joystick (teleop path)
- `/mtt_aux_cmd` (mtt_driver/MttAuxCommand): Auxiliary commands from joystick (teleop node only)

#### Telemetry Topics (CANBus_Specification.md v1.1)
- `/mtt_speed` (std_msgs/Float64): Vehicle speed in km/h
- `/mtt_distance` (std_msgs/Float64): Cumulative distance in kilometers
- `/mtt_temperature` (std_msgs/Float64MultiArray): Temperature sensors [A, B] in °C
- `/mtt_tachometer` (mtt_driver/MttTachometerData): Complete tachometer data structure
- `/mtt_odometry` (nav_msgs/Odometry): Standard ROS2 odometry for navigation stack
- `/mtt/temperature_a` (sensor_msgs/Temperature): Main sensor temperature A
- `/mtt/temperature_b` (sensor_msgs/Temperature): Main sensor temperature B

## Controller Mapping

The default controller mapping is configured for 8BitDo controllers:

- **Left Stick Vertical** (axis 1): Forward/backward motion
- **Right Stick Horizontal** (axis 3): Steering
- **Right Trigger** (axis 5): Brake control
- **D-Pad Vertical** (axis 7): Winch control (up=in, down=out)
- **Right Shoulder Button** (button 5): Dead man's switch

## Safety Features

- **Dead Man's Switch**: Motion is disabled unless the dead man's switch is active
- **E-Stop**: Releasing the dead man's switch immediately stops all motion
- **Brake Priority**: Brake commands take precedence over throttle commands
- **Safe Defaults**: All commands default to safe values (stopped, braked)

## CAN Interface (CANBus_Specification.md v1.1)

The driver uses SocketCAN for communication and implements the current CAN bus specification:

- **0x001**: Joystick/remote controller 
- **0x001**: Current default command arbitration ID used by this stack and observed on the vehicle
- **0x100**: Historical/alternate control ID still mentioned in the CAN spec and older notes
- **0x2ff**: Tachometer data (receive only)

The launch file now exposes the command arbitration ID explicitly:

```bash
ros2 launch mtt_driver mtt_composable_system.launch.py can_interface:=can0 can_id:=1
```

`can_id:=1` corresponds to `0x001`.

```bash
sudo ip link set can0 up type can bitrate 250000
```

### Critical Safety Requirements

- **Security switch MUST be unlocked** for operation (controlled via dead man's switch)
- **Light state acts as emergency stop** (test both states to determine operational mode)
- **Direction is controlled by master system** (this driver)
- **Emergency stop** immediately stops all motion and applies maximum brake

### Tachometer Data

The driver automatically receives and processes tachometer data from the vehicle:
- **Speed calculation**: Real-time km/h from encoder ticks
- **Distance tracking**: Cumulative distance with ~2-3mm accuracy  
- **Temperature monitoring**: Two temperature sensors for diagnostics
- **Gear ratios**: Precisely calculated using manufacturer specifications

### Odometry Integration

The driver provides comprehensive odometry data compatible with ROS2 navigation:

#### Data Processing
- **Real-time speed calculation**: Uses manufacturer gear ratios and encoder data
- **Distance tracking**: Cumulative odometry with direction awareness
- **Standard ROS2 format**: Compatible with nav2 and SLAM algorithms
- **MSB-first parsing**: Correctly handles CAN 0x2FF byte order per specification

#### Frame IDs
- **odom**: Fixed odometry frame for navigation
- **base_link**: Default vehicle base frame for transforms

#### Integration with Navigation
The odometry data is published in standard format for integration with:
- **Nav2**: ROS2 navigation stack
- **SLAM algorithms**: Real-time mapping
- **TF2**: Transform broadcasting
- **Robot localization**: Sensor fusion

## Troubleshooting

### CAN Bus Issues
- Ensure the CAN interface is up and properly configured
- Check CAN bus connections and termination
- Verify the correct CAN interface name in the driver

### Joystick Issues
- Verify there is a corresponding yaml file in the config and that file is included in `joystick_mappings.yaml`
- Verify joystick is connected: `ls /dev/input/js*`
- Check joystick permissions: `sudo chmod a+rw /dev/input/js0`
- Test joystick input: `ros2 topic echo /joy`

### Build Issues
- Ensure all dependencies are installed
- Check that the workspace is properly sourced
- Verify Python path includes the package
