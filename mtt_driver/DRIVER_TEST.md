# MTT Driver Initialization Test

This test launch file is designed to test the MTT driver initialization and basic frame generation **without** the ROS wrapper.

## Purpose

- Test the basic initialization of the MTT CAN driver
- Verify initial frame generation
- Check that all driver state variables are properly initialized
- Test thread startup and shutdown
- Validate CAN frame transmission without ROS dependencies

## Usage

### Quick Test (with automatic vcan setup)

```bash
# Build the package first
cd /home/robot/mtt_project/mtt_ws
colcon build --packages-select mtt_driver

# Source the workspace
source install/setup.bash

# Run the test with automatic virtual CAN setup
ros2 launch mtt_driver mtt_driver_init_test.launch.py
```

### Manual CAN Interface Setup

If you want to use a different CAN interface or set it up manually:

```bash
# Setup virtual CAN interface manually
sudo modprobe vcan
sudo ip link add dev vcan0 type vcan
sudo ip link set up vcan0

# Run test without automatic setup
ros2 launch mtt_driver mtt_driver_init_test.launch.py setup_vcan:=false

# Or use a different CAN interface
ros2 launch mtt_driver mtt_driver_init_test.launch.py can_interface:=can1 setup_vcan:=false
```

### Direct Script Execution

You can also run the test script directly:

```bash
# Using default vcan0
ros2 run mtt_driver mtt_test_node

# Using specific CAN interface
ros2 run mtt_driver mtt_test_node can1
```

## What the Test Does

1. **Interface Check**: Verifies the CAN interface exists
2. **Driver Initialization**: Creates MTTCanDriver instance with test parameters
3. **Frame Verification**: Checks initial CAN frame content
4. **State Validation**: Verifies all driver state variables
5. **Thread Testing**: Starts sender/listener threads
6. **Frame Transmission**: Sends frames for 3 seconds
7. **Clean Shutdown**: Properly stops the driver

## Expected Output

The test should show:
- Successful CAN interface detection
- Driver initialization confirmation
- Initial CAN frame in hex format
- Driver state variables (direction, winch, etc.)
- Live frame transmission
- Clean shutdown

## Differences from Full System

This test runs **only** the low-level CAN driver (`MTTCanDriver`) without:
- ROS wrapper (`MTTRosWrapper`)
- ROS topics (cmd_vel, mtt_aux_cmd, etc.)
- Odometry publishing
- Parameter server integration

This isolation helps debug driver-level issues without ROS complexity.

# MTT Driver Standalone Initialization Test

This directory contains tools for testing the MTT driver initialization frame without the ROS wrapper.

## Files Created

### 1. `scripts/test_driver_init.py`
A standalone Python script that tests the MTT driver initialization and basic functionality:
- Tests driver initialization on a specified CAN interface
- Verifies initial CAN frame generation
- Checks initial state values
- Monitors frame transmission for 3 seconds
- Performs clean shutdown

### 2. `launch/mtt_driver_standalone_test.launch.py`
A ROS2 launch file that runs the standalone test:
- Optionally sets up virtual CAN interface (vcan0)
- Runs the driver initialization test
- Provides detailed logging

## Usage

### Running the standalone script directly:
```bash
cd /home/robot/mtt_project/mtt_ws/src/mtt_driver/mtt_driver
python3 ../scripts/test_driver_init.py [CAN_INTERFACE]
```

### Running via ROS2 launch:
```bash
# Basic test (uses vcan0, no automatic setup)
ros2 launch mtt_driver mtt_driver_standalone_test.launch.py

# With automatic vcan0 setup
ros2 launch mtt_driver mtt_driver_standalone_test.launch.py setup_vcan:=true

# Custom CAN interface (manual setup required)
ros2 launch mtt_driver mtt_driver_standalone_test.launch.py can_interface:=can0 setup_vcan:=false
```

## Test Output

The test performs the following checks:

1. **Basic driver initialization** - Verifies the driver can connect to the CAN interface
2. **Initial CAN frame** - Shows the hexadecimal representation of the initial frame
3. **Initial state values** - Displays all driver state variables
4. **Thread verification** - Confirms sender/listener threads start automatically
5. **Frame monitoring** - Shows 6 consecutive frames being sent
6. **Clean shutdown** - Verifies proper cleanup

### Expected Initial Frame
```
Initial frame: 00 60 00 7F 00 7F 01 00
```

Which in decimal is: `[0, 96, 0, 127, 0, 128, 0, 0]`

Where:
- `00`: Vehicle type (single track)
- `60`: Global switches (0x60 with direction bit cleared for reverse)
- `00`: Throttle (idle)
- `7F`: Winch (neutral - 127 decimal)
- `00`: Brake (idle)
- `7F`: Steer (center - 127 decimal)
- `01`: Direction mode (close loop)
- `00`: Reserved byte

## Key Benefits

- **No ROS wrapper dependency** - Tests the core driver logic only
- **Standalone operation** - Can run independently of ROS infrastructure  
- **Clear verification** - Shows exact initialization frame and state values
- **Automated setup** - Optional virtual CAN interface creation
- **Clean testing** - Proper initialization and cleanup cycle
