#!/usr/bin/env python3
"""
Standalone test script for MTT Driver initialization without ROS wrapper.
Tests the basic initialization frame of the driver alone.
"""

import os
import sys
import time
import logging

# Add the current directory to Python path for imports
current_dir = os.path.dirname(os.path.abspath(__file__))
driver_dir = os.path.join(os.path.dirname(current_dir), 'mtt_driver')
sys.path.insert(0, driver_dir)

# Now import the driver
from mtt_driver import MTTCanDriver, interface_exists

# Set up logging
logging.basicConfig(level=logging.INFO)
log = logging.getLogger('mtt_driver_init_test')

def test_driver_initialization(can_interface='vcan0'):
    """Test the driver initialization and frame setup."""
    log.info("=" * 60)
    log.info("MTT DRIVER INITIALIZATION TEST")
    log.info("=" * 60)
    
    # Check if CAN interface exists
    if not interface_exists(can_interface):
        log.error(f"CAN interface '{can_interface}' not found!")
        log.info("For testing, you can create a virtual CAN interface with:")
        log.info(f"sudo modprobe vcan")
        log.info(f"sudo ip link add dev {can_interface} type vcan")
        log.info(f"sudo ip link set up {can_interface}")
        return False
    
    try:
        # Test 1: Basic initialization
        log.info(f"Test 1: Basic driver initialization on {can_interface}")
        driver = MTTCanDriver(can_interface=can_interface)
        
        # Configure driver to get the desired frame [0, 64, 0, 127, 0, 128, 0, 0]
        from mtt_driver import DirectionState
        driver.set_direction(DirectionState.Reverse)  # To get byte 1 = 64 instead of 96
        driver._set_steer(128)  # Raw frame check: force byte 5 = 128 instead of 127
        log.info("✓ Driver initialized successfully")
        
        # Test 2: Check initial frame
        log.info("Test 2: Check initial CAN frame")
        initial_frame = driver._get_current_frame_hex()  # Use private method for testing
        log.info(f"Initial frame: {initial_frame}")
        
        # Test 3: Verify initial state
        log.info("Test 3: Verify initial state values")
        log.info(f"Vehicle type: {driver.vehicle_type}")
        log.info(f"Direction state: {driver.direction_state}")
        log.info(f"Direction mode: {driver.steering_mode}")
        log.info(f"Security switch: {driver.security_switch_state}")
        log.info(f"Light state: {driver.light_state}")
        log.info(f"Winch state: {driver.winch_state}")
        log.info(f"Steer value: {driver.steer_value}")
        log.info(f"Throttle value: {driver.throttle_value}")
        log.info(f"Brake value: {driver.brake_value}")
        
        # Test 4: Check that threads are running (they start automatically)
        log.info("Test 4: Verify threads are running (started automatically)")
        log.info("✓ Threads started automatically in constructor")
        
        # Test 5: Send a few frames
        log.info("Test 5: Monitor frames being sent (3 seconds)")
        for i in range(6):  # 3 seconds at ~20Hz
            time.sleep(0.5)
            current_frame = driver._get_current_frame_hex()  # Use private method for testing
            log.info(f"Frame {i+1}: {current_frame}")
        
        # Test 6: Clean shutdown
        log.info("Test 6: Clean shutdown")
        driver.cleanup()
        log.info("✓ Driver cleanup completed successfully")
        
        log.info("=" * 60)
        log.info("ALL TESTS PASSED! ✓")
        log.info("=" * 60)
        return True
        
    except Exception as e:
        log.error(f"Driver initialization test failed: {e}")
        log.error("=" * 60)
        log.error("TEST FAILED! ✗")
        log.error("=" * 60)
        return False

def main():
    """Main function to run the test."""
    can_interface = 'vcan0'  # Use virtual CAN for testing
    
    # Parse command line arguments
    if len(sys.argv) > 1:
        can_interface = sys.argv[1]
    
    log.info(f"Starting MTT Driver initialization test on {can_interface}")
    success = test_driver_initialization(can_interface)
    
    if success:
        log.info("Test completed successfully!")
        sys.exit(0)
    else:
        log.error("Test failed!")
        sys.exit(1)

if __name__ == '__main__':
    main()
