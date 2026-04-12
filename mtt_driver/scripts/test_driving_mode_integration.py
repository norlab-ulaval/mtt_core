#!/usr/bin/env python3
"""
Integration test for MTT-154 driving mode switching.
Tests the complete flow: Wrapper -> MttDrivingMode topic -> Odometry Manager
"""

import rclpy
from rclpy.node import Node
from mtt_msgs.msg import MttDrivingMode
from mtt_interfaces.srv import SetVehiculeTypeSrv, GetVehiculeTypeSrv
from std_srvs.srv import Trigger
import time


class DrivingModeIntegrationTest(Node):
    def __init__(self):
        super().__init__('driving_mode_test')

        self.declare_parameter('set_mode_service', 'mtt/set_driving_mode')
        self.declare_parameter('get_mode_service', 'mtt/get_driving_mode')
        self.declare_parameter('reset_odometry_service', 'mtt/reset_odometry')

        set_mode_service = self.get_parameter('set_mode_service').value
        get_mode_service = self.get_parameter('get_mode_service').value
        reset_odometry_service = self.get_parameter('reset_odometry_service').value

        self.set_mode_cli = self.create_client(SetVehiculeTypeSrv, set_mode_service)
        self.get_mode_cli = self.create_client(GetVehiculeTypeSrv, get_mode_service)
        self.reset_odom_cli = self.create_client(Trigger, reset_odometry_service)
        
        # Wait for services
        self.get_logger().info("Waiting for services to be available...")
        
        services = [
            (self.set_mode_cli, set_mode_service),
            (self.get_mode_cli, get_mode_service),
            (self.reset_odom_cli, reset_odometry_service)
        ]
        
        for client, name in services:
            if not client.wait_for_service(timeout_sec=5.0):
                self.get_logger().error(f"Service {name} not available!")
                return
            else:
                self.get_logger().info(f"Service {name} is available")

    def test_mode_switching(self):
        """Test driving mode switching with service calls."""
        
        # Test 1: Get current mode
        self.get_logger().info("=== Test 1: Get current mode ===")
        request = GetVehiculeTypeSrv.Request()
        future = self.get_mode_cli.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            response = future.result()
            self.get_logger().info(f"Current mode: {response.vehicule_type} ({response.type_name})")
        else:
            self.get_logger().error("Failed to get current mode")
            return False

        # Test 2: Switch to Dual Differential (mode 1)
        self.get_logger().info("=== Test 2: Switch to Dual Differential ===")
        request = SetVehiculeTypeSrv.Request()
        request.vehicule_type = 1  # DUAL_DIFFERENTIAL
        future = self.set_mode_cli.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            response = future.result()
            self.get_logger().info(f"Set mode result: {response.success}")
        else:
            self.get_logger().error("Failed to set mode to Dual Differential")
            return False

        time.sleep(1)  # Allow propagation

        # Test 3: Verify mode change
        self.get_logger().info("=== Test 3: Verify mode change ===")
        request = GetVehiculeTypeSrv.Request()
        future = self.get_mode_cli.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            response = future.result()
            self.get_logger().info(f"New mode: {response.vehicule_type} ({response.type_name})")
            if response.vehicule_type == 1:
                self.get_logger().info("✓ Mode change successful!")
            else:
                self.get_logger().error("✗ Mode change failed!")
                return False
        else:
            self.get_logger().error("Failed to verify mode change")
            return False

        # Test 4: Switch to Dual Serpentine (mode 2)
        self.get_logger().info("=== Test 4: Switch to Dual Serpentine ===")
        request = SetVehiculeTypeSrv.Request()
        request.vehicule_type = 2  # DUAL_SERPENTINE
        future = self.set_mode_cli.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            response = future.result()
            self.get_logger().info(f"Set mode result: {response.success}")
        else:
            self.get_logger().error("Failed to set mode to Dual Serpentine")
            return False

        time.sleep(1)  # Allow propagation

        # Test 5: Reset odometry
        self.get_logger().info("=== Test 5: Reset odometry ===")
        request = Trigger.Request()
        future = self.reset_odom_cli.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            response = future.result()
            self.get_logger().info(f"Reset odometry: {response.success} - {response.message}")
        else:
            self.get_logger().error("Failed to reset odometry")
            return False

        # Test 6: Return to Single Trailer (mode 0)
        self.get_logger().info("=== Test 6: Return to Single Trailer ===")
        request = SetVehiculeTypeSrv.Request()
        request.vehicule_type = 0  # SINGLE_TRAILER
        future = self.set_mode_cli.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            response = future.result()
            self.get_logger().info(f"Set mode result: {response.success}")
        else:
            self.get_logger().error("Failed to set mode to Single Trailer")
            return False

        self.get_logger().info("=== All tests completed successfully! ===")
        return True


def main():
    rclpy.init()
    
    test_node = DrivingModeIntegrationTest()
    
    # Run tests
    if test_node.test_mode_switching():
        test_node.get_logger().info("✓ Integration test PASSED")
    else:
        test_node.get_logger().error("✗ Integration test FAILED")
    
    test_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
