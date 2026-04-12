#!/usr/bin/env python3
"""MTT-154 ROS2 Wrapper: bridges ROS topics to low-level CAN driver."""

import rclpy
import time
import threading
import logging
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import Bool, UInt8
from mtt_msgs.msg import MttTachometerData, MttVehicleStatus, MttAuxCommand, MttDrivingMode
from mtt_interfaces.srv import SetVehiculeTypeSrv, GetVehiculeTypeSrv
from .mtt_driver import (
    MTTCanDriver,
    WinchState,
    DirectionState,
    SecuritySwitchState,
    LightState
)


class MTTRosWrapper(Node):
    """ROS2 node exposing cmd_vel + aux command control and tachometer/odometry outputs."""

    def __init__(self):
        super().__init__("mtt_ros_wrapper")
        self.declare_parameter("can_interface", "can0")
        self.declare_parameter("driver_log_level", "INFO")
        self.declare_parameter("control_frequency_hz", 50.0)
        self.declare_parameter("can_frame_frequency_hz", 20.0)
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("can_id", 0x001)
        self.declare_parameter("direction_switch_threshold", 0.05)
        self.declare_parameter("telemetry_timeout_seconds", 0.5)
        self.declare_parameter("command_timeout_seconds", 0.5)

        can_interface = self.get_parameter("can_interface").get_parameter_value().string_value
        driver_log_level_str = self.get_parameter("driver_log_level").get_parameter_value().string_value
        control_frequency_hz = self.get_parameter("control_frequency_hz").get_parameter_value().double_value
        can_frame_frequency_hz = self.get_parameter("can_frame_frequency_hz").get_parameter_value().double_value

        self.control_period = 1.0 / max(1e-3, control_frequency_hz)
        self.can_frame_period = 1.0 / max(1e-3, can_frame_frequency_hz)
        self.base_frame = self.get_parameter("base_frame").get_parameter_value().string_value
        self.can_id = int(self.get_parameter("can_id").get_parameter_value().integer_value)
        self.direction_switch_threshold = self.get_parameter("direction_switch_threshold").get_parameter_value().double_value
        self.telemetry_timeout_seconds = self.get_parameter("telemetry_timeout_seconds").get_parameter_value().double_value
        self.command_timeout_seconds = self.get_parameter("command_timeout_seconds").get_parameter_value().double_value

    
        driver_log_level = getattr(logging, driver_log_level_str.upper(), logging.INFO)


        self.get_logger().info(f"CAN interface: {can_interface}")

        try:
            self.driver = MTTCanDriver(
                can_interface,
                log_level=driver_log_level,
                can_id=self.can_id,
                telemetry_timeout_seconds=self.telemetry_timeout_seconds,
            )
            self.get_logger().info("MTT Driver initialized")
        except Exception as e:
            self.get_logger().fatal(f"Could not start driver: {e}")
            raise

        # Driver serialization lock
        self.driver_lock = threading.RLock()
        # Deadbands to prevent oscillating idle frames (tolerance)
        self.throttle_deadband = 0.345
        self.steer_deadband = 0.1

        self._last_aux_cmd = None
        
        # Current steering input for odometry feedback (raw value sent to CAN bus)
        self.current_steering_input = 0.0  # Raw steering input (-1.0 to +1.0)
        self.last_cmd_vel_time = None
        self.command_timeout_active = False
        self.teleop_estop_active = False
        self.teleop_estop_seen = False
        self.active_safety_locks = set()

        self.create_subscription(TwistStamped, "cmd_vel", self.cmd_vel_callback, 10)
        self.create_subscription(MttAuxCommand, "mtt_aux_cmd", self.aux_cmd_callback, 10)
        self.create_subscription(Bool, "teleop_estop", self.estop_callback, 10)
        
        # Single aggregated publisher for all vehicle status
        # Publishers for telemetry data
        self.tachometer_pub = self.create_publisher(MttTachometerData, "mtt_tachometer", 10)  # Pure odometry data
        self.vehicle_status_pub = self.create_publisher(MttVehicleStatus, "mtt_status", 10)   # High-level monitoring
        
        self.driving_mode_pub = self.create_publisher(MttDrivingMode, "mtt_driving_mode", 10)
        self.set_mode_srv = self.create_service(SetVehiculeTypeSrv, "mtt/set_driving_mode", self._srv_set_mode)
        self.get_mode_srv = self.create_service(GetVehiculeTypeSrv, "mtt/get_driving_mode", self._srv_get_mode)
        
        # Current driving mode (default: single trailer)
        self.current_driving_mode = 0  # SINGLE_TRAILER
        
        # Track current steering input for odometry feedback
        self.current_steering_input = 0.0
        
        # Keep only essential command feedback
        self.steer_pub = self.create_publisher(UInt8, "mtt_steer_cmd", 10)

        self.ctrl_timer = self.create_timer(self.control_period, self.control_loop)
        self.can_frame_timer = self.create_timer(self.can_frame_period, self.send_can_frame)

    def send_can_frame(self):
        with self.driver_lock:
            self.driver.send_can_frame()

    def _set_safety_lock(self, reason: str, active: bool):
        if active:
            if reason in self.active_safety_locks:
                return
            self.active_safety_locks.add(reason)
        else:
            if reason not in self.active_safety_locks:
                return
            self.active_safety_locks.remove(reason)
        self._sync_safety_switch()

    def _sync_safety_switch(self):
        desired_state = (
            SecuritySwitchState.SafetyLocked
            if self.active_safety_locks
            else SecuritySwitchState.SafetyUnlocked
        )
        with self.driver_lock:
            if self.driver.get_security_switch_state() == desired_state:
                return
            self.driver._set_security_switch(desired_state)

    def _describe_safety_state(self, driver_state: str) -> str:
        if not self.active_safety_locks:
            return driver_state
        reasons = ",".join(sorted(self.active_safety_locks))
        return f"{driver_state}:{reasons}"

    def estop_callback(self, msg: Bool):
        self.teleop_estop_seen = True
        self.teleop_estop_active = bool(msg.data)
        self._set_safety_lock("teleop_estop", self.teleop_estop_active)

    def cmd_vel_callback(self, msg: TwistStamped):
        lin = float(msg.twist.linear.x)
        ang = float(msg.twist.angular.z)

        if abs(lin) < self.throttle_deadband:
            lin = 0.0
        if abs(ang) < self.steer_deadband:
            ang = 0.0

        throttle_percent = min(1.0, abs(lin))

        # self.get_logger().info(f"throttle_percent:: {throttle_percent}")

        with self.driver_lock:
            # Origin/main behavior: set direction directly from cmd sign
            self.driver.set_throttle(throttle_percent)
            self.driver.set_steer(max(-1.0, min(1.0, ang)))
            direction = DirectionState.Forward if lin >= 0 else DirectionState.Reverse
            self.driver.set_direction(direction)
            
            # Store the actual steering input sent to CAN bus for odometry feedback
            # This is the raw input value (-1.0 to +1.0) that was sent to the hardware
            self.current_steering_input = max(-1.0, min(1.0, ang))
            self.last_cmd_vel_time = time.monotonic()
            self.command_timeout_active = False
            steer_raw = self.driver.get_command_state_snapshot()["steer_raw"]

        steer_msg = UInt8()
        steer_msg.data = steer_raw if steer_raw is not None else 0
        self.steer_pub.publish(steer_msg)

    def aux_cmd_callback(self, msg: MttAuxCommand):
        if msg != self._last_aux_cmd:
            self._last_aux_cmd = msg
            with self.driver_lock:
                self.driver.set_brake(msg.brake)
                if msg.light_state == 1:
                    self.driver.set_light_state(LightState.On)
                else:
                    self.driver.set_light_state(LightState.Off)
                    if msg.winch_command == 1:
                        self.driver.set_winch_state(WinchState.WinchIn)
                    elif msg.winch_command == 2:
                        self.driver.set_winch_state(WinchState.WinchOut)
                    else:
                        self.driver.set_winch_state(WinchState.WinchNeutral)

    def control_loop(self):
        """Main control loop - publishes tachometer data."""
        self._apply_command_timeout_if_needed()
        self._publish_vehicle_data()

    def _cmd_vel_is_fresh(self):
        if self.last_cmd_vel_time is None:
            return False
        if self.command_timeout_seconds <= 0.0:
            return True
        return (time.monotonic() - self.last_cmd_vel_time) <= self.command_timeout_seconds

    def _apply_command_timeout_if_needed(self):
        if self._cmd_vel_is_fresh() or self.last_cmd_vel_time is None:
            return
        if self.command_timeout_active:
            return

        with self.driver_lock:
            self.driver.set_throttle(0.0)
            self.driver.set_steer(0.0)
            self.current_steering_input = 0.0
            self.command_timeout_active = True

        self.get_logger().warning(
            f"No cmd_vel received for {self.command_timeout_seconds:.2f}s; neutralizing throttle and steering"
        )

    def _set_driving_mode(self, mode: int):
        """
        Change driving mode and notify odometry manager.
        
        Args:
            mode: Driving mode (0=SINGLE_TRAILER, 1=DUAL_DIFFERENTIAL, 2=DUAL_SERPENTINE)
        """
        if mode != self.current_driving_mode:
            self.get_logger().info(f"Changing driving mode from {self.current_driving_mode} to {mode}")
            
            # Update internal state
            self.current_driving_mode = mode
            
            # Publish the command so the odometry manager switches too
            mode_msg = MttDrivingMode()
            mode_msg.mode = mode
            # Optional: pass parameters here, e.g. "track=1.2 wheelbase=2.4"
            mode_msg.mode_parameters = ""
            self.driving_mode_pub.publish(mode_msg)
            
            # Mode-specific driver configuration can be added here if needed
            
            self.get_logger().info(f"Successfully changed to driving mode {mode}")
            return True
        return False

    def _srv_set_mode(self, request, response):
        """Service handler for setting driving mode using SetVehiculeTypeSrv."""
        try:
            # 1) Switch internal state
            ok = self._set_driving_mode(int(request.vehicule_type))
            
            # 2) Publish the topic for the Odometry Manager
            msg = MttDrivingMode()
            msg.mode = int(request.vehicule_type)
            msg.mode_parameters = ""  # Could be extended to support parameters
            self.driving_mode_pub.publish(msg)

            response.success = bool(ok)
            return response
        except Exception as e:
            self.get_logger().error(f"set_mode failed: {e}")
            response.success = False
            return response

    def _srv_get_mode(self, request, response):
        """Service handler for getting current driving mode using GetVehiculeTypeSrv."""
        try:
            cur = int(getattr(self, "current_driving_mode", 0))
            # Optional: mapping for human-readable names
            names = {0: "SINGLE_TRAILER", 1: "DUAL_DIFFERENTIAL", 2: "DUAL_SERPENTINE"}
            response.vehicule_type = cur
            response.type_name = names.get(cur, "UNKNOWN")
            return response
        except Exception as e:
            self.get_logger().error(f"get_mode failed: {e}")
            response.vehicule_type = 255  # Invalid value
            response.type_name = "ERROR"
            return response

    def _publish_vehicle_data(self):
        """Publish both tachometer data (for odometry) and vehicle status (for monitoring)."""
        # Use snapshot methods to avoid torn reads
        tach_data = self.driver.get_tachometer_snapshot()
        odometry_data = self.driver.get_odometry_snapshot()
        command_state = self.driver.get_command_state_snapshot()
        cmd_vel_is_fresh = self._cmd_vel_is_fresh()
        
        # Always publish vehicle status for safety monitoring, regardless of tachometer data
        status_msg = MttVehicleStatus()
        status_msg.header.stamp = self.get_clock().now().to_msg()
        status_msg.header.frame_id = self.base_frame
        
        # Motion data only when tachometer data is available
        if tach_data.new_data_available:
            status_msg.speed_ms = odometry_data["speed_ms"]
            status_msg.speed_kmh = odometry_data["speed_kmh"]
            status_msg.distance_km = odometry_data["absolute_distance_m"] / 1000.0
            status_msg.direction = odometry_data["direction"]
            status_msg.temperature_a = odometry_data["temperature_a"]
            status_msg.temperature_b = odometry_data["temperature_b"]
            status_msg.tachometer_instant = tach_data.tachometer_instant
            status_msg.tachometer_cumulative = tach_data.tachometer_cumulative
            
            # Publish pure tachometer data for odometry node
            tachometer_msg = MttTachometerData()
            tachometer_msg.header.stamp = self.get_clock().now().to_msg()
            tachometer_msg.header.frame_id = self.base_frame
            tachometer_msg.tachometer_instant = tach_data.tachometer_instant
            tachometer_msg.tachometer_cumulative = tach_data.tachometer_cumulative
            tachometer_msg.speed_ms = odometry_data["speed_ms"]
            tachometer_msg.speed_kmh = odometry_data["speed_kmh"]
            tachometer_msg.distance_km = odometry_data["absolute_distance_m"] / 1000.0
            tachometer_msg.direction = odometry_data["direction"]
            tachometer_msg.main_sensor_temp_a = odometry_data["temperature_a"]
            tachometer_msg.main_sensor_temp_b = odometry_data["temperature_b"]
            # Include current steering input for articulated vehicle odometry
            # This is the raw steering command (-1.0 to +1.0) sent to CAN bus
            tachometer_msg.steer_cmd = self.current_steering_input
            self.tachometer_pub.publish(tachometer_msg)
        else:
            # Set default values when no motion data is available
            status_msg.speed_ms = 0.0
            status_msg.speed_kmh = 0.0
            status_msg.distance_km = 0.0
            status_msg.direction = "Unknown"
            status_msg.temperature_a = 0.0
            status_msg.temperature_b = 0.0
            status_msg.tachometer_instant = 0
            status_msg.tachometer_cumulative = 0
        
        status_msg.steer_position = command_state["steer_raw"]
        status_msg.emergency_stop_active = bool(self.active_safety_locks) or (
            command_state["safety_state"] == SecuritySwitchState.SafetyLocked.name
        )
        status_msg.remote_connected = False
        status_msg.deadman_active = self.teleop_estop_seen and not self.teleop_estop_active
        status_msg.safety_state = self._describe_safety_state(command_state["safety_state"])
        self.vehicle_status_pub.publish(status_msg)

    def destroy_node(self):
        """Clean shutdown with emergency stop."""
        self.get_logger().info("Shutting down MTT driver - applying emergency stop")

        # Cancel timers first to prevent race conditions
        for timer in (
            getattr(self, "can_frame_timer", None),
            getattr(self, "ctrl_timer", None),
        ):
            if timer:
                timer.cancel()

        if hasattr(self, "driver") and self.driver:
            with self.driver_lock:
                self.driver.emergency_stop()
                self.driver.send_can_frame()
                self.driver.cleanup()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    wrapper_node = MTTRosWrapper()
    try:
        rclpy.spin(wrapper_node)
    except KeyboardInterrupt:
        pass
    finally:
        wrapper_node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
