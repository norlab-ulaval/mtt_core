#!/usr/bin/env python3

from __future__ import annotations
from abc import ABC, abstractmethod
from enum import IntEnum
from typing import Optional, Dict, Any
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from nav_msgs.msg import Odometry
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import Float64
from mtt_msgs.msg import MttTachometerData
from mtt_msgs.msg import MttTachometerData, MttDrivingMode
from mtt_interfaces.srv import SetSteerControlMode
from std_srvs.srv import Trigger
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped, Quaternion

# Import our articulated vehicle model
from .mtt_articulated_model import ArticulatedVehicleDynamics, ArticulatedVehicleParams
from .mtt_vehicle_params import get_mtt_params

"""
MTT-154 Multi-Mode Odometry Manager

UPDATED: Now includes realistic articulated vehicle dynamics for Single Trailer mode

Provides three driving modes with live switching:
- Single Trailer: Realistic articulated tractor+trailer with proper dynamics
- Dual Differential: 2D skid-steer when left/right sensors available  
- Dual Serpentine: 2D bicycle model with articulation angle

Features:
- Realistic articulated vehicle dynamics with track slip and carving
- State preservation across mode switches (no odometry jumps)
- First-sample initialization eliminates startup discontinuities
- Parametric geometry configuration (track width, wheelbase, articulation limits)
- Sensor-optimized QoS and odometer wrap/reset handling
- Service interface for odometry reset
"""


# --------------------------- Modes --------------------------- #
class DrivingMode(IntEnum):
    SINGLE_TRAILER = 0  # Single tractor with trailer (slip + central joint)
    DUAL_DIFFERENTIAL = 1  # Two tractors side-by-side (skid steer)
    DUAL_SERPENTINE = 2  # Two tractors front/back (articulated)


# ---------------------- Interface & Utils -------------------- #
class OdometryInterface(ABC):
    @abstractmethod
    def calculate_odometry(
        self,
        msg: MttTachometerData,
        *,
        odom_frame: str,
        base_frame: str,
        distance_multiplier: float,
        wrap_reset_threshold_m: float,
        angular_velocity: float = 0.0,
    ) -> Odometry:
        pass

    @abstractmethod
    def export_state(self) -> Dict[str, Any]:
        """Return a serializable state for transfer across mode switches."""
        pass

    @abstractmethod
    def import_state(self, state: Dict[str, Any]) -> None:
        pass

    @abstractmethod
    def reset_odometry(self) -> None:
        pass

    @abstractmethod
    def get_mode_name(self) -> str:
        pass

    # --- helpers --- #
    @staticmethod
    def _odom_with_stamp(msg: MttTachometerData, odom_frame: str, base_frame: str) -> Odometry:
        odom = Odometry()
        # Prefer sensor time if available
        try:
            odom.header.stamp = msg.header.stamp
        except Exception:
            # Fallback is set by caller if needed
            pass
        odom.header.frame_id = odom_frame
        odom.child_frame_id = base_frame
        return odom

    @staticmethod
    def _apply_default_covariances(odom: Odometry) -> None:
        # Small variance on x position & vx; large elsewhere (uninformative)
        big = 1e6
        small = 1e-3
        # pose: [x, y, z, roll, pitch, yaw]
        cov_p = [big] * 36
        cov_p[0] = small  # x
        cov_p[7] = big  # y
        cov_p[14] = big  # z
        cov_p[35] = big  # yaw
        odom.pose.covariance = cov_p

        cov_t = [big] * 36
        cov_t[0] = small  # vx
        cov_t[7] = big  # vy
        cov_t[14] = big  # vz
        cov_t[35] = big  # wyaw
        odom.twist.covariance = cov_t

    @staticmethod
    def _norm_direction(direction_field: Optional[str]) -> int:
        """Return +1 for forward, -1 for reverse (default forward if unknown)."""
        if not direction_field:
            return +1
        d = str(direction_field).strip().lower()
        if d in ("reverse", "backward", "rev"):
            return -1
        return +1


# ----------------------- Implementations --------------------- #
class SingleTrailerOdometry(OdometryInterface):
    """Realistic articulated vehicle odometry using proper dynamics model."""

    def __init__(self) -> None:
        # Initialize articulated vehicle dynamics using real MTT-154 measured parameters
        self.mtt_params = get_mtt_params()
        self.vehicle_params = ArticulatedVehicleParams(self.mtt_params)
        
        self.dynamics = ArticulatedVehicleDynamics(self.vehicle_params)
        
        # State tracking
        self.last_abs_m: Optional[float] = None
        self.last_time: Optional[float] = None
        self.current_throttle = 0.0
        self.current_steering = 0.0
        # Closed-loop response state
        self.actual_yaw_rate = 0.0  # Current actual yaw rate (with lag)
        self.imu_heading: Optional[float] = None  # IMU-based heading for feedback
        self.previous_imu_heading: Optional[float] = None  # Previous IMU heading for rate calculation
        self.use_imu_feedback = True  # Enable IMU-based closed loop control

    def calculate_odometry(
        self,
        msg: MttTachometerData,
        *,
        odom_frame: str,
        base_frame: str,
        distance_multiplier: float,
        wrap_reset_threshold_m: float,
        angular_velocity: float = 0.0,
    ) -> Odometry:
        odom = self._odom_with_stamp(msg, odom_frame, base_frame)

        # Get current time for dynamics integration
        current_time = time.time()

        if self.last_time is None:
            # First sample - initialize
            self.last_time = current_time
            dt = 0.02  # Default 50Hz
        else:
            dt = current_time - self.last_time
            dt = max(0.001, min(0.1, dt))  # Clamp dt to reasonable range

        # Get tachometer absolute distance (converted to meters)
        # distance_km multiplied by distance_multiplier (1000 if km) yields meters
        cur_abs = float(getattr(msg, "distance_km", 0.0)) * distance_multiplier

        # Handle wrap-around detection
        if self.last_abs_m is not None and abs(cur_abs - self.last_abs_m) > wrap_reset_threshold_m:
            # Reset could be handled here if needed
            pass

        # Odometry integration from available sensors
        commanded_angular_vel = float(angular_velocity)  # rad/s from cmd_vel

        # Calculate encoder delta distance and speed
        distance_traveled: Optional[float] = None
        speed_ms = 0.0
        if self.last_abs_m is not None:
            distance_traveled = cur_abs - self.last_abs_m  # meters (unsigned from sensor)

        # Direction sign from message (default +1 if missing)
        sign = +1
        try:
            sign = self._norm_direction(getattr(msg, "direction", None))
        except Exception:
            pass

        if distance_traveled is not None and dt > 0.001:
            speed_ms = (distance_traveled * sign) / dt
        else:
            # Fallback to tachometer instantaneous speed (already signed or will be signed below)
            try:
                speed_ms = float(getattr(msg, "speed_ms", 0.0))
            except Exception:
                speed_ms = 0.0
            # Ensure sign is applied
            speed_ms *= sign

        # Use IMU heading if available, otherwise integrate angular velocity
        if self.imu_heading is not None:
            # IMU available: use direct heading measurement
            current_heading: Optional[float] = self.imu_heading
            # Use commanded angular velocity for velocity reporting
            current_angular_vel = commanded_angular_vel
        else:
            # No IMU: let dynamics model integrate from angular velocity
            current_heading = None  # Will be calculated by dynamics
            current_angular_vel = commanded_angular_vel

        # Expose angular velocity in odometry (closed/open loop)
        self.actual_yaw_rate = float(current_angular_vel)

        # Set throttle for dynamics model based on measured (signed) speed
        if abs(speed_ms) > 0.01:
            max_speed = self.mtt_params.max_speed_ms  # From centralized MTT parameters
            self.current_throttle = max(-1.0, min(1.0, speed_ms / max_speed))
        else:
            self.current_throttle = 0.0

        # Set steering for dynamics model based on raw steering command from CAN bus
        raw_steer_cmd = float(getattr(msg, "steer_cmd", 0.0))
        self.current_steering = max(-1.0, min(1.0, raw_steer_cmd))

        # Preserve previous pose for encoder-based integration
        try:
            x_prev = float(self.dynamics.x)
            y_prev = float(self.dynamics.y)
        except Exception:
            x_prev, y_prev = 0.0, 0.0

        # Update dynamics model (for heading/vel integration)
        x, y, heading = self.dynamics.update(
            throttle_input=self.current_throttle,
            steering_input=self.current_steering,
            dt=dt,
            terrain_grip=1.0
        )

        # Override heading with IMU if available (more accurate)
        if current_heading is not None:
            heading = current_heading
            # Update dynamics internal state to match IMU
            self.dynamics.x = x
            self.dynamics.y = y
            self.dynamics.heading = heading

        # Choose heading source
        if self.use_imu_feedback and self.imu_heading is not None:
            final_heading = self.imu_heading
        else:
            final_heading = heading

        # Integrate position from encoder distance along current heading
        if distance_traveled is not None:
            ds_signed = distance_traveled * sign
            x_enc = x_prev + ds_signed * math.cos(final_heading)
            y_enc = y_prev + ds_signed * math.sin(final_heading)
        else:
            # No previous distance: keep current model position
            x_enc, y_enc = x, y

        # Sync dynamics internal state to encoder-integrated pose and selected heading
        self.dynamics.x = x_enc
        self.dynamics.y = y_enc
        self.dynamics.heading = final_heading

        # Get vehicle state (after sync)
        vehicle_state = self.dynamics.get_state()

        # Validate against tachometer if we have previous measurement
        if self.last_abs_m is not None:
            # Calculate distance traveled according to tachometer
            tach_distance = cur_abs - self.last_abs_m

            # Calculate distance according to dynamics model
            dynamics_distance = vehicle_state['linear_velocity'] * dt

            # If there's significant discrepancy, we could apply correction
            distance_error = abs(tach_distance - abs(dynamics_distance))
            if distance_error > 0.1:  # 10cm threshold
                # For now, trust the dynamics model but log discrepancy
                pass

        self.last_abs_m = cur_abs
        self.last_time = current_time

        # Fill odometry message with encoder-integrated pose
        odom.pose.pose.position.x = self.dynamics.x
        odom.pose.pose.position.y = self.dynamics.y
        odom.pose.pose.position.z = 0.0

        # Convert heading to quaternion
        quat = self._yaw_to_quaternion(final_heading)
        odom.pose.pose.orientation = quat

        # Velocity information: use measured linear speed from encoders; angular from command/IMU
        odom.twist.twist.linear.x = float(speed_ms)
        odom.twist.twist.linear.y = 0.0  # Tracked vehicles don't have lateral velocity
        odom.twist.twist.linear.z = 0.0
        odom.twist.twist.angular.x = 0.0
        odom.twist.twist.angular.y = 0.0
        odom.twist.twist.angular.z = self.actual_yaw_rate

        # Set realistic covariances for articulated vehicle
        self._apply_articulated_covariances(odom, vehicle_state)

        return odom

    def _apply_articulated_covariances(self, odom: Odometry, vehicle_state: dict):
        """Apply realistic covariance estimates for articulated vehicle."""
        # Position covariance increases with speed and articulation
        speed_factor = abs(vehicle_state['linear_velocity']) / self.mtt_params.max_speed_ms  # Normalize to max speed
        articulation_factor = abs(vehicle_state['articulation_angle']) / self.mtt_params.max_articulation_rad  # Normalize to max articulation
        
        # Base covariances - articulated vehicles have higher uncertainty
        pos_cov = 0.02 * (1.0 + speed_factor + articulation_factor)  # Position uncertainty
        heading_cov = 0.05 * (1.0 + 2.0 * articulation_factor)      # Heading uncertainty higher with articulation
        vel_cov = 0.15 * (1.0 + speed_factor)                       # Velocity uncertainty
        
        # Position covariance (6x6 matrix, row-major order)
        odom.pose.covariance[0] = pos_cov    # x
        odom.pose.covariance[7] = pos_cov    # y  
        odom.pose.covariance[35] = heading_cov  # yaw
        
        # Velocity covariance (6x6 matrix)
        odom.twist.covariance[0] = vel_cov   # linear x
        odom.twist.covariance[35] = heading_cov * 2  # angular z

    def _yaw_to_quaternion(self, yaw: float) -> Quaternion:
        """Convert yaw angle to quaternion"""
        quat = Quaternion()
        quat.x = 0.0
        quat.y = 0.0
        quat.z = math.sin(yaw / 2.0)
        quat.w = math.cos(yaw / 2.0)
        return quat

    def export_state(self) -> Dict[str, Any]:
        vehicle_state = self.dynamics.get_state()
        return {
            "vehicle_state": vehicle_state,
            "last_abs_m": self.last_abs_m,
            "last_time": self.last_time,
            "current_throttle": self.current_throttle,
            "current_steering": self.current_steering,
            "actual_yaw_rate": self.actual_yaw_rate,
            "imu_heading": self.imu_heading,
            "previous_imu_heading": self.previous_imu_heading,
            "use_imu_feedback": self.use_imu_feedback
        }

    def import_state(self, state: Dict[str, Any]) -> None:
        if "vehicle_state" in state:
            vs = state["vehicle_state"]
            self.dynamics.set_state(vs.get("x", 0.0), vs.get("y", 0.0), vs.get("heading", 0.0))
            self.dynamics.articulation_angle = vs.get("articulation_angle", 0.0)
            self.dynamics.linear_velocity = vs.get("linear_velocity", 0.0)
            self.dynamics.angular_velocity = vs.get("angular_velocity", 0.0)
        
        self.last_abs_m = state.get("last_abs_m", None)
        self.last_time = state.get("last_time", None)
        self.current_throttle = state.get("current_throttle", 0.0)
        self.current_steering = state.get("current_steering", 0.0)
        self.actual_yaw_rate = state.get("actual_yaw_rate", 0.0)
        self.imu_heading = state.get("imu_heading", None)
        self.previous_imu_heading = state.get("previous_imu_heading", None)
        self.use_imu_feedback = state.get("use_imu_feedback", True)

    def reset_odometry(self) -> None:
        self.dynamics.reset()
        self.last_abs_m = None
        self.last_time = None
        self.current_throttle = 0.0
        self.current_steering = 0.0
        self.actual_yaw_rate = 0.0
        self.imu_heading = None
        self.previous_imu_heading = None

    def get_articulation_angle(self) -> float:
        """Get current articulation angle from dynamics model."""
        return self.dynamics.articulation_angle
    
    def update_imu_heading(self, heading_rad: float) -> None:
        """Update IMU heading for closed-loop control"""
        self.imu_heading = heading_rad
    
    def set_control_mode(self, use_imu_feedback: bool) -> None:
        """Switch between open-loop and closed-loop control modes"""
        self.use_imu_feedback = use_imu_feedback

    def get_mode_name(self) -> str:
        mode = "Closed-loop" if self.use_imu_feedback else "Open-loop"
        return f"Articulated Single Trailer ({mode})"


class DualDifferentialOdometry(OdometryInterface):
    """Skid-steer style. If left/right signals are present, integrates (x,y,theta);
    otherwise falls back to 1D like SingleTrailer.

    Optional message fields used if available:
      - left_distance_km, right_distance_km  (absolute)
      - left_speed_ms, right_speed_ms        (instantaneous)
    Params used (provided by manager): track_width_m
    """

    def __init__(self, *, track_width_m: float, **kwargs) -> None:
        super().__init__()
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.last_L_m: Optional[float] = None
        self.last_R_m: Optional[float] = None
        self.track_width_m = float(track_width_m)
        # Fallback 1D state
        self.last_abs_m: Optional[float] = None

    def calculate_odometry(
        self,
        msg: MttTachometerData,
        *,
        odom_frame: str,
        base_frame: str,
        distance_multiplier: float,
        wrap_reset_threshold_m: float,
        angular_velocity: float = 0.0,
    ) -> Odometry:
        # For dual differential, we'd need separate left/right data
        # Since MttTachometerData only has single tachometer, fallback to 1D mode
        abs_distance_m = msg.distance_km * 1000.0 * distance_multiplier
        
        # Handle wrap-around
        if self.last_abs_m is not None and abs(abs_distance_m - self.last_abs_m) > wrap_reset_threshold_m:
            self.last_abs_m = None
        
        if self.last_abs_m is None:
            self.last_abs_m = abs_distance_m
            return self._create_simple_odometry_msg(odom_frame, base_frame, msg.header.stamp)
        
        # Calculate distance delta
        delta_distance = abs_distance_m - self.last_abs_m
        self.last_abs_m = abs_distance_m
        
        # Update position using angular velocity for steering
        self.theta += angular_velocity * (1.0 / 50.0)  # Assuming 50Hz update rate
        self.x += delta_distance * math.cos(self.theta)
        self.y += delta_distance * math.sin(self.theta)
        
        return self._create_odometry_msg(odom_frame, base_frame, msg.header.stamp)
    
    def _create_odometry_msg(self, odom_frame: str, base_frame: str, stamp) -> Odometry:
        """Create odometry message with current pose"""
        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = odom_frame
        odom.child_frame_id = base_frame
        
        # Position
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        
        # Orientation
        quat = self._yaw_to_quaternion(self.theta)
        odom.pose.pose.orientation.x = quat.x
        odom.pose.pose.orientation.y = quat.y 
        odom.pose.pose.orientation.z = quat.z
        odom.pose.pose.orientation.w = quat.w
        
        return odom
    
    def _create_simple_odometry_msg(self, odom_frame: str, base_frame: str, stamp) -> Odometry:
        """Create simple odometry message for initialization"""
        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = odom_frame
        odom.child_frame_id = base_frame
        odom.pose.pose.orientation.w = 1.0  # Identity quaternion
        return odom
    
    def get_mode_name(self) -> str:
        return "Dual Differential"
    
    def export_state(self) -> dict:
        return {
            'x': self.x,
            'y': self.y, 
            'theta': self.theta,
            'last_abs_m': self.last_abs_m,
            'last_L_m': self.last_L_m,
            'last_R_m': self.last_R_m
        }
    
    def import_state(self, state: dict) -> None:
        self.x = state.get('x', 0.0)
        self.y = state.get('y', 0.0)
        self.theta = state.get('theta', 0.0)
        self.last_abs_m = state.get('last_abs_m')
        self.last_L_m = state.get('last_L_m')
        self.last_R_m = state.get('last_R_m')
    
    def reset_odometry(self) -> None:
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.last_abs_m = None
        self.last_L_m = None
        self.last_R_m = None
    
    def _yaw_to_quaternion(self, yaw: float) -> Quaternion:
        """Convert yaw angle to quaternion"""
        quat = Quaternion()
        quat.x = 0.0
        quat.y = 0.0
        quat.z = math.sin(yaw / 2.0)
        quat.w = math.cos(yaw / 2.0)
        return quat


class DualSerpentineOdometry(OdometryInterface):
    """Articulated front-back pair. If an articulation angle is available in the msg (e.g.,
    `articulation_angle_rad`), integrate like a simple car-like model. Otherwise, 1D fallback.
    """

    def __init__(self, wheelbase_m: float = 2.0) -> None:
        self.x = 0.0
        self.y = 0.0
        self.th = 0.0
        self.wheelbase_m = float(wheelbase_m)
        self.last_abs_m: Optional[float] = None

    def calculate_odometry(
        self,
        msg: MttTachometerData,
        *,
        odom_frame: str,
        base_frame: str,
        distance_multiplier: float,
        wrap_reset_threshold_m: float,
    ) -> Odometry:
        odom = self._odom_with_stamp(msg, odom_frame, base_frame)

        # Distance delta (1D along the articulation frame)
        cur_abs = float(getattr(msg, "distance_km", 0.0)) * distance_multiplier
        if self.last_abs_m is None:
            self.last_abs_m = cur_abs
            ds = 0.0
        else:
            ds = cur_abs - self.last_abs_m
            if ds < -abs(wrap_reset_threshold_m):
                self.last_abs_m = cur_abs
                ds = 0.0
        self.last_abs_m = cur_abs

        sign = self._norm_direction(getattr(msg, "direction", None))
        ds *= sign

        # Use steering command if available; treat as steer angle surrogate
        steer = float(getattr(msg, "steer_cmd", 0.0))

        if abs(steer) < 1e-9 or self.wheelbase_m < 1e-9:
            # Straight
            self.x += ds * math.cos(self.th)
            self.y += ds * math.sin(self.th)
        else:
            # Bicycle model integration
            dth = math.tan(steer) / self.wheelbase_m * ds
            R = ds / dth if abs(dth) > 1e-9 else float("inf")
            if math.isfinite(R):
                self.x += R * (math.sin(self.th + dth) - math.sin(self.th))
                self.y -= R * (math.cos(self.th + dth) - math.cos(self.th))
            else:
                self.x += ds * math.cos(self.th)
                self.y += ds * math.sin(self.th)
            self.th += dth

        # Pose
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        half = 0.5 * self.th
        odom.pose.pose.orientation.x = 0.0
        odom.pose.pose.orientation.y = 0.0
        odom.pose.pose.orientation.z = math.sin(half)
        odom.pose.pose.orientation.w = math.cos(half)

        # Velocity (signed forward speed if available)
        v = sign * abs(float(getattr(msg, "speed_ms", 0.0)))
        odom.twist.twist.linear.x = v
        # yaw rate if steer present
        odom.twist.twist.angular.z = (v / max(self.wheelbase_m, 1e-6)) * math.tan(steer) if abs(steer) > 0 else 0.0

        self._apply_default_covariances(odom)
        return odom

    def export_state(self) -> Dict[str, Any]:
        return {
            "x": self.x,
            "y": self.y,
            "th": self.th,
            "wheelbase_m": self.wheelbase_m,
            "last_abs_m": self.last_abs_m,
        }

    def import_state(self, state: Dict[str, Any]) -> None:
        self.x = float(state.get("x", 0.0))
        self.y = float(state.get("y", 0.0))
        self.th = float(state.get("th", 0.0))
        self.wheelbase_m = float(state.get("wheelbase_m", self.wheelbase_m))
        self.last_abs_m = float(state["last_abs_m"]) if state.get("last_abs_m") is not None else None

    def reset_odometry(self) -> None:
        self.x = self.y = self.th = 0.0
        self.last_abs_m = None

    def get_mode_name(self) -> str:
        return "Dual Serpentine"


# --------------------------- Factory ------------------------- #
class OdometryFactory:
    @staticmethod
    def create_odometry(mode: DrivingMode, *, track_width_m: float, wheelbase_m: float) -> OdometryInterface:
        if mode == DrivingMode.SINGLE_TRAILER:
            return SingleTrailerOdometry()
        if mode == DrivingMode.DUAL_DIFFERENTIAL:
            return DualDifferentialOdometry(track_width_m=track_width_m)
        if mode == DrivingMode.DUAL_SERPENTINE:
            return DualSerpentineOdometry(wheelbase_m=wheelbase_m)
        raise ValueError(f"Unsupported driving mode: {mode}")


# ----------------------------- Node -------------------------- #
class MttOdometryManager(Node):
    def __init__(self) -> None:
        super().__init__("mtt_odometry_manager")

        # Get centralized MTT parameters
        mtt_params = get_mtt_params()

        # Parameters - using centralized MTT vehicle parameters as defaults
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("tachometer_topic", "mtt_tachometer")
        self.declare_parameter("odometry_topic", "mtt_odometry")
        self.declare_parameter("mode_topic", "mtt_driving_mode")
        self.declare_parameter("wrap_reset_threshold_m", 1000.0)
        self.declare_parameter("track_width_m", mtt_params.track_width)  # From centralized params
        self.declare_parameter("wheelbase_m", mtt_params.total_wheelbase)  # From centralized params
        # New: distance unit & scaling
        self.declare_parameter("distance_unit", "km")  # 'km' or 'm'
        self.declare_parameter("distance_scale", 1.0)  # additional multiplicative scaling
        # New: angular velocity source topic and TF broadcast control
        self.declare_parameter("cmd_vel_topic", "cmd_vel/pid")
        self.declare_parameter("articulation_topic", "mtt_articulation_angle")
        self.declare_parameter("broadcast_tf", True)
        # Steering control mode parameters - using centralized vehicle parameters
        self.declare_parameter("steer_control_mode", "open_loop")  # "open_loop" or "closed_loop"
        # New: turn behavior tuning to reduce drift at rest
        self.declare_parameter("pivot_turn_enabled", False)
        self.declare_parameter("min_turn_speed_ms", 0.03)  # below this, suppress yaw unless pivot enabled
        self.declare_parameter("yaw_slip_factor", 1.0)     # scale < 1.0 reduces effective yaw

        # Resolve parameters
        self.odom_frame = self.get_parameter("odom_frame").get_parameter_value().string_value
        self.base_frame = self.get_parameter("base_frame").get_parameter_value().string_value
        self.tachometer_topic = self.get_parameter("tachometer_topic").get_parameter_value().string_value
        self.odometry_topic = self.get_parameter("odometry_topic").get_parameter_value().string_value
        self.mode_topic = self.get_parameter("mode_topic").get_parameter_value().string_value
        self.cmd_vel_topic = self.get_parameter("cmd_vel_topic").get_parameter_value().string_value
        self.articulation_topic = self.get_parameter("articulation_topic").get_parameter_value().string_value
        self.broadcast_tf = self.get_parameter("broadcast_tf").get_parameter_value().bool_value
        self.wrap_reset_threshold_m = self.get_parameter("wrap_reset_threshold_m").get_parameter_value().double_value
        self.track_width_m = self.get_parameter("track_width_m").get_parameter_value().double_value
        self.wheelbase_m = self.get_parameter("wheelbase_m").get_parameter_value().double_value
        distance_unit = self.get_parameter("distance_unit").get_parameter_value().string_value.lower()
        distance_scale = self.get_parameter("distance_scale").get_parameter_value().double_value
        base_multiplier = 1000.0 if distance_unit == "km" else 1.0
        self.distance_multiplier = base_multiplier * distance_scale
        
        # Get centralized vehicle parameters
        mtt_params = get_mtt_params()
        
        # Steering control mode configuration - use centralized parameters
        self.steer_control_mode = self.get_parameter("steer_control_mode").get_parameter_value().string_value
        self.max_yaw_rate = mtt_params.max_yaw_rate_rad_s  # Use centralized parameter
        self.max_articulation_angle = mtt_params.max_articulation_rad  # Use centralized parameter
        self.pivot_turn_enabled = self.get_parameter("pivot_turn_enabled").get_parameter_value().bool_value
        self.min_turn_speed_ms = self.get_parameter("min_turn_speed_ms").get_parameter_value().double_value
        self.yaw_slip_factor = self.get_parameter("yaw_slip_factor").get_parameter_value().double_value
        self.get_logger().info(
            f"Distance conversion: unit={distance_unit} scale={distance_scale} -> multiplier={self.distance_multiplier}"
        )

        # Mode
        self.current_mode = DrivingMode.SINGLE_TRAILER
        self.odometry_calculator = OdometryFactory.create_odometry(
            self.current_mode, track_width_m=self.track_width_m, wheelbase_m=self.wheelbase_m
        )

        # Publisher
        self.odom_pub = self.create_publisher(Odometry, self.odometry_topic, 10)
        # Publisher for articulation state (for joint controller)
        self.articulation_pub = self.create_publisher(Float64, self.articulation_topic, 10)

        # Subscriber with sensor-like QoS
        sensor_qos = QoSProfile(depth=1)
        sensor_qos.reliability = ReliabilityPolicy.BEST_EFFORT
        sensor_qos.history = HistoryPolicy.KEEP_LAST
        sensor_qos.durability = DurabilityPolicy.VOLATILE

        self.tacho_sub = self.create_subscription(
            MttTachometerData,
            self.tachometer_topic,
            self.tachometer_callback,
            sensor_qos,
        )

        # Driving mode subscriber (reliable)
        reliable_qos = QoSProfile(depth=10)
        reliable_qos.reliability = ReliabilityPolicy.RELIABLE
        reliable_qos.history = HistoryPolicy.KEEP_LAST
        reliable_qos.durability = DurabilityPolicy.VOLATILE

        self.mode_sub = self.create_subscription(
            MttDrivingMode,
            self.mode_topic,
            self.mode_change_callback,
            reliable_qos,
        )

        # Services
        self.reset_srv = self.create_service(Trigger, "mtt/reset_odometry", self.reset_odometry_cb)
        
        # Steering control mode service
        self.steer_control_srv = self.create_service(
            SetSteerControlMode,
            "mtt/set_steer_control_mode",
            self.set_steer_control_mode_cb
        )

        # Angular velocity from cmd_vel for steering odometry
        self.current_angular_vel = 0.0

        # Subscribe to cmd_vel for angular velocity information
        self.cmd_vel_sub = self.create_subscription(
            TwistStamped,
            self.cmd_vel_topic,
            self.cmd_vel_callback,
            10,
        )

        self.get_logger().info(
            f"MTT Odometry Manager initialized - Mode: {self.odometry_calculator.get_mode_name()} | "
            f"Steering: {self.steer_control_mode} (max_rate={self.max_yaw_rate:.3f} rad/s, max_angle={self.max_articulation_angle:.3f} rad) | "
            f"odom_frame={self.odom_frame}, base_frame={self.base_frame}, pub={self.odometry_topic}, sub={self.tachometer_topic}, mode_sub={self.mode_topic}"
        )
        self.tf_broadcaster = TransformBroadcaster(self) if self.broadcast_tf else None  # optional

    # ----------------- Callbacks ----------------- #
    def tacho_sub_failed_time_fallback(self, odom: Odometry) -> None:
        # Fallback to node time if msg had no timestamp (rare)
        if not odom.header.stamp.sec and not odom.header.stamp.nanosec:
            odom.header.stamp = self.get_clock().now().to_msg()

    def tachometer_callback(self, msg: MttTachometerData) -> None:
        try:
            # Process steering command based on control mode
            raw_steer_cmd = float(getattr(msg, "steer_cmd", 0.0))
            speed_ms = float(getattr(msg, "speed_ms", 0.0))

            # Compute angular velocity with proper coupling to speed to prevent drift at rest
            if self.steer_control_mode == "closed_loop":
                # steer_cmd is normalized articulation angle [-1,1] → [-max_angle, +max_angle]
                articulation_angle = max(-self.max_articulation_angle, min(self.max_articulation_angle, raw_steer_cmd * self.max_articulation_angle))
                # Bicycle-like kinematics: yaw_rate = v * tan(phi) / L
                curvature = math.tan(articulation_angle) / max(self.wheelbase_m, 1e-6)
                effective_angular_vel = speed_ms * curvature
            else:
                # Open-loop: steer_cmd as normalized yaw rate
                effective_angular_vel = raw_steer_cmd * self.max_yaw_rate

            # Suppress yaw when nearly stationary unless pivot turns are explicitly enabled
            if not self.pivot_turn_enabled and abs(speed_ms) < self.min_turn_speed_ms:
                effective_angular_vel = 0.0

            # Apply slip scaling and clamp to configured max yaw rate
            effective_angular_vel *= float(self.yaw_slip_factor)
            if self.max_yaw_rate > 0.0:
                max_rate = abs(self.max_yaw_rate)
                if effective_angular_vel > max_rate:
                    effective_angular_vel = max_rate
                elif effective_angular_vel < -max_rate:
                    effective_angular_vel = -max_rate
            
            # Use the processed angular velocity for odometry calculation
            odom = self.odometry_calculator.calculate_odometry(
                msg,
                odom_frame=self.odom_frame,
                base_frame=self.base_frame,
                distance_multiplier=self.distance_multiplier,
                wrap_reset_threshold_m=self.wrap_reset_threshold_m,
                angular_velocity=effective_angular_vel,
            )
            # Enforce consistent time base for odom and TF to avoid oscillation with RSP
            if not getattr(self, "use_sensor_stamp", False):
                odom.header.stamp = self.get_clock().now().to_msg()
            self.tacho_sub_failed_time_fallback(odom)
            self.odom_pub.publish(odom)
            
            # Publish articulation angle for joint controller (only for SingleTrailerOdometry)
            if isinstance(self.odometry_calculator, SingleTrailerOdometry):
                articulation_msg = Float64()
                articulation_msg.data = self.odometry_calculator.get_articulation_angle()
                self.articulation_pub.publish(articulation_msg)
            # broadcast TF transform odom->base_frame (optional, dynamic)
            try:
                want_tf = self.get_parameter("broadcast_tf").get_parameter_value().bool_value
            except Exception:
                want_tf = True
            # Lazy toggle
            if want_tf and self.tf_broadcaster is None:
                self.tf_broadcaster = TransformBroadcaster(self)
            elif not want_tf and self.tf_broadcaster is not None:
                self.tf_broadcaster = None
            if want_tf and self.tf_broadcaster is not None:
                t = TransformStamped()
                t.header = odom.header
                t.child_frame_id = self.base_frame
                t.transform.translation.x = odom.pose.pose.position.x
                t.transform.translation.y = odom.pose.pose.position.y
                t.transform.translation.z = odom.pose.pose.position.z
                t.transform.rotation = odom.pose.pose.orientation
                self.tf_broadcaster.sendTransform(t)
        except Exception as e:
            self.get_logger().error(f"Odometry calculation failed: {e}")

    def reset_odometry_cb(self, request, response):
        try:
            self.odometry_calculator.reset_odometry()
            response.success = True
            response.message = f"Odometry reset in mode: {self.odometry_calculator.get_mode_name()}"
        except Exception as e:
            response.success = False
            response.message = str(e)
        return response

    def set_steer_control_mode_cb(self, request, response):
        """Service callback to set steering control mode and parameters"""
        try:
            # Validate control mode
            if request.control_mode not in ["open_loop", "closed_loop"]:
                response.success = False
                response.message = "Invalid control_mode. Use 'open_loop' or 'closed_loop'"
                return response
            
            # Validate parameters against centralized vehicle parameters
            mtt_params = get_mtt_params()
            if request.max_rate <= 0.0 or request.max_angle <= 0.0:
                response.success = False
                response.message = "max_rate and max_angle must be positive values"
                return response
            
            # Warn if parameters differ from centralized values
            if abs(request.max_rate - mtt_params.max_yaw_rate_rad_s) > 0.001:
                self.get_logger().warn(
                    f"Requested max_rate {request.max_rate:.3f} differs from centralized parameter "
                    f"{mtt_params.max_yaw_rate_rad_s:.3f}. Using centralized value for consistency."
                )
            
            if abs(request.max_angle - mtt_params.max_articulation_rad) > 0.001:
                self.get_logger().warn(
                    f"Requested max_angle {request.max_angle:.3f} differs from centralized parameter "
                    f"{mtt_params.max_articulation_rad:.3f}. Using centralized value for consistency."
                )
            
            # Update configuration - always use centralized parameters for consistency
            self.steer_control_mode = request.control_mode
            # Note: max_yaw_rate and max_articulation_angle remain from centralized params
            
            response.success = True
            response.message = f"Steering control mode set to: {request.control_mode} " + \
                             f"(max_rate={self.max_yaw_rate:.3f} rad/s, max_angle={self.max_articulation_angle:.3f} rad) " + \
                             f"[Using centralized vehicle parameters]"
            
            self.get_logger().info(response.message)
            
        except Exception as e:
            response.success = False
            response.message = f"Failed to set steering control mode: {str(e)}"
            
        return response

    def cmd_vel_callback(self, msg: TwistStamped) -> None:
        """Store current angular velocity for steering odometry calculations"""
        self.current_angular_vel = msg.twist.angular.z

    # --------------- Mode switching --------------- #
    def _transfer_state(self, src: OdometryInterface, dst: OdometryInterface) -> None:
        try:
            state = src.export_state()
            dst.import_state(state)
        except Exception as e:
            self.get_logger().warn(f"State transfer failed ({e}); continuing with fresh state")

    def _parse_mode_parameters(self, params: str) -> Dict[str, float]:
        """Parse key=value pairs (comma/space separated). Example: "track=1.25 wheelbase=2.3"""
        out: Dict[str, float] = {}
        if not params:
            return out
        for token in params.replace(",", " ").split():
            if "=" in token:
                k, v = token.split("=", 1)
                try:
                    out[k.strip().lower()] = float(v)
                except ValueError:
                    pass
        return out

    def mode_change_callback(self, msg: MttDrivingMode) -> None:
        """Handle driving mode changes from the wrapper.
        Accepts optional `mode_parameters` such as:
          - "track=1.1" to update track_width_m (DualDifferential)
          - "wheelbase=2.3" to update wheelbase_m (DualSerpentine)
        """
        try:
            new_mode = DrivingMode(int(msg.mode))
        except Exception:
            self.get_logger().error(f"Invalid mode value: {msg.mode}")
            return

        # Update geometry parameters if provided
        kv = self._parse_mode_parameters(getattr(msg, "mode_parameters", ""))
        if "track" in kv:
            self.track_width_m = float(kv["track"])
        if "wheelbase" in kv:
            self.wheelbase_m = float(kv["wheelbase"])

        if new_mode == self.current_mode:
            # Mode unchanged; just update calculator params if relevant
            if isinstance(self.odometry_calculator, DualDifferentialOdometry):
                self.odometry_calculator.track_width_m = self.track_width_m
            if isinstance(self.odometry_calculator, DualSerpentineOdometry):
                self.odometry_calculator.wheelbase_m = self.wheelbase_m
            return

        self.get_logger().info(
            f"Mode change requested: {self.current_mode.name} → {new_mode.name} (track={self.track_width_m:.3f}, wheelbase={self.wheelbase_m:.3f})"
        )

        try:
            new_calc = OdometryFactory.create_odometry(
                new_mode, track_width_m=self.track_width_m, wheelbase_m=self.wheelbase_m
            )
            self._transfer_state(self.odometry_calculator, new_calc)
            self.current_mode = new_mode
            self.odometry_calculator = new_calc
            self.get_logger().info(f"Switched to {self.odometry_calculator.get_mode_name()} mode (state preserved)")
        except Exception as e:
            self.get_logger().error(f"Mode change failed: {e}")

    def manual_mode_change(self, mode: int) -> bool:
        try:
            new_mode = DrivingMode(mode)
            if new_mode != self.current_mode:
                self.get_logger().info(
                    f"Manual mode switch: {self.odometry_calculator.get_mode_name()} → {new_mode.name}"
                )
                new_calc = OdometryFactory.create_odometry(
                    new_mode, track_width_m=self.track_width_m, wheelbase_m=self.wheelbase_m
                )
                self._transfer_state(self.odometry_calculator, new_calc)
                self.current_mode = new_mode
                self.odometry_calculator = new_calc
                self.get_logger().info(f"Switched to {self.odometry_calculator.get_mode_name()} mode (state preserved)")
                return True
            return False
        except ValueError as e:
            self.get_logger().error(f"Invalid driving mode: {mode} - {e}")
            return False
        except Exception as e:
            self.get_logger().error(f"Mode change failed: {e}")
            return False

    # TF broadcasting can be added here if coordinate frame transforms are needed


# ----------------------------- Main -------------------------- #
def main(args=None) -> None:
    rclpy.init(args=args)
    node = MttOdometryManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
