#!/usr/bin/env python3
"""
MTT Articulated Vehicle Dynamics Model
Implements realistic dynamics for tracked articulated vehicle with central steering joint.
"""

import math
import numpy as np
from typing import Tuple, Optional
from dataclasses import dataclass
from typing import Optional
from .mtt_vehicle_params import MTTVehicleParams, get_mtt_params


@dataclass 
class ArticulatedVehicleParams:
    """Physical parameters of the articulated vehicle - uses real measured MTT-154 values."""
    
    def __init__(self, mtt_params: Optional[MTTVehicleParams] = None):
        """Initialize with real MTT-154 measured parameters."""
        if mtt_params is None:
            mtt_params = get_mtt_params()
            
        # Vehicle geometry - from real vehicle measurements
        self.front_wheelbase: float = mtt_params.l_f        # Distance from track center to hitch pin (m)
        self.rear_wheelbase: float = mtt_params.l_r         # Distance from hitch pin to trailer axle (m)  
        self.track_width: float = mtt_params.track_width    # Track width (m)
        
        # Track dynamics - tunable parameters
        self.track_slip_coeff: float = mtt_params.track_slip_coeff      # Lateral slip coefficient
        self.track_grip_coeff: float = mtt_params.track_grip_coeff      # Forward grip coefficient  
        self.carving_factor: float = mtt_params.carving_factor          # Track digging effect in turns
        
        # Articulation limits - from real vehicle measurements (±50°)
        self.max_articulation_angle: float = mtt_params.max_articulation_rad  # Maximum joint angle (rad)
        self.articulation_response: float = mtt_params.articulation_response  # Joint response rate
        self.max_yaw_rate: float = mtt_params.max_yaw_rate_rad_s              # Maximum yaw rate (rad/s)
        
        # Speed limits - from centralized MTT parameters
        self.max_speed_ms: float = mtt_params.max_speed_ms                    # Maximum vehicle speed (m/s)
        
        # Speed-dependent factors - tunable parameters
        self.slip_speed_factor: float = mtt_params.slip_speed_factor         # Slip increases with speed
        self.min_speed_for_steering: float = mtt_params.min_speed_for_steering  # Minimum speed for effective steering (m/s)


class ArticulatedVehicleDynamics:
    """
    Articulated vehicle dynamics model for MTT.
    
    This model simulates:
    1. Front tractor propulsion
    2. Articulated joint steering
    3. Track slip and carving
    4. Trailer following dynamics
    """
    
    def __init__(self, params: Optional[ArticulatedVehicleParams] = None):
        self.params = params or ArticulatedVehicleParams()
        
        # Vehicle state
        self.x = 0.0              # Position X (m)
        self.y = 0.0              # Position Y (m)
        self.heading = 0.0        # Vehicle heading (rad)
        self.articulation_angle = 0.0  # Joint angle (rad)
        
        # Velocities
        self.linear_velocity = 0.0     # Forward velocity (m/s)
        self.angular_velocity = 0.0    # Heading change rate (rad/s)
        
        # Track forces and slip
        self.front_slip_angle = 0.0    # Front track slip angle
        self.rear_slip_angle = 0.0     # Rear track slip angle
        
    def update(self, 
               throttle_input: float,        # Forward/reverse command (-1 to 1)
               steering_input: float,        # Steering command (-1 to 1)
               dt: float,                    # Time step (s)
               terrain_grip: float = 1.0     # Terrain grip factor (0-1)
               ) -> Tuple[float, float, float]:
        """
        Update vehicle dynamics for one time step.
        
        Returns:
            (x, y, heading) - New vehicle position and orientation
        """
        
        # 1. Calculate target articulation angle from steering input
        target_articulation = steering_input * self.params.max_articulation_angle
        
        # 2. Update articulation angle with response dynamics
        articulation_error = target_articulation - self.articulation_angle
        self.articulation_angle += articulation_error * self.params.articulation_response * dt
        
        # Limit articulation angle
        self.articulation_angle = max(-self.params.max_articulation_angle,
                                     min(self.params.max_articulation_angle,
                                         self.articulation_angle))
        
        # 3. Calculate forward velocity with proper damping for tracked vehicle
        max_velocity = self.params.max_speed_ms  # From centralized MTT parameters
        target_velocity = throttle_input * max_velocity
        
        # Heavy tracked vehicle behavior: strong damping when no throttle
        if abs(throttle_input) < 0.01:  # No throttle input
            # Strong deceleration due to track friction and vehicle weight
            decel_rate = 3.0  # m/s^2 - tracks provide strong braking
            if abs(self.linear_velocity) > 0.01:
                decel_sign = -1.0 if self.linear_velocity > 0 else 1.0
                decel = decel_sign * decel_rate * dt
                if abs(decel) >= abs(self.linear_velocity):
                    self.linear_velocity = 0.0  # Full stop
                else:
                    self.linear_velocity += decel
            else:
                self.linear_velocity = 0.0
        else:
            # Normal acceleration/deceleration with throttle input
            accel_factor = terrain_grip * self.params.track_grip_coeff
            velocity_error = target_velocity - self.linear_velocity
            acceleration = velocity_error * accel_factor * 2.0  # Faster response
            self.linear_velocity += acceleration * dt
        
        # 4. Calculate vehicle kinematics - tracked vehicles need forward motion to steer
        if abs(self.linear_velocity) > self.params.min_speed_for_steering:
            # Effective steering based on articulation and speed
            effective_steering_angle = self.articulation_angle
            
            # Reduced slip for heavy tracked vehicle (much more grip than wheeled)
            speed_factor = min(1.0, abs(self.linear_velocity) / 2.0)  # Normalize to 2 m/s
            slip_factor = self.params.track_slip_coeff * 0.3  # Much less slip for tracks
            
            # Calculate angular velocity using articulated vehicle model
            wheelbase_effective = self.params.front_wheelbase + self.params.rear_wheelbase
            
            # The articulated joint creates a virtual steering angle
            virtual_steering_angle = math.atan(
                wheelbase_effective * math.sin(effective_steering_angle) /
                (self.params.front_wheelbase + self.params.rear_wheelbase * math.cos(effective_steering_angle))
            )
            
            # Minimal carving effect for realistic behavior
            carving_effect = self.params.carving_factor * 0.2 * abs(virtual_steering_angle)  # Reduced
            enhanced_steering = virtual_steering_angle * (1.0 + carving_effect)
            
            # Calculate angular velocity - proportional to forward speed
            self.angular_velocity = (self.linear_velocity * math.tan(enhanced_steering) / 
                                   wheelbase_effective) * (1.0 - slip_factor)
            
            # Calculate slip angles for front and rear tracks
            if abs(self.linear_velocity) > 0.01:
                self.front_slip_angle = effective_steering_angle - self.angular_velocity * self.params.front_wheelbase / self.linear_velocity
                self.rear_slip_angle = -self.angular_velocity * self.params.rear_wheelbase / self.linear_velocity
            else:
                self.front_slip_angle = 0.0
                self.rear_slip_angle = 0.0

        else:
            # CRITICAL: At low/zero speeds, tracked vehicles cannot steer effectively
            # Angular velocity decays rapidly when stopped
            decay_rate = 10.0  # rad/s^2 - fast angular decay for stopped tracked vehicle
            if abs(self.angular_velocity) > 0.01:
                decay_sign = -1.0 if self.angular_velocity > 0 else 1.0
                decay = decay_sign * decay_rate * dt
                if abs(decay) >= abs(self.angular_velocity):
                    self.angular_velocity = 0.0  # Full stop
                else:
                    self.angular_velocity += decay
            else:
                self.angular_velocity = 0.0
            
            self.front_slip_angle = 0.0
            self.rear_slip_angle = 0.0
        
        # 5. Update position and heading
        self.heading += self.angular_velocity * dt
        
        # Normalize heading
        self.heading = math.atan2(math.sin(self.heading), math.cos(self.heading))
        
        # Update position
        self.x += self.linear_velocity * math.cos(self.heading) * dt
        self.y += self.linear_velocity * math.sin(self.heading) * dt
        
        return self.x, self.y, self.heading
    
    def get_state(self) -> dict:
        """Get current vehicle state."""
        return {
            'x': self.x,
            'y': self.y,
            'heading': self.heading,
            'linear_velocity': self.linear_velocity,
            'angular_velocity': self.angular_velocity,
            'articulation_angle': self.articulation_angle,
            'front_slip_angle': self.front_slip_angle,
            'rear_slip_angle': self.rear_slip_angle
        }
    
    def set_state(self, x: float, y: float, heading: float):
        """Set vehicle position and heading."""
        self.x = x
        self.y = y
        self.heading = heading
    
    def reset(self):
        """Reset vehicle to origin."""
        self.x = 0.0
        self.y = 0.0
        self.heading = 0.0
        self.articulation_angle = 0.0
        self.linear_velocity = 0.0
        self.angular_velocity = 0.0
        self.front_slip_angle = 0.0
        self.rear_slip_angle = 0.0


def calculate_instantaneous_center_of_rotation(vehicle_state: dict, params: ArticulatedVehicleParams) -> Tuple[float, float]:
    """Calculate instantaneous center of rotation for articulated vehicle."""
    if abs(vehicle_state['angular_velocity']) < 1e-6:
        return float('inf'), float('inf')
    
    radius = vehicle_state['linear_velocity'] / vehicle_state['angular_velocity']
    
    icr_x = -radius * math.sin(vehicle_state['heading'])
    icr_y = radius * math.cos(vehicle_state['heading'])
    
    global_icr_x = vehicle_state['x'] + icr_x
    global_icr_y = vehicle_state['y'] + icr_y
    
    return global_icr_x, global_icr_y
