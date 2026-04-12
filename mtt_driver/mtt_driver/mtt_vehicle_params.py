#!/usr/bin/env python3
"""
MTT-154 Vehicle Parameters - Centralized Configuration

All physical vehicle parameters measured from real MTT-154 vehicle testing.
This is the single source of truth for vehicle geometry, drivetrain, and steering parameters.
"""

import math
from dataclasses import asdict
from dataclasses import dataclass
from typing import Optional


@dataclass
class MTTVehicleParams:
    """Centralized MTT-154 vehicle parameters from real vehicle measurements."""
    
    # =============================================================================
    # DRIVETRAIN PARAMETERS (from vehicle testing)
    # =============================================================================
    
    # Vehicle geometry - measured distances
    l_f: float = 0.9    # Tractor length: center of track to hitch pin (m) #TODO: add a point in the urdf
    l_r: float = 1.5    # Trailer length: hitch pin to trailer axle (m) #TODO: add a point in the urdf
    
    # Track/sprocket system - measured performance
    r_sprocket_eff: float = 0.0202  # Effective sprocket radius: N revs vs ground distance (m)
                                    # Derived from 12.7mm/2π
    gear_ratio_raw: float = 12.36   # Gear ratio: main sprocket to track (unitless)
    ticks_per_rev: int = 10         # Encoder ticks per motor revolution
    
    # Individual gear components (per mtt_encoder_methodology.md)
    mtt_gear1: int = 16             # First gear stage
    mtt_gear2: int = 36             # Second gear stage  
    mtt_gear3: int = 15             # Third gear stage
    mtt_gear4: int = 32             # Fourth gear stage
    mtt_gear_drive: int = 8         # Drive gear
    mtt_gear_track: int = 54        # Track gear
    
    # CRITICAL DISTINCTION (per mtt_encoder_methodology.md):
    mtt_sprocket_teeth: int = 5     # Sprocket teeth for mechanical gear calculations  
    mtt_tacho_teeth: int = 10       # Tachometer disc slots (= ticks_per_rev)
    mtt_encoder_teeth: int = 10     # Legacy name (same as ticks_per_rev)
    
    # Track dimensions (computed from measurements)
    mtt_track_length_cm: float = 393.0      # Track length in cm
    
    # Computed drivetrain parameters
    @property 
    def wheel_radius(self) -> float:
        """Effective wheel radius for joint controller."""
        return self.r_sprocket_eff
        
    @property
    def mtt_track_length_km(self) -> float:
        """Track length in km (firmware convention)."""
        return self.mtt_track_length_cm / 100000.0
        
    @property
    def mtt_track_length_m(self) -> float:
        """Track length in meters."""
        return self.mtt_track_length_cm / 100.0
    
    @property
    def total_wheelbase(self) -> float:
        """Total vehicle wheelbase (front track to rear axle)."""
        return self.l_f + self.l_r
        
    @property
    def mechanical_gear_ratio(self) -> float:
        """Theoretical mechanical gear reduction (per mtt_encoder_methodology.md)."""
        return (self.mtt_gear2 / self.mtt_gear1) * (self.mtt_gear4 / self.mtt_gear3) * (self.mtt_gear_track / self.mtt_gear_drive)
        
    @property
    def encoder_final_ratio(self) -> float:
        """Final encoder ratio for speed calculations (mechanical × ticks per rev)."""
        return self.mechanical_gear_ratio * self.mtt_tacho_teeth
    
    @property 
    def legacy_encoder_final_ratio(self) -> float:
        """Legacy encoder ratio calculation (INCORRECT - for comparison only)."""
        # This is the old incorrect formula that mixed sprocket and tachometer teeth
        ratio1 = (self.mtt_gear2 / self.mtt_gear1) * self.mtt_encoder_teeth
        ratio2 = (self.mtt_gear4 / self.mtt_gear3) * ratio1
        return ((self.mtt_gear_track / self.mtt_gear_drive) * ratio2) * 2
    
    # =============================================================================
    # STEERING PARAMETERS (from vehicle testing)
    # =============================================================================
    
    # Physical steering limits - measured sweep to stops
    max_articulation_deg: float = 50.0  # Maximum steering angle (degrees) ±50°
    
    # CAN bus steering interface - measured values
    steering_center_byte: int = 127     # Center position in CAN frame
    steering_halfspan_byte: int = 100   # Byte delta from center to max
                                        # Note: Measured in open-loop, may differ in closed-loop
                                        # Keep current values pending closed-loop testing
    
    # Steering precision
    steering_deadband_deg: float = 0.5  # Steering deadband (degrees)
    steering_invert: bool = False       # No inversion needed
    
    # CAN steering interface - computed parameters
    steering_max_byte: int = 255        # Maximum steering byte value
    steering_deadband_normalized: float = 0.5 / 50.0  # Deadband as fraction of max angle
    
    # Computed steering parameters
    @property
    def max_articulation_rad(self) -> float:
        """Maximum articulation angle in radians."""
        return math.radians(self.max_articulation_deg)
    
    @property
    def steering_deadband_rad(self) -> float:
        """Steering deadband in radians."""
        return math.radians(self.steering_deadband_deg)
        
    @property
    def steering_range_bytes(self) -> int:
        """Total steering range in CAN bytes."""
        return 2 * self.steering_halfspan_byte
    
    # =============================================================================
    # VEHICLE DYNAMICS PARAMETERS (for ArticulatedVehicleDynamics)
    # =============================================================================
    
    # Track dynamics - tuned for heavy tracked vehicle behavior
    track_width: float = 1.2            # Track width (m) - estimated
    track_slip_coeff: float = 0.05      # Low slip coefficient (tracks have excellent grip) 
    track_grip_coeff: float = 0.95      # High forward grip coefficient
    carving_factor: float = 0.1         # Minimal track digging effect in turns
    
    # Articulation response - tuned for realistic behavior
    articulation_response: float = 0.8  # Good response rate for hydraulic system
    max_yaw_rate_rad_s: float = math.radians(45) / 6.0  # Max yaw rate: 45° in 6s (realistic for heavy vehicle)
    
    # Speed parameters - measured limits for MTT-154
    max_speed_ms: float = 2.0           # Maximum vehicle speed (m/s) - conservative for safety
    
    # Speed-dependent factors - conservative for safety
    slip_speed_factor: float = 0.02     # Minimal slip increase with speed
    min_speed_for_steering: float = 0.1  # Minimum speed for effective steering (m/s)


# Global singleton instance - single source of truth
MTT_PARAMS = MTTVehicleParams()


def get_mtt_params() -> MTTVehicleParams:
    """Get the global MTT vehicle parameters."""
    return MTT_PARAMS


def update_mtt_params(**kwargs) -> None:
    """Update specific MTT vehicle parameters."""
    global MTT_PARAMS
    for key, value in kwargs.items():
        if hasattr(MTT_PARAMS, key):
            setattr(MTT_PARAMS, key, value)
        else:
            raise ValueError(f"Unknown parameter: {key}")


# =============================================================================
# PARAMETER VALIDATION
# =============================================================================

def validate_params(params: Optional[MTTVehicleParams] = None) -> bool:
    """Validate vehicle parameters for consistency."""
    if params is None:
        params = MTT_PARAMS
        
    # Basic sanity checks
    assert params.l_f > 0, "Front wheelbase must be positive"
    assert params.l_r > 0, "Rear wheelbase must be positive" 
    assert params.r_sprocket_eff > 0, "Sprocket radius must be positive"
    assert params.gear_ratio_raw > 1, "Gear ratio must be greater than 1"
    assert params.ticks_per_rev > 0, "Ticks per revolution must be positive"
    assert 0 < params.max_articulation_deg <= 90, "Max articulation must be 0-90 degrees"
    assert params.steering_center_byte == 127, "Steering center must be 127"
    assert params.steering_halfspan_byte > 0, "Steering halfspan must be positive"
    
    return True


# Validate on import
validate_params()


def main(args=None):
    """Print the current MTT vehicle parameters."""
    del args
    for key, value in asdict(MTT_PARAMS).items():
        print(f"{key}: {value}")
