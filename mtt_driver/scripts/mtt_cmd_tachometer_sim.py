#!/usr/bin/env python3
"""
MTT Command-Based Tachometer Simulator

Listens to CAN frames on vcan0 (from driver) and generates realistic 2FF tachometer responses.
This provides much better testing than fake tachometer since it responds to actual commands.

Usage:
    python3 mtt_cmd_tachometer_sim.py --can-interface vcan0
"""

import can
import struct
import time
import math
import argparse
import signal
import sys
from dataclasses import dataclass
from typing import Optional


@dataclass
class VehicleState:
    """Simple vehicle dynamics state for tachometer simulation."""
    position_m: float = 0.0
    velocity_ms: float = 0.0
    cumulative_ticks: int = 0
    _tick_accumulator: float = 0.0  # internal high-res accumulator
    last_update: float = 0.0
    
    # Vehicle parameters (matching real MTT-154)
    max_velocity: float = 2.0  # m/s
    acceleration: float = 1.5  # m/s^2
    deceleration: float = 2.0  # m/s^2
    
    # Encoder simulation (following mtt_encoder_methodology.md)
    # Theoretical calculation: final_ratio = 324.0, track_length = 3.93 m  
    # -> ticks_per_meter = final_ratio / track_length = 324.0 / 3.93 ≈ 82.44
    # Both instant and cumulative use the same theoretical calculation
    ticks_per_meter_cumulative: float = 324.0 / 3.93  # ≈ 82.44
    ticks_per_meter_instant: float = 324.0 / 3.93     # ≈ 82.44


class MTTCommandTachometerSim:
    """Simulates tachometer responses based on actual CAN commands."""
    
    def __init__(self, can_interface='vcan0'):
        self.can_interface = can_interface
        self.vehicle = VehicleState()
        self.running = True
        
        # CAN setup
        try:
            self.bus = can.interface.Bus(can_interface, bustype='socketcan')
            print(f"Connected to {can_interface}")
        except Exception as e:
            print(f"Failed to connect to {can_interface}: {e}")
            sys.exit(1)
            
        # Current commands from driver
        self.target_throttle = 0.0  # 0.0 to 1.0
        self.current_direction = 1  # 1=forward, -1=reverse
        self.last_command_time = time.time()
        
        # 2FF publishing
        self.tachometer_period = 0.02  # 50Hz like real hardware
        self.last_tachometer_time = 0.0
        
    def decode_driver_frame(self, data: bytes) -> bool:
        """Decode driver CAN frame (0x001) to extract throttle and direction."""
        if len(data) != 8:
            return False
            
        try:
            # MTT driver frame format (from mtt_driver.py):
            # [0] = vehicle_type
            # [1] = switches (direction in bit 5, security in bit 3)  
            # [2] = throttle (0-230)
            # [3] = winch
            # [4] = brake 
            # [5] = steer
            # [6] = steering_mode
            
            throttle_raw = data[2]  # 0-230 range
            switches = data[1]
            
            # Extract direction from bit 5 of switches
            direction_forward = bool(switches & 0b00100000)
            self.current_direction = 1 if direction_forward else -1
            
            # Convert throttle to 0.0-1.0 range
            self.target_throttle = min(1.0, throttle_raw / 230.0)
            
            self.last_command_time = time.time()
            return True
            
        except Exception as e:
            print(f"Error decoding frame: {e}")
            return False
    
    def update_vehicle_dynamics(self, dt: float):
        """Simple vehicle dynamics integration."""
        if dt <= 0:
            return
            
        # Target velocity based on throttle and direction
        target_velocity = self.target_throttle * self.vehicle.max_velocity * self.current_direction
        
        # Simple acceleration model
        velocity_error = target_velocity - self.vehicle.velocity_ms
        if abs(velocity_error) > 0.01:  # Deadband
            if velocity_error > 0:
                accel = min(self.vehicle.acceleration * dt, velocity_error)
            else:
                accel = max(-self.vehicle.deceleration * dt, velocity_error)
            self.vehicle.velocity_ms += accel
        else:
            # Snap to target to avoid lingering small residual velocities
            self.vehicle.velocity_ms = target_velocity
        
        # Update position
        self.vehicle.position_m += self.vehicle.velocity_ms * dt

        # Incremental tick accumulation based on path length (always non-decreasing)
        incr_ticks = abs(self.vehicle.velocity_ms) * dt * self.vehicle.ticks_per_meter_cumulative
        self.vehicle._tick_accumulator += incr_ticks
        # Emit integer ticks, keep fractional remainder
        whole_ticks = int(self.vehicle._tick_accumulator)
        if whole_ticks > 0:
            self.vehicle.cumulative_ticks += whole_ticks
            self.vehicle._tick_accumulator -= whole_ticks
        
    def create_tachometer_frame(self) -> can.Message:
        """Create 2FF tachometer frame matching real hardware format."""
        
        # Simulate temperature sensors (25-35°C range)
        temp_a = int(30 + 5 * math.sin(time.time() / 20))  # Slow variation
        temp_b = int(32 + 3 * math.cos(time.time() / 15))
        
        # Instantaneous RPS using theoretical calculation (final_ratio = 324.0)
        velocity_abs = abs(self.vehicle.velocity_ms)
        rps_raw = int(round(velocity_abs * self.vehicle.ticks_per_meter_instant))
        
        # Pack frame: temp_a(int8), temp_b(int8), rps(uint16_be), cumulative(uint32_be)
        frame_data = struct.pack('>bbHI', 
                               temp_a, temp_b, 
                               rps_raw, 
                               self.vehicle.cumulative_ticks)
        
        return can.Message(arbitration_id=0x2FF, data=frame_data, is_extended_id=False)
    
    def run(self):
        """Main simulation loop."""
        print("MTT Command Tachometer Simulator started")
        print("Listening for driver commands on 0x001, publishing tachometer on 0x2FF")
        print("Press Ctrl+C to stop")
        
        try:
            while self.running:
                current_time = time.time()
                
                # Listen for driver commands (non-blocking)
                try:
                    message = self.bus.recv(timeout=0.001)
                    if message and message.arbitration_id == 0x001:
                        if self.decode_driver_frame(message.data):
                            print(f"Command: throttle={self.target_throttle:.2f}, "
                                  f"direction={'FWD' if self.current_direction > 0 else 'REV'}, "
                                  f"vel={self.vehicle.velocity_ms:.2f} m/s")
                except can.CanOperationError:
                    pass  # No message available
                
                # Update vehicle dynamics  
                if self.vehicle.last_update > 0:
                    dt = current_time - self.vehicle.last_update
                    self.update_vehicle_dynamics(dt)
                self.vehicle.last_update = current_time
                
                # Publish tachometer at 50Hz
                if current_time - self.last_tachometer_time >= self.tachometer_period:
                    tach_frame = self.create_tachometer_frame()
                    try:
                        self.bus.send(tach_frame)
                    except can.CanOperationError as e:
                        print(f"Failed to send tachometer frame: {e}")
                    
                    self.last_tachometer_time = current_time
                    
                    # Debug output every 2 seconds
                    if int(current_time) % 2 == 0 and current_time - int(current_time) < 0.1:
                        print(f"State: pos={self.vehicle.position_m:.2f}m, "
                              f"vel={self.vehicle.velocity_ms:.2f}m/s, "
                              f"ticks={self.vehicle.cumulative_ticks}")
                
                # Small sleep to prevent CPU spinning
                time.sleep(0.001)
                
        except KeyboardInterrupt:
            print("\nShutting down simulator...")
        finally:
            self.cleanup()
    
    def cleanup(self):
        """Clean shutdown."""
        self.running = False
        if hasattr(self, 'bus'):
            self.bus.shutdown()
        print("Simulator stopped")
    
    def signal_handler(self, signum, frame):
        """Handle Ctrl+C gracefully."""
        self.running = False


def main():
    parser = argparse.ArgumentParser(description='MTT Command-Based Tachometer Simulator')
    parser.add_argument('--can-interface', default='vcan0', 
                       help='CAN interface (default: vcan0)')
    
    args = parser.parse_args()
    
    simulator = MTTCommandTachometerSim(args.can_interface)
    
    # Handle Ctrl+C gracefully
    signal.signal(signal.SIGINT, simulator.signal_handler)
    signal.signal(signal.SIGTERM, simulator.signal_handler)
    
    simulator.run()


if __name__ == '__main__':
    main()
