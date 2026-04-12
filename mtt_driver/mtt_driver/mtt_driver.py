#!/usr/bin/env python3
"""MTT-154 CAN driver: low-level CAN control + tachometer parsing (no ROS deps)."""

import can
from enum import Enum
import logging
import time
import socket
import fcntl
import struct
import threading
from dataclasses import dataclass, replace
from .mtt_vehicle_params import get_mtt_params


log = logging.getLogger("MTTCanDriver")


def setup_logging(level=logging.INFO):
    """Configure system logging for the MTT driver."""
    log.setLevel(level)
    if not log.handlers:
        handler = logging.StreamHandler()
        handler.setFormatter(logging.Formatter("%(asctime)s - %(name)s - %(levelname)s - %(message)s"))
        log.addHandler(handler)


# Default logging setup - can be overridden by calling setup_logging()
setup_logging()

# Get real MTT-154 measured parameters - single source of truth
_mtt_params = get_mtt_params()

# MTT-154 Encoder and Odometry Constants - from real vehicle measurements
MTT_GEAR1 = _mtt_params.mtt_gear1
MTT_GEAR2 = _mtt_params.mtt_gear2  
MTT_GEAR3 = _mtt_params.mtt_gear3
MTT_GEAR4 = _mtt_params.mtt_gear4
MTT_GEAR_DRIVE = _mtt_params.mtt_gear_drive
MTT_GEAR_TRACK = _mtt_params.mtt_gear_track
MTT_ENCODER_TEETH = _mtt_params.mtt_encoder_teeth  # Legacy (10 ticks per motor rev)
MTT_SPROCKET_TEETH = _mtt_params.mtt_sprocket_teeth  # Mechanical: 5 teeth  
MTT_TACHO_TEETH = _mtt_params.mtt_tacho_teeth       # Tachometer: 10 slots
MTT_TRACK_LENGTH_CM = _mtt_params.mtt_track_length_cm  # Measured track length
MTT_TRACK_LENGTH_KM = _mtt_params.mtt_track_length_km  # Firmware convention
MTT_TRACK_LENGTH_M = _mtt_params.mtt_track_length_m

CAN_MAIN_TELEMETRY = 0x2FF

MTT_SWITCHES_VEHICLE_TYPE = 0
MTT_SWITCHES_GLOBAL = 1
MTT_ANALOG_THROTTLE = 2
MTT_ANALOG_WINCH = 3
MTT_ANALOG_BRAKE = 4
MTT_ANALOG_STEER = 5
MTT_SWITCHES_DIRECTION_MODE = 6


class SteeringMode(Enum):
    """Steering control mode: open loop (yaw rate) vs closed loop (articulation angle)"""
    OpenLoop = 0
    CloseLoop = 1


class DirectionState(Enum):
    Forward = 0x00
    Reverse = 0x01


class WinchState(Enum):
    WinchNeutral = 0x7F  # 127 - matches RF remote idle frame
    WinchIn = 0xE5  # 229
    WinchOut = 0x18  # 24


class SecuritySwitchState(Enum):
    SafetyLocked = 0x00
    SafetyUnlocked = 0x01


class VehicleType(Enum):
    VehicleSingleTrack = 0x00
    VehicleSbsLeft = 0x01
    VehicleSbsRight = 0x02


class LightState(Enum):
    Off = 0x00
    On = 0x01


@dataclass
class TachometerData:
    """Tachometer state (temps, raw ticks, timestamp)."""

    main_sensor_temp_a: float = 0.0
    main_sensor_temp_b: float = 0.0
    tachometer_instant: int = 0
    tachometer_cumulative: int = 0
    timestamp: float = 0.0
    monotonic_timestamp: float = 0.0
    new_data_available: bool = False

    def get_speed_ms(self, final_ratio: float) -> float:
        """Speed m/s from raw RPS."""
        if final_ratio == 0.0:
            return 0.0
        speed_kmh = (self.tachometer_instant / final_ratio) * MTT_TRACK_LENGTH_KM * 3600
        speed_ms = speed_kmh / 3.6
        return speed_ms

    def get_speed_kmh(self, final_ratio: float) -> float:
        """Speed km/h from raw RPS."""
        if final_ratio == 0.0:
            return 0.0
        return (self.tachometer_instant / final_ratio) * MTT_TRACK_LENGTH_KM * 3600

    def __str__(self):
        return (
            f"TachometerData(TempA={self.main_sensor_temp_a}°C, TempB={self.main_sensor_temp_b}°C, "
            f"Speed={self.tachometer_instant} RPS, Cumulative={self.tachometer_cumulative}, "
            f"Timestamp={self.timestamp:.3f}, NewData={self.new_data_available})"
        )


def interface_exists(ifname):
    """Check if a network interface exists (e.g. 'can0')."""
    SIOCGIFINDEX = 0x8933
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        fcntl.ioctl(s.fileno(), SIOCGIFINDEX, struct.pack("256s", ifname.encode("utf-8")[:15]))
        return True
    except OSError:
        return False


# Raw device resolution constants - from real MTT-154 measurements
THROTTLE_MAX = 230  # Firmware defined max usable throttle
BRAKE_MAX = 255  # Full scale brake
STEER_MAX = _mtt_params.steering_max_byte  # Steering value range: 255
STEER_CENTER = _mtt_params.steering_center_byte  # Center position: 127 (measured)
STEER_DEADBAND = _mtt_params.steering_deadband_normalized  # Normalized deadband: 0.5°/50° = 0.01


class MTTCanDriver:
    """Control/telemetry low-level CAN driver.

    High-level helpers accept normalized values (0..1 or -1..1) to abstract
    raw device resolution values from upstream clients.
    """

    def __init__(
        self,
        can_interface="can0",
        log_level=logging.INFO,
        can_id=0x001,
        telemetry_timeout_seconds=0.5,
    ):
        """Construct driver."""
        # Configure this driver's logging level
        setup_logging(log_level)
        self.frame_lock = threading.RLock()  # Re-entrant lock to avoid self-deadlock
        self.bus_reset_lock = threading.Lock()
        self.can_id = can_id
        self.can_interface = can_interface
        self.telemetry_timeout_seconds = max(0.0, float(telemetry_timeout_seconds))
        self._last_bus_reset_monotonic = 0.0
        self.bus_reset_cooldown_seconds = 1.0
        self.current_direction = DirectionState.Forward
        self.bus = None

        # Check CAN interface availability and initialize
        self._check_and_initialize_can_interface()
        
        # Initialize driver state variables
        self.vehicle_type = None
        self.steer_value = None
        self.throttle_value = None
        self.brake_value = None
        self.winch_state = None
        self.security_switch_state = None
        self.direction_state = None
        self.steering_mode = None
        self.light_state = None
        self.can_listener_running = True
        self.can_array = [0] * 8
        self._setup_initial_frame()

        self.tachometer_data = TachometerData()
        self.encoder_final_ratio = self._calculate_gear_ratio()
        self.can_listener_thread = threading.Thread(target=self._can_listener, daemon=True)
        self.can_listener_thread.start()

    def _setup_initial_frame(self):
        """Initialize all driver variables to default values."""
        log.info("Setting up initial frame...")
        
        self.set_vehicle_type(VehicleType.VehicleSingleTrack)
        log.info(f"Vehicle type set: {self.vehicle_type}")
        
        self._set_security_switch(SecuritySwitchState.SafetyUnlocked)
        log.info(f"Security switch set: {self.security_switch_state}")
        
        self.set_direction(DirectionState.Forward)
        log.info(f"Direction set: {self.direction_state}")
        
        self.set_light_state(LightState.Off)
        log.info(f"Light state set: {self.light_state}")
        
        self._set_throttle(0)
        log.info(f"Throttle set: {self.throttle_value}")
        
        self.set_winch_state(WinchState.WinchNeutral)
        log.info(f"Winch state set: {self.winch_state}")
        
        self._set_brake(0)
        log.info(f"Brake set: {self.brake_value}")
        
        
        self._set_steer(STEER_CENTER)
        log.info(f"Steer set: {self.steer_value}")
        
        self.set_steering_mode(SteeringMode.CloseLoop)
        log.info(f"Steering mode set: {self.steering_mode}")
        
        log.info("Initial frame setup completed - all variables initialized")

    def _check_and_initialize_can_interface(self, timeout_seconds=10, check_interval=0.5):
        """Check CAN interface availability and initialize the bus.

        Args:
            timeout_seconds (int): Maximum time to wait for interface availability
            check_interval (float): Time between availability checks

        Raises:
            Exception: If CAN interface is not available or initialization fails
        """
        elapsed = 0.0
        log.debug(f"Waiting for CAN interface '{self.can_interface}'...")

        while not interface_exists(self.can_interface) and elapsed < timeout_seconds:
            time.sleep(check_interval)
            elapsed += check_interval

        if not interface_exists(self.can_interface):
            log.error(f"CAN interface '{self.can_interface}' not found after {timeout_seconds} seconds.")
            raise Exception(f"CAN interface '{self.can_interface}' not available")

        try:
            self.bus = self._open_can_bus()
            log.info(f"Successfully initialized CAN bus on '{self.can_interface}'")
        except OSError as e:
            log.error(f"Failed to initialize CAN bus: {e}")
            raise

    def _open_can_bus(self):
        """Open the CAN bus with thread-safe send/recv operations when available."""
        bus_class = getattr(can, "ThreadSafeBus", can.interface.Bus)
        return bus_class(interface="socketcan", channel=self.can_interface)

    def __del__(self):
        self.cleanup()

    def cleanup(self):
        """Stop threads + shutdown bus."""
        self.can_listener_running = False
        if hasattr(self, "can_listener_thread"):
            self.can_listener_thread.join(timeout=1.0)

        with self.bus_reset_lock:
            if hasattr(self, "bus") and self.bus:
                self.bus.shutdown()
                self.bus = None
                log.info("CAN driver cleaned up")

    def _calculate_gear_ratio(self):
        """Compute final gear ratio per mtt_encoder_methodology.md."""
        # Mechanical gear reduction (motor rev per sprocket rev)
        G_theory = (MTT_GEAR2 / MTT_GEAR1) * (MTT_GEAR4 / MTT_GEAR3) * (MTT_GEAR_TRACK / MTT_GEAR_DRIVE)
        
        # Final ratio: mechanical × ticks per motor revolution  
        final_ratio = G_theory * MTT_TACHO_TEETH
        
        log.debug(f"Mechanical gear ratio (G_theory): {G_theory}")
        log.debug(f"Final encoder ratio: {final_ratio}")
        return final_ratio

    def _calculate_absolute_distance(self, cumulative_ticks):
        """Convert cumulative encoder ticks to absolute distance in meters (hardware specification)."""
        if cumulative_ticks == 0:
            return 0.0

        # Per mtt_encoder_methodology.md: Use consistent encoder_final_ratio = 324.0
        # (No division by 2, since final_ratio already accounts for 10 ticks per motor revolution)
        absolute_distance_m = (cumulative_ticks / self.encoder_final_ratio) * MTT_TRACK_LENGTH_M
        return absolute_distance_m

    #######################
    def _can_listener(self):
        """Listener thread: processes 0x2FF frames."""
        while self.can_listener_running:
            try:
                bus = self.bus
                if bus is None:
                    time.sleep(0.1)
                    continue
                message = bus.recv(timeout=0.1)
                if message and message.arbitration_id == CAN_MAIN_TELEMETRY:
                    self._process_tachometer_data(message.data)
            except (OSError, can.CanOperationError) as e:
                if self.can_listener_running:
                    log.warning(f"CAN listener error on {self.can_interface}: {e}")
                    self._reset_can_bus("receive failure")
                    time.sleep(0.1)
            except Exception as e:
                if self.can_listener_running:
                    log.debug(f"Unexpected CAN listener error: {e}")
                    time.sleep(0.1)

    def _process_tachometer_data(self, data):
        """Decode 0x2FF frame into tachometer state."""
        if len(data) != 8:
            return
        temp_a = struct.unpack("b", data[0:1])[0]
        temp_b = struct.unpack("b", data[1:2])[0]
        tachimeter_instant = struct.unpack(">H", data[2:4])[0]
        tachimeter_cumulative = struct.unpack(">I", data[4:8])[0]

        with self.frame_lock:
            self.tachometer_data.main_sensor_temp_a = float(temp_a)
            self.tachometer_data.main_sensor_temp_b = float(temp_b)
            self.tachometer_data.tachometer_instant = tachimeter_instant
            self.tachometer_data.tachometer_cumulative = tachimeter_cumulative
            self.tachometer_data.timestamp = time.time()
            self.tachometer_data.monotonic_timestamp = time.monotonic()
            self.tachometer_data.new_data_available = True

            # Convert cumulative ticks to absolute distance (hardware specification)
            absolute_distance_m = self._calculate_absolute_distance(tachimeter_cumulative)

            if tachimeter_cumulative % 50 == 0:
                speed_ms = self._get_current_speed_ms()
                log.debug(
                    f"Tachometer data - Speed: {speed_ms:.2f} m/s, "
                    f"Cumulative: {tachimeter_cumulative}, "
                    f"Temp A: {temp_a}°C, Temp B: {temp_b}°C"
                )

    def _get_current_speed_ms(self) -> float:
        """Signed speed (m/s)."""
        with self.frame_lock:
            if not self._tachometer_is_fresh_locked():
                return 0.0
            speed = self.tachometer_data.get_speed_ms(self.encoder_final_ratio)
            if self.current_direction == DirectionState.Reverse:
                speed = -speed
            return speed

    def _get_current_speed_kmh(self) -> float:
        """Signed speed (km/h)."""
        with self.frame_lock:
            if not self._tachometer_is_fresh_locked():
                return 0.0
            speed = self.tachometer_data.get_speed_kmh(self.encoder_final_ratio)
            if self.current_direction == DirectionState.Reverse:
                speed = -speed
            return speed

    def _tachometer_is_fresh_locked(self) -> bool:
        """Return True when telemetry has arrived recently enough to trust motion outputs."""
        if not self.tachometer_data.new_data_available:
            return False
        if self.telemetry_timeout_seconds <= 0.0:
            return True
        if self.tachometer_data.monotonic_timestamp <= 0.0:
            self.tachometer_data.new_data_available = False
            return False
        is_fresh = (time.monotonic() - self.tachometer_data.monotonic_timestamp) <= self.telemetry_timeout_seconds
        if not is_fresh:
            self.tachometer_data.new_data_available = False
        return is_fresh

    def get_tachometer_snapshot(self) -> TachometerData:
        """Return an immutable snapshot (copy) of tachometer data."""
        with self.frame_lock:
            snapshot = replace(self.tachometer_data)
            snapshot.new_data_available = self._tachometer_is_fresh_locked()
            return snapshot

    def get_odometry_snapshot(self) -> dict:
        """Return a consistent snapshot dict (distance always positive)."""
        with self.frame_lock:
            telemetry_is_fresh = self._tachometer_is_fresh_locked()
            speed_ms = 0.0
            speed_kmh = 0.0
            if telemetry_is_fresh:
                speed_ms = self.tachometer_data.get_speed_ms(self.encoder_final_ratio)
                speed_kmh = self.tachometer_data.get_speed_kmh(self.encoder_final_ratio)
                if self.current_direction == DirectionState.Reverse:
                    speed_ms = -speed_ms
                    speed_kmh = -speed_kmh
            ts = self.tachometer_data.timestamp
            monotonic_ts = self.tachometer_data.monotonic_timestamp
            return {
                "speed_ms": speed_ms,
                "speed_kmh": speed_kmh,
                "cumulative_ticks": self.tachometer_data.tachometer_cumulative,
                "absolute_distance_m": self._calculate_absolute_distance(self.tachometer_data.tachometer_cumulative),
                "timestamp": ts,
                "temperature_a": self.tachometer_data.main_sensor_temp_a,
                "temperature_b": self.tachometer_data.main_sensor_temp_b,
                "direction": self.get_direction().name,
                "data_age_ms": ((time.monotonic() - monotonic_ts) * 1000 if monotonic_ts > 0 else 0),
                "telemetry_is_fresh": telemetry_is_fresh,
            }
            # "direction": self.current_direction.name if self.current_direction else "Unknown",

    def get_command_state_snapshot(self) -> dict:
        """Return the current command-side state sent to the vehicle."""
        with self.frame_lock:
            return {
                "steer_raw": int(self.steer_value) if self.steer_value is not None else 0,
                "throttle_raw": int(self.throttle_value) if self.throttle_value is not None else 0,
                "brake_raw": int(self.brake_value) if self.brake_value is not None else 0,
                "direction": self.direction_state.name if self.direction_state else "Unknown",
                "safety_state": self.security_switch_state.name if self.security_switch_state else "Unknown",
            }


    #########################
    def emergency_stop(self):
        """Latch E-stop."""
        with self.frame_lock:
            if self.security_switch_state != SecuritySwitchState.SafetyLocked:
                self._set_security_switch(SecuritySwitchState.SafetyLocked)
            self._set_throttle(0)
            self._set_brake(BRAKE_MAX)
            self.set_winch_state(WinchState.WinchNeutral)
        log.warning("Local E-STOP applied in driver")

    def release_estop(self):
        """Release E-stop and restore idle command values."""
        with self.frame_lock:
            if self.security_switch_state != SecuritySwitchState.SafetyUnlocked:
                log.debug(f"Setting security switch to unlocked: before={self.can_array[MTT_SWITCHES_GLOBAL]:02X}")
                self._set_security_switch(SecuritySwitchState.SafetyUnlocked)
                log.debug(f"Setting security switch to unlocked: after={self.can_array[MTT_SWITCHES_GLOBAL]:02X}")
            else:
                log.debug("Security switch already unlocked")

        log.info("E-STOP released")

    def _set_security_switch(self, switch_value):
        """Set security switch (bit7)."""
        if switch_value == SecuritySwitchState.SafetyLocked:
            with self.frame_lock:
                self.can_array[MTT_SWITCHES_GLOBAL] &= 0b11110111
                self.security_switch_state = SecuritySwitchState.SafetyLocked
                log.debug("Safety locked")
            return True

        elif switch_value == SecuritySwitchState.SafetyUnlocked:
            with self.frame_lock:
                self.can_array[MTT_SWITCHES_GLOBAL] |= 0b00001000
                self.security_switch_state = SecuritySwitchState.SafetyUnlocked
                log.debug("Safety unlocked")
            return True

        else:
            log.error(f"invalid value for switch_value: {switch_value}")
            return False

    def _set_steer(self, steer_value):
        """Set raw steering value (0..STEER_MAX). Center ~STEER_CENTER."""
        if not isinstance(steer_value, int):
            log.error(f"steer_value is not an integer: {steer_value}")
            return False

        if steer_value >= 0 and steer_value <= STEER_MAX:
            with self.frame_lock:
                self.steer_value = steer_value
                self.can_array[MTT_ANALOG_STEER] = steer_value
            return True
        else:
            log.warning(f"out of bound value {steer_value} for steer_value")
            return False

    def _set_throttle(self, throttle_value):
        """Set raw throttle (0..THROTTLE_MAX)."""
        if not isinstance(throttle_value, int):
            log.error(f"throttle_value is not an integer: {throttle_value}")
            return

        if throttle_value >= 0 and throttle_value <= THROTTLE_MAX:
            with self.frame_lock:
                self.throttle_value = throttle_value
                self.can_array[MTT_ANALOG_THROTTLE] = throttle_value
            # log.info(f"throttle_value: {throttle_value}")
            return True
        else:
            log.warning(f"out of bound value {throttle_value} for throttle_value")
            return False

    def _set_brake(self, brake_value):
        """Set raw brake (0..BRAKE_MAX)."""
        if not isinstance(brake_value, int):
            log.error(f"brake_value is not an integer: {brake_value}")
            return False

        if brake_value >= 0 and brake_value <= BRAKE_MAX:
            with self.frame_lock:
                self.brake_value = brake_value
                self.can_array[MTT_ANALOG_BRAKE] = brake_value
            return True
        else:
            log.warning(f"out of bound value {brake_value} for brake_value")
            return False

    def set_direction(self, direction):
        """Set direction bit."""
        if direction == DirectionState.Forward:
            with self.frame_lock:
                self.can_array[MTT_SWITCHES_GLOBAL] |= 0b00100000
                self.direction_state = DirectionState.Forward
                self.current_direction = DirectionState.Forward
            return True

        elif direction == DirectionState.Reverse:
            with self.frame_lock:
                self.can_array[MTT_SWITCHES_GLOBAL] &= 0b11011111
                self.direction_state = DirectionState.Reverse
                self.current_direction = DirectionState.Reverse
            return True

        else:
            log.error(f"invalid value for direction: {direction}")
            return False

    def set_winch_state(self, winch_state):
        """Set winch state."""
        if winch_state == WinchState.WinchNeutral:
            self.can_array[MTT_ANALOG_WINCH] = winch_state.value
            self.winch_state = WinchState.WinchNeutral

            return True

        elif winch_state == WinchState.WinchIn:
            self.can_array[MTT_ANALOG_WINCH] = winch_state.value
            self.winch_state = WinchState.WinchIn

            return True

        elif winch_state == WinchState.WinchOut:
            self.can_array[MTT_ANALOG_WINCH] = winch_state.value
            self.winch_state = WinchState.WinchOut

            return True
        else:
            log.error(f"invalid value for winch_state: {winch_state}")
            return False

    def set_light_state(self, light_state):
        """Set light bit."""
        if light_state == LightState.Off:
            with self.frame_lock:
                self.light_state = LightState.Off
                self.can_array[MTT_SWITCHES_GLOBAL] |= 0b01000000
            return True

        elif light_state == LightState.On:
            with self.frame_lock:
                self.light_state = LightState.On
                self.can_array[MTT_SWITCHES_GLOBAL] &= 0b10111111
            return True
        else:
            log.error(f"invalid value for light_state: {light_state}")
            return False

    def set_steering_mode(self, steering_mode):
        """Set open/close loop bit0 of byte6."""
        if steering_mode == SteeringMode.CloseLoop:
            with self.frame_lock:
                self.can_array[MTT_SWITCHES_DIRECTION_MODE] |= 0b00000001
                self.steering_mode = SteeringMode.CloseLoop
            return True

        elif steering_mode == SteeringMode.OpenLoop:
            with self.frame_lock:
                self.can_array[MTT_SWITCHES_DIRECTION_MODE] &= 0b11111110
                self.steering_mode = SteeringMode.OpenLoop
            return True
        else:
            log.error(f"invalid value for steering_mode: {steering_mode}")
            return False

    def set_vehicle_type(self, vehicle_type):
        """Set vehicle type byte0."""
        if vehicle_type == VehicleType.VehicleSingleTrack:
            with self.frame_lock:
                self.can_array[MTT_SWITCHES_VEHICLE_TYPE] = vehicle_type.value
                self.vehicle_type = VehicleType.VehicleSingleTrack
            return True

        elif vehicle_type == VehicleType.VehicleSbsLeft:
            with self.frame_lock:
                self.can_array[MTT_SWITCHES_VEHICLE_TYPE] = vehicle_type.value
                self.vehicle_type = VehicleType.VehicleSbsLeft
            return True

        elif vehicle_type == VehicleType.VehicleSbsRight:
            with self.frame_lock:
                self.can_array[MTT_SWITCHES_VEHICLE_TYPE] = vehicle_type.value
                self.vehicle_type = VehicleType.VehicleSbsRight
            return True

        else:
            log.error(f"invalid value for vehicle_type: {vehicle_type}")
            return False

    def get_security_switch_state(self):
        """Get current security switch state (thread-safe)."""
        with self.frame_lock:
            return self.security_switch_state

    def _get_current_frame_hex(self):
        """Hex dump of current frame."""
        with self.frame_lock:
            return " ".join(f"{b:02X}" for b in self.can_array)

    def get_direction(self):
        """Get current direction from CAN frame."""
        with self.frame_lock:
            bit_set = self.can_array[MTT_SWITCHES_GLOBAL] & 0b00100000
            return DirectionState.Forward if bit_set else DirectionState.Reverse



    def send_can_frame(self):
        """Send keepalive frame to maintain communication."""
        # Check if all variables are properly initialized
        uninitialized_vars = []
        if self.vehicle_type == None:
            uninitialized_vars.append("vehicle_type")
        if self.steer_value == None:
            uninitialized_vars.append("steer_value")
        if self.throttle_value == None:
            uninitialized_vars.append("throttle_value")
        if self.brake_value == None:
            uninitialized_vars.append("brake_value")
        if self.winch_state == None:
            uninitialized_vars.append("winch_state")
        if self.security_switch_state == None:
            uninitialized_vars.append("security_switch_state")
        if self.direction_state == None:
            uninitialized_vars.append("direction_state")
        if self.steering_mode == None:
            uninitialized_vars.append("steering_mode")
        if self.light_state == None:
            uninitialized_vars.append("light_state")

        if uninitialized_vars:
            log.warning(f"CAN driver variables not initialized: {', '.join(uninitialized_vars)}")
            return

        bus = self.bus
        if bus is None:
            log.warning("CAN bus not initialized")
            return

        try:
            with self.frame_lock:
                frame_data = self.can_array.copy()

            message = can.Message(arbitration_id=self.can_id, data=frame_data, is_extended_id=False)
            bus.send(message)

        except (OSError, can.CanOperationError) as e:
            log.error(f"CAN send failed on {self.can_interface}: {e}")
            self._reset_can_bus("send failure")

    def set_throttle(self, percent: float):
        """Set throttle using percentage 0..1 (values outside range are clamped)."""
        if isinstance(percent, (int, float)) and not isinstance(percent, bool):
            percent = max(0.0, min(1.0, float(percent)))
            raw = int(round(percent * THROTTLE_MAX))
            self._set_throttle(raw)
            # log.info(f"Sending: {percent}")
            # log.info(f"RAW: {raw}")

            return True
        else:
            log.error(f"throttle percent not numeric: {percent}")
            return False

    def set_brake(self, percent: float):
        """Set brake using percentage 0..1 (values outside range are clamped)."""
        if isinstance(percent, (int, float)) and not isinstance(percent, bool):
            percent = max(0.0, min(1.0, float(percent)))
            raw = int(round(percent * BRAKE_MAX))
            self._set_brake(raw)
            return True
        else:
            log.error(f"brake percent not numeric: {percent}")
            return False

    def set_steer(self, steer_value: float):
        """Set steering using normalized value -1..1 (values outside range are clamped)."""
        if isinstance(steer_value, (int, float)) and not isinstance(steer_value, bool):
            clamped_value = max(-1.0, min(1.0, float(steer_value)))
            if abs(clamped_value) < STEER_DEADBAND:
                raw = STEER_CENTER
            else:
                raw = int(round((clamped_value + 1.0) * 0.5 * STEER_MAX))
                if raw in (STEER_CENTER, STEER_CENTER + 1) and abs(clamped_value) < (STEER_DEADBAND * 1.5):
                    raw = STEER_CENTER
            self._set_steer(raw)
            return True
        else:
            log.error(f"steer normalized not numeric: {steer_value}")
            return False

    def _reset_can_bus(self, reason: str):
        """Re-open the CAN bus after a transport-level failure without spamming resets."""
        now = time.monotonic()
        if (now - self._last_bus_reset_monotonic) < self.bus_reset_cooldown_seconds:
            return False

        with self.bus_reset_lock:
            now = time.monotonic()
            if (now - self._last_bus_reset_monotonic) < self.bus_reset_cooldown_seconds:
                return False
            self._last_bus_reset_monotonic = now
            old_bus = self.bus
            self.bus = None

            if old_bus is not None:
                try:
                    old_bus.shutdown()
                except Exception:
                    pass

            try:
                self.bus = self._open_can_bus()
                log.warning(f"Reinitialized CAN bus on '{self.can_interface}' after {reason}")
                return True
            except Exception as init_e:
                log.error(f"Failed to reinitialize CAN bus on '{self.can_interface}': {init_e}")
                self.bus = None
                return False


def main(args=None):
    mtt_driver = MTTCanDriver()


if __name__ == "__main__":
    main()
