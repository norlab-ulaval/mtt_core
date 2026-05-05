# MTT-154 Vehicle Status Message - Aggregated telemetry data
# Contains all vehicle status information in a single clean message

# Standard ROS header
std_msgs/Header header

# CAN transport / freshness
string can_interface
uint32 command_can_id        # Command arbitration ID in use (0x001 or 0x100)
uint32 telemetry_can_id      # Telemetry arbitration ID, normally 0x2FF
bool telemetry_seen_once
bool telemetry_fresh
float64 telemetry_age_ms
bool tachometer_is_synthetic
string tachometer_source

# Movement and positioning
float64 speed_ms            # Current speed in meters per second
float64 speed_kmh           # Current speed in kilometers per hour  
float64 distance_km         # Total distance traveled in kilometers
string direction            # Current direction: "Forward", "Reverse", or "Unknown"

# Mechanical status
float64 temperature_a       # Main sensor temperature A in Celsius
float64 temperature_b       # Main sensor temperature B in Celsius
uint8 steer_position        # Legacy alias for steer_raw
int8 main_sensor_temp_a_raw
int8 main_sensor_temp_b_raw

# Raw sensor data (for debugging and advanced usage)
uint16 tachometer_instant     # Instantaneous tachometer reading (RPS)
uint32 tachometer_cumulative  # Cumulative tachometer reading (total ticks)
uint16 tachometer_instant_ticks_per_s
uint32 tachometer_cumulative_ticks
uint8 vehicle_type_raw
string vehicle_type_label
bool security_unlocked
bool light_off_estop_patch
bool direction_reverse
uint8 throttle_raw
uint8 brake_raw
uint8 steer_raw
float64 steer_normalized
float64 command_linear_speed_ms
float64 effective_linear_speed_command_ms
bool hold_assist_active
string hold_assist_mode
float64 hold_assist_output_ms
uint8 winch_raw
string winch_state
bool steering_mode_closed_loop
uint8 reserved_byte_7

# System status
bool emergency_stop_active   # Emergency stop state
bool remote_connected        # Remote controller connection status
bool deadman_active          # deadman_is_active
bool command_timeout_active  # Driver neutralized throttle/steering because cmd_vel went stale
bool can_debug_enabled       # mtt_can_node is publishing decoded CAN debug frames
string safety_state          #  System estopped (start, wrapper closed, remote lost, deadman released), 
bool has_main_controller_version
uint32 main_hardware_revision_raw
uint32 main_software_revision_raw
bool has_battery_controller_version
uint32 battery_hardware_revision_raw
uint32 battery_software_revision_raw
