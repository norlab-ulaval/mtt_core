# MTT-154 Vehicle Status Message - Aggregated telemetry data
# Contains all vehicle status information in a single clean message

# Standard ROS header
std_msgs/Header header

# Movement and positioning
float64 speed_ms            # Current speed in meters per second
float64 speed_kmh           # Current speed in kilometers per hour  
float64 distance_km         # Total distance traveled in kilometers
string direction            # Current direction: "Forward", "Reverse", or "Unknown"

# Mechanical status
float64 temperature_a       # Main sensor temperature A in Celsius
float64 temperature_b       # Main sensor temperature B in Celsius
uint8 steer_position        # Current steering position (raw value)

# Raw sensor data (for debugging and advanced usage)
uint16 tachometer_instant     # Instantaneous tachometer reading (RPS)
uint32 tachometer_cumulative  # Cumulative tachometer reading (total ticks)

# System status
bool emergency_stop_active   # Emergency stop state
bool remote_connected        # Remote controller connection status
bool deadman_active          # deadman_is_active
string safety_state          #  System estopped (start, wrapper closed, remote lost, deadman released), 