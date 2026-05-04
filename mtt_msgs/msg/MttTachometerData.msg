# MTT-154 Tachometer Data Message (CANBus_Specification.md v1.1)
# Contains complete telemetry data from vehicle CAN 0x2FF

# Standard ROS header
std_msgs/Header header

# Source telemetry state
uint32 telemetry_can_id      # Source CAN ID, normally 0x2FF
bool telemetry_seen_once     # true once at least one real 0x2FF frame has been received
bool telemetry_fresh         # true when the last 0x2FF frame is within the configured timeout
float64 telemetry_age_ms     # age of the last received 0x2FF frame in milliseconds
bool tachometer_is_synthetic # true when the 0x2FF stream is generated from software, not a physical encoder
string tachometer_source     # "real" or "cmd_sim"

# Temperature sensors in Celsius
float64 main_sensor_temp_a
float64 main_sensor_temp_b

# Raw tachometer values (MSB-first from CAN)
uint16 tachometer_instant     # Instantaneous speed in raw encoder ticks per second (RPS)
uint32 tachometer_cumulative  # Cumulative distance in raw encoder ticks

# Calculated speed values
float64 speed_ms    # Speed in meters per second
float64 speed_kmh   # Speed in kilometers per hour

# Calculated distance
float64 distance_km # Cumulative distance in kilometers

# Vehicle direction state
string direction    # Current direction: "Forward" or "Reverse"

# Steering command value sent to CAN bus
float64 steer_cmd    # Raw steering command (-1.0 to +1.0) sent to vehicle
                     # Interpretation depends on control mode (open/closed loop)

# Shared command-motion model debug (valid for synthetic tachometer / fallback)
bool model_state_valid
float64 model_command_linear_speed_ms
float64 model_speed_ms
float64 model_articulation_command_rad
float64 model_articulation_effective_rad
float64 model_curvature_nominal_m_inv
float64 model_curvature_effective_m_inv
float64 model_yaw_rate_nominal_rad_s
float64 model_yaw_rate_effective_rad_s
