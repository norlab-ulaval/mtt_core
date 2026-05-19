std_msgs/Header header

# Commanded articulation angle from motion model (from mtt_tachometer)
float64 command_rad
bool    command_valid

# Hardware encoder feedback — yaw (from /hardware/articulation_angle, ADC2 12-bit)
float64 hardware_rad
bool    hardware_fresh

# Effective yaw angle used by odometry (hardware if fresh, else model estimate)
float64 effective_rad
string  effective_source   # "hardware" or "model"

# Hardware encoder feedback — pitch (from /hardware/articulation_pitch_rad, ADC1 8-bit)
# Hitch longitudinal/tilt angle. pitch_rad = 0 until LUT is calibrated.
float64 pitch_rad
float64 pitch_bits_raw     # filtered raw ADC bits — for offline LUT calibration
bool    pitch_fresh

# LiDAR perception result (from trailer_detector KF, trailer/articulation_angle)
float64 lidar_rad
bool    lidar_detected

# Diagnostic residuals
float64 command_residual_rad         # command_rad - effective_rad
float64 hardware_lidar_residual_rad  # hardware_rad - lidar_rad (valid only when both fresh)
