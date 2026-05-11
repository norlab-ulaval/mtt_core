std_msgs/Header header

# Commanded articulation angle from motion model (from mtt_tachometer)
float64 command_rad
bool    command_valid

# Hardware encoder feedback (from /hardware/articulation_angle)
float64 hardware_rad
bool    hardware_fresh

# Effective angle used by odometry (hardware if fresh, else model estimate)
float64 effective_rad
string  effective_source   # "hardware" or "model"

# LiDAR perception result (from trailer_detector KF, trailer/articulation_angle)
float64 lidar_rad
bool    lidar_detected

# Diagnostic residuals
float64 command_residual_rad         # command_rad - effective_rad
float64 hardware_lidar_residual_rad  # hardware_rad - lidar_rad (valid only when both fresh)
