# MTT health / operator monitoring message.
# This message is intentionally explicit:
# - raw fields keep the CAN payload meaning
# - estimated fields are marked as such
# - invalid physical quantities remain invalid instead of inventing units

std_msgs/Header header

# ── Runtime / command context ────────────────────────────────────────
string can_interface
uint32 command_can_id
bool external_control_active
bool can_debug_enabled
bool can_debug_available
string steer_control_mode
string cmd_angular_mode
string direction
bool security_unlocked
bool light_off_estop_patch
bool emergency_stop_active
bool deadman_active
bool remote_connected
bool command_timeout_active

# ── Command snapshot ─────────────────────────────────────────────────
uint8 throttle_raw
uint8 brake_raw
uint8 steer_raw
float64 steer_normalized
float64 commanded_linear_speed_ms
float64 commanded_angular_input
float64 commanded_yaw_rate_rad_s

# ── Telemetry / CAN freshness ────────────────────────────────────────
bool telemetry_fresh
float64 telemetry_age_ms
bool tachometer_present
bool tachometer_stale
bool tachometer_is_synthetic
string tachometer_source
float32 telemetry_frame_hz
float32 bms_cell_frame_hz
float32 bms_sys_frame_hz
float32 bms_core_frame_hz
float32 bms_datetime_frame_hz
float32 charger_status_frame_hz

# ── Temperatures ─────────────────────────────────────────────────────
float64 main_sensor_temp_a_c
float64 main_sensor_temp_b_c
float32 cell_temp_1_c
float32 cell_temp_2_c
float32 cell_temp_3_c
float32 cell_temp_4_c
float32 ambient_temp_c
float32 mosfet_temp_c
float32 heatpad_a_temp_c
float32 heatpad_b_temp_c
string controller_temp_state
string encoder_temp_state
string battery_temp_state

# ── Battery / power ──────────────────────────────────────────────────
uint8 soc_percent
bool low_soc_warning
int16 battery_current_raw
float32 battery_current_estimated_a
bool battery_current_estimated_valid
uint16 battery_voltage_raw
float32 battery_voltage_v
bool battery_voltage_valid
float32 power_watts
bool power_valid
uint16 charge_time_remaining_min
bool heatpad_a_on
bool heatpad_b_on

# ── Fallback command-only motion estimate ────────────────────────────
bool fallback_active
bool fallback_low_confidence
string fallback_reason
float64 fallback_speed_ms
float64 fallback_yaw_rate_rad_s
float64 fallback_distance_m
float64 fallback_heading_rad

# ── Shared command-motion model debug ────────────────────────────────
bool model_state_valid
float64 model_command_linear_speed_ms
float64 model_speed_ms
float64 model_articulation_command_rad
float64 model_articulation_effective_rad
float64 model_curvature_nominal_m_inv
float64 model_curvature_effective_m_inv
float64 model_yaw_rate_nominal_rad_s
float64 model_yaw_rate_effective_rad_s

# ── Warnings / operator state ────────────────────────────────────────
bool brake_and_throttle_conflict
bool high_encoder_temp
bool high_controller_temp
bool bms_temp_warning
bool tachometer_fault_suspected
bool can_stale_warning
bool steering_mode_mismatch
string health_summary
string[] warnings
