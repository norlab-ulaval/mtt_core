# MTT-154 Battery Management System Data
# Decoded from CAN frames 0x600-0x603 and charger frames 0x1806E5F4 / 0x18FF50E5.
# Published by mtt_can_node on topic "mtt_battery/status".
#
# Semantic rule:
# - fields ending with `_raw` are direct CAN values
# - physical fields are published only when their scaling is actually known
# - `*_valid` flags must be checked before trusting physical values

std_msgs/Header header

# ── Frame availability ───────────────────────────────────────────────
bool has_soc                  # true if 0x602 has been received
bool has_cell_temps           # true if 0x600 has been received
bool has_sys_temps            # true if 0x601 has been received
bool has_datetime             # true if 0x603 raw date/time fields have been received
bool has_charger_command      # true if charger command frame 0x1806E5F4 has been received
bool has_charger_status       # true if charger status frame 0x18FF50E5 has been received

# ── Core battery state (0x602) ───────────────────────────────────────
uint8 soc_percent             # State of charge (0-100 %)
int16 battery_current_raw
uint16 battery_voltage_raw
uint16 charge_time_remaining_min
bool heatpad_a_on
bool heatpad_b_on
uint8 heatpads_reserved

# ── Physical interpretation validity ────────────────────────────────
bool battery_current_estimated_valid
bool battery_voltage_valid
bool power_valid

# ── Physical / estimated quantities ─────────────────────────────────
float32 battery_current_estimated_a   # From the provided DBC scale: current_raw * 0.0103 - 0.72
float32 battery_voltage_v             # Not published until voltage scaling is confirmed
float32 power_watts                   # Not published until voltage scaling is confirmed
float32 energy_consumed_wh            # Integrated only when `power_valid` is true

# ── Battery temperatures: cell groups (0x600) ───────────────────────
# These are documented as direct int16 temperatures in the local CAN spec.
int16 cell_temp_1_raw
int16 cell_temp_2_raw
int16 cell_temp_3_raw
int16 cell_temp_4_raw
float32 cell_temp_1_c
float32 cell_temp_2_c
float32 cell_temp_3_c
float32 cell_temp_4_c

# ── Battery temperatures: enclosure / power electronics (0x601) ─────
# Ambient = battery enclosure ambient.
# MOSFET = battery power electronics temperature.
# Heatpad A/B = heating pad temperatures inside the battery system.
int16 ambient_temp_raw
int16 mosfet_temp_raw
int16 heatpad_a_temp_raw
int16 heatpad_b_temp_raw
float32 ambient_temp_c
float32 mosfet_temp_c
float32 heatpad_a_temp_c
float32 heatpad_b_temp_c

# ── Raw date/time payload (0x603) ────────────────────────────────────
uint16 charge_time_remaining_603_raw
uint16 year_month_raw
uint16 day_hour_raw
uint16 minute_second_raw

# ── Charger 29-bit frames (raw only for now) ─────────────────────────
uint16 charger_max_voltage_raw
uint16 charger_max_current_raw
uint16 charger_configured_voltage_raw
uint16 charger_configured_current_raw
