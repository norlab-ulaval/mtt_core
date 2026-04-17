# MTT-154 Battery Management System Data
# Decoded from CAN frames 0x600-0x603 (ROYPOW S51105 BMS).
# Published by mtt_can_node on topic "mtt_battery/status".

std_msgs/Header header

# ── State of Charge & Pack (from 0x602) ─────────────────────────────
float32 soc_percent           # State of charge (0-100 %)
float32 pack_voltage          # Total pack voltage (V)
float32 pack_current          # Pack current (A), positive = discharging
float32 remaining_capacity    # Remaining capacity (Ah)

# ── Cell temperatures (from 0x601) ──────────────────────────────────
float32 cell_temp_1           # Cell group 1 temperature (°C)
float32 cell_temp_2           # Cell group 2 temperature (°C)
float32 cell_temp_3           # Cell group 3 temperature (°C)
float32 cell_temp_4           # Cell group 4 temperature (°C)

# ── System temperatures (from 0x603) ────────────────────────────────
float32 ambient_temp          # Ambient temperature (°C)
float32 mosfet_temp           # MOSFET temperature (°C)

# ── Computed power metrics ───────────────────────────────────────────
float32 power_watts           # Instantaneous electrical power (W) = voltage * current
float32 energy_consumed_wh    # Cumulative energy consumed this session (Wh)

# ── Data freshness flags ─────────────────────────────────────────────
bool has_soc                  # true if 0x602 has been received
bool has_cell_temps           # true if 0x601 has been received
bool has_sys_temps            # true if 0x603 has been received
