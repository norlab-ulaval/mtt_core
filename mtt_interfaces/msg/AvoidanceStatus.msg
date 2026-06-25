std_msgs/Header header

uint8 DISABLED=0
uint8 IDLE=1
uint8 FOLLOWING=2
uint8 HOLDING=3
uint8 PLANNING=4
uint8 SHADOW_READY=5
uint8 EXECUTING=6
uint8 RECOVERING=7
uint8 BLOCKED=8
uint8 SENSOR_FAULT=9

uint8 state
bool shadow_mode
bool execution_enabled
bool obstacle_stop
bool plan_available
bool plan_valid
float32 obstacle_age_s
float32 min_clearance_m
string detail
