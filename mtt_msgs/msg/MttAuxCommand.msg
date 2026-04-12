# Custom message for MTT-154 auxiliary commands

# Analog Brake Command
# Range: 0.0 (no brake) to 1.0 (full brake)
float32 brake

# Winch Command
# WINCH_NEUTRAL = 0
# WINCH_IN = 1
# WINCH_OUT = 2
uint8 winch_command

# Light light_state
int32 light_state  # 0=off, 1=on
