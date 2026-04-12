# MTT-154 Driving Mode Command
# Defines the operational mode for the vehicle system

# Driving mode constants
uint8 SINGLE_TRAILER = 0      # Default: Single tractor with trailer (slip + central joint)
uint8 DUAL_DIFFERENTIAL = 1   # Two tractors side-by-side (skid steer behavior)
uint8 DUAL_SERPENTINE = 2     # Two tractors in line (front-back serpentine)

# Current requested mode
uint8 mode

# Optional parameters for mode-specific configuration
string mode_parameters
