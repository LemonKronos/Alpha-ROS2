# Control input in body frame FLU

std_msgs/Header header

#: How much support is allowed: higher mean allow more control support
uint8 control_support
uint8 NONE = 0 # No safety
uint8 AUTOPILOT = 1 # Allow RTL
uint8 REACTIVE_OA = 2 # Reactive safety
uint8 ACROBATIC_OA = 3 # Acrobatic maneuver

#: Who piloting the drone: lower rank can override the higher ones
uint8 pilot
uint8 DEV = 0
uint8 HUMAN = 1
uint8 LEADER = 2 # Swarm leader
uint8 PLANNER = 3 # Machine planner
uint8 SAFETY = 4 # For when control signal is lost

bool DISARM = 0
bool ARM = 1
bool control_state

# Rate velocity rad/s
float32 roll
float32 pitch
float32 yaw

# Velocity m/s
float32 forward
float32 left
float32 up

# Wings mode
uint8 UNCHANGE = 0
uint8 MULTICOPTER = 1
uint8 FIXWINGS = 2
uint8 wings_mode
