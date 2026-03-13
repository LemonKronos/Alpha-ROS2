# Control input in FLU
# This act as the interface for both human and AI control

std_msgs/Header header

uint8 TESTER = 0
uint8 REACTIVE_OA = 1
uint8 ACROBATIC_OA = 2
uint8 LEADER = 3
uint8 NAVIGATOR = 4
uint8 OPERATOR = 5
uint8 control_by 

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
