
#_ Acrobatic OA actuator command output and safety value 

std_msgs/Header header

# Motor actuators
# [@range -1, 1] Normalized thrust. where 1 means maximum positive thrust, -1 maximum negative (if not supported by the output, <0 maps to NaN). NaN maps to disarmed (stop the motors)
float32[12] motors_control

# Servo actuators
# [@range -1, 1] Normalized output. 1 means maximum positive position. -1 maximum negative position (if not supported by the output, <0 maps to NaN). NaN maps to disarmed.
float32[8] servos_control 