# Contain corection for movement in body frame FRD
# Design to send to VehicleAttitudeSetpoint
# If this topic exist, it will replace ControlInput

std_msgs/Header header

float32[4] quaternion

float32 yaw_velocity # rad/s

float32[3] thrust # Normalized [-1, 1]