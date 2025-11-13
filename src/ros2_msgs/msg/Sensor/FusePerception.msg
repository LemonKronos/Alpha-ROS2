# Combine all necessary date for intermediate node
# Include Odometry, Contact, Lidar down, Lidar 2d, Depth camera and RGB camera

std_msgs/Header header

# Frame
uint8 POSE_FRAME_UNKNOWN = 0
uint8 POSE_FRAME_NED     = 1
uint8 POSE_FRAME_FRD     = 2
uint8 frame

# Position
float32[3] position

# Quaternion
float32[4] q

# Velocity
float32[3] velocity

# Angular velocity
float32[3] angular_velocity

# Contact sensor
bool bearable_contact
bool critical_contact

# Lidar down
float32 below_distance

# Lidar around contour
uint16 num_close    # Number of close contours
uint16 num_far      # Number of far contours
ros2_msgs/Lidar2dSector[] close_obstacles
ros2_msgs/Lidar2dSector[] far_obstacles

# Depth camera front
sensor_msgs/Image depth_cam_front

# RGB camera front
sensor_msgs/Image rgb_cam_front
