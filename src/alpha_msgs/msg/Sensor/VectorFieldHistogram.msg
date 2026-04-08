# VectorFieldHistogram.msg
# Store the Vector Field Histogram and the closest obstacle

std_msgs/Header header

# The VFH as int32 array, fixed size
int32[81] vfh_part

# Closest obstacle in spherical coordinate # Or could make a array of important points
geometry_msgs/Point32 closest_obstacle