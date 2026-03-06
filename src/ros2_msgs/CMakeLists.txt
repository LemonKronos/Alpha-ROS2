cmake_minimum_required(VERSION 3.5)
project(ros2_msgs)

find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Collect all msg files
set(MSGS_DIR "${CMAKE_CURRENT_SOURCE_DIR}/msg")
file(GLOB_RECURSE ROS2_MSGS RELATIVE "${CMAKE_CURRENT_SOURCE_DIR}" "${MSGS_DIR}/*.msg")

# Generate the messages
rosidl_generate_interfaces(${PROJECT_NAME}
  ${ROS2_MSGS}
  DEPENDENCIES std_msgs   # <-- Important! External msg packages must be listed here
  DEPENDENCIES sensor_msgs
  DEPENDENCIES geometry_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
