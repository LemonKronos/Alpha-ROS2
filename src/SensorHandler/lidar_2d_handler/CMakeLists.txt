cmake_minimum_required(VERSION 3.8)
project(lidar_2d_handler)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(ros2_msgs REQUIRED)
find_package(global_utils REQUIRED)

include_directories(
  include
)

# Sources
add_executable(lidar_2d_handler
  src/main.cpp
  src/lidar2d.cpp
  )

ament_target_dependencies(lidar_2d_handler
  rclcpp
  std_msgs
  px4_msgs
  sensor_msgs
  ros2_msgs
  global_utils
)

install(TARGETS
  lidar_2d_handler
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

ament_package()
