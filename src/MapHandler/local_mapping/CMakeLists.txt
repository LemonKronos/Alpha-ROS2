cmake_minimum_required(VERSION 3.8)
project(local_mapping)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(global_utils REQUIRED)
find_package(ros2_msgs REQUIRED)

include_directories(
  include
)

# Sources
add_executable(local_mapping
  src/main.cpp
  src/local_mapping.cpp
  src/visualizer_lidar2d.cpp
  )

ament_target_dependencies(local_mapping
  rclcpp
  std_msgs
  ros2_msgs
  global_utils
  visualization_msgs
)

install(TARGETS
  local_mapping
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

ament_package()
