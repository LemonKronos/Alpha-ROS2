cmake_minimum_required(VERSION 3.8)
project(simulation_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(alpha_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(ros_gz_interfaces REQUIRED)
find_package(global_utils REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(
  include
)

add_executable(simulation_control
  src/simulation_control.cpp
)

ament_target_dependencies(simulation_control
  rclcpp
  alpha_msgs
  ros_gz_interfaces
  global_utils
  cv_bridge
  OpenCV
)

install(TARGETS
  simulation_control
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
