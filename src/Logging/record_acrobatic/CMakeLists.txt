cmake_minimum_required(VERSION 3.8)
project(record_acrobatic)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ros2_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(ros_gz_interfaces REQUIRED)
find_package(global_utils REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

# Find CNPY (Might need manual path if not in standard /usr/local)
find_library(CNPY_LIB cnpy)

include_directories(
  include
)


# --- Record Acrobatic Node ---
add_executable(record_acrobatic
  src/record_acrobatic.cpp
)

ament_target_dependencies(record_acrobatic
  rclcpp
  ros2_msgs
  sensor_msgs
  ros_gz_interfaces
  global_utils
  cv_bridge
  OpenCV
)

target_link_libraries(record_acrobatic ${CNPY_LIB} z) # Link CNPY and ZLIB (CNPY uses zlib)

# --- Add Noob Control ---
add_executable(add_noob_control
  src/add_noob_control.cpp
)

ament_target_dependencies(add_noob_control
  rclcpp
  ros2_msgs
  ros_gz_interfaces
  global_utils
  cv_bridge
  OpenCV
)

target_link_libraries(add_noob_control ${CNPY_LIB} z) # Link CNPY and ZLIB (CNPY uses zlib)


install(TARGETS
  record_acrobatic
  add_noob_control
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
