cmake_minimum_required(VERSION 3.8)
project(octo_map)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(global_utils REQUIRED)
find_package(ros2_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(octomap REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(include)

# ####################################### COMPONENT |>

add_library(octo_map_component SHARED
  src/octo_map.cpp
  src/octo_map_threads.cpp
  src/component.cpp
)

target_include_directories(octo_map_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${OCTOMAP_INCLUDE_DIRS}
)

target_link_libraries(octo_map_component
  ${OCTOMAP_LIBRARIES}
)

ament_target_dependencies(octo_map_component
  rclcpp
  rclcpp_components
  global_utils
  ros2_msgs
  std_msgs
  sensor_msgs
  tf2_ros
  tf2_sensor_msgs
  tf2_eigen
  octomap
)

rclcpp_components_register_nodes(octo_map_component "alpha_brain::OctoMapNode")

# ####################################### EXECUTABLE |>

add_executable(octo_map
  src/main.cpp
)

target_link_libraries(octo_map
  octo_map_component
)

ament_target_dependencies(octo_map
  rclcpp
  rclcpp_components
)

install(TARGETS
  octo_map_component 
  octo_map
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
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
