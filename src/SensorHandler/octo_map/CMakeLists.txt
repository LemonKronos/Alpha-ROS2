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
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(octomap REQUIRED)
find_package(octomap_ros REQUIRED)
find_package(octomap_msgs REQUIRED)
find_package(PCL REQUIRED) # PCL is a bit special, usually found via PCLConfig

include_directories(include)

# ####################################### COMPONENT |>

add_library(octo_map_component SHARED
src/octo_map.cpp
)

target_include_directories(octo_map_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${PCL_INCLUDE_DIRS}
  ${OCTOMAP_INCLUDE_DIRS}
)

target_link_libraries(octo_map_component
  ${PCL_LIBRARIES}
  ${OCTOMAP_LIBRARIES}
)

ament_target_dependencies(octo_map_component
  rclcpp
  rclcpp_components
  global_utils
  ros2_msgs
  pcl_conversions
  pcl_ros
  octomap_ros
  octomap_msgs
)

rclcpp_components_register_nodes(octo_map_component "OctoMapNode")

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
