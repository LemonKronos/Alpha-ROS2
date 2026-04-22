cmake_minimum_required(VERSION 3.8)
project(depth_cam)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(global_utils REQUIRED)
find_package(alpha_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(octomap REQUIRED)
find_package(voxblox REQUIRED)
find_package(voxblox_ros REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(include)

# ####################################### COMPONENT |>

add_library(depth_cam_component SHARED
  src/depth_cam.cpp
  src/depth_cam_threads.cpp
  src/component.cpp
)

target_include_directories(depth_cam_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${OCTOMAP_INCLUDE_DIRS}
)

target_link_libraries(depth_cam_component
  ${OCTOMAP_LIBRARIES}
)

ament_target_dependencies(depth_cam_component
  rclcpp
  rclcpp_components
  global_utils
  alpha_msgs
  std_msgs
  sensor_msgs
  tf2_ros
  tf2_sensor_msgs
  tf2_eigen
  Eigen3
  octomap
  voxblox
  voxblox_ros
)

rclcpp_components_register_nodes(depth_cam_component "alpha_brain::DepthCamNode")

# ####################################### EXECUTABLE |>

add_executable(depth_cam
  src/main.cpp
)

target_link_libraries(depth_cam
  depth_cam_component
)

ament_target_dependencies(depth_cam
  rclcpp
  rclcpp_components
)

install(TARGETS
  depth_cam_component 
  depth_cam
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
