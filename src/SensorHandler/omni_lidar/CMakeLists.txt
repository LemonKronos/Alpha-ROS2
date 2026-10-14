cmake_minimum_required(VERSION 3.8)
project(omni_lidar)

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
find_package(voxblox REQUIRED)
find_package(voxblox_ros REQUIRED)
find_package(glog REQUIRED) #! Voxblox leaky dependency
find_package(Eigen3 REQUIRED)

include_directories(include)

# ####################################### COMPONENT |>

add_library(omni_lidar_component SHARED
  src/omni_lidar.cpp
  src/component.cpp
)

target_include_directories(omni_lidar_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(omni_lidar_component
  glog::glog #! Voxblox leaky dependency
)

ament_target_dependencies(omni_lidar_component
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
  voxblox
  voxblox_ros
)

rclcpp_components_register_nodes(omni_lidar_component "alpha_brain::OmniLidarNode")

# ####################################### EXECUTABLE |>

add_executable(omni_lidar
  src/main.cpp
)

target_link_libraries(omni_lidar
  omni_lidar_component
)

ament_target_dependencies(omni_lidar
  rclcpp
  rclcpp_components
)

install(TARGETS
  omni_lidar_component 
  omni_lidar
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

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
