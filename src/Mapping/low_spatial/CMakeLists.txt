cmake_minimum_required(VERSION 3.8)
project(low_spatial)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(global_utils REQUIRED)
find_package(alpha_msgs REQUIRED)
find_package(voxblox REQUIRED)
find_package(voxblox_ros REQUIRED)
find_package(glog REQUIRED) #! Voxblox leaky dependency
find_package(Eigen3 REQUIRED)

include_directories(include)

# ####################################### COMPONENT |>

add_library(low_spatial_component SHARED
  src/low_spatial.cpp
  src/component.cpp
)

target_include_directories(low_spatial_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(low_spatial_component
  glog::glog #! Voxblox leaky dependency
)

ament_target_dependencies(low_spatial_component
  rclcpp
  rclcpp_components
  global_utils
  alpha_msgs
  Eigen3
  voxblox
  voxblox_ros
)

rclcpp_components_register_nodes(low_spatial_component "alpha_brain::LowSpatialNode")

# ####################################### EXECUTABLE |>

add_executable(low_spatial
  src/main.cpp
)

target_link_libraries(low_spatial
  low_spatial_component
)

ament_target_dependencies(low_spatial
  rclcpp
  rclcpp_components
)

install(TARGETS
  low_spatial_component
  low_spatial
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
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
