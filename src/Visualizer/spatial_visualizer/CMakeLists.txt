cmake_minimum_required(VERSION 3.8)
project(spatial_visualizer)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(global_utils REQUIRED)
find_package(alpha_msgs REQUIRED)

include_directories(include)

# ####################################### COMPONENT |>

add_library(spatial_visualizer_component SHARED
  src/spatial_visualizer.cpp
  src/component.cpp
)

target_include_directories(spatial_visualizer_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(spatial_visualizer_component
  glog::glog #! Voxblox leaky dependency
)

ament_target_dependencies(spatial_visualizer_component
  rclcpp
  rclcpp_components
  global_utils
  alpha_msgs
)

rclcpp_components_register_nodes(spatial_visualizer_component "spatial_container::SpatialVisualizerNode")

# ####################################### EXECUTABLE |>

add_executable(spatial_visualizer
  src/main.cpp
)

target_link_libraries(spatial_visualizer
  spatial_visualizer_component
)

ament_target_dependencies(spatial_visualizer
  rclcpp
  rclcpp_components
)

install(TARGETS
  spatial_visualizer_component
  spatial_visualizer
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
