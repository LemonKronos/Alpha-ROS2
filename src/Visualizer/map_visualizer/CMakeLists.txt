cmake_minimum_required(VERSION 3.8)
project(map_visualizer)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(global_utils REQUIRED)
find_package(alpha_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(voxblox REQUIRED)
find_package(voxblox_ros REQUIRED)
find_package(glog REQUIRED) #! Voxblox leaky dependency
find_package(Eigen3 REQUIRED)

include_directories(include)

# ####################################### COMPONENT |>

add_library(map_visualizer_component SHARED
  src/map_visualizer.cpp
  src/component.cpp # <-- Added this line
)

target_include_directories(map_visualizer_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(map_visualizer_component
  glog::glog #! Voxblox leaky dependency
)

ament_target_dependencies(map_visualizer_component
  rclcpp
  rclcpp_components
  global_utils
  alpha_msgs
  visualization_msgs
  geometry_msgs
  std_msgs
  tf2_ros
  tf2_eigen
  Eigen3
  voxblox
  voxblox_ros
)

rclcpp_components_register_nodes(map_visualizer_component "alpha_brain::MapVisualizerNode")

# ####################################### EXECUTABLE |>

add_executable(map_visualizer
  src/main.cpp
)

target_link_libraries(map_visualizer
  map_visualizer_component
)

ament_target_dependencies(map_visualizer
  rclcpp
  rclcpp_components
)

# ####################################### INSTALL |>

install(TARGETS
  map_visualizer_component 
  map_visualizer
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()