cmake_minimum_required(VERSION 3.8)
project(low_spacial)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(alpha_msgs REQUIRED)
find_package(global_utils REQUIRED)
find_package(octomap REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(
  include
)

# ####################################### COMPONENT |>

add_library(low_spacial_component SHARED
  src/low_spacial.cpp
  src/component.cpp
)

target_include_directories(low_spacial_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${OCTOMAP_INCLUDE_DIRS}
)

target_link_libraries(low_spacial_component
  ${OCTOMAP_LIBRARIES}
)

ament_target_dependencies(low_spacial_component
  rclcpp
  rclcpp_components
  global_utils
  alpha_msgs
  Eigen3
  octomap
)

rclcpp_components_register_nodes(low_spacial_component "alpha_brain::LowSpacialNode")

# ####################################### EXECUTABLE |>

add_executable(low_spacial
  src/main.cpp
)

target_link_libraries(low_spacial
  low_spacial_component
)

ament_target_dependencies(low_spacial
  rclcpp
  rclcpp_components
)

install(TARGETS
  low_spacial_component
  low_spacial
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
