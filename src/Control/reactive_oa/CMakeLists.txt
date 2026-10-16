cmake_minimum_required(VERSION 3.8)
project(reactive_oa)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(alpha_msgs REQUIRED)
find_package(global_utils REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(osqp REQUIRED)
find_package(OsqpEigen REQUIRED)
find_package(glog REQUIRED) #! Voxblox leaky dependency

include_directories(include)

# ####################################### COMPONENT |>

add_library(reactive_oa_component SHARED
  src/reactive_oa.cpp
  src/component.cpp
)

target_include_directories(reactive_oa_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(reactive_oa_component
  glog::glog #! Voxblox leaky dependency
)

ament_target_dependencies(reactive_oa_component
  rclcpp
  alpha_msgs
  global_utils
  visualization_msgs
  Eigen3
  osqp
  OsqpEigen
)

rclcpp_components_register_nodes(reactive_oa_component "alpha_brain::ReactiveOANode")

# ####################################### EXECUTABLE |>

add_executable(reactive_oa
  src/main.cpp
)

target_link_libraries(reactive_oa
  reactive_oa_component
)

ament_target_dependencies(reactive_oa
  rclcpp
  rclcpp_components
)

target_link_libraries(reactive_oa
  osqp::osqp
  OsqpEigen::OsqpEigen
)

install(TARGETS
  reactive_oa_component
  reactive_oa
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
