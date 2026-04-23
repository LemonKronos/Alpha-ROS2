cmake_minimum_required(VERSION 3.8)
project(global_utils)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(alpha_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(voxblox REQUIRED)
find_package(voxblox_ros REQUIRED)

# build the library (no main function needed)
add_library(${PROJECT_NAME}
  src/surrounding.cpp
  src/utils.cpp
  src/system_config.cpp
  src/alpha_brain.cpp
)

set_target_properties(global_utils PROPERTIES POSITION_INDEPENDENT_CODE ON)

target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${EIGEN3_INCLUDE_DIR}
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  std_msgs
  alpha_msgs
  Eigen3
  voxblox
  voxblox_ros
)

# export + install the library
install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# install headers
install(
  DIRECTORY include/
  DESTINATION include
)

# export info for other packages
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(rclcpp std_msgs Eigen3 voxblox voxblox_ros)

# testing (optional)
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
