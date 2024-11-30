cmake_minimum_required(VERSION 3.8)
project(vqw_package)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

find_package(roboclaw_hardware_interface REQUIRED)
find_package(roboclaw_serial REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rmw REQUIRED)
find_package(std_msgs REQUIRED)

find_package(backward_ros REQUIRED)

set(CMAKE_VERBOSE_MAKEFILE ON)

# Include directories
include_directories(
  include
)


add_executable(vqw_package src/vqwNav_node.cpp)
ament_target_dependencies(vqw_package rclcpp std_msgs roboclaw_hardware_interface roboclaw_serial)

##add_executable(test_roboclaw src/vqwNav_test_roboclaw.cpp)
##ament_target_dependencies(test_roboclaw rclcpp std_msgs roboclaw_hardware_interface roboclaw_serial)

target_include_directories(vqw_package PUBLIC
 $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
 $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)

target_compile_features(vqw_package PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

### target_link_libraries(${PROJECT_NAME} roboclaw_serial)


ament_target_dependencies(
  vqw_package
  roboclaw_hardware_interface
  roboclaw_serial
  rclcpp
)


install(TARGETS vqw_package
  DESTINATION lib/${PROJECT_NAME})

  # install(TARGETS test_roboclaw
  # DESTINATION lib/${PROJECT_NAME})

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
