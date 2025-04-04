cmake_minimum_required(VERSION 3.8)
project(joy_pan_tilt)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(backward_ros REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)


add_library(joy_pan_tilt_component src/joy_pan_tilt.cpp)
##add_library(joy_pan_tilt::joy_pan_tilt ALIAS joy_pan_tilt)

target_compile_features(joy_pan_tilt_component PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_include_directories(joy_pan_tilt_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)

ament_target_dependencies(
  joy_pan_tilt_component
  "pluginlib"
  "rclcpp"
  "rclcpp_components"
  "rclcpp_lifecycle"
  "backward_ros"
  "std_msgs"
  "sensor_msgs"
)

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(joy_pan_tilt_component PRIVATE "JOY_PAN_TILT_BUILDING_LIBRARY")

rclcpp_components_register_node(
  joy_pan_tilt_component
    PLUGIN "joy_pan_tilt::JoyPanTilt"
    EXECUTABLE joy_pan_tilt
)

##pluginlib_export_plugin_description_file(${PROJECT_NAME} joy_pan_tilt.xml)

install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

ament_export_libraries(
  joy_pan_tilt_component
)
ament_export_targets(
  export_joy_pan_tilt_component
)

install(
  TARGETS joy_pan_tilt_component
  EXPORT export_joy_pan_tilt_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
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

ament_export_include_directories(
  "include/${PROJECT_NAME}"
)

ament_package()
