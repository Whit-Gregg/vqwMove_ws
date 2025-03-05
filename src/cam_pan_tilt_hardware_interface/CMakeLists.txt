cmake_minimum_required(VERSION 3.8)
project(cam_pan_tilt_hardware_interface)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(backward_ros REQUIRED)

pluginlib_export_plugin_description_file(hardware_interface cam_pan_tilt_hardware_interface.xml)

add_library(${PROJECT_NAME} 
    src/cam_pan_tilt_hardware_interface.cpp
    src/i2c_controller.cpp
    src/pwm_servo_driver.cpp
)
add_library(cam_pan_tilt_hardware_interface::cam_pan_tilt_hardware_interface ALIAS cam_pan_tilt_hardware_interface)

target_compile_features(${PROJECT_NAME} PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)

ament_target_dependencies(
  ${PROJECT_NAME}
  "rclcpp"
  "pluginlib"
  "hardware_interface"
  ##"position_controllers"
)

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(${PROJECT_NAME} PRIVATE "CAM_PAN_TILT_HARDWARE_INTERFACE_BUILDING_LIBRARY")

# Install the plugin description xml file
install(FILES
  cam_pan_tilt_hardware_interface.xml
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)
install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)



##~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
add_executable(Test_PWMServoDriver 
    src/Test_PWMServoDriver.cpp
    src/i2c_controller.cpp
    src/pwm_servo_driver.cpp
    )

target_compile_features(Test_PWMServoDriver PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

target_include_directories(Test_PWMServoDriver PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)

ament_target_dependencies(Test_PWMServoDriver
  "rclcpp"
  "backward_ros"
)

install(
  TARGETS Test_PWMServoDriver
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
##~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


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
ament_export_libraries(
  ${PROJECT_NAME}
)
ament_export_targets(
  export_${PROJECT_NAME}
)

ament_package()
