cmake_minimum_required(VERSION 3.8)
project(bno086_hardware_interface)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)  ##    -Wpedantic
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_srvs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(controller_interface REQUIRED)

pluginlib_export_plugin_description_file(hardware_interface bno086_hardware_interface.xml)


# Include directories
include_directories(
  include/bno086_hardware_interface
  )


add_library(${PROJECT_NAME} 
            src/Bno086HardwareInterface.cpp 
            src/elapsedMillis.cpp
            src/vqwPipe_Driver_Linux.cpp
            src/vqwPipe.cpp
            src/CRC8.c
           )
## add_library(bno086_hardware_interface::Bno086HardwareInterface ALIAS Bno086HardwareInterface)

target_compile_features(${PROJECT_NAME} PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17


target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)

ament_target_dependencies(
  ${PROJECT_NAME}
  "rclcpp"
  "std_srvs"
  "sensor_msgs"
  hardware_interface
  controller_interface
  pluginlib
)

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(${PROJECT_NAME} PRIVATE "BNO086_HARDWARE_INTERFACE_BUILDING_LIBRARY")

# # install(
# #   DIRECTORY include/
# #   DESTINATION include/${PROJECT_NAME}
# # )

install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install the plugin description xml file
install(FILES
  bno086_hardware_interface.xml
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

# # ament_export_include_directories(
# #   "include/${PROJECT_NAME}"
# # )
ament_export_libraries(
  Bno086HardwareInterface
)
ament_export_targets(
  export_${PROJECT_NAME}
)

ament_package()
