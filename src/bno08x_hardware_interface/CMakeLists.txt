cmake_minimum_required(VERSION 3.8)
project(bno08x_hardware_interface)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

find_package(pluginlib REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)


add_library(${PROJECT_NAME} SHARED
      src/Bno08xHardwareInterface.cpp
      src/elapsedMillis.cpp
      src/vqwPipe.cpp
      src/vqwPipe_Channel.cpp
      src/vqwPipe_Driver_Linux.cpp
      src/CRC8.c
      )
  
add_library(bno08x_hardware_interface::Bno08xHardwareInterface ALIAS ${PROJECT_NAME})



add_executable(bno08x_Test 
        src/bno08x_Test.cpp 
        src/CRC8.c
        src/elapsedMillis.cpp
        src/vqwPipe.cpp
        src/vqwPipe_Channel.cpp
        src/vqwPipe_Driver_Linux.cpp)

target_link_libraries(bno08x_Test PUBLIC /usr/lib/aarch64-linux-gnu/libserial.so)    

install(TARGETS bno08x_Test 
        RUNTIME DESTINATION bin
        )
ament_target_dependencies(
          bno08x_Test PUBLIC
          rclcpp
        )


ament_target_dependencies( ${PROJECT_NAME} 
      hardware_interface 
      rclcpp 
      rclcpp_lifecycle 
      pluginlib)

# Include directories
include_directories(
  include
  include/bno08x_hardware_interface
  )


target_compile_features(${PROJECT_NAME} PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(${PROJECT_NAME} PRIVATE "BNO08X_HARDWARE_INTERFACE_BUILDING_LIBRARY")

pluginlib_export_plugin_description_file(hardware_interface bno08x_hardware_interface.xml)

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

if(BUILD_TESTING)
find_package(ament_lint_auto REQUIRED)
find_package(ament_cmake_gmock REQUIRED)
find_package(hardware_interface REQUIRED)
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
