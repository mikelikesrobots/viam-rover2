cmake_minimum_required(VERSION 3.8)
project(viam_rover2 LANGUAGES CXX)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(THIS_PACKAGE_INCLUDE_DEPENDS
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  sensor_msgs
  std_msgs
)

list(APPEND THIS_PACKAGE_INCLUDE_DEPENDS
  behaviortree_cpp
  action_msgs
  diagnostic_msgs
  geometry_msgs
  rclcpp_action
  nav2_msgs
  nav_msgs
  nav2_behavior_tree
  tf2
  tf2_geometry_msgs
  tf2_ros
  visualization_msgs
)

find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

add_library(${PROJECT_NAME}
  SHARED
  hardware/src/rover2_system.cpp
  hardware/src/rp2040_bridge.cpp
  hardware/src/rp2040_sensor_node.cpp
)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_20)
target_include_directories(${PROJECT_NAME} PUBLIC
$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/hardware/include>
$<INSTALL_INTERFACE:include/viam_rover2>
)
ament_target_dependencies(
  ${PROJECT_NAME} PUBLIC
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(${PROJECT_NAME} PRIVATE "VIAM_ROVER2_BUILDING_DLL")

pluginlib_export_plugin_description_file(hardware_interface viam_rover2.xml)

install(
  DIRECTORY hardware/include/
  DESTINATION include/viam_rover2
)
install(
  DIRECTORY description/ros2_control description/urdf
  DESTINATION share/viam_rover2
)
install(
  DIRECTORY bringup/launch bringup/config
  DESTINATION share/viam_rover2
)

install(TARGETS
  ${PROJECT_NAME}
  EXPORT export_viam_rover2
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# ── RP2040 firmware (cross-compiled, external process) ──────────────────────

option(BUILD_FIRMWARE "Build RP2040 firmware during colcon build" ON)

if(BUILD_FIRMWARE)
  set(FW_SOURCE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/firmware/rp2040")
  set(FW_BUILD_DIR  "${CMAKE_CURRENT_BINARY_DIR}/firmware_build")
  set(FW_UF2        "${FW_BUILD_DIR}/rover2_rp2040.uf2")

  set(FW_SOURCES
    "${FW_SOURCE_DIR}/src/main.c"
    "${FW_SOURCE_DIR}/src/motor.c"
    "${FW_SOURCE_DIR}/src/motor.h"
    "${FW_SOURCE_DIR}/src/encoder.c"
    "${FW_SOURCE_DIR}/src/encoder.h"
    "${FW_SOURCE_DIR}/src/protocol.c"
    "${FW_SOURCE_DIR}/src/protocol.h"
    "${FW_SOURCE_DIR}/src/telemetry.c"
    "${FW_SOURCE_DIR}/src/telemetry.h"
    "${FW_SOURCE_DIR}/src/i2c_sensors.c"
    "${FW_SOURCE_DIR}/src/i2c_sensors.h"
    "${FW_SOURCE_DIR}/CMakeLists.txt"
    "${FW_SOURCE_DIR}/pico_sdk_import.cmake"
  )

  add_custom_command(
    OUTPUT  "${FW_UF2}"
    COMMAND ${CMAKE_COMMAND} -E make_directory "${FW_BUILD_DIR}"
    COMMAND ${CMAKE_COMMAND}
            -S "${FW_SOURCE_DIR}"
            -B "${FW_BUILD_DIR}"
            -DCMAKE_BUILD_TYPE=Release
    COMMAND ${CMAKE_COMMAND} --build "${FW_BUILD_DIR}" --parallel
    DEPENDS ${FW_SOURCES}
    COMMENT "Building RP2040 firmware"
    VERBATIM
  )

  add_custom_command(
    OUTPUT  "${FW_BUILD_DIR}/flash.stamp"
    COMMAND "${FW_SOURCE_DIR}/flash.sh" "${FW_UF2}"
    COMMAND ${CMAKE_COMMAND} -E touch "${FW_BUILD_DIR}/flash.stamp"
    DEPENDS "${FW_UF2}"
    COMMENT "Flashing RP2040 firmware"
    VERBATIM
  )

  add_custom_target(firmware ALL DEPENDS "${FW_BUILD_DIR}/flash.stamp")

  install(
    FILES "${FW_UF2}"
    DESTINATION share/${PROJECT_NAME}/firmware
  )

  install(
    PROGRAMS
      "${FW_SOURCE_DIR}/flash.sh"
    DESTINATION share/${PROJECT_NAME}/firmware
  )
endif()

if(BUILD_TESTING)
  find_package(ament_cmake_gmock REQUIRED)
  find_package(ros2_control_test_assets REQUIRED)
  ament_add_gmock(test_rover2_system hardware/test/test_rover2_system.cpp)
  target_include_directories(test_rover2_system PRIVATE include)
  target_link_libraries(test_rover2_system ${PROJECT_NAME})
  ament_target_dependencies(test_rover2_system
    pluginlib
    ros2_control_test_assets
  )
endif()

ament_export_targets(export_viam_rover2 HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})

ament_package()
