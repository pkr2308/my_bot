cmake_minimum_required(VERSION 3.8)
project(new_bot)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(controller_manager REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# Find serial library
find_package(PkgConfig REQUIRED)
pkg_check_modules(SERIAL REQUIRED libserial)

# Create hardware interface library
add_library(
  new_bot_hardware_interface
  SHARED
  src/rp2040_hardware_interface.cpp
)

target_compile_features(new_bot_hardware_interface PUBLIC c_std_99 cxx_std_17)

target_include_directories(new_bot_hardware_interface PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${SERIAL_INCLUDE_DIRS}
)

ament_target_dependencies(
  new_bot_hardware_interface PUBLIC
  controller_manager
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  sensor_msgs
  geometry_msgs
)

# Link serial library
target_link_libraries(new_bot_hardware_interface PUBLIC ${SERIAL_LIBRARIES})

# Export plugin
pluginlib_export_plugin_description_file(hardware_interface new_bot_hardware_interface.xml)

# Install targets
install(
  TARGETS new_bot_hardware_interface
  EXPORT new_bot_hardware_interface
  DESTINATION lib
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  DIRECTORY config description launch scripts
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_targets(new_bot_hardware_interface HAS_LIBRARY_TARGET)
ament_export_dependencies(
  controller_manager
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  sensor_msgs
  geometry_msgs
)

ament_package()
