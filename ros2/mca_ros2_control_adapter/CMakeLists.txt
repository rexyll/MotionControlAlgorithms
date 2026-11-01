
cmake_minimum_required(VERSION 3.16)
project(mca_ros2_control_adapter LANGUAGES CXX)

find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)

add_library(mca_ros2_control_adapter SHARED
  src/actuator_hardware.cpp
  src/mock_actuator.cpp
  src/system_hardware.cpp
  src/mock_system.cpp)
target_compile_features(mca_ros2_control_adapter PUBLIC cxx_std_17)
target_compile_options(mca_ros2_control_adapter PRIVATE
  -Werror=deprecated-declarations)
target_include_directories(mca_ros2_control_adapter PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../../include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(mca_ros2_control_adapter
  hardware_interface pluginlib rclcpp rclcpp_lifecycle)

pluginlib_export_plugin_description_file(
  hardware_interface mca_ros2_control_adapter.xml)

install(DIRECTORY include/ DESTINATION include)
install(TARGETS mca_ros2_control_adapter
  EXPORT export_mca_ros2_control_adapter
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
ament_export_targets(export_mca_ros2_control_adapter HAS_LIBRARY_TARGET)
ament_export_dependencies(
  hardware_interface pluginlib rclcpp rclcpp_lifecycle)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_mock_actuator_plugin test/test_mock_actuator_plugin.cpp)
  ament_target_dependencies(test_mock_actuator_plugin
    hardware_interface pluginlib rclcpp_lifecycle)
  ament_add_gtest(test_mock_system_plugin test/test_mock_system_plugin.cpp)
  ament_target_dependencies(test_mock_system_plugin
    hardware_interface pluginlib rclcpp_lifecycle)
endif()

ament_package()
