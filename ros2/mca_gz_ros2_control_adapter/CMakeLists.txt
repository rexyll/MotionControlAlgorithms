cmake_minimum_required(VERSION 3.16)
project(mca_gz_ros2_control_adapter LANGUAGES CXX)

find_package(ament_cmake REQUIRED)
find_package(gz_ros2_control REQUIRED)
# Gazebo Harmonic ships the versioned CMake package/target names.
# The CI image installs libgz-sim8-dev, so use the matching gz-sim8 package.
find_package(gz-sim8 REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)

add_library(mca_gz_ros2_control_adapter SHARED src/gazebo_system.cpp)
target_compile_features(mca_gz_ros2_control_adapter PUBLIC cxx_std_17)
target_compile_options(mca_gz_ros2_control_adapter PRIVATE
  -Wall -Wextra -Wpedantic -Werror=deprecated-declarations)
target_include_directories(mca_gz_ros2_control_adapter PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../../include>
  $<INSTALL_INTERFACE:include>)
# ament_target_dependencies() uses the plain target_link_libraries() signature,
# so keep the explicit Gazebo link on the same signature.
target_link_libraries(mca_gz_ros2_control_adapter gz-sim8::gz-sim8)
ament_target_dependencies(mca_gz_ros2_control_adapter
  gz_ros2_control hardware_interface pluginlib rclcpp rclcpp_lifecycle)

pluginlib_export_plugin_description_file(
  gz_ros2_control mca_gz_ros2_control_adapter.xml)

install(DIRECTORY include/ DESTINATION include)
install(TARGETS mca_gz_ros2_control_adapter
  EXPORT export_mca_gz_ros2_control_adapter
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
ament_export_targets(export_mca_gz_ros2_control_adapter HAS_LIBRARY_TARGET)
ament_export_dependencies(gz_ros2_control hardware_interface pluginlib)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_gazebo_system_plugin test/test_gazebo_system_plugin.cpp)
  ament_target_dependencies(test_gazebo_system_plugin
    gz_ros2_control hardware_interface pluginlib rclcpp_lifecycle)
endif()

ament_package()
