cmake_minimum_required(VERSION 3.16)
project(mca_gazebo_adapter LANGUAGES CXX)

find_package(gz-sim8 REQUIRED)
find_package(gz-plugin2 REQUIRED COMPONENTS register)
find_package(gz-common5 REQUIRED)
find_package(gz-math7 REQUIRED)
find_package(gz-transport13 REQUIRED)
find_package(gz-msgs10 REQUIRED)

add_executable(mca_transport_subscriber_probe test/transport_subscriber_probe.cpp)
target_compile_features(mca_transport_subscriber_probe PRIVATE cxx_std_17)
target_link_libraries(mca_transport_subscriber_probe PRIVATE gz-transport13::gz-transport13 gz-msgs10::gz-msgs10)

add_library(mca_gazebo_adapter SHARED src/scalar_joint_system.cpp)
target_compile_features(mca_gazebo_adapter PRIVATE cxx_std_17)
target_include_directories(mca_gazebo_adapter PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/../../include)
target_link_libraries(mca_gazebo_adapter PRIVATE
  gz-sim8::gz-sim8
  gz-plugin2::register
  gz-common5::gz-common5
  gz-math7::gz-math7
  gz-transport13::gz-transport13
  gz-msgs10::gz-msgs10)
set_target_properties(mca_gazebo_adapter PROPERTIES
  CXX_VISIBILITY_PRESET hidden
  VISIBILITY_INLINES_HIDDEN ON)
install(TARGETS mca_gazebo_adapter
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
