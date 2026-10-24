cmake_minimum_required(VERSION 3.20)
project(MotionControlAlgorithmsP846 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(Eigen3 REQUIRED)
find_package(GTest REQUIRED)

add_executable(mca_p8_46_nonlinear_mpc_test nonlinear_mpc_boundary_evaluation_test.cpp)
target_include_directories(mca_p8_46_nonlinear_mpc_test PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/../../include)
target_link_libraries(mca_p8_46_nonlinear_mpc_test PRIVATE Eigen3::Eigen GTest::gtest_main)

enable_testing()
include(GoogleTest)
gtest_discover_tests(mca_p8_46_nonlinear_mpc_test)
