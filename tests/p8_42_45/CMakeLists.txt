cmake_minimum_required(VERSION 3.20)
project(P8_42_45_NonlinearMpcRepresentabilityTests LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(Eigen3 REQUIRED)
find_package(GTest REQUIRED)

add_executable(p8_42_45_nonlinear_mpc_regression_tests nonlinear_mpc_representability_test.cpp)
target_include_directories(p8_42_45_nonlinear_mpc_regression_tests PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/../../include)
target_link_libraries(p8_42_45_nonlinear_mpc_regression_tests PRIVATE Eigen3::Eigen GTest::gtest_main)

enable_testing()
include(GoogleTest)
gtest_discover_tests(p8_42_45_nonlinear_mpc_regression_tests)
