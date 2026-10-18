cmake_minimum_required(VERSION 3.20)
project(MotionControlAlgorithms VERSION 0.1.0 LANGUAGES CXX)

option(MCA_BUILD_TESTS "Build unit tests" ON)
option(MCA_BUILD_BENCHMARKS "Build benchmarks" ON)
option(MCA_ENABLE_WARNINGS "Enable strict compiler warnings" ON)
set(MCA_EIGEN3_DIR "" CACHE PATH "Optional Eigen3 CMake package directory")

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(MCA_EIGEN3_DIR)
  list(PREPEND CMAKE_PREFIX_PATH "${MCA_EIGEN3_DIR}")
endif()
find_package(Eigen3 CONFIG REQUIRED)

add_library(mca_core INTERFACE)
target_include_directories(mca_core INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(mca_core INTERFACE Eigen3::Eigen)
target_compile_features(mca_core INTERFACE cxx_std_17)

add_library(mca_benchmark STATIC src/benchmark/metrics.cpp)
target_link_libraries(mca_benchmark PUBLIC mca_core)
target_compile_features(mca_benchmark PUBLIC cxx_std_17)

if(MCA_ENABLE_WARNINGS)
  if(MSVC)
    target_compile_options(mca_core INTERFACE /W4 /permissive-)
    target_compile_options(mca_benchmark PRIVATE /W4 /permissive-)
  else()
    target_compile_options(mca_core INTERFACE -Wall -Wextra -Wpedantic -Wconversion -Wshadow)
    target_compile_options(mca_benchmark PRIVATE -Wall -Wextra -Wpedantic -Wconversion -Wshadow)
  endif()
endif()

if(MCA_BUILD_TESTS)
  enable_testing()
  find_package(GTest REQUIRED)
  add_executable(mca_core_tests
    tests/core/math_test.cpp
    tests/core/types_test.cpp
    tests/benchmark/metrics_test.cpp
    tests/trajectory/scalar_trajectory_test.cpp
    tests/plant/first_order_test.cpp
    tests/plant/second_order_test.cpp
    tests/control/pid_test.cpp
    tests/control/feedforward_test.cpp
    tests/control/cascaded_servo_test.cpp
    tests/plant/dc_motor_test.cpp
    tests/plant/bldc_pmsm_test.cpp
    tests/experiment/single_axis_test.cpp
    tests/experiment/cascaded_servo_test.cpp
  )
  target_link_libraries(mca_core_tests PRIVATE mca_benchmark GTest::gtest_main)
  include(GoogleTest)
  gtest_discover_tests(mca_core_tests)
endif()

if(MCA_BUILD_BENCHMARKS)
  find_package(benchmark QUIET)
  if(benchmark_FOUND)
    add_executable(mca_core_benchmark benchmarks/core_math_benchmark.cpp)
    target_link_libraries(mca_core_benchmark PRIVATE mca_core benchmark::benchmark)
  endif()
endif()

install(DIRECTORY include/ DESTINATION include)
