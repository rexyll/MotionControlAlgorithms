# Install script for directory: F:/workspace/_coding_workspace/eigen-3.3.9/unsupported/Eigen

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "F:/workspace/_coding_workspace/MotionControlAlgorithms - 副本/deps/install")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "Release")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Devel" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/eigen3/unsupported/Eigen" TYPE FILE FILES
    "F:/workspace/_coding_workspace/eigen-3.3.9/unsupported/Eigen/AdolcForward"
    "F:/workspace/_coding_workspace/eigen-3.3.9/unsupported/Eigen/AlignedVector3"
    "F:/workspace/_coding_workspace/eigen-3.3.9/unsupported/Eigen/ArpackSupport"
    "F:/workspace/_coding_workspace/eigen-3.3.9/unsupported/Eigen/AutoDiff"
    "F:/workspace/_coding_workspace/eigen-3.3.9/unsupported/Eigen/BVH"
    "F:/workspace/_coding_workspace/eigen-3.3.9/unsupported/Eigen/EulerAngles"
    "F:/workspace/_coding_workspace/eigen-3.3.9/unsupported/Eigen/FFT"
    "F:/workspace/_coding_workspace/eigen-3.3.9/unsupported/Eigen/IterativeSolvers"
    "F:/workspace/_coding_workspace/eigen-3.3.9/unsupported/Eigen/KroneckerProduct"
    "F:/workspace/_coding_workspace/eigen-3.3.9/unsupported/Eigen/LevenbergMarquardt"
    "F:/workspace/_coding_workspace/eigen-3.3.9/unsupported/Eigen/MatrixFunctions"
    "F:/workspace/_coding_workspace/eigen-3.3.9/unsupported/Eigen/MoreVectorization"
    "F:/workspace/_coding_workspace/eigen-3.3.9/unsupported/Eigen/MPRealSupport"
    "F:/workspace/_coding_workspace/eigen-3.3.9/unsupported/Eigen/NonLinearOptimization"
    "F:/workspace/_coding_workspace/eigen-3.3.9/unsupported/Eigen/NumericalDiff"
    "F:/workspace/_coding_workspace/eigen-3.3.9/unsupported/Eigen/OpenGLSupport"
    "F:/workspace/_coding_workspace/eigen-3.3.9/unsupported/Eigen/Polynomials"
    "F:/workspace/_coding_workspace/eigen-3.3.9/unsupported/Eigen/Skyline"
    "F:/workspace/_coding_workspace/eigen-3.3.9/unsupported/Eigen/SparseExtra"
    "F:/workspace/_coding_workspace/eigen-3.3.9/unsupported/Eigen/SpecialFunctions"
    "F:/workspace/_coding_workspace/eigen-3.3.9/unsupported/Eigen/Splines"
    )
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Devel" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/eigen3/unsupported/Eigen" TYPE DIRECTORY FILES "F:/workspace/_coding_workspace/eigen-3.3.9/unsupported/Eigen/src" FILES_MATCHING REGEX "/[^/]*\\.h$")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for each subdirectory.
  include("F:/workspace/_coding_workspace/MotionControlAlgorithms - 副本/deps/build/eigen/unsupported/Eigen/CXX11/cmake_install.cmake")

endif()

string(REPLACE ";" "\n" CMAKE_INSTALL_MANIFEST_CONTENT
       "${CMAKE_INSTALL_MANIFEST_FILES}")
if(CMAKE_INSTALL_LOCAL_ONLY)
  file(WRITE "F:/workspace/_coding_workspace/MotionControlAlgorithms - 副本/deps/build/eigen/unsupported/Eigen/install_local_manifest.txt"
     "${CMAKE_INSTALL_MANIFEST_CONTENT}")
endif()
