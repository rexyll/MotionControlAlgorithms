# Install script for directory: F:/workspace/_coding_workspace/eigen-3.3.9/Eigen

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
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/eigen3/Eigen" TYPE FILE FILES
    "F:/workspace/_coding_workspace/eigen-3.3.9/Eigen/Cholesky"
    "F:/workspace/_coding_workspace/eigen-3.3.9/Eigen/CholmodSupport"
    "F:/workspace/_coding_workspace/eigen-3.3.9/Eigen/Core"
    "F:/workspace/_coding_workspace/eigen-3.3.9/Eigen/Dense"
    "F:/workspace/_coding_workspace/eigen-3.3.9/Eigen/Eigen"
    "F:/workspace/_coding_workspace/eigen-3.3.9/Eigen/Eigenvalues"
    "F:/workspace/_coding_workspace/eigen-3.3.9/Eigen/Geometry"
    "F:/workspace/_coding_workspace/eigen-3.3.9/Eigen/Householder"
    "F:/workspace/_coding_workspace/eigen-3.3.9/Eigen/IterativeLinearSolvers"
    "F:/workspace/_coding_workspace/eigen-3.3.9/Eigen/Jacobi"
    "F:/workspace/_coding_workspace/eigen-3.3.9/Eigen/LU"
    "F:/workspace/_coding_workspace/eigen-3.3.9/Eigen/MetisSupport"
    "F:/workspace/_coding_workspace/eigen-3.3.9/Eigen/OrderingMethods"
    "F:/workspace/_coding_workspace/eigen-3.3.9/Eigen/PaStiXSupport"
    "F:/workspace/_coding_workspace/eigen-3.3.9/Eigen/PardisoSupport"
    "F:/workspace/_coding_workspace/eigen-3.3.9/Eigen/QR"
    "F:/workspace/_coding_workspace/eigen-3.3.9/Eigen/QtAlignedMalloc"
    "F:/workspace/_coding_workspace/eigen-3.3.9/Eigen/SPQRSupport"
    "F:/workspace/_coding_workspace/eigen-3.3.9/Eigen/SVD"
    "F:/workspace/_coding_workspace/eigen-3.3.9/Eigen/Sparse"
    "F:/workspace/_coding_workspace/eigen-3.3.9/Eigen/SparseCholesky"
    "F:/workspace/_coding_workspace/eigen-3.3.9/Eigen/SparseCore"
    "F:/workspace/_coding_workspace/eigen-3.3.9/Eigen/SparseLU"
    "F:/workspace/_coding_workspace/eigen-3.3.9/Eigen/SparseQR"
    "F:/workspace/_coding_workspace/eigen-3.3.9/Eigen/StdDeque"
    "F:/workspace/_coding_workspace/eigen-3.3.9/Eigen/StdList"
    "F:/workspace/_coding_workspace/eigen-3.3.9/Eigen/StdVector"
    "F:/workspace/_coding_workspace/eigen-3.3.9/Eigen/SuperLUSupport"
    "F:/workspace/_coding_workspace/eigen-3.3.9/Eigen/UmfPackSupport"
    )
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Devel" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/eigen3/Eigen" TYPE DIRECTORY FILES "F:/workspace/_coding_workspace/eigen-3.3.9/Eigen/src" FILES_MATCHING REGEX "/[^/]*\\.h$")
endif()

string(REPLACE ";" "\n" CMAKE_INSTALL_MANIFEST_CONTENT
       "${CMAKE_INSTALL_MANIFEST_FILES}")
if(CMAKE_INSTALL_LOCAL_ONLY)
  file(WRITE "F:/workspace/_coding_workspace/MotionControlAlgorithms - 副本/deps/build/eigen/Eigen/install_local_manifest.txt"
     "${CMAKE_INSTALL_MANIFEST_CONTENT}")
endif()
