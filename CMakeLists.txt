cmake_minimum_required(VERSION 3.11)
project(pym3t LANGUAGES CXX)

option(USE_AZURE_KINECT "Use Azure Kinect" OFF)
option(USE_REALSENSE "Use RealSense D435" ON)
option(USE_GTEST "Use gtest" OFF)

cmake_policy(SET CMP0148 OLD) # required for current pybind11

# set(CMAKE_BUILD_TYPE "RELEASE") set(CMAKE_BUILD_TYPE "DEBUG")

# set(CMAKE_CXX_STANDARD 17)

# We need libraries to be generated with Position Independent Code otherwise
# compilation error in the pybind11 code set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}
# -fPIC")

# add_subdirectory(extern/3DObjectTracking/M3T)

include(FetchContent)
FetchContent_Declare(
  pybind11
  GIT_REPOSITORY "https://github.com/pybind/pybind11"
  GIT_TAG "v2.11.1")
FetchContent_Declare(
  m3t
  GIT_REPOSITORY "https://github.com/DLR-RM/3DObjectTracking.git"
  GIT_TAG "master"
  CMAKE_ARGS "-DUSE_AZURE_KINECT=${USE_AZURE_KINECT}"
  "-DUSE_REALSENSE=${USE_REALSENSE}" "-DUSE_GTEST=${USE_GTEST}" SOURCE_SUBDIR
  "M3T")
FetchContent_MakeAvailable(pybind11 m3t)

# Create library for the extensions to m3t
add_library(m3t_ext src/dummy_camera.cpp include/pym3t/dummy_camera.h)
target_compile_features(m3t_ext PUBLIC cxx_std_17)
target_include_directories(
  m3t_ext
  PUBLIC $<INSTALL_INTERFACE:include>
         $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include>
         $<BUILD_INTERFACE:${m3t_SOURCE_DIR}/include>)
target_link_libraries(m3t_ext PUBLIC m3t)

pybind11_add_module(_pym3t_mod MODULE src/pym3t.cpp)
target_link_libraries(_pym3t_mod PUBLIC m3t_ext)
target_include_directories(
  _pym3t_mod PUBLIC $<INSTALL_INTERFACE:include>
                    $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include>)

install(TARGETS _pym3t_mod DESTINATION pym3t)
