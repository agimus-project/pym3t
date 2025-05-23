cmake_minimum_required(VERSION 3.15)
project(pym3t LANGUAGES CXX)

option(USE_AZURE_KINECT "Use Azure Kinect" OFF)
option(USE_REALSENSE "Use RealSense D435" OFF)
option(USE_GTEST "Use gtest" OFF)
option(USE_XFEATURES2D "Use OpenCV xfeatures2d" OFF)

if(CMAKE_VERSION VERSION_GREATER_EQUAL "3.27.0")
  cmake_policy(SET CMP0148 OLD) # required for current pybind11
endif()

# Fix this error:
# /usr/bin/ld: libm3t_ext.a(dummy_camera.cpp.o): relocation R_X86_64_PC32 against symbol
# `_ZSt4cerr@@GLIBCXX_3.4' can not be used when making a shared object; recompile with -fPIC
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

IF(USE_REALSENSE)
  ADD_DEFINITIONS(-DPYM3T_WITH_REALSENSE)
  LIST(APPEND CFLAGS_DEPENDENCIES "-DPYM3T_WITH_REALSENSE")
ENDIF(USE_REALSENSE)

include(FetchContent)
FetchContent_Declare(
  pybind11
  GIT_REPOSITORY "https://github.com/pybind/pybind11"
  GIT_TAG "v2.11.1")
FetchContent_Declare(
  m3t
  GIT_REPOSITORY "https://github.com/agimus-project/3DObjectTracking"
  GIT_TAG "master"
  SOURCE_SUBDIR "M3T" CMAKE_ARGS "-DUSE_AZURE_KINECT=${USE_AZURE_KINECT}"
  "-DUSE_REALSENSE=${USE_REALSENSE}" "-DUSE_GTEST=${USE_GTEST}" "-DUSE_XFEATURES2D=${USE_XFEATURES2D}")
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
