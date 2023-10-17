cmake_minimum_required(VERSION 3.10)
project(pym3t LANGUAGES CXX)

find_package(pybind11 CONFIG REQUIRED)

# TODO: find a way to pass these options as command line arguments
set(USE_AZURE_KINECT OFF CACHE BOOL "Use Azure Kinect")
set(USE_REALSENSE ON CACHE BOOL "Use RealSense D435")
set(USE_GTEST OFF CACHE BOOL "Use gtest")

set(CMAKE_BUILD_TYPE "RELEASE")
# set(CMAKE_BUILD_TYPE "DEBUG")

set(CMAKE_CXX_STANDARD 17)

# We need libraries to be generated with Position Independent Code
# otherwise compilation error in the pybind11 code 
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC")

add_subdirectory(extern/3DObjectTracking/M3T)

# Create library for the extensions to m3t
add_library(m3t_ext src/dummy_camera.cpp)
target_include_directories(m3t_ext PUBLIC include)
target_link_libraries(m3t_ext PUBLIC m3t)

pybind11_add_module(_pym3t_mod MODULE src/pym3t.cpp)
target_link_libraries(_pym3t_mod PUBLIC m3t)
target_link_libraries(_pym3t_mod PUBLIC m3t_ext)
target_compile_features(_pym3t_mod PUBLIC cxx_std_17)
target_include_directories(_pym3t_mod PUBLIC include)

install(TARGETS _pym3t_mod DESTINATION pym3t)