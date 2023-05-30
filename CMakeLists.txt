cmake_minimum_required(VERSION 3.10)
project(pycig LANGUAGES CXX)

find_package(pybind11 CONFIG REQUIRED)

# TODO: find a way to pass these options as command line arguments
set(USE_AZURE_KINECT OFF CACHE BOOL "Use Azure Kinect")
set(USE_REALSENSE ON CACHE BOOL "Use RealSense D435")
set(CMAKE_BUILD_TYPE OFF CACHE BOOL "")

set(CMAKE_CXX_STANDARD 17)

# We need libraries to be generated with Position Independent Code
# otherwise compilation error in the pybind11 code 
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC")

add_subdirectory(extern/3DObjectTracking/ICG)

# Create library for the extensions to icg
add_library(icg_ext src/dummy_camera.cpp)
target_include_directories(icg_ext PUBLIC include)
target_link_libraries(icg_ext PUBLIC icg)

pybind11_add_module(_pyicg_mod MODULE src/pyicg.cpp)
target_link_libraries(_pyicg_mod PUBLIC icg)
target_link_libraries(_pyicg_mod PUBLIC icg_ext)
target_compile_features(_pyicg_mod PUBLIC cxx_std_17)
target_include_directories(_pyicg_mod PUBLIC include)

install(TARGETS _pyicg_mod DESTINATION pyicg)