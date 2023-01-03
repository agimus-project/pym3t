cmake_minimum_required(VERSION 3.10)
project(pybind11_minimal_project LANGUAGES CXX)

find_package(pybind11 CONFIG REQUIRED)

set(USE_AZURE_KINECT OFF CACHE BOOL "Use Azure Kinect")
set(USE_REALSENSE ON CACHE BOOL "Use RealSense D435")

set(CMAKE_CXX_STANDARD 17)

add_subdirectory(extern/3DObjectTracking/ICG)


pybind11_add_module(_pyicg_mod MODULE src/pyicg.cpp)

target_link_libraries(_pyicg_mod PUBLIC icg)
target_compile_features(_pyicg_mod PUBLIC cxx_std_17)
target_include_directories(_pyicg_mod PUBLIC include)

install(TARGETS _pyicg_mod DESTINATION pyicg)