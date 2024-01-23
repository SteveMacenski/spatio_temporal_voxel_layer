cmake_minimum_required(VERSION 3.18)
project(openvdb_vendor)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_vendor_package REQUIRED)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${PROJECT_SOURCE_DIR}/cmake/")

# Find OpenVDB . If not found on the system, will compile from source
option(USE_SYSTEM_OPENVDB "Use system pre-installed OpenVDB" ON)
include(cmake/find_dependencies.cmake)

ament_export_include_directories(include)

ament_package()
