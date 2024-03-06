cmake_minimum_required(VERSION 3.18)
project(openvdb_vendor)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_vendor_package REQUIRED)

ament_vendor(${PROJECT_NAME}
  VCS_URL https://github.com/AcademySoftwareFoundation/openvdb.git
  VCS_VERSION v10.0.1
  VCS_TYPE git
  CMAKE_ARGS
    -DCONCURRENT_MALLOC=Tbbmalloc
    -DOPENVDB_BUILD_BINARIES=OFF
)

# When this package is found, make sure to find the real OpenVDB
ament_package(
  CONFIG_EXTRAS "openvdb_vendor-extras.cmake"
)
