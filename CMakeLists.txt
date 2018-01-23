cmake_minimum_required(VERSION 2.8.3)
project(spatio_temporal_voxel_layer)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake/")

find_package(catkin REQUIRED COMPONENTS
  costmap_2d
  geometry_msgs
  pluginlib
  sensor_msgs
  std_msgs
  dynamic_reconfigure
  laser_geometry
  message_filters
  pcl_conversions
  pcl_ros
  roscpp
  tf
  visualization_msgs
)

find_package(PCL REQUIRED)
find_package(OpenVDB REQUIRED)
find_package(TBB REQUIRED)
remove_definitions(-DDISABLE_LIBUSB-1.0)
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED COMPONENTS system thread)
include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${OpenVDB_INCLUDE_DIR}
    ${TBB_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
)

#add_definitions(${EIGEN3_DEFINITIONS})

generate_dynamic_reconfigure_options(
  cfg/SpatioTemporalVoxelLayerPlugin.cfg
)

catkin_package(
    INCLUDE_DIRS
        include
        ${EIGEN3_INCLUDE_DIRS}
        ${PCL_INCLUDE_DIRS}
        ${OpenVDB_INCLUDE_DIR}
        ${TBB_INCLUDE_DIRS}
    LIBRARIES spatio_temporal_voxel_layer
    CATKIN_DEPENDS
        dynamic_reconfigure
        geometry_msgs
        laser_geometry
        message_filters
        pcl_ros
        pluginlib
        roscpp
        sensor_msgs
        std_msgs
        costmap_2d
        tf
        visualization_msgs
    DEPENDS
        TBB
        OpenVDB
        PCL
        EIGEN3
        Boost
)

add_library(spatio_temporal_voxel_layer
            src/spatio_temporal_voxel_layer.cpp
            src/level_set.cpp
)

target_link_libraries(spatio_temporal_voxel_layer
  ${PCL_LIBRARIES}
  ${OpenVDB_LIBRARIES}
  ${TBB_LIBRARIES}
  ${Boost_LIBRARIES}
  ${catkin_LIBRARIES}
)

install(TARGETS
    spatio_temporal_voxel_layer
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES costmap_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
