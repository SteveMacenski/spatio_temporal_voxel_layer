cmake_minimum_required(VERSION 2.8.3)
project(spatio_temporal_voxel_layer)

add_definitions (-std=c++17)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${PROJECT_SOURCE_DIR}/cmake/")

find_package(catkin REQUIRED COMPONENTS
  costmap_2d
  geometry_msgs
  pluginlib
  sensor_msgs
  std_msgs
  laser_geometry
  message_filters
  pcl_conversions
  pcl_ros
  roscpp
  tf2_ros
  tf2_geometry_msgs
  visualization_msgs
  message_generation
  dynamic_reconfigure
)

add_service_files(DIRECTORY msgs
  FILES
  SaveGrid.srv
)

generate_dynamic_reconfigure_options(
  cfg/SpatioTemporalVoxelLayer.cfg
)

generate_messages(
   DEPENDENCIES
   std_msgs
   dynamic_reconfigure
)

find_package(PCL REQUIRED)
if(NOT "${PCL_LIBRARIES}" STREQUAL "")
  # This package fails to build on Debian Stretch with a linking error against
  # 'Qt5::Widgets'.  This is a transitive dependency that comes in to PCL via
  # the PCL dependency on VTK.  However, we don't actually care about the Qt
  # dependencies for this package, so just remove them.  This is similar to the
  # workaround in https://github.com/ros-perception/perception_pcl/pull/151,
  # and can be removed when Stretch goes out of support.
  list(REMOVE_ITEM PCL_LIBRARIES
    "vtkGUISupportQt"
    "vtkGUISupportQtOpenGL"
    "vtkGUISupportQtSQL"
    "vtkGUISupportQtWebkit"
    "vtkViewsQt"
    "vtkRenderingQt")
endif()

find_package(OpenVDB REQUIRED)
find_package(TBB REQUIRED)
remove_definitions(-DDISABLE_LIBUSB-1.0)
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED COMPONENTS system thread)
include_directories(
    include
    ${BOOST_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
    ${catkin_INCLUDE_DIRS}
    ${OpenVDB_INCLUDE_DIR}
    ${TBB_INCLUDE_DIRS}
)

add_definitions(${EIGEN3_DEFINITIONS})

catkin_package(
    INCLUDE_DIRS
        include
        ${BOOST_INCLUDE_DIRS}
        ${EIGEN3_INCLUDE_DIRS}
        ${PCL_INCLUDE_DIRS}
        ${OpenVDB_INCLUDE_DIR}
        ${TBB_INCLUDE_DIRS}
    LIBRARIES ${PROJECT_NAME}
    CATKIN_DEPENDS
        geometry_msgs
        laser_geometry
        message_filters
        pcl_ros
        pcl_conversions
        pluginlib
        roscpp
        sensor_msgs
        std_msgs
        costmap_2d
        tf2_ros
        tf2_geometry_msgs
        visualization_msgs
        dynamic_reconfigure
    DEPENDS
        TBB
        OpenVDB
        PCL
        EIGEN3
        Boost
)

add_library(${PROJECT_NAME}
            src/spatio_temporal_voxel_layer.cpp
            src/spatio_temporal_voxel_grid.cpp
            src/measurement_buffer.cpp
            src/frustum_models/depth_camera_frustum.cpp
            src/frustum_models/three_dimensional_lidar_frustum.cpp
            src/vdb2pc.cpp
)
add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_generate_messages_cpp ${PROJECT_NAME}_gencfg)
target_link_libraries(${PROJECT_NAME}
  ${Boost_LIBRARIES}
  ${EIGEN3_LIBRARIES}
  ${PCL_LIBRARIES}
  ${OpenVDB_LIBRARIES}
  ${TBB_LIBRARIES}
  ${catkin_LIBRARIES}
)

# For testing on standalone, don't want to ship with production builds
#add_executable(minimal_test
#            test/minimal_test.cpp
#)
#target_link_libraries(minimal_test
#  ${Boost_LIBRARIES}
#  ${EIGEN3_LIBRARIES}
#  ${PCL_LIBRARIES}
#  ${OpenVDB_LIBRARIES}
#  ${TBB_LIBRARIES}
#  ${catkin_LIBRARIES}
#)

install(TARGETS ${PROJECT_NAME} #minimal_test
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES costmap_plugins.xml test/minimal_test.launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY example
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
