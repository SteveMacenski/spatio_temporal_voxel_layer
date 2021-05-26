/*********************************************************************
 *
 * Software License Agreement
 *
 *  Copyright (c) 2018, Simbe Robotics, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Simbe Robotics, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * Author: Steve Macenski (steven.macenski@simberobotics.com)
 * Purpose: Implement OpenVDB's voxel library with ray tracing for our
 *          internal voxel grid layer.
 *********************************************************************/

#ifndef VOLUME_GRID_H_
#define VOLUME_GRID_H_

// PCL
#include <pcl_ros/transforms.h>
#include <pcl/PCLPointCloud2.h>
// ROS
#include <ros/ros.h>
// STL
#include <math.h>
#include <unordered_map>
#include <unordered_set>
#include <ctime>
#include <iostream>
#include <utility>
#include <vector>
// msgs
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Point.h>
// TBB
#include <tbb/parallel_do.h>
// OpenVDB
#include <openvdb/openvdb.h>
#include <openvdb/tools/GridTransformer.h>
#include <openvdb/tools/RayIntersector.h>
// measurement struct and buffer
#include <spatio_temporal_voxel_layer/measurement_buffer.hpp>
#include <spatio_temporal_voxel_layer/frustum_models/depth_camera_frustum.hpp>
#include <spatio_temporal_voxel_layer/frustum_models/three_dimensional_lidar_frustum.hpp>
// Mutex and locks
#include <boost/thread.hpp>
#include <boost/thread/recursive_mutex.hpp>

namespace volume_grid
{

enum GlobalDecayModel
{
  LINEAR = 0,
  EXPONENTIAL = 1,
  PERSISTENT = 2
};

// Structure for an occupied cell for map
struct occupany_cell
{
  occupany_cell(const double& _x, const double& _y) :
    x(_x), y(_y)
  {
  }

  bool operator==(const occupany_cell& other) const
  {
    return x==other.x && y==other.y;
  }

  double x, y;
};

// Structure for wrapping frustum model and necessary metadata
struct frustum_model
{
  frustum_model(geometry::Frustum* _frustum, const double& _factor) :
    frustum(_frustum), accel_factor(_factor)
  {
  }
  ~frustum_model()
  {
    if (frustum)
    {
      delete frustum;
    }
  }
  geometry::Frustum* frustum;
  const double accel_factor;
};

// Core voxel grid structure and interface
class SpatioTemporalVoxelGrid
{
public:
  // conveniences for line lengths
  typedef openvdb::math::Ray<openvdb::Real> GridRay;
  typedef openvdb::math::Ray<openvdb::Real>::Vec3T Vec3Type;

  SpatioTemporalVoxelGrid(const float& voxel_size, const double& background_value,
                          const int& decay_model, const double& voxel_decay,
                          const bool& pub_voxels);
  ~SpatioTemporalVoxelGrid(void);

  // Core making and clearing functions
  void Mark(const std::vector<observation::MeasurementReading>& marking_observations);
  void operator()(const observation::MeasurementReading& obs) const;
  void ClearFrustums(const std::vector<observation::MeasurementReading>& clearing_observations, \
                     std::unordered_set<occupany_cell>& cleared_cells);

  // Get the pointcloud of the underlying occupancy grid
  void GetOccupancyPointCloud(sensor_msgs::PointCloud2::Ptr& pc2);
  std::unordered_map<occupany_cell, uint>* GetFlattenedCostmap();

  // Clear the grid
  bool ResetGrid(void);

  // Save the file to file with size information
  bool SaveGrid(const std::string& file_name, double& map_size_bytes);

protected:
  // Initialize grid metadata and library
  void InitializeGrid(void);

  // grid accessor methods
  bool MarkGridPoint(const openvdb::Coord& pt, const double& value) const;
  bool ClearGridPoint(const openvdb::Coord& pt) const;

  // Check occupancy status of the grid
  bool IsGridEmpty(void) const;

  // Get time information for clearing
  double GetTemporalClearingDuration(const double& time_delta);
  double GetFrustumAcceleration(const double& time_delta, \
                                const double& acceleration_factor);
  void TemporalClearAndGenerateCostmap(std::vector<frustum_model>& frustums, \
                                       std::unordered_set<occupany_cell>& cleared_cells);

  // Populate the costmap ROS api and pointcloud with a marked point
  void PopulateCostmapAndPointcloud(const openvdb::Coord& pt);

  // Utilities for tranformation
  openvdb::Vec3d WorldToIndex(const openvdb::Vec3d& coord) const;
  openvdb::Vec3d IndexToWorld(const openvdb::Coord& coord) const;

  mutable openvdb::DoubleGrid::Ptr _grid;
  int                             _decay_model;
  double                          _background_value, _voxel_size, _voxel_decay;
  bool                            _pub_voxels;
  std::vector<geometry_msgs::Point32>*   _grid_points;
  std::unordered_map<occupany_cell, uint>* _cost_map;
  boost::mutex                            _grid_lock;
};

} // end volume_grid namespace

// hash function for unordered_map of occupancy_cells
namespace std {
template <>
struct hash<volume_grid::occupany_cell>
{
  std::size_t operator()(const volume_grid::occupany_cell& k) const
  {
    return ((std::hash<double>()(k.x) ^ (std::hash<double>()(k.y) << 1)) >> 1);
  }
};

} // end std namespace

#endif
