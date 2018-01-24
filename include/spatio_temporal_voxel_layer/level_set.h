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
 *   * Neither the name of Willow Garage, Inc. nor the names of its
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

// ros
#include <ros/ros.h>
// STL
#include <math.h>
// measurement struct
#include <spatio_temporal_voxel_layer/measurement_reading.h>
// msgs
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Point.h>
#include <visualization_msgs/Marker.h>
// TBB
#include <tbb/parallel_do.h>
// OpenVDB
#include <openvdb/openvdb.h>
#include <openvdb/tools/GridTransformer.h>
#include <openvdb/tools/RayIntersector.h>
// pcl
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include <pcl_ros/point_cloud.h>

#ifndef OPENVDB_GRID_H_
#define OPENVDB_GRID_H_

namespace spatio_temporal_voxel_layer
{

struct parallel_request
{
  parallel_request(const MeasurementReading& _obs, const bool& _marking) :
                                                        observation(_obs),
                                                        marking(_marking)
  {
    return;
  }

  MeasurementReading observation;
  bool marking;
};

class LevelSet
{
public:
  typedef openvdb::math::Ray<openvdb::Real> GridRay;
  typedef openvdb::math::Ray<openvdb::Real>::Vec3T Vec3Type;

  LevelSet(const float& voxel_size, const int& background_value, const bool& rolling);
  ~LevelSet();

  // mark and clear
  void ParallelizeMark(const std::vector<MeasurementReading>& marking_observations);
  void operator()(const MeasurementReading& obs) const;
  void TemporallyClearFrustums(const std::vector<MeasurementReading>& clearing_observations);

  // visualize and projection for ROS
  void GetOccupancyPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr& pc);
  void GetFlattenedCostmap(std::vector<std::vector<int> >& costmap, \
                          const int& mark_threshold);

  // ROS required primitives
  bool ResetLevelSet(void);
  void ResizeLevelSet(int cells_dx, int cells_dy, double resolution, double origin_x, double origin_y);

  // transformation functions
  openvdb::Vec3d IndexToWorld(const openvdb::Coord& coord) const;
  openvdb::Vec3d WorldToIndex(const openvdb::Vec3d& coord) const;

protected:
  void InitializeGrid(const bool& rolling);
  bool MarkLevelSetPoint(const openvdb::Coord& pt, const int& value, openvdb::FloatGrid::Accessor& accessor) const;
  bool ClearLevelSetPoint(const openvdb::Coord& pt, openvdb::FloatGrid::Accessor& accessor) const;
  bool IsGridEmpty() const;

  mutable openvdb::FloatGrid::Ptr _grid;
  int                             _background_value;
  double                          _voxel_size;
  bool                            _pub_voxels;
};

};
#endif