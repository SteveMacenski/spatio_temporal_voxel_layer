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
 *********************************************************************/

#include <spatio_temporal_voxel_layer/level_set.h>

namespace spatio_temporal_voxel_layer
{

/*****************************************************************************/
LevelSet::LevelSet(const float& voxel_size, const int& background_value, const bool& rolling) : 
                                       _background_value( background_value ), \
                                       _voxel_size( voxel_size )
/*****************************************************************************/
{
  this->InitializeGrid(rolling);
}

/*****************************************************************************/
void LevelSet::InitializeGrid(const bool& rolling)
/*****************************************************************************/
{
  // initialize the OpenVDB Grid volume
  openvdb::initialize();

  _grid = openvdb::FloatGrid::create( _background_value );

  openvdb::Mat4d m = openvdb::Mat4d::identity();
  m.preScale(openvdb::Vec3d(_voxel_size, _voxel_size, _voxel_size));
  m.preTranslate(openvdb::Vec3d(0, 0, 0));
  m.preRotate(openvdb::math::Z_AXIS, 0);

  _grid->setTransform(openvdb::math::Transform::createLinearTransform( m ));
  _grid->insertMeta("ObstacleLayerVoxelGrid", openvdb::BoolMetadata( 1 ));
  _grid->setName("SpatioTemporalVoxelLayer");
  _grid->insertMeta("Rolling", openvdb::BoolMetadata( rolling ));
  _grid->insertMeta("Effective Voxel Size in Meters", openvdb::FloatMetadata( _voxel_size ));

  _grid->setGridClass(openvdb::GRID_LEVEL_SET);
}

/*****************************************************************************/
void LevelSet::ParallelizeMarkAndClear(const std::vector<costmap_2d::Observation>& marking_observations, \
                                       const std::vector<costmap_2d::Observation>& clearing_observations)
/*****************************************************************************/
{
  std::vector<parallel_request> observation_requests;
  for (std::vector<costmap_2d::Observation>::const_iterator it = marking_observations.begin(); \
                                                       it != marking_observations.end(); ++it)
  {
    observation_requests.push_back(parallel_request(*it, true));
  }
  for (std::vector<costmap_2d::Observation>::const_iterator it = clearing_observations.begin(); \
                                                       it != clearing_observations.end(); ++it)
  {
    observation_requests.push_back(parallel_request(*it, false));
  }
  if (observation_requests.size() > 0) 
  {
    tbb::parallel_do(observation_requests, *this);
  }
  return;
}

/*****************************************************************************/
void LevelSet::operator()(const parallel_request& obs) const
/*****************************************************************************/
{
  openvdb::FloatGrid::Accessor accessor = _grid->getAccessor();

  if (obs.marking)
  {
    double mark_range_2 = obs.observation.obstacle_range_ * obs.observation.obstacle_range_;

    for (pcl::PointCloud<pcl::PointXYZ>::const_iterator it = obs.observation.cloud_->points.begin(); \
                                                    it < obs.observation.cloud_->points.end(); ++it)
    {
      double distance_2 = (it->x - obs.observation.origin_.x) * (it->x - obs.observation.origin_.x) \
                        + (it->y - obs.observation.origin_.y) * (it->y - obs.observation.origin_.y) \
                        + (it->z - obs.observation.origin_.z) * (it->z - obs.observation.origin_.z);
      if (distance_2 > mark_range_2 || distance_2 < 0.0001)
      {
        continue;
      }
      openvdb::Vec3d mark_grid(this->WorldToIndex(openvdb::Vec3d(it->x, it->y, it->z)));
      if(!MarkLevelSetPoint(openvdb::Coord(mark_grid[0], mark_grid[1], mark_grid[2]), 255, accessor)) {
        ROS_WARN_THROTTLE(1.,"Failed to mark point in levelset coordinates (%f %f %f)", 
                                            mark_grid[0], mark_grid[1], mark_grid[2]);
      }
    }
  }
  else 
  {
    if(_grid->empty())
    {
      return;
    }  
    openvdb::v3_1::tools::LevelSetRayIntersector<openvdb::FloatGrid> _tracer(*_grid);
    double raytrace_range_2 = obs.observation.raytrace_range_ * obs.observation.raytrace_range_;

    for (pcl::PointCloud<pcl::PointXYZ>::const_iterator it = obs.observation.cloud_->points.begin(); \
                                                    it < obs.observation.cloud_->points.end(); ++it)
    {
      double distance_2 = (it->x - obs.observation.origin_.x) * (it->x - obs.observation.origin_.x) \
                        + (it->y - obs.observation.origin_.y) * (it->y - obs.observation.origin_.y) \
                        + (it->z - obs.observation.origin_.z) * (it->z - obs.observation.origin_.z);
      if (distance_2 > (raytrace_range_2+0.1) || distance_2 < 0.0001)
      {
        continue;
      }
      RaytraceLevelSet(obs.observation.origin_, *it, std::sqrt(distance_2), _tracer, accessor);
    }
  }
}

/*****************************************************************************/
void LevelSet::RaytraceLevelSet(const geometry_msgs::Point& origin, \
                                const pcl::PointXYZ& terminal, \
                                const double& mag, \
                                openvdb::v3_1::tools::LevelSetRayIntersector<openvdb::FloatGrid>& _tracer, \
                                openvdb::FloatGrid::Accessor& accessor) const
/*****************************************************************************/
{
  GridRay ray(Vec3Type(origin.x, origin.y, origin.z), \
              Vec3Type((terminal.x-origin.x)/mag,     \
                       (terminal.y-origin.y)/mag,     \
                       (terminal.z-origin.z)/mag ), 0.01, mag);

  if(!_tracer.setWorldRay(ray)) {
    return;
  }

  // find hits in volume and clear them
  std::vector<GridRay::TimeSpan> hits;
  _tracer.hits(hits);
  
  if (hits.empty())
  {
    return;
  }

  for (std::vector<GridRay::TimeSpan>::const_iterator iter = hits.begin(); iter != hits.end(); ++iter)
  {
    openvdb::Vec3d clear_pose = _tracer.getIndexPos(iter->mid());
    if(!ClearLevelSetPoint(openvdb::Coord(clear_pose[0], clear_pose[1], clear_pose[2]), accessor))
    {
     ROS_WARN_THROTTLE(1.,"Failed to clear point (%f %f %f).", clear_pose[0], \
                                               clear_pose[1], clear_pose[2]);
    }
  }
}

/*****************************************************************************/
void LevelSet::ProjectVoxelGridTo2DPlane(std::vector<std::vector<int> >& flattened_costmap, \
                                       const int& mark_threshold, const int& size_x, \
                                       const int& size_y)
/*****************************************************************************/
{
  // project voxelgrid onto 2D plane for conversion to ROS Costmap2d, optimize GPU TBB
  // todo see if it has some projection functions to use, or cache values - as marked or cleared +1/-1 each tile

  // for each active non-background cell, add 1 to the 2D projection, once > mark_threshold, give FATAL cost
  if(_grid->empty())
  {
    return;
  }
  /*
  for (openvdb::FloatGrid::ValueOnCIter citer = _grid->cbeginValueOn(); citer; ++citer)
  {
    if (citer.getValue() > _background_value)
    {
      openvdb::Coord pose_(citer.getCoord());
      ROS_WARN("pt: temp %i %i", pose_[0], pose_[1]);
      //if (pose_[0] <= size_x && pose_[1] <= size_y)
      //{
        if (flattened_costmap[pose_[0]][pose_[1]] != 255 )
        {
          flattened_costmap[pose_[0]][pose_[1]] += 1;
          if (flattened_costmap[pose_[0]][pose_[1]] >= mark_threshold)
          {
            flattened_costmap[pose_[0]][pose_[1]] = 255;
            ROS_INFO("temp I marked a 2d");
          }
        }
      //}
    }
  } */
}

/*****************************************************************************/
void LevelSet::GridToPointCloud2(pcl::PointCloud<pcl::PointXYZ>& pc)
/*****************************************************************************/
{
  // convert the openvdb voxel grid to a pointcloud for publishing
  if(_grid->empty())
  {
    return;
  }

  for (openvdb::FloatGrid::ValueOnCIter citer = _grid->cbeginValueOn(); citer; ++citer) // enforce temporal constraints here
  {
    if (citer.getValue() > _background_value )
    {
      openvdb::Vec3d pose_world = _grid->indexToWorld(citer.getCoord());
      pc.push_back(pcl::PointXYZ(pose_world[0], pose_world[1], pose_world[2]));
    }
  }
}

/*****************************************************************************/
void LevelSet::CopyLevelSetRegion()
/*****************************************************************************/
{
  ROS_FATAL("no copying VDB implemented, rolling costmaps are not possible. Do not proceed");
}

/*****************************************************************************/
bool LevelSet::ResetLevelSet(void)
/*****************************************************************************/
{
  // clear the voxel grid
  _grid->clear();
  if (_grid->empty())
  {
    ROS_INFO("Level set has been reset.");
    return true;
  }
  return false;
}

/*****************************************************************************/
void LevelSet::ResizeLevelSet(int cells_dx, int cells_dy, double resolution, \
                             double origin_x, double origin_y)
/*****************************************************************************/
{
  ROS_WARN("resizing VDB grid is not yet implemented");
}

/*****************************************************************************/
bool LevelSet::MarkLevelSetPoint(const openvdb::Coord& pt, const int& value, openvdb::FloatGrid::Accessor& accessor) const
/*****************************************************************************/
{
  int curr_value = accessor.getValue(pt);
  if (curr_value == _background_value || curr_value < value)
  {
    accessor.setValueOn(pt, value);
  }
  return accessor.getValue(pt) == value;
}

/*****************************************************************************/
bool LevelSet::ClearLevelSetPoint(const openvdb::Coord& pt, openvdb::FloatGrid::Accessor& accessor) const
/*****************************************************************************/
{
  accessor.setValueOff(pt, _background_value);
  return !accessor.isValueOn(pt);
}

/*****************************************************************************/
openvdb::Vec3d LevelSet::IndexToWorld(const openvdb::Coord& coord) const
/*****************************************************************************/
{
  // Applies tranform stored in getTransform.
  return _grid->indexToWorld(coord);
}

/*****************************************************************************/
openvdb::Vec3d LevelSet::WorldToIndex(const openvdb::Vec3d& vec) const
/*****************************************************************************/
{
  // Applies inverse tranform stored in getTransform.
  return _grid->worldToIndex(vec);
}

}; // end namespace
