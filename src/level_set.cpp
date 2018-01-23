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


// minimize number of full grid iterations, PC2 + flattening 1 iteration. Try to get inside of clearing if last?

/*****************************************************************************/
LevelSet::LevelSet(const float& voxel_size, const int& background_value, const bool& rolling) : 
                                       _background_value( background_value ), \
                                       _voxel_size( voxel_size )
/*****************************************************************************/
{
  this->InitializeGrid(rolling);
}

/*****************************************************************************/
LevelSet::~LevelSet()
/*****************************************************************************/
{
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
void LevelSet::TemporallyClearFrustums(const std::vector<costmap_2d::Observation>& clearing_observations)
/*****************************************************************************/
{

  // (1) parameterize from sensor orientation / frames for frustum
  // (3) TF orientation
  // (4) frustum stuff what it means - visualize in RVIZ vis msgs lines 
  // (5) embed times and do


  // maybe define planes and check if interior. 
  // iterate through full grid and use as time to evaluate all time and acceleration? + project down + pointcloud conversion

  // The points (0,0,0), (100,0,0), (0,50,0) and (100,50,0) will map to
  // the corners of the near plane of the frustum, while the corners
  // of the far plane will be the images of (0,0,120), (100,0,120),
  // (0,50,120) and (100,50,120).

  if(this->IsGridEmpty())
  {
    return;
  }

  openvdb::FloatGrid::Accessor accessor = _grid->getAccessor();

  for (uint i=0; i!=clearing_observations.size(); i++)
  {
    const geometry_msgs::Point& origin = clearing_observations.at(i).origin_;

    const openvdb::Vec3d near_plane = this->WorldToIndex(openvdb::Vec3d(0, 0, 0));
    const openvdb::Vec3d far_plane = this->WorldToIndex(openvdb::Vec3d(0.4,0.4,7));

    const openvdb::math::BBox<openvdb::Vec3d> bbox(near_plane, far_plane); //planes of camera, from its geometry TODO

    const double taper = .2;// The far plane of the frustum will be twice as big as the near plane TODO
    const double depth = 1;// The depth of the frustum will be 10 times the x-width of the near plane TODO
    openvdb::math::NonlinearFrustumMap* frustumTransform = new openvdb::math::NonlinearFrustumMap(bbox, taper, depth);
    frustumTransform->preScale(openvdb::Vec3d(_voxel_size, _voxel_size, _voxel_size));
    frustumTransform->postTranslate(openvdb::math::Vec3d(origin.x, origin.y, origin.z));
    frustumTransform->postRotate(0., openvdb::math::Z_AXIS); //tf get rpy from baselink->camera depth frame transform TODO

    const openvdb::math::BBox<openvdb::Vec3d> frustumIndexBounds = frustumTransform->getBBox();

    for (openvdb::FloatGrid::ValueOnCIter citer = _grid->cbeginValueOn(); citer; ++citer) 
    {
      const openvdb::Coord pt_ig(citer.getCoord());                                             // point in index-grid
      const openvdb::Vec3d pt_if(frustumTransform->applyInverseMap(this->IndexToWorld(pt_ig))); // point in index-frustum
      if (!frustumIndexBounds.isInside(pt_if))
      {
        // check temporal constraints here and clear if needed TODO for now passing
        continue;
      }
      else
      {
        // check temporal constraints here and accelerate by chosen model TODO for now just clearing
        ROS_WARN_THROTTLE(2.,"a point was in frustum!");
        if(!this->ClearLevelSetPoint(pt_ig, accessor))
        {
          ROS_WARN_THROTTLE(1.,"Failed to clear point in levelset.");
        }
      }


    }
    delete frustumTransform;
  }
}

/*****************************************************************************/
void LevelSet::ParallelizeMark(const std::vector<costmap_2d::Observation>& marking_observations)
/*****************************************************************************/
{
  if (marking_observations.size() > 0) 
  {
    tbb::parallel_do(marking_observations, *this);
  }
  return;
}

/*****************************************************************************/
void LevelSet::operator()(const costmap_2d::Observation& obs) const
/*****************************************************************************/
{
  openvdb::FloatGrid::Accessor accessor = _grid->getAccessor();
  double mark_range_2 = obs.obstacle_range_ * obs.obstacle_range_;

  for (pcl::PointCloud<pcl::PointXYZ>::const_iterator it = obs.cloud_->points.begin(); \
                                                  it < obs.cloud_->points.end(); ++it)
  {
    double distance_2 = (it->x - obs.origin_.x) * (it->x - obs.origin_.x) \
                      + (it->y - obs.origin_.y) * (it->y - obs.origin_.y) \
                      + (it->z - obs.origin_.z) * (it->z - obs.origin_.z);
    if (distance_2 > mark_range_2 || distance_2 < 0.0001)
    {
      continue;
    }
    openvdb::Vec3d mark_grid(this->WorldToIndex(openvdb::Vec3d(it->x, it->y, it->z)));
    if(!this->MarkLevelSetPoint(openvdb::Coord(mark_grid[0], mark_grid[1], mark_grid[2]), 255, accessor)) {
      ROS_WARN_THROTTLE(1.,"Failed to mark point in levelset coordinates (%f %f %f)", 
                                          mark_grid[0], mark_grid[1], mark_grid[2]);
    }
  }
}

/*****************************************************************************/
void LevelSet::GetFlattenedCostmap(std::vector<std::vector<int> >& flattened_costmap, \
                                   const int& mark_threshold)
/*****************************************************************************/
{
  // project voxelgrid onto 2D plane for conversion to ROS Costmap2d, optimize GPU TBB
  // todo see if it has some projection functions to use, or cache values - as marked or cleared +1/-1 each tile

  // for each active non-background cell, add 1 to the 2D projection, once > mark_threshold, give FATAL cost
  if(this->IsGridEmpty())
  {
    return;
  }
}

/*****************************************************************************/
void LevelSet::GetOccupancyPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr& pc)
/*****************************************************************************/
{
  // convert the openvdb voxel grid to a pointcloud for publishing
  if(this->IsGridEmpty())
  {
    return;
  }

  for (openvdb::FloatGrid::ValueOnCIter citer = _grid->cbeginValueOn(); citer; ++citer)
  {
    if (citer.getValue() > _background_value )
    {
      openvdb::Vec3d pose_world = _grid->indexToWorld(citer.getCoord());
      pc->push_back(pcl::PointXYZ(pose_world[0], pose_world[1], pose_world[2]));
    }
  }
}

/*****************************************************************************/
bool LevelSet::ResetLevelSet(void)
/*****************************************************************************/
{
  // clear the voxel grid
  _grid->clear();
  if (this->IsGridEmpty())
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

/*****************************************************************************/
bool LevelSet::IsGridEmpty() const
/*****************************************************************************/
{
  // Returns grid's population status
  return _grid->empty();
}

}; // end namespace
