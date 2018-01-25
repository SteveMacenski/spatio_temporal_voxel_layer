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

#include <spatio_temporal_voxel_layer/level_set.hpp>

namespace volume_grid
{

/*****************************************************************************/
LevelSet::LevelSet(const float& voxel_size, const int& background_value, \
                                    const bool& rolling) : 
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
  _grid->insertMeta("Effective Voxel Size in Meters", \
                    openvdb::FloatMetadata( _voxel_size ));

  _grid->setGridClass(openvdb::GRID_LEVEL_SET);
}

/*****************************************************************************/
void LevelSet::ParallelizeClearFrustums(const \
               std::vector<observation::MeasurementReading>& clearing_readings)
/*****************************************************************************/
{
  if(this->IsGridEmpty())
  {
    return;
  }

  if (clearing_readings.size() > 0) 
  {
    tbb::parallel_do(clearing_readings, *this);
  }
}

/*****************************************************************************/
void LevelSet::ParallelizeMark(const \
                std::vector<observation::MeasurementReading>& marking_readings)
/*****************************************************************************/
{
  if (marking_readings.size() > 0) 
  {
    tbb::parallel_do(marking_readings, *this);
  }
  return;
}

/*****************************************************************************/
void LevelSet::operator()(const observation::MeasurementReading& obs) const
/*****************************************************************************/
{
  openvdb::FloatGrid::Accessor accessor = _grid->getAccessor();

  if (obs._marking)
  {
    double mark_range_2 = obs._obstacle_range_in_m * obs._obstacle_range_in_m;

    pcl::PointCloud<pcl::PointXYZ>::const_iterator it;
    for (it = obs._cloud->points.begin(); it < obs._cloud->points.end(); ++it)
    {
      double distance_2 = (it->x - obs._origin.x) * (it->x - obs._origin.x) \
                        + (it->y - obs._origin.y) * (it->y - obs._origin.y) \
                        + (it->z - obs._origin.z) * (it->z - obs._origin.z);
      if (distance_2 > mark_range_2 || distance_2 < 0.0001)
      {
        continue;
      }
      openvdb::Vec3d mark_grid(this->WorldToIndex( \
                                       openvdb::Vec3d(it->x, it->y, it->z)));
      if(!this->MarkLevelSetPoint(openvdb::Coord( \
                 mark_grid[0], mark_grid[1], mark_grid[2]), 255, accessor)) {
        ROS_WARN_THROTTLE(1., \
                    "Failed to mark point in levelset coordinates (%f %f %f)", 
                                    mark_grid[0], mark_grid[1], mark_grid[2]);
      }
    }
  }

  if (obs._clearing)
  {
    geometry::Frustum frustum(obs._vertical_fov_in_rad,   \
                              obs._horizontal_fov_in_rad, \
                              obs._min_z_in_m,            \
                              obs._max_z_in_m);
    frustum.SetPosition(obs._origin);
    frustum.SetOrientation();
    frustum.TransformPlaneNormals();

    openvdb::FloatGrid::ValueOnCIter citer = _grid->cbeginValueOn();
    for (citer; citer; ++citer) 
    {
      const openvdb::Coord pt_index(citer.getCoord());
      if (!frustum.IsInside(this->IndexToWorld(pt_index)))
      {
        // check temporal constraints here and clear if needed TODO or
        // not, and do in 
        //costmap flat/PC2 since this would happen for each threaded obs 
        //iterate through 
        //full grid TODO
        continue;
      }
      else
      {
        // check temporal constraints here and accelerate by chosen model TODO
        ROS_WARN_THROTTLE(2.,"a point was in frustum!");
        if(!this->ClearLevelSetPoint(pt_index, accessor))
        {
          ROS_WARN_THROTTLE(1.,"Failed to clear point in levelset.");
        }
      }
    }
  }
}

/*****************************************************************************/
void LevelSet::GetFlattenedCostmap( \
                           std::vector<std::vector<int> >& flattened_costmap, \
                           const int& mark_threshold)
/*****************************************************************************/
{
  // project voxelgrid onto 2D plane for conversion to ROS Costmap2d
  if(this->IsGridEmpty())
  {
    return;
  }
  //TODO
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
  openvdb::FloatGrid::ValueOnCIter citer = _grid->cbeginValueOn();
  for (citer; citer; ++citer)
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
bool LevelSet::MarkLevelSetPoint(const openvdb::Coord& pt, \
                const int& value, openvdb::FloatGrid::Accessor& accessor) const
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
bool LevelSet::ClearLevelSetPoint(const \
              openvdb::Coord& pt, openvdb::FloatGrid::Accessor& accessor) const
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
