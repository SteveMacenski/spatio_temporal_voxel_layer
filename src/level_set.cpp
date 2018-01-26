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
LevelSet::LevelSet(const float& voxel_size, const int& background_value) : 
                            _background_value( background_value ), \
                            _voxel_size( voxel_size ),             \
                            _pc(new pcl::PointCloud<pcl::PointXYZ>)
/*****************************************************************************/
{
  this->InitializeGrid();
}

/*****************************************************************************/
LevelSet::~LevelSet()
/*****************************************************************************/
{
}

/*****************************************************************************/
void LevelSet::InitializeGrid()
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
  _grid->setName("SpatioTemporalVoxelLayer");
  _grid->insertMeta("Voxel Size", openvdb::FloatMetadata( _voxel_size ));
  _grid->setGridClass(openvdb::GRID_LEVEL_SET);
}

/*****************************************************************************/
void LevelSet::ParallelizeClearFrustums(const \
               std::vector<observation::MeasurementReading>& clearing_readings)
/*****************************************************************************/
{
  scoped_lock l(_grid_lock);

  if(this->IsGridEmpty())
  {
    return;
  }
  openvdb::FloatGrid::Accessor accessor = _grid->getAccessor();

  if (clearing_readings.size() > 0) 
  {
    std::vector<observation::MeasurementReading>::const_iterator it = \
                                                    clearing_readings.begin();
    for (it; it!=clearing_readings.end(); ++it)
    {
      const observation::MeasurementReading& obs = *it;

      geometry::Frustum frustum(obs._vertical_fov_in_rad,   \
                                obs._horizontal_fov_in_rad, \
                                obs._min_z_in_m,            \
                                obs._max_z_in_m);
      frustum.SetPosition(obs._origin);
      frustum.SetOrientation(obs._orientation);
      frustum.TransformPlaneNormals();

      openvdb::FloatGrid::ValueOnCIter citer = _grid->cbeginValueOn();
      for (citer; citer; ++citer) 
      {
        const openvdb::Coord pt_index(citer.getCoord());
        if (!frustum.IsInside(this->IndexToWorld(pt_index)))
        {
          continue;
        }

        // check temporal constraints here and accelerate by chosen model TODO
        if(!this->ClearLevelSetPoint(pt_index, accessor))
        {
          ROS_WARN_THROTTLE(2.,"Failed to clear point in levelset.");
        }
      }
    }
  }

  return;
}

/*****************************************************************************/
void LevelSet::ParallelizeMark(const \
                std::vector<observation::MeasurementReading>& marking_readings)
/*****************************************************************************/
{
  scoped_lock l(_grid_lock);

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
                 mark_grid[0], mark_grid[1], mark_grid[2]), 255., accessor)) { //TODO embed timestamp here
        ROS_WARN_THROTTLE(1., "Failed to mark point in levelset coordinates");
      }
    }
  }
}

/*****************************************************************************/
void LevelSet::GetFlattenedCostmap( \
                                 std::vector<occupany_cell>& flattened_costmap)
/*****************************************************************************/
{
  scoped_lock l(_grid_lock);

  if(this->IsGridEmpty())
  {
    return;
  }

  _pc->clear();

  openvdb::FloatGrid::Accessor accessor = _grid->getAccessor();
  openvdb::FloatGrid::ValueOnCIter citer = _grid->cbeginValueOn();
  for (citer; citer; ++citer)
  {
    float cell_value = citer.getValue();
    if (cell_value > _background_value ) // TODO check timestamps for expiring
    {
      openvdb::Vec3d pose_world = _grid->indexToWorld(citer.getCoord());
      if (_pub_voxels)
      {
        _pc->push_back(pcl::PointXYZ(pose_world[0], pose_world[1], \
                                     pose_world[2]));
      }

      //hash function this  TODO
      // projected_list<struct, uint16>, projected_list[struct]++;
      // store this for return not the vector, check if there first
      bool match = false;
      int costmap_size = flattened_costmap.size(); 
      for (uint i=0; i!=costmap_size; i++)
      {
        if (pose_world[0] == flattened_costmap.at(i).x && \
                pose_world[1] == flattened_costmap.at(i).y)
        {
          flattened_costmap.at(i).value += 1;
          match = true;
          continue;
        }
      }
      if (!match)
      {
        flattened_costmap.push_back(occupany_cell(pose_world[0], \
                                                     pose_world[1], 1));
      }
    }
    else
    {
      if(!this->ClearLevelSetPoint(citer.getCoord(), accessor))
      {
        ROS_WARN_THROTTLE(2.,"Failed to clear point in levelset.");
      }
    }
  }

  return;
}

/*****************************************************************************/
void LevelSet::GetOccupancyPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr& pc)
/*****************************************************************************/
{
  if(this->IsGridEmpty())
  {
    return;
  }
  pc = _pc;
  return;
}

/*****************************************************************************/
bool LevelSet::ResetLevelSet(void)
/*****************************************************************************/
{
  // clear the voxel grid
  scoped_lock l(_grid_lock);
  _grid->clear();
  if (this->IsGridEmpty())
  {
    ROS_INFO("Level set has been reset.");
    return true;
  }
  return false;
}

/*****************************************************************************/
bool LevelSet::MarkLevelSetPoint(const openvdb::Coord& pt, \
                const float& value, openvdb::FloatGrid::Accessor& accessor) const
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
  if (accessor.isValueOn(pt))
  {
    accessor.setValueOff(pt, _background_value);
  }
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
