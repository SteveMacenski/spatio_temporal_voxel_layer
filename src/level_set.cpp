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
LevelSet::LevelSet(const float& voxel_size, const int& background_value,\
                   const int& decay_model, const double& voxel_decay) : 
                            _background_value( background_value ),   \
                            _voxel_size( voxel_size ),               \
                            _decay_model(decay_model),               \
                            _voxel_decay(voxel_decay),               \
                            _pc(new pcl::PointCloud<pcl::PointXYZ>)
/*****************************************************************************/
{
  this->InitializeGrid();
}

/*****************************************************************************/
LevelSet::~LevelSet(void)
/*****************************************************************************/
{
}

/*****************************************************************************/
void LevelSet::InitializeGrid(void)
/*****************************************************************************/
{
  // initialize the OpenVDB Grid volume
  openvdb::initialize();

  _grid = openvdb::DoubleGrid::create( _background_value );

  openvdb::Mat4d m = openvdb::Mat4d::identity();
  m.preScale(openvdb::Vec3d(_voxel_size, _voxel_size, _voxel_size));
  m.preTranslate(openvdb::Vec3d(0, 0, 0));
  m.preRotate(openvdb::math::Z_AXIS, 0);

  _grid->setTransform(openvdb::math::Transform::createLinearTransform( m ));
  _grid->setName("SpatioTemporalVoxelLayer");
  _grid->insertMeta("Voxel Size", openvdb::FloatMetadata( _voxel_size ));
  _grid->setGridClass(openvdb::GRID_LEVEL_SET);
  return;
}

/*****************************************************************************/
void LevelSet::ClearFrustums(const \
               std::vector<observation::MeasurementReading>& clearing_readings)
/*****************************************************************************/
{
  // accelerate the decay of voxels interior to the frustum
  if(this->IsGridEmpty())
  {
    return;
  }

  openvdb::DoubleGrid::Accessor accessor = _grid->getAccessor();

  std::vector<observation::MeasurementReading>::const_iterator it = \
                                                  clearing_readings.begin();
  for (it; it != clearing_readings.end(); ++it) // parallelize: ticket 18 TODO
  {
    geometry::Frustum frustum(it->_vertical_fov_in_rad,   \
                              it->_horizontal_fov_in_rad, \
                              it->_min_z_in_m,            \
                              it->_max_z_in_m);
    frustum.SetPosition(it->_origin);
    frustum.SetOrientation(it->_orientation);
    frustum.TransformPlaneNormals();

    openvdb::DoubleGrid::ValueOnCIter citer = _grid->cbeginValueOn();
    for (citer; citer; ++citer) 
    {
      const openvdb::Coord pt_index(citer.getCoord());
      if ( frustum.IsInside(this->IndexToWorld(pt_index)) )
      {
        const double accel_decay_time = \
                            GetAcceleratedDecayTime(it->_decay_acceleration);
        if ( citer.getValue() < accel_decay_time)
        {
          // accelerate this value by how much? Ticket #23 TODO
          // if(!this->MarkLevelSetPoint(pt_index, \
          //    citer.getValue()-accel_decay_time))
          // {
          //   ROS_WARN_THROTTLE(2.,"Failed to clear point.");
          // }
          ROS_WARN_THROTTLE(2., "acceleration!");
          this->ClearLevelSetPoint(pt_index);
        }
        else
        {
          // clear this value it's expired by acceleration
          // if(!this->ClearLevelSetPoint(pt_index))
          // {
          //   ROS_WARN_THROTTLE(2.,"Failed to clear point.");
          // }
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
  // mark the grid in parallel
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
  if (obs._marking)
  {
    float mark_range_2 = obs._obstacle_range_in_m * obs._obstacle_range_in_m;
    const double cur_time = ros::WallTime::now().toSec();

    pcl::PointCloud<pcl::PointXYZ>::const_iterator it;
    for (it = obs._cloud->points.begin(); it < obs._cloud->points.end(); ++it)
    {
      float distance_2 = (it->x - obs._origin.x) * (it->x - obs._origin.x) \
                        + (it->y - obs._origin.y) * (it->y - obs._origin.y) \
                        + (it->z - obs._origin.z) * (it->z - obs._origin.z);
      if (distance_2 > mark_range_2 || distance_2 < 0.0001)
      {
        continue;
      }
      openvdb::Vec3d mark_grid(this->WorldToIndex( \
                                       openvdb::Vec3d(it->x, it->y, it->z)));

      if(!this->MarkLevelSetPoint(openvdb::Coord( \
            mark_grid[0], mark_grid[1], mark_grid[2]), cur_time)) {
        ROS_WARN_THROTTLE(1., "Failed to mark point.");
      }
    }
  }
  return;
}

/*****************************************************************************/
void LevelSet::GetFlattenedCostmap( \
                             std::unordered_map<occupany_cell, uint>& cell_map)
/*****************************************************************************/
{
  // retreive the 2D costmap to project to layered costmaps
  if(this->IsGridEmpty())
  {
    return;
  }

  _pc->clear();

  openvdb::DoubleGrid::ValueOnCIter citer = _grid->cbeginValueOn();

  const double decay_time = GetDecayTime(); 

  for (citer; citer; ++citer)
  {
    double cell_time = citer.getValue();
    if ( cell_time > decay_time )
    {
      openvdb::Vec3d pose_world = _grid->indexToWorld(citer.getCoord());
      if (_pub_voxels)
      {
        _pc->push_back(pcl::PointXYZ(pose_world[0], pose_world[1], \
                                     pose_world[2]));
      }

      std::unordered_map<occupany_cell, uint>::iterator cell;
      cell = cell_map.find(occupany_cell(pose_world[0], pose_world[1]));
      if (cell != cell_map.end())
      {
        cell->second += 1;
      }
      else
      {
        cell_map[occupany_cell(pose_world[0], pose_world[1])] = 1;
      }
    }
    else
    {
      if(!this->ClearLevelSetPoint(citer.getCoord()))
      {
        ROS_WARN_THROTTLE(2.,"Failed to clear point in levelset.");
      }
    }
  }
  return;
}

/*****************************************************************************/
double LevelSet::GetDecayTime(void)
/*****************************************************************************/
{
  // use configurable model to get desired decay time
  const double cur_time = ros::WallTime::now().toSec();

  if (_decay_model == 0) // linear
  {
    return cur_time - _voxel_decay;
  }
  else if (_decay_model == 1) // exponential
  {
    return cur_time * std::exp(cur_time * _voxel_decay);
  }
  return 0.; // permanent
}

/*****************************************************************************/
double LevelSet::GetAcceleratedDecayTime(const double& acceleration_factor)
/*****************************************************************************/
{
  // use configurable model to get desired decay time
  const double cur_time = ros::WallTime::now().toSec();

  if (_decay_model == 0) // linear
  {
    return cur_time; //TODO Ticket #23
  }
  else if (_decay_model == 1) // exponential
  {
    return cur_time; //TODO Ticket #23
  }
  return 0.; // permanent
}

/*****************************************************************************/
void LevelSet::GetOccupancyPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr& pc)
/*****************************************************************************/
{
  // return the pointcloud stored
  if(!this->IsGridEmpty())
  {
    pc = _pc;
  }
  return;
}

/*****************************************************************************/
bool LevelSet::ResetLevelSet(void)
/*****************************************************************************/
{
  // clear the voxel grid
  _grid->clear();
  if (this->IsGridEmpty())
  {
    return true;
  }
  return false;
}

/*****************************************************************************/
bool LevelSet::MarkLevelSetPoint(const openvdb::Coord& pt, \
                                                     const double& value) const
/*****************************************************************************/
{
  // marking the OpenVDB set
  openvdb::DoubleGrid::Accessor accessor = _grid->getAccessor();

  int curr_value = accessor.getValue(pt);
  accessor.setValueOn(pt, value);
  return accessor.getValue(pt) == value;
}

/*****************************************************************************/
bool LevelSet::ClearLevelSetPoint(const openvdb::Coord& pt) const
/*****************************************************************************/
{
  // clearing the OpenVDB set
  openvdb::DoubleGrid::Accessor accessor = _grid->getAccessor();

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
bool LevelSet::IsGridEmpty(void) const
/*****************************************************************************/
{
  // Returns grid's population status
  return _grid->empty();
}

}; // end namespace
