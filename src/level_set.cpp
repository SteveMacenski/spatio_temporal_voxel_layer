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
                            _pc(new pcl::PointCloud<pcl::PointXYZ>), \
                         _cost_map(new std::unordered_map<occupany_cell, uint>)
/*****************************************************************************/
{
  this->InitializeGrid();
}

/*****************************************************************************/
LevelSet::~LevelSet(void)
/*****************************************************************************/
{ 
  // pcl pointclouds free themselves
  delete _cost_map;
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

  _pc->clear();
  _cost_map->clear();

  std::vector<frustum_model> obs_frustums;

  if(clearing_readings.size() == 0)
  {
    obs_frustums.push_back(frustum_model(geometry::Frustum(0.,0.,0.,0.), 0.));
    TemporalClearAndGenerateCostmap(obs_frustums);
    return;
  }

  obs_frustums.reserve(clearing_readings.size());

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
    obs_frustums.emplace_back(frustum, it->_decay_acceleration);
  }
  TemporalClearAndGenerateCostmap(obs_frustums);
  return;
}

/*****************************************************************************/
void LevelSet::TemporalClearAndGenerateCostmap(                               \
                                          std::vector<frustum_model>& frustums)
/*****************************************************************************/
{
  // check each point in the grid for inclusion in a frustum
  openvdb::DoubleGrid::ValueOnCIter cit_grid = _grid->cbeginValueOn();
  for (cit_grid; cit_grid; ++cit_grid)
  {
    const openvdb::Coord pt_index(cit_grid.getCoord());

    std::vector<frustum_model>::iterator frustum_it = frustums.begin();
    bool frustum_cycle = false;

    for(frustum_it; frustum_it != frustums.end(); ++frustum_it)
    {
      if ( frustum_it->frustum.IsInside(this->IndexToWorld(pt_index)) )
      {
        frustum_cycle = true;
        const double accel_decay_time = \
                            GetAcceleratedDecayTime(frustum_it->accel_factor);
        if (true) //cit_grid.getValue() < accel_decay_time)
        {
          // accelerate the values stored. Ticket #23 TODO
          // if(!this->MarkLevelSetPoint(pt_index, \
          //    cit_grid.getValue()-accel_decay_time))
          // {
          //   std::cout << "Failed to clear point." << std::endl;
          // }
          this->ClearLevelSetPoint(pt_index); // temp for testing
        }
        else
        {
          // expired by acceleration
          // if(!this->ClearLevelSetPoint(pt_index))
          // {
          //   std::cout << "Failed to clear point." << std::endl;
          // }
          break;
        }
      }
    }

    // if not inside any, check against nominal decay model
    if(!frustum_cycle)  
    {
      const double decay_time = GetDecayTime();
      if(cit_grid.getValue() < decay_time)
      {
        if(!this->ClearLevelSetPoint(pt_index))
        {
          std::cout << "Failed to clear point." << std::endl;
        }
        break;
      }
    }
    // if here, we can add to costmap and PC2
    PopulateCostmapAndPointcloud(pt_index);
  }
}

/*****************************************************************************/
void LevelSet::PopulateCostmapAndPointcloud(const openvdb::Coord& pt)
/*****************************************************************************/
{
  // add pt to the pointcloud and costmap
  openvdb::Vec3d pose_world = _grid->indexToWorld(pt);
  if (_pub_voxels)
  {
    _pc->push_back(pcl::PointXYZ(pose_world[0], pose_world[1], \
                                 pose_world[2]));
  }

  std::unordered_map<occupany_cell, uint>::iterator cell;
  cell = _cost_map->find(occupany_cell(pose_world[0], pose_world[1]));
  if (cell != _cost_map->end())
  {
    cell->second += 1;
  }
  else
  {
    _cost_map->insert(std::make_pair( \
                              occupany_cell(pose_world[0], pose_world[1]), 1));
  }
}

/*****************************************************************************/
void LevelSet::ParallelizeMark(const \
                std::vector<observation::MeasurementReading>& marking_readings)
/*****************************************************************************/
{
  // mark the grid in parallel
  if (marking_readings.size() > 0) 
  {
    //tbb::parallel_do(marking_readings, *this);
    for (int i=0; i!= marking_readings.size(); i++)
    {
      (*this)(marking_readings.at(i));
    }
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
        std::cout << "Failed to mark point." << std::endl;
      }
    }
  }
  return;
}

/*****************************************************************************/
std::unordered_map<occupany_cell, uint>* LevelSet::GetFlattenedCostmap()
/*****************************************************************************/
{
  return _cost_map;
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
  try
  {
    _grid->clear();
    if (this->IsGridEmpty())
    {
      return true;
    }
  }
  catch (...)
  {
    std::cout << "Failed to reset costmap, please try again." << std::endl;
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

/*****************************************************************************/
bool LevelSet::SaveGrid(const std::string& file_name)
/*****************************************************************************/
{
  try
  {
    openvdb::io::File file(file_name + ".vdb");
    openvdb::GridPtrVec grids = { _grid };
    file.write(grids);
    file.close();
    return true;
  }
  catch (...)
  {
    return false;
  }
  return false; // best offense is a good defense
}

}; // end namespace
