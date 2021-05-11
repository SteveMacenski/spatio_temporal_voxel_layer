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
 *********************************************************************/

#include <spatio_temporal_voxel_layer/spatio_temporal_voxel_grid.hpp>

namespace volume_grid
{

/*****************************************************************************/
SpatioTemporalVoxelGrid::SpatioTemporalVoxelGrid(const float& voxel_size, \
                   const double& background_value, const int& decay_model,\
                   const double& voxel_decay, const bool& pub_voxels) :
                   _background_value(background_value),                   \
                   _voxel_size(voxel_size),                               \
                   _decay_model(decay_model),                             \
                   _voxel_decay(voxel_decay),                             \
                   _pub_voxels(pub_voxels),                               \
                   _grid_points(new std::vector<geometry_msgs::Point32>),   \
                   _cost_map(new std::unordered_map<occupany_cell, uint>)
/*****************************************************************************/
{
  this->InitializeGrid();
}

/*****************************************************************************/
SpatioTemporalVoxelGrid::~SpatioTemporalVoxelGrid(void)
/*****************************************************************************/
{
  // pcl pointclouds free themselves
  if (_cost_map)
  {
    delete _cost_map;    
  }

  if (_grid_points)
  {
    delete _grid_points;
  }
}

/*****************************************************************************/
void SpatioTemporalVoxelGrid::InitializeGrid(void)
/*****************************************************************************/
{
  // initialize the OpenVDB Grid volume
  openvdb::initialize();

  // make it default to background value
  _grid = openvdb::DoubleGrid::create( _background_value );

  // setup scale and tranform
  openvdb::Mat4d m = openvdb::Mat4d::identity();
  m.preScale(openvdb::Vec3d(_voxel_size, _voxel_size, _voxel_size));
  m.preTranslate(openvdb::Vec3d(0, 0, 0));
  m.preRotate(openvdb::math::Z_AXIS, 0);

  // setup transform and other metadata
  _grid->setTransform(openvdb::math::Transform::createLinearTransform( m ));
  _grid->setName("SpatioTemporalVoxelLayer");
  _grid->insertMeta("Voxel Size", openvdb::FloatMetadata( _voxel_size ));
  _grid->setGridClass(openvdb::GRID_LEVEL_SET);

  return;
}

/*****************************************************************************/
void SpatioTemporalVoxelGrid::ClearFrustums( \
      const std::vector<observation::MeasurementReading>& clearing_readings, \
      std::unordered_set<occupany_cell>& cleared_cells)
/*****************************************************************************/
{
  boost::unique_lock<boost::mutex> lock(_grid_lock);

  // accelerate the decay of voxels interior to the frustum
  if(this->IsGridEmpty())
  {
    return;
  }

  _grid_points->clear();
  _cost_map->clear();

  std::vector<frustum_model> obs_frustums;

  if(clearing_readings.size() == 0)
  {
    TemporalClearAndGenerateCostmap(obs_frustums, cleared_cells);
    return;
  }

  obs_frustums.reserve(clearing_readings.size());

  std::vector<observation::MeasurementReading>::const_iterator it = \
                                                  clearing_readings.begin();
  for (it; it != clearing_readings.end(); ++it)
  {
    geometry::Frustum* frustum;
    if (it->_model_type == DEPTH_CAMERA)
    {
      frustum = new geometry::DepthCameraFrustum(it->_vertical_fov_in_rad,
                                                 it->_horizontal_fov_in_rad,
                                                 it->_min_z_in_m,
                                                 it->_max_z_in_m);
    }
    else if (it->_model_type == THREE_DIMENSIONAL_LIDAR)
    {
      frustum = new geometry::ThreeDimensionalLidarFrustum( \
                                                    it->_vertical_fov_in_rad,
                                                    it->_vertical_fov_padding_in_m,
                                                    it->_horizontal_fov_in_rad,
                                                    it->_min_z_in_m,
                                                    it->_max_z_in_m);
    }
    else
    {
      // add else if statement for each implemented model
      delete frustum;
      continue;
    }

    frustum->SetPosition(it->_origin);
    frustum->SetOrientation(it->_orientation);
    frustum->TransformModel();
    obs_frustums.emplace_back(frustum, it->_decay_acceleration);
  }
  TemporalClearAndGenerateCostmap(obs_frustums, cleared_cells);
  return;
}

/*****************************************************************************/
void SpatioTemporalVoxelGrid::TemporalClearAndGenerateCostmap(                \
                              std::vector<frustum_model>& frustums,           \
                              std::unordered_set<occupany_cell>& cleared_cells)
/*****************************************************************************/
{
  // sample time once for all clearing readings
  const double cur_time = ros::WallTime::now().toSec();

  // check each point in the grid for inclusion in a frustum
  openvdb::DoubleGrid::ValueOnCIter cit_grid = _grid->cbeginValueOn();
  for (cit_grid; cit_grid.test(); ++cit_grid)
  {
    const openvdb::Coord pt_index(cit_grid.getCoord());
    const openvdb::Vec3d pose_world = this->IndexToWorld(pt_index);

    std::vector<frustum_model>::iterator frustum_it = frustums.begin();
    bool frustum_cycle = false;
    bool cleared_point = false;

    const double time_since_marking = cur_time - cit_grid.getValue();
    const double base_duration_to_decay = GetTemporalClearingDuration( \
                                                            time_since_marking);

    for(frustum_it; frustum_it != frustums.end(); ++frustum_it)
    {
      if (!frustum_it->frustum)
      {
        continue;
      }

      if ( frustum_it->frustum->IsInside(pose_world) )
      {
        frustum_cycle = true;

        const double frustum_acceleration = GetFrustumAcceleration( \
                                  time_since_marking, frustum_it->accel_factor);

        const double time_until_decay = base_duration_to_decay - \
          frustum_acceleration;
        if (time_until_decay < 0.)
        {
          // expired by acceleration
          cleared_point = true;
          if(!this->ClearGridPoint(pt_index))
          {
            std::cout << "Failed to clear point." << std::endl;
          }
          break;
        }
        else
        {
          const double updated_mark = cit_grid.getValue() -
            frustum_acceleration;
          if(!this->MarkGridPoint(pt_index, updated_mark))
          {
            std::cout << "Failed to update mark." << std::endl;
          }
          break;
        }
      }
    }

    // if not inside any, check against nominal decay model
    if(!frustum_cycle)
    {
      if (base_duration_to_decay < 0.)
      {
        // expired by temporal clearing
        cleared_point = true;
        if(!this->ClearGridPoint(pt_index))
        {
          std::cout << "Failed to clear point." << std::endl;
        }
      }
    }
    
    if (cleared_point)
    {
      cleared_cells.insert(occupany_cell(pose_world[0], pose_world[1]));
    }
    else
    {
      // if here, we can add to costmap and PC2
      PopulateCostmapAndPointcloud(pt_index);
    }
  }
}

/*****************************************************************************/
void SpatioTemporalVoxelGrid::PopulateCostmapAndPointcloud(const \
                                                            openvdb::Coord& pt)
/*****************************************************************************/
{
  // add pt to the pointcloud and costmap
  openvdb::Vec3d pose_world = _grid->indexToWorld(pt);

  if (_pub_voxels)
  {
    geometry_msgs::Point32 point;
    point.x = pose_world[0];
    point.y = pose_world[1];
    point.z = pose_world[2];
    _grid_points->push_back(point);
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
void SpatioTemporalVoxelGrid::Mark(const \
                std::vector<observation::MeasurementReading>& marking_readings)
/*****************************************************************************/
{
  boost::unique_lock<boost::mutex> lock(_grid_lock);

  // mark the grid
  if (marking_readings.size() > 0)
  {
    //tbb::parallel_do(marking_readings, *this); /*must do via merged trees*/
    for (int i=0; i!= marking_readings.size(); i++)
    {
      (*this)(marking_readings.at(i));
    }
  }
  return;
}

/*****************************************************************************/
void SpatioTemporalVoxelGrid::operator()(const \
                                    observation::MeasurementReading& obs) const
/*****************************************************************************/
{
  if (obs._marking)
  {
    float mark_range_2 = obs._obstacle_range_in_m * obs._obstacle_range_in_m;
    const double cur_time = ros::WallTime::now().toSec();

    const sensor_msgs::PointCloud2& cloud = *(obs._cloud);
    sensor_msgs::PointCloud2ConstIterator<float> iter_x(cloud, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iter_y(cloud, "y");
    sensor_msgs::PointCloud2ConstIterator<float> iter_z(cloud, "z");

    for (iter_x, iter_y, iter_z; iter_x !=iter_x.end(); \
         ++iter_x, ++iter_y, ++iter_z)
    {
      float distance_2 = \
          (*iter_x - obs._origin.x) * (*iter_x - obs._origin.x) \
          + (*iter_y - obs._origin.y) * (*iter_y - obs._origin.y) \
          + (*iter_z - obs._origin.z) * (*iter_z - obs._origin.z);
      if (distance_2 > mark_range_2 || distance_2 < 0.0001)
      {
        continue;
      }
      openvdb::Vec3d mark_grid(this->WorldToIndex( \
                                 openvdb::Vec3d(*iter_x, *iter_y, *iter_z)));

      if(!this->MarkGridPoint(openvdb::Coord(mark_grid[0], mark_grid[1], \
                                             mark_grid[2]), cur_time))
      {
        std::cout << "Failed to mark point." << std::endl;
      }
    }
  }
  return;
}

/*****************************************************************************/
std::unordered_map<occupany_cell, uint>*
                                 SpatioTemporalVoxelGrid::GetFlattenedCostmap()
/*****************************************************************************/
{
  return _cost_map;
}

/*****************************************************************************/
double SpatioTemporalVoxelGrid::GetTemporalClearingDuration(const double& time_delta)
/*****************************************************************************/
{
  // use configurable model to get desired decay time
  if (_decay_model == 0) // linear
  {
    return _voxel_decay - time_delta;
  }
  else if (_decay_model == 1) // exponential
  {
    return _voxel_decay * std::exp(-time_delta);
  }
  return _voxel_decay; // PERSISTENT
}

/*****************************************************************************/
double SpatioTemporalVoxelGrid::GetFrustumAcceleration( \
                                             const double& time_delta, \
                                             const double& acceleration_factor)
/*****************************************************************************/
{
  const double acceleration = 1. / 6. * acceleration_factor * \
    (time_delta * time_delta * time_delta);
  return acceleration;
}

/*****************************************************************************/
void SpatioTemporalVoxelGrid::GetOccupancyPointCloud( \
                                            sensor_msgs::PointCloud2::Ptr& pc2)
/*****************************************************************************/
{
  // convert the grid points stored in a PointCloud2
  pc2->width  = _grid_points->size();
  pc2->height = 1;
  pc2->is_dense = true;

  sensor_msgs::PointCloud2Modifier modifier(*pc2);

  modifier.setPointCloud2Fields(3,
                                "x", 1, sensor_msgs::PointField::FLOAT32,
                                "y", 1, sensor_msgs::PointField::FLOAT32,
                                "z", 1, sensor_msgs::PointField::FLOAT32);
  modifier.setPointCloud2FieldsByString(1, "xyz");

  sensor_msgs::PointCloud2Iterator<float>iter_x(*pc2, "x");
  sensor_msgs::PointCloud2Iterator<float>iter_y(*pc2, "y");
  sensor_msgs::PointCloud2Iterator<float>iter_z(*pc2, "z");

  for(std::vector<geometry_msgs::Point32>::iterator it = _grid_points->begin(); \
      it != _grid_points->end(); ++it)
  {
    const geometry_msgs::Point32& pt = *it;
    *iter_x = pt.x;
    *iter_y = pt.y;
    *iter_z = pt.z;
    ++iter_x; ++iter_y; ++iter_z;
  }

  return;
}

/*****************************************************************************/
bool SpatioTemporalVoxelGrid::ResetGrid(void)
/*****************************************************************************/
{
  boost::unique_lock<boost::mutex> lock(_grid_lock);

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
bool SpatioTemporalVoxelGrid::MarkGridPoint(const openvdb::Coord& pt, \
                                                     const double& value) const
/*****************************************************************************/
{
  // marking the OpenVDB set
  openvdb::DoubleGrid::Accessor accessor = _grid->getAccessor();

  accessor.setValueOn(pt, value);
  return accessor.getValue(pt) == value;
}

/*****************************************************************************/
bool SpatioTemporalVoxelGrid::ClearGridPoint(const openvdb::Coord& pt) const
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
openvdb::Vec3d SpatioTemporalVoxelGrid::IndexToWorld(const \
                                                   openvdb::Coord& coord) const
/*****************************************************************************/
{
  // Applies tranform stored in getTransform.
  return _grid->indexToWorld(coord);
}

/*****************************************************************************/
openvdb::Vec3d SpatioTemporalVoxelGrid::WorldToIndex(const \
                                                     openvdb::Vec3d& vec) const
/*****************************************************************************/
{
  // Applies inverse tranform stored in getTransform.
  return _grid->worldToIndex(vec);
}

/*****************************************************************************/
bool SpatioTemporalVoxelGrid::IsGridEmpty(void) const
/*****************************************************************************/
{
  // Returns grid's population status
  return _grid->empty();
}

/*****************************************************************************/
bool SpatioTemporalVoxelGrid::SaveGrid(const std::string& file_name, \
                                                        double& map_size_bytes)
/*****************************************************************************/
{
  try
  {
    openvdb::io::File file(file_name + ".vdb");
    openvdb::GridPtrVec grids = { _grid };
    file.write(grids);
    file.close();
    map_size_bytes = _grid->memUsage();
    return true;
  }
  catch (...)
  {
    map_size_bytes = 0.;
    return false;
  }
  return false; // best offense is a good defense
}

}; // end namespace
