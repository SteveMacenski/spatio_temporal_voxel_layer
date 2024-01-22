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
 * Purpose: Measurement reading structure containing info needed for marking
 *          and frustum clearing.
 *********************************************************************/

#ifndef SPATIO_TEMPORAL_VOXEL_LAYER__MEASUREMENT_READING_H_
#define SPATIO_TEMPORAL_VOXEL_LAYER__MEASUREMENT_READING_H_

#include <memory>

// msgs
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

enum ModelType
{
  DEPTH_CAMERA = 0,
  THREE_DIMENSIONAL_LIDAR = 1
};

namespace observation
{

// Measurement Reading
struct MeasurementReading
{
  /*****************************************************************************/
  MeasurementReading()
  /*****************************************************************************/
    : _cloud(std::make_shared < sensor_msgs::msg::PointCloud2 > ())
  {
  }

  /*****************************************************************************/
  MeasurementReading(
    geometry_msgs::msg::Point & origin, sensor_msgs::msg::PointCloud2 cloud,
    double obstacle_range, double min_z, double max_z, double vFOV,
    double vFOVPadding, double hFOV, double decay_acceleration, bool marking,
    bool clearing, ModelType model_type)
  /*****************************************************************************/
    : _origin(origin),
    _cloud(std::make_shared < sensor_msgs::msg::PointCloud2 > (cloud)),
    _obstacle_range_in_m(obstacle_range),
    _min_z_in_m(min_z),
    _max_z_in_m(max_z),
    _vertical_fov_in_rad(vFOV),
    _vertical_fov_padding_in_m(vFOVPadding),
    _horizontal_fov_in_rad(hFOV),
    _marking(marking),
    _clearing(clearing),
    _decay_acceleration(decay_acceleration),
    _model_type(model_type)
  {
  }

  /*****************************************************************************/
  MeasurementReading(sensor_msgs::msg::PointCloud2 cloud, double obstacle_range)
  /*****************************************************************************/
    : _cloud(std::make_shared < sensor_msgs::msg::PointCloud2 > (cloud)),
    _obstacle_range_in_m(obstacle_range)
  {
  }

  /*****************************************************************************/
  MeasurementReading(const MeasurementReading & obs)
  /*****************************************************************************/
    : _origin(obs._origin),
    _orientation(obs._orientation),
    _cloud(std::make_shared < sensor_msgs::msg::PointCloud2 > (*(obs._cloud))),
    _obstacle_range_in_m(obs._obstacle_range_in_m),
    _min_z_in_m(obs._min_z_in_m),
    _max_z_in_m(obs._max_z_in_m),
    _vertical_fov_in_rad(obs._vertical_fov_in_rad),
    _vertical_fov_padding_in_m(obs._vertical_fov_padding_in_m),
    _horizontal_fov_in_rad(obs._horizontal_fov_in_rad),
    _marking(obs._marking),
    _clearing(obs._clearing),
    _decay_acceleration(obs._decay_acceleration),
    _model_type(obs._model_type)
  {
  }

  geometry_msgs::msg::Point _origin;
  geometry_msgs::msg::Quaternion _orientation;
  std::shared_ptr < sensor_msgs::msg::PointCloud2 > _cloud;
  double _obstacle_range_in_m, _min_z_in_m, _max_z_in_m;
  double _vertical_fov_in_rad, _vertical_fov_padding_in_m, _horizontal_fov_in_rad;
  double _marking, _clearing, _decay_acceleration;
  ModelType _model_type;
};

}  // namespace observation

#endif  // SPATIO_TEMPORAL_VOXEL_LAYER__MEASUREMENT_READING_H_
