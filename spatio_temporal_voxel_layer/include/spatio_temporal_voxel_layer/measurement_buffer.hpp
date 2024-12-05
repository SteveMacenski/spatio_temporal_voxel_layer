/********************************************************************
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
 * Purpose: buffer incoming measurements for grid
 *********************************************************************/

#ifndef SPATIO_TEMPORAL_VOXEL_LAYER__MEASUREMENT_BUFFER_HPP_
#define SPATIO_TEMPORAL_VOXEL_LAYER__MEASUREMENT_BUFFER_HPP_

// STL
#include <vector>
#include <list>
#include <string>
#include <chrono>
#include <memory>
// measurement structs
#include "spatio_temporal_voxel_layer/measurement_reading.h"
// PCL
#include "pcl/common/transforms.h"
#include "pcl/filters/voxel_grid.h"
#include "pcl_conversions/pcl_conversions.h"
#include "pcl/filters/passthrough.h"
// ROS
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
// TF
#include "tf2_ros/buffer.h"
#include "message_filters/subscriber.hpp"
// msgs
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
// Mutex
#include "boost/thread.hpp"

namespace buffer
{

enum class Filters
{
  NONE = 0,
  VOXEL = 1,
  PASSTHROUGH = 2
};

// conveniences for line lengths
typedef std::list<observation::MeasurementReading>::iterator readings_iter;
typedef std::unique_ptr<sensor_msgs::msg::PointCloud2> point_cloud_ptr;

// Measurement buffer
class MeasurementBuffer
{
public:
  MeasurementBuffer(
    const std::string & source_name,
    const std::string & topic_name,
    const double & observation_keep_time,
    const double & expected_update_rate,
    const double & min_obstacle_height,
    const double & max_obstacle_height,
    const double & obstacle_range,
    tf2_ros::Buffer & tf,
    const std::string & global_frame,
    const std::string & sensor_frame,
    const double & tf_tolerance,
    const double & min_d,
    const double & max_d,
    const double & vFOV,
    const double & vFOVPadding,
    const double & hFOV,
    const double & decay_acceleration,
    const bool & marking,
    const bool & clearing,
    const double & voxel_size,
    const Filters & filter,
    const int & voxel_min_points,
    const bool & enabled,
    const bool & clear_buffer_after_reading,
    const ModelType & model_type,
    rclcpp::Clock::SharedPtr clock,
    rclcpp::Logger logger);

  ~MeasurementBuffer(void);

  // Buffers for different types of pointclouds
  void BufferROSCloud(const sensor_msgs::msg::PointCloud2 & cloud);

  // Get measurements from the buffer
  void GetReadings(std::vector<observation::MeasurementReading> & observations);

  // enabler setter getter
  bool IsEnabled(void) const;
  void SetEnabled(const bool & enabled);

  // Source name getter
  std::string GetSourceName(void) const;

  // params setters
  void SetMinObstacleHeight(const double & min_obstacle_height);
  void SetMaxObstacleHeight(const double & max_obstacle_height);
  void SetMinZ(const double & min_z);
  void SetMaxZ(const double & max_z);
  void SetVerticalFovPadding(const double & vertical_fov_padding);
  void SetHorizontalFovAngle(const double & horizontal_fov_angle);
  void SetVerticalFovAngle(const double & vertical_fov_angle);

  // State knoweldge if sensors are operating as expected
  bool UpdatedAtExpectedRate(void) const;
  void ResetLastUpdatedTime(void);
  void ResetAllMeasurements(void);
  bool ClearAfterReading(void);

  // Public mutex locks
  void Lock(void);
  void Unlock(void);

private:
  // Removing old observations from buffer
  void RemoveStaleObservations(void);

  tf2_ros::Buffer & _buffer;
  const rclcpp::Duration _observation_keep_time, _expected_update_rate;
  rclcpp::Time _last_updated;
  boost::recursive_mutex _lock;
  std::string _global_frame, _sensor_frame, _source_name, _topic_name;
  std::list<observation::MeasurementReading> _observation_list;
  double _min_obstacle_height, _max_obstacle_height, _obstacle_range, _tf_tolerance;
  double _min_z, _max_z, _vertical_fov, _vertical_fov_padding, _horizontal_fov;
  double _decay_acceleration, _voxel_size;
  bool _marking, _clearing;
  Filters _filter;
  int _voxel_min_points;
  bool _clear_buffer_after_reading, _enabled;
  ModelType _model_type;
  rclcpp::Clock::SharedPtr clock_;
  rclcpp::Logger logger_;
};

}  // namespace buffer

#endif  // SPATIO_TEMPORAL_VOXEL_LAYER__MEASUREMENT_BUFFER_HPP_
