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

#ifndef MEASUREMENT_BUFFER_H_
#define MEASUREMENT_BUFFER_H_

// measurement structs
#include <spatio_temporal_voxel_layer/measurement_reading.h>
// PCL
#include <pcl_ros/transforms.h>
#include <pcl/filters/approximate_voxel_grid.h>
// STL
#include <vector>
#include <list>
#include <string>
// ROS
#include <ros/ros.h>
#include <ros/time.h>
// TF
#include "tf2_ros/transform_listener.h"
#include "tf/transform_listener.h"
#include "tf2_ros/message_filter.h"
#include <tf2_ros/buffer.h>
#include "message_filters/subscriber.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <tf2/transform_datatypes.h>
// msgs
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseStamped.h>
// Mutex
#include <boost/thread.hpp>

namespace buffer
{

// conveniences for line lengths
typedef std::list<observation::MeasurementReading>::iterator readings_iter;
typedef pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud_ptr;

// Measurement buffer
class MeasurementBuffer
{
public:
  MeasurementBuffer(const std::string& topic_name,       \
                    const double& observation_keep_time, \
                    const double& expected_update_rate,  \
                    const double& min_obstacle_height,   \
                    const double& max_obstacle_height,   \
                    const double& obstacle_range,        \
                    tf2_ros::Buffer& tf,                 \
                    const std::string& global_frame,     \
                    const std::string& sensor_frame,     \
                    const double& tf_tolerance,          \
                    const double& min_d,                 \
                    const double& max_d,                 \
                    const double& vFOV,                  \
                    const double& hFOV,                  \
                    const double& decay_acceleration,    \
                    const bool& marking,                 \
                    const bool& clearing,                \
                    const double& voxel_size,            \
                    const bool& voxel_filter,            \
                    const bool& clear_buffer_after_reading);

  ~MeasurementBuffer(void);

  // Buffers for different types of pointclouds
  void BufferROSCloud(const sensor_msgs::PointCloud2& cloud);
  void BufferPCLCloud(const pcl::PointCloud<pcl::PointXYZ>& cloud);

  // Get measurements from the buffer
  void GetReadings(std::vector<observation::MeasurementReading>& observations);

  // State knoweldge if sensors are operating as expected
  bool UpdatedAtExpectedRate(void) const;
  void ResetLastUpdatedTime(void);

  // Public mutex locks
  void Lock(void);
  void Unlock(void);

private:
  // Removing old observations from buffer
  void RemoveStaleObservations(void);

  tf2_ros::Buffer& _buffer;
  const ros::Duration _observation_keep_time, _expected_update_rate;
  boost::recursive_mutex _lock;
  ros::Time _last_updated;
  std::string _global_frame, _topic_name, _sensor_frame;
  std::list<observation::MeasurementReading> _observation_list;
  double _min_obstacle_height, _max_obstacle_height, _obstacle_range, _tf_tolerance;
  double _min_z, _max_z, _vertical_fov, _horizontal_fov, _decay_acceleration;
  double _voxel_size;
  bool _marking, _clearing, _voxel_filter, _clear_buffer_after_reading;
  geometry_msgs::TransformStamped tf_stamped;
};

} // namespace buffer

#endif // MEASUREMENT_BUFFER_H_
