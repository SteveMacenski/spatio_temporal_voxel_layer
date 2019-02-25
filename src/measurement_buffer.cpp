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

#include <spatio_temporal_voxel_layer/measurement_buffer.hpp>

namespace buffer
{

/*****************************************************************************/
MeasurementBuffer::MeasurementBuffer(const std::string& topic_name,       \
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
                                     const bool& clear_buffer_after_reading) :
/*****************************************************************************/
    _buffer(tf), _observation_keep_time(observation_keep_time),
    _expected_update_rate(expected_update_rate),_last_updated(ros::Time::now()), 
    _global_frame(global_frame), _sensor_frame(sensor_frame),
    _topic_name(topic_name), _min_obstacle_height(min_obstacle_height), 
    _max_obstacle_height(max_obstacle_height), _obstacle_range(obstacle_range),
    _tf_tolerance(tf_tolerance), _min_z(min_d), _max_z(max_d), 
    _vertical_fov(vFOV), _horizontal_fov(hFOV),
    _decay_acceleration(decay_acceleration), _marking(marking),
    _clearing(clearing), _voxel_size(voxel_size), _voxel_filter(voxel_filter),
    _clear_buffer_after_reading(clear_buffer_after_reading)
{
}

/*****************************************************************************/
MeasurementBuffer::~MeasurementBuffer(void)
/*****************************************************************************/
{
}

/*****************************************************************************/
void MeasurementBuffer::BufferROSCloud(const sensor_msgs::PointCloud2& cloud)
/*****************************************************************************/
{
  tryspatio_temporal_voxel_layer
  {
    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL(cloud, pcl_pc2);
    pcl::PointCloud < pcl::PointXYZ > pcl_cloud;
    pcl::fromPCLPointCloud2(pcl_pc2, pcl_cloud);
    BufferPCLCloud(pcl_cloud);
  }
  catch (pcl::PCLException& ex)
  {
    ROS_ERROR("Failed to convert to pcl type, dropping observation: %s", \
              ex.what());
    return;
  }
}

/*****************************************************************************/
void MeasurementBuffer::BufferPCLCloud(const \
                                         pcl::PointCloud<pcl::PointXYZ>& cloud)
/*****************************************************************************/
{
  // add a new measurement to be populated
  _observation_list.push_front(observation::MeasurementReading());

  const std::string origin_frame = \
                  _sensor_frame == "" ? cloud.header.frame_id : _sensor_frame;

  try
  {
    // transform into global frame
    geometry_msgs::PoseStamped  local_pose, global_pose;
    local_pose.pose.position.x=0;
    local_pose.pose.position.y=0;
    local_pose.pose.position.z=0;
    local_pose.pose.orientation.x=0;
    local_pose.pose.orientation.y=0;
    local_pose.pose.orientation.z=0;
    local_pose.pose.orientation.w=1;
    local_pose.header.stamp = pcl_conversions::fromPCL(cloud.header).stamp;
    local_pose.header.frame_id = origin_frame;

    _buffer.canTransform(_global_frame, local_pose.header.frame_id , \
                         local_pose.header.stamp, ros::Duration(0.5));
    _buffer.transform(local_pose, global_pose, _global_frame);

    _observation_list.front()._origin.x = global_pose.pose.position.x;
    _observation_list.front()._origin.y = global_pose.pose.position.y;
    _observation_list.front()._origin.z = global_pose.pose.position.z;

    _observation_list.front()._orientation = global_pose.pose.orientation;
    _observation_list.front()._obstacle_range_in_m = _obstacle_range;
    _observation_list.front()._min_z_in_m = _min_z;
    _observation_list.front()._max_z_in_m = _max_z;
    _observation_list.front()._vertical_fov_in_rad = _vertical_fov;
    _observation_list.front()._horizontal_fov_in_rad = _horizontal_fov;
    _observation_list.front()._decay_acceleration = _decay_acceleration;
    _observation_list.front()._clearing = _clearing;
    _observation_list.front()._marking = _marking;

    if (_clearing && !_marking)
    {
      // no need to buffer points
      return;
    }

    point_cloud_ptr cld_global(new pcl::PointCloud<pcl::PointXYZ>);
    geometry_msgs::TransformStamped tf_stamped = _buffer.lookupTransform(_global_frame, cloud.header.frame_id.c_str(), pcl_conversions::fromPCL(cloud.header).stamp);
    tf::StampedTransform tf_transform;
    tf::transformMsgToTF(tf_stamped.transform, tf_transform);
    tf_transform.frame_id_ = _global_frame;
    tf_transform.stamp_ = tf_stamped.header.stamp;
    pcl_ros::transformPointCloud(cloud, *cld_global, tf_transform);
    cld_global->header.stamp = cloud.header.stamp;

    // if user wants to use a voxel filter
    if ( _voxel_filter )
    {
      // remove nans because they show things down
      point_cloud_ptr cld_no_nan(new pcl::PointCloud<pcl::PointXYZ>);
      std::vector<int> indices;
      pcl::removeNaNFromPointCloud(*cld_global, *cld_no_nan, indices);

      // minimize information needed to process
      cld_global->clear();
      pcl::ApproximateVoxelGrid<pcl::PointXYZ> sor1;
      sor1.setInputCloud (cld_no_nan);
      sor1.setLeafSize ((float)_voxel_size,
                        (float)_voxel_size,
                        (float)_voxel_size);
      sor1.filter (*cld_global);
    }

    // remove points that are below or above our height restrictions
    pcl::PointCloud<pcl::PointXYZ>& obs_cloud = \
                                *(_observation_list.front()._cloud);
    unsigned int cloud_size = cld_global->points.size();

    obs_cloud.points.resize(cloud_size);
    unsigned int point_count = 0;
    pcl::PointCloud<pcl::PointXYZ>::iterator it;
    for (it = cld_global->begin(); it != cld_global->end(); ++it)
    {
      if (it->z <= _max_obstacle_height && it->z >= _min_obstacle_height)
      {
        obs_cloud.points.at(point_count++) = *it;
      }
    }

    // resize the cloud for the number of legal points
    obs_cloud.points.resize(point_count);
    obs_cloud.header.stamp = cloud.header.stamp;
    obs_cloud.header.frame_id = cld_global->header.frame_id;
  }
  catch (tf::TransformException& ex)
  {
    // if fails, remove the empty observation
    _observation_list.pop_front();
    ROS_ERROR( \
      "TF Exception for sensor frame: %s, cloud frame: %s, %s", \
      _sensor_frame.c_str(), cloud.header.frame_id.c_str(), ex.what());
    return;
  }

  _last_updated = ros::Time::now();
  RemoveStaleObservations();
}

/*****************************************************************************/
void MeasurementBuffer::GetReadings( \
                    std::vector<observation::MeasurementReading>& observations)
/*****************************************************************************/
{
  RemoveStaleObservations();

  for (readings_iter it = _observation_list.begin(); \
                                          it != _observation_list.end(); ++it)
  {
    observations.push_back(*it);
  }

  // clear from buffer so we don't over report
  if (_clear_buffer_after_reading || _observation_keep_time.toSec() == 0.)
  {
    _observation_list.clear();
  }
}

/*****************************************************************************/
void MeasurementBuffer::RemoveStaleObservations(void)
/*****************************************************************************/
{
  if (_observation_list.empty())
  {
    return;
  }

  readings_iter it = _observation_list.begin();
  if (_observation_keep_time == ros::Duration(0.0))
  {
    _observation_list.erase(++it, _observation_list.end());
    return;
  }

  for (it = _observation_list.begin(); it != _observation_list.end(); ++it)
  {
    observation::MeasurementReading& obs = *it;
    const ros::Duration time_diff = \
            _last_updated - pcl_conversions::fromPCL(obs._cloud->header).stamp;

    if (time_diff > _observation_keep_time)
    {
      _observation_list.erase(it, _observation_list.end());
      return;
    }
  }
}

/*****************************************************************************/
bool MeasurementBuffer::UpdatedAtExpectedRate(void) const
/*****************************************************************************/
{
  if (_expected_update_rate == ros::Duration(0.0))
  {
    return true;
  }

  const ros::Duration update_time = ros::Time::now() - _last_updated;
  const bool current = update_time.toSec() <= _expected_update_rate.toSec();
  if (!current)
  {
    ROS_WARN_THROTTLE(10.,
      "%s buffer updated in %.2fs, it should be updated every %.2fs.",
      _topic_name.c_str(), update_time.toSec(), _expected_update_rate.toSec());
  }
  return current;
}

/*****************************************************************************/
void MeasurementBuffer::ResetLastUpdatedTime(void)
/*****************************************************************************/
{
  _last_updated = ros::Time::now();
}

/*****************************************************************************/
void MeasurementBuffer::Lock(void)
/*****************************************************************************/
{
  _lock.lock();
}

/*****************************************************************************/
void MeasurementBuffer::Unlock(void)
/*****************************************************************************/
{
  _lock.unlock();
}

}  // namespace buffer

