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

#include "spatio_temporal_voxel_layer/spatio_temporal_voxel_layer.hpp"

namespace spatio_temporal_voxel_layer {

/*****************************************************************************/
SpatioTemporalVoxelLayer::SpatioTemporalVoxelLayer(void)
/*****************************************************************************/
{
}

/*****************************************************************************/
SpatioTemporalVoxelLayer::~SpatioTemporalVoxelLayer(void)
/*****************************************************************************/
{
  // I prefer to manage my own memory, I know others like std::unique_ptr
  if (_voxel_grid)
  {
    delete _voxel_grid;    
  }
}

/*****************************************************************************/
void SpatioTemporalVoxelLayer::onInitialize(void)
/*****************************************************************************/
{
  ROS_INFO("%s being initialized as SpatioTemporalVoxelLayer!", \
           getName().c_str());

  // initialize parameters, grid, and sub/pubs
  ros::NodeHandle nh("~/" + name_), g_nh, prefix_nh;

  _global_frame = std::string(layered_costmap_->getGlobalFrameID());
  ROS_INFO("%s's global frame is %s.", \
                                    getName().c_str(), _global_frame.c_str());

  bool track_unknown_space;
  double transform_tolerance, voxel_decay, map_save_time;
  std::string topics_string;
  int decay_model_int;
  // source names
  nh.param("observation_sources", topics_string, std::string(""));
  // timeout in seconds for transforms
  nh.param("transform_tolerance", transform_tolerance, 0.2);
  // whether to default on
  nh.param("enabled", _enabled, true);
  enabled_ = _enabled; // costmap_2d for some unexplicable reason uses globals
  // publish the voxel grid to visualize
  nh.param("publish_voxel_map", _publish_voxels, false);
  // size of each voxel in meters
  nh.param("voxel_size", _voxel_size, 0.05);
  // 1=takes highest in layers, 0=takes current layer
  nh.param("combination_method", _combination_method, 1);
  // number of voxels per vertical needed to have obstacle
  nh.param("mark_threshold", _mark_threshold, 0);
  // clear under robot footprint
  nh.param("update_footprint_enabled", _update_footprint_enabled, true);
  // keep tabs on unknown space
  nh.param("track_unknown_space", track_unknown_space, \
                                  layered_costmap_->isTrackingUnknown());
  nh.param("decay_model", decay_model_int, 0);
  volume_grid::GlobalDecayModel decay_model = \
                   static_cast<volume_grid::GlobalDecayModel>(decay_model_int);
  // decay param
  nh.param("voxel_decay", voxel_decay, -1.);
  // whether to map or navigate
  nh.param("mapping_mode", _mapping_mode, false);
  // if mapping, how often to save a map for safety
  nh.param("map_save_duration", map_save_time, 60.);
  ROS_INFO("%s loaded parameters from parameter server.", getName().c_str());

  if (_mapping_mode)
  {
    _map_save_duration = ros::Duration(map_save_time);
    _last_map_save_time = ros::Time::now() - _map_save_duration;
  }

  if (track_unknown_space)
  {
    default_value_ = costmap_2d::NO_INFORMATION;
  } else {
    default_value_ = costmap_2d::FREE_SPACE;
  }

  if (_publish_voxels)
  {
    _voxel_pub = nh.advertise<sensor_msgs::PointCloud2>("voxel_grid", 1);
  }
  _grid_saver = nh.advertiseService("spatiotemporal_voxel_grid/save_grid", \
                                 &SpatioTemporalVoxelLayer::SaveGridCallback, \
                                  this);

  _voxel_grid = new volume_grid::SpatioTemporalVoxelGrid(_voxel_size, \
                                                        (double)default_value_, \
                                                        decay_model, \
                                                        voxel_decay, \
                                                        _publish_voxels);
  matchSize();
  current_ = true;
  ROS_INFO("%s created underlying voxel grid.", getName().c_str());

  const std::string tf_prefix = tf::getPrefixParam(prefix_nh);
  std::stringstream ss(topics_string);
  std::string source;
  while (ss >> source)
  {
    ros::NodeHandle source_node(nh, source);

    // get the parameters for the specific topic
    double observation_keep_time, expected_update_rate, min_obstacle_height;
    double max_obstacle_height, min_z, max_z, vFOV, hFOV, decay_acceleration;
    std::string topic, sensor_frame, data_type;
    bool inf_is_valid, clearing, marking, voxel_filter, clear_after_reading;

    source_node.param("topic", topic, source);
    source_node.param("sensor_frame", sensor_frame, std::string(""));
    source_node.param("observation_persistence", observation_keep_time, 0.0);
    source_node.param("expected_update_rate", expected_update_rate, 0.0);
    source_node.param("data_type", data_type, std::string("PointCloud2"));
    source_node.param("min_obstacle_height", min_obstacle_height, 0.0);
    source_node.param("max_obstacle_height", max_obstacle_height, 3.0);
    source_node.param("inf_is_valid", inf_is_valid, false);
    source_node.param("clearing", clearing, false);
    source_node.param("marking", marking, true);
    // minimum distance from camera it can see
    source_node.param("min_z", min_z, 0.);
    // maximum distance from camera it can see
    source_node.param("max_z", max_z, 10.);
    // vertical FOV angle in rad
    source_node.param("vertical_fov_angle", vFOV, 0.7);
    // horizontal FOV angle in rad
    source_node.param("horizontal_fov_angle", hFOV, 1.04);
    // acceleration scales the model's decay in presence of readings
    source_node.param("decay_acceleration", decay_acceleration, 0.);
    // performs an approximate voxel filter over the data to reduce
    source_node.param("voxel_filter", voxel_filter, false);
    // clears measurement buffer after reading values from it
    source_node.param("clear_after_reading", clear_after_reading, false);
    // model type - default depth camera frustum model
    int model_type_int;
    source_node.param("model_type", model_type_int, 0);
    ModelType model_type = static_cast<ModelType>(model_type_int);

    if (!sensor_frame.empty())
    {
     sensor_frame = tf::resolve(tf_prefix, sensor_frame);
    }

    if (!(data_type == "PointCloud2" || data_type == "LaserScan"))
    {
      throw std::runtime_error( \
          "Only topics that use pointclouds or laser scans are supported.");
    }

    std::string obstacle_range_param_name;
    double obstacle_range = 3.0;
    if (source_node.searchParam("obstacle_range", obstacle_range_param_name))
    {
      source_node.getParam(obstacle_range_param_name, obstacle_range);
    }

    // create an observation buffer
    _observation_buffers.push_back(
        boost::shared_ptr <buffer::MeasurementBuffer>
        (new buffer::MeasurementBuffer(topic, observation_keep_time,     \
        expected_update_rate, min_obstacle_height, max_obstacle_height,  \
        obstacle_range, *tf_, _global_frame,                             \
        sensor_frame, transform_tolerance, min_z, max_z, vFOV,           \
        hFOV, decay_acceleration, marking, clearing, _voxel_size,        \
        voxel_filter, clear_after_reading, model_type)));

    // Add buffer to marking observation buffers
    if (marking == true)
    {
      _marking_buffers.push_back(_observation_buffers.back());
    }

    // Add buffer to clearing observation buffers
    if (clearing == true)
    {
      _clearing_buffers.push_back(_observation_buffers.back());
    }

    // create a callback for the topic
    if (data_type == "LaserScan")
    {
      boost::shared_ptr < message_filters::Subscriber<sensor_msgs::LaserScan>
          > sub(new message_filters::Subscriber<sensor_msgs::LaserScan>(g_nh, \
                                                                   topic, 50));
      _observation_subscribers.push_back(sub);

      boost::shared_ptr < tf::MessageFilter<sensor_msgs::LaserScan>
          > filter(new tf::MessageFilter<sensor_msgs::LaserScan>(*sub, \
                                                     *tf_, _global_frame, 50));

      if (inf_is_valid)
      {
        filter->registerCallback(
            boost::bind(&SpatioTemporalVoxelLayer::LaserScanValidInfCallback, \
                                        this, _1,_observation_buffers.back()));
      } else {
        filter->registerCallback(
            boost::bind(&SpatioTemporalVoxelLayer::LaserScanCallback, \
                                        this, _1, _observation_buffers.back()));
      }

      _observation_subscribers.push_back(sub);
      _observation_notifiers.push_back(filter);

      _observation_notifiers.back()->setTolerance(ros::Duration(0.05));
    }

    else if (data_type == "PointCloud2")
    {
      boost::shared_ptr < message_filters::Subscriber<sensor_msgs::PointCloud2>
          > sub(new message_filters::Subscriber<sensor_msgs::PointCloud2>(g_nh, \
                                                                     topic, 50));
      _observation_subscribers.push_back(sub);

      boost::shared_ptr < tf::MessageFilter<sensor_msgs::PointCloud2>
          > filter(new tf::MessageFilter<sensor_msgs::PointCloud2>(*sub, \
                                                  *tf_, _global_frame, 50));
      filter->registerCallback(
          boost::bind(&SpatioTemporalVoxelLayer::PointCloud2Callback, this, _1, \
                                                   _observation_buffers.back()));

      _observation_subscribers.push_back(sub);
      _observation_notifiers.push_back(filter);
    }

    if (sensor_frame != "")
    {
      std::vector<std::string> target_frames;
      target_frames.reserve(2);
      target_frames.push_back(_global_frame);
      target_frames.push_back(sensor_frame);
      _observation_notifiers.back()->setTargetFrames(target_frames);
    }
  }
  ROS_INFO("%s initialization complete!", getName().c_str());
}

/*****************************************************************************/
void SpatioTemporalVoxelLayer::LaserScanCallback( \
                const sensor_msgs::LaserScanConstPtr& message, \
                const boost::shared_ptr<buffer::MeasurementBuffer>& buffer)
/*****************************************************************************/
{
  // laser scan where infinity is invalid callback function
  sensor_msgs::PointCloud2 cloud;
  cloud.header = message->header;
  try
  {
    _laser_projector.transformLaserScanToPointCloud(\
                             message->header.frame_id, *message, cloud, *tf_);
  } catch (tf::TransformException &ex) {
    ROS_WARN("TF returned a transform exception to frame %s: %s", \
            _global_frame.c_str(), ex.what());
    _laser_projector.projectLaser(*message, cloud);
  }
  // buffer the point cloud
  buffer->Lock();
  buffer->BufferROSCloud(cloud);
  buffer->Unlock();
}

/*****************************************************************************/
void SpatioTemporalVoxelLayer::LaserScanValidInfCallback( \
                const sensor_msgs::LaserScanConstPtr& raw_message, \
                const boost::shared_ptr<buffer::MeasurementBuffer>& buffer)
/*****************************************************************************/
{
  // Filter infinity to max_range
  float epsilon = 0.0001;
  sensor_msgs::LaserScan message = *raw_message;
  for (size_t i = 0; i < message.ranges.size(); i++)
  {
    float range = message.ranges[i];
    if (!std::isfinite(range) && range > 0)
    {
      message.ranges[i] = message.range_max - epsilon;
    }
  }
  sensor_msgs::PointCloud2 cloud;
  cloud.header = message.header;
  try {
    _laser_projector.transformLaserScanToPointCloud( \
                              message.header.frame_id, message, cloud, *tf_);
  } catch (tf::TransformException &ex) {
    ROS_WARN("TF returned a transform exception to frame %s: %s", \
             _global_frame.c_str(), ex.what());
    _laser_projector.projectLaser(message, cloud);
  }
  // buffer the point cloud
  buffer->Lock();
  buffer->BufferROSCloud(cloud);
  buffer->Unlock();
}

/*****************************************************************************/
void SpatioTemporalVoxelLayer::PointCloud2Callback( \
                const sensor_msgs::PointCloud2ConstPtr& message, \
                const boost::shared_ptr<buffer::MeasurementBuffer>& buffer)
/*****************************************************************************/
{
  // buffer the point cloud
  buffer->Lock();
  buffer->BufferROSCloud(*message);
  buffer->Unlock();
}

/*****************************************************************************/
bool SpatioTemporalVoxelLayer::GetMarkingObservations( \
      std::vector<observation::MeasurementReading>& marking_observations) const
/*****************************************************************************/
{
  // get marking observations and static marked areas
  bool current = true;

  for (unsigned int i=0; i!=_marking_buffers.size(); ++i) //1
  {
    _marking_buffers[i]->Lock();
    _marking_buffers[i]->GetReadings(marking_observations);
    current = _marking_buffers[i]->UpdatedAtExpectedRate();
    _marking_buffers[i]->Unlock();
  }
  marking_observations.insert(marking_observations.end(),   \
                              _static_observations.begin(), \
                              _static_observations.end());
  return current;
}

/*****************************************************************************/
bool SpatioTemporalVoxelLayer::GetClearingObservations( \
     std::vector<observation::MeasurementReading>& clearing_observations) const
/*****************************************************************************/
{
  // get clearing observations
  bool current = true;
  for (unsigned int i = 0; i < _clearing_buffers.size(); ++i)
  {
    _clearing_buffers[i]->Lock();
    _clearing_buffers[i]->GetReadings(clearing_observations);
    current = _clearing_buffers[i]->UpdatedAtExpectedRate();
    _clearing_buffers[i]->Unlock();
  }
  return current;
}

/*****************************************************************************/
bool SpatioTemporalVoxelLayer::updateFootprint(double robot_x, double robot_y, \
                                               double robot_yaw, double* min_x,\
                                               double* min_y, double* max_x,   \
                                               double* max_y)
/*****************************************************************************/
{
  // updates layer costmap to include footprint for clearing in voxel grid
  if (!_update_footprint_enabled)
  {
    return false;
  }
  costmap_2d::transformFootprint(robot_x, robot_y, robot_yaw, getFootprint(), \
                                 _transformed_footprint);
  for (unsigned int i = 0; i < _transformed_footprint.size(); i++)
  {
    touch(_transformed_footprint[i].x, _transformed_footprint[i].y, \
          min_x, min_y, max_x, max_y);
  }
}

/*****************************************************************************/
void SpatioTemporalVoxelLayer::activate(void)
/*****************************************************************************/
{
  // subscribe and place info in buffers from sensor sources
  ROS_INFO("%s was activated.", getName().c_str());

  observation_subscribers_iter sub_it = _observation_subscribers.begin();
  for (sub_it; sub_it != _observation_subscribers.end(); ++sub_it)
  {
    (*sub_it)->subscribe();
  }

  observation_buffers_iter buf_it = _observation_buffers.begin();
  for (buf_it; buf_it != _observation_buffers.end(); ++buf_it)
  {
    (*buf_it)->ResetLastUpdatedTime();
  }
}

/*****************************************************************************/
void SpatioTemporalVoxelLayer::deactivate(void)
/*****************************************************************************/
{
  // unsubscribe from all sensor sources
  ROS_INFO("%s was deactivated.", getName().c_str());

  observation_subscribers_iter sub_it = _observation_subscribers.begin();
  for (sub_it; sub_it != _observation_subscribers.end(); ++sub_it)
  {
    if (*sub_it != NULL)
    {
      (*sub_it)->unsubscribe();
    }
  }
}

/*****************************************************************************/
void SpatioTemporalVoxelLayer::reset(void)
/*****************************************************************************/
{
  // reset layer
  Costmap2D::resetMaps();
  this->ResetGrid();
  current_ = true;
  observation_buffers_iter it = _observation_buffers.begin();
  for (it; it != _observation_buffers.end(); ++it)
  {
    (*it)->ResetLastUpdatedTime();
  }
}

/*****************************************************************************/
bool SpatioTemporalVoxelLayer::AddStaticObservations( \
                                    const observation::MeasurementReading& obs)
/*****************************************************************************/
{
  // observations to always be added to the map each update cycle not marked
  ROS_INFO("%s: Adding static observation to map.", getName().c_str());

  try {
    _static_observations.push_back(obs);
    return true;
  } catch(...) {
    ROS_WARN("Could not add static observations to voxel layer");
    return false;
  }
}

/*****************************************************************************/
bool SpatioTemporalVoxelLayer::RemoveStaticObservations(void)
/*****************************************************************************/
{
  // kill all static observations added to each update cycle
  ROS_INFO("%s: Removing static observations to map.", getName().c_str());

  try {
    _static_observations.clear();
    return true;
  } catch(...) {
    ROS_WARN("Couldn't remove static observations from %s.", \
             getName().c_str());
    return false;
  }
}

/*****************************************************************************/
void SpatioTemporalVoxelLayer::ResetGrid(void)
/*****************************************************************************/
{
  if (!_voxel_grid->ResetGrid())
  {
   ROS_WARN("Did not clear level set in %s!", getName().c_str());
  }
}

/*****************************************************************************/
void SpatioTemporalVoxelLayer::matchSize(void)
/*****************************************************************************/
{
  // match the master costmap size, volume_grid maintains full w/ expiration.
  CostmapLayer::matchSize();
}

/*****************************************************************************/
void SpatioTemporalVoxelLayer::updateCosts( \
                                    costmap_2d::Costmap2D& master_grid, \
                                    int min_i, int min_j, int max_i, int max_j)
/*****************************************************************************/
{
  // update costs in master_grid with costmap_
  if(!_enabled)
  {
    return;
  }

  if (_update_footprint_enabled)
  {
    setConvexPolygonCost(_transformed_footprint, costmap_2d::FREE_SPACE);
  }

  switch (_combination_method)
  {
  case 0:
    updateWithOverwrite(master_grid, min_i, min_j, max_i, max_j);
  case 1:
    updateWithMax(master_grid, min_i, min_j, max_i, max_j);
  default:
    break;
  }
  return;
}

/*****************************************************************************/
void SpatioTemporalVoxelLayer::UpdateROSCostmap(double* min_x, double* min_y, \
                                                double* max_x, double* max_y)
/*****************************************************************************/
{
  // grabs map of occupied cells from grid and adds to costmap_
  Costmap2D::resetMaps();

  std::unordered_map<volume_grid::occupany_cell, uint>::iterator it;
  for (it = _voxel_grid->GetFlattenedCostmap()->begin();
       it != _voxel_grid->GetFlattenedCostmap()->end(); ++it)
  {
    uint map_x, map_y;
    if ( it->second >= _mark_threshold && \
         worldToMap(it->first.x, it->first.y, map_x, map_y))
    {
      costmap_[getIndex(map_x, map_y)] = costmap_2d::LETHAL_OBSTACLE;
      touch(it->first.x, it->first.y, min_x, min_y, max_x, max_y);
    }
  }
}

/*****************************************************************************/
void SpatioTemporalVoxelLayer::updateBounds( \
                    double robot_x, double robot_y, double robot_yaw, \
                    double* min_x, double* min_y, double* max_x, double* max_y)
/*****************************************************************************/
{
  // grabs new max bounds for the costmap
  if (!_enabled)
  {
    return;
  }

  // Steve's Note June 22, 2018
  // I dislike this necessity, I can't remove the master grid's knowledge about
  // STVL on the fly so I have play games with the API even though this isn't
  // really a rolling plugin implementation. It works, but isn't ideal.
  if (layered_costmap_->isRolling())
  {
    updateOrigin(robot_x-getSizeInMetersX()/2, robot_y-getSizeInMetersY()/2);
  }

  useExtraBounds(min_x, min_y, max_x, max_y);

  bool current = true;
  std::vector<observation::MeasurementReading> marking_observations, \
                                               clearing_observations;
  current = GetMarkingObservations(marking_observations) && current;
  current = GetClearingObservations(clearing_observations) && current;
  current_ = current;

  // navigation mode: clear observations, mapping mode: save maps and publish
  if (!_mapping_mode)
  {
    _voxel_grid->ClearFrustums(clearing_observations);
  }
  else if (ros::Time::now() - _last_map_save_time > _map_save_duration)
  {
    _last_map_save_time = ros::Time::now();
    time_t rawtime;
    struct tm* timeinfo;
    char time_buffer[100];
    time (&rawtime);
    timeinfo = localtime (&rawtime);
    strftime(time_buffer, 100, "%F-%r", timeinfo);

    spatio_temporal_voxel_layer::SaveGrid srv;
    srv.request.file_name.data = time_buffer;
    SaveGridCallback(srv.request, srv.response);
  }

  // mark observations
  _voxel_grid->Mark(marking_observations);

  // update the ROS Layered Costmap
  UpdateROSCostmap(min_x, min_y, max_x, max_y);

  // publish point cloud in navigation mode
  if (_publish_voxels && !_mapping_mode)
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr pc(new pcl::PointCloud<pcl::PointXYZ>);
    _voxel_grid->GetOccupancyPointCloud(pc);
    sensor_msgs::PointCloud2 pc2;
    pcl::toROSMsg(*pc, pc2);
    pc2.header.frame_id = _global_frame;
    pc2.header.stamp = ros::Time::now();
    _voxel_pub.publish(pc2);
  }

  // update footprint
  updateFootprint(robot_x, robot_y, robot_yaw, min_x, min_y, max_x, max_y);
  return;
}

/*****************************************************************************/
bool SpatioTemporalVoxelLayer::SaveGridCallback( \
                         spatio_temporal_voxel_layer::SaveGrid::Request& req, \
                         spatio_temporal_voxel_layer::SaveGrid::Response& resp)
/*****************************************************************************/
{
  double map_size_bytes;
  if( _voxel_grid->SaveGrid(req.file_name.data, map_size_bytes) )
  {
    ROS_INFO( \
      "SpatioTemporalVoxelGrid: Saved %s grid! Has memory footprint of %f bytes.",
      req.file_name.data.c_str(), map_size_bytes);
    resp.map_size_bytes = map_size_bytes;
    resp.status = true;
    return true;
  }
  ROS_WARN("SpatioTemporalVoxelGrid: Failed to save grid.");
  resp.status = false;
  return false;
}

}; // end namespace

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(spatio_temporal_voxel_layer::SpatioTemporalVoxelLayer, costmap_2d::Layer);
