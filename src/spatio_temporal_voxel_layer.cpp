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
  delete _level_set;
}

/*****************************************************************************/
void SpatioTemporalVoxelLayer::onInitialize(void)
/*****************************************************************************/
{
  ROS_INFO("%s being initialized as SpatioTemporalVoxelLayer!", getName().c_str());

  // initialize parameters, grid, and sub/pubs
  ros::NodeHandle nh("~/" + name_), g_nh, prefix_nh;

  _rolling_window = layered_costmap_->isRolling();
  _global_frame = std::string(layered_costmap_->getGlobalFrameID());

  bool track_unknown_space;
  double transform_tolerance;
  int background;
  std::string topics_string;
  nh.param("observation_sources", topics_string, std::string(""));
  nh.param("background_value", background, 0);
  nh.param("transform_tolerance", transform_tolerance, 0.2);
  nh.param("enabled", _enabled, true);
  nh.param("publish_voxel_map", _publish_voxels, true);
  nh.param("voxel_size", _voxel_size, 0.05);
  nh.param("track_unknown_space", track_unknown_space, layered_costmap_->isTrackingUnknown());

  ROS_DEBUG("%s: Rolling Window: %s", getName().c_str(), _rolling_window?"true":"false");
  ROS_DEBUG("%s: global frame: %s", getName().c_str(), _global_frame.c_str());
  ROS_DEBUG("%s: background_value: %i", getName().c_str(), background);
  ROS_DEBUG("%s: transform_tolerance: %f", getName().c_str(), transform_tolerance);
  ROS_DEBUG("%s: enabled: %s", getName().c_str(),  _enabled?"true":"false");
  ROS_DEBUG("%s: publish_voxel_map: %s", getName().c_str(), _voxel_size?"true":"false");
  ROS_DEBUG("%s: track_unknown_space: %s", getName().c_str(), track_unknown_space?"true":"false");
  ROS_DEBUG("%s: Subscribed to Topics %s", getName().c_str(), topics_string.c_str());

  if (track_unknown_space)
  {
    default_value_ = costmap_2d::NO_INFORMATION;
  } else {
    default_value_ = costmap_2d::FREE_SPACE;
  }

  if (_publish_voxels)
  {
    _voxel_pub = nh.advertise<sensor_msgs::PointCloud2>("voxel_grid" , 1);
  }

  _level_set = new LevelSet(_voxel_size, background, _rolling_window);
  matchSize();
  current_ = true;

  const std::string tf_prefix = tf::getPrefixParam(prefix_nh);
  std::stringstream ss(topics_string);
  std::string source;
  while (ss >> source)
  {
    ros::NodeHandle source_node(nh, source);

    // get the parameters for the specific topic
    double observation_keep_time, expected_update_rate, min_obstacle_height;
    double max_obstacle_height, min_z, max_z, vFOV, hFOV;
    std::string topic, sensor_frame, data_type;
    bool inf_is_valid, clearing, marking;

    source_node.param("topic", topic, source);
    source_node.param("sensor_frame", sensor_frame, std::string(""));
    source_node.param("voxel_decay", _voxel_decay, -1.); //temporal -1 means keep forever. 0. means most recent only, 20 means 20seconds
    source_node.param("voxel_decay_static", _voxel_decay, -1.); // decay when voxel is part of a connected component of static map
    source_node.param("observation_persistence", observation_keep_time, 0.0);
    source_node.param("expected_update_rate", expected_update_rate, 0.0);
    source_node.param("data_type", data_type, std::string("PointCloud2"));
    source_node.param("min_obstacle_height", min_obstacle_height, 0.0);
    source_node.param("max_obstacle_height", max_obstacle_height, 2.0);
    source_node.param("inf_is_valid", inf_is_valid, false);
    source_node.param("clearing", clearing, false);
    source_node.param("marking", marking, true);
    source_node.param("min_z", min_z, 0.); // minimum distance from camera it can see
    source_node.param("max_z", max_z, 10.); // maximum distance from camera it can see
    source_node.param("vertical_fov_angle", vFOV, 0.7); // vertical FOV angle in rad
    source_node.param("horizontal_fov_angle", hFOV, 1.04); // horizontal FOV angle in rad

    if (!sensor_frame.empty())
    {
     sensor_frame = tf::resolve(tf_prefix, sensor_frame);
    }

    if (!(data_type == "PointCloud2" || data_type == "LaserScan"))
    {
      ROS_FATAL("Only topics that use pointcloud2s or laser scans are supported.");
      throw std::runtime_error( \
                "Only topics that use pointclouds or laser scans are supported.");
    }

    std::string obstacle_range_param_name;

    // get the obstacle range for the sensor
    double obstacle_range = 3.0;
    if (source_node.searchParam("obstacle_range", obstacle_range_param_name))
    {
      source_node.getParam(obstacle_range_param_name, obstacle_range);
    }

    ROS_DEBUG("Creating an observation buffer for source %s, topic %s, frame %s", \
              source.c_str(), topic.c_str(), sensor_frame.c_str());

    // create an observation buffer
    _observation_buffers.push_back(
        boost::shared_ptr <buffer::MeasurementBuffer> 
        (new buffer::MeasurementBuffer(topic, observation_keep_time,     \
        expected_update_rate, min_obstacle_height, max_obstacle_height,  \
        obstacle_range, *tf_, _global_frame,                             \
        sensor_frame, transform_tolerance, min_z, max_z, vFOV, hFOV)));

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

    ROS_DEBUG(
      "Created an observation buffer for source %s, topic %s, global frame: %s, "
      "expected update rate: %.2f, observation persistence: %.2f",
      source.c_str(), topic.c_str(), _global_frame.c_str(), \
      expected_update_rate, observation_keep_time);

    // create a callback for the topic
    if (data_type == "LaserScan")
    {
      boost::shared_ptr < message_filters::Subscriber<sensor_msgs::LaserScan>
          > sub(new message_filters::Subscriber<sensor_msgs::LaserScan>(g_nh, topic, 50));
      _observation_subscribers.push_back(sub);

      boost::shared_ptr < tf::MessageFilter<sensor_msgs::LaserScan>
          > filter(new tf::MessageFilter<sensor_msgs::LaserScan>(*sub, *tf_, _global_frame, 50));

      if (inf_is_valid)
      {
        filter->registerCallback(
            boost::bind(&SpatioTemporalVoxelLayer::LaserScanValidInfCallback, this, _1, \
                                                           _observation_buffers.back()));
      } else {
        filter->registerCallback(
            boost::bind(&SpatioTemporalVoxelLayer::LaserScanCallback, this, _1, \
                                                    _observation_buffers.back()));
      }

      _observation_subscribers.push_back(sub);
      _observation_notifiers.push_back(filter);

      _observation_notifiers.back()->setTolerance(ros::Duration(0.05));
    } 

    else if (data_type == "PointCloud2")
    {
      boost::shared_ptr < message_filters::Subscriber<sensor_msgs::PointCloud2>
          > sub(new message_filters::Subscriber<sensor_msgs::PointCloud2>(g_nh, topic, 50));
      _observation_subscribers.push_back(sub);

      boost::shared_ptr < tf::MessageFilter<sensor_msgs::PointCloud2>
          > filter(new tf::MessageFilter<sensor_msgs::PointCloud2>(*sub, *tf_, _global_frame, 50));
      filter->registerCallback(
          boost::bind(&SpatioTemporalVoxelLayer::PointCloud2Callback, this, _1, \
                                                   _observation_buffers.back()));

      _observation_subscribers.push_back(sub);
      _observation_notifiers.push_back(filter);
    } 

    if (sensor_frame != "")
    {
      std::vector < std::string > target_frames;
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
    _laser_projector.transformLaserScanToPointCloud(message->header.frame_id, *message, cloud, *tf_);
  } catch (tf::TransformException &ex) {
    ROS_WARN("TF returned a transform exception to frame %s: %s", _global_frame.c_str(),
             ex.what());
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
    _laser_projector.transformLaserScanToPointCloud(message.header.frame_id, message, cloud, *tf_);
  } catch (tf::TransformException &ex) {
    ROS_WARN("TF returned a transform exception to frame %s: %s", _global_frame.c_str(), 
             ex.what());
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
bool SpatioTemporalVoxelLayer::updateFootprint(double robot_x, double robot_y,  \
                                               double robot_yaw, double* min_x, \
                                               double* min_y, double* max_x,    \
                                               double* max_y)
/*****************************************************************************/
{
  // updates layer costmap to include footprint for clearing in voxel grid
  if (!_update_footprint_enabled)
  {
    return false;
  }
  costmap_2d::transformFootprint(robot_x, robot_y, robot_yaw, getFootprint(), _transformed_footprint);
  for (unsigned int i = 0; i < _transformed_footprint.size(); i++)
  {
    touch(_transformed_footprint[i].x, _transformed_footprint[i].y, min_x, min_y, max_x, max_y);
  }
}

/*****************************************************************************/
void SpatioTemporalVoxelLayer::updateCosts( \
                                    costmap_2d::Costmap2D& master_grid, \
                                    int min_i, int min_j, int max_i, int max_j)
/*****************************************************************************/
{
  // update costs in master_grid with local costmap_ values updated from the VDB grid
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
    return;
  case 1:
    updateWithMax(master_grid, min_i, min_j, max_i, max_j);
    return;
  default:
    return;
  }
}

/*****************************************************************************/
void SpatioTemporalVoxelLayer::activate(void)
/*****************************************************************************/
{
  // subscribe and place info in buffers from sensor sources
  ROS_INFO("%s was activated.", getName().c_str());

  for (unsigned int i = 0; i < _observation_subscribers.size(); ++i)
  {
    if (_observation_subscribers[i] != NULL)
    {
      _observation_subscribers[i]->subscribe();
    }
  }

  for (unsigned int i = 0; i < _observation_buffers.size(); ++i)
  {
    if (_observation_buffers[i])
    {
      _observation_buffers[i]->ResetLastUpdatedTime();
    }
  }
}

/*****************************************************************************/
void SpatioTemporalVoxelLayer::deactivate(void)
/*****************************************************************************/
{
  // unsubscribe from all sensor sources
  ROS_INFO("%s was deactivated.", getName().c_str());

  for (unsigned int i = 0; i < _observation_subscribers.size(); ++i)
  {
    if (_observation_subscribers[i] != NULL)
    {
       _observation_subscribers[i]->unsubscribe();
    }
  }
}

/*****************************************************************************/
void SpatioTemporalVoxelLayer::reset(void)
/*****************************************************************************/
{
  // reset layer
  deactivate();
  this->resetMaps();
  current_ = true;
  activate();
}

/*****************************************************************************/
bool SpatioTemporalVoxelLayer::AddStaticObservations(const observation::MeasurementReading& obs)
/*****************************************************************************/
{
  // observations to always be added to the map each update cycle not explicitly marked on the map.
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
    ROS_WARN("Could not remove static observations from %s.", getName().c_str());
    return false;
  }
}

/*****************************************************************************/
void SpatioTemporalVoxelLayer::resetMaps(void)
/*****************************************************************************/
{
  // takes care of ROS 2D costmap
  Costmap2D::resetMaps();

  // takes care of our layer TODO, keeps doing:
  /*move_base: /usr/include/openvdb/tree/InternalNode.h:3110: void openvdb::v3_1::tree::InternalNode<_ChildNodeType, Log2Dim>::setChildNode(openvdb::v3_1::Index, openvdb::v3_1::tree::InternalNode<_ChildNodeType, Log2Dim>::ChildNodeType*) [with _ChildNodeType = openvdb::v3_1::tree::InternalNode<openvdb::v3_1::tree::LeafNode<int, 3u>, 4u>; unsigned int Log2Dim = 5u; openvdb::v3_1::Index = unsigned int; openvdb::v3_1::tree::InternalNode<_ChildNodeType, Log2Dim>::ChildNodeType = openvdb::v3_1::tree::InternalNode<openvdb::v3_1::tree::LeafNode<int, 3u>, 4u>]: Assertion `mChildMask.isOff(i)' failed.
*/
  //if (!_level_set->ResetLevelSet())
 // {
 //   ROS_WARN("Did not clear level set in %s!", getName().c_str());
  //}
}

/*****************************************************************************/
void SpatioTemporalVoxelLayer::matchSize(void)
/*****************************************************************************/
{
  // match the master costmap size

  //takes care of 2D ROS costmap
  CostmapLayer::matchSize(); 

  //takes care of our level set, probably dont need to do
  _level_set->ResizeLevelSet(layered_costmap_->getCostmap()->getSizeInCellsX(), \
                             layered_costmap_->getCostmap()->getSizeInCellsY(), \
                             layered_costmap_->getCostmap()->getResolution(),   \
                             layered_costmap_->getCostmap()->getOriginX(),      \
                             layered_costmap_->getCostmap()->getOriginY());
}

/*****************************************************************************/
void SpatioTemporalVoxelLayer::updateOrigin(double new_origin_x, double new_origin_y)
/*****************************************************************************/
{
  // takes care of 2D ROS costmap
  Costmap2D::updateOrigin(new_origin_x, new_origin_y);
}

/*****************************************************************************/
void SpatioTemporalVoxelLayer::updateBounds( \
                    double robot_x, double robot_y, double robot_yaw, \
                    double* min_x, double* min_y, double* max_x, double* max_y)
/*****************************************************************************/
{
  // update bounds of costmap update, ray trace freespace, and mark voxels
  if (_rolling_window)
  {
    updateOrigin(robot_x - getSizeInMetersX() / 2.0, robot_y - getSizeInMetersY() / 2.0);
  } 
  if (!_enabled)
  {
    return;
  }

  useExtraBounds(min_x, min_y, max_x, max_y);

  bool current = true;
  std::vector<observation::MeasurementReading> marking_observations, clearing_observations;
  current = GetMarkingObservations(marking_observations) && current;
  current = GetClearingObservations(clearing_observations) && current;
  current_ = current;

  // mark and clear observations
  _level_set->TemporallyClearFrustums(clearing_observations);
  _level_set->ParallelizeMark(marking_observations);

  // update the ROS Layered Costmap
  UpdateROSCostmap(*min_x, *min_y, *max_x, *max_y);

  // update footprint 
  updateFootprint(robot_x, robot_y, robot_yaw, min_x, min_y, max_x, max_y);

  // publish point cloud
  if (_publish_voxels)
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr pc(new pcl::PointCloud<pcl::PointXYZ>);
    _level_set->GetOccupancyPointCloud(pc);
    sensor_msgs::PointCloud2 pc2;
    pcl::toROSMsg(*pc, pc2);
    pc2.header.frame_id = std::string("map");
    pc2.header.stamp = ros::Time::now();
    _voxel_pub.publish(pc2);
  }
}

/*****************************************************************************/
void SpatioTemporalVoxelLayer::UpdateROSCostmap(double min_x, double min_y, \
                                                double max_x, double max_y)
/*****************************************************************************/
{
  // project 3D voxels to 2D, populate costmap_ and touch for updates
  std::vector<std::vector<int> > flattened_costmap(size_x_, std::vector<int>(size_y_, 0));
  _level_set->GetFlattenedCostmap(flattened_costmap, _mark_threshold); //return only incides of meaning TOOD ##
  Costmap2D::resetMaps(); //todo <-- why is this here? probably bad

  for (int i=0; i!= size_x_; i++) // then 1 for loop over useful stuff ##
  {
    for (int j=0; j!=size_y_; j++)
    {
      if ( flattened_costmap[i][j] > 0 ) //check for mark threahold here ##
      {
        openvdb::Coord pose_index(i,j,0);
        openvdb::Vec3d pose_world(_level_set->IndexToWorld(pose_index));

        unsigned int map_x, map_y;
        worldToMap(pose_world.x(), pose_world.y(), map_x, map_y);
        unsigned int index = getIndex(map_x, map_y);
        costmap_[index] = costmap_2d::LETHAL_OBSTACLE;
        touch(pose_world.x(), pose_world.x(), &min_x, &min_y, &max_x, &max_y);
      }
    }
  }
}

}; // end namespace

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(spatio_temporal_voxel_layer::SpatioTemporalVoxelLayer, costmap_2d::Layer);