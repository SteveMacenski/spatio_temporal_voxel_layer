/*********************************************************************
 *
 * Software License Agreement
 *
 *  Copyright (c) 2018, Simbe Robotics, Inc.
 *  Copyright (c) 2021, Samsung Research America
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
 *                         stevenmacenski@gmail.com
 * Purpose: Replace the ROS voxel grid / obstacle layers using VoxelGrid
 *          with OpenVDB's more efficient and capacble voxel library with
 *          ray tracing and knn.
 *********************************************************************/

#ifndef SPATIO_TEMPORAL_VOXEL_LAYER__SPATIO_TEMPORAL_VOXEL_LAYER_HPP_
#define SPATIO_TEMPORAL_VOXEL_LAYER__SPATIO_TEMPORAL_VOXEL_LAYER_HPP_

// STL
#include <time.h>
#include <vector>
#include <string>
#include <iostream>
#include <memory>
#include <unordered_set>
// voxel grid
#include "spatio_temporal_voxel_layer/spatio_temporal_voxel_grid.hpp"
// ROS
#include "rclcpp/rclcpp.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
// costmap
#include "nav2_costmap_2d/layer.hpp"
#include "nav2_costmap_2d/layered_costmap.hpp"
#include "nav2_costmap_2d/costmap_layer.hpp"
#include "nav2_costmap_2d/footprint.hpp"
// openVDB
#include "openvdb/openvdb.h"
// msgs
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "spatio_temporal_voxel_layer/srv/save_grid.hpp"
#include "std_srvs/srv/set_bool.hpp"
// projector
#include "laser_geometry/laser_geometry.hpp"
// tf
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/message_filter.h"
#include "message_filters/subscriber.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2/buffer_core.h"

namespace spatio_temporal_voxel_layer
{

// conveniences for line lengths
typedef std::vector<
  std::shared_ptr<message_filters::SubscriberBase<rclcpp_lifecycle::LifecycleNode>>
  >::iterator observation_subscribers_iter;
typedef std::vector<std::shared_ptr<buffer::MeasurementBuffer>>::iterator observation_buffers_iter;

// Core ROS voxel layer class
class SpatioTemporalVoxelLayer : public nav2_costmap_2d::CostmapLayer
{
public:
  SpatioTemporalVoxelLayer(void);
  virtual ~SpatioTemporalVoxelLayer(void);

  // Core Functions
  virtual void onInitialize(void);
  virtual void updateBounds(
    double robot_x, double robot_y, double robot_yaw,
    double * min_x, double * min_y, double * max_x, double * max_y);
  virtual void updateCosts(
    nav2_costmap_2d::Costmap2D & master_grid, int min_i, int min_j, int max_i, int max_j);

  // Functions to interact with other layers
  virtual void matchSize(void);

  // Functions for layer high level operations
  virtual void reset(void);
  virtual void activate(void);
  virtual void deactivate(void);
  virtual void clearArea(int start_x, int start_y, int end_x, int end_y, bool invert_area=false) override;

  virtual bool isClearable() {return true;}

  // Functions for sensor feeds
  bool GetMarkingObservations(std::vector<observation::MeasurementReading> & marking_observations)
  const;
  bool GetClearingObservations(std::vector<observation::MeasurementReading> & marking_observations)
  const;
  void ObservationsResetAfterReading() const;

  // Functions to interact with maps
  void UpdateROSCostmap(
    double * min_x, double * min_y, double * max_x, double * max_y,
    std::unordered_set<volume_grid::occupany_cell> & cleared_cells);
  bool updateFootprint(
    double robot_x, double robot_y, double robot_yaw,
    double * min_x, double * min_y, double * max_x, double * max_y);
  void ResetGrid(void);

  // Saving grids callback for openVDB
  void SaveGridCallback(
    const std::shared_ptr<rmw_request_id_t>/*header*/,
    std::shared_ptr<spatio_temporal_voxel_layer::srv::SaveGrid::Request> req,
    std::shared_ptr<spatio_temporal_voxel_layer::srv::SaveGrid::Response> resp);

private:
  // Sensor callbacks
  void LaserScanCallback(
    sensor_msgs::msg::LaserScan::ConstSharedPtr message,
    const std::shared_ptr<buffer::MeasurementBuffer> & buffer);
  void LaserScanValidInfCallback(
    sensor_msgs::msg::LaserScan::ConstSharedPtr raw_message,
    const std::shared_ptr<buffer::MeasurementBuffer> & buffer);
  void PointCloud2Callback(
    sensor_msgs::msg::PointCloud2::ConstSharedPtr message,
    const std::shared_ptr<buffer::MeasurementBuffer> & buffer);

  // Functions for adding static obstacle zones
  bool AddStaticObservations(const observation::MeasurementReading & obs);
  bool RemoveStaticObservations(void);

  // Enable/Disable callback
  void BufferEnablerCallback(const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
    std::shared_ptr<std_srvs::srv::SetBool::Response> response,
    const std::shared_ptr<buffer::MeasurementBuffer> buffer,
    const std::shared_ptr<message_filters::SubscriberBase<rclcpp_lifecycle::LifecycleNode>>
      & subcriber
    );

  /**
   * @brief Callback executed when a paramter change is detected
   * @param parameters list of changed parameters
   */
  rcl_interfaces::msg::SetParametersResult
    dynamicParametersCallback(std::vector<rclcpp::Parameter> parameters);

  laser_geometry::LaserProjection _laser_projector;
  std::vector<std::shared_ptr<message_filters::SubscriberBase<rclcpp_lifecycle::LifecycleNode>>>
    _observation_subscribers;
  std::vector<std::shared_ptr<tf2_ros::MessageFilterBase>> _observation_notifiers;
  std::vector<std::shared_ptr<buffer::MeasurementBuffer>> _observation_buffers;
  std::vector<std::shared_ptr<buffer::MeasurementBuffer>> _marking_buffers;
  std::vector<std::shared_ptr<buffer::MeasurementBuffer>> _clearing_buffers;
  std::vector<rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr> _buffer_enabler_servers;

  bool _publish_voxels, _mapping_mode, was_reset_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr _voxel_pub;
  rclcpp::Service<spatio_temporal_voxel_layer::srv::SaveGrid>::SharedPtr _grid_saver;
  std::unique_ptr<rclcpp::Duration> _map_save_duration;
  rclcpp::Time _last_map_save_time;
  std::string _global_frame;
  double _voxel_size, _voxel_decay;
  int _combination_method, _mark_threshold;
  volume_grid::GlobalDecayModel _decay_model;
  bool _update_footprint_enabled, _enabled;
  std::vector<geometry_msgs::msg::Point> _transformed_footprint;
  std::vector<observation::MeasurementReading> _static_observations;
  std::unique_ptr<volume_grid::SpatioTemporalVoxelGrid> _voxel_grid;
  boost::recursive_mutex _voxel_grid_lock;

  std::string _topics_string;

  // Dynamic parameters handler
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr dyn_params_handler;
};

}  // namespace spatio_temporal_voxel_layer
#endif  // SPATIO_TEMPORAL_VOXEL_LAYER__SPATIO_TEMPORAL_VOXEL_LAYER_HPP_
