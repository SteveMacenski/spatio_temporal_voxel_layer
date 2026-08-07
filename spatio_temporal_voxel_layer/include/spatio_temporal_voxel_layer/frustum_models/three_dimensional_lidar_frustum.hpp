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
 * Author: Nicolas Varas (nicovaras@gmail.com)
 * Purpose: Structure for handling 3D Lidar FOVs to construct frustums
 *          and associated methods
 *********************************************************************/

#ifndef SPATIO_TEMPORAL_VOXEL_LAYER__FRUSTUM_MODELS__THREE_DIMENSIONAL_LIDAR_FRUSTUM_HPP_
#define SPATIO_TEMPORAL_VOXEL_LAYER__FRUSTUM_MODELS__THREE_DIMENSIONAL_LIDAR_FRUSTUM_HPP_

// M_PI
#include <cmath>
// STL
#include <string>
#include <vector>
// STVL
#include "spatio_temporal_voxel_layer/frustum_models/frustum.hpp"

namespace geometry
{

// A class to model a spinning 3D Lidar frustum in world space
class ThreeDimensionalLidarFrustum : public Frustum
{
public:
  ThreeDimensionalLidarFrustum(
    const double & vFOV, const double & vFOVOffset, const double & vFOVPadding,
    const double & hFOV, const double & min_dist, const double & max_dist
#if VISUALIZE_FRUSTUM
    , const std::string & frame_id = ""
    , const std::string & source_name = ""
    , bool is_marking = false
#endif
  );
  virtual ~ThreeDimensionalLidarFrustum(void);

  // Does nothing in 3D lidar model
  virtual void TransformModel(void);

  // determine if a point is inside of the transformed frustum
  virtual bool IsInside(const openvdb::Vec3d & pt);

  // set pose of 3d lidar in global space
  virtual void SetPosition(const geometry_msgs::msg::Point & origin);
  virtual void SetOrientation(const geometry_msgs::msg::Quaternion & quat);

#if VISUALIZE_FRUSTUM
  // Configure visualization mode: true = marking (green), false = clearing (red).
  void SetVisualizationMode(bool is_marking);
  // Set obstacle height bounds for visualization
  void SetObstacleHeightBounds(double min_height, double max_height);
  // Set the frame id used for visualization markers.
  void SetFrameId(const std::string & frame_id);
  // Set the observation source name for visualization namespaces.
  void SetSourceName(const std::string & source_name);
#endif

private:
  // utils to find useful frustum metadata
  double Dot(const VectorWithPt3D &, const openvdb::Vec3d &) const;
  double Dot(const VectorWithPt3D &, const Eigen::Vector3d &) const;
#if VISUALIZE_FRUSTUM
  void compute_ring_points(
    double radial_distance, double z_center, double z_extent, double padding, double azimuth_min,
    double azimuth_step, std::vector<Eigen::Vector3d> & top_ring,
    std::vector<Eigen::Vector3d> & bottom_ring) const;
  void sample_ring_xy(
    double radius, double azimuth_min, double azimuth_step, int num_samples,
    std::vector<Eigen::Vector3d> & ring_points) const;
#endif

  double _vFOV, _vFOVOffset, _vFOVPadding, _hFOV, _min_d, _max_d;
  double _hFOVhalf;
  double _min_d_squared, _max_d_squared;
  double _tan_vFOVhalf;
  double _tan_vFOVhalf_squared;
  Eigen::Vector3d _position;
  Eigen::Quaterniond _orientation;
  Eigen::Quaterniond _orientation_conjugate;
  bool _valid_frustum;
  bool _full_hFOV;

#if VISUALIZE_FRUSTUM
  bool _is_marking;
  std::string _frame_id;
  std::string _source_name;
  double _min_obstacle_height, _max_obstacle_height;
  // Sampled points on the frustum surface in the sensor frame, used only
  // for visualization and debugging.
  std::vector<Eigen::Vector3d> _top_near_pts;
  std::vector<Eigen::Vector3d> _bottom_near_pts;
  std::vector<Eigen::Vector3d> _top_far_pts;
  std::vector<Eigen::Vector3d> _bottom_far_pts;
  rclcpp::Node::SharedPtr _node;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr _frustum_pub;
#endif
};

#if VISUALIZE_FRUSTUM
namespace observation
{
struct MeasurementReading;
}
// Publish 3D LiDAR frustum visualization for a marking reading. Isolated here
// so the grid class does not contain frustum-visualization logic.
void PublishMarkingFrustumVisualizationIfEnabled(
  const observation::MeasurementReading & reading);
#endif

}  // namespace geometry

#endif  // SPATIO_TEMPORAL_VOXEL_LAYER__FRUSTUM_MODELS__THREE_DIMENSIONAL_LIDAR_FRUSTUM_HPP_
