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
 *********************************************************************/

#include <spatio_temporal_voxel_layer/frustum_models/three_dimensional_lidar_frustum.hpp>

#if VISUALIZE_FRUSTUM
#include "spatio_temporal_voxel_layer/measurement_reading.h"
#endif

namespace geometry
{

#if VISUALIZE_FRUSTUM
namespace
{
// Shared ROS node and publisher used for all 3D lidar frustum
// visualizations. Making these static ensures the topic remains
// advertised persistently rather than flickering with each
// temporary frustum instance.
rclcpp::Node::SharedPtr g_three_dimensional_lidar_frustum_node;
rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
  g_three_dimensional_lidar_frustum_pub;
}  // namespace
#endif

/*****************************************************************************/
ThreeDimensionalLidarFrustum::ThreeDimensionalLidarFrustum(
  const double & vFOV, const double & vFOVOffset, const double & vFOVPadding,
  const double & hFOV, const double & min_dist, const double & max_dist
#if VISUALIZE_FRUSTUM
  , const std::string & frame_id
  , const std::string & source_name
  , bool is_marking
#endif
)
: _vFOV(vFOV),
  _vFOVOffset(vFOVOffset),
  _vFOVPadding(vFOVPadding),
  _hFOV(hFOV),
  _min_d(min_dist),
  _max_d(max_dist)
/*****************************************************************************/
{
  _hFOVhalf = _hFOV / 2.0;
  _tan_vFOVhalf = tan(_vFOV / 2.0);
  _tan_vFOVhalf_squared = _tan_vFOVhalf * _tan_vFOVhalf;
  _min_d_squared = _min_d * _min_d;
  _max_d_squared = _max_d * _max_d;
  _full_hFOV = false;
  if (_hFOV > 6.27) {
    _full_hFOV = true;
  }

#if VISUALIZE_FRUSTUM
  _is_marking = is_marking;
  _frame_id = frame_id;
  _source_name = source_name;
  _min_obstacle_height = 0.0;
  _max_obstacle_height = 3.0;

  // Pre-sample a set of points on the frustum surface in the sensor frame.
  // We construct rings at the minimum and maximum range and at the upper
  // and lower vertical FOV limits, then distribute samples over azimuth.
  constexpr int num_azimuth_samples = 64;
  constexpr double two_pi = 2.0 * M_PI;

  const bool use_full_circle = _full_hFOV;
  const double azimuth_min = use_full_circle ? 0.0 : -_hFOVhalf;
  const double azimuth_max = use_full_circle ? two_pi : _hFOVhalf;
  const double azimuth_step =
    (azimuth_max - azimuth_min) / static_cast<double>(num_azimuth_samples);

  // Near-range ring
  if (_min_d > 0.0) {
    const double radial_near = _min_d;
    const double z_center_near = radial_near * std::tan(_vFOVOffset);
    const double z_extent_near = radial_near * std::tan(_vFOV / 2.0);
    compute_ring_points(
      radial_near, z_center_near, z_extent_near, _vFOVPadding, azimuth_min, azimuth_step,
      _top_near_pts, _bottom_near_pts);
  }

  // Far-range ring
  // For marking, _min_d might be 0.0, so we allow _max_d >= _min_d
  if (_max_d > 0.0 && _max_d >= _min_d) {
    const double radial_far = _max_d;
    const double z_center_far = radial_far * std::tan(_vFOVOffset);
    const double z_extent_far = radial_far * std::tan(_vFOV / 2.0);
    compute_ring_points(
      radial_far, z_center_far, z_extent_far, _vFOVPadding, azimuth_min, azimuth_step, _top_far_pts,
      _bottom_far_pts);
  }
#endif
}

/*****************************************************************************/
ThreeDimensionalLidarFrustum::~ThreeDimensionalLidarFrustum(void)
/*****************************************************************************/
{
}

/*****************************************************************************/
void ThreeDimensionalLidarFrustum::compute_ring_points(
  double radial_distance, double z_center, double z_extent, double padding, double azimuth_min,
  double azimuth_step, std::vector<Eigen::Vector3d> & top_ring,
  std::vector<Eigen::Vector3d> & bottom_ring) const
/*****************************************************************************/
{
  constexpr int kNumAzimuthSamples = 64;
  std::vector<Eigen::Vector3d> ring_xy;
  ring_xy.reserve(kNumAzimuthSamples + 1);
  sample_ring_xy(radial_distance, azimuth_min, azimuth_step, kNumAzimuthSamples, ring_xy);

  const double z_upper = z_center + z_extent + padding;
  const double z_lower = z_center - z_extent - padding;

  for (const auto & pt_xy : ring_xy) {
    const double x = pt_xy.x();
    const double y = pt_xy.y();
    const double z_upper = z_center + z_extent + padding;
    const double z_lower = z_center - z_extent - padding;

    top_ring.emplace_back(x, y, z_upper);
    bottom_ring.emplace_back(x, y, z_lower);
  }
}

/*****************************************************************************/
void ThreeDimensionalLidarFrustum::sample_ring_xy(
  double radius, double azimuth_min, double azimuth_step, int num_samples,
  std::vector<Eigen::Vector3d> & ring_points) const
/*****************************************************************************/
{
  for (int i = 0; i <= num_samples; ++i) {
    const double az = azimuth_min + azimuth_step * static_cast<double>(i);
    const double x = radius * std::cos(az);
    const double y = radius * std::sin(az);
    ring_points.emplace_back(x, y, 0.0);
  }
}

#if VISUALIZE_FRUSTUM
/*****************************************************************************/
void ThreeDimensionalLidarFrustum::SetVisualizationMode(bool is_marking)
/*****************************************************************************/
{
  _is_marking = is_marking;
}

/*****************************************************************************/
void ThreeDimensionalLidarFrustum::SetObstacleHeightBounds(double min_height, double max_height)
/*****************************************************************************/
{
  _min_obstacle_height = min_height;
  _max_obstacle_height = max_height;
}

/*****************************************************************************/
void ThreeDimensionalLidarFrustum::SetFrameId(const std::string & frame_id)
/*****************************************************************************/
{
  _frame_id = frame_id;
}

/*****************************************************************************/
void ThreeDimensionalLidarFrustum::SetSourceName(const std::string & source_name)
/*****************************************************************************/
{
  _source_name = source_name;
}

/*****************************************************************************/
void PublishMarkingFrustumVisualizationIfEnabled(
  const observation::MeasurementReading & reading)
/*****************************************************************************/
{
  if (reading._model_type != THREE_DIMENSIONAL_LIDAR) {
    return;
  }
  if (reading._marking == 0.0) {
    return;
  }
  if (reading._obstacle_range_in_m <= 0.0) {
    return;
  }
  geometry::ThreeDimensionalLidarFrustum frustum(
    reading._vertical_fov_in_rad, reading._vertical_fov_offset_in_rad,
    reading._vertical_fov_padding_in_m, reading._horizontal_fov_in_rad, 0.0,
    reading._obstacle_range_in_m,
    reading._cloud ? reading._cloud->header.frame_id : std::string(""),
    reading._source_name,
    true);
  frustum.SetPosition(reading._origin);
  frustum.SetOrientation(reading._orientation);
  frustum.SetObstacleHeightBounds(
    reading._min_obstacle_height, reading._max_obstacle_height);
  frustum.TransformModel();
}
#endif

/*****************************************************************************/
void ThreeDimensionalLidarFrustum::TransformModel(void)
/*****************************************************************************/
{
  _orientation_conjugate = _orientation.conjugate();
  _valid_frustum = true;

#if VISUALIZE_FRUSTUM
  // Ensure the shared node and publisher exist so that the topic
  // remains persistently advertised across temporary frustum
  // instances.
  if (!g_three_dimensional_lidar_frustum_node) {
    g_three_dimensional_lidar_frustum_node =
      std::make_shared<rclcpp::Node>("lidar_frustum_publisher");
    g_three_dimensional_lidar_frustum_pub =
      g_three_dimensional_lidar_frustum_node
        ->create_publisher<visualization_msgs::msg::MarkerArray>("lidar_frustum", 10);
  }

  // Publish a simple visualization of the frustum in the global frame.
  if (_top_far_pts.empty()) {
    return;
  }
  if (_bottom_far_pts.empty()) {
    return;
  }

  visualization_msgs::msg::MarkerArray msg_list;

  const std::string frame_id = _frame_id.empty() ? std::string("map") : _frame_id;

  // Base color: green for marking, red for clearing.
  const float base_r = _is_marking ? 0.0f : 1.0f;
  const float base_g = _is_marking ? 1.0f : 0.0f;
  const float base_b = 0.0f;

  const std::string source_suffix =
    _source_name.empty() ? std::string("unknown_source") : _source_name;
  const std::string base_ns_prefix = _is_marking ? "marking_" : "clearing_";
  const std::string base_ns = base_ns_prefix + source_suffix;

  // Transform a local point in the sensor frame into a global geometry point.
  auto to_global_point = [&](const Eigen::Vector3d & local_pt) {
    const Eigen::Vector3d global_pt = _orientation * local_pt + _position;
    geometry_msgs::msg::Point p;
    p.x = global_pt[0];
    p.y = global_pt[1];
    p.z = global_pt[2];
    return p;
  };

  // Append a segment (two points) to a line marker using local-frame points.
  auto add_segment = [&](
                       std::vector<geometry_msgs::msg::Point> & container,
                       const Eigen::Vector3d & start_local, const Eigen::Vector3d & end_local) {
    container.push_back(to_global_point(start_local));
    container.push_back(to_global_point(end_local));
  };

  // Factory for a basic marker with common fields populated.
  auto make_marker = [&](
                       const std::string & ns, int id, int32_t type, float scale_x, float r,
                       float g, float b, float a) {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = frame_id;
    marker.header.stamp = g_three_dimensional_lidar_frustum_node->now();
    marker.ns = ns;
    marker.id = id;
    marker.type = type;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.scale.x = scale_x;
    marker.color.r = r;
    marker.color.g = g;
    marker.color.b = b;
    marker.color.a = a;
    marker.pose.orientation.w = 1.0;
    return marker;
  };

  // Helper lambda to build a closed LINE_STRIP ring from a set of local points.
  auto add_ring_marker =
    [&](const std::vector<Eigen::Vector3d> & pts, int id, float r, float g, float b) {
      auto ring_marker =
        make_marker(base_ns, id, visualization_msgs::msg::Marker::LINE_STRIP, 0.03F, r, g, b, 1.0F);

      ring_marker.points.reserve(pts.size() + 1);
      for (const auto & local_pt : pts) {
        ring_marker.points.push_back(to_global_point(local_pt));
      }
      // Close the ring by repeating the first point if any exist.
      if (!ring_marker.points.empty()) {
        ring_marker.points.push_back(ring_marker.points.front());
      }
      msg_list.markers.push_back(ring_marker);
    };

  // Top and bottom rings at far range (main frustum footprint)
  add_ring_marker(_top_far_pts, 0, base_r, base_g, base_b);
  add_ring_marker(_bottom_far_pts, 1, base_r, base_g, base_b);

  // Optionally, also show near rings if available.
  if (!_top_near_pts.empty() && !_bottom_near_pts.empty()) {
    const float near_scale = 0.6f;
    add_ring_marker(
      _top_near_pts, 2, base_r * near_scale, base_g * near_scale, base_b * near_scale);
    add_ring_marker(
      _bottom_near_pts, 3, base_r * near_scale, base_g * near_scale, base_b * near_scale);
  }

  // Add rays from the lidar origin to the top and bottom far planes.
  auto rays_marker = make_marker(
    base_ns, 4, visualization_msgs::msg::Marker::LINE_LIST, 0.02F, base_r, base_g, base_b, 1.0F);

  // Use the same azimuth sampling as the precomputed rings so that rays
  // and vertical segments align exactly with ring vertices.
  const bool use_full_circle = _full_hFOV;
  const double azimuth_min = use_full_circle ? 0.0 : -_hFOVhalf;
  const double azimuth_max = use_full_circle ? (2.0 * M_PI) : _hFOVhalf;

  // Markers for vertical circle segments at far and near radii.
  auto vertical_far_marker = make_marker(
    base_ns, 5, visualization_msgs::msg::Marker::LINE_LIST, 0.02F, base_r, base_g, base_b, 1.0F);

  auto vertical_near_marker = make_marker(
    base_ns, 6, visualization_msgs::msg::Marker::LINE_LIST, 0.02F, base_r, base_g, base_b, 1.0F);

  const int num_vertical_samples = 24;  // resolution of each vertical circle

  // Draw vertical circle segments at a given radius in the plane of an azimuth.
  auto add_vertical_circle_segments =
    [&](double radius, double azimuth, std::vector<geometry_msgs::msg::Point> & out_points) {
      if (radius <= 0.0) {
        return;
      }

      double prev_x = 0.0;
      double prev_y = 0.0;
      double prev_z = 0.0;
      bool has_prev = false;

      for (int j = 0; j <= num_vertical_samples; ++j) {
        // Parameter t goes from -1 (bottom) to +1 (top) of the FOV
        const double t =
          -1.0 + (2.0 * static_cast<double>(j) / static_cast<double>(num_vertical_samples));

        // Horizontal distance is constant (the radius)
        const double x_local = radius * std::cos(azimuth);
        const double y_local = radius * std::sin(azimuth);

        // Z varies according to the frustum formula: z = r*tan(offset) + t*(r*tan(vFOV/2) +
        // padding)
        const double z_center = radius * std::tan(_vFOVOffset);
        const double z_extent = radius * std::tan(_vFOV / 2.0);
        const double z_local = z_center + t * (z_extent + _vFOVPadding);

        if (has_prev) {
          const Eigen::Vector3d p0_local(prev_x, prev_y, prev_z);
          const Eigen::Vector3d p1_local(x_local, y_local, z_local);
          add_segment(out_points, p0_local, p1_local);
        }

        prev_x = x_local;
        prev_y = y_local;
        prev_z = z_local;
        has_prev = true;
      }
    };

  if (!_top_far_pts.empty() && !_bottom_far_pts.empty()) {
    const std::size_t points_per_ring = _top_far_pts.size();
    if (points_per_ring < 2) {
      // Not enough points to form rays / vertical segments.
      msg_list.markers.push_back(rays_marker);
      if (!vertical_far_marker.points.empty()) {
        msg_list.markers.push_back(vertical_far_marker);
      }
      if (!vertical_near_marker.points.empty()) {
        msg_list.markers.push_back(vertical_near_marker);
      }
      // continue with planes below
    } else {
      const int num_samples = static_cast<int>(points_per_ring - 1);
      const double azimuth_step = (azimuth_max - azimuth_min) / static_cast<double>(num_samples);

      // Downsample rays and vertical segments so they are drawn
      // approximately every 30 degrees in azimuth.
      constexpr double desired_azimuth_step_deg = 30.0;
      const double desired_azimuth_step_rad = desired_azimuth_step_deg * (M_PI / 180.0);
      int azimuth_stride = static_cast<int>(std::round(desired_azimuth_step_rad / azimuth_step));
      if (azimuth_stride < 1) {
        azimuth_stride = 1;
      }

      for (int i = 0; i <= num_samples; i += azimuth_stride) {
        const std::size_t idx = static_cast<std::size_t>(i);

        const Eigen::Vector3d & top_far_local = _top_far_pts.at(idx);
        const Eigen::Vector3d & bottom_far_local = _bottom_far_pts.at(idx);

        // Compute azimuth for this ray (in sensor frame) using the same
        // grid as the precomputed rings.
        const double azimuth = azimuth_min + azimuth_step * static_cast<double>(i);

        geometry_msgs::msg::Point start_point;

        // If we have an inner ring (min_d > 0), rays span from inner to outer.
        // Otherwise, rays start from the LiDAR center.
        if (_min_d > 0.0 && !_top_near_pts.empty() && !_bottom_near_pts.empty()) {
          // Use inner ring points as the start of the rays
          const Eigen::Vector3d & top_near_local = _top_near_pts.at(idx);
          const Eigen::Vector3d & bottom_near_local = _bottom_near_pts.at(idx);

          // Ray to top plane: from inner ring to outer ring
          add_segment(rays_marker.points, top_near_local, top_far_local);

          // Ray to bottom plane: from inner ring to outer ring
          add_segment(rays_marker.points, bottom_near_local, bottom_far_local);
        } else {
          // No inner ring: rays start from the LiDAR center
          start_point.x = _position[0];
          start_point.y = _position[1];
          start_point.z = _position[2];

          // Ray to top plane.
          rays_marker.points.push_back(start_point);
          rays_marker.points.push_back(to_global_point(top_far_local));

          // Ray to bottom plane.
          rays_marker.points.push_back(start_point);
          rays_marker.points.push_back(to_global_point(bottom_far_local));
        }

        // Vertical circle segments at far and near radii in the plane of this azimuth.
        add_vertical_circle_segments(_max_d, azimuth, vertical_far_marker.points);
        if (_min_d > 0.0 && !_top_near_pts.empty() && !_bottom_near_pts.empty()) {
          add_vertical_circle_segments(_min_d, azimuth, vertical_near_marker.points);
        }
      }
    }
  }

  msg_list.markers.push_back(rays_marker);
  if (!vertical_far_marker.points.empty()) {
    msg_list.markers.push_back(vertical_far_marker);
  }
  if (!vertical_near_marker.points.empty()) {
    msg_list.markers.push_back(vertical_near_marker);
  }

  // Add horizontal planes for min and max obstacle heights
  // IMPORTANT: These planes are in the GLOBAL/COSTMAP frame (not sensor frame)
  // because min/max_obstacle_height filtering happens AFTER transformation to global frame.
  // The planes are horizontal (constant z in global frame) and show where points are clipped.
  // NOTE: Only show these planes for marking frustums, as clearing frustums don't use
  // obstacle height filtering (clearing-only observations skip point cloud processing).
  if (_is_marking && _max_d > 0.0) {
    const int num_plane_samples = 64;
    const bool use_full_circle = _full_hFOV;
    const double azimuth_min = use_full_circle ? 0.0 : -_hFOVhalf;
    const double azimuth_max = use_full_circle ? (2.0 * M_PI) : _hFOVhalf;
    const double azimuth_step =
      (azimuth_max - azimuth_min) / static_cast<double>(num_plane_samples);

    // All vertical constraint planes share a single namespace derived from the
    // observation source name so they can be easily managed together in RViz.
    const std::string vertical_constraints_ns =
      (_source_name.empty() ? std::string("unknown_source") : _source_name) +
      std::string("_vertical_constraints");

    // Helper to create a horizontal plane marker in the global frame. The
    // plane is horizontal (constant z in global frame) and extends radially
    // from the sensor.
    auto create_plane_marker = [&](double height_z_global, int id) {
      auto plane_marker = make_marker(
        vertical_constraints_ns, id, visualization_msgs::msg::Marker::LINE_STRIP, 0.03F, base_r,
        base_g, base_b, 0.6F);  // Slightly transparent

      // Sample local XY points on a ring at the maximum range.
      std::vector<Eigen::Vector3d> ring_xy;
      ring_xy.reserve(num_plane_samples + 1);
      sample_ring_xy(_max_d, azimuth_min, azimuth_step, num_plane_samples, ring_xy);

      // Create a circle/arc at constant z in GLOBAL frame. We sample
      // points radially from the sensor position at the specified
      // global z height.
      for (const auto & p_local_xy : ring_xy) {
        const Eigen::Vector3d p_local(p_local_xy.x(), p_local_xy.y(), 0.0);
        Eigen::Vector3d p_global = _orientation * p_local + _position;

        // Override z to the global height (this matches how filtering works)
        // The filtering happens in global frame, so z is set to the global height value
        p_global.z() = height_z_global;

        geometry_msgs::msg::Point p;
        p.x = p_global[0];
        p.y = p_global[1];
        p.z = p_global[2];
        plane_marker.points.push_back(p);
      }
      return plane_marker;
    };

    // Min and max obstacle height planes (in global frame)
    visualization_msgs::msg::Marker min_plane = create_plane_marker(_min_obstacle_height, 7);
    visualization_msgs::msg::Marker max_plane = create_plane_marker(_max_obstacle_height, 8);
    if (!min_plane.points.empty()) {
      msg_list.markers.push_back(min_plane);
    }
    if (!max_plane.points.empty()) {
      msg_list.markers.push_back(max_plane);
    }
  }

  g_three_dimensional_lidar_frustum_pub->publish(msg_list);

#endif
}

/*****************************************************************************/
bool ThreeDimensionalLidarFrustum::IsInside(const openvdb::Vec3d & pt)
/*****************************************************************************/
{
  Eigen::Vector3d point_in_global_frame(pt[0], pt[1], pt[2]);
  Eigen::Vector3d transformed_pt = _orientation_conjugate *
    (point_in_global_frame - _position);

  const double radial_distance_squared =
    (transformed_pt[0] * transformed_pt[0]) +
    (transformed_pt[1] * transformed_pt[1]);

  // Check if inside frustum valid range
  if (radial_distance_squared > _max_d_squared ||
    radial_distance_squared < _min_d_squared)
  {
    return false;
  }

  // Check if inside frustum valid vFOV
  const double z_rel = transformed_pt[2] - sqrt(radial_distance_squared) * std::tan(_vFOVOffset);
  const double v_padded = fabs(z_rel) + _vFOVPadding;

  if ((v_padded * v_padded / radial_distance_squared) > _tan_vFOVhalf_squared)
  {
    return false;
  }

  // Check if inside frustum valid hFOV, unless hFOV is full-circle (360 degree)
  if (!_full_hFOV) {
    if (transformed_pt[0] > 0) {
      if (fabs(atan(transformed_pt[1] / transformed_pt[0])) > _hFOVhalf) {
        return false;
      }
    } else if (fabs(atan(transformed_pt[0] / transformed_pt[1])) + M_PI_2 > _hFOVhalf) {
      return false;
    }
  }

  return true;
}

/*****************************************************************************/
void ThreeDimensionalLidarFrustum::SetPosition(
  const geometry_msgs::msg::Point & origin)
/*****************************************************************************/
{
  _position = Eigen::Vector3d(origin.x, origin.y, origin.z);
}

/*****************************************************************************/
void ThreeDimensionalLidarFrustum::SetOrientation(
  const geometry_msgs::msg::Quaternion & quat)
/*****************************************************************************/
{
  _orientation = Eigen::Quaterniond(quat.w, quat.x, quat.y, quat.z);
}

/*****************************************************************************/
double ThreeDimensionalLidarFrustum::Dot(
  const VectorWithPt3D & plane_pt, const openvdb::Vec3d & query_pt) const
/*****************************************************************************/
{
  return plane_pt.x * query_pt[0] +
         plane_pt.y * query_pt[1] +
         plane_pt.z * query_pt[2];
}

/*****************************************************************************/
double ThreeDimensionalLidarFrustum::Dot(
  const VectorWithPt3D & plane_pt, const Eigen::Vector3d & query_pt) const
/*****************************************************************************/
{
  return plane_pt.x * query_pt[0] +
         plane_pt.y * query_pt[1] +
         plane_pt.z * query_pt[2];
}

}  // namespace geometry
