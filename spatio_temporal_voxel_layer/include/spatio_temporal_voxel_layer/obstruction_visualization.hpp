/*********************************************************************
 *
 * Software License Agreement
 *
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
 *   * Neither the name of the project nor the names of its
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
 *********************************************************************/

#ifndef SPATIO_TEMPORAL_VOXEL_LAYER__OBSTRUCTION_VISUALIZATION_HPP_
#define SPATIO_TEMPORAL_VOXEL_LAYER__OBSTRUCTION_VISUALIZATION_HPP_

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <string>
#include <utility>
#include <vector>

#include "geometry_msgs/msg/point.hpp"
#include "spatio_temporal_voxel_layer/obstruction_polygons.hpp"
#include "visualization_msgs/msg/marker.hpp"

namespace spatio_temporal_voxel_layer
{
namespace obstruction_visualization
{

inline std::vector<geometry_msgs::msg::Point>
sampleObstructionBoundary(const geometry::AngularPolygon &vertices, double visualization_range)
{
  constexpr size_t kSamplesPerEdge = 12;

  std::vector<geometry_msgs::msg::Point> loop;
  const size_t n = vertices.size();

  std::vector<geometry::Vec3D> directions(n);
  for (size_t i = 0; i < n; ++i) {
    // Convert each azimuth/elevation pair into a unit direction on the sphere.
    const double cos_el = std::cos(vertices[i].elevation);
    directions[i] = {cos_el * std::cos(vertices[i].azimuth), cos_el * std::sin(vertices[i].azimuth),
                     std::sin(vertices[i].elevation)};
  }

  loop.reserve(n * kSamplesPerEdge + 1);
  for (size_t i = 0; i < n; ++i) {
    const auto &a = directions[i];
    const auto &b = directions[(i + 1) % n];
    // The dot product of unit directions is cos(omega), their angle apart.
    const double dot = std::clamp(a.x * b.x + a.y * b.y + a.z * b.z, -1.0, 1.0);
    const double omega = std::acos(dot);
    const double sin_omega = std::sin(omega);

    for (size_t k = 0; k < kSamplesPerEdge; ++k) {
      const double t = static_cast<double>(k) / static_cast<double>(kSamplesPerEdge);
      const bool degenerate = sin_omega < 1e-9;
      // Slerp samples the shorter spherical arc; use linear weights near zero
      // angle.
      const double wa = degenerate ? (1.0 - t) : std::sin((1.0 - t) * omega) / sin_omega;
      const double wb = degenerate ? t : std::sin(t * omega) / sin_omega;
      const geometry::Vec3D direction{wa * a.x + wb * b.x, wa * a.y + wb * b.y,
                                      wa * a.z + wb * b.z};
      const double length = std::sqrt(direction.x * direction.x + direction.y * direction.y +
                                      direction.z * direction.z);
      if (length < 1e-12) {
        continue;
      }

      geometry_msgs::msg::Point point;
      // Normalize the interpolated direction and scale it to the visualization
      // radius.
      point.x = visualization_range * direction.x / length;
      point.y = visualization_range * direction.y / length;
      point.z = visualization_range * direction.z / length;
      loop.push_back(point);
    }
  }

  if (!loop.empty()) {
    // Repeat the first point so the LINE_STRIP closes the boundary.
    loop.push_back(loop.front());
  }
  return loop;
}

inline std::vector<visualization_msgs::msg::Marker>
buildObstructionMarkers(const std::string &polygons_param, const std::vector<double> &min_ranges,
                        const std::string &frame, const std::string &ns, double visualization_range)
{
  std::vector<visualization_msgs::msg::Marker> markers;
  if (polygons_param.empty() || frame.empty()) {
    return markers;
  }

  const auto polygons = geometry::parsePolygonsFromString(polygons_param);
  for (size_t i = 0; i < polygons.size(); ++i) {
    const double min_range = i < min_ranges.size() ? min_ranges[i] : 0.0;
    const std::vector<std::pair<double, float>> loops =
        min_range > 0.0
            ? std::vector<std::pair<double, float>>{{visualization_range, 1.0f}, {min_range, 0.35f}}
            : std::vector<std::pair<double, float>>{{visualization_range, 1.0f}};

    for (size_t l = 0; l < loops.size(); ++l) {
      visualization_msgs::msg::Marker marker;
      marker.header.frame_id = frame;
      marker.ns = ns;
      marker.id = static_cast<int>(i) * 2 + static_cast<int>(l);
      marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
      marker.action = visualization_msgs::msg::Marker::ADD;
      marker.pose.orientation.w = 1.0;
      marker.scale.x = 0.02;
      marker.color.r = 1.0f;
      marker.color.g = 0.55f;
      marker.color.a = loops[l].second;
      marker.frame_locked = true;
      marker.points = sampleObstructionBoundary(polygons[i], loops[l].first);
      markers.push_back(std::move(marker));
    }
  }

  return markers;
}

} // namespace obstruction_visualization
} // namespace spatio_temporal_voxel_layer

#endif // SPATIO_TEMPORAL_VOXEL_LAYER__OBSTRUCTION_VISUALIZATION_HPP_
