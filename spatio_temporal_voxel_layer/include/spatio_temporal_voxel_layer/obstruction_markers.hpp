/*********************************************************************
 *
 * Software License Agreement (BSD-3-Clause)
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
 * Purpose: Draw obstruction polygon outlines as marker line strips
 *
 *********************************************************************/

#ifndef SPATIO_TEMPORAL_VOXEL_LAYER__OBSTRUCTION_MARKERS_HPP_
#define SPATIO_TEMPORAL_VOXEL_LAYER__OBSTRUCTION_MARKERS_HPP_

#include <algorithm>
#include <cmath>
#include <string>
#include <vector>

#include "visualization_msgs/msg/marker_array.hpp"

#include "spatio_temporal_voxel_layer/obstruction_polygons.hpp"

namespace geometry
{

/**
 * @brief Trace a polygon's true boundary at a fixed range from the sensor.
 *
 * Edges are geodesics, so the boundary is not the flat az/el outline the vertices suggest.
 *
 * @param vertices polygon corners as (azimuth, elevation)
 * @param radius distance from the sensor to place the outline, in meter
 * @param samples_per_edge points emitted per edge; 12 is smooth at these sizes
 * @return closed loop of points
 */
inline std::vector<geometry_msgs::msg::Point> sampleBoundary(
  const AngularPolygon & vertices, double radius, size_t samples_per_edge = 12)
{
  std::vector<geometry_msgs::msg::Point> loop;
  const size_t n = vertices.size();
  if (n < 2 || samples_per_edge == 0) {
    return loop;
  }

  // Same (az, el) -> direction convention as ConvexCone::fromAngularVertices.
  std::vector<Vec3D> dirs(n);
  for (size_t i = 0; i < n; ++i) {
    const double cos_el = std::cos(vertices[i].elevation);
    dirs[i] = {cos_el * std::cos(vertices[i].azimuth), cos_el * std::sin(vertices[i].azimuth),
      std::sin(vertices[i].elevation)};
  }

  loop.reserve(n * samples_per_edge + 1);
  for (size_t i = 0; i < n; ++i) {
    const Vec3D & a = dirs[i];
    const Vec3D & b = dirs[(i + 1) % n];
    const double dot = std::clamp(a.x * b.x + a.y * b.y + a.z * b.z, -1.0, 1.0);
    const double omega = std::acos(dot);
    const double sin_omega = std::sin(omega);

    for (size_t k = 0; k < samples_per_edge; ++k) {
      const double t = static_cast<double>(k) / static_cast<double>(samples_per_edge);
      // Straight blend for a vanishing arc, where slerp would divide by ~0.
      const bool degenerate = sin_omega < 1e-9;
      const double wa = degenerate ? (1.0 - t) : std::sin((1.0 - t) * omega) / sin_omega;
      const double wb = degenerate ? t : std::sin(t * omega) / sin_omega;

      const Vec3D d{wa * a.x + wb * b.x, wa * a.y + wb * b.y, wa * a.z + wb * b.z};
      const double len = std::sqrt(d.x * d.x + d.y * d.y + d.z * d.z);
      if (len < 1e-12) {
        continue;
      }
      geometry_msgs::msg::Point p;
      p.x = radius * d.x / len;
      p.y = radius * d.y / len;
      p.z = radius * d.z / len;
      loop.push_back(p);
    }
  }

  if (!loop.empty()) {
    loop.push_back(loop.front());
  }
  return loop;
}

/**
 * @brief Outlines for one source's obstruction polygons, ready to publish.
 *
 * Markers are placed in the sensor frame, where a polygon is static, so stamp can be 0.
 *
 * @param polygons_param the source's obstruction_polygons string; empty yields no markers
 * @param min_ranges the source's obstruction_min_ranges; empty means no near-side clearing
 * @param frame frame the polygons are defined in, i.e. the sensor's
 * @param ns marker namespace, conventionally the observation source name
 * @param radius distance to draw the outline at, in meter
 * @return one LINE_STRIP per polygon, plus a dimmer one at min_range where a source sets it.
 *         Empty if the string does not parse.
 */
inline std::vector<visualization_msgs::msg::Marker> buildObstructionMarkers(
  const std::string & polygons_param, const std::vector<double> & min_ranges,
  const std::string & frame, const std::string & ns, double radius)
{
  std::vector<visualization_msgs::msg::Marker> markers;
  if (polygons_param.empty() || frame.empty()) {
    return markers;
  }

  std::vector<AngularPolygon> polygons;
  try {
    polygons = parsePolygonsFromString(polygons_param);
  } catch (const std::exception &) {
    return markers;  // already rejected on the way in; nothing to draw
  }

  for (size_t i = 0; i < polygons.size(); ++i) {
    const double min_range = i < min_ranges.size() ? min_ranges[i] : 0.0;

    // The outline itself, then where retention actually begins, drawn dimmer so it reads as
    // secondary. Two ids per polygon so the two never collide.
    const std::vector<std::pair<double, float>> loops = min_range > 0.0 ?
      std::vector<std::pair<double, float>>{{radius, 1.0f}, {min_range, 0.35f}} :
      std::vector<std::pair<double, float>>{{radius, 1.0f}};

    for (size_t l = 0; l < loops.size(); ++l) {
      visualization_msgs::msg::Marker msg;
      msg.header.frame_id = frame;
      // Stamp left at zero: the outline is static in the sensor frame
      msg.ns = ns;
      msg.id = static_cast<int>(i) * 2 + static_cast<int>(l);
      msg.type = visualization_msgs::msg::Marker::LINE_STRIP;
      msg.action = visualization_msgs::msg::Marker::ADD;
      msg.pose.orientation.w = 1.0;
      msg.scale.x = 0.02;
      msg.color.r = 1.0f;
      msg.color.g = 0.55f;
      msg.color.a = loops[l].second;
      msg.frame_locked = true;

      msg.points = sampleBoundary(polygons[i], loops[l].first);
      markers.push_back(std::move(msg));
    }
  }

  return markers;
}

}  // namespace geometry

#endif  // SPATIO_TEMPORAL_VOXEL_LAYER__OBSTRUCTION_MARKERS_HPP_
