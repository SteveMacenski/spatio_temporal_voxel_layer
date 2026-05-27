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
 * Purpose: Convex polygon obstruction zones for frustum blind spot filtering.
 *          Points whose coordinates in sensor-local frame fall inside
 *          any obstruction polygon are excluded from frustum clearing.
 *********************************************************************/

#ifndef SPATIO_TEMPORAL_VOXEL_LAYER__OBSTRUCTION_POLYGONS_HPP_
#define SPATIO_TEMPORAL_VOXEL_LAYER__OBSTRUCTION_POLYGONS_HPP_

#include <vector>
#include <cmath>
#include <string>
#include <sstream>
#include <utility>
#include <algorithm>

#include "rclcpp/rclcpp.hpp"

namespace geometry
{

/**
 * @brief A generic 2D convex polygon with precomputed half-plane representation
 *        for fast point-in-polygon queries.
 */
struct ConvexPolygon2D
{
  // Vertices as (x, y) pairs, must be convex and consistently wound
  std::vector<std::pair<double, double>> vertices;

  // Precomputed edge normals and offsets for half-plane test
  // For edge from v[i] to v[(i+1)%n], the inward normal and offset
  std::vector<double> nx, ny, d;

  /**
   * @brief Precompute half-plane representation for fast point-in-polygon.
   * @return true if polygon is valid (convex, >= 3 vertices)
   */
  bool precompute()
  {
    size_t n = vertices.size();
    if (n < 3) {
      return false;
    }

    nx.resize(n);
    ny.resize(n);
    d.resize(n);

    // Compute signed area to determine winding
    double area = 0.0;
    for (size_t i = 0; i < n; ++i) {
      size_t j = (i + 1) % n;
      area += vertices[i].first * vertices[j].second;
      area -= vertices[j].first * vertices[i].second;
    }
    bool ccw = area > 0.0;

    for (size_t i = 0; i < n; ++i) {
      size_t j = (i + 1) % n;
      double ex = vertices[j].first - vertices[i].first;
      double ey = vertices[j].second - vertices[i].second;

      // Inward normal (perpendicular to edge, pointing inside)
      if (ccw) {
        nx[i] = -ey;  // rotate edge 90° CCW for inward normal of CCW polygon
        ny[i] = ex;
      } else {
        nx[i] = ey;   // rotate edge 90° CW for inward normal of CW polygon
        ny[i] = -ex;
      }

      // Normalize
      double len = std::sqrt(nx[i] * nx[i] + ny[i] * ny[i]);
      if (len < 1e-12) {
        return false;  // degenerate edge
      }
      nx[i] /= len;
      ny[i] /= len;

      // Half-plane offset: the signed distance from the origin to the edge along
      // the normal direction. By precomputing d = n·v (where v is any point on
      // the edge), the inside test becomes n·p ≥ d — a single dot product and
      // compare per edge — rather than n·(p - v) which requires a subtraction.
      d[i] = nx[i] * vertices[i].first + ny[i] * vertices[i].second;
    }

    return true;
  }

  /**
   * @brief Check if a 2D point is inside this convex polygon.
   * Uses half-plane intersection: point must be on the inner side of all edges.
   */
  bool isInside(double x, double y) const
  {
    for (size_t i = 0; i < nx.size(); ++i) {
      if (nx[i] * x + ny[i] * y < d[i]) {
        return false;
      }
    }
    return true;
  }
};

/**
 * @brief Validate obstruction polygons for use in azimuth/elevation space.
 * Checks vertex count, NaN, azimuth bounds [0, 2π], and precomputes half-planes.
 * @return vector of valid, precomputed polygons
 */
inline std::vector<ConvexPolygon2D> validatePolygons(
  const std::vector<ConvexPolygon2D> & input,
  const rclcpp::Logger & logger)
{
  std::vector<ConvexPolygon2D> valid;
  valid.reserve(input.size());

  for (size_t idx = 0; idx < input.size(); ++idx) {
    auto poly = input[idx];
    if (poly.vertices.size() < 3) {
      RCLCPP_WARN(
        logger,
        "Obstruction polygon %zu has < 3 vertices, skipping.",
        idx);
      continue;
    }

    // Check for NaN
    bool has_nan = false;
    for (const auto & v : poly.vertices) {
      if (!std::isfinite(v.first) || !std::isfinite(v.second)) {
        has_nan = true;
        break;
      }
    }
    if (has_nan) {
      RCLCPP_WARN(
        logger,
        "Obstruction polygon %zu has NaN/inf vertices, skipping.",
        idx);
      continue;
    }

    // Check azimuth bounds — warn if any vertex outside [0, 2π]
    bool out_of_range = false;
    for (const auto & v : poly.vertices) {
      if (v.first < 0.0 || v.first > 2.0 * M_PI) {
        out_of_range = true;
        break;
      }
    }
    if (out_of_range) {
      RCLCPP_WARN(
        logger,
        "Obstruction polygon %zu has vertices outside [0, 2pi] azimuth range, "
        "skipping. Polygons crossing the 0/2pi boundary are not supported.",
        idx);
      continue;
    }

    if (!poly.precompute()) {
      RCLCPP_WARN(
        logger,
        "Obstruction polygon %zu failed precomputation (non-convex or degenerate), skipping.",
        idx);
      continue;
    }

    valid.push_back(std::move(poly));
  }

  return valid;
}

/**
 * @brief Check if a point falls inside any of the obstruction polygons.
 */
inline bool isInsideAnyObstruction(
  const std::vector<ConvexPolygon2D> & polygons,
  double x, double y)
{
  for (const auto & poly : polygons) {
    if (poly.isInside(x, y)) {
      return true;
    }
  }
  return false;
}

/**
 * @brief Parse a list of polygons from a footprint-style string.
 *
 * Format: "[[x1,y1, x2,y2, x3,y3], [x4,y4, x5,y5, x6,y6]]"
 * Each inner [...] is one polygon with flat x/y vertex pairs.
 *
 * @return vector of parsed (unvalidated) polygons
 */
inline std::vector<ConvexPolygon2D> parsePolygonsFromString(
  const std::string & input,
  const rclcpp::Logger & logger)
{
  std::vector<ConvexPolygon2D> polygons;

  // Find the outer brackets
  size_t outer_start = input.find('[');
  size_t outer_end = input.rfind(']');
  if (outer_start == std::string::npos || outer_end == std::string::npos ||
    outer_end <= outer_start)
  {
    RCLCPP_WARN(logger, "Obstruction polygons string has invalid format.");
    return polygons;
  }

  // Parse inner polygon brackets
  size_t pos = outer_start + 1;
  int poly_idx = 0;
  while (pos < outer_end) {
    // Find next inner '['
    size_t inner_start = input.find('[', pos);
    if (inner_start == std::string::npos || inner_start >= outer_end) {
      break;
    }
    size_t inner_end = input.find(']', inner_start);
    if (inner_end == std::string::npos || inner_end > outer_end) {
      RCLCPP_WARN(logger, "Obstruction polygons string has unmatched brackets.");
      break;
    }

    // Extract the comma-separated numbers between inner brackets
    std::string nums_str = input.substr(inner_start + 1, inner_end - inner_start - 1);
    std::vector<double> values;
    std::istringstream ss(nums_str);
    std::string token;
    while (std::getline(ss, token, ',')) {
      // Trim whitespace
      size_t start = token.find_first_not_of(" \t");
      if (start == std::string::npos) {continue;}
      try {
        values.push_back(std::stod(token.substr(start)));
      } catch (const std::exception &) {
        RCLCPP_WARN(
          logger, "Obstruction polygon %d: failed to parse value '%s'.",
          poly_idx, token.c_str());
      }
    }

    if (values.size() >= 6 && values.size() % 2 == 0) {
      ConvexPolygon2D poly;
      for (size_t i = 0; i < values.size(); i += 2) {
        poly.vertices.emplace_back(values[i], values[i + 1]);
      }
      polygons.push_back(std::move(poly));
    } else if (!values.empty()) {
      RCLCPP_WARN(
        logger,
        "Obstruction polygon %d needs >= 6 values (3 x/y pairs), got %zu. Skipping.",
        poly_idx, values.size());
    }

    poly_idx++;
    pos = inner_end + 1;
  }

  return polygons;
}

/**
 * @brief Parse obstruction polygons from a single ROS2 string parameter.
 *
 * Expected parameter format (footprint-style):
 *   obstruction_polygons: "[[x1,y1, x2,y2, x3,y3], [x4,y4, x5,y5, x6,y6]]"
 *
 * For STVL use, x=azimuth (rad), y=elevation (rad).
 */
template<typename NodeT>
std::vector<ConvexPolygon2D> parseObstructionPolygonsFromParams(
  NodeT node,
  const std::string & param_prefix,
  const rclcpp::Logger & logger)
{
  std::string param_name = param_prefix + ".obstruction_polygons";
  std::string polygons_str;
  if (!node->has_parameter(param_name)) {
    node->declare_parameter(param_name, std::string(""));
  }
  node->get_parameter(param_name, polygons_str);

  if (polygons_str.empty()) {
    return {};
  }

  auto polygons = parsePolygonsFromString(polygons_str, logger);

  if (!polygons.empty()) {
    RCLCPP_INFO(
      logger,
      "Parsed %zu obstruction polygon(s) for %s",
      polygons.size(), param_prefix.c_str());
  }

  return validatePolygons(polygons, logger);
}

}  // namespace geometry

#endif  // SPATIO_TEMPORAL_VOXEL_LAYER__OBSTRUCTION_POLYGONS_HPP_
