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
 *          Points (voxels) whose coordinates in sensor-local frame fall inside
 *          any obstruction polygon are excluded from frustum clearing.
 *********************************************************************/

#ifndef SPATIO_TEMPORAL_VOXEL_LAYER__OBSTRUCTION_POLYGONS_HPP_
#define SPATIO_TEMPORAL_VOXEL_LAYER__OBSTRUCTION_POLYGONS_HPP_

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <memory>
#include <optional>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include "rclcpp/rclcpp.hpp"

namespace geometry
{

// Polygons are supplied in the 2D angular domain, sorted by size, converted to
// the 3D directional domain, and used in the ObstructionFilter class to do
// fast point-in-polygon checks
// Polygon data pipeline: parse from parameters -> vertices to 3d directions ->
//                        3d directions to edge plane normals -> 3D normals to flat SoA.

// ---------------------------------------------------------------------------
// 2D / angular domain
// ---------------------------------------------------------------------------

struct SphericalPoint
{
  double azimuth;    // radians
  double elevation;  // radians
};
using AngularPolygon = std::vector<SphericalPoint>;

/**
 * @brief Angular area of a 2D (azimuth, elevation) polygon via the shoelace formula.
 */
inline double angularArea(const AngularPolygon & vertices)
{
  const size_t n = vertices.size();
  double area2 = 0.0;
  for (size_t i = 0; i < n; ++i) {
    const auto & v0 = vertices[i];
    const auto & v1 = vertices[(i + 1) % n];
    area2 += v0.azimuth * v1.elevation - v1.azimuth * v0.elevation;
  }
  return std::fabs(area2) * 0.5;
}

// ---------------------------------------------------------------------------
// 3D / directional domain
// ---------------------------------------------------------------------------

struct Vec3D
{
  double x, y, z;
};

/**
 * @brief The 3D/directional counterpart of the 2D angular polygon: a convex
 *        polyhedral cone through the origin, defined as the intersection of
 *        great-circle half-planes (one per polygon edge). Used for
 *        point-in-polygon tests on directions instead of points.
 */
class ConvexCone
{
public:
  /**
   * @brief Project angular (az/el) vertices onto the unit sphere and compute the
   *        great-circle half-plane normal for each edge.
   * @return the resulting cone, or std::nullopt if the vertices are too few, produce a
   *         degenerate edge, or are not spherically convex.
   */
  static std::optional<ConvexCone> fromAngularVertices(const AngularPolygon & vertices)
  {
    const size_t n = vertices.size();
    if (n < 3) {
      return std::nullopt;
    }

    // 2D -> 3D: convert (azimuth, elevation) vertices to 3D unit direction vectors
    // from the sensor origin.
    std::vector<Vec3D> dirs(n);
    for (size_t i = 0; i < n; ++i) {
      const double az = vertices[i].azimuth;
      const double el = vertices[i].elevation;
      const double cos_el = std::cos(el);
      dirs[i] = {cos_el * std::cos(az), cos_el * std::sin(az), std::sin(el)};
    }

    // Compute edge normals as cross products of adjacent direction vectors
    // The cross product will give the normal to the plane spanned by the two direction vectors.
    std::vector<Vec3D> normals(n);
    for (size_t i = 0; i < n; ++i) {
      size_t j = (i + 1) % n;  // index of next vertex
      normals[i].x = dirs[i].y * dirs[j].z - dirs[i].z * dirs[j].y;
      normals[i].y = dirs[i].z * dirs[j].x - dirs[i].x * dirs[j].z;
      normals[i].z = dirs[i].x * dirs[j].y - dirs[i].y * dirs[j].x;

      // Check for degenerate edge (coincident vertices)
      const double len_sq =
        normals[i].x * normals[i].x + normals[i].y * normals[i].y + normals[i].z * normals[i].z;

      constexpr double kMinEdgeNormSquared = 1e-12;  // reject edges < ~0.001° apart
      if (len_sq < kMinEdgeNormSquared) {
        return std::nullopt;
      }
    }

    // Determine orientation: centroid direction should be on inside of all half-planes
    Vec3D centroid{0.0, 0.0, 0.0};
    for (const auto & d : dirs) {
      centroid.x += d.x;
      centroid.y += d.y;
      centroid.z += d.z;
    }

    // flip normals if pointing outward (dot product with centroid < 0)
    const double test_dot =
      normals[0].x * centroid.x + normals[0].y * centroid.y + normals[0].z * centroid.z;
    if (test_dot < 0.0) {
      for (auto & norm : normals) {
        norm.x = -norm.x;
        norm.y = -norm.y;
        norm.z = -norm.z;
      }
    }

    // Verify spherical convexity. For each edge all vertices must lay on same side
    constexpr double kConvexEps = 1e-9;  // tolerance for vertices lying on an edge's line
    for (size_t i = 0; i < n; ++i) {
      for (size_t k = 0; k < n; ++k) {
        const double side =
          normals[i].x * dirs[k].x + normals[i].y * dirs[k].y + normals[i].z * dirs[k].z;
        if (side < -kConvexEps) {
          return std::nullopt;  // non-convex polygon: reject
        }
      }
    }

    return ConvexCone(std::move(normals));
  }

  const std::vector<Vec3D> & normals() const { return normals_; }

private:
  explicit ConvexCone(std::vector<Vec3D> normals) : normals_(std::move(normals)) {}

  std::vector<Vec3D> normals_;
};

// ---------------------------------------------------------------------------
// Parsing and validation
// ---------------------------------------------------------------------------

/**
 * @brief Validated 3D cone with angular area.
 */
struct ValidatedPolygon
{
  double angular_area;  // for size based sorting
  ConvexCone cone;
};

/**
 * @brief Perform validity checks for each polygon
 * @return vector of valid, precomputed polygons paired with their 2D angular area
 */
inline std::vector<ValidatedPolygon> validatePolygons(
  const std::vector<AngularPolygon> & input, const rclcpp::Logger & logger)
{
  std::vector<ValidatedPolygon> valid;
  valid.reserve(input.size());

  for (size_t idx = 0; idx < input.size(); ++idx) {
    const auto & vertices = input[idx];
    if (vertices.size() < 3) {
      RCLCPP_WARN(logger, "Obstruction polygon %zu has < 3 vertices, skipping.", idx);
      continue;
    }

    // Check for NaN/inf
    bool is_finite = false;
    for (const auto & v : vertices) {
      if (!std::isfinite(v.azimuth) || !std::isfinite(v.elevation)) {
        is_finite = true;
        break;
      }
    }
    if (is_finite) {
      RCLCPP_WARN(logger, "Obstruction polygon %zu has NaN/inf vertices, skipping.", idx);
      continue;
    }

    // Verify azimuth within bounds [0-2pi] and elevation within bounds [-pi/2, pi/2].
    bool azimuth_out_of_range = false;
    bool elevation_out_of_range = false;
    for (const auto & v : vertices) {
      if (v.azimuth < 0.0 || v.azimuth > 2.0 * M_PI) {
        azimuth_out_of_range = true;
        break;
      }
      if (v.elevation < -M_PI_2 || v.elevation > M_PI_2) {
        elevation_out_of_range = true;
        break;
      }
    }
    if (azimuth_out_of_range) {
      RCLCPP_WARN(
        logger,
        "Obstruction polygon %zu has vertices outside [0, 2pi] azimuth range, "
        "skipping. Polygons crossing the 0/2pi boundary are not supported.",
        idx);
      continue;
    }
    if (elevation_out_of_range) {
      RCLCPP_WARN(
        logger,
        "Obstruction polygon %zu has vertices outside [-pi/2, pi/2] elevation range, skipping.",
        idx);
      continue;
    }

    std::optional<ConvexCone> cone = ConvexCone::fromAngularVertices(vertices);
    if (!cone) {
      RCLCPP_ERROR(
        logger,
        "Obstruction polygon %zu failed precomputation (non-convex or degenerate), skipping.", idx);
      continue;
    }

    valid.push_back({angularArea(vertices), std::move(*cone)});
  }

  return valid;
}

/**
 * @brief Parse a list of polygons from a footprint-style string.
 *
 * Format: "[[x1,y1, x2,y2, x3,y3], [x4,y4, x5,y5, x6,y6]]"
 * Each inner [...] is one polygon with flat x/y vertex pairs.
 *
 * @return vector of parsed angular-domain polygons
 */
inline std::vector<AngularPolygon> parsePolygonsFromString(
  const std::string & input, const rclcpp::Logger & logger)
{
  std::vector<AngularPolygon> polygons;

  // Find the outer brackets
  size_t outer_start = input.find('[');
  size_t outer_end = input.rfind(']');
  if (
    outer_start == std::string::npos || outer_end == std::string::npos ||
    outer_end <= outer_start) {
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
      if (start == std::string::npos) {
        continue;
      }
      try {
        values.push_back(std::stod(token.substr(start)));
      } catch (const std::exception &) {
        RCLCPP_WARN(
          logger, "Obstruction polygon %d: failed to parse value '%s'.", poly_idx, token.c_str());
      }
    }

    if (values.size() >= 6 && values.size() % 2 == 0) {
      AngularPolygon vertices;
      vertices.reserve(values.size() / 2);
      for (size_t i = 0; i < values.size(); i += 2) {
        vertices.push_back({values[i], values[i + 1]});
      }
      polygons.push_back(std::move(vertices));
    } else if (!values.empty()) {
      RCLCPP_WARN(
        logger, "Obstruction polygon %d needs >= 6 values (3 x/y pairs), got %zu. Skipping.",
        poly_idx, values.size());
    }

    poly_idx++;
    pos = inner_end + 1;
  }

  return polygons;
}

/**
 * @brief Flat, contiguous Structure-of-Arrays (SoA) store of obstruction polygon
 *        3D edge normals, queried via isObstructed() on the frustum point-clearing
 *        hot path.
 */
class ObstructionFilter
{
public:
  ObstructionFilter() = default;

  bool empty() const { return polygons_.empty(); }

  /**
   * @brief Check if a 3D direction falls inside any obstruction polygon
   */
  inline bool isObstructed(float x, float y, float z) const
  {
    if (polygons_.empty()) return false;

    const float * nx = nx_.data();
    const float * ny = ny_.data();
    const float * nz = nz_.data();

    // to be obstructed (in polygon), each dot product of direction with polygon normals should
    // be positive. If there is any negative dot product, this direction is not obstructed
    for (const auto & span : polygons_) {
      const uint32_t end = span.start + span.count;
      int any_negative = 0;
      for (uint32_t i = span.start; i < end; ++i) {
        const float dot = nx[i] * x + ny[i] * y + nz[i] * z;
        any_negative = any_negative | (dot < 0.0f);
      }
      if (!any_negative) {  // if all positive, direction is obstructed by this polygon
        return true;
      }
    }
    return false;
  }

  /**
   * @brief Parse obstruction polygons from a single ROS2 string parameter.
   *
   * Expected parameter format (footprint-style):
   *   obstruction_polygons: "[[x1,y1, x2,y2, x3,y3], [x4,y4, x5,y5, x6,y6]]"
   *   x=azimuth (rad), y=elevation (rad).
   */
  template <typename NodeT>
  static std::shared_ptr<ObstructionFilter> fromParams(
    NodeT node, const std::string & param_prefix, const rclcpp::Logger & logger)
  {
    std::string param_name = param_prefix + ".obstruction_polygons";
    std::string polygons_str;
    if (!node->has_parameter(param_name)) {
      node->declare_parameter(param_name, std::string(""));
    }
    node->get_parameter(param_name, polygons_str);

    if (polygons_str.empty()) {
      return nullptr;
    }

    auto raw_polygons = parsePolygonsFromString(polygons_str, logger);

    if (!raw_polygons.empty()) {
      RCLCPP_INFO(
        logger, "Parsed %zu obstruction polygon(s) for %s", raw_polygons.size(),
        param_prefix.c_str());
    }

    auto valid_polygons = validatePolygons(raw_polygons, logger);
    if (valid_polygons.empty()) {
      return nullptr;
    }

    auto filter = std::make_shared<ObstructionFilter>();
    filter->flattenAndSortPolygons(valid_polygons);
    return filter;
  }

private:
  // Span to indicate where each polygon's normals are in the vectors
  struct Span
  {
    uint32_t start;
    uint32_t count;
  };

  // Flat, contiguous Structure-of-Arrays (SoA) storage of all obstruction
  // polygon edge normals, for fast point in polygon testing.
  std::vector<float> nx_;
  std::vector<float> ny_;
  std::vector<float> nz_;
  std::vector<Span> polygons_;

  /**
   * @brief Convert validated cones to flat SoA, largest angular area first.
   */
  void flattenAndSortPolygons(const std::vector<ValidatedPolygon> & polygons)
  {
    // Index polygons, largest-area first
    std::vector<const ValidatedPolygon *> ordered;
    ordered.reserve(polygons.size());
    for (const auto & poly : polygons) {
      ordered.push_back(&poly);
    }
    std::sort(
      ordered.begin(), ordered.end(), [](const ValidatedPolygon * a, const ValidatedPolygon * b) {
        return a->angular_area > b->angular_area;
      });

    // Store normals in flat vectors
    size_t total_edges = 0;
    for (const auto * poly : ordered) {
      total_edges += poly->cone.normals().size();
    }
    nx_.reserve(total_edges);
    ny_.reserve(total_edges);
    nz_.reserve(total_edges);
    polygons_.reserve(ordered.size());

    for (const auto * poly : ordered) {
      Span span;
      span.start = static_cast<uint32_t>(nx_.size());
      span.count = static_cast<uint32_t>(poly->cone.normals().size());
      for (const auto & normal : poly->cone.normals()) {
        nx_.push_back(static_cast<float>(normal.x));
        ny_.push_back(static_cast<float>(normal.y));
        nz_.push_back(static_cast<float>(normal.z));
      }
      polygons_.push_back(span);
    }
  }
};

}  // namespace geometry

#endif  // SPATIO_TEMPORAL_VOXEL_LAYER__OBSTRUCTION_POLYGONS_HPP_
