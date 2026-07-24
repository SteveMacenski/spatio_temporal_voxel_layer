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

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include "rclcpp/rclcpp.hpp"

namespace geometry
{

struct SphericalPoint
{
  double azimuth;    // radians
  double elevation;  // radians
};

struct Vec3D
{
  double x, y, z;
};

/**
 * @brief A convex polygon defined in (azimuth, elevation) space with precomputed
 *        3D great-circle normals for spherical point-in-polygon testing via Cartesian dot products.
 *
 * Vertices are stored as (azimuth, elevation) for parsing/validation/serialization.
 * At precompute time, they are converted to 3D unit direction vectors and each edge
 * becomes a great-circle plane through the origin. The runtime isInside() test uses
 * only dot products on 3D points.
 *
 */
struct ConvexPolygon2D
{
  // Vertices as angular coordinates in radians
  std::vector<SphericalPoint> vertices;

  // Precomputed 3D great-circle plane normals (one per edge)
  std::vector<Vec3D> normals;

  /**
   * @brief Convert angular (az/el) vertices to 3D directions and precompute edge normals.
   * @return true if polygon is valid (convex, >= 3 vertices, no degenerate edges)
   */
  bool precompute()
  {
    size_t n = vertices.size();
    if (n < 3) {
      return false;
    }

    // Convert (azimuth, elevation) vertices to 3D unit direction vectors from sensor origin
    std::vector<Vec3D> dirs(n);
    for (size_t i = 0; i < n; ++i) {
      double az = vertices[i].azimuth;
      double el = vertices[i].elevation;
      // Convert spherical to Cartesian coordinates
      double cos_el = std::cos(el);
      dirs[i] = {cos_el * std::cos(az), cos_el * std::sin(az), std::sin(el)};
    }

    // Compute edge normals as cross products of adjacent direction vectors
    normals.resize(n);
    for (size_t i = 0; i < n; ++i) {
      size_t j = (i + 1) % n;  // index of next vertex
      normals[i].x = dirs[i].y * dirs[j].z - dirs[i].z * dirs[j].y;
      normals[i].y = dirs[i].z * dirs[j].x - dirs[i].x * dirs[j].z;
      normals[i].z = dirs[i].x * dirs[j].y - dirs[i].y * dirs[j].x;

      // Check for degenerate edge (coincident vertices)
      double len_sq =
        normals[i].x * normals[i].x + normals[i].y * normals[i].y + normals[i].z * normals[i].z;

      constexpr double kMinEdgeNormSquared = 1e-12;  // reject edges < ~0.001° apart
      if (len_sq < kMinEdgeNormSquared) {
        return false;
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
    double test_dot =
      normals[0].x * centroid.x + normals[0].y * centroid.y + normals[0].z * centroid.z;
    if (test_dot < 0.0) {
      for (auto & norm : normals) {
        norm.x = -norm.x;
        norm.y = -norm.y;
        norm.z = -norm.z;
      }
    }

    // Verify spherical convexity. isInside()'s sequential cross product needs spherical convexity to be valid.
    // Check that all vertices are on the same side of each edge's great-circle plane.
    constexpr double kConvexEps = 1e-9;  // tolerance for vertices lying on an edge's line
    for (size_t i = 0; i < n; ++i) {
      for (size_t k = 0; k < n; ++k) {
        double side =
          normals[i].x * dirs[k].x + normals[i].y * dirs[k].y + normals[i].z * dirs[k].z;
        if (side < -kConvexEps) {
          return false;  // non-convex polygon: reject rather than mis-test
        }
      }
    }

    return true;
  }

  /**
   * @brief Check if a 3D direction is inside this polygon.
   * Takes the raw cartesian direction from sensor origin, no normalization needed
   * since dot-product sign is scale-invariant.
   */
  bool isInside(double x, double y, double z) const
  {
    for (const auto & n : normals) {
      // if any of the dot-product tests fail, early return false
      if (n.x * x + n.y * y + n.z * z < 0.0) {
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
  const std::vector<ConvexPolygon2D> & input, const rclcpp::Logger & logger)
{
  std::vector<ConvexPolygon2D> valid;
  valid.reserve(input.size());

  for (size_t idx = 0; idx < input.size(); ++idx) {
    auto poly = input[idx];
    if (poly.vertices.size() < 3) {
      RCLCPP_WARN(logger, "Obstruction polygon %zu has < 3 vertices, skipping.", idx);
      continue;
    }

    // Check for NaN
    bool has_nan = false;
    for (const auto & v : poly.vertices) {
      if (!std::isfinite(v.azimuth) || !std::isfinite(v.elevation)) {
        has_nan = true;
        break;
      }
    }
    if (has_nan) {
      RCLCPP_WARN(logger, "Obstruction polygon %zu has NaN/inf vertices, skipping.", idx);
      continue;
    }

    // Check azimuth bounds — warn if any vertex outside [0, 2π]
    bool out_of_range = false;
    for (const auto & v : poly.vertices) {
      if (v.azimuth < 0.0 || v.azimuth > 2.0 * M_PI) {
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
      RCLCPP_ERROR(
        logger,
        "Obstruction polygon %zu failed precomputation (non-convex or degenerate), skipping.", idx);
      continue;
    }

    valid.push_back(std::move(poly));
  }

  return valid;
}

/**
 * @brief Check if a 3D direction falls inside any of the obstruction polygons.
 * Takes raw cartesian direction (x, y, z) from sensor frame.
 */
inline bool isInsideAnyObstruction(
  const std::vector<ConvexPolygon2D> & polygons, double x, double y, double z)
{
  for (const auto & poly : polygons) {
    if (poly.isInside(x, y, z)) {
      return true;
    }
  }
  return false;
}

/**
 * @brief Flat, contiguous Structure-of-Arrays (SoA) storage of all obstruction
 *        polygon edge normals, for fast point in polygon testing.
 */
struct ObstructionField
{
  // All edge normals for all polygons, contiguous, SoA
  std::vector<float> nx;
  std::vector<float> ny;
  std::vector<float> nz;

  // Span to indicate where each polygon's normals are in the vectors
  struct Span
  {
    uint32_t start;
    uint32_t count;
  };
  std::vector<Span> polygons;

  bool empty() const { return polygons.empty(); }
};

/**
 * @brief Angular area size of a polygon using shoelace formula
 */
inline double polygonAngularArea(const ConvexPolygon2D & p)
{
  const size_t n = p.vertices.size();
  double area2 = 0.0;
  for (size_t i = 0; i < n; ++i) {
    const auto & v0 = p.vertices[i];
    const auto & v1 = p.vertices[(i + 1) % n];
    area2 += v0.azimuth * v1.elevation - v1.azimuth * v0.elevation;
  }
  return std::fabs(area2) * 0.5;
}

/**
 * @brief Convert validated/precomputed polygons to flat SoA and sort by angular area
 */
inline ObstructionField flattenAndSortPolygons(const std::vector<ConvexPolygon2D> & polygons)
{
  ObstructionField field;

  // Index polygons, largest-area first
  std::vector<const ConvexPolygon2D *> poly2d_ordered;
  poly2d_ordered.reserve(polygons.size());
  for (const auto & poly : polygons) {
    poly2d_ordered.push_back(&poly);
  }
  std::sort(
    poly2d_ordered.begin(), poly2d_ordered.end(),
    [](const ConvexPolygon2D * a, const ConvexPolygon2D * b) {
      return polygonAngularArea(*a) > polygonAngularArea(*b);
    });

  // Store normals in flat vectors
  size_t total_edges = 0;
  for (const auto * poly : poly2d_ordered) {
    total_edges += poly->normals.size();
  }
  field.nx.reserve(total_edges);
  field.ny.reserve(total_edges);
  field.nz.reserve(total_edges);
  field.polygons.reserve(poly2d_ordered.size());

  for (const auto * poly : poly2d_ordered) {
    ObstructionField::Span span;
    span.start = static_cast<uint32_t>(field.nx.size());
    span.count = static_cast<uint32_t>(poly->normals.size());
    for (const auto & normal : poly->normals) {
      field.nx.push_back(static_cast<float>(normal.x));
      field.ny.push_back(static_cast<float>(normal.y));
      field.nz.push_back(static_cast<float>(normal.z));
    }
    field.polygons.push_back(span);
  }

  return field;
}

/**
 * @brief Check if a 3D direction falls inside any obstruction polygon
 */
inline bool isInsideAnyObstruction(const ObstructionField & field, float x, float y, float z)
{
  const float * nx = field.nx.data();
  const float * ny = field.ny.data();
  const float * nz = field.nz.data();

  for (const auto & span : field.polygons) {
    const uint32_t end = span.start + span.count;
    int any_negative = 0;
    for (uint32_t i = span.start; i < end; ++i) {
      const float dot = nx[i] * x + ny[i] * y + nz[i] * z;
      any_negative = any_negative | (dot < 0.0f);
    }
    if (!any_negative) {
      return true;  // inside this polygon -> inside some obstruction
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
  const std::string & input, const rclcpp::Logger & logger)
{
  std::vector<ConvexPolygon2D> polygons;

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
      ConvexPolygon2D poly;
      for (size_t i = 0; i < values.size(); i += 2) {
        poly.vertices.push_back({values[i], values[i + 1]});
      }
      polygons.push_back(std::move(poly));
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
 * @brief Parse obstruction polygons from a single ROS2 string parameter.
 *
 * Expected parameter format (footprint-style):
 *   obstruction_polygons: "[[x1,y1, x2,y2, x3,y3], [x4,y4, x5,y5, x6,y6]]"
 *
 * For STVL use, x=azimuth (rad), y=elevation (rad).
 */
template <typename NodeT>
std::vector<ConvexPolygon2D> parseObstructionPolygonsFromParams(
  NodeT node, const std::string & param_prefix, const rclcpp::Logger & logger)
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
      logger, "Parsed %zu obstruction polygon(s) for %s", polygons.size(), param_prefix.c_str());
  }

  return validatePolygons(polygons, logger);
}

}  // namespace geometry

#endif  // SPATIO_TEMPORAL_VOXEL_LAYER__OBSTRUCTION_POLYGONS_HPP_
