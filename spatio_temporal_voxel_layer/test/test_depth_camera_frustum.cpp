// Copyright 2026 A Gudamu
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <cmath>
#include <tuple>

#include "gtest/gtest.h"
#include "spatio_temporal_voxel_layer/frustum_models/depth_camera_frustum.hpp"

// vertical FOV, horizontal FOV [deg], min and max depth [m]
class FrustumTest
  : public ::testing::TestWithParam<std::tuple<double, double, double, double>>
{
};

// A pinhole camera looking along +z sees a point when its depth is between
// min and max and its image coordinates x/z, y/z are within the FOV.
TEST_P(FrustumTest, matchesPinholeModel)
{
  const double vfov = std::get<0>(GetParam()) * M_PI / 180.;
  const double hfov = std::get<1>(GetParam()) * M_PI / 180.;
  const double min_d = std::get<2>(GetParam());
  const double max_d = std::get<3>(GetParam());
  const double tan_h = std::tan(hfov / 2.);
  const double tan_v = std::tan(vfov / 2.);

  geometry::DepthCameraFrustum frustum(vfov, hfov, min_d, max_d);
  geometry_msgs::msg::Point origin;
  geometry_msgs::msg::Quaternion identity;
  identity.w = 1.0;
  frustum.SetPosition(origin);
  frustum.SetOrientation(identity);
  frustum.TransformModel();

  const double step = 0.0937;  // not a divisor of the bounds
  const double margin = 0.01;  // skip points this close to a face
  int checked = 0, wrong_inside = 0, wrong_outside = 0;
  for (double z = -1.0; z < max_d + 1.0; z += step) {
    for (double x = -1.5 * max_d * tan_h; x < 1.5 * max_d * tan_h; x += step) {
      for (double y = -1.5 * max_d * tan_v; y < 1.5 * max_d * tan_v; y += step) {
        const double dx = tan_h * z - std::fabs(x);
        const double dy = tan_v * z - std::fabs(y);
        const double dz = std::min(z - min_d, max_d - z);
        if (std::fabs(dx) < margin || std::fabs(dy) < margin || std::fabs(dz) < margin) {
          continue;
        }
        const bool expected = dx > 0. && dy > 0. && dz > 0.;
        const bool inside = frustum.IsInside(openvdb::Vec3d(x, y, z));
        checked++;
        wrong_inside += inside && !expected;
        wrong_outside += !inside && expected;
      }
    }
  }
  EXPECT_GT(checked, 1000);
  EXPECT_EQ(wrong_inside, 0) << "points outside the camera's view counted as inside";
  EXPECT_EQ(wrong_outside, 0) << "points in the camera's view counted as outside";
}

INSTANTIATE_TEST_SUITE_P(
  Cameras, FrustumTest,
  ::testing::Values(
    std::make_tuple(100., 100., 0.3, 3.0),
    std::make_tuple(58., 87., 0.3, 3.0),
    std::make_tuple(40., 60., 0.1, 7.0)));
