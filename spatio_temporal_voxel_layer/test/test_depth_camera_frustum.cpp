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
 * Author: A Gudamu
 *********************************************************************/

#include <cmath>
#include <tuple>

#include <Eigen/Geometry>

#include "gtest/gtest.h"
#include "spatio_temporal_voxel_layer/frustum_models/depth_camera_frustum.hpp"

namespace
{

// Camera pose in the world: position (x, y, z) [m] and roll, pitch, yaw [deg]
struct Pose
{
  double x, y, z, roll, pitch, yaw;
};

// Checks IsInside() against a pinhole camera that looks along its +z axis:
// a point is in view when its depth is between min and max and its image
// coordinates x/z, y/z are within the FOV. The points are sampled in the
// camera frame and moved to the world with the camera pose, so a non-identity
// pose exercises TransformModel().
void ExpectMatchesPinholeModel(
  double vfov_deg, double hfov_deg, double min_d, double max_d, const Pose & pose)
{
  const double vfov = vfov_deg * M_PI / 180.;
  const double hfov = hfov_deg * M_PI / 180.;
  const double tan_h = std::tan(hfov / 2.);
  const double tan_v = std::tan(vfov / 2.);

  const Eigen::Quaterniond q =
    Eigen::AngleAxisd(pose.yaw * M_PI / 180., Eigen::Vector3d::UnitZ()) *
    Eigen::AngleAxisd(pose.pitch * M_PI / 180., Eigen::Vector3d::UnitY()) *
    Eigen::AngleAxisd(pose.roll * M_PI / 180., Eigen::Vector3d::UnitX());
  const Eigen::Vector3d t(pose.x, pose.y, pose.z);

  geometry::DepthCameraFrustum frustum(vfov, hfov, min_d, max_d);
  geometry_msgs::msg::Point position;
  position.x = pose.x;
  position.y = pose.y;
  position.z = pose.z;
  geometry_msgs::msg::Quaternion orientation;
  orientation.x = q.x();
  orientation.y = q.y();
  orientation.z = q.z();
  orientation.w = q.w();
  frustum.SetPosition(position);
  frustum.SetOrientation(orientation);
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
        const Eigen::Vector3d w = q * Eigen::Vector3d(x, y, z) + t;
        const bool inside = frustum.IsInside(openvdb::Vec3d(w.x(), w.y(), w.z()));
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

}  // namespace

// vertical FOV, horizontal FOV [deg], min and max depth [m]
class FrustumTest
  : public ::testing::TestWithParam<std::tuple<double, double, double, double>>
{
};

TEST_P(FrustumTest, matchesPinholeModelAtIdentityPose)
{
  const auto [vfov, hfov, min_d, max_d] = GetParam();
  ExpectMatchesPinholeModel(vfov, hfov, min_d, max_d, Pose{0., 0., 0., 0., 0., 0.});
}

TEST_P(FrustumTest, matchesPinholeModelAtTransformedPose)
{
  const auto [vfov, hfov, min_d, max_d] = GetParam();
  // translated and rotated about all three axes
  ExpectMatchesPinholeModel(vfov, hfov, min_d, max_d, Pose{1.5, -0.7, 0.4, 20., -35., 120.});
  // optical frame of a camera mounted on a robot looking along the robot's +x
  ExpectMatchesPinholeModel(vfov, hfov, min_d, max_d, Pose{0.2, 0., 1.1, -90., 0., -90.});
}

INSTANTIATE_TEST_SUITE_P(
  Cameras, FrustumTest,
  ::testing::Values(
    std::make_tuple(100., 100., 0.3, 3.0),
    std::make_tuple(58., 87., 0.3, 3.0),
    std::make_tuple(40., 60., 0.1, 7.0)));
