/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2026, Ardavan
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following disclaimer
 *     in the documentation and/or other materials provided with the
 *     distribution.
 *   * Neither the name of the copyright holder nor the names of its
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

#include <gtest/gtest.h>

#include "spatio_temporal_voxel_layer/frustum_models/three_dimensional_lidar_frustum.hpp"

namespace
{

geometry_msgs::msg::Quaternion IdentityOrientation()
{
  geometry_msgs::msg::Quaternion orientation;
  orientation.w = 1.0;
  return orientation;
}

geometry::ThreeDimensionalLidarFrustum MakeFrustum(
  const double roll = 0.0, const double pitch = 0.0, const double yaw = 0.0)
{
  geometry::ThreeDimensionalLidarFrustum frustum(
    1.0, 0.0, 0.0, 1.0, 0.1, 10.0, roll, pitch, yaw);
  geometry_msgs::msg::Point origin;
  frustum.SetPosition(origin);
  frustum.SetOrientation(IdentityOrientation());
  frustum.TransformModel();
  return frustum;
}

TEST(ThreeDimensionalLidarFrustum, DefaultsToPositiveXForward)
{
  auto frustum = MakeFrustum();

  EXPECT_TRUE(frustum.IsInside(openvdb::Vec3d(2.0, 0.0, 0.0)));
  EXPECT_FALSE(frustum.IsInside(openvdb::Vec3d(0.0, 2.0, 0.0)));
  EXPECT_FALSE(frustum.IsInside(openvdb::Vec3d(-2.0, 0.0, 0.0)));
}

TEST(ThreeDimensionalLidarFrustum, YawCanSelectPositiveYForward)
{
  auto frustum = MakeFrustum(0.0, 0.0, 1.5707963267948966);

  EXPECT_TRUE(frustum.IsInside(openvdb::Vec3d(0.0, 2.0, 0.0)));
  EXPECT_FALSE(frustum.IsInside(openvdb::Vec3d(2.0, 0.0, 0.0)));
}

TEST(ThreeDimensionalLidarFrustum, YawCanSelectAnIntermediateDirection)
{
  auto frustum = MakeFrustum(0.0, 0.0, 0.7853981633974483);

  EXPECT_TRUE(frustum.IsInside(openvdb::Vec3d(2.0, 2.0, 0.0)));
  EXPECT_FALSE(frustum.IsInside(openvdb::Vec3d(2.0, -2.0, 0.0)));
}

TEST(ThreeDimensionalLidarFrustum, PitchCanSelectAThreeDimensionalDirection)
{
  auto frustum = MakeFrustum(0.0, -0.7853981633974483, 0.0);

  EXPECT_TRUE(frustum.IsInside(openvdb::Vec3d(2.0, 0.0, 2.0)));
  EXPECT_FALSE(frustum.IsInside(openvdb::Vec3d(2.0, 0.0, -2.0)));
}

}  // namespace
