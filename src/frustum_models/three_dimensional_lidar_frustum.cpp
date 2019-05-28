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

#define CONEPADDING 0.0 //This value shifts the frustum "cone" outwards without shifting its angle.

namespace geometry
{

/*****************************************************************************/
ThreeDimensionalLidarFrustum::ThreeDimensionalLidarFrustum(const double &vFOV,
                                                           const double &hFOV,
                                                           const double &min_dist,
                                                           const double &max_dist) : _vFOV(vFOV),
                                                                                     _hFOV(hFOV), _min_d(min_dist),
                                                                                     _max_d(max_dist)
/*****************************************************************************/
{
  _valid_frustum = true;
  ros::NodeHandle nh;

  _hFOVhalf = _hFOV / 2.0;

  _tan_vFOVhalf = tan(_vFOV / 2.0);
  _tan_vFOVhalf_squared = _tan_vFOVhalf * _tan_vFOVhalf;
  _min_d_squared = _min_d * _min_d;
  _max_d_squared = _max_d * _max_d;
  if (_hFOV >= 6.28)
  {
    _full_hFOV = true;
  }
  else{
    _full_hFOV = false;
  }
}

/*****************************************************************************/
ThreeDimensionalLidarFrustum::~ThreeDimensionalLidarFrustum(void)
/*****************************************************************************/
{
}

/*****************************************************************************/
void ThreeDimensionalLidarFrustum::TransformModel(void)
/*****************************************************************************/
{
  _orientation_conjugate = _orientation.conjugate();
}

/*****************************************************************************/
bool ThreeDimensionalLidarFrustum::IsInside(const openvdb::Vec3d &pt)
/*****************************************************************************/
{

  Eigen::Vector3d point_in_global_frame(pt[0], pt[1], pt[2]);
  Eigen::Vector3d transformed_point =
      _orientation_conjugate * (point_in_global_frame - _position);

  double radial_distance_squared = ((transformed_point[0] * transformed_point[0]) + (transformed_point[1] * transformed_point[1]));

  // Check if inside frustum valid range
  if (radial_distance_squared > _max_d_squared || radial_distance_squared < _min_d_squared)
  {
    return false;
  }

  // // Check if inside frustum valid vFOV
  if (((fabs(transformed_point[2]) + CONEPADDING) * (fabs(transformed_point[2]) + CONEPADDING) / radial_distance_squared) > _tan_vFOVhalf_squared)
  {
    return false;
  }

  // Check if inside frustum valid hFOV, unless hFOV is full-circle (360 degree)
  if (!_full_hFOV)
  {
    if (transformed_point[0] > 0)
    {
      if (fabs(atan(transformed_point[1] / transformed_point[0])) > _hFOVhalf)
      {
        return false;
      }
    }
    else
    {
      if ((fabs(atan(transformed_point[0] / transformed_point[1])) + 1.570796) > _hFOVhalf)
      {
        return false;
      }
    }
  }

  return true;
}

/*****************************************************************************/
void ThreeDimensionalLidarFrustum::SetPosition(const geometry_msgs::Point &origin)
/*****************************************************************************/
{
  _position = Eigen::Vector3d(origin.x, origin.y, origin.z);
}

/*****************************************************************************/
void ThreeDimensionalLidarFrustum::SetOrientation(const geometry_msgs::Quaternion &quat)
/*****************************************************************************/
{
  _orientation = Eigen::Quaterniond(quat.w, quat.x, quat.y, quat.z);
}

/*****************************************************************************/
double ThreeDimensionalLidarFrustum::Dot(const VectorWithPt3D &plane_pt,
                                         const openvdb::Vec3d &query_pt) const
/*****************************************************************************/
{
  return plane_pt.x * query_pt[0] + plane_pt.y * query_pt[1] + plane_pt.z * query_pt[2];
}

/*****************************************************************************/
double ThreeDimensionalLidarFrustum::Dot(const VectorWithPt3D &plane_pt,
                                         const Eigen::Vector3d &query_pt) const
/*****************************************************************************/
{
  return plane_pt.x * query_pt[0] + plane_pt.y * query_pt[1] + plane_pt.z * query_pt[2];
}

} // namespace geometry
