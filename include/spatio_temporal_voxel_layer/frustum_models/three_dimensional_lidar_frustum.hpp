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
 * Purpose: Structure for handling 3D Lidar FOVs to construct frustums
 *          and associated methods
 *********************************************************************/

#ifndef THREE_DIMENSIONAL_LIDAR_FRUSTUM_H_
#define THREE_DIMENSIONAL_LIDAR_FRUSTUM_H_

// STVL
#include <spatio_temporal_voxel_layer/frustum_models/frustum.hpp>
// M_PI
#include <cmath>

namespace geometry
{

// A class to model a spinning 3D Lidar frustum in world space
class ThreeDimensionalLidarFrustum : public Frustum
{
public:
  ThreeDimensionalLidarFrustum(const double& vFOV, const double& vFOVPadding, 
          const double& hFOV, const double& min_dist, const double& max_dist);
  virtual ~ThreeDimensionalLidarFrustum(void);

  // Does nothing in 3D lidar model
  virtual void TransformModel(void);

  // determine if a point is inside of the transformed frustum
  virtual bool IsInside(const openvdb::Vec3d& pt);

  // set pose of 3d lidar in global space
  virtual void SetPosition(const geometry_msgs::Point& origin);
  virtual void SetOrientation(const geometry_msgs::Quaternion& quat);

private:
  // utils to find useful frustum metadata
  double Dot(const VectorWithPt3D&, const openvdb::Vec3d&) const;
  double Dot(const VectorWithPt3D&, const Eigen::Vector3d&) const;

  double _vFOV, _vFOVPadding, _hFOV, _min_d, _max_d;
  double _hFOVhalf;
  double _min_d_squared, _max_d_squared;
  double _tan_vFOVhalf;
  double _tan_vFOVhalf_squared;
  Eigen::Vector3d _position;
  Eigen::Quaterniond _orientation;
  Eigen::Quaterniond _orientation_conjugate;
  bool _valid_frustum;
  bool _full_hFOV;

};

} // end namespace

#endif
