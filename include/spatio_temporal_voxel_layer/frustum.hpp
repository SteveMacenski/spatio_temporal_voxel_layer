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
 *   * Neither the name of Willow Garage, Inc. nor the names of its
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
 * Author: Steve Macenski (steven.macenski@simberobotics.com)
 * Purpose: Structure for handling camera FOVs to construct frustums
 *          and associated methods
 *********************************************************************/

#ifndef FRUSTUM_H_
#define FRUSTUM_H_

// Eigen
#include <Eigen/Geometry>
// STL
#include <vector>
// OpenVDB
#include <openvdb/openvdb.h>
// msgs
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>

namespace geometry
{

struct Vector3D
{
  Vector3D(const double& x_, const double& y_, const double& z_) : x(x_), y(y_), z(z_)
  {}
  Vector3D(void) : x(), y(), z()
  {}
  void TransformFrames(const Eigen::Affine3d& homogeneous_transform)
  {
    Eigen::Vector3d vec_t = homogeneous_transform * Eigen::Vector3d(x,y,z);
    x = vec_t[0]; y = vec_t[1]; z = vec_t[2];
  }

  double x, y, z;
};

class Frustum
{
public:
  Frustum(const double& vFOV, const double& hFOV, const double& min_dist, const double& max_dist);
  ~Frustum(void);

  void TransformPlaneNormals(void);
  bool IsInside(const openvdb::Vec3d& pt);

  void SetPosition(const geometry_msgs::Point& origin);
  void SetOrientation(const geometry_msgs::Quaternion& quat);

private:
  void ComputePlaneNormals(void);
  bool Dot(const Vector3D&, const openvdb::Vec3d&) const;

  double _vFOV, _hFOV, _min_d, _max_d;
  std::vector<Vector3D> _plane_normals;
  Eigen::Vector3d _position;
  Eigen::Quaterniond _orientation;
};

} // end namespace

#endif
