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
 * Author: Steve Macenski (steven.macenski@simberobotics.com)
 * Purpose: Abstract class for a frustum model implementation
 *********************************************************************/

#ifndef FRUSTUM_H_
#define FRUSTUM_H_

// Eigen
#include <Eigen/Geometry>
// STL
#include <vector>
#include <cassert>
// OpenVDB
#include <openvdb/openvdb.h>
// msgs
#include <geometry_msgs/Point.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Pose.h>
// ROS
#include <ros/ros.h>

namespace geometry
{

// A structure for maintaining vectors and points in world spaces
struct VectorWithPt3D
{
  VectorWithPt3D(const double& x_, const double& y_, \
                 const double& z_, const Eigen::Vector3d& p0) : \
                 x(x_), y(y_), z(z_), initial_point(p0)
  {
  }
  
  VectorWithPt3D(void) : x(0.), y(0.), z(0.)
  {
  }

  inline VectorWithPt3D operator*(double a)
  {
    return VectorWithPt3D(a*x, a*y, a*z, initial_point);
  }

  // given a transform, transform its information
  void TransformFrames(const Eigen::Affine3d& homogeneous_transform)
  {
    Eigen::Vector3d vec_t = homogeneous_transform.rotation() * Eigen::Vector3d(x,y,z);
    vec_t.normalize();
    x = vec_t[0]; y = vec_t[1]; z = vec_t[2];
    initial_point = homogeneous_transform * initial_point;
    return;
  }

  double x, y, z;
  Eigen::Vector3d initial_point;
};

// A class to model a depth sensor frustum in world space
class Frustum
{
public:
  Frustum() {};
  virtual ~Frustum(void) {};

  // determine if a point is inside of the transformed frustum
  virtual bool IsInside(const openvdb::Vec3d& pt) = 0;

  // set pose of depth camera in global space
  virtual void SetPosition(const geometry_msgs::Point& origin) = 0;
  virtual void SetOrientation(const geometry_msgs::Quaternion& quat) = 0;

  // transform model to the current coordinates
  virtual void TransformModel(void) = 0;

private:
  Eigen::Vector3d _position;
  Eigen::Quaterniond _orientation;
  bool _valid_frustum;
};

} // end namespace

#endif
