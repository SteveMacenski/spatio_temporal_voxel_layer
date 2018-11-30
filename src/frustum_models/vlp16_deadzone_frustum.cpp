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
 *********************************************************************/

#include <spatio_temporal_voxel_layer/frustum_models/vlp16_deadzone_frustum.hpp>

namespace geometry
{

/*****************************************************************************/
VLP16DeadzoneFrustum::VLP16DeadzoneFrustum(const double &vFOV, const double &hFOV,
                                                           const double &min_dist, const double &max_dist) : _vFOV(vFOV), _hFOV(hFOV), _min_d(min_dist), _max_d(max_dist)
/*****************************************************************************/
{
  // ROS_INFO("Using VLP16 Frustum");
  // _valid_frustum = false;
  _valid_frustum = true;
  ros::NodeHandle nh;
#if VISUALIZE_FRUSTUM
  _frustumPub = nh.advertise<visualization_msgs::MarkerArray>("/frustum", 1);
  // give enough time for publisher to register, don't use in production.
  ros::Duration(0.5).sleep();
#endif

_vFOVhalf = _vFOV / 2.0;
  // this->ComputePlaneNormals();
}

/*****************************************************************************/
VLP16DeadzoneFrustum::~VLP16DeadzoneFrustum(void)
/*****************************************************************************/
{
}

/*****************************************************************************/
void VLP16DeadzoneFrustum::ComputePlaneNormals(void)
/*****************************************************************************/
{

}

/*****************************************************************************/
void VLP16DeadzoneFrustum::TransformModel(void)
/*****************************************************************************/
{ return;
}

/*****************************************************************************/
bool VLP16DeadzoneFrustum::IsInside(const openvdb::Vec3d &pt)
/*****************************************************************************/
{

  /*/ Unrotated Cylinder
  float local_x = pt[0] - _position[0];
  float local_y = pt[1] - _position[1];
  float radial_distance = sqrt((local_x * local_x) + (local_y * local_y));

  if (radial_distance > (3))
  {
    return false;
  }
  /*/

  //VLP Frustum
  double local_x = pt[0] - _position[0];
  double local_y = pt[1] - _position[1];
  // float local_z = pt[2] - _position[2];

  double radial_distance = sqrt( (local_x * local_x) + (local_y * local_y) );

  // Keep if inside minimum distance or beyond maximum range)
  if (radial_distance > _max_d || radial_distance < _min_d )
  {
    return false;
  }
  
  // Keep if outside vFOV
  if (fabs(atan((pt[2] - _position[2])/radial_distance)) > _vFOVhalf)
  {
    return false;
  } 

  return true;
}

/*****************************************************************************/
void VLP16DeadzoneFrustum::SetPosition(const geometry_msgs::Point &origin)
/*****************************************************************************/
{
  _position = Eigen::Vector3d(origin.x, origin.y, origin.z);
}

/*****************************************************************************/
void VLP16DeadzoneFrustum::SetOrientation(const geometry_msgs::Quaternion &quat)
/*****************************************************************************/
{
  _orientation = Eigen::Quaterniond(quat.w, quat.x, quat.y, quat.z);
}

/*****************************************************************************/
double VLP16DeadzoneFrustum::Dot(const VectorWithPt3D &plane_pt,
                                         const openvdb::Vec3d &query_pt) const
/*****************************************************************************/
{
  return plane_pt.x * query_pt[0] + plane_pt.y * query_pt[1] + plane_pt.z * query_pt[2];
}

/*****************************************************************************/
double VLP16DeadzoneFrustum::Dot(const VectorWithPt3D &plane_pt,
                                         const Eigen::Vector3d &query_pt) const
/*****************************************************************************/
{
  return plane_pt.x * query_pt[0] + plane_pt.y * query_pt[1] + plane_pt.z * query_pt[2];
}

} // namespace geometry
