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

#include <vector>
#include "spatio_temporal_voxel_layer/frustum_models/depth_camera_frustum.hpp"

namespace geometry
{

/*****************************************************************************/
DepthCameraFrustum::DepthCameraFrustum(
  const double & vFOV, const double & hFOV, const double & min_dist,
  const double & max_dist)
: _vFOV(vFOV), _hFOV(hFOV), _min_d(min_dist), _max_d(max_dist)
/*****************************************************************************/
{
  _valid_frustum = false;
  this->ComputePlaneNormals();
}

/*****************************************************************************/
DepthCameraFrustum::~DepthCameraFrustum(void)
/*****************************************************************************/
{
}

/*****************************************************************************/
void DepthCameraFrustum::ComputePlaneNormals(void)
/*****************************************************************************/
{
  // give ability to construct with bogus values
  if (_vFOV == 0 && _hFOV == 0) {
    _valid_frustum = false;
    return;
  }

  // Z vector and deflected vector capture
  std::vector<Eigen::Vector3d> deflected_vecs;
  deflected_vecs.reserve(4);
  Eigen::Vector3d Z = Eigen::Vector3d::UnitZ();

  // rotate going CCW
  Eigen::Affine3d rx =
    Eigen::Affine3d(Eigen::AngleAxisd(_vFOV / 2., Eigen::Vector3d::UnitX()));
  Eigen::Affine3d ry =
    Eigen::Affine3d(Eigen::AngleAxisd(_hFOV / 2., Eigen::Vector3d::UnitY()));
  deflected_vecs.push_back(rx * ry * Z);

  rx = Eigen::Affine3d(Eigen::AngleAxisd(-_vFOV / 2., Eigen::Vector3d::UnitX()));
  deflected_vecs.push_back(rx * ry * Z);

  ry = Eigen::Affine3d(Eigen::AngleAxisd(-_hFOV / 2., Eigen::Vector3d::UnitY()));
  deflected_vecs.push_back(rx * ry * Z);

  rx = Eigen::Affine3d(Eigen::AngleAxisd(_vFOV / 2., Eigen::Vector3d::UnitX()));
  deflected_vecs.push_back(rx * ry * Z);

  // get and store CCW 4 corners for each 2 planes at ends
  std::vector<Eigen::Vector3d> pt_;
  pt_.reserve(2 * deflected_vecs.size());
  std::vector<Eigen::Vector3d>::iterator it;
  for (it = deflected_vecs.begin(); it != deflected_vecs.end(); ++it) {
    pt_.push_back(*(it) * _min_d);
    pt_.push_back(*(it) * _max_d);
  }

  assert(pt_.size() == 8);

  // cross each plane and get normals
  const Eigen::Vector3d v_01(pt_[1][0] - pt_[0][0],
    pt_[1][1] - pt_[0][1], pt_[1][2] - pt_[0][2]);
  const Eigen::Vector3d v_13(pt_[3][0] - pt_[1][0],
    pt_[3][1] - pt_[1][1], pt_[3][2] - pt_[1][2]);
  Eigen::Vector3d T_n(v_13.cross(v_01));
  T_n.normalize();
  _plane_normals.push_back(VectorWithPt3D(T_n[0], T_n[1], T_n[2], pt_[0]));

  const Eigen::Vector3d v_23(pt_[3][0] - pt_[2][0],
    pt_[3][1] - pt_[2][1], pt_[3][2] - pt_[2][2]);
  const Eigen::Vector3d v_35(pt_[5][0] - pt_[3][0],
    pt_[5][1] - pt_[3][1], pt_[5][2] - pt_[3][2]);
  Eigen::Vector3d T_l(v_35.cross(v_23));
  T_l.normalize();
  _plane_normals.push_back(VectorWithPt3D(T_l[0], T_l[1], T_l[2], pt_[2]));

  const Eigen::Vector3d v_45(pt_[5][0] - pt_[4][0],
    pt_[5][1] - pt_[4][1], pt_[5][2] - pt_[4][2]);
  const Eigen::Vector3d v_57(pt_[7][0] - pt_[5][0],
    pt_[7][1] - pt_[5][1], pt_[7][2] - pt_[5][2]);
  Eigen::Vector3d T_b(v_57.cross(v_45));
  T_b.normalize();
  _plane_normals.push_back(VectorWithPt3D(T_b[0], T_b[1], T_b[2], pt_[4]));

  const Eigen::Vector3d v_67(pt_[7][0] - pt_[6][0],
    pt_[7][1] - pt_[6][1], pt_[7][2] - pt_[6][2]);
  const Eigen::Vector3d v_71(pt_[1][0] - pt_[7][0],
    pt_[1][1] - pt_[7][1], pt_[1][2] - pt_[7][2]);
  Eigen::Vector3d T_r(v_71.cross(v_67));
  T_r.normalize();
  _plane_normals.push_back(VectorWithPt3D(T_r[0], T_r[1], T_r[2], pt_[6]));

  // far plane
  Eigen::Vector3d T_1(v_57.cross(v_71));
  T_1.normalize();
  _plane_normals.push_back(VectorWithPt3D(T_1[0], T_1[1], T_1[2], pt_[7]));

  // near plane
  _plane_normals.push_back(VectorWithPt3D(
      T_1[0], T_1[1], T_1[2], pt_[2]) * -1);

  assert(_plane_normals.size() == 6);
  _valid_frustum = true;
}

/*****************************************************************************/
void DepthCameraFrustum::TransformModel(void)
/*****************************************************************************/
{
  if (!_valid_frustum) {
    return;
  }

  Eigen::Affine3d T = Eigen::Affine3d::Identity();
  T.pretranslate(_orientation.inverse() * _position);
  T.prerotate(_orientation);

  std::vector<VectorWithPt3D>::iterator it;
  for (it = _plane_normals.begin(); it != _plane_normals.end(); ++it) {
    it->TransformFrames(T);
  }
}

/*****************************************************************************/
bool DepthCameraFrustum::IsInside(const openvdb::Vec3d & pt)
/*****************************************************************************/
{
  if (!_valid_frustum) {
    return false;
  }

  std::vector<VectorWithPt3D>::iterator it;
  for (it = _plane_normals.begin(); it != _plane_normals.end(); ++it) {
    Eigen::Vector3d p_delta(pt[0] - it->initial_point[0],
      pt[1] - it->initial_point[1], pt[2] - it->initial_point[2]);
    p_delta.normalize();

    if (Dot(*it, p_delta) > 0.) {
      return false;
    }
  }
  return true;
}

/*****************************************************************************/
void DepthCameraFrustum::SetPosition(const geometry_msgs::msg::Point & origin)
/*****************************************************************************/
{
  _position = Eigen::Vector3d(origin.x, origin.y, origin.z);
}

/*****************************************************************************/
void DepthCameraFrustum::SetOrientation(
  const geometry_msgs::msg::Quaternion & quat)
/*****************************************************************************/
{
  _orientation = Eigen::Quaterniond(quat.w, quat.x, quat.y, quat.z);
}

/*****************************************************************************/
double DepthCameraFrustum::Dot(
  const VectorWithPt3D & plane_pt, const openvdb::Vec3d & query_pt) const
/*****************************************************************************/
{
  return plane_pt.x * query_pt[0] +
         plane_pt.y * query_pt[1] +
         plane_pt.z * query_pt[2];
}

/*****************************************************************************/
double DepthCameraFrustum::Dot(
  const VectorWithPt3D & plane_pt, const Eigen::Vector3d & query_pt) const
/*****************************************************************************/
{
  return plane_pt.x * query_pt[0] +
         plane_pt.y * query_pt[1] +
         plane_pt.z * query_pt[2];
}

}  // namespace geometry
