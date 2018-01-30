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
 *********************************************************************/

#include <spatio_temporal_voxel_layer/frustum.hpp>

namespace geometry
{

/*****************************************************************************/
Frustum::Frustum(const double& vFOV, const double& hFOV,     \
                 const double& min_dist, const double& max_dist) :  
                                   _vFOV(vFOV), _hFOV(hFOV), \
                                   _min_d(min_dist), _max_d(max_dist)
/*****************************************************************************/
{
  _valid_frustum = false;
  this->ComputePlaneNormals();
}

/*****************************************************************************/
Frustum::~Frustum(void)
/*****************************************************************************/
{
}

/*****************************************************************************/
void Frustum::ComputePlaneNormals(void)
/*****************************************************************************/
{
  // Z vector and deflected vector capture
  std::vector<Eigen::Vector3d> deflected_vecs;
  Eigen::Vector3d Z = Eigen::Vector3d::UnitZ();

  // rotate going CCW
  Eigen::Affine3d rx =
    Eigen::Affine3d(Eigen::AngleAxisd(_hFOV/2.,Eigen::Vector3d::UnitX()));
  Eigen::Affine3d ry =
    Eigen::Affine3d(Eigen::AngleAxisd(_vFOV/2.,Eigen::Vector3d::UnitY()));
  deflected_vecs.push_back(rx * ry * Z);

  rx = Eigen::Affine3d(Eigen::AngleAxisd(-_hFOV/2.,Eigen::Vector3d::UnitX()));
  deflected_vecs.push_back(rx * ry * Z);

  ry = Eigen::Affine3d(Eigen::AngleAxisd(-_vFOV/2.,Eigen::Vector3d::UnitY()));
  deflected_vecs.push_back(rx * ry * Z);

  rx = Eigen::Affine3d(Eigen::AngleAxisd( _hFOV/2.,Eigen::Vector3d::UnitX()));
  deflected_vecs.push_back(rx * ry * Z);

  // get and store CCW 4 corners for each 2 planes at ends
  std::vector<Eigen::Vector3d> pt_;
  std::vector<Eigen::Vector3d>::iterator it;
  for (it = deflected_vecs.begin(); it != deflected_vecs.end(); ++it)
  {
    pt_.push_back(*(it) * _min_d);
    pt_.push_back(*(it) * _max_d);
  }

  // cross each plane and get normals
  // Top plane
  const Eigen::Vector3d v_01(pt_[1][0]-pt_[0][0], \
                    pt_[1][1]-pt_[0][1], pt_[1][2]-pt_[0][2]);
  const Eigen::Vector3d v_13(pt_[3][0]-pt_[1][0], \
                    pt_[3][1]-pt_[1][1], pt_[3][2]-pt_[1][2]);
  const Eigen::Vector3d T_n(v_01.cross(v_13));
  _plane_normals.push_back(VectorWithPt3D(T_n[0],T_n[1],T_n[2],pt_[0]));

  // left plane
  const Eigen::Vector3d v_23(pt_[3][0]-pt_[2][0], \
                    pt_[3][1]-pt_[2][1], pt_[3][2]-pt_[2][2]);
  const Eigen::Vector3d v_35(pt_[5][0]-pt_[3][0], \
                    pt_[5][1]-pt_[3][1], pt_[5][2]-pt_[3][2]);
  const Eigen::Vector3d T_l(v_23.cross(v_35));
  _plane_normals.push_back(VectorWithPt3D(T_l[0],T_l[1],T_l[2],pt_[2]));

  // bottom plane
  const Eigen::Vector3d v_45(pt_[5][0]-pt_[4][0], \
                    pt_[5][1]-pt_[4][1], pt_[5][2]-pt_[4][2]);
  const Eigen::Vector3d v_57(pt_[7][0]-pt_[5][0], \
                    pt_[7][1]-pt_[5][1], pt_[7][2]-pt_[5][2]);
  const Eigen::Vector3d T_b(v_45.cross(v_57));
  _plane_normals.push_back(VectorWithPt3D(T_b[0],T_b[1],T_b[2],pt_[4]));

  // right plane
  const Eigen::Vector3d v_67(pt_[7][0]-pt_[6][0], \
                    pt_[7][1]-pt_[6][1], pt_[7][2]-pt_[6][2]);
  const Eigen::Vector3d v_71(pt_[1][0]-pt_[7][0], \
                    pt_[1][1]-pt_[7][1], pt_[1][2]-pt_[7][2]);
  const Eigen::Vector3d T_r(v_67.cross(v_71));
  _plane_normals.push_back(VectorWithPt3D(T_r[0],T_r[1],T_r[2],pt_[6]));

  // near plane
  const Eigen::Vector3d v_02(pt_[2][0]-pt_[0][0], \
                    pt_[2][1]-pt_[0][1], pt_[2][2]-pt_[0][2]);
  const Eigen::Vector3d v_24(pt_[4][0]-pt_[2][0], \
                    pt_[4][1]-pt_[2][1], pt_[4][2]-pt_[2][2]);
  const Eigen::Vector3d T_0(v_02.cross(v_24));
  _plane_normals.push_back(VectorWithPt3D(T_0[0],T_0[1],T_0[2],pt_[0]));

  // far plane
  const Eigen::Vector3d v_17(pt_[7][0]-pt_[1][0], \
                    pt_[7][1]-pt_[1][1], pt_[7][2]-pt_[1][2]);
  const Eigen::Vector3d v_75(pt_[5][0]-pt_[7][0], \
                    pt_[5][1]-pt_[7][1], pt_[5][2]-pt_[7][2]);
  const Eigen::Vector3d T_1(v_17.cross(v_75));
  _plane_normals.push_back(VectorWithPt3D(T_1[0],T_1[1],T_1[2],pt_[1]));

  // flip direction if wrong, they shouldn't be if positive values given
  Eigen::Vector3d test_pt(0., 0., (_max_d + _min_d)/2.);
  for (uint i = 0; i!= _plane_normals.size(); i++)
  {
    const VectorWithPt3D q = _plane_normals.at(i);
    if (q.x*test_pt[0]+q.y*test_pt[1]+q.z*test_pt[2] < 0.)
    {
      _plane_normals.at(i) = _plane_normals.at(i) * -1.;
    }
  }

  _valid_frustum = true;
  return;
}

/*****************************************************************************/
void Frustum::TransformPlaneNormals(void)
/*****************************************************************************/
{
  Eigen::Affine3d T = Eigen::Affine3d::Identity();
  T.translate(_position);
  T.rotate(_orientation);

  std::vector<VectorWithPt3D>::iterator it;
  for (it = _plane_normals.begin(); it != _plane_normals.end(); ++it)
  {
    it->TransformFrames(T);
  }
}

/*****************************************************************************/
bool Frustum::IsInside(const openvdb::Vec3d& pt)
/*****************************************************************************/
{
  // TODO 2 options, right side of 6 planes or 
  // find the azimuth and altitude of it, check in FOV, find dist, check range
  if (!_valid_frustum)
  {
    return false;
  }

  std::vector<VectorWithPt3D>::iterator it;
  for (it = _plane_normals.begin(); it != _plane_normals.end(); ++it)
  {
    const Eigen::Vector3d p_delta(pt[0] - it->initial_point[0], \
                                  pt[1] - it->initial_point[1], \
                                  pt[2] - it->initial_point[2]);
    if (Dot(*it, p_delta)  < 0.)
    {
      return false;
    }
  }
  return true;
}

/*****************************************************************************/
void Frustum::SetPosition(const geometry_msgs::Point& origin)
/*****************************************************************************/
{
  _position = Eigen::Vector3d(origin.x, origin.y, origin.z);
}

/*****************************************************************************/
void Frustum::SetOrientation(const geometry_msgs::Quaternion& quat)
/*****************************************************************************/
{
  _orientation = Eigen::Quaterniond(quat.w, quat.x, quat.y, quat.z);
}

/*****************************************************************************/
double Frustum::Dot(const VectorWithPt3D& plane_pt, \
                                          const openvdb::Vec3d& query_pt) const
/*****************************************************************************/
{
  return plane_pt.x*query_pt[0]+plane_pt.y*query_pt[1]+plane_pt.z*query_pt[2];
}

/*****************************************************************************/
double Frustum::Dot(const VectorWithPt3D& plane_pt, \
                                         const Eigen::Vector3d& query_pt) const
/*****************************************************************************/
{
  return plane_pt.x*query_pt[0]+plane_pt.y*query_pt[1]+plane_pt.z*query_pt[2];
}

} // end namespace

