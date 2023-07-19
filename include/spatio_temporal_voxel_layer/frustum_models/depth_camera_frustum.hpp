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
 * Purpose: Structure for handling camera FOVs to construct frustums
 *          and associated methods
 *********************************************************************/

#ifndef DEPTH_FRUSTUM_H_
#define DEPTH_FRUSTUM_H_

// STVL
#include <spatio_temporal_voxel_layer/frustum_models/frustum.hpp>

namespace geometry
{

// visualize the frustum should someone other than me care
#define VISUALIZE_FRUSTUM 0

// A class to model a depth sensor frustum in world space
class DepthCameraFrustum : public Frustum
{
public:
  DepthCameraFrustum(const double& vFOV, const double& hFOV,
          const double& min_dist, const double& max_dist);
  virtual ~DepthCameraFrustum(void);

  // transform plane normals by depth camera pose
  virtual void TransformModel(void);

  // determine if a point is inside of the transformed frustum
  virtual bool IsInside(const openvdb::Vec3d& pt);

  // set pose of depth camera in global space
  virtual void SetPosition(const geometry_msgs::Point& origin);
  virtual void SetOrientation(const geometry_msgs::Quaternion& quat);

private:
  // utils to find useful frustum metadata
  void ComputePlaneNormals(void);
  double Dot(const VectorWithPt3D&, const openvdb::Vec3d&) const;
  double Dot(const VectorWithPt3D&, const Eigen::Vector3d&) const;

  double _vFOV, _hFOV, _min_d, _max_d;
  std::vector<VectorWithPt3D> _plane_normals;
  Eigen::Vector3d _position;
  Eigen::Quaterniond _orientation;
  bool _valid_frustum;

  #if VISUALIZE_FRUSTUM
    std::vector<Eigen::Vector3d> _frustum_pts;
    ros::Publisher _frustumPub;
  #endif
};

} // end namespace

#endif
