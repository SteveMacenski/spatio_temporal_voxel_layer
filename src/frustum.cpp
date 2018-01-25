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

#include <spatio_temporal_voxel_layer/frustum.hpp>

namespace geometry
{

/*****************************************************************************/
Frustum::Frustum(const double& vFOV, const double& hFOV, const double& min_dist, \
        const double& max_dist) :  _vFOV(vFOV), _hFOV(hFOV), _min_d(min_dist), _max_d(max_dist)
/*****************************************************************************/
{
  this->ComputePlaneNormals();
}

/*****************************************************************************/
Frustum::~Frustum(void)
/*****************************************************************************/
{
}

/*****************************************************************************/
void Frustum::TransformPlaneNormals(void)
/*****************************************************************************/
{
  Eigen::Affine3d T = Eigen::Affine3d::Identity();
  T.translate(_position);
  T.rotate(_orientation);

  std::vector<Vector3D>::iterator it;
  for (it = _plane_normals.begin(); it != _plane_normals.end(); ++it)
  {
    it->TransformFrames(T);
  }
}

/*****************************************************************************/
void Frustum::ComputePlaneNormals(void)
/*****************************************************************************/
{
  //TODO

  // find center vector + dists

  // deflect to 4 corners

  // separate into point for each plane

  // cross them for a-positive normals

  // store  normals
}

/*****************************************************************************/
bool Frustum::IsInside(const openvdb::Vec3d& pt) const
/*****************************************************************************/
{
  std::vector<Vector3D>::iterator it = _plane_normals.begin();
  for (it; it!=_plane_normals.end(); ++it)
  {
    if (Dot(*it, pt) < 0.)
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
bool Frustum::Dot(const Vector3D& plane_pt, \
                                          const openvdb::Vec3d& query_pt) const
/*****************************************************************************/
{
  return plane_pt.x*query_pt[0]+plane_pt.y*query_pt[1]+plane_pt.z*query_pt[2];
}

} // end namespace

