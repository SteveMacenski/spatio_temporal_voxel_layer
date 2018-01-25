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

// STL
#include <vector>
// OpenVDB
#include <openvdb/openvdb.h>

namespace geometry
{

struct Vector3D
{
  Vector3D(const double& x_, const double& y_, const double& z_) : x(x_), y(y_), z(z_)
  {
  }

  Vector3D() : x(), y(), z()
  {
  }

  void TransformFrames()
  {

  }
  double x,y,z;
}

class Frustum
{
public:
  Frustum(void);
  Frustum(const double& vFOV, const double& hFOV, const double& min_dist, const double& max_dist);
  ~Frustum(void);

  void ComputePlaneNormals();
  void SetPosition();
  void SetOrientation();

  bool IsInside(const openvdb::Vec3d& pt);

private:
  void TransformPlaneNormals();

  double _vFOV, _hFOV, _min_d, _max_d;
  std::vector<Vector3D> _plane_normals;
};

} // end namespace

#endif


























// const openvdb::Vec3d near_plane = this->WorldToIndex(openvdb::Vec3d(0, 0, 0));
// const openvdb::Vec3d far_plane = this->WorldToIndex(openvdb::Vec3d(0.4,0.4,5));

// const openvdb::math::BBox<openvdb::Vec3d> bbox(near_plane, far_plane); //planes of camera, from its geometry TODO

// const double taper = 1;// The far plane of the frustum will be twice as big as the near plane TODO
// const double depth = 1;// The depth of the frustum will be 10 times the x-width of the near plane TODO
// openvdb::math::NonlinearFrustumMap* frustumTransform = new openvdb::math::NonlinearFrustumMap(bbox, taper, depth);
// frustumTransform->preScale(openvdb::Vec3d(_voxel_size, _voxel_size, _voxel_size));
// frustumTransform->postTranslate(openvdb::math::Vec3d(origin.x, origin.y, origin.z));
// //frustumTransform->postRotate(0., openvdb::math::Z_AXIS); //tf get rpy from baselink->camera depth frame transform TODO

// const openvdb::math::BBox<openvdb::Vec3d> frustumIndexBounds = frustumTransform->getBBox();

// openvdb::Vec3d min = frustumTransform->applyMap(frustumIndexBounds.min());
// openvdb::Vec3d max = frustumTransform->applyMap(frustumIndexBounds.max());
// visualization_msgs::Marker msg;
// msg.header.frame_id = "map";msg.ns = "frustum";msg.id = 0;
// msg.type= 5;msg.action = 0;msg.pose.orientation.w = 1;
// msg.scale.x = 0.1;msg.scale.z = 0.1;msg.scale.y = 0.1;
// msg.color.r = 1.0f;msg.color.a = 1;
// geometry_msgs::Point p1;
// p1.x = min.x();
// p1.y = min.y();
// p1.z = min.z();
// geometry_msgs::Point p2;
// p2.x = max.x();
// p2.y = min.y();
// p2.z = min.z();
// geometry_msgs::Point p3;
// p3.x = max.x();
// p3.y = max.y();
// p3.z = min.z();
// geometry_msgs::Point p4;
// p4.x = min.x();
// p4.y = max.y();
// p4.z = min.z();
// geometry_msgs::Point p5;
// p5.x = min.x();
// p5.y = min.y();
// p5.z = max.z();
// geometry_msgs::Point p6;
// p6.x = max.x();
// p6.y = min.y();
// p6.z = max.z();
// geometry_msgs::Point p7;
// p7.x = min.x();
// p7.y = max.y();
// p7.z = max.z();
// geometry_msgs::Point p8;
// p8.x = max.x();
// p8.y = max.y();
// p8.z = max.z();
// msg.points.push_back(p1);msg.points.push_back(p2);
// msg.points.push_back(p2);msg.points.push_back(p3);
// msg.points.push_back(p3);msg.points.push_back(p4);
// msg.points.push_back(p4);msg.points.push_back(p1);

// msg.points.push_back(p5);msg.points.push_back(p6);
// msg.points.push_back(p6);msg.points.push_back(p8);
// msg.points.push_back(p8);msg.points.push_back(p7);
// msg.points.push_back(p5);msg.points.push_back(p7);
// ros::NodeHandle nh;
// static ros::Publisher visPub = nh.advertise<visualization_msgs::Marker>("/stupid_pts", 5);
// visPub.publish(msg);
