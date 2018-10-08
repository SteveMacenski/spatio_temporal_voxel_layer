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
 * Purpose: Test minimum configuration of costmap_2d
/*********************************************************************/


#include <ros/ros.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <tf/transform_broadcaster.h>
#include <thread>

#include "tf2_ros/transform_listener.h"
#include "tf2_ros/message_filter.h"
#include "message_filters/subscriber.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"


/*****************************************************************************/
void TransformThread()
/*****************************************************************************/
{
  tf::TransformBroadcaster tfB;
  tf::Transform transform;
  transform.setIdentity();
  transform.setOrigin(tf::Vector3(2.,2.,0.));

  ros::Rate r(20);
  while (ros::ok())
  {
    tfB.sendTransform(tf::StampedTransform(transform, ros::Time::now(), \
                                                        "map", "base_link"));
    tfB.sendTransform(tf::StampedTransform(transform, ros::Time::now(), \
                                                "base_link", "camera_link"));
    r.sleep();
    ros::spinOnce();
  }
}

/*****************************************************************************/
int main(int argc, char **argv)
/*****************************************************************************/
{
  ros::init(argc, argv, "STVL_minimal_test");

  tf2_ros::Buffer tf_buffer;
  tf2_ros::TransformListener _tf2(tf_buffer);
  std::thread t(TransformThread);

  costmap_2d::Costmap2DROS costmap("global_costmap", tf_buffer);
  costmap.start();

  ros::Rate r(10);
  while (ros::ok())
  {
    // then make a thread to do something with a costmap pointer...
    ros::spinOnce();
    r.sleep();
  }
  return 1;
}
