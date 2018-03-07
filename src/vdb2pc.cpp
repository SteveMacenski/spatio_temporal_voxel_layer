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

#include <spatio_temporal_voxel_layer/vdb2pc.hpp>

namespace utilities
{

/*****************************************************************************/
VDB2PCLPointCloud::VDB2PCLPointCloud()
/*****************************************************************************/
{
  openvdb::initialize();
}

/*****************************************************************************/
void VDB2PCLPointCloud::SetFile(const std::string& file_name)
/*****************************************************************************/
{
  _file_name = file_name;
}

/*****************************************************************************/
bool VDB2PCLPointCloud::GetCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud)
/*****************************************************************************/
{
  openvdb::io::File file(_file_name);
  file.open();
  openvdb::GridBase::Ptr baseGrid;
  openvdb::DoubleGrid::Ptr grid;

  bool valid_grid = false;

  for (openvdb::io::File::NameIterator nameIter = file.beginName();
                                nameIter != file.endName(); ++nameIter)
  {
    if (nameIter.gridName() == "SpatioTemporalVoxelLayer")
    {
      baseGrid = file.readGrid(nameIter.gridName());
      grid = openvdb::gridPtrCast<openvdb::DoubleGrid>(baseGrid);
      valid_grid = true;
    }
  }

  if (!valid_grid)
  {
    std::cout << "No valid grid inside of provided file." << std::endl;
    return false;
  }

  //populate pcl pointcloud
  openvdb::DoubleGrid::ValueOnCIter cit_grid = grid->cbeginValueOn();
  for (cit_grid; cit_grid; ++cit_grid)
  {
    const openvdb::Vec3d pt = grid->indexToWorld(cit_grid.getCoord());
    cloud->push_back(pcl::PointXYZ(pt[0], pt[1], pt[2]));
  }

  return true;
}

} // end namespace