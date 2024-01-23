# MIT License
#
# # Copyright (c) 2023 Ignacio Vizzo, University of Bonn
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

if(USE_SYSTEM_OPENVDB)
  # When OpenVDB is available on the system, we just go for the dynamic version
  # of it
  include(GNUInstallDirs)
  list(APPEND CMAKE_MODULE_PATH "${CMAKE_INSTALL_FULL_LIBDIR}/spatio_temporal_voxel_layer/cmake/OpenVDB")
  find_package(OpenVDB QUIET)
  if(OpenVDB_FOUND AND OpenVDB_USES_BLOSC)
    # We need to get these hidden dependencies (if available) to static link
    # them inside our library
    target_link_libraries(OpenVDB::openvdb INTERFACE Blosc::blosc)
  endif()
endif()

# When not using a pre installed version of OpenVDB we assume that no
# dependencies are installed and therefore build blos-c, tbb, and libboost from
# soruce
if(NOT USE_SYSTEM_OPENVDB OR NOT OpenVDB_FOUND)
  set(USE_SYSTEM_OPENVDB OFF)
  message(WARNING "Building OpenVDB from source")
  include(${CMAKE_CURRENT_LIST_DIR}/OpenVDB/OpenVDB.cmake)
endif()
