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
include(ExternalProject)
include(GNUInstallDirs)

find_package(TBB REQUIRED COMPONENTS tbbmalloc)
find_package(Threads REQUIRED)
find_package(Blosc REQUIRED)
find_package(ZLIB REQUIRED)

# Simulate a local installation of OpenVDB library
ExternalProject_Add(
  external_openvdb
  PREFIX openvdb
  GIT_REPOSITORY https://github.com/AcademySoftwareFoundation/openvdb.git
  GIT_TAG v10.0.1
  GIT_SHALLOW ON
  UPDATE_COMMAND ""
  CMAKE_ARGS -DCMAKE_INSTALL_PREFIX=<INSTALL_DIR>
             -DCMAKE_BUILD_TYPE=Release
             -DOPENVDB_BUILD_PYTHON_MODULE=OFF
             -DOPENVDB_CORE_SHARED=ON
             -DOPENVDB_CORE_STATIC=OFF
             -DOPENVDB_BUILD_BINARIES=OFF
             -DOPENVDB_CXX_STRICT=OFF
             -DUSE_CCACHE=ON)

# Simulate importing OpenVDB::openvdb
ExternalProject_Get_Property(external_openvdb INSTALL_DIR)
add_library(OpenVDBHelper INTERFACE)
add_dependencies(OpenVDBHelper external_openvdb)
target_include_directories(OpenVDBHelper
                           INTERFACE ${INSTALL_DIR}/${CMAKE_INSTALL_INCLUDEDIR})
target_link_directories(OpenVDBHelper INTERFACE
                        ${INSTALL_DIR}/${CMAKE_INSTALL_LIBDIR})
target_link_libraries(OpenVDBHelper INTERFACE openvdb)

# Setup Visible dependencies
set(_OPENVDB_DEPENDENCIES)
list(APPEND _OPENVDB_DEPENDENCIES Boost::iostreams)
list(APPEND _OPENVDB_DEPENDENCIES Threads::Threads)
list(APPEND _OPENVDB_DEPENDENCIES TBB::tbb)
list(APPEND _OPENVDB_DEPENDENCIES Blosc::blosc)
target_link_libraries(OpenVDBHelper INTERFACE ${_OPENVDB_DEPENDENCIES})
add_library(OpenVDB::openvdb ALIAS OpenVDBHelper)
