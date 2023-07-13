# MIT License
#
# # Copyright (c) 2022 Ignacio Vizzo, Cyrill Stachniss, University of Bonn
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
ExternalProject_Add(
  external_blosc
  PREFIX blosc
  URL https://github.com/Blosc/c-blosc/archive/refs/tags/v1.5.0.tar.gz
  URL_HASH SHA256=208ba4db0e5116421ed2fbbdf2adfa3e1d133d29a6324a0f47cf2d71f3810c92
  UPDATE_COMMAND ""
  CMAKE_ARGS -DCMAKE_INSTALL_PREFIX=<INSTALL_DIR>
             ${ExternalProject_CMAKE_ARGS}
             ${ExternalProject_CMAKE_CXX_FLAGS}
             # Custom OpenVDB build settings
             -DBUILD_STATIC=ON
             -DBUILD_TESTS=OFF
             -DBUILD_BENCHMARKS=OFF
             -DPREFER_EXTERNAL_COMPLIBS=OFF)

# Simulate importing Blosc::blosc for OpenVDBHelper target
ExternalProject_Get_Property(external_blosc INSTALL_DIR)
set(BLOSC_ROOT ${INSTALL_DIR} CACHE INTERNAL "BLOSC_ROOT Install directory")
add_library(BloscHelper INTERFACE)
add_dependencies(BloscHelper external_blosc)
target_include_directories(BloscHelper INTERFACE ${INSTALL_DIR}/include)
target_link_directories(BloscHelper INTERFACE ${INSTALL_DIR}/lib)
target_link_libraries(BloscHelper INTERFACE blosc.a)
add_library(Blosc::blosc ALIAS BloscHelper)
