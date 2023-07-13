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

set(BOOST_URL "https://boostorg.jfrog.io/artifactory/main/release/1.71.0/source/boost_1_71_0.tar.bz2")
set(BOOST_URL_SHA256 "d73a8da01e8bf8c7eda40b4c84915071a8c8a0df4a6734537ddde4a8580524ee")
set(BOOST_CONFIGURE <SOURCE_DIR>/bootstrap.sh --with-libraries=iostreams,regex)
set(BOOST_INSTALL
    <SOURCE_DIR>/b2
    install
    link=static
    warnings=off
    cxxflags=-fPIC
    cflags=-fPIC
    --prefix=<INSTALL_DIR>)

include(ExternalProject)
ExternalProject_Add(
  external_boost
  PREFIX boost
  URL ${BOOST_URL}
  URL_HASH SHA256=${BOOST_URL_SHA256}
  BUILD_IN_SOURCE true
  CONFIGURE_COMMAND "${BOOST_CONFIGURE}"
  BUILD_COMMAND ""
  INSTALL_COMMAND "${BOOST_INSTALL}")

# Simulate importing Boost::iostreams for OpenVDBHelper target
ExternalProject_Get_Property(external_boost INSTALL_DIR)
set(BOOST_ROOT ${INSTALL_DIR} CACHE INTERNAL "Boost libraries Install directory")
add_library(BoostIostreamsHelper INTERFACE)
add_dependencies(BoostIostreamsHelper external_boost_iostreams)
target_include_directories(BoostIostreamsHelper INTERFACE ${INSTALL_DIR}/include)
target_link_directories(BoostIostreamsHelper INTERFACE ${INSTALL_DIR}/lib)
target_link_libraries(BoostIostreamsHelper INTERFACE boost_iostreams.a)
add_library(Boost::iostreams ALIAS BoostIostreamsHelper)
