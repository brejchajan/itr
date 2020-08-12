#!/bin/bash

# Copyright 2020 CPhoto@FIT, Brno University of Technology,
# Faculty of Information Technology,
# Božetěchova 2, 612 00, Brno, Czech Republic
#
# Redistribution and use in source code form, with or without modification,
# are permitted provided that the following conditions are met:
#
# 1. Redistributions must retain the above copyright notice, this list of
#    conditions and the following disclaimer.
#
# 2. Neither the name of the copyright holder nor the names of its contributors
#    may be used to endorse or promote products derived from this software
#    without specific prior written permission.
#
# 3. Redistributions must be pursued only for non-commercial research
#    collaboration and demonstration purposes.
#
# 4. Where separate files retain their original licence terms
#    (e.g. MPL 2.0, Apache licence), these licence terms are announced, prevail
#    these terms and must be complied.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
# THE IMPLIED WARRANTIES OF  FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
# IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
# ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
# OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
# HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
# STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY
# WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY
# OF SUCH DAMAGE.


# By using this script you accept that this script will download,
# compile, and install thir party software dependencies for the
# Immersive Trip Reports (ITR) software package. We are not responsible for
# the third party software downloaded by this script. For respective
# licensing conditions, please see individual license files within the
# downloaded software.

ITR_PATH=$(pwd)

if [ $# -lt 3 ]; then
    echo "Usage: $0 <build_path> <install_path> <build type = Release|Debug> [<num processes>]"
    echo "Tool to download and build itr + dependencies within defined
    build directory. All products will be installed into install_path.
    <num processes> is optional and defines how many build processes will be
    created for building. Default = 8."
    exit 0
fi

BUILD_PATH=$(realpath $1)
INSTALL_PATH=$(realpath $2)
BUILD_TYPE=$3

if [ "$BUILD_TYPE" != "Release" ] && [ "$BUILD_TYPE" != "Debug" ];then
    echo "Build type must be Release or Debug."
    exit 0
fi

numproc=8
if [ $# -eq 4 ]; then
	numproc=$4
fi

if [ ! -d "$BUILD_PATH" ]; then
    mkdir -p $BUILD_PATH
fi

cd $BUILD_PATH

deps_cmd="sudo dnf install gcc-c++ jasper-libs gdal-libs libGLEW freeglut libpng \
git-core cmake bzip2 libjpeg-turbo \
libjpeg-turbo-devel libpng libpng-devel libtiff libtiff-devel libXi \
libXi-devel libXmu libXmu-devel libXrandr libXrandr-devel boost boost-devel \
libxml2 libxml2-devel mesa-libGL*-devel freeglut-devel libdrm-devel protobuf \
protobuf-devel libcurl libcurl-devel gdal gdal-libs gdal-devel"
echo "Please enter password to install following dependencies:"
echo "$deps_cmd"
# install dependenciesgdal
$deps_cmd

# build glog
git clone https://github.com/google/glog.git
cd glog
mkdir build
cd build
cmake -DCMAKE_PREFIX_PATH="$INSTALL_PATH" -DCMAKE_INSTALL_PREFIX="$INSTALL_PATH" \
-DCMAKE_CXX_FLAGS="-I $INSTALL_PATH/include -L $INSTALL_PATH/lib64" \
-DCMAKE_BUILD_TYPE="$BUILD_TYPE" .. && \
make -j$numproc && make install

cd $BUILD_PATH

# build OpenSceneGraph
git clone https://github.com/openscenegraph/OpenSceneGraph.git
cd OpenSceneGraph
mkdir build
cd build
cmake -DOPENGL_PROFILE=GL3 -DOSG_GL3_AVAILABLE=TRUE \
-DCMAKE_PREFIX_PATH="$INSTALL_PATH" -DCMAKE_INSTALL_PREFIX="$INSTALL_PATH" -DCMAKE_CXX_FLAGS="-I $INSTALL_PATH/include -L $INSTALL_PATH/lib64" \
-DCMAKE_BUILD_TYPE="Release" ..
make -j$numproc && make install

cd $BUILD_PATH

if [ $BUILD_TYPE == "Debug" ]; then
    cd OpenSceneGraph
    mkdir build_debug
    cd build_debug
    cmake -DOPENGL_PROFILE=GL3 -DOSG_GL3_AVAILABLE=TRUE \
    -DCMAKE_PREFIX_PATH="$INSTALL_PATH" -DCMAKE_INSTALL_PREFIX="$INSTALL_PATH" -DCMAKE_CXX_FLAGS="-I $INSTALL_PATH/include -L $INSTALL_PATH/lib64" \
    -DCMAKE_BUILD_TYPE="Debug" ..
    make -j$numproc && make install

    cd $BUILD_PATH
fi

# build osgearth
git clone https://github.com/gwaldron/osgearth.git
cd osgearth
git checkout -- src/osgEarth/Capabilities
git checkout b13364c350287768edf9f88c3c4e0dc8502c1281
rm src/osgEarth/Capabilities
cp $ITR_PATH/patches/osgearth/Capabilities src/osgEarth/Capabilities
mkdir -p build
cd build && \
cmake -DCMAKE_CXX_FLAGS=-std=c++11 \
-DCMAKE_PREFIX_PATH="$INSTALL_PATH" -DCMAKE_INSTALL_PREFIX="$INSTALL_PATH" -DCMAKE_CXX_FLAGS="-I $INSTALL_PATH/include -L $INSTALL_PATH/lib64" \
-DCMAKE_BUILD_TYPE="Release" ..
make -j$numproc && make install

cd $BUILD_PATH

if [ $BUILD_TYPE == "Debug" ]; then
    cd osgearth
    mkdir -p build_debug
    cd build_debug
    cmake -DCMAKE_CXX_FLAGS=-std=c++11 \
    -DCMAKE_PREFIX_PATH="$INSTALL_PATH" -DCMAKE_INSTALL_PREFIX="$INSTALL_PATH" -DCMAKE_CXX_FLAGS="-I $INSTALL_PATH/include -L $INSTALL_PATH/lib64" \
    -DCMAKE_BUILD_TYPE="Debug" ..
    make -j$numproc && make install
    cd $BUILD_PATH
fi

# download and install eigen
#if [ ! -f "3.2.10.tar.bz2" ]; then
    #wget http://bitbucket.org/eigen/eigen/get/3.2.10.tar.bz2
#fi
#tar xvjf 3.2.10.tar.bz2 && cd eigen-eigen-b9cd8366d4e8 && \
git clone https://gitlab.com/libeigen/eigen.git
cd eigen
mkdir build
cd build
cmake -DCMAKE_PREFIX_PATH="$INSTALL_PATH" -DCMAKE_INSTALL_PREFIX="$INSTALL_PATH" -DCMAKE_CXX_FLAGS="-I $INSTALL_PATH/include -L $INSTALL_PATH/lib64" ..
make install

cd $BUILD_PATH

#build Ceres
git clone https://ceres-solver.googlesource.com/ceres-solver
cd ceres-solver
git checkout aeebc66bd8ff5db65b852dac0cbac28f618ac5d8
mkdir -p build
cd build
cmake -DEIGEN_INCLUDE_DIR_HINTS="$INSTALL_PATH/include/eigen3" \
-DEIGEN_DIR="$INSTALL_PATH/include/eigen3" \
-DCMAKE_BUILD_TYPE="$BUILD_TYPE" \
-DCMAKE_PREFIX_PATH="$INSTALL_PATH" -DCMAKE_INSTALL_PREFIX="$INSTALL_PATH" -DCMAKE_CXX_FLAGS="-I $INSTALL_PATH/include -L $INSTALL_PATH/lib64" ..
make -j$numproc && make install

cd $BUILD_PATH

#build OpenMVG
git clone --recursive https://github.com/openMVG/openMVG.git
cd openMVG
git checkout 35da8f72c68
cd ..
echo "Copying patch of CMakeLists.txt for OpenMVG..."
cp $ITR_PATH/patches/openMVG/CMakeLists.txt openMVG/src/CMakeLists.txt
mkdir -p openMVG_Build
cd openMVG_Build
cmake -DEIGEN_INCLUDE_DIR_HINTS="$INSTALL_PATH/include/eigen3" \
-DCMAKE_MODULE_PATH="$BUILD_PATH/eigen-eigen-b9cd8366d4e8/cmake" \
-DEIGEN_INCLUDE_DIR="$INSTALL_PATH/include/eigen3" \
-DCMAKE_BUILD_TYPE="$BUILD_TYPE" \
-DCMAKE_POLICY_DEFAULT_CMP0100="NEW" \
-DCMAKE_PREFIX_PATH="$INSTALL_PATH" -DCMAKE_INSTALL_PREFIX="$INSTALL_PATH" -DCMAKE_CXX_FLAGS="-I $INSTALL_PATH/include -L $INSTALL_PATH/lib64" \
../openMVG/src
make -j$numproc && make install

cd $BUILD_PATH

#copy easyexif headers
echo "Copying easyexif: $INSTALL_PATH/openMVG/third_party/easyexif"
mkdir -p $INSTALL_PATH/include/openMVG/third_party/easyexif && cp $BUILD_PATH/openMVG/src/third_party/easyexif/exif.h $INSTALL_PATH/include/openMVG/third_party/easyexif/

cd $BUILD_PATH

#build libnabo
git clone https://github.com/ethz-asl/libnabo.git
cd libnabo
mkdir build
cd build
cmake -DCMAKE_BUILD_TYPE="$BUILD_TYPE" \
-DEIGEN_INCLUDE_DIR="$INSTALL_PATH/include/eigen3" \
-DCMAKE_PREFIX_PATH="$INSTALL_PATH" -DCMAKE_INSTALL_PREFIX="$INSTALL_PATH" -DCMAKE_CXX_FLAGS="-I $INSTALL_PATH/include -L $INSTALL_PATH/lib64" ..
make -j$numproc && make install

cd $BUILD_PATH

#build libpointmatcher
git clone https://github.com/ethz-asl/libpointmatcher.git
cd libpointmatcher
mkdir build
cd build
cmake -DEIGEN_INCLUDE_DIR="$INSTALL_PATH/include/eigen3" -DCMAKE_BUILD_TYPE="$BUILD_TYPE" \
-DCMAKE_PREFIX_PATH="$INSTALL_PATH" -DCMAKE_INSTALL_PREFIX="$INSTALL_PATH" -DCMAKE_CXX_FLAGS="-I $INSTALL_PATH/include -L $INSTALL_PATH/lib64" ..
make -j$numproc && make install

cd $BUILD_PATH
git clone https://github.com/brejchajan/nvm_to_openmvg.git
cd nvm_to_openmvg
mkdir build
cd build
cmake -DCMAKE_BUILD_TYPE="$BUILD_TYPE" \
-DEIGEN_INCLUDE_DIR_HINTS="$INSTALL_PATH/include/eigen3" -DEigen3_DIR="$INSTALL_PATH/include/eigen3" \
-DCMAKE_FIND_DEBUG_MODE=ON -DCMAKE_PREFIX_PATH="$INSTALL_PATH;/usr/lib64" -DCMAKE_LIBRARY_PATH="$INSTALL_PATH/lib64;$INSTALL_PATH/lib;/usr/lib64" \
-DCMAKE_INSTALL_PREFIX="$INSTALL_PATH" -DCMAKE_CXX_FLAGS="-I$INSTALL_PATH/include -I$INSTALL_PATH/include/openMVG -L$INSTALL_PATH/lib64 -L$INSTALL_PATH/lib -Wl,-rpath-link,$INSTALL_PATH/lib64:$INSTALL_PATH/lib" \
-DCMAKE_EXE_LINKER_FLAGS="-L$INSTALL_PATH/lib64" -Dglog_DIR="/usr/lib64" ..
make -j$numproc && make install

cd $BUILD_PATH

mkdir -p itr/build
cd itr/build
cmake -DCMAKE_BUILD_TYPE="$BUILD_TYPE" \
-DEIGEN_INCLUDE_DIR_HINTS="$INSTALL_PATH/include/eigen3" -DEigen3_DIR="$INSTALL_PATH/include/eigen3" \
-DCMAKE_FIND_DEBUG_MODE=ON -DCMAKE_PREFIX_PATH="$INSTALL_PATH;/usr/lib64" -DCMAKE_LIBRARY_PATH="$INSTALL_PATH/lib64;$INSTALL_PATH/lib;/usr/lib64" \
-DCMAKE_INSTALL_PREFIX="$INSTALL_PATH" -DCMAKE_CXX_FLAGS="-I$INSTALL_PATH/include -I$INSTALL_PATH/include/openMVG -L$INSTALL_PATH/lib64 -L$INSTALL_PATH/lib -Wl,-rpath-link,$INSTALL_PATH/lib64:$INSTALL_PATH/lib" \
-DCMAKE_EXE_LINKER_FLAGS="-L$INSTALL_PATH/lib64" -Dglog_DIR="/usr/lib64" $ITR_PATH
make -j$numproc && make install

comment="# added by itr installer"
echo "Do you want to add install dir bin and lib, lib64 to PATH in ~/.bashrc? Backup of ~/.bashrc will be made. [y/n]"
read -n1 response
echo ""
if [ "$response" == "y" ]; then
	find_comment=$(cat ~/.bashrc | grep "$comment")
	if [ "$find_comment" != "$comment" ]; then
		echo "Backing up ~/.bashrc to $BUILD_PATH/.bashrc_backup"
		cp ~/.bashrc $BUILD_PATH/.bashrc_backup

		echo "" >> ~/.bashrc
		echo $comment >> ~/.bashrc
		echo "export PATH=$INSTALL_PATH/bin:\$PATH" >> ~/.bashrc
		echo "export LD_LIBRARY_PATH=$INSTALL_PATH/lib:$INSTALL_PATH/lib64:\$LD_LIBRARY_PATH" >> ~/.bashrc

		echo "Added bin and library paths to ~/.bashrc."
	else
		echo "Skipping adding install path to ~/.bashrc, because it is already present."
	fi
fi
