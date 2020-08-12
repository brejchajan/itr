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



ITR_PATH=$(pwd)

#export CC=/usr/local/bin/gcc-8
#export CXX=/usr/local/bin/g++-8

if [ $# -lt 3 ]; then
    echo "Usage: $0 <build_path> <install_path> <build type = Release|Debug> [<num processes>]"
    echo "Tool to download and build itr + dependencies within defined
    build directory. All products will be installed into install_path.
    <num processes> is optional and defines how many build processes will be
    created for building. Default = 8."
    exit 0
fi

if [ $(which brew) == "" ]; then
    echo "Please, install Homebrew package manager first."
    exit 0
fi

#coreutils contains realpath
deps_cmd="brew install gdal proj libxml2 coreutils protobuf rocksdb cereal glog"
echo "Installing dependencies using homebrew.:"
echo "$deps_cmd"
# install dependencies
#$deps_cmd

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


# #build glog
# git clone https://github.com/google/glog.git
# cd glog && mkdir build_dir
# cd build_dir && \
# cmake -DCMAKE_PREFIX_PATH="$INSTALL_PATH" -DCMAKE_INSTALL_PREFIX="$INSTALL_PATH" \
# -DCMAKE_CXX_FLAGS="-I $INSTALL_PATH/include -L $INSTALL_PATH/lib" \
# -DCMAKE_BUILD_TYPE="$BUILD_TYPE" .. && \
# make -j$numproc && make install
#
# cd $BUILD_PATH

# build OpenSceneGraph
git clone https://github.com/openscenegraph/OpenSceneGraph.git
cd $BUILD_PATH/OpenSceneGraph
git checkout 76d7ec8e0fbbcadba53424c5399cb9197aee60a4
cd $BUILD_PATH
echo "Copying patch for GraphicsWindowCocoa: cp $ITR_PATH/patches/OpenSceneGraph/osgViewer/GraphicsWindowCocoa.mm OpenSceneGraph/src/osgViewer/GraphicsWindowCocoa.mm"
cp $ITR_PATH/patches/OpenSceneGraph/osgViewer/GraphicsWindowCocoa.mm OpenSceneGraph/src/osgViewer/GraphicsWindowCocoa.mm
cd OpenSceneGraph && mkdir build
cd build && \
cmake -DOPENGL_PROFILE=GL3 -DOSG_GL3_AVAILABLE=TRUE \
 -DOSG_GL1_AVAILABLE=FALSE -DOSG_GL2_AVAILABLE=FALSE \
 -DOSG_GLES1_AVAILABLE=FALSE -DOSG_GLES2_AVAILABLE=FALSE -DOSG_GLES3_AVAILABLE=FALSE \
 -DOSG_GL_DISPLAYLISTS_AVAILABLE=FALSE -DOSG_GL_FIXED_FUNCTION_AVAILABLE=FALSE \
 -DOSG_GL_MATRICES_AVAILABLE=FALSE -DOSG_GL_VERTEX_ARRAY_FUNCS_AVAILABLE=FALSE \
 -DOSG_GL_VERTEX_FUNCS_AVAILABLE=FALSE \
-DCMAKE_PREFIX_PATH="$INSTALL_PATH" -DCMAKE_INSTALL_PREFIX="$INSTALL_PATH" -DCMAKE_CXX_FLAGS="-I $INSTALL_PATH/include -L $INSTALL_PATH/lib64" \
-DCMAKE_BUILD_TYPE="Release" \
-DCMAKE_CXX_FLAGS="-std=c++11" \
.. && \
make -j$numproc && make install

cd $BUILD_PATH

if [ $BUILD_TYPE == "Debug" ]; then
    cd OpenSceneGraph && mkdir build_debug
    cd build_debug && \
    cmake -DOPENGL_PROFILE=GL3 -DOSG_GL3_AVAILABLE=TRUE \
     -DOSG_GL1_AVAILABLE=FALSE -DOSG_GL2_AVAILABLE=FALSE \
     -DOSG_GLES1_AVAILABLE=FALSE -DOSG_GLES2_AVAILABLE=FALSE -DOSG_GLES3_AVAILABLE=FALSE \
     -DOSG_GL_DISPLAYLISTS_AVAILABLE=FALSE -DOSG_GL_FIXED_FUNCTION_AVAILABLE=FALSE \
     -DOSG_GL_MATRICES_AVAILABLE=FALSE -DOSG_GL_VERTEX_ARRAY_FUNCS_AVAILABLE=FALSE \
     -DOSG_GL_VERTEX_FUNCS_AVAILABLE=FALSE \
    -DCMAKE_PREFIX_PATH="$INSTALL_PATH" -DCMAKE_INSTALL_PREFIX="$INSTALL_PATH" -DCMAKE_CXX_FLAGS="-I $INSTALL_PATH/include -L $INSTALL_PATH/lib64" \
    -DCMAKE_BUILD_TYPE="Debug" \
    -DCMAKE_CXX_FLAGS="-std=c++11" \
    .. && \
    make -j$numproc && make install

    cd $BUILD_PATH
fi

# install GEOS
cd $BUILD_PATH
curl http://download.osgeo.org/geos/geos-3.7.3.tar.bz2 --output $BUILD_PATH/geos-3.7.3.tar.bz2
tar -jxvf $BUILD_PATH/geos-3.7.3.tar.bz2
cd $BUILD_PATH/geos-3.7.3
mkdir build
cd build
cmake -DCMAKE_PREFIX_PATH="$INSTALL_PATH;/usr/local" -DCMAKE_INSTALL_PREFIX="$INSTALL_PATH" -DCMAKE_CXX_FLAGS="-I $INSTALL_PATH/include" \
-DCMAKE_BUILD_TYPE="Release" ..
make -j$numproc
make install

cd $BUILD_PATH

# build osgearth
git clone https://github.com/gwaldron/osgearth.git
cd $BUILD_PATH/osgearth
git checkout b13364c350287768edf9f88c3c4e0dc8502c1281
cd $BUILD_PATH
rm osgearth/src/osgEarth/Capabilities
cp $ITR_PATH/patches/osgearth/Capabilities osgearth/src/osgEarth/Capabilities
cd osgearth && mkdir -p build
cd build && \
cmake -DCMAKE_CXX_FLAGS=-std=c++11 -DCMAKE_CXX_STANDARD=11 \
-DCMAKE_PREFIX_PATH="$INSTALL_PATH;/usr/local" -DCMAKE_INSTALL_PREFIX="$INSTALL_PATH" -DCMAKE_CXX_FLAGS="-I $INSTALL_PATH/include" \
-DCMAKE_BUILD_TYPE="Release" -DGEOS_DIR="$INSTALL_PATH" -DGEOS_INCLUDE_DIR="$INSTALL_PATH/include" \
.. && \
make -j$numproc && make install

cd $BUILD_PATH

if [ $BUILD_TYPE == "Debug" ]; then
    cd osgearth && mkdir -p build_debug
    cd build_debug && \
    cmake -DCMAKE_CXX_FLAGS=-std=c++11 -DCMAKE_CXX_STANDARD=11 \
    -DCMAKE_PREFIX_PATH="$INSTALL_PATH" -DCMAKE_INSTALL_PREFIX="$INSTALL_PATH" -DCMAKE_CXX_FLAGS="-I $INSTALL_PATH/include -L $INSTALL_PATH/lib" \
    -DCMAKE_BUILD_TYPE="Debug" \
    .. && \
    make -j$numproc && make install
    cd $BUILD_PATH
fi

# download and install eigen
if [ ! -f "3.3.7.tar.bz2" ]; then
    curl -LO http://bitbucket.org/eigen/eigen/get/3.3.7.tar.bz2
    #http://bitbucket.org/eigen/eigen/get/3.2.10.tar.bz2
fi
tar xvjf 3.3.7.tar.bz2 && cd eigen-eigen-323c052e1731 && \
EIGEN_DIR=$BUILD_PATH/eigen-eigen-323c052e1731
mkdir -p build && cd build && \
cmake -DCMAKE_PREFIX_PATH="$INSTALL_PATH;/usr/local/" -DCMAKE_INSTALL_PREFIX="$INSTALL_PATH" -DCMAKE_CXX_FLAGS="-I $INSTALL_PATH/include -L $INSTALL_PATH/lib" .. \
&& make install

cd $BUILD_PATH

#build Ceres
git clone https://ceres-solver.googlesource.com/ceres-solver
cd ceres-solver && \
git checkout aeebc66bd8ff5db65b852dac0cbac28f618ac5d8
mkdir -p build_dir
cd build_dir && \
cmake -DEIGEN_INCLUDE_DIR_HINTS="$EIGEN_DIR" \
-DEIGEN_DIR="$EIGEN_DIR" \
-DEIGEN_INCLUDE_DIR="$EIGEN_DIR" \
-DCMAKE_BUILD_TYPE="$BUILD_TYPE" \
-DCMAKE_PREFIX_PATH="$INSTALL_PATH" -DCMAKE_INSTALL_PREFIX="$INSTALL_PATH" -DCMAKE_CXX_FLAGS="-I $INSTALL_PATH/include -L $INSTALL_PATH/lib" \
.. && \
make -j$numproc && make install

cd $BUILD_PATH

#build OpenMVG
cd $BUILD_PATH
git clone --recursive https://github.com/openMVG/openMVG.git
cd openMVG
git checkout 35da8f72c68
cd ..
git checkout -- src/CMakeLists.txt
git checkout master
cd $BUILD_PATH
#echo "Copying patch of CMakeLists.txt for OpenMVG..."
cp $ITR_PATH/patches/openMVG/CMakeLists.txt openMVG/src/CMakeLists.txt
cp $ITR_PATH/patches/openMVG/InterfaceMVS.h openMVG/src/software/SfM/export/InterfaceMVS.h
mkdir -p openMVG_Build
cd openMVG_Build && \
cmake -DEIGEN_INCLUDE_DIR_HINTS="$EIGEN_DIR" \
-DCMAKE_MODULE_PATH="$EIGEN_DIR/cmake" \
-DEIGEN_INCLUDE_DIR="$EIGEN_DIR" \
-DCMAKE_BUILD_TYPE="$BUILD_TYPE" \
-DMINIGLOG="OFF" \
-DCMAKE_POLICY_DEFAULT_CMP0100="NEW" \
-DCeres_DIR="$BUILD_PATH/ceres-solver/build_dir" \
-DCMAKE_PREFIX_PATH="$INSTALL_PATH" -DCMAKE_INSTALL_PREFIX="$INSTALL_PATH" -DCMAKE_CXX_FLAGS="-I $INSTALL_PATH/include -I $INSTALL_PATH/include/eigen3 -I/usr/local/include -L $INSTALL_PATH/lib -framework Accelerate -lglog -L/usr/local/lib -lgflags" \
../openMVG/src && \
make -j$numproc && make install
# #-Dglog_DIR="$INSTALL_PATH"

cd $BUILD_PATH
# fix missing cereal library - cereal is header only and OpenMVG cmake non-release target definitions contain it for some reason
cat $INSTALL_PATH/share/OpenMVG/cmake/OpenMVGTargets.cmake | sed 's|;cereal||g' > $INSTALL_PATH/share/OpenMVG/cmake/OpenMVGTargets_new.cmake
mv $INSTALL_PATH/share/OpenMVG/cmake/OpenMVGTargets_new.cmake $INSTALL_PATH/share/OpenMVG/cmake/OpenMVGTargets.cmake
#
# #copy easyexif headers
# echo "Copying easyexif: $INSTALL_PATH/openMVG/third_party/easyexif"
# mkdir -p $INSTALL_PATH/include/openMVG/third_party/easyexif && cp $BUILD_PATH/openMVG/src/third_party/easyexif/exif.h $INSTALL_PATH/include/openMVG/third_party/easyexif/
#
cd $BUILD_PATH

#build libnabo
git clone https://github.com/ethz-asl/libnabo.git
cd libnabo && mkdir build
cd build && \
cmake -DCMAKE_BUILD_TYPE="$BUILD_TYPE" \
-DEIGEN_INCLUDE_DIR_HINTS="$EIGEN_DIR" \
-DEIGEN_DIR="$EIGEN_DIR" \
-DEIGEN_INCLUDE_DIR="$EIGEN_DIR" \
-DCMAKE_PREFIX_PATH="$INSTALL_PATH" -DCMAKE_INSTALL_PREFIX="$INSTALL_PATH" -DCMAKE_CXX_FLAGS="-I $INSTALL_PATH/include -I/usr/local/include -L $INSTALL_PATH/lib" \
.. && make -j$numproc && make install

cd $BUILD_PATH

#build libpointmatcher
git clone https://github.com/ethz-asl/libpointmatcher.git
cd libpointmatcher
mkdir build
cd build && \
cmake -DEIGEN_INCLUDE_DIR="$EIGEN_DIR" -DCMAKE_BUILD_TYPE="$BUILD_TYPE" \
-Dlibnabo_DIR="$INSTALL_PATH/share/libnabo/cmake" \
-DCMAKE_PREFIX_PATH="$INSTALL_PATH" -DCMAKE_INSTALL_PREFIX="$INSTALL_PATH" -DCMAKE_CXX_FLAGS="-I $INSTALL_PATH/include -L $INSTALL_PATH/lib" \
.. && \
make -j$numproc && make install

cd $BUILD_PATH
git clone https://github.com/brejchajan/nvm_to_openmvg.git
cd nvm_to_openmvg
mkdir build
cd build
cmake -DCMAKE_BUILD_TYPE="$BUILD_TYPE" \
-DEIGEN_INCLUDE_DIR_HINTS="$EIGEN_DIR" \
-DCMAKE_FIND_DEBUG_MODE=ON -DCMAKE_PREFIX_PATH="$INSTALL_PATH" -DCMAKE_LIBRARY_PATH="$INSTALL_PATH/lib;$INSTALL_PATH/lib" \
-DCMAKE_INSTALL_PREFIX="$INSTALL_PATH" -DCMAKE_CXX_FLAGS="-I$INSTALL_PATH/include -I$INSTALL_PATH/include/openMVG -L$INSTALL_PATH/lib -L$INSTALL_PATH/lib" \
-DCMAKE_EXE_LINKER_FLAGS="-L$INSTALL_PATH/lib" ..
make -j$numproc && make install

cd $BUILD_PATH

#-Wl,-rpath-link,$INSTALL_PATH/lib:$INSTALL_PATH/lib
mkdir -p itr/build
cd itr/build
cmake -DCMAKE_BUILD_TYPE="$BUILD_TYPE" \
-DEIGEN_INCLUDE_DIR_HINTS="$EIGEN_DIR" \
-DCMAKE_FIND_DEBUG_MODE=ON -DCMAKE_PREFIX_PATH="$INSTALL_PATH" -DCMAKE_LIBRARY_PATH="$INSTALL_PATH/lib;$INSTALL_PATH/lib" \
-DCMAKE_INSTALL_PREFIX="$INSTALL_PATH" -DCMAKE_CXX_FLAGS="-I$INSTALL_PATH/include -I$INSTALL_PATH/include/openMVG -L$INSTALL_PATH/lib -L$INSTALL_PATH/lib" \
-DCMAKE_EXE_LINKER_FLAGS="-L$INSTALL_PATH/lib" \
$ITR_PATH
make -j$numproc && make install

comment="# added by itr installer"
echo "Do you want to add install dir bin and lib, to PATH in ~/.bashrc? Backup of ~/.bashrc will be made. [y/n]"
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
		echo "export LD_LIBRARY_PATH=$INSTALL_PATH/lib:\$LD_LIBRARY_PATH" >> ~/.bashrc

		echo "Added bin and library paths to ~/.bashrc."
	else
		echo "Skipping adding install path to ~/.bashrc, because it is already present."
	fi
fi
