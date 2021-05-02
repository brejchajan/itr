# Author: Jan Brejcha <brejcha@adobe.com>, <brejcha@gmail.com>, <ibrejcha@fit.vutbr.cz>
# Project: Immersive Trip Reports 2017-2018
# AdobePatentID="P7840-US"
#
# Project: LandscapeAR 2018-2020
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


FILE(GLOB_RECURSE INCLUDES "${PROJECT_SOURCE_DIR}/include/*.h")
include_directories(${PROJECT_SOURCE_DIR}/include)
aux_source_directory(${PROJECT_SOURCE_DIR}/src SOURCES)
aux_source_directory(${PROJECT_SOURCE_DIR}/src/terrainsampling SOURCES)

find_package(OpenGL COMPONENTS GLX EGL OpenGL)
if ((TARGET OpenGL::OpenGL AND TARGET OpenGL::GLX AND TARGET OpenGL::EGL))
    SET(OPENGL_EGL_FOUND true)
    aux_source_directory(${PROJECT_SOURCE_DIR}/src/egl SOURCES)
endif()

add_executable(${PROJECT_NAME} ${SOURCES} ${INCLUDES})

include_directories(${PROJECT_SOURCE_DIR}/thirdparty/nanoflann/include)
include_directories(${PROJECT_SOURCE_DIR}/thirdparty/json/src)
include_directories(${PROJECT_BINARY_DIR})

# find dependent libraries
find_package(OsgEarth REQUIRED)
find_package(OpenSceneGraph 3.5.6 REQUIRED COMPONENTS osgUtil osgDB osg osgGA osgViewer)
find_package(OpenThreads REQUIRED)
find_package(OpenMVG REQUIRED)
find_package(libpointmatcher 1.2.3 REQUIRED)
#find_package(Eigen3 QUIET)
find_package(glog 0.3.5 REQUIRED)
find_package(LibXML2 REQUIRED)

# BOOST
set(Boost_USE_STATIC_LIBS OFF)
set(Boost_USE_MULTITHREADED ON)
set(Boost_USE_STATIC_RUNTIME OFF)
find_package(Boost 1.60 REQUIRED thread chrono program_options date_time filesystem system regex iostreams)

include_directories(${OSGEARTH_INCLUDE_DIR})
include_directories(${OSG_INCLUDE_DIR})
include_directories(${OPENMVG_INCLUDE_DIRS})
include_directories(${OPENMVG_INCLUDE_DIRS}/third_party)
include_directories(${libpointmatcher_INCLUDE_DIRS})

#EIGEN COMES THROUGH OpenMVG CMake transitivity!
#if (EIGEN_INCLUDE_DIR_HINTS)
#        include_directories(${EIGEN_INCLUDE_DIR_HINTS})
#else()
#        if (NOT EIGEN3_FOUND)
#                message(SEND_ERROR "EIGEN3 NOT FOUND")
#        else()
#                include_directories(${EIGEN3_INCLUDE_DIR})
#        endif()
#endif()

include_directories(${LIBXML2_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME} ${OSGEARTH_LIBRARY})
target_link_libraries(${PROJECT_NAME} ${OSGEARTHUTIL_LIBRARY})
target_link_libraries(${PROJECT_NAME} ${OSGEARTHSYMBOLOGY_LIBRARY})

target_link_libraries(${PROJECT_NAME} cpp_exiftool)
target_link_libraries(${PROJECT_NAME} ${OPENTHREADS_LIBRARY})

target_link_libraries(${PROJECT_NAME} ${OSG_LIBRARY})
target_link_libraries(${PROJECT_NAME} ${OSGVIEWER_LIBRARY})
target_link_libraries(${PROJECT_NAME} ${OSGGA_LIBRARY})
target_link_libraries(${PROJECT_NAME} ${OSGDB_LIBRARY})
target_link_libraries(${PROJECT_NAME} ${OSGUTIL_LIBRARY})


target_link_libraries(${PROJECT_NAME}
    OpenMVG::openMVG_easyexif
    OpenMVG::openMVG_exif
    OpenMVG::openMVG_fast
    OpenMVG::openMVG_features
    OpenMVG::openMVG_geometry
    OpenMVG::openMVG_image
    OpenMVG::openMVG_kvld
    OpenMVG::openMVG_lemon
    OpenMVG::openMVG_linearProgramming
    OpenMVG::openMVG_lInftyComputerVision
    OpenMVG::openMVG_matching
    OpenMVG::openMVG_matching_image_collection
    OpenMVG::openMVG_multiview
    OpenMVG::openMVG_numeric
    OpenMVG::openMVG_robust_estimation
    OpenMVG::openMVG_sfm
    OpenMVG::openMVG_stlplus
    OpenMVG::openMVG_system)
target_link_libraries(${PROJECT_NAME} ${libpointmatcher_LIBRARIES})
target_link_libraries(${PROJECT_NAME} glog::glog)
target_link_libraries(${PROJECT_NAME} ${LIBXML2_LIBRARIES})
#target_link_libraries(${PROJECT_NAME} "easyexif")

if(Boost_FOUND)
    include_directories(${Boost_INCLUDE_DIRS})
    target_link_libraries(${PROJECT_NAME} ${Boost_LIBRARIES})
endif()

if (OPENGL_EGL_FOUND)
    MESSAGE("OpenGL GLVND EGL FOUND, enabling offscreen EGL rendering option")
    target_compile_definitions(${PROJECT_NAME} PRIVATE WITH_EGL)
    target_link_libraries(${PROJECT_NAME} OpenGL::OpenGL OpenGL::GLX OpenGL::EGL)
endif()

install(TARGETS ${PROJECT_NAME} RUNTIME DESTINATION bin)
