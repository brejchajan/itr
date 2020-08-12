/**
* @Author: Jan Brejcha <janbrejcha>
* @Date:   28.1.2018
* @Email:  brejcha@adobe.com, ibrejcha@fit.vutbr.cz, brejchaja@gmail.com
*/
/*
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
*/


#ifndef TERRAINRENDERER_H
#define TERRAINRENDERER_H

#ifdef WITH_EGL
#include "egl/pixelbufferegl.h"
#include "egl/eglcapabilities.h"
#undef Success
#undef Status
#endif

//local headers
#include "gridsampler.h"
#include "preloadarearangecallback.h"
#include "terrainsampling/equirectcamerabuilder.h"
#include "util.h"

//osgEarth headers
#include <osgEarth/MapNode>
#include <osgEarth/SpatialReference>
#include <osgEarth/GeoData>
#include <osgEarth/GeoTransform>


//OSG headers
#include <osg/ref_ptr>
#include <osg/Vec3d>
#include <osgViewer/Viewer>
#include <osgDB/DatabasePager>

//stl headers
#include <memory>
#include <stdexcept>
#include <fstream>

using namespace osgEarth;
using namespace osgViewer;
using namespace std;

namespace itr {

/**
 * @brief The TerrainRenderer class
 * Automatically visits the defined places and renders them.
 */
class TerrainRenderer{
public:

    enum CameraMode{
        EQUIRECTANGULAR,
        PERSPECTIVE
    };

    /**
     * @brief TerrainRenderer
     * Creates terrain renderer to render panoramas sampled on rectangular grid.
     * It can operate in two modes - equirectangular to render equirectangular
     * panoramic images, and perspective, to render overlapping perspective
     * imagery around each position on the rectangular grid.
     * @param mapNode   A map to render.
     * @param center    Center geo-coordinate of the grid.
     * @param width     Width of the geo-rectangle in meters.
     * @param height    Height of the geo-rectangle in meters.
     * @param offset_x offset between consecutive samples in x-direction
     * in meters
     * @param offset_y offset between consecutive samples in y-direction
     * in meters
     * @param face_resolution   resolution of a single face of the underlying
     * cubemap. The the width of the quirectangular panorama is then
     * 4*face_resolution, height 2*face_resolution.
     * @param outpath   output path where to store the rendered images
     * @param camera_mode   CameraMode::EQUIRECTANGULAR for equirectangular
     *                      panoramas, CameraMode::PERSPECTIVE for overlapping
     *                      perspective imagery.
     * @param fov   Aplicable when perspective camera mode is used.
     *              Field-of-view in degrees for each perspective view.
     *              Default = 60 degrees.
     * @param number_views Aplicable when perspective camera mode is used.
     *                     Number of views sampled around the vertical axis.
     *                     The views are sampled uniformly, so fov and this
     *                     parameter defines the overlap between consecutive
     *                     views. Default = 12, which means overlap of 50%.
     */
    TerrainRenderer(Viewer *viewer, MapNode *mapNode,
                    GeoPoint center, double width, double height,
                    double offset_x, double offset_y, int face_resolution,
                    string outpath,
                    CameraMode camera_mode = CameraMode::EQUIRECTANGULAR,
                    float fov = 60.0, unsigned int number_views = 12,
                    bool useEGL = false);


    /**
     * @brief render
     * Start the rendering process of all places of this terrain renderer.
     */
    void render();

    static void waitUntilTerrainLoaded(Viewer *viewer);

    static void waitUntilTerrainLoadingStarts(Viewer *viewer);



private:
    /// Geo coordinates of places to visit, in ECEF coordinates.
    vector<osg::Vec3d> places;

    Viewer *viewer;

    osg::ref_ptr<MapNode> mapNode;

    std::shared_ptr<GridSampler> gridSampler;

    int faceResolution;

    SpatialReference *srs;

    std::shared_ptr<EquirectCameraBuilder> cubeCam;

    WindowCaptureCallback *wcc;

    string outpath;

    CameraMode cameraMode;

    float perspective_fov;

    unsigned int num_perspective_views;

    bool useEGL;

    /**
     * @brief moveCameraToPlace
     * Moves the camera to the specified place.
     * The camera is rotated with the unit rotation matrix, so it is
     * upright on the given place.
     * @param place to place the camera.
     * @param heading heading of the camera in degrees. Default 0.0.
     */
    void moveCameraToPlace(osg::Vec3d &place, float heading = 0.0);

    /**
     * @brief resizeWindows
     * Resizes windows according to the face resolution.
     */
    void resizeWindows();
};

}

#endif // TERRAINRENDERER_H
