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


#include "terrainsampling/terrainrenderer.h"

namespace itr {


TerrainRenderer::TerrainRenderer(Viewer *viewer, MapNode *mapNode,
                                 GeoPoint center, double width, double height,
                                 double offset_x, double offset_y,
                                 int face_resolution, string outpath,
                                 CameraMode camera_mode,
                                 float fov, unsigned int number_views,
                                 bool useEGL):
    viewer(viewer), mapNode(mapNode), faceResolution(face_resolution),
    outpath(outpath), cameraMode(camera_mode),
    perspective_fov(fov), num_perspective_views(number_views),
    useEGL(useEGL)
{
    std::cout << "Creating grid sampler..." << std::endl;
    gridSampler = shared_ptr<GridSampler>(
                new GridSampler(mapNode, center, width, height,
                                offset_x, offset_y, 50.0));
    gridSampler->sample(places);
    std::cout << "Sampled " << places.size() << " places." << std::endl;
    places.clear();

    srs = SpatialReference::get("epsg:4326");

    if (cameraMode == CameraMode::EQUIRECTANGULAR)
    {
        cubeCam = shared_ptr<EquirectCameraBuilder>(
                    new EquirectCameraBuilder(viewer, face_resolution));
    }
    else {
        wcc = new WindowCaptureCallback;
        wcc->addToViewerMainCamera(*viewer);
    }
    viewer->realize();

    if (!Util::checkDirExists(outpath))
    {
        throw std::runtime_error("Unable to create output directory: " +
                                 outpath);
    }

}

void TerrainRenderer::resizeWindows()
{
    double fovy, aspect, znear, zfar;
    viewer->getCamera()->getProjectionMatrixAsPerspective(fovy, aspect, znear, zfar);
    znear = 1;
    zfar = 500000;

    if (cameraMode == CameraMode::EQUIRECTANGULAR)
    {
        cubeCam->getCaptureCallback()->setNearFar(znear, zfar);
        viewer->getCamera()->setProjectionMatrixAsPerspective(fovy, aspect,
                                                              znear, zfar);
    }
    else
    {
        if (!useEGL)
        {
            osgViewer::ViewerBase::Windows windows;
            viewer->getWindows(windows);
            int x, y, ww, hw;
            for (size_t i = 0; i < windows.size(); ++i)
            {
                windows[i]->getWindowRectangle(x, y, ww, hw);
                windows[i]->setWindowRectangle(x, y, faceResolution, faceResolution);
                viewer->getCamera()->setViewport(0, 0, faceResolution, faceResolution);
            }
        }
#ifdef WITH_EGL
        else
        {
            PixelBufferEGL *gc = dynamic_cast<PixelBufferEGL*>(viewer->getCamera()->getGraphicsContext());
            if (gc)
            {
                std::cout << "resizing PixelBufferEGL" << std::endl;
                int hw = gc->getHeight();
                //gc->makeCurrent();
                gc->resizeImplementation(faceResolution, faceResolution);
                //gc->releaseContext();
                viewer->getCamera()->setViewport(0, 0, faceResolution, faceResolution);
            }
        }
#endif

        viewer->getCamera()->setProjectionMatrixAsPerspective(static_cast<double>(perspective_fov), 1.0,
                                                              znear, zfar);
        wcc->setNearFar(znear, zfar);
    }
}

void TerrainRenderer::moveCameraToPlace(osg::Vec3d &place, float heading)
{
    double heading_rad = ((static_cast<double>(heading) * M_PI) / 180.0);

    GeoPoint gp(srs->getGeocentricSRS(), place.x(), place.y(), place.z(),
                ALTMODE_ABSOLUTE);
    Matrix l2w;
    gp.createLocalToWorld(l2w);

    osg::Vec3d up = Vec3d(0.0, 0.0, -1.0);
    osg::Vec3d up_world = up * l2w;
    Quat heading_rot(heading_rad, up);
    osg::Vec3d look = heading_rot * Vec3d(0.0, 1.0, 0.0) * l2w;
    osg::Vec3d forward = look - place;
    viewer->getCamera()->setViewMatrixAsLookAt(place, place + forward, up_world);
}


void TerrainRenderer::waitUntilTerrainLoadingStarts(Viewer *viewer)
{
    osgDB::DatabasePager *dp = viewer->getDatabasePager();
    unsigned int fileReqListSize = dp->getFileRequestListSize();
    std::cout << "Waiting until terrain loading starts..." << std::endl;
    while (fileReqListSize <= 0)
    {
        //wait until several terrain LODs load
        viewer->frame();
        fileReqListSize = dp->getFileRequestListSize();
    }
    std::cout << "Terrain loading started." << std::endl;
}


void TerrainRenderer::waitUntilTerrainLoaded(Viewer *viewer)
{
    osgDB::DatabasePager *dp = viewer->getDatabasePager();

    unsigned int fileReqListSize = 100;
    std::cout << "Waiting for the terrain to load..." << ", viewer done: " << viewer->done() << std::endl;
    while (!viewer->done() && dp->getRequestsInProgress())//fileReqListSize > 0
    {
        std::cout << "file req list size: " << dp->getFileRequestListSize() << std::endl;
        viewer->frame();
        fileReqListSize = dp->getFileRequestListSize();
    }
    std::cout << "Terrain loaded." << std::endl;
}

void TerrainRenderer::render()
{
    std::cout << "Starting rendering..." << std::endl;
    mapNode->getOrCreateStateSet()->setMode(GL_CULL_FACE, osg::StateAttribute::ON);
    viewer->setCameraManipulator(nullptr);
    viewer->getCamera()->setComputeNearFarMode(Camera::ComputeNearFarMode::DO_NOT_COMPUTE_NEAR_FAR);

    unsigned int num_views = 1;
    if (cameraMode == CameraMode::PERSPECTIVE)
    {
        num_views = num_perspective_views;
        resizeWindows();
    }

    float heading_delta = 360.0f / num_views;
    osg::Vec3d p = gridSampler->nextPosition();
    osg::Vec3d elev_p(p.x(), p.y(), p.z());
    Eigen::Vector3d scene_center(elev_p.x(), elev_p.y(), elev_p.z());

    boost::filesystem::path output_path(outpath);
    boost::filesystem::path scene_info_outpath = output_path / "scene_info.txt";
    std::ofstream scene_info(scene_info_outpath.string());
    scene_info << "center:" << scene_center.x() << " " << scene_center.y() << " " << scene_center.z() << std::endl;
    scene_info.close();

    float heading = 0.0f;
    for (unsigned int i = 0; i < num_views; ++i)
    {
        gridSampler->resetPosition();
        osg::Vec3d p = gridSampler->nextPosition();
        osg::Vec3d elev_p(p.x(), p.y(), p.z());
        moveCameraToPlace(elev_p, heading);
        waitUntilTerrainLoadingStarts(viewer);

        while (p.x() != 0 || p.y() != 0 || p.z() != 0)
        {
            moveCameraToPlace(elev_p, heading);
            //terrain reloads after the camera has been moved
            for (int i = 0; i < 100; ++i)
            {
                //wait a few frames until terrain loading kicks in
                viewer->frame();
            }            waitUntilTerrainLoaded(viewer);

            GeoPoint gp(srs->getGeocentricSRS(), elev_p.x(), elev_p.y(), elev_p.z(),
                        ALTMODE_ABSOLUTE);
            gp.makeGeographic();


            string output_filename = to_string(gp.x()) + "_" +
                    to_string(gp.y()) + "_" + to_string(gp.z());
            output_path = boost::filesystem::path(outpath);

            if (cameraMode == CameraMode::EQUIRECTANGULAR)
            {
                output_filename += ".png";
                output_path /= output_filename;
                cubeCam->getCaptureCallback()->saveOnNextFrame(output_path.string(), true);
                viewer->frame();
                cubeCam->getCaptureCallback()->waitUntilDone();
            }
            else
            {
                output_filename += "_" + std::to_string(heading);
                string modelview_filename = output_filename + "_pose.txt";
                string projection_filename = output_filename + "_projection.txt";
                output_filename += ".png";
                boost::filesystem::path modelview_outpath = output_path / modelview_filename;
                boost::filesystem::path projection_outpath = output_path / projection_filename;

                const Eigen::Matrix4d modelview = Eigen::Map<Eigen::Matrix4d>(viewer->getCamera()->getViewMatrix().ptr());
                osg::Matrixd proj = viewer->getCamera()->getProjectionMatrix();
                const Eigen::Matrix4d proj_eigen = Eigen::Map<Eigen::Matrix4d>(proj.ptr());

                Util::printMatrixToFile(modelview_outpath.string(), modelview, scene_center);
                Util::printMatrixToFile(projection_outpath.string(), proj_eigen);

                if (useEGL)
                {
                    wcc->saveOnNextFrame(output_path.string());
                    viewer->frame();
                    wcc->saveOnNextFrame(output_path.string());
                    viewer->frame();
                }

                output_path /= output_filename;
                wcc->saveOnNextFrame(output_path.string(), true);
                viewer->frame();
                wcc->waitUntilDone();
            }
            p = gridSampler->nextPosition();
            elev_p = osg::Vec3d(p.x(), p.y(), p.z());
        }
        heading += heading_delta;
    }

}

} //namespace itr
