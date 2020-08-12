/**
* @Author: Jan Brejcha <janbrejcha>
* @Date:   07.08.2017
* @Email:  brejcha@adobe.com, ibrejcha@fit.vutbr.cz, brejchaja@gmail.com
* @Project: ImmersiveTripReports 2017-2018
* AdobePatentID="P7840-US"
*
* @Project LandscapeAR 2018
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


#ifdef WITH_EGL
#include "egl/pixelbufferegl.h"
#include "egl/eglcapabilities.h"
#undef Success
#undef Status
#endif

// STL headers
#include <iostream>
#include <string>
#include <set>
#include <memory>
#include <cmath>
#include <iomanip>
#include <cstdlib>
#include <cstdio>
#include <ctime>
#include <tuple>
#include <sstream>
#include <cstdio>

// local headers
#include "sfmloader.h"
#include "ITRConfig.h"
#include "photoview.h"
#include "bbox3d.h"
#include "animationspline.h"
#include "jsonexporter.h"
#include "terrainsampling/terrainrenderer.h"
#include "preloadarearangecallback.h"
#include "util.h"

//OSGEarth headers
#include "osgEarth/Map"
#include "osgEarth/MapNode"
#include "osgEarthDrivers/xyz/XYZOptions"
#include "osgEarth/ImageLayer"
#include "osgEarthUtil/EarthManipulator"
#include "osgEarthUtil/ExampleResources"
#include "osgEarthUtil/LODBlending"
#include "osgEarthUtil/LogarithmicDepthBuffer"
#include "osgEarth/Viewpoint"
#include "osgEarthDrivers/engine_mp/MPTerrainEngineOptions"
#include "osgEarth/TerrainEngineNode"
#include "osgEarth/Metrics"
#include "osgEarth/Registry"
#include "osgEarth/Capabilities"
#include "osgEarth/GeoMath"

//OSG headers
#include "osgViewer/Viewer"
#include "osgViewer/config/SingleWindow"
#include "osgViewer/config/SingleScreen"
#include "osgGA/TrackballManipulator"
#include "osg/ArgumentParser"
#include "osg/Point"
#include "osgGA/GUIEventHandler"
#include "osgGA/CameraManipulator"
#include "osgGA/AnimationPathManipulator"
#include "osgGA/FlightManipulator"
#include "osgGA/FirstPersonManipulator"
#include "osgEarth/CullingUtils"
#include "osg/CullFace"
#include "osgUtil/Optimizer"
#include "osgDB/XmlParser"

//BOOST headers
#include <boost/filesystem.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/regex.hpp>
#include <boost/regex/pattern_except.hpp>

//openMVG headers
#include "openMVG/stl/split.hpp"
#include "openMVG/types.hpp"

using namespace osgEarth;
using namespace osgEarth::Drivers;
using namespace osgEarth::Util;
using namespace osgViewer;
using namespace std;
using namespace osgEarth::Drivers::MPTerrainEngine;
using namespace itr;
using std::size_t;

class AdobeViewerKeyboardEventHandler : public osgGA::GUIEventHandler
{
public:

    AdobeViewerKeyboardEventHandler(ref_ptr<MapNode> mapNode,   //FIXME smart pointer
                                    ref_ptr<EarthManipulator> manip,
                                    vector< shared_ptr<AbstractPhotoView> > photoviews,
                                    vector< shared_ptr<SfMLoader> >sfmLoaders,
                                    string root_dir,
                                    double center_lat = 360, //nonsense values will be ignored
                                    double center_lon = 360,
                                    bool sortPhotoviewsDate = true,
                                    bool useEGL = false):
        mapNode(mapNode), photoviews(photoviews), earth_manip(manip),
        cameraManipInUse(true), sfmLoaders(sfmLoaders),
        center_lat(center_lat), center_lon(center_lon),
        root_dir(root_dir),
        sortPhotoviewsDate(sortPhotoviewsDate),
        output_path("."),
        useEGL(useEGL),
        pv_state(PhotoviewsState::SHOW_ALL),
        znear(PhotoView::PHOTOPLANE_OFFSET - 0.1),
        zfar(500000),
        exportFlythrough(true),
        exportOnlyPicked(false)
    {
        if (znear <= 0)
        {
            znear = 1;
        }
        srs = SpatialReference::get("epsg:4326");
        fp_manip = new osgGA::FirstPersonManipulator;
        fp_manip->setVerticalAxisFixed(false);

        osgViewer::ViewerBase::Windows windows;
    }


    virtual bool handle(const osgGA::GUIEventAdapter& ea,osgGA::GUIActionAdapter& aa)
    {
        osgViewer::Viewer* viewer = dynamic_cast<osgViewer::Viewer*>(&aa);
        if (!viewer)
        {
            return( false );
        }

        switch(ea.getEventType())
        {
        case(osgGA::GUIEventAdapter::KEYDOWN):
        {
            switch(ea.getKey())
            {
            case 't':
                //toggle photoviews position markers
                for (shared_ptr<AbstractPhotoView> pv : photoviews)
                {
                    pv->toggleViewState();
                }
                return true;
                break;
            case 'c':
                //toggle camera manipulator
                toggleCameraManipulator(viewer);
                return true;
                break;
            case 'f':
                //flythrough
                flythroughAnimation(viewer);
                break;
            case 'p':
                //continue flythrough
                flythroughAnimationFromPoint(viewer);
                break;
            case 'o':
                fitCamerasToTerrain();
                break;
            case 'u':
                togglePointCloud();
                break;
            case 'i':
                adjustRotationScale();
                break;
            case 'n':
                runICP();
                break;
            case 'j':
                saveToJson();
                break;
            case 'a':
                adjustGPS(viewer);
                break;
            case 'r':
                renderPhotoViews(viewer);
                break;
            case 'd':
                calculateMean3DDistance();
                break;
            case 'h':
                togglePhotoviews();
                break;
            case 'g':
                goToFirstPhotoview();
            case 'k':
            {
                if (anim_spline)
                {
                    anim_spline->togglePathShown();
                }
                break;
            }
            /*case 's': //attempt to write saving from interactive view
                shared_ptr<SfMLoader> loader;
                shared_ptr<AbstractPhotoView> p = std::make_shared<PhotoView>(mapNode, "screencapture");
                osg::ref_ptr<WindowCaptureCallback> wcc = new WindowCaptureCallback;
                wcc->addToViewerMainCamera(*viewer);
                Hash_Map<IndexT, Eigen::Vector3d> cld;
                vector<osg::Vec3d> points;
                Eigen::Vector3d centroid(0, 0, 0);
                void renderPhotoViewIntoDir(viewer,
                                            loader, p,
                                            output_path,
                                            wcc,
                                            0, cld, points,
                                            centroid,
                                            bool dry_run = false,
                                            string output_render_base = "")*/
            default:
                return false;
            }
        }
        case (osgGA::GUIEventAdapter::MOVE):
        {
            // Record mouse location for the button press
            //   and move events.
            _mX = ea.getX();
            _mY = ea.getY();
            return (false);
        }
        case osgGA::GUIEventAdapter::RELEASE:
        {
            // If the mouse hasn't moved since the last
            //   button press or move event, perform a
            //   pick. (Otherwise, the trackball
            //   manipulator will handle it.)
            if (_mX == ea.getX() && _mY == ea.getY())
            {
                osg::ref_ptr<osg::Node> pickedNode = pick(ea.getXnormalized(),
                                                          ea.getYnormalized(),
                                                          viewer);
                if (pickedNode.valid())
                {
                    //select this photoview to be used as control point
                    //for flythrough animation
                    PositionAttitudeTransform *gt = dynamic_cast<PositionAttitudeTransform*>(pickedNode->getParent(0));
                    shared_ptr<AbstractPhotoView> p = findPhotoViewByTransf(gt);
                    if ((ea.getModKeyMask() & osgGA::GUIEventAdapter::MODKEY_LEFT_CTRL) == osgGA::GUIEventAdapter::MODKEY_LEFT_CTRL)
                    {
                        if (p)
                        {
                            osg::Quat o = p->getOrientationLocal();
                            double yaw, pitch, roll;
                            itr::Util::toEulerAngle(o, roll, pitch, yaw);
                            yaw = -yaw * 180.0 / M_PI;
                            pitch = -pitch * 180.0 / M_PI;
                            roll = (roll * 180.0 / M_PI);
                            std::cerr << "test yaw: " << yaw << ", pitch: " << roll << ", roll: " << pitch << std::endl;

                            vector< shared_ptr<AbstractPhotoView> >::iterator it = std::find(picked_photoviews.begin(), picked_photoviews.end(), p);
                            if (it == picked_photoviews.end())
                            {
                                p->setOpacity(1.0);
                                //picked_photoviews.push_back(p);
                                //we need to insert forwards, since the
                                //animation path works in reverse order.
                                picked_photoviews.insert(picked_photoviews.begin(), p);
                            }
                            else
                            {
                                p->setOpacity(0.5);
                                picked_photoviews.erase(it);
                            }
                        }
                        else
                        {
                            std::cerr << "Picked photoview cannot be found." << std::endl;
                        }
                    }
                    else
                    {
                        moveCameraAtPhotoView(p, viewer, true);
                    }
                }
                else
                {
                    std::cout << "Invalid node." << std::endl;
                }
            }
            return (false);
        }
        default:
            return false;
        }
    }

    void goToFirstPhotoview()
    {
        if (photoviews.size() > 0)
        {
            shared_ptr<AbstractPhotoView> &p = photoviews[0];
            PositionAttitudeTransform *gt = p->getTransf();

            Viewpoint vp;
            vp.focalPoint()->set(srs->getGeocentricSRS(), gt->getPosition(), AltitudeMode::ALTMODE_ABSOLUTE);
            vp.range()->set(400, Units::METERS);
            earth_manip->setHomeViewpoint(vp, 3.0);
            earth_manip->home(3.0);
        }
    }

    void togglePhotoviews()
    {
        pv_state = static_cast<PhotoviewsState>((pv_state + 1) % PHOTOVIEWS_STATE_LAST);
        switch (pv_state) {
        case SHOW_ALL:
        {
            showAllPhotoViews();
            break;
        }
        case SHOW_ALL_USER:
        {
            showAllUserPhotoViews();
            break;
        }
        case SHOW_PICKED:
        {
            showAllPickedPhotoViews();
            break;
        }
        case SHOW_NONE:
        {
            hideAllPhotoViews();
            break;
        }
        default:
        {
            break;
        }
        }

    }

    void fitIntoWindow(shared_ptr<PhotoView> pv,
                       osgViewer::Viewer* viewer,
                       double fixed_fov = -1,
                       double fixed_aspect = -1,
                       int fixed_width = -1)
    {
        double w = pv->getImageWidth();
        double h = pv->getImageHeight();
        double aspect = w / h;

        if (fixed_aspect > 0)
        {
            aspect = fixed_aspect;
        }

        if (!useEGL)
        {
            std::cout << "Resizing windows..." << std::endl;
            osgViewer::ViewerBase::Windows windows;
            viewer->getWindows(windows);
            std::cout << "nuber of windows " << windows.size() << std::endl;
            int x, y, ww, hw;
            windows[0]->getWindowRectangle(x, y, ww, hw);
            if (fixed_width > 0)
            {
                if (h > w)
                {
                    std::cout << "Fixed height, width: " << static_cast<int>(aspect * fixed_width) << std::endl;
                    windows[0]->setWindowRectangle(0, 0, static_cast<int>(aspect * fixed_width), fixed_width);
                }
                else
                {
                    windows[0]->setWindowRectangle(0, 0, fixed_width, static_cast<int>((1.0/aspect) * fixed_width));
                }
            }
            else
            {
                windows[0]->setWindowRectangle(0, 0, static_cast<int>(hw * aspect), hw);
            }

            std::cout << "Windows resized." << std::endl;
        }
#ifdef WITH_EGL
        else
        {
            PixelBufferEGL *gc = dynamic_cast<PixelBufferEGL*>(viewer->getCamera()->getGraphicsContext());
            if (gc)
            {
                std::cout << "resizing PixelBufferEGL" << std::endl;
                int hw = gc->getHeight();
                int win_width = static_cast<int>(hw * aspect);
                int win_height = hw;
                if (fixed_width > 0)
                {
                    win_width = fixed_width;
                    win_height = static_cast<int>((1.0/aspect) * fixed_width);
                }
                //gc->makeCurrent();
                gc->resizeImplementation(win_width, win_height);
                //gc->releaseContext();
                viewer->getCamera()->setViewport(0, 0, win_width, win_height);
            }
        }
#endif

        std::cout << "setting projection matrix" << std::endl;
        std::cout << "fov: " << fixed_fov << ", " << pv->getFOVYDeg() << ", aspect: " << aspect << ", znear: " << znear << ", zfar: " << zfar << std::endl;
        viewer->getCamera()->setProjectionMatrixAsPerspective(fixed_fov > 0 ? fixed_fov : pv->getFOVYDeg(), aspect,
                                                              znear, zfar);
        osg::Matrixd proj = viewer->getCamera()->getProjectionMatrix();
        const Eigen::Matrix4d proj_eigen = Eigen::Map<Eigen::Matrix4d>(proj.ptr());
        std::cout << proj_eigen << std::endl;

        std::cout << "projection matrix set" << std::endl;
    }

    /**
     * @brief moveCameraAtPhotoView
     * Moves the camera at the specified photoview.
     * @param fitWindow if set to true, window will be resized to precisely fit
     * current photoview.
     * @param p
     */
    void moveCameraAtPhotoView(shared_ptr<AbstractPhotoView> p,
                               osgViewer::Viewer* viewer,
                               bool fitWindow = false,
                               double fixed_fov = -1,
                               double fixed_aspect = -1,
                               int fixed_width = -1,
                               bool overlay = false)
    {
        shared_ptr<PhotoView> pv = std::dynamic_pointer_cast<PhotoView>(p);
        if (pv)
        {
            pickedPhotoView = pv;
            //load full res photo
            pv->loadFullResolutionPhoto();
            std::cout << "Updating photo texture" << std::endl;
            pv->updatePhotoTexture(overlay);
            std::cout << "Photo texture updated" << std::endl;
        }
        //set the camera to match this photoview
        PositionAttitudeTransform *gt = p->getTransf();
        if (gt != NULL)
        {
            Quat orientation = p->orientation();
            double d = PhotoView::PHOTOPLANE_OFFSET;
            Vec3d eye = orientation * Vec3d(0.0, 0.0, 1.0) * d;

            Viewpoint vp;
            vp.focalPoint()->set(srs->getGeocentricSRS(), gt->getPosition(), AltitudeMode::ALTMODE_ABSOLUTE);
            vp.range()->set(400, Units::METERS);
            earth_manip->setHomeViewpoint(vp, 0.0);
            /*viewer->getCamera()->setViewMatrixAsLookAt(gt->getPosition(), gt->getPosition() + eye, orientation * osg::Vec3d(0, -1, 0));*/

            //set first person manip
            fp_manip->setTransformation(gt->getPosition(), gt->getPosition() + eye, orientation * osg::Vec3d(0, -1, 0));
            fp_manip->setHomePosition(gt->getPosition(), gt->getPosition() + eye, orientation * osg::Vec3d(0, -1, 0));
            fp_manip->setVerticalAxisFixed(false);
            double fovy, aspect, _znear, _zfar;

            const Eigen::Matrix4d modelview = Eigen::Map<Eigen::Matrix4d>(fp_manip->getInverseMatrix().ptr());
            std::cout << "modelview matrix: " << modelview << std::endl;

            viewer->getCamera()->setComputeNearFarMode(Camera::ComputeNearFarMode::DO_NOT_COMPUTE_NEAR_FAR);
            viewer->getCamera()->getProjectionMatrixAsPerspective(fovy, aspect,
                                                                  _znear, _zfar);

            if (pv && fitWindow)
            {
                fitIntoWindow(pv, viewer, fixed_fov, fixed_aspect, fixed_width);
            }
            else
            {
                std::cout << "setting projection matrix" << std::endl;
                viewer->getCamera()->setProjectionMatrixAsPerspective(fixed_fov > 0 ? fixed_fov : 60, fixed_aspect > 0 ? fixed_aspect : p->getAspect(),
                                                                      znear, zfar);
                std::cout << "projection matrix set" << std::endl;
            }

            hideOtherPhotoViews(gt);

            if (cameraManipInUse)
            {
                //turn off camera manipulator if needed
                //so we can set the camera
                std::cout << "Toggling camera manipulator" << std::endl;
                toggleCameraManipulator(viewer);
                std::cout << "Camera manipulator toggled" << std::endl;
            }
        }
    }

    void calculateMean3DDistance()
    {
        double total_mean = 0;
        double total_point_cnt = 0;
        vector<double> all_dists;
        vector<double> all_rmse;
        for (shared_ptr<SfMLoader> loader : sfmLoaders)
        {
            total_point_cnt += loader->pointCount();
        }
        for (shared_ptr<SfMLoader> loader : sfmLoaders)
        {
            std::tuple<int, double, double, vector<double> > err = loader->calculate3DStatsForData();
            int cloud_size = std::get<0>(err);
            double mean_error = std::get<1>(err);
            double median_error = std::get<2>(err);
            vector<double> dists = std::get<3>(err);

            if (median_error < 1000000000)
            {
                std::cout << "Scene: " << loader->getComponentDir()
                          << ", num points: " << loader->pointCount()
                          << ", mean 3D distance: " << mean_error
                          << ", median distance: " << median_error << std::endl;
                total_mean += (mean_error * cloud_size) / total_point_cnt;
                all_dists.insert(all_dists.end(), dists.begin(), dists.end());

                vector<double> rmse = loader->reprojectionRMSE();
                all_rmse.insert(all_rmse.end(), rmse.begin(), rmse.end());
            }
        }
        double total_mean_rmse = 0;
        double total_mean_rmse_filtered = 0;
        for (double e : all_rmse)
        {
            //std::cout << "single rmse: " << e << std::endl;
            if (e < 1000000000000)
            {
                total_mean_rmse_filtered += (e / all_rmse.size());
            }
            total_mean_rmse += (e / all_rmse.size());
        }
        std::sort(all_rmse.begin(), all_rmse.end());
        double total_median_rmse = 0.0;
        if (all_rmse.size() > 0)
        {
            total_median_rmse = all_rmse[all_rmse.size() / 2];
        }

        std::sort(all_dists.begin(), all_dists.end());
        double total_median = 0.0;
        if (all_dists.size() > 0)
        {
            total_median = all_dists[all_dists.size() / 2];
        }

        double total_mean_filtered = 0;
        for (double d : all_dists)
        {
            //std::cout << "single dist: " << d << std::endl;
            if (d < 1000000000000)
            {
                total_mean_filtered += (d / all_dists.size());
            }
        }

        std::cout << "TOTAL MEAN 3D DISTANCE: " << total_mean << std::endl;
        std::cout << "TOTAL MEAN 3D DISTANCE FILTERED: " << total_mean_filtered << std::endl;
        std::cout << "TOTAL MEDIAN 3D DISTANCE: " << total_median << std::endl;

        std::cout << "TOTAL MEAN RMSE: " << total_mean_rmse << std::endl;
        std::cout << "TOTAL MEAN RMSE FILTERED: " << total_mean_rmse_filtered << std::endl;
        std::cout << "TOTAL MEDIAN_RMSE: " << total_median_rmse << std::endl;

    }


    /**
     * @brief isAlreadyRendered
     * taken from https://stackoverflow.com/questions/1257721/
     * can-i-use-a-mask-to-iterate-files-in-a-directory-with-boost
     * @param output_path path where rendered files are stored
     * @param file_name base name which we schould check for existence,
     * eg "myphoto" without extension.
     * @return true if <file_name>*_texture.jpg exists in the output_path.
     */
    bool isAlreadyRendered(string output_path, string file_name)
    {

        const std::string target_path(output_path);
        const boost::regex my_filter(file_name + ".*_texture\.jpg");

        //std::vector< std::string > all_matching_files;

        boost::filesystem::directory_iterator end_itr;
        for(boost::filesystem::directory_iterator i(target_path);
            i != end_itr; ++i)
        {
            // Skip if not a file
            if( !boost::filesystem::is_regular_file( i->status() ) ) continue;

            boost::smatch what;

            // For V3:
            if( !boost::regex_match( i->path().filename().string(), what, my_filter ) ) continue;

            // File matches, store it
            return true;
            //all_matching_files.push_back( i->leaf() );
        }
        return false;
    }

    void cleanMapNode()
    {
        std::queue<osg::ref_ptr<osg::Node>> queue;
        queue.push(mapNode);
        while (!queue.empty())
        {
            osg::ref_ptr<osg::Node> n = queue.front();
            queue.pop();
            osg::Group *g = n->asGroup();
            if (g && g->getNumChildren() == 0)
            {
                for (unsigned int i = 0; i < g->getNumParents(); ++i)
                {
                    //g->setDataVariance(DataVariance::DYNAMIC);
                    //g->getParent(i)->setDataVariance(DataVariance::DYNAMIC);
                    g->getParent(i)->removeChild(n.get());
                }
            }
            if (g)
            {
                for (unsigned int i = 0; i < g->getNumChildren(); ++i)
                {
                    queue.push(g->getChild(i));
                }
            }
        }


    }

    void renderPhotoViewsMemoryTest(osgViewer::Viewer *viewer)
    {
        if (!viewer->isRealized())
        {
            viewer->realize();
        }
        double dist_thr = 250.0;
        bool firstPhotoview = true;
        for (shared_ptr<SfMLoader> loader : sfmLoaders)
        {
            vector<openMVG::IndexT> viewIds = loader->getPhotoviewKeys(dist_thr * dist_thr);
            for (openMVG::IndexT viewId : viewIds)
            {
                shared_ptr<AbstractPhotoView> p = loader->getPhotoView(viewId);
                moveCameraAtPhotoView(p, viewer, true);
                if (firstPhotoview)
                {
                    firstPhotoview = true;
                    for (int i = 0; i < 100; ++i)
                    {
                        viewer->frame();
                    }
                }
                TerrainRenderer::waitUntilTerrainLoaded(viewer);
                if (!cameraManipInUse)
                {
                    toggleCameraManipulator(viewer);
                }
                hideAllPhotoViews();
                earth_manip->home(0.0);
                for (int i = 0; i < 600; ++i)
                {
                    std::cout << "homing: " << i << std::endl;
                    viewer->frame();
                }
                //osgUtil::Optimizer opt;
                //opt.optimize(mapNode, osgUtil::Optimizer::OPTIMIZE_TEXTURE_SETTINGS);
                //viewer->getDatabasePager()->cancel();
                //viewer->getDatabasePager()->clear();
                showAllPhotoViews();
                std::cout << "viewId test done: " << viewId << std::endl;
            }
        }
    }

    bool isPickedPhotoview(shared_ptr<AbstractPhotoView> p)
    {
        vector< shared_ptr<AbstractPhotoView> >::iterator it =
                std::find(picked_photoviews.begin(), picked_photoviews.end(), p);
        return it != picked_photoviews.end();
    }


    void printTracks(fs::path photo_tracks_outpath,
                     const std::vector< std::tuple<IndexT, std::string, int> > &tracks)
    {
        fstream f(photo_tracks_outpath.string(), f.out);
        for (const std::tuple<IndexT, std::string, int> &track : tracks)
        {
            f << std::get<0>(track); //viewId
            f << " ";
            f << std::get<1>(track); //photoBase
            f << " ";
            f << std::get<2>(track); //num of shared 3D points
            f << std::endl;
        }
        f.close();
    }

    void renderPhotoViewIntoDir(osgViewer::Viewer* viewer,
                                shared_ptr<SfMLoader> loader, shared_ptr<AbstractPhotoView> p,
                                fs::path output_dir,
                                osg::ref_ptr<WindowCaptureCallback> wcc,
                                IndexT viewId, const Hash_Map<IndexT, Eigen::Vector3d> &cld,
                                const vector<osg::Vec3d> &points,
                                Eigen::Vector3d centroid,
                                bool dry_run = false,
                                string output_render_base = "")
    {
        string photoName = p->getPhotoName();
        fs::path photoPath(photoName);
        string photo_base = output_render_base.size() > 0 ? output_render_base : photoPath.stem().string();

        p->setOpacity(1.0f);


        string photo = photo_base + ".png";
        string photo_transparent = photo_base + "_transparent.png";
        string photo_texture = photo_base + "_texture.png";
        string photo_modelview = photo_base + "_modelview.txt";
        string photo_projection = photo_base + "_projection.txt";
        string photo_sfm_points = photo_base + "_sfm_points.txt";
        string photo_sfm_observations = photo_base + "_sfm_observations.txt";
        string photo_terrain_points = photo_base + "_terrain_points.txt";

        fs::path photo_outpath = output_dir / photo;
        fs::path photo_transparent_outpath = output_dir / photo_transparent;
        fs::path photo_texture_outpath = output_dir / photo_texture;
        fs::path photo_modelview_outpath = output_dir / photo_modelview;
        fs::path photo_projection_outpath = output_dir / photo_projection;
        fs::path photo_sfm_points_outpath = output_dir/ photo_sfm_points;
        fs::path photo_sfm_observations_outpath = output_dir / photo_sfm_observations;
        fs::path photo_terrain_points_outpath = output_dir / photo_terrain_points;

        //save 3D points info
        bool render = true;
        if (loader)
        {
            render = loader->savePhotoview3DProjections(
                        viewId, cld, points,
                        photo_sfm_points_outpath.string(),
                        photo_sfm_observations_outpath.string(),
                        photo_terrain_points_outpath.string());
        }
        //render the photo
        if (render)
        {

            const Eigen::Matrix4d modelview = Eigen::Map<Eigen::Matrix4d>(fp_manip->getInverseMatrix().ptr());
            itr::Util::printMatrixToFile(photo_modelview_outpath.string(), modelview, centroid);
            osg::Matrixd proj = viewer->getCamera()->getProjectionMatrix();
            const Eigen::Matrix4d proj_eigen = Eigen::Map<Eigen::Matrix4d>(proj.ptr());
            itr::Util::printMatrixToFile(photo_projection_outpath.string(), proj_eigen);

            if (!dry_run)
            {
                //render terrain corresponding to the photo
                hidePhotoView(p);
                TerrainRenderer::waitUntilTerrainLoaded(viewer);

                if (useEGL)
                {
                    //dirty hack to force the scenegraph to draw into correct buffer
                    wcc->saveOnNextFrame(photo_texture_outpath.string());
                    viewer->frame();
                    wcc->saveOnNextFrame(photo_texture_outpath.string());
                    viewer->frame();
                }
                wcc->setNearFar(znear, zfar);
                wcc->saveOnNextFrame(photo_texture_outpath.string(), true);
                viewer->frame();
                wcc->waitUntilDone();

                showPhotoView(p);
                p->setOpacity(1.0f);
                wcc->saveOnNextFrame(photo_outpath.string());
                viewer->frame();
                wcc->waitUntilDone();

                //render the transparent photo with observations and 3D points
                p->setOpacity(0.5f);
                wcc->saveOnNextFrame(photo_transparent_outpath.string());
                viewer->frame();
                wcc->waitUntilDone();
            }
        }
    }


    void renderPhotoViews(osgViewer::Viewer* viewer, bool onlyPicked = false,
                          int fixed_width = 1024, bool overlay = false,
                          bool dry_run = false,
                          Eigen::Vector3d scene_center = Eigen::Vector3d(0.0, 0.0, 0.0))
    {
        double fixed_fov = 60.0;
        double fixed_aspect = 1.0;

        if (!viewer->isRealized())
        {
            viewer->realize();
        }

        double dist_thr = 250.0;
        osg::ref_ptr<WindowCaptureCallback> wcc = new WindowCaptureCallback;
        wcc->addToViewerMainCamera(*viewer);

        //make sure output dir exists
        fs::path p_outpath(output_path);
        p_outpath /= "real";
        if (!boost::filesystem::exists(p_outpath))
        {
            fs::create_directories(p_outpath);
        }

        fs::path p_outpath_fixed(output_path);
        p_outpath_fixed /= "fixed";
        if (!boost::filesystem::exists(p_outpath_fixed))
        {
            fs::create_directories(p_outpath_fixed);
        }

        fs::path csv_outpath(output_path);
        csv_outpath /= "photo_errors.csv";
        fstream f(csv_outpath.string(), f.out | f.app);
        //write csv header
        f << "photo name";
        f << ";";
        f << "Mean distance 3D sfm - 3D sample";
        f << ";";
        f << "Median distance 3D sfm - 3D sample";
        f << ";";
        f << "3D-2D Reprojection RMSE";
        f << std::endl;
        f << std::flush;

        for (shared_ptr<SfMLoader> loader : sfmLoaders)
        {
            vector<openMVG::IndexT> viewIds = loader->getPhotoviewKeys(dist_thr * dist_thr);
            //gets updated when we waited for the terrain to be loaded
            shared_ptr<AbstractPhotoView> last_photoview;

            vector<openMVG::IndexT> viewIdsToRen;
            for (openMVG::IndexT viewId : viewIds)
            {
                std::cout << "processing viewID: " << viewId << std::endl;
                shared_ptr<AbstractPhotoView> p = loader->getPhotoView(viewId);
                string photoName = p->getPhotoName();
                fs::path photoPath(photoName);
                string photo_base = photoPath.stem().string();
                fs::path photo_parent_path = photoPath.parent_path();
                string pp_base = photo_parent_path.stem().string();
                std::cout << "pp base " << pp_base << std::endl;
                if (pp_base.find("render_uniform") != std::string::npos
                        or pp_base.find("render") != std::string::npos)
                {
                   // we don't want to render renders
                   continue;
                }
                if (onlyPicked && !isPickedPhotoview(p))
                {
                    continue;
                }
                if (p->getAspect() > 2.0)
                {
                    continue;
                }
                viewIdsToRen.push_back(viewId);
            }


            for (openMVG::IndexT viewId : viewIdsToRen)
            {
                std::cout << "Total number of photoviews to render: " << viewIdsToRen.size() << std::endl;
                std::cout << "processing viewID: " << viewId << std::endl;
                shared_ptr<AbstractPhotoView> p = loader->getPhotoView(viewId);
                string photoName = p->getPhotoName();
                fs::path photoPath(photoName);
                string photo_base = photoPath.stem().string();
                fs::path photo_parent_path = photoPath.parent_path();
                string pp_base = photo_parent_path.stem().string();
                std::cout << "pp base " << pp_base << std::endl;
                if (pp_base.find("render_uniform") != std::string::npos
                        or pp_base.find("render") != std::string::npos)
                {
                   // we don't want to render renders
                   continue;
                }
                if (isAlreadyRendered(output_path, photo_base))
                {
                    continue;
                }
                if (onlyPicked && !isPickedPhotoview(p))
                {
                    continue;
                }
                if (p->getAspect() > 2.0)
                {
                    continue;
                }

                //get 3D sfm point cloud for this viewId
                clock_t begin = clock();
                const Hash_Map<IndexT, Eigen::Vector3d> &cld = loader->createCloudMapFromDataForViewId(viewId);
                clock_t end = clock();
                double elapsed_secs = double(end - begin) / CLOCKS_PER_SEC;
                std::cout << "creatingCloudMapFromDataForViewId took: " << elapsed_secs << std::endl;

                begin = clock();
                //sample the terrain in neighbourhood of cld
                vector<osg::Vec3d> points = loader->sampleTerrain(cld);
                end = clock();
                elapsed_secs = double(end - begin) / CLOCKS_PER_SEC;
                std::cout << "terrain sampling took: " << elapsed_secs << std::endl;

                begin = clock();
                std::tuple<int, double, double> err = loader->getPhotoViewError(viewId, cld, points);
                int cloud_size = std::get<0>(err);
                double mean_error = std::get<1>(err);
                double median_error = std::get<2>(err);

                vector<double> rmse = loader->reprojectionRMSE(viewId);
                double total_mean_rmse = 0;
                for (double e : rmse)
                {
                    total_mean_rmse += (e / rmse.size());
                }
                std::sort(rmse.begin(), rmse.end());
                double total_median_rmse = std::numeric_limits<double>::max();
                if (rmse.size() > 0)
                {
                    total_median_rmse = rmse[rmse.size() / 2];
                }
                end = clock();
                elapsed_secs = double(end - begin) / CLOCKS_PER_SEC;
                std::cout << "error calculation took: " << elapsed_secs << std::endl;

                //write csv
                std::ostringstream mean_s, median_s, med_rmse, cloud_size_s;
                mean_s << mean_error;
                median_s << median_error;
                med_rmse << total_median_rmse;
                cloud_size_s << cloud_size;

                string photo_basename = fs::basename(photoName);
                f << photo_basename;
                f << ";";
                f << mean_s.str();
                f << ";";
                f << median_s.str();
                f << ";";
                f << med_rmse.str();
                f << ";";
                f << cloud_size_s.str();
                f << std::endl;
                f << std::flush;

                /*std::pair<Eigen::Vector3d, Eigen::Vector3d> current_ce = loader->calculateSceneCentroidAndExtent();
                Eigen::Vector3d current_centroid = current_ce.first;
                Eigen::Vector3d current_extent = current_ce.second;*/
                //std::cout << "Scene centroid: " << current_centroid.x() << ", " << current_centroid.y() << ", " << current_centroid.z() << std::endl;

                // save scene center into scene_info.txt file
                boost::filesystem::path scene_info_outpath(output_path);
                scene_info_outpath /= "scene_info.txt";
                std::ofstream scene_info(scene_info_outpath.string());
                scene_info << "center:" << scene_center.x() << " " << scene_center.y() << " " << scene_center.z() << std::endl;
                scene_info.close();

                //save with which photoviews the current photoview share 3D points and a how many
                string photo_tracks = photo_base + "_tracks.txt";
                fs::path photo_tracks_outpath(output_path);
                photo_tracks_outpath /= photo_tracks;

                std::vector< std::tuple<IndexT, std::string, int> > tracks = loader->getViewsSharing3DPointsWithViewId(viewId);
                printTracks(photo_tracks_outpath, tracks);

                renderPhotoviewRealAndFixedIntoDir(viewer, loader, p,
                                                   p_outpath, p_outpath_fixed,
                                                   wcc, viewId, cld, points,
                                                   scene_center, fixed_fov,
                                                   fixed_aspect, fixed_width,
                                                   overlay, dry_run);
            }
        }
        f.close();
        wcc->remove();
    }

    void renderPhotoviewRealAndFixedIntoDir(
            osgViewer::Viewer* viewer,
            shared_ptr<SfMLoader> loader, shared_ptr<AbstractPhotoView> p,
            fs::path p_outpath_real, fs::path p_outpath_fixed,
            osg::ref_ptr<WindowCaptureCallback> wcc,
            IndexT viewId, const Hash_Map<IndexT, Eigen::Vector3d> &cld,
            const vector<osg::Vec3d> &points,
            Eigen::Vector3d centroid,
            double fixed_fov = 60.0, double fixed_aspect = 1.0,
            int fixed_width = 1024, bool overlay = false, bool dry_run = false)
    {
        //save whether photoview is above ground
        string photoName = p->getPhotoName();
        fs::path photoPath(photoName);
        string photo_base = photoPath.stem().string();

        string photo_properties = photo_base + "_properties.txt";
        fs::path photo_properties_outpath(output_path);
        photo_properties_outpath /= photo_properties;

        Vec3d photo_ecef = p->getECEFPosition();
        GeoPoint photo_pos(srs->getGeocentricSRS(), photo_ecef.x(), photo_ecef.y(), photo_ecef.z(), ALTMODE_ABSOLUTE);
        GeoPoint photo_wgs84 = photo_pos.transform(srs);

        fstream f_prop(photo_properties_outpath.string(), f_prop.out);
        f_prop << "isAboveGround ";
        f_prop << p->isAboveGround();
        f_prop << std::endl;
        f_prop << "fov ";
        f_prop << p->getFOVDeg();
        f_prop << std::endl;
        f_prop << "gps ";
        f_prop << photo_wgs84.y();
        f_prop << " ";
        f_prop << photo_wgs84.x();
        f_prop << " ";
        f_prop << photo_wgs84.z();
        f_prop << std::endl;
        f_prop << " ";
        f_prop << p->distanceAboveGround();
        f_prop << std::endl;
        f_prop.close();

        //before render make sure to load terrain
        moveCameraAtPhotoView(p, viewer, true, -1, -1, fixed_width, overlay);

        for (int i = 0; i < 100; ++i)
        {
            //wait a few frames until terrain loading kicks in
            viewer->frame();
        }

        //render real
        moveCameraAtPhotoView(p, viewer, true, -1, -1, fixed_width, overlay);
        renderPhotoViewIntoDir(viewer, loader, p, p_outpath_real, wcc, viewId, cld, points, centroid, dry_run);
        toggleCameraManipulator(viewer);

        //render fixed
        moveCameraAtPhotoView(p, viewer, true, fixed_fov, fixed_aspect, fixed_width, overlay);
        renderPhotoViewIntoDir(viewer, loader, p, p_outpath_fixed, wcc, viewId, cld, points, centroid, dry_run);
        toggleCameraManipulator(viewer);
    }

    osg::ref_ptr<Geode> addPointCloud(Eigen::MatrixX3d pt_3d)
    {
        osg::ref_ptr<Geode> pointCloud = new Geode;
        osg::ref_ptr<Geometry> pointGeom = new Geometry;
        pointCloud->addDrawable(pointGeom);

        osg::ref_ptr<Vec3Array> vert = new Vec3Array;
        osg::ref_ptr<Vec4Array> colors = new osg::Vec4Array;

        osg::Vec4 color(1.0, 0.0, 0.0, 1.0);
        for (int i = 0; i < pt_3d.rows(); ++i)
        {
            Eigen::Vector3d pt = pt_3d.row(i);
            vert->push_back(osg::Vec3(pt[0], pt[1], pt[2]));
            colors->push_back(color);
        }
        pointGeom->setVertexArray(vert);
        pointGeom->setColorArray(colors);
        pointGeom->setColorBinding(Geometry::BIND_PER_VERTEX);
        pointGeom->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::POINTS,
                                                       0, vert->size()));
        pointGeom->getOrCreateStateSet()->setRenderBinDetails(INT_MAX, "RenderBin");
        osgEarth::Registry::shaderGenerator().run(pointCloud);
        mapNode->addChild(pointCloud);
        return pointCloud;
        //osgDB::writeNodeFile(*pointCloud, "/Users/janbrejcha/Downloads/reprojected_image.obj");
    }

    int removeFile(fs::path file_path)
    {
        int res = remove(file_path.string().c_str());
        if (res != 0)
        {
            std::cerr << "Unable to delete: " << file_path.string() << std::endl;
        }
        return res;
    }

    void removeRenderedFiles(string render_basename)
    {
        //remove rendered files, if the view was rejected.
        string photo = render_basename + ".png";
        string photo_transparent = render_basename + "_transparent.png";
        string photo_texture = render_basename + "_texture.png";
        string photo_texture_depth = render_basename + "_texture_depth.txt.gz";
        string photo_modelview = render_basename + "_modelview.txt";
        string photo_projection = render_basename + "_projection.txt";

        fs::path output_path_real(output_path);
        output_path_real /= "real";

        fs::path photo_path = output_path_real / photo;
        fs::path photo_transparent_path = output_path_real / photo_transparent;
        fs::path photo_texture_path = output_path_real / photo_texture;
        fs::path photo_texture_depth_path = output_path_real / photo_texture_depth;
        fs::path photo_modelview_path = output_path_real / photo_modelview;
        fs::path photo_projection_path = output_path_real / photo_projection;

        removeFile(photo_path);
        removeFile(photo_transparent_path);
        removeFile(photo_texture_path);
        removeFile(photo_texture_depth_path);
        removeFile(photo_modelview_path);
        removeFile(photo_projection_path);
    }

    /**
     * @brief randomlySampleAndRenderAdditionalViews
     * Generate novel views looking at roughly the same content as the parent
     * photoview (pv) from different places.
     * @param viewer
     * @param pv parent photoview. Novel photoviews will look roughly at the
     * same content as this photoview.
     * @param p_outpath
     * @param wcc
     * @param scene_center
     * @param count
     * @param stdev_meters standard deviation in meters which will be used to
     * randomly sample the novel position using normal distribution. Mean will
     * be set automatically to the poisiton of the original photoview.
     * @param above_ground how much above ground will be the novel views
     * located.
     * @param tolerance defines how much the 3D point can be behind the
     * depth map during the depth test after projecting 3D points from the
     * original view to the novel view.
     */
    void randomlySampleAndRenderAdditionalViews(osgViewer::Viewer* viewer,
                                                shared_ptr<AbstractPhotoView> pv,
                                                fs::path p_outpath,
                                                osg::ref_ptr<WindowCaptureCallback> wcc,
                                                Eigen::Vector3d scene_center, int count,
                                                int fixed_width,
                                                double stdev_meters = 500,
                                                double above_ground = 10,
                                                double tolerance = 3)
    {
        std::cout << "RandomlySampleAndRenderAdditionalViews" << std::endl;
        ///move the camera at the parent photoview
        moveCameraAtPhotoView(pv, viewer, true);

        ///get the modelview and projection matrices
        osg::Matrixd mv = fp_manip->getInverseMatrix();
        Eigen::Matrix4d mv_eigen = Eigen::Map<Eigen::Matrix4d>(mv.ptr());
        osg::Matrixd proj = viewer->getCamera()->getProjectionMatrix();
        Eigen::Matrix4d proj_eigen = Eigen::Map<Eigen::Matrix4d>(proj.ptr());
        std::cout << "proj eigen: " << proj_eigen << std::endl;

        ///move the modelview matrix to origin
        Eigen::Matrix3d R = mv_eigen.block(0, 0, 3, 3);
        Eigen::Vector3d t = mv_eigen.block(0, 3, 3, 1);
        Eigen::Vector3d pv_center = -R.transpose() * t;
        mv_eigen.block(0, 3, 3, 1) = Eigen::Vector3d(0, 0, 0);

        ///Look for depth map of the parent photoview.
        string photoName = pv->getPhotoName();
        fs::path photoPath(photoName);
        string photo_base = photoPath.stem().string();
        fs::path depth_path(output_path);
        depth_path /= "real";
        depth_path /= photo_base + "_texture_depth.txt.gz";
        Eigen::MatrixXf depth = itr::Util::loadDepthMap(depth_path.string());
        Eigen::Matrix3d intrinsics = itr::Util::projectionToIntrinsics(proj_eigen, depth.cols(), depth.rows());
        std::cout << "depth size: " << depth.cols() << ", " << depth.rows() << std::endl;

        ///unproject 2D coordinates from the parent photoview into the 3D world.
        Eigen::MatrixX3d pt_3d = itr::Util::unproject_image(depth, mv_eigen, intrinsics, 1);
        Eigen::MatrixX3d pt_3d_orig = pt_3d;
        pt_3d.rowwise() += pv_center.transpose();

        //show the points int the map
        //osg::ref_ptr<Geode> point_cloud = addPointCloud(pt_3d);

        int count_rendered = 0;
        std::vector< std::tuple<IndexT, std::string, int> > tracks;
        std::tuple<openMVG::IndexT, std::string, int> track_orig = std::tuple<openMVG::IndexT, std::string, int>(0, photo_base, pt_3d.rows());
        tracks.push_back(track_orig);
        while (count_rendered < count)
        {
            ///randomly sample the novel view candidate
            //get photoview lat lon
            osg::Vec3 pv_pos = pv->getECEFPosition();
            GeoPoint pv_pos_ecef(srs->getGeocentricSRS(), pv_pos.x(), pv_pos.y(), pv_pos.z());
            GeoPoint pv_pos_wgs84 = pv_pos_ecef.transform(srs);
            double lon_rad = (pv_pos_wgs84.x() / 180.0) * M_PI;
            double lat_rad = (pv_pos_wgs84.y() / 180.0) * M_PI;
            double out_lat_rad, out_lon_rad;

            //get lat moved by stdev_meters to the north and calculate difference
            GeoMath::destination(lat_rad, lon_rad, 0, stdev_meters,
                                 out_lat_rad, out_lon_rad);
            double lat_dt = ((out_lat_rad - lat_rad) * 180.0) / M_PI;
            //get lon moved by stdev_meters to the east and calculate difference
            GeoMath::destination(lat_rad, lon_rad, M_PI / 2.0, stdev_meters,
                                 out_lat_rad, out_lon_rad);
            double lon_dt = ((out_lon_rad - lon_rad) * 180.0) / M_PI;
            Eigen::Matrix2d covar;
            covar << lon_dt, 0,
                     0, lat_dt;
            Eigen::Vector2d mean(pv_pos_wgs84.x(), pv_pos_wgs84.y());
            itr::Util::normal_random_variable sample(mean, covar);

            Eigen::Vector2d sampled_pos = sample();
            osg::ref_ptr<ElevationPool> ep = mapNode->getMap()->getElevationPool();
            osg::ref_ptr<ElevationEnvelope> ee = ep->createEnvelope(srs, 16);
            float elev = ee->getElevation(sampled_pos.x(), sampled_pos.y()) + above_ground;
            GeoPoint sampled_gp_wgs84(srs, sampled_pos.x(), sampled_pos.y(),
                                      elev, ALTMODE_ABSOLUTE);
            //GeoPoint sampled_gp_wgs84(srs, 8.050154757529711, 46.42213430007219, 2217.551226114553, ALTMODE_ABSOLUTE);
            //elev = ee->getElevation(8.083600, 46.439913) + above_ground;
            //GeoPoint sampled_gp_wgs84(srs, 8.083600, 46.439913, elev, ALTMODE_ABSOLUTE);


            GeoPoint sampled_gp_ecef = sampled_gp_wgs84.transform(srs->getGeocentricSRS());
            osg::Vec3d sampled_pos_ecef(sampled_gp_ecef.x(), sampled_gp_ecef.y(), sampled_gp_ecef.z());

            //std::cout << "Sampled position: " << sampled_pos.y() << ", " << sampled_pos.x() << ", " << elev << std::endl;
            //std::cout << "Sampled position ECEF" << sampled_pos_ecef.x() << ", " << sampled_pos_ecef.y() << ", " << sampled_pos_ecef.z() << std::endl;
            Eigen::Vector3d pt_centroid = pt_3d.colwise().mean();
            osg::Vec3d pt_centroid_osg(pt_centroid.x(), pt_centroid.y(), pt_centroid.z());

            double fov_2 = pv->getFOVDeg() / 2.0;
            for (int heading = -fov_2; heading <= fov_2; heading += fov_2)
            {
                //move camera to the new sampled position
                osg::Matrix l2w;
                sampled_gp_ecef.createLocalToWorld(l2w);
                osg::Vec3d up = Vec3d(0.0, 0.0, -1.0);
                osg::Vec3d up_world = up * l2w;
                osg::Quat heading_rot((heading * M_PI) / 180.0, up_world);
                osg::Vec3d look_dir = heading_rot * (pt_centroid_osg - sampled_pos_ecef);
                osg::Vec3d look_center = sampled_pos_ecef + look_dir;

                //viewer->getCamera()->setViewMatrixAsLookAt(sampled_pos_ecef, pt_centroid_osg, up_world);
                fp_manip->setTransformation(sampled_pos_ecef, look_center, up_world);
                fp_manip->setHomePosition(sampled_pos_ecef, look_center, up_world);
                fp_manip->setVerticalAxisFixed(false);

                shared_ptr<PhotoView> ppv = std::dynamic_pointer_cast<PhotoView>(pv);
                double aspect = static_cast<double>(depth.cols()) / depth.rows();
                std::cout << "aspect: " << aspect << std::endl;
                if (ppv)
                {
                    fitIntoWindow(ppv, viewer, -1, aspect, fixed_width);
                }

                viewer->setCameraManipulator(fp_manip);
                viewer->getCamera()->setComputeNearFarMode(Camera::ComputeNearFarMode::DO_NOT_COMPUTE_NEAR_FAR);
                viewer->getCamera()->setProjectionMatrixAsPerspective(pv->getFOVYDeg(), aspect, znear, zfar);
                osg::Vec3d random_cam_dir = (pt_centroid_osg - sampled_pos_ecef);
                random_cam_dir.normalize();
                Eigen::Vector3d random_cam_dir_eigen(random_cam_dir.x(), random_cam_dir.y(), random_cam_dir.z());

                //render novel view depth map
                for (int i = 0; i < 10; ++i)
                {
                    viewer->frame();
                }
                TerrainRenderer::waitUntilTerrainLoaded(viewer);

                const Hash_Map<IndexT, Eigen::Vector3d> cld;
                const vector<osg::Vec3d> points;
                shared_ptr<SfMLoader> emptyLoader;
                fs::path real_outpath(output_path);
                real_outpath /= "real";
                std::string lat_str = std::to_string(sampled_pos.y());
                std::string lon_str = std::to_string(sampled_pos.x());
                std::string heading_str = std::to_string(heading + fov_2);
                std::replace(lat_str.begin(), lat_str.end(), '.', '-');
                std::replace(lon_str.begin(), lon_str.end(), '.', '-');
                std::replace(heading_str.begin(), heading_str.end(), '.', '-');

                string render_basename = photo_base + "_random_" + lat_str + "_" + lon_str + "_" + heading_str;
                renderPhotoViewIntoDir(viewer, emptyLoader, pv, real_outpath, wcc, 0, cld, points, scene_center, false, render_basename);

                //load depth map
                fs::path random_depth_path(output_path);
                random_depth_path /= "real";
                random_depth_path /= render_basename + "_texture_depth.txt.gz";
                Eigen::MatrixXf random_depth = itr::Util::loadDepthMap(random_depth_path.string());

                double rdm = random_depth.mean();
                double dm = depth.mean();
                double low = dm - (depth.maxCoeff() / 4.0);
                std::cout << "random_depth_mean " << rdm << ", depth_mean " << dm << ", low: " << low << std::endl;
                if (rdm < low)
                {
                    std::cout << "depth is not big enough, rejecting." << std::endl;
                    //the depth is not big enough, reject the sampled view
                    removeRenderedFiles(render_basename);
                    continue;
                }

                //get intrinsics of the rendered image
                osg::Matrixd proj_random = viewer->getCamera()->getProjectionMatrix();
                Eigen::Matrix4d proj_random_eigen = Eigen::Map<Eigen::Matrix4d>(proj_random.ptr());
                Eigen::Matrix3d intrinsics = itr::Util::projectionToIntrinsics(proj_random_eigen, random_depth.cols(), random_depth.rows());

                //project 3D points observed from the parent view into the novel view
                osg::Matrixd mv_random = fp_manip->getInverseMatrix();
                Eigen::Matrix4d mv_random_eigen = Eigen::Map<Eigen::Matrix4d>(mv_random.ptr());
                //std::cout << "sampled modelview matrix: " << mv_random_eigen << std::endl;
                Eigen::Matrix3d R_random = mv_random_eigen.block(0, 0, 3, 3);
                Eigen::Vector3d t_random = mv_random_eigen.block(0, 3, 3, 1);
                Eigen::Vector3d pv_center_random = -R_random.transpose() * t_random;
                Eigen::Vector3d pv_center_random_o = pv_center_random - pv_center;
                mv_random_eigen.block(0, 3, 3, 1) = Eigen::Vector3d(0, 0, 0);

                //filter 3D points which are in front of the camera
                //std::cout << "Pt3D before: \n" << pt_3d.block(0, 0, 5, 3) << std::endl;
                Eigen::MatrixX3d pt_3d_o = (pt_3d_orig.rowwise() - pv_center_random_o.transpose()).matrix();
                //(pt_3d.rowwise() - pv_center_random.transpose()).matrix();
                std::cout << "pv center random: " << pv_center_random << std::endl;
                std::cout << "Sampled position ECEF" << sampled_pos_ecef.x() << ", " << sampled_pos_ecef.y() << ", " << sampled_pos_ecef.z() << std::endl;

                //std::cout << "Pt3D after: \n" << pt_3d.block(0, 0, 5, 3) << std::endl;
                //std::cout << "Check: " << pt_3d_o.row(0) << " =? " << pt_3d.row(0) - pv_center_random.transpose() << std::endl;
                std::vector<Eigen::Vector3d> pt_3d_o_vec;
                for (int i = 0; i < pt_3d_o.rows(); ++i)
                {
                    if (true || random_cam_dir_eigen.dot(pt_3d_o.row(i)) > 0)
                    {
                        pt_3d_o_vec.push_back(pt_3d_o.row(i));
                    }
                }
                Eigen::MatrixX3d pt_3d_o_front(pt_3d_o_vec.size(), 3);
                for (int i = 0; i < pt_3d_o_vec.size(); ++i)
                {
                    pt_3d_o_front.row(i) = pt_3d_o_vec[i];
                }
                std::cout << "total 3D points: " << pt_3d.rows() << ", 3D point in front of the camera: " << pt_3d_o_front.rows() << std::endl;
                std::cout << "Pt3D_o_front: \n" << pt_3d_o_front.block(0, 0, 5, 3) << std::endl;

                Eigen::MatrixX2d pt_2d_random = itr::Util::project(pt_3d_o_front, mv_random_eigen, intrinsics);
                int total_pt_cnt = pt_3d.rows();

                int count_inside = 0;
                std::vector<Eigen::Vector4d> accepted;

                for (int i = 0; i < pt_2d_random.rows(); ++i)
                {
                    Eigen::Vector2d pt = pt_2d_random.row(i);
                    if (pt.x() >= 0 && pt.x() <= random_depth.cols() &&
                        pt.y() >= 0 && pt.y() <= random_depth.rows())
                    {
                        //point is projected inside the image
                        Eigen::Vector4d p = Eigen::Vector4d::Constant(1);
                        p.head(3) = pt_3d_o_front.row(i);
                        Eigen::Vector4d p_proj = mv_random_eigen * p;
                        p_proj /= p_proj.w();
                        double dist = -p_proj.z();
                        int x = static_cast<int>(pt.x());
                        int y = static_cast<int>(pt.y());
                        //depth test - if projected point is nearer than depth, it is visible
                        if (dist > 0 && random_depth(y, x) < 500000 && random_depth(y, x) > dist - tolerance)
                        {
                            //the point passed depth test, accept it

                            ++count_inside;
                            Eigen::Vector4d ptd(pt.x(), pt.y(), dist, random_depth(y, x));
                            accepted.push_back(ptd);
                        }

                    }
                }
                Eigen::MatrixX4d accepted_mat(accepted.size(), 4);
                for (int i = 0; i < accepted.size(); ++i)
                {
                    accepted_mat.row(i) = accepted[i];
                }

                //count number of visible points, if more than some percentage, accept
                double fraction_inside = static_cast<double>(count_inside) / total_pt_cnt;
                std::cout << "pt_2d_random inside: \n" << count_inside << ", " << fraction_inside << std::endl;

                if (count_inside < 1000)
                {
                    // not enough accepted points, discard the view.
                    removeRenderedFiles(render_basename);
                }
                else
                {
                    Eigen::ArrayXd acc_x = accepted_mat.block(0, 0, accepted_mat.rows(), 1).array();
                    Eigen::ArrayXd acc_y = accepted_mat.block(0, 1, accepted_mat.rows(), 1).array();
                    double x_std = std::sqrt((acc_x - acc_x.mean()).square().sum() / acc_x.size()-1);
                    double y_std = std::sqrt((acc_y - acc_y.mean()).square().sum() / acc_y.size()-1);

                    std::cout << "std x: " << x_std << ", std y: " << y_std << std::endl;

                    if (x_std > 100 && y_std > 30)
                    {
                        //the points are reasonably spread over the image,
                        //accept it
                        fs::path random_point_path(output_path);
                        random_point_path /= "real";
                        random_point_path /= render_basename + "_accepted.txt";
                        std::ofstream file(random_point_path.string());
                        if (file.is_open())
                        {
                            file << accepted_mat << std::endl;
                        }
                        file.close();

                        std::tuple<openMVG::IndexT, std::string, int> track =
                                std::tuple<openMVG::IndexT, std::string, int>(count_rendered + 1, render_basename, count_inside);

                        tracks.push_back(track);
                        ++count_rendered;
                    }
                    else
                    {
                        //the points are located on a small spot, which is useless,
                        //discard it
                        removeRenderedFiles(render_basename);
                    }
                }
            }
        }

        string photo_tracks = photo_base + "_tracks.txt";
        fs::path photo_tracks_outpath(output_path);
        photo_tracks_outpath /= photo_tracks;
        std::sort(tracks.begin(), tracks.end(),
                  [](std::tuple<openMVG::IndexT, std::string, int> &a,
                     std::tuple<openMVG::IndexT, std::string, int> &b)
                    {return std::get<2>(a) > std::get<2>(b);});
        printTracks(photo_tracks_outpath, tracks);

    }

    void setOutputPath(string outpath)
    {
        output_path = outpath;
    }


    void setPickedPhotoViews(vector< shared_ptr<AbstractPhotoView> > pvs)
    {
        picked_photoviews = pvs;
    }
    void setAllPhotoViews(vector< shared_ptr<AbstractPhotoView> > pvs)
    {
        all_photoviews = pvs;
    }


    void setSortPhotoviewsDate(bool sort)
    {
        sortPhotoviewsDate = sort;
    }

    void hideAllPhotoViews()
    {
        for (const shared_ptr<AbstractPhotoView> p : photoviews)
        {
            hidePhotoView(p);
        }
    }

    void fitCamerasToTerrain()
    {
        if (sfmLoaders.size() > 0)
        {
            for (shared_ptr<SfMLoader> l : sfmLoaders)
            {
                l->fitPhotoViewsToTerrain();
            }
        }
        else
        {
            for (const shared_ptr<AbstractPhotoView> p : photoviews)
            {
                p->fitPositionToTerrain();
            }
        }
    }

    void adjustRotationScale()
    {
        for (shared_ptr<SfMLoader> l : sfmLoaders)
        {
            l->adjustRotationScale(mapNode);
        }
    }


    void setupWindowFlythrough(osgViewer::Viewer *viewer, int width = 512, double aspect = 1.0)
    {

        int w = width;
        int h = static_cast<int>(width/aspect);

        if (!useEGL)
        {
            osgViewer::ViewerBase::Windows windows;
            viewer->getWindows(windows);
            int x, y, ww, hw;
            windows[0]->getWindowRectangle(x, y, ww, hw);
            windows[0]->setWindowRectangle(0, 0, w, h);
        }
#ifdef WITH_EGL
        else
        {
            PixelBufferEGL *gc = dynamic_cast<PixelBufferEGL*>(viewer->getCamera()->getGraphicsContext());
            if (gc)
            {
                std::cout << "resizing PixelBufferEGL" << std::endl;
                //gc->makeCurrent();
                gc->resizeImplementation(w, h);
                //gc->releaseContext();
                viewer->getCamera()->setViewport(0, 0, w, h);
            }
        }
#endif
    }


    void flythroughAnimationFromPoint(osgViewer::Viewer* viewer, bool nostops = false, bool saveFrames = false)
    {
        Vector3d centroid(0.0, 0.0, 0.0);
        //we don't need centroid otherwise
        if (saveFrames)
        {
            for (shared_ptr<SfMLoader> loader : sfmLoaders)
            {
                std::pair<Eigen::Vector3d, Eigen::Vector3d> current_ce = loader->calculateSceneCentroidAndExtent();
                Eigen::Vector3d current_centroid = current_ce.first;
                Eigen::Vector3d current_extent = current_ce.second;
                centroid += current_centroid;
            }
            centroid /= sfmLoaders.size();
        }

        if (!viewer->isRealized())
        {
            viewer->realize();
        }

        osg::ref_ptr<WindowCaptureCallback> wcc = new WindowCaptureCallback;
        if (saveFrames)
        {
            wcc->addToViewerMainCamera(*viewer);
            setupWindowFlythrough(viewer);
        }

        if (anim_spline->isPathShown())
        {
            anim_spline->togglePathShown();
        }

        if (cameraManipInUse)
        {
            viewer->setCameraManipulator(0);
            viewer->getCamera()->setComputeNearFarMode(Camera::ComputeNearFarMode::DO_NOT_COMPUTE_NEAR_FAR);
        }
        fp_manip->setVerticalAxisFixed(false);

        osg::Vec3d center = anim_spline->center();
        double fovy, aspect, znear, zfar;
        viewer->getCamera()->getProjectionMatrixAsPerspective(fovy, aspect,
                                                              znear, zfar);
        if (pickedPhotoView)
        {
            pickedPhotoView->loadSmallResolutionPhoto();
            pickedPhotoView->updatePhotoTexture();
            pickedPhotoView = nullptr;
        }

        long frame_idx = 0;
        fs::path frame_outpath(output_path);
        while (!anim_spline->finished())
        {
            clock_t begin = clock();

            osg::Vec3d pos = anim_spline->position();
            center = anim_spline->center();
            osg::Vec3d up = anim_spline->up();
            fovy = saveFrames ? 50 : anim_spline->fovy();
            aspect = saveFrames ? 1.0 : anim_spline->aspect();

            /*std::cout << "pos:" << pos.x() << ", " << pos.y() << ", " << pos.z()
                      << "center: " << center.x() << ", " << center.y() << ", " << center.z()
                      << "up: " << up.x() << ", " << up.y() << ", " << up.z() << std::endl;*/

            viewer->getCamera()->setViewMatrixAsLookAt(pos,
                                                       center,
                                                       up);

            double _znear = saveFrames ? this->znear : 1;
            double _zfar = saveFrames ? this->zfar : 500000;

            viewer->getCamera()->setProjectionMatrixAsPerspective(fovy, aspect,
                                                                  _znear, _zfar);
            if (anim_spline->nextFrame())
            {
                std::cout << "Interrupt, approached to photo." << std::endl;

                shared_ptr<AbstractPhotoView> apv = anim_spline->getControlPoint().getPhotoView();
                shared_ptr<PhotoView> pv = std::dynamic_pointer_cast<PhotoView>(apv);
                if (pv)
                {
                    pickedPhotoView = pv;
                    pv->loadFullResolutionPhoto();
                    pv->updatePhotoTexture();
                }
                if (!nostops)
                {
                    //we want to stop at each photo
                    fp_manip->setTransformation(pos, center, up);
                    fp_manip->setHomePosition(pos, center, up);
                    viewer->setCameraManipulator(fp_manip);
                    return;
                }
            }
            clock_t end = clock();
            double elapsed_secs = double(end - begin) / CLOCKS_PER_SEC;
            viewer->frame(elapsed_secs);

            if (saveFrames)
            {
                char fname[32];
                sprintf(fname, "flythrough_frame_%010d", frame_idx);
                string frame_name = (string)fname;
                fs::path curr_frame_outpath = frame_outpath / (frame_name + ".png");
                fs::path photo_modelview_outpath = frame_outpath / (frame_name + "_pose.txt");
                fs::path photo_projection_outpath = frame_outpath / (frame_name + "_projection.txt");

                const Eigen::Matrix4d modelview = Eigen::Map<Eigen::Matrix4d>(viewer->getCamera()->getViewMatrix().ptr());
                itr::Util::printMatrixToFile(photo_modelview_outpath.string(), modelview, centroid);
                osg::Matrixd proj = viewer->getCamera()->getProjectionMatrix();
                const Eigen::Matrix4d proj_eigen = Eigen::Map<Eigen::Matrix4d>(proj.ptr());
                itr::Util::printMatrixToFile(photo_projection_outpath.string(), proj_eigen);

                if (useEGL)
                {
                    //dirty hack to force the scenegraph to draw into correct buffer
                    wcc->saveOnNextFrame(curr_frame_outpath.string());
                    viewer->frame();
                    wcc->saveOnNextFrame(curr_frame_outpath.string());
                    viewer->frame();
                }
                wcc->setNearFar(this->znear, this->zfar);

                TerrainRenderer::waitUntilTerrainLoaded(viewer);
                wcc->saveOnNextFrame(curr_frame_outpath.string(), true);
                viewer->frame();
                wcc->waitUntilDone();
                ++frame_idx;
            }
            anim_spline->setFramerate(30.0);
        }
        std::cout << "Animation finished." << std::endl;
        //show all photoviews again
        showAllPhotoViews();
        //restore camera manupulator
        viewer->getCamera()->setComputeNearFarMode(Camera::ComputeNearFarMode::COMPUTE_NEAR_FAR_USING_BOUNDING_VOLUMES);
        viewer->setCameraManipulator(earth_manip);

        Viewpoint vp;
        vp.focalPoint()->set(srs->getGeocentricSRS(), center, AltitudeMode::ALTMODE_ABSOLUTE);
        vp.range()->set(400, Units::METERS);
        earth_manip->setHomeViewpoint(vp, 0.0);
    }

    void flythroughAnimation(osgViewer::Viewer* viewer)
    {
        //prepare the animation path
        createAnimationPath();

        double fovy, aspect, znear, zfar;
        viewer->getCamera()->getProjectionMatrixAsPerspective(fovy, aspect,
                                                              znear, zfar);
        anim_spline->setFovy(fovy);
        anim_spline->setAspect(aspect);
        //start the flythrough manually by pressing f, and then p.
        //flythroughAnimationFromPoint(viewer);
    }

    void setExportFlythrough(bool flag)
    {
        exportFlythrough = flag;
    }

    void setExportOnlyPicked(bool flag)
    {
        exportOnlyPicked = flag;
    }


private:
    float _mX, _mY;
    ref_ptr<MapNode> mapNode;
    vector< shared_ptr<AbstractPhotoView> > photoviews;
    ref_ptr<EarthManipulator> earth_manip;
    bool cameraManipInUse;
    vector< shared_ptr<SfMLoader> > sfmLoaders;
    ref_ptr<SpatialReference> srs;
    shared_ptr<AnimationSpline> anim_spline;
    ref_ptr<osgGA::FirstPersonManipulator> fp_manip;
    vector< shared_ptr<AbstractPhotoView> > picked_photoviews;
    vector< shared_ptr<AbstractPhotoView> > all_photoviews;

    // coordinates that will be used to throw away far photoviews.
    double center_lat, center_lon;
    /// Dir containing all the component directories.
    string root_dir;

    /// If true, then picked photoviews are sorted according to the time
    /// and date of creation.
    bool sortPhotoviewsDate;

    shared_ptr<PhotoView> pickedPhotoView;

    //output path to store the rendered images in
    string output_path;

    bool useEGL;

    bool exportFlythrough;
    bool exportOnlyPicked;

    enum PhotoviewsState{
        SHOW_NONE = 0,
        SHOW_ALL = 1,
        SHOW_ALL_USER = 2,
        SHOW_PICKED = 3,
        PHOTOVIEWS_STATE_LAST = 4
    };

    PhotoviewsState pv_state;

    double znear;
    double zfar;


    osg::ref_ptr<osg::Node> pick( const double x, const double y,
               osgViewer::Viewer* viewer )
    {
        osg::ref_ptr<osg::Node> _selectedNode;

        if (!viewer->getSceneData())
            // Nothing to pick.
            return NULL;

        double w( .005 ), h( .005 );
        osgUtil::PolytopeIntersector* picker =
                new osgUtil::PolytopeIntersector(
                    osgUtil::Intersector::PROJECTION,
                    x-w, y-h, x+w, y+h );

        osgUtil::IntersectionVisitor iv(picker);
        viewer->getCamera()->accept(iv);

        if (picker->containsIntersections())
        {
            const osg::NodePath& nodePath =
                    picker->getFirstIntersection().nodePath;
            unsigned int idx = nodePath.size();
            while (idx--)
            {
                // Find the LAST MatrixTransform in the node
                //   path; this will be the MatrixTransform
                //   to attach our callback to.
                PositionAttitudeTransform* mt =
                        dynamic_cast<PositionAttitudeTransform*>(nodePath[idx]);
                if (mt == NULL)
                    continue;

                // If we get here, we just found a position attitude transform.
                _selectedNode = mt;
                if (!_selectedNode.valid())
                {
                    //invalid node, continue
                    continue;
                }
                break;
            }
            if (_selectedNode.valid())
                return _selectedNode;
        }
        return _selectedNode;
    }

    void runICP()
    {
        for (shared_ptr<SfMLoader> l : sfmLoaders)
        {
            l->alignPointCloudICP(mapNode);
            l->saveSfmData("sfm_data_icp.bin");
        }
    }

    void adjustGPS(osgViewer::Viewer* viewer)
    {
        for (shared_ptr<SfMLoader> sfmLoader : sfmLoaders)
        {
            sfmLoader->alignGPS();
        }
    }

    static void toEulerianAngle(const Quat& q, double& roll, double& pitch, double& yaw)
    {
        double ysqr = q.y() * q.y();
        // roll (x-axis rotation)
        double t0 = +2.0 * (q.w() * q.x() + q.y() * q.z());
        double t1 = +1.0 - 2.0 * (q.x() * q.x() + ysqr);
        roll = std::atan2(t0, t1);

        // pitch (y-axis rotation)
        double t2 = +2.0 * (q.w() * q.y() - q.z() * q.x());
        t2 = ((t2 > 1.0) ? 1.0 : t2);
        t2 = ((t2 < -1.0) ? -1.0 : t2);
        pitch = std::asin(t2);

        // yaw (z-axis rotation)
        double t3 = +2.0 * (q.w() * q.z() + q.x() * q.y());
        double t4 = +1.0 - 2.0 * (ysqr + q.z() * q.z());
        yaw = std::atan2(t3, t4);
    }

    void savePickedPhotoviewsStr(string ppPath)
    {
        ofstream out(ppPath);
        vector< shared_ptr<AbstractPhotoView> >::reverse_iterator it;

        fs::path parent_path(root_dir);
        for (it = picked_photoviews.rbegin();
             it != picked_photoviews.rend(); ++it)
        {
            string name = (*it)->getPhotoName();
            fs::path child_path(name);
            string relpath = fs::relative(child_path, parent_path).string();
            out << relpath << std::endl;
        }
        out.close();
    }


    void saveToJson()
    {
        //initialize the animation path
        if (exportFlythrough && picked_photoviews.size() > 4)
        {
            std::cout << "creating animation path" << std::endl;
            createAnimationPath();
        }
        else
        {
            anim_spline = nullptr;
        }


        JsonExporter je(photoviews, mapNode, picked_photoviews, anim_spline, exportOnlyPicked);
        fs::path json_path = fs::path(root_dir) / fs::path("scene.json");
        je.save(json_path.string(), root_dir, center_lat, center_lon);

        fs::path kml_path = fs::path(root_dir) / fs::path("scene.kml");
        je.saveToKML(kml_path.string(), root_dir, center_lat, center_lon);

        fs::path ppath = fs::path(root_dir) / fs::path("picked_photoviews.txt");
        savePickedPhotoviewsStr(ppath.string());

    }

    void toggleCameraManipulator(osgViewer::Viewer* viewer)
    {
        if (cameraManipInUse)
        {
            viewer->setCameraManipulator(fp_manip);
        }
        else
        {
            //load low-res photo
            if (pickedPhotoView)
            {
                pickedPhotoView->loadSmallResolutionPhoto();
                pickedPhotoView->updatePhotoTexture();
            }

            viewer->getCamera()->setComputeNearFarMode(Camera::ComputeNearFarMode::COMPUTE_NEAR_FAR_USING_BOUNDING_VOLUMES);
            viewer->setCameraManipulator(earth_manip);
            showAllPhotoViews();
        }
        cameraManipInUse = !cameraManipInUse;
    }

    void togglePointCloud()
    {
        for (shared_ptr<SfMLoader> l : sfmLoaders)
        {
            l->toggleViewState(mapNode);
        }
    }

    vector< shared_ptr<AbstractPhotoView> > getAllPhotoViews()
    {
        if (all_photoviews.size() == 0)
        {
            return getPickedPhotoViews();
        }
        else
        {
            if (sortPhotoviewsDate)
            {
                std::sort(all_photoviews.begin(), all_photoviews.end(), PhotoViewEarlier());
            }

            //TODO: print out the sorted photoviews and figure out why they
            //are not sorted correctly
            for (shared_ptr<AbstractPhotoView> pv : all_photoviews)
            {
                std::cout << "photoview: " << pv->getPhotoName() << ", create date: " << pv->getCreateDate() << std::endl;
            }
            return all_photoviews;
        }
    }

    vector< shared_ptr<AbstractPhotoView> > getPickedPhotoViews()
    {
        if (picked_photoviews.size() < 4)
        {
            if (sortPhotoviewsDate)
            {
                std::sort(photoviews.begin(), photoviews.end(), PhotoViewEarlier());
            }
            vector< shared_ptr<AbstractPhotoView> >::const_iterator it;
            vector< shared_ptr<AbstractPhotoView> > picked_pvs;
            osg::Vec3d prev = (*photoviews.begin())->getECEFPositionElevated(0);
            picked_pvs.push_back(*photoviews.begin());
            if (photoviews.size() >= 4)
            {
                int idx = 0;
                for (it = photoviews.begin() + 1; it != photoviews.end(); ++it)
                {
                    /*time_t created = (*it)->getCreateDate();
                    char buff[20];
                    strftime(buff, 20, "%Y-%m-%d %H:%M:%S", localtime(&created));
                    std::cout << string(buff) << std::endl;*/

                    osg::Vec3d p = (*it)->getECEFPositionElevated(0);
                    // FIXME: hack to have points at least 100m far from each other.
                    // user will pick the photos himself in the future
                    //double length = (prev - p).length();
                    //if (length >= 100)
                    //{
                        picked_pvs.push_back(*it);
                        prev = p;
                    //}
                    ++idx;
                }
            }
            return picked_pvs;
        }
        else
        {
            if (sortPhotoviewsDate)
            {
                std::sort(picked_photoviews.begin(), picked_photoviews.end(), PhotoViewEarlier());
            }
            return picked_photoviews;
        }
    }




    void createAnimationPath()
    {
        //remove old spline from graph
        if (anim_spline != NULL)
        {
            anim_spline->remove();
            anim_spline.reset();
        }

        //hide all photoviews
        hideAllPhotoViews();

        vector< shared_ptr<AbstractPhotoView> > pvs = getPickedPhotoViews();

        //show picked photoviews
        for (const shared_ptr<AbstractPhotoView> &pv : pvs)
        {
            showPhotoView(pv);
            pv->setOpacity(0);
        }

        vector< shared_ptr<AbstractPhotoView> > all_pvs = getAllPhotoViews();

        //generate the new animation spline
        anim_spline = std::make_shared<AnimationSpline>(pvs, all_pvs, mapNode, srs);

    }

    osg::Quat directionUpToQuat(osg::Vec3d direction, osg::Vec3d up)
    {
        //inspired by https://stackoverflow.com/questions/18558910/direction-vector-to-rotation-matrix
        osg::Vec3d xaxis = up ^ direction;
        xaxis.normalize();
        osg::Vec3d yaxis = direction ^ xaxis;
        yaxis.normalize();
        osg::Matrixd rot(xaxis.x(), xaxis.y(), xaxis.z(), 0,
                          yaxis.x(), yaxis.y(), yaxis.z(), 0,
                          direction.x(), direction.y(), direction.z(), 0,
                          0, 0, 0, 1);

        osg::Matrixd rot_t(xaxis.x(), yaxis.x(), direction.x(), 0,
                          xaxis.y(), yaxis.y(), direction.y(), 0,
                          xaxis.z(), yaxis.z(), direction.z(), 0,
                          0, 0, 0, 1);
        return rot_t.getRotate();
    }


    shared_ptr<AbstractPhotoView> findPhotoViewByTransf(PositionAttitudeTransform *gt)
    {
        for (shared_ptr<AbstractPhotoView> p : photoviews)
        {
            PositionAttitudeTransform *transf = p->getTransf();
            if (transf == gt)
            {
                return p;
            }
        }
        return nullptr;
    }

    void hideOtherPhotoViews(PositionAttitudeTransform *gt)
    {
        for (const shared_ptr<AbstractPhotoView> p : photoviews)
        {
            PositionAttitudeTransform *transf = p->getTransf();
            if (transf != gt)
            {
                //mapNode->removeChild(transf);
                hidePhotoView(p);
            }
        }
    }



    void hidePhotoView(shared_ptr<AbstractPhotoView> p)
    {
        PositionAttitudeTransform *transf = p->getTransf();
        mapNode->removeChild(transf);
        mapNode->removeChild(p->getGeoTransf());
    }

    void showPhotoView(shared_ptr<AbstractPhotoView> p)
    {
        vector< shared_ptr<AbstractPhotoView> >::iterator it =
                std::find(picked_photoviews.begin(), picked_photoviews.end(), p);
        if (it != picked_photoviews.end())
        {
            p->setOpacity(1.0f);
        }
        else
        {
            p->setOpacity(0.5f);
        }
        PositionAttitudeTransform *transf = p->getTransf();
        if (!mapNode->containsNode(transf))
        {
            mapNode->addChild(transf);
            mapNode->addChild(p->getGeoTransf());
        }
    }

    void showAllPhotoViews()
    {
        hideAllPhotoViews();
        for (const shared_ptr<AbstractPhotoView> &p : photoviews)
        {
            showPhotoView(p);
        }
    }

    void showAllUserPhotoViews()
    {
        if (all_photoviews.size() == 0)
        {
            showAllPhotoViews();
        }
        else
        {
            hideAllPhotoViews();
            for (const shared_ptr<AbstractPhotoView> &p : photoviews)
            {
                vector< shared_ptr<AbstractPhotoView> >::iterator it =
                        std::find(all_photoviews.begin(), all_photoviews.end(), p);
                if (it != all_photoviews.end())
                {
                    showPhotoView(p);
                }
            }
        }
    }

    void showAllPickedPhotoViews()
    {
        if (picked_photoviews.size() == 0)
        {
            showAllPhotoViews();
        }
        else
        {
            hideAllPhotoViews();
            for (const shared_ptr<AbstractPhotoView> &p : photoviews)
            {
                vector< shared_ptr<AbstractPhotoView> >::iterator it =
                        std::find(picked_photoviews.begin(), picked_photoviews.end(), p);
                if (it != picked_photoviews.end())
                {
                    showPhotoView(p);
                }
            }
        }
    }


};


string versionString()
{
    return std::to_string(ITR_VERSION_MAJOR) + "." +
            std::to_string(ITR_VERSION_MINOR);
}

void loadPointCloud(MapNode *mapNode, string pointcloud_path)
{
    std::cout << "Loading point cloud: " << pointcloud_path << std::endl;
    osg::Node* pModel = osgDB::readNodeFile(pointcloud_path);

    if(!pModel)
    {
        std::cout << "Error: Couldn't find model!" << std::endl;
    }

    osgEarth::Registry::shaderGenerator().run(pModel);

    PositionAttitudeTransform *pTransf = new PositionAttitudeTransform;
    pTransf->addChild(pModel);

    mapNode->addChild(pTransf);
}

void loadPointCloud(osg::Group *root, string pointcloud_path)
{
    std::cout << "Loading point cloud: " << pointcloud_path << std::endl;
    osg::Node* pModel = osgDB::readNodeFile(pointcloud_path);

    if(!pModel)
    {
        std::cout << "Error: Couldn't find model!" << std::endl;
    }

    osgEarth::Registry::shaderGenerator().run(pModel);

    PositionAttitudeTransform *pTransf = new PositionAttitudeTransform;
    pTransf->addChild(pModel);

    root->addChild(pTransf);
}


vector<string> find_images(const string& path)
{
    vector<string> m_file_list;
    if (!path.empty())
    {
        fs::path apk_path(path);
        fs::recursive_directory_iterator end;

        for (fs::recursive_directory_iterator i(apk_path); i != end; ++i)
        {
            const fs::path cp = (*i);
            string ext = fs::extension(cp);
            boost::algorithm::to_lower(ext);
            if (ext == ".jpg" || ext == ".jpeg" || ext == ".png" ||
                ext == ".tiff" || ext == ".tif")
            {
                m_file_list.push_back(cp.string());
            }
        }
    }
    return m_file_list;
}

vector<string> find_point_cloud(const string& path)
{
    vector<string> m_file_list;
    if (!path.empty())
    {
        fs::path apk_path(path);
        fs::recursive_directory_iterator end;

        for (fs::recursive_directory_iterator i(apk_path); i != end; ++i)
        {
            const fs::path cp = (*i);
            string ext = fs::extension(cp);
            string basename = fs::basename(cp);
            boost::algorithm::to_lower(ext);
            boost::algorithm::to_lower(basename);
            if (basename == "colorized" && ext == ".ply")
            {
                m_file_list.push_back(cp.string());
            }
        }
    }
    return m_file_list;
}


vector<string> find_component_dirs(const string& path)
{
    vector<string> m_file_list;
    if (!path.empty())
    {
        try
        {
            boost::regex pattern("^[0-9]+$");
            fs::path apk_path(path);
            fs::directory_iterator end;

            for (fs::directory_iterator i(apk_path); i != end; ++i)
            {
                const fs::path cp = (*i);
                string ext = fs::extension(cp);
                string basename = fs::basename(cp);
                boost::algorithm::to_lower(ext);
                boost::algorithm::to_lower(basename);

                if (boost::regex_match(basename, pattern))
                {
                    m_file_list.push_back(cp.string());
                }
            }

        }
        catch (boost::regex_error &re)
        {
            std::cerr << "regex error: " << re.what() << ", code: " << re.code() << std::endl;
        }
    }
    return m_file_list;
}


vector<string> loadPickedPhotoviewsStr(string ppPath)
{
    vector<string> pickedPhotoviews;
    ifstream in(ppPath);
    std::string str;
    while (std::getline(in, str))
    {
        fs::path p(str);
        string pname = fs::basename(p) + fs::extension(p);
        if (pname.length() > 0)
        {
            //we need to insert at the beginning since the animation
            //path starts at the last element.
            std::cout << "picked photoview: " << pname << std::endl;
            pickedPhotoviews.insert(pickedPhotoviews.begin(), pname);
        }
    }
    return pickedPhotoviews;
}

vector< shared_ptr<AbstractPhotoView> >
loadPickedPhotoviews(vector<string> pickedPhotoviewsStr,
                     vector< shared_ptr<AbstractPhotoView> > &photoviews,
                     MapNode *mapNode,
                     double center_lat = 360, //nonsense values will be ignored
                     double center_lon = 360)
{
    vector< shared_ptr<AbstractPhotoView> > pickedPvs;
    //vector< shared_ptr<AbstractPhotoView> >::iterator it;
    //vector<string>::iterator sit;

    SpatialReference *srs = SpatialReference::get("epsg:4326");
    GeoPoint our_center(srs, center_lon, center_lat, 1.8, ALTMODE_RELATIVE);
    our_center.transformZ(ALTMODE_ABSOLUTE,
                          mapNode->getTerrainEngine()->getTerrain());
    GeoPoint our_center_ecef;
    our_center.transform(srs->getGeocentricSRS(), our_center_ecef);

    osg::Vec3d our_center_xyz(our_center_ecef.x(),
                              our_center_ecef.y(),
                              our_center_ecef.z());

    for (const string &pn : pickedPhotoviewsStr)
    {
        bool found = false;
        for (shared_ptr<AbstractPhotoView> pv : photoviews)
        {
            fs::path pp(pv->getPhotoName());
            string name = fs::basename(pp);
            string pns = fs::basename(fs::path(pn));

            //check distance from scene center
            if (center_lat >= -90 && center_lat <= 90 &&
                center_lon >= -180 && center_lon <= 180)
            {
                Vec3d center_xyz = pv->getECEFPosition();
                double dist = (center_xyz - our_center_xyz).length();
                if (dist > 40000)
                {
                    std::cout << "Not using picked photoview: " << name << " due too big distance from scene center. " << center_xyz.x() << ", " << center_xyz.y() << ", " << center_xyz.z() <<  std::endl;
                    continue;
                }
            }

            std::cout << "comparing: " << name << ", " << pns << std::endl;
            if (name == pns)
            {
                found = true;
                pickedPvs.push_back(pv);
                break;
            }
        }
        if (!found)
        {
            std::cerr << "Unable to find picked photoview: " << pn << std::endl;
        }
    }
    //remove not picked photoviews from scene graph
    /*for (it = photoviews.begin(); it != photoviews.end(); ++it)
    {
        fs::path pp((*it)->getPhotoName());
        string name = fs::basename(pp) + fs::extension(pp);
        sit = std::find(pickedPhotoviewsStr.begin(), pickedPhotoviewsStr.end(), name);
        if (sit == pickedPhotoviewsStr.end())
        {
            //remove this photoview from graph since it is not picked
            std::cerr << "Removing photoview: " << name << " since it is not picked." << std::endl;
            kh->hidePhotoView(*it);
        }
    }*/
    return pickedPvs;
}

int usage(const char* name, ArgumentParser &arguments)
{
    OE_NOTICE
        << "\nUsage: " << name << " <file>.earth" << std::endl
        << std::endl
        << "MANDATORY PARAMETERS" << std::endl
        << "====================" << std::endl << std::endl
        << "<file>.earth      : The definition file for OSGEarth to \n\
        define data layers to be loaded (terrain, textures, etc.). To use default \n\
        earth file, use `example.earth` located in the root of this \n\
        repository. This argument is supposed to be *ALWAYS* the last argument." << std::endl
        << std::endl
        << "OPTIONAL PARAMETERS" << std::endl
        << "====================" << std::endl << std::endl
        << "  --directory <components>          : The directory containing SfM \n\
        reconstructed components or exported renderings with metadata using \n\
        --render-photoviews option. The components directory is supposed to contain\n\
        numbered subdirectories with individual SfM components. To reconstruct the \n\
        scene, you may use OpenMVG, or COLMAP. Each subdirectory is supposed to \n\
        contain all component images as well as the file containing scene data. \n\
        In case of OpenMVG reconstruction, this file will be sfm_data.bin, in case \n\
        of COLMAP use sfm_data.nvm."
        << "  --all-photoviews <txt_file>       : A file containing list of \n\
        photographs which will be visible in the viewer. From this set of \n\
        photographs the user can pick the photographs for the presentation. \n\
        Typically the list will contain all user-taken photographs. \n\
        The list must contain the path relative to the image, e.g., if the \n\
        program is run with --directory mydir, and mydir contains \n\
        subdirectories 0/ and 1/, then the file list would contain:\n\
            0/img1.jpg \n\
            0/img2.jpg \n\
            ...        \n\
            1/imgN.jpg" << std::endl << std::endl
        << "  --picked-photoviews <txt_file>    : A file containing list of \n\
        photoviews at which the fly-through will stop. This is especially \n\
        usable when you create a version of a fly-through which you want to \n\
        resume later." << std::endl << std::endl
        << "  --output <output dir>             : An output path to store \n\
        rendered images (using option --render-photoviews and \n\
        --render-flythrough)"
        << std::endl << std::endl
        << "  --sort-time                       : If this flag is set in \n\
        combination with --all-photoviews, the photoviews will be sorted \n\
        according to the time of creation found in exif. The sort WILL NOT \n\
        occur, if the date of creation cannot be find for all photographs \n\
        specified by the --all-photoviews flag." << std::endl << std::endl
        << "  --render-photoviews <image_width> : Set this flag to render the \n\
        dataset of synthetic and corresponding real images in batch mode.\n\
           " << std::endl << std::endl
        << "  --render-flythrough               : Renders the flythrough \n\
        in non-interactive session into directory defined with --output."
        << std::endl << std::endl
        << "  --no-export-flythrough            : Do not export flythrough \n\
        into KML file and json file for VR."
        << std::endl << std::endl
        << "  --export-picked-only              : Export only views defined \n\
        with --picked-photoviews into KML file or json for VR."
        << std::endl << std::endl
        << "  --egl <GPU index>                 : When combined with \n\
        --render-photoviews and when compiled on Linux machine with support of \n\
        EGL (NVidia only), this will enable offscreen rendering \n\
        (mainly for usage on cluster). The support for EGL is detected \n\
        automatically by CMake - if this flag is not available, it means \n\
        that CMake did not detect the libraries needed for EGL."
        << std::endl << std::endl
        << "  --window <x> <y> <width> <height>   : Sets the position and size\n\
        of the window." << std::endl << std::endl
        << "  --scene-center <lat> <lon>          : Set scene center to the specified coordinates." << std::endl << std::endl
        << "  --export-photoparams <output dir> : Must be combined with --directory option. \n\
         Exports the camera poses into photoparam directories for project Locate. \n\
         The output photoparams will be saved into the location specified by \n\
         the --output flag, each photoparam will be saved \n\
         to a separate directory."  << std::endl << std::endl
        << "  --render-grid <center-lat> <center-lon> <grid-width> \n\
        <grid-height> <offset-x> <offset-y> <resolution> <outpath> \n\
                                   : Renders panoramas placed on a regular \n\
        lattice. <center-lat>, <center-lon> are coordinates of the lattice \n\
        center, <grid-width>, <grid-height> defines the size of the lattice \n\
        int meters, <offset-x>, <offset-y> defines the distance in meters \n\
        between consecutive samples in x and y-direction, respectively. \n\
        <resolution> defines the resolution of the a single rendered view."
        << "  --render-grid-mode <mode>     : Defines the camera mode. \n\
        Valid options are: \"equirectangular\" (default) \n\
        and \"perspective\". When \"equirectangular\" is used, the rendered \n\
        output is projected as an equirectangular panorama (full 360x180 deg).\n\
        When \"perspective\" is used, the output is panorama sampled from \n\
        neighboring perspective images rotated by certain angle. The angle \n\
        is derived from the --render-grid-fov (field of view of each view) \n\
        and --render-grid-num-views (number of views looking around each \n\
        position." << std::endl << std::endl
        << "--render-grid-fov <fov> : Field-of-view which will be used for \n\
        rendering views on rectangular lattice. Default = 60 degrees."
        << std::endl << std::endl
        << "--render-grid-num-views <num-views> : Number of views \n\
        which will be rendered from each lattie position (rotated to form a \n\
        panorama. Default = 12."
        << std::endl << std::endl
        << "--pose-from-gps <lat> <lon> <output_prefix> : Moves the camera \n\
        to the specified position, queries the terrain for elevation, and \n\
        stores corresponding modelview and projection matrices into \n\
        <output_prefix>_modelview.txt and <output_prefix>_projection.txt files."
        << std::endl << std::endl
        << "--single-photoparam <lat> <lon> <fov> <rotation_matrix_xml_file> \n\
        <image_path>                        : Loads the single photoparam \n\
        from project Locate. Do not combine with --directory."
        << std::endl << std::endl
        << "--render-random <count> <stdev_meters> <above_ground_meters> \n\
                                            : May be combined only with \n\
        --single-photoparam option. Randomly samples novel views looking \n\
        on the same scene as the original photoparam. <stdev_meters> defines \n\
        the standard deviation of the normal distribution used to randomly \n\
        sample the positions of the novel views, <count> defines how many \n\
        views will be generated and <above_ground_meters> defines how much \n\
        above ground will be the novel views placed."
        << std::endl << std::endl
        << MapNodeHelper().usage() << std::endl;

    return 0;
}


class NodeRefCullCallback : public osg::NodeCallback
{
public:
    std::set<osg::ref_ptr<osg::Node>> nodeset;
    int nodeCount;
    static NodeRefCullCallback& getInstance()
    {
        static NodeRefCullCallback    instance; // Guaranteed to be destroyed.
                              // Instantiated on first use.
        return instance;
    }
private:
   NodeRefCullCallback()
   {
        nodeCount = 0;// Constructor? (the {} brackets) are needed here.
   }

    // C++ 11
    // =======
    // We can use the better technique of deleting the methods
    // we don't want.
public:
    NodeRefCullCallback(NodeRefCullCallback const&)               = delete;
    void operator=(NodeRefCullCallback const&)  = delete;

    // Note: Scott Meyers mentions in his Effective Modern
    //       C++ book, that deleted functions should generally
    //       be public as it results in better error messages
    //       due to the compilers behavior to check accessibility
    //       before deleted status

    virtual void operator()(osg::Node* node, osg::NodeVisitor* nv)
    {
        std::cout << "parent count: " << node->getParents().size() << ", ref count: " << node->referenceCount() << std::endl;
    }
};


class BFSNodeVisitor: public osg::NodeVisitor
{
public:

    std::queue<osg::ref_ptr<osg::Node>> _pendingNodesToVisit;
    int nodeCount;
    int nodeCount2;
    int numGroups;
    int numBareGroups;
    int bareNodeCount;
    int groupsNoChildren;
    int groupsChildGroup;

    BFSNodeVisitor() : osg::NodeVisitor( osg::NodeVisitor::TRAVERSE_ALL_CHILDREN )
    {
        nodeCount = 0;
        nodeCount2 = 0;
        numGroups = 0;
        bareNodeCount = 0;
        numBareGroups = 0;
        groupsNoChildren = 0;
        groupsChildGroup = 0;
    }

    virtual void apply ( osg::Node& node )
    {
        ++bareNodeCount;
        if (node.getName().length() > 0)
        {
            //std::cout << "Processing " << node.getName() << ", ref count: " << node.referenceCount() << std::endl;
            ++nodeCount;
            if (node.referenceCount() > 1)
            {
                ++nodeCount2;
            }
        }
    }

    virtual void apply ( osg::Group& group )
    {
        ++numBareGroups;
        if (group.getNumChildren() == 0)
        {
            ++groupsNoChildren;
        }
        if (group.getNumChildren() == 1)
        {
            osg::Node *node = group.getChild(0);
            if (node->asGroup())
            {
                ++groupsChildGroup;
            }
        }
        if (group.getName().length() > 0)
        {
            ++numGroups;
            //std::cout << "Processing group " << group.getName() << std::endl;
        }
        for(int i = 0; i < group.getNumChildren(); i++)
        {
            _pendingNodesToVisit.push(group.getChild(i));
        }
        while(_pendingNodesToVisit.size() != 0)
        {
            osg::Node* node = _pendingNodesToVisit.front();
            _pendingNodesToVisit.pop();
            node->accept(*this);
        }
    }
};


class CullCallback : public osg::NodeCallback
{
        virtual void operator()(osg::Node* node, osg::NodeVisitor* nv)
        {
            NodeRefCullCallback::getInstance().nodeCount = 0;
            MapNode *mapNode = (MapNode *)node;
            BFSNodeVisitor bnv;
            node->accept(bnv);
            std::cout << "node count: " << bnv.nodeCount << ", nodeCount2: " << bnv.nodeCount2
                      << ", numGroups:  " << bnv.numGroups << ", numBareGroups: " << bnv.numBareGroups
                      << ", groupsNoChildren: " << bnv.groupsNoChildren << ", groupsOneChildGroup: " << bnv.groupsChildGroup
                      <<  ", bareNodeCount: " << bnv.bareNodeCount << ", " << std::endl;

            //std::cout<<"cull callback - pre traverse: "<<node->className()<<std::endl;
            traverse(node,nv);
        }
};


void loadPhotoviewsFromExportedDir(string input_dir, vector< shared_ptr<AbstractPhotoView> > &photoviews, MapNode *mapNode, Viewer &viewer)
{
    // export rendered photoviews to a photoparam representation
    // for Locate project.
    fs::path input_path(input_dir);
    fs::path sc_name("scene_info.txt");
    fs::path sc_path = input_path / fs::path("..") / sc_name;
    Eigen::Vector3d scene_center = itr::Util::loadSceneCenter(
                                        sc_path.string()
                                   );
    //std::cout << "Loaded scene center: " << scene_center << std::endl;
    // find all file bases to be export
    fs::directory_iterator end;
    vector<string> basenames;
    for (fs::directory_iterator dit(input_path); dit != end; ++dit)
    {
        const fs::path cp = (*dit);
        const string cfname = cp.filename().string();
        std::string::size_type n;
        n = cfname.find("_modelview.txt");
        if (n != std::string::npos)
        {
            string base = cfname.substr(0, n);
            basenames.push_back(base);
        }
    }

    // export each file
    osg::Matrixd orig_proj = viewer.getCamera()->getProjectionMatrix();

    for (string &base : basenames)
    {
        fs::path img_path = input_path / fs::path(base + ".png");
        if (!fs::exists(img_path))
        {
            img_path = input_path / fs::path(base + ".jpg");
            if (!fs::exists(img_path))
            {
                throw std::runtime_error(
                    "Unable to find image to export. Image must be jpg or \
                    png. Assumed image path: " + img_path.string()
                );
            }
        }
        fs::path mv_path = input_path / fs::path(base + "_modelview.txt");
        if (!fs::exists(mv_path))
        {
            throw std::runtime_error(
                "Unable to find image modelview at path: "
                + mv_path.string()
            );
        }
        fs::path pr_path = input_path / fs::path(base + "_projection.txt");
        if (!fs::exists(pr_path))
        {
            throw std::runtime_error(
                "Unable to find image projection at path: "
                + pr_path.string()
            );
        }

        Eigen::Matrix4d mv = itr::Util::loadMatrix4dFromFile(
                                mv_path.string(), scene_center
                             );
        Eigen::Matrix4d proj = itr::Util::loadMatrix4dFromFile(
                                pr_path.string()
                               );


        shared_ptr<PhotoView> pv = make_shared<PhotoView>(mapNode, img_path.string());

        osg::Matrixd proj_osg(proj.data());
        viewer.getCamera()->setProjectionMatrix(proj_osg);
        double fovy, aspect, znear, zfar;
        viewer.getCamera()->getProjectionMatrixAsPerspective(fovy, aspect, znear, zfar);

        Eigen::Matrix4d identity = Eigen::Matrix4d::Identity();
        identity(1, 1) = -1;
        identity(2, 2) = -1;
        mv = identity * mv;
        Eigen::Matrix3d rot = mv.block(0, 0, 3, 3).transpose();
        Eigen::Quaternion<double> rotq(rot);
        osg::Quat quat(rotq.x(), rotq.y(), rotq.z(), rotq.w());

        Eigen::Vector3d t = -rot * mv.block(0, 3, 3, 1);
        pv->setECEFPosition(t.x(), t.y(), t.z());
        pv->setOrientation(quat);
        pv->setFOVYDeg(fovy);
        photoviews.push_back(pv);
    }
    viewer.getCamera()->setProjectionMatrix(orig_proj);
}


int main(int argc, char *argv[])
{
    bool use_egl = false;

    //setup OpenSceneGraph
    osg::ArgumentParser arguments(&argc,argv);
    Viewer viewer; //(arguments)

#ifdef WITH_EGL
    int gpu_idx;
    if (arguments.read("--egl", gpu_idx))
    {
        std::cout << "GPU index main " << gpu_idx << std::endl;
        use_egl = true;
    }
#endif
    string dirname;
    // help?
    if (arguments.read("--help"))
    {
        usage(argv[0], arguments);
    }
    if (arguments.read("--version"))
    {
        cout << "ITR version: " << versionString() << std::endl;
        exit(0);
    }

    float vfov = -1.0f;
    arguments.read("--vfov", vfov);


    viewer.setThreadingModel(ViewerBase::ThreadingModel::SingleThreaded);
    std::cout << "threading model: " << viewer.getThreadingModel() << std::endl;
    //std::cout << "Created viewer." << std::endl;

#ifdef WITH_EGL
    if (use_egl)
    {
        GraphicsContext::Traits *egl_traits = new GraphicsContext::Traits;
        std::cout << "With EGL!" << std::endl;
        egl_traits->width = 1024;
        egl_traits->height = 1024;
        egl_traits->pbuffer = true;

        PixelBufferEGL *eglContext = new PixelBufferEGL(egl_traits, gpu_idx);
        std::cout << "trying to realize EGL context directly" << std::endl;
        eglContext->realize();
        //eglContext->makeCurrent();
        std::cout << "EGL context realized directly" << std::endl;
        viewer.getCamera()->setGraphicsContext(eglContext);
        viewer.getCamera()->setViewport(0, 0, egl_traits->width, egl_traits->height);
        std::cout << "trying to realize viewer" << std::endl;
        viewer.realize();
        std::cout << "viewer realized" << std::endl;

        //set osgearth capabilities to reflect our custom context
        EGLCapabilities *eglCap = new EGLCapabilities(eglContext);
        //eglContext->releaseContext();
        osgEarth::Registry::instance()->setCapabilities(eglCap);
    }
#endif
    if (!use_egl)
    {
		osg::ref_ptr<GraphicsContext::Traits> traits = new GraphicsContext::Traits;
		traits->x = 0;
		traits->y = 0;
        traits->width = 1200;
        traits->height = 800;
		traits->windowDecoration = true; // window border etc.
		traits->doubleBuffer = true;
		traits->sharedContext = 0;
		traits->vsync = false;
		traits->windowName = "ITR - Immersive Trip Reports";
		traits->readDISPLAY();
		osg::ref_ptr<GraphicsContext> graphicsContext = GraphicsContext::createGraphicsContext(traits.get());
		viewer.getCamera()->setGraphicsContext(graphicsContext);
		viewer.getCamera()->setViewport(0, 0, traits->width, traits->height);
        viewer.getCamera()->setProjectionMatrixAsPerspective(60, (float)traits->width/(float)traits->height,
                                                             1, 900000);

        /*viewer.apply(new osgViewer::SingleWindow);
        osgViewer::ViewerBase::Windows windows;
        viewer.getWindows(windows);
        if (windows.size() > 0)
        {
            std::cout << "Setting window name to ITR" << std::endl;
            windows[0]->setWindowName("ITR");
        }*/
    }

    viewer.getCamera()->setClearColor(osg::Vec4(0.0, 0.0, 0.0, 1.0));

    //FIXME: WHAT IS THIS? SEEMS TO CAUSE TROUBLE WITH FILLING UP THE RAM
    // Tell the database pager to not modify the unref settings
    //viewer.getDatabasePager()->setUnrefImageDataAfterApplyPolicy( true, false );


    //FIXME: BE EXTREMLY CAUTIOUS ABOUT HOW MANY PAGED LODs CAN BE STORED IN MEMORY!
    //TO LOW NUMBER CAUSES SLOW TERRAIN RELOADIG, TO HIGH NUMBER CAUSES MACHINE SWAPPING!!!
    //std::cout << "target max num of page lod before setup: " << viewer.getDatabasePager()->getTargetMaximumNumberOfPageLOD() << std::endl;
    //viewer.getDatabasePager()->setTargetMaximumNumberOfPageLOD(1000);
    //std::cout << "target max num of page lod after setup: " << viewer.getDatabasePager()->getTargetMaximumNumberOfPageLOD() << std::endl;

    // thread-safe initialization of the OSG wrapper manager. Calling this here
    // prevents the "unsupported wrapper" messages from OSG
    osgDB::Registry::instance()->getObjectWrapperManager()->findWrapper("osg::Image");

    // install our default manipulator (do this before calling load)
    EarthManipulator *earth_manip = new EarthManipulator(arguments);
        earth_manip->getSettings()->setMinMaxPitch(-90, 90);
        viewer.setCameraManipulator(earth_manip);
    viewer.setCameraManipulator( earth_manip );

    // disable the small-feature culling
    viewer.getCamera()->setSmallFeatureCullingPixelSize(-1.0f);

    // set a near/far ratio that is smaller than the default. This allows us to get
    // closer to the ground without near clipping. If you need more, use --logdepth
    viewer.getCamera()->setNearFarRatio(0.00001);

    if ( vfov > 0.0 )
    {
        double fov, ar, n, f;
        viewer.getCamera()->getProjectionMatrixAsPerspective(fov, ar, n, f);
        viewer.getCamera()->setProjectionMatrixAsPerspective(vfov, ar, n, f);
    }


    // load an earth file, and support all or our example command-line options
    // and earth file <external> tags
    //osg::ref_ptr<osg::Node> node = new osg::Node;

    //osg::Node* node = osgDB::readNodeFiles(arguments);

    osg::Node* node = MapNodeHelper().load(arguments, &viewer);
    if ( node )
    {
        viewer.setSceneData( node );
        //Metrics::run(viewer);
    }
    else
    {
        return usage(argv[0], arguments);
    }/**/

#ifdef WITH_EGL
    if (use_egl)
    {
        std::cout << "Disabling realize operation" << std::endl;
        //unload realize operation possibly set by MapNodeHelper().load()
        viewer.setRealizeOperation(NULL);
    }
#endif

    MapNode *mapNode = MapNode::get(node);
    if (arguments.read("--numberOfViews", dirname))
    {
        itr::SfMLoader sfmLoader(dirname, mapNode);
        std::cout << sfmLoader.getViewsCount() << std::endl;
        exit(0);
    }
    // need to set up in earth file
    //MapNodeOptions &mno = mapNode->getMapNodeOptions();
    //TerrainOptions &to = mno.getTerrainOptions();
    //to.maxLOD() = 16;
    //to.minTileRangeFactor() = 45;
    //to.clusterCulling() = false;
    osg::StateSet* ss = mapNode->getOrCreateStateSet();
    ss->setAttributeAndModes(
                new osg::CullFace(),
                osg::StateAttribute::OFF | osg::StateAttribute::OVERRIDE );
    ss->setMode( GL_CULL_FACE, osg::StateAttribute::OFF);

    //will be ignored for nonsense values
    float center_lat = 360;
    float center_lon = 360;

    SpatialReference *srs = SpatialReference::get("epsg:4326");
    Eigen::Vector3d scene_center(0.0, 0.0, 0.0);
    if (arguments.read("--scene-center", center_lat, center_lon))
    {
        std::cout << "Using center lat: " << center_lat << ", " << center_lon << std::endl;
        osg::ref_ptr<ElevationPool> ep = mapNode->getMap()->getElevationPool();
        osg::ref_ptr<ElevationEnvelope> ee = ep->createEnvelope(srs, 16);

        float elev = ee->getElevation(center_lon, center_lat);
        GeoPoint fit_pos(srs, center_lon, center_lat, static_cast<double>(elev), ALTMODE_ABSOLUTE);
        GeoPoint fitted_xyz = fit_pos.transform(srs->getGeocentricSRS());
        scene_center = Eigen::Vector3d(fitted_xyz.x(), fitted_xyz.y(), fitted_xyz.z());
        std::cout << "Using scene center: " << scene_center << std::endl;
    }

    vector< shared_ptr<AbstractPhotoView> > photoviews;

    vector<string> pickedPhotoviewsStr;
    string ppPath;
    if (arguments.read("--picked-photoviews", ppPath))
    {
        cout << "Reading picked photoviews: " << ppPath << std::endl;
        pickedPhotoviewsStr = loadPickedPhotoviewsStr(ppPath);
    }
    vector<string> allPhotoviewsStr;
    string apPath;
    if (arguments.read("--all-photoviews", apPath))
    {
        //all photoviews are used for generating more precise flythrough spline
        //picked photoviews (above) are the views where the animation stops.
        cout << "Reading all photoviews: " << apPath << std::endl;
        allPhotoviewsStr = loadPickedPhotoviewsStr(apPath);
    }

    bool dry_run = false;
    if (arguments.read("--dry-run"))
    {
        dry_run = true;
    }

    if (arguments.read("--directory", dirname))
    {
        cout << "Reading whole directory " << dirname << std::endl;

        vector<string> component_dirs = find_component_dirs(dirname);
        if (component_dirs.size() == 0)
        {
            //if no components are found, we are probably inside a component
            //and so we try to run the program on this component directly.
            component_dirs.push_back(dirname);
        }
        vector< ref_ptr<osg::Node> > pointClouds;
        vector< shared_ptr<itr::SfMLoader> > sfmLoaders;

        string output_photoparam_dir;

        for (string &component_dir : component_dirs)
        {
            std::cout << "Loading component: " << component_dir << std::endl;
            //SfM data
            try
            {
                shared_ptr<itr::SfMLoader> sfmLoader(new SfMLoader(component_dir, mapNode, center_lat, center_lon));
                //first load point cloud, it will be automatically aligned using ICP
                set< shared_ptr<AbstractPhotoView> > component_photoviews = sfmLoader->loadPhotoparams(mapNode);
                ref_ptr<osg::Node> pointCloud = sfmLoader->loadPointCloud(mapNode);
                sfmLoader->updateViewState(mapNode);
                pointClouds.push_back(pointCloud);
                /*photoviews.insert(component_photoviews.begin(),
                                  component_photoviews.end());*/
                std::copy(component_photoviews.begin(), component_photoviews.end(),
                          std::back_inserter(photoviews));
                sfmLoaders.push_back(sfmLoader);
            }
            catch (std::runtime_error &re)
            {
                std::cerr << "Skipping component: " << component_dir << ", due to ERROR: " << re.what() << std::endl;
            }
            //break;
        }

        string posenet_outpath;
        if (arguments.read("--print-posenet-poses", posenet_outpath))
        {
            for (shared_ptr<itr::SfMLoader> loader : sfmLoaders)
            {
                loader->printPoseNetPoses(posenet_outpath);
            }
            return 0;
        }

        if (photoviews.size() == 0)
        {
            // if loading sfm scene failed to load any photoviews,
            // try to load exported photoviews
            try{
                loadPhotoviewsFromExportedDir(dirname, photoviews,
                                              mapNode, viewer);
            }
            catch(runtime_error &re)
            {
                std::cerr << "Unable to load exported photoviews. "
                          << re.what() << std::endl;
            }
        }

        /*Viewpoint vp;
        vp.setNode(photoviews[0]->getGeode());

        earth_manip->setViewpoint(vp);
        earth_manip->home(5);*/


        //other viewer settings
        //viewer.getCamera()->setCullingMode(osg::CullSettings::ENABLE_ALL_CULLING);

        //CullCallback *cc = new CullCallback;
        //mapNode->setCullCallback(cc);

        osgDB::ReaderWriter::Options *opts = new osgDB::ReaderWriter::Options;
        opts->setObjectCacheHint(osgDB::ReaderWriter::Options::CACHE_NONE);
        osgDB::Registry::instance()->setOptions(opts);

        double fovy, aspect, znear, zfar;
        viewer.getCamera()->getProjectionMatrixAsPerspective(fovy, aspect,
                                                              znear, zfar);
        viewer.getCamera()->setProjectionMatrixAsPerspective(60, aspect,
                                                              5, 900000);

        //LODBlending* effect = new LODBlending();
        //mapNode->getTerrainEngine()->addEffect( effect );

#ifndef WITH_EGL
    std::cout << "Enabling logarithmic depthbuffer" << std::endl;
    //LogarithmicDepthBuffer logdepth;
    //logdepth.install( viewer.getCamera() );
#endif

        ref_ptr<AdobeViewerKeyboardEventHandler> kh =
                new AdobeViewerKeyboardEventHandler(mapNode, earth_manip,
                                                    photoviews, sfmLoaders,
                                                    dirname,
                                                    center_lat,
                                                    center_lon, false, use_egl);

        string outpath = "";
        if (arguments.read("--output", outpath))
        {
            kh->setOutputPath(outpath);
        }

        if (arguments.read("--export-photoparams"))
        {
            if (outpath == "")
            {
                throw std::runtime_error("Cannot export photoparams because "
                                         "output directory is not set. Please, "
                                         "set output directory by --output "
                                         "option.");
            }
            for (shared_ptr<AbstractPhotoView> apv : photoviews)
            {
                Vec3d pos = apv->getECEFPosition();
                GeoPoint gp(srs->getGeocentricSRS(), pos.x(), pos.y(), pos.z(), ALTMODE_ABSOLUTE);
                GeoPoint gp_wgs = gp.transform(srs);
                osg::Quat orientation = apv->getOrientationLocal();
                osg::Matrixd pv_rot;
                orientation.get(pv_rot);

                Quat init_orient(M_PI, osg::Vec3(1, 0, 0),  //pitch
                                 0, osg::Vec3(0, 1, 0),     //roll
                                 0, osg::Vec3(0, 0, 1)      //yaw
                                 );
                Matrixd init_orient_mat;
                init_orient.get(init_orient_mat);
                Matrixd init_orient_mat_inv;
                init_orient_mat_inv.invert(init_orient_mat);

                Matrixd locate_rot = init_orient_mat_inv * pv_rot;
                Matrixd locate_rot_inv;
                locate_rot_inv.transpose(locate_rot);
                fs::path output_dir(outpath);
                fs::path photo_path(apv->getPhotoName());
                fs::path photo_filename = photo_path.filename();
                output_dir /= fs::basename(photo_filename);
                fs::create_directories(output_dir);
                fs::path output_path_rot = output_dir / fs::path("rotationC2G.xml");
                fs::path output_path_rot_inv = output_dir / fs::path("rotationG2C.xml");
                fs::path output_path_info = output_dir / fs::path("info.txt");
                fs::path output_path_photo = output_dir / photo_filename;

                itr::Util::printLocateRotationMatrixXML(locate_rot, output_path_rot.string());
                itr::Util::printLocateRotationMatrixXML(locate_rot_inv, output_path_rot_inv.string());

                ofstream info(output_path_info.string());
                info << fs::basename(photo_filename) << " " << gp_wgs.y() << " " << gp_wgs.x() << " " << gp_wgs.z() << " " << apv->getFOVDeg() << std::endl;
                info.close();
                try
                {
                    fs::copy_file(photo_path, output_path_photo);
                }
                catch (fs::filesystem_error &fse)
                {
                    std::cerr << "Error while copying photo: " << fse.what()
                              << std::endl;
                }
            }
            return 0;
        }

        if (pickedPhotoviewsStr.size() > 0)
        {
            vector< shared_ptr<AbstractPhotoView> > pickedPvs =
                    loadPickedPhotoviews(pickedPhotoviewsStr, photoviews,
                                         mapNode,
                                         center_lat, center_lon);

            kh->setPickedPhotoViews(pickedPvs);
        }
        if (allPhotoviewsStr.size() > 0)
        {
            vector< shared_ptr<AbstractPhotoView> > allPvs =
                    loadPickedPhotoviews(allPhotoviewsStr, photoviews,
                                         mapNode,
                                         center_lat, center_lon);

            //in case all photoviews which shall be used as control points
            //do not have valid time, we cannot sort according to the time.
            bool sortPvsDateTime = false;
            if (arguments.read("--sort-time", outpath))
            {
                sortPvsDateTime = true;
            }
            for (shared_ptr<AbstractPhotoView> pv : allPvs)
            {
                if (pv->getCreateDate() < 0)
                {
                    sortPvsDateTime = false;
                    break;
                }
            }
            kh->setSortPhotoviewsDate(sortPvsDateTime);
            kh->setAllPhotoViews(allPvs);
        }
        std::cout << "before render photoviews" << std::endl;
        int image_width;
        bool overlay = false;
        if (arguments.read("--overlay"))
        {
            //enable backface culling
            ss->setAttributeAndModes(
                        new osg::CullFace(),
                        osg::StateAttribute::ON | osg::StateAttribute::OVERRIDE );
            ss->setMode( GL_CULL_FACE, osg::StateAttribute::ON);

            //render terrain overlaid to the image
            overlay = true;
            for (shared_ptr<AbstractPhotoView> pv : photoviews)
            {
                if (!pv->isAboveGround())
                {
                    pv->fitPositionToTerrain();
                }
            }
        }
        if (arguments.read("--render-photoviews", image_width))
        {
            std::cout << "Trying to move underground photoviews to the terrain level." << std::endl;
            for (shared_ptr<SfMLoader> l : sfmLoaders)
            {
                l->fitPhotoViewsToTerrain();
            }
            bool usePicked = pickedPhotoviewsStr.size() > 0;
            std::cout << "Going into rendering photoviews." << std::endl;
            if (!viewer.isRealized())
            {
                std::cout << "Realizing viewer" << std::endl;
                viewer.realize();
            }
            kh->renderPhotoViews(&viewer, usePicked, image_width, overlay,
                                 dry_run, scene_center);
            return 0;
        }

        if (arguments.read("--render-flythrough"))
        {
            kh->fitCamerasToTerrain();
            kh->adjustRotationScale();
            kh->flythroughAnimation(&viewer);
            kh->hideAllPhotoViews();
            kh->flythroughAnimationFromPoint(&viewer, true, true);
        }

        if (arguments.read("--no-export-flythrough"))
        {
            kh->setExportFlythrough(false);
        }

        if (arguments.read("--export-picked-only"))
        {
            kh->setExportOnlyPicked(true);
        }


        viewer.addEventHandler(kh);
    }

    /* Intended to be used to import photoparams from project Locate.*/
    float lat;
    float lon;
    float fov;
    string rotation_matrix_xml_file;
    string image_path;
    if (arguments.read("--single-photoparam", lat, lon, fov,
                       rotation_matrix_xml_file, image_path))
    {
        std::shared_ptr<PhotoView> pv = std::make_shared<PhotoView>(mapNode, image_path);
        //fix the FOV for portrait images
        if (pv->isPortrait())
        {
            fov = (fov / pv->getImageWidth()) * pv->getImageHeight();
        }
        pv->setWGS84Position(static_cast<double>(lat), static_cast<double>(lon));
        Vec3d pos = pv->getECEFPosition();
        GeoPoint photo_point(srs->getGeocentricSRS(), pos.x(), pos.y(), pos.z(), ALTMODE_ABSOLUTE);

        Eigen::Matrix3d rot = itr::Util::loadLocateRotationMatrixXML(rotation_matrix_xml_file);
        Eigen::Quaternion<double> rotq(rot);
        Quat quat(rotq.x(), rotq.y(), rotq.z(), rotq.w());
        Matrixd rotmat;
        quat.get(rotmat);

        Matrixd l2w;
        photo_point.createLocalToWorld(l2w);

        Quat init_orient(M_PI, osg::Vec3(1, 0, 0),  //pitch
                         0, osg::Vec3(0, 1, 0),     //roll
                         0, osg::Vec3(0, 0, 1)      //yaw
                         );
        Matrixd init_orient_mat;
        init_orient.get(init_orient_mat);

        Matrixd final_rot = init_orient_mat * rotmat * l2w;
        pv->setOrientation(final_rot.getRotate());

        pv->setFOV((static_cast<double>(fov) / 180.0) * M_PI);

        vector< shared_ptr<AbstractPhotoView> > photoviews;
        photoviews.push_back(pv);
        vector< shared_ptr<SfMLoader> > sfmLoaders;

        ref_ptr<AdobeViewerKeyboardEventHandler> kh =
                new AdobeViewerKeyboardEventHandler(mapNode, earth_manip,
                                                    photoviews, sfmLoaders,
                                                    ".",
                                                    lat,
                                                    lon, false, use_egl);

        string output_path = ".";
        if (arguments.read("--output", output_path))
        {
            kh->setOutputPath(output_path);
        }

        bool overlay = false;
        if (arguments.read("--overlay"))
        {
            //enable backface culling
            ss->setAttributeAndModes(
                        new osg::CullFace(),
                        osg::StateAttribute::ON | osg::StateAttribute::OVERRIDE );
            ss->setMode( GL_CULL_FACE, osg::StateAttribute::ON);

            //render terrain overlaid to the image
            overlay = true;
        }

        int image_width;
        if (arguments.read("--render-photoviews", image_width))
        {
            std::shared_ptr<SfMLoader> empty_loader;
            const Hash_Map<IndexT, Eigen::Vector3d> cld;
            const vector<osg::Vec3d> points;

            fs::path outpath(output_path);
            fs::path p_outpath(output_path);
            p_outpath /= "real";
            if (!boost::filesystem::exists(p_outpath))
            {
                fs::create_directories(p_outpath);
            }

            fs::path p_outpath_fixed(output_path);
            p_outpath_fixed /= "fixed";
            if (!boost::filesystem::exists(p_outpath_fixed))
            {
                fs::create_directories(p_outpath_fixed);
            }

            boost::filesystem::path scene_info_outpath = outpath / "scene_info.txt";
            std::ofstream scene_info(scene_info_outpath.string());
            scene_info << "center:" << scene_center.x() << " " << scene_center.y() << " " << scene_center.z() << std::endl;
            scene_info.close();

            osg::ref_ptr<WindowCaptureCallback> wcc = new WindowCaptureCallback;
            wcc->addToViewerMainCamera(viewer);



            kh->renderPhotoviewRealAndFixedIntoDir(&viewer, empty_loader, pv,
                                                   p_outpath, p_outpath_fixed,
                                                   wcc, 0, cld, points,
                                                   scene_center, 60.0, 1.0,
                                                   image_width, overlay,
                                                   dry_run);


            int count = 10;
            double stdev_meters = 500;
            double above_ground = 10;
            if (arguments.read("--render-random", count, stdev_meters, above_ground))
            {
                //randomly sample and render additional views which
                //look on the same scene as the original photoview.
                kh->randomlySampleAndRenderAdditionalViews(&viewer, pv, p_outpath, wcc, scene_center, count, image_width, stdev_meters, above_ground);
            }
            return 0;
        }
        std::cout << "Adding event handler" << std::endl;
        viewer.addEventHandler(kh);

    }

    //preload area callback
    PreloadAreaRangeCallback *parc = new PreloadAreaRangeCallback(photoviews);


    if (arguments.read("--preload-latlon", lat, lon))
    {
        std::cout << "Preload lat lon: " << lat << ", " << lon << std::endl;
        SpatialReference *srs = SpatialReference::get("epsg:4326");
        GeoPoint fit_pos(srs, lon, lat, 1.8, ALTMODE_RELATIVE);
        fit_pos.transformZ(ALTMODE_ABSOLUTE,
                           mapNode->getTerrainEngine()->getTerrain());
        GeoPoint fitted_xyz = fit_pos.transform(srs->getGeocentricSRS());
        osg::Vec3d preload_pos = osg::Vec3d(fitted_xyz.x(), fitted_xyz.y(), fitted_xyz.z());
        parc->addPosition(preload_pos);
    }
    mapNode->getTerrainEngine()->setComputeRangeCallback(parc);

    float c_lat;
    float c_lon;
    double width;
    double height;
    double offset_x;
    double offset_y;
    int face_resolution;
    string outpath;

    string mode;
    typedef TerrainRenderer::CameraMode CameraMode;
    CameraMode cameraMode = CameraMode::EQUIRECTANGULAR;
    if (arguments.read("--render-grid-mode", mode))
    {
        if (mode != "equirectangular" && mode != "perspective")
        {
            throw std::runtime_error("Invalid option for --render-grid-mode.");
        }
        if (mode == "perspective")
        {
            cameraMode = CameraMode::PERSPECTIVE;
        }
    }

    fov = 60.0f;
    unsigned int num_views = 12;

    if (arguments.read("--render-grid-fov", fov))
    {
        assert(fov > 0.0);
    }
    else
    {
        fov = 60.0f;
    }

    if (!arguments.read("--render-grid-num-views", num_views))
    {
        num_views = 12;
    }

    if (arguments.read("--render-grid", c_lat, c_lon, width, height, offset_x,
                       offset_y, face_resolution, outpath))
    {
#ifndef WITH_EGL
        if (cameraMode == CameraMode::PERSPECTIVE)
        {
            osg::ref_ptr<osgViewer::SingleWindow> sw = new osgViewer::SingleWindow(0, 0, face_resolution, face_resolution);
            sw->setWindowDecoration(false);
            viewer.apply(sw);
        }
#endif

        std::cout << "face resolution: " << face_resolution << std::endl;
        SpatialReference *srs = SpatialReference::get("epsg:4326");
        GeoPoint center(srs, c_lon, c_lat, 1.8, ALTMODE_RELATIVE);
        center.transformZ(ALTMODE_ABSOLUTE,
                          mapNode->getTerrainEngine()->getTerrain());

        TerrainRenderer tr(&viewer, mapNode, center, width, height,
                           offset_x, offset_y, face_resolution, outpath,
                           cameraMode, fov, num_views, use_egl);

        tr.render();
        return 0;
    }

    string output_prefix;
    if (arguments.read("--pose-from-gps", lat, lon, output_prefix))
    {
        SpatialReference *srs = SpatialReference::get("epsg:4326");
        osgGA::FirstPersonManipulator *fp_manip = new osgGA::FirstPersonManipulator;
        fp_manip->setVerticalAxisFixed(false);

        osg::ref_ptr<ElevationPool> ep = mapNode->getMap()->getElevationPool();
        osg::ref_ptr<ElevationEnvelope> ee = ep->createEnvelope(srs, 16);

        float elev = ee->getElevation(lon, lat);
        GeoPoint fit_pos(srs, lon, lat, static_cast<double>(elev), ALTMODE_ABSOLUTE);
        GeoPoint fitted_xyz = fit_pos.transform(srs->getGeocentricSRS());
        Vec3d pos = Vec3d(fitted_xyz.x(), fitted_xyz.y(), fitted_xyz.z());

        Viewpoint vp;
        vp.focalPoint()->set(srs->getGeocentricSRS(), pos, AltitudeMode::ALTMODE_ABSOLUTE);
        vp.range()->set(0, Units::METERS);
        earth_manip->setHomeViewpoint(vp, 0.0);
        earth_manip->home(0);

        Matrixd l2w;
        fitted_xyz.createLocalToWorld(l2w);
        Quat orientation(-M_PI / 2.0, osg::Vec3(1, 0, 0),           //roll
                         0, osg::Vec3(0, 1, 0),            //pitch
                         0, osg::Vec3(0, 0, 1)              //yaw
                         );

        orientation *= l2w.getRotate();
        double d = PhotoView::PHOTOPLANE_OFFSET;
        Vec3d eye = orientation * Vec3d(0.0, 0.0, 1.0) * d;

        //set first person manip
        fp_manip->setTransformation(pos, pos + eye, orientation * osg::Vec3d(0, -1, 0));
        fp_manip->setHomePosition(pos, pos + eye, orientation * osg::Vec3d(0, -1, 0));
        fp_manip->setVerticalAxisFixed(false);
        viewer.setCameraManipulator(fp_manip);

        string modelview_outpath = output_prefix + "_modelview.txt";
        string projection_outpath = output_prefix + "_projection.txt";

        const Eigen::Matrix4d modelview = Eigen::Map<Eigen::Matrix4d>(fp_manip->getInverseMatrix().ptr());
        itr::Util::printMatrixToFile(modelview_outpath, modelview);

        osg::Matrixd proj = viewer.getCamera()->getProjectionMatrix();
        const Eigen::Matrix4d proj_eigen = Eigen::Map<Eigen::Matrix4d>(proj.ptr());
        itr::Util::printMatrixToFile(projection_outpath, proj_eigen);
        return 0;
    }


    //open window and start drawing loop

    std::cout << "Before RUN" << std::endl;

    //Metrics::run(viewer);
    viewer.realize();
    return viewer.run();

    return 0;

}
